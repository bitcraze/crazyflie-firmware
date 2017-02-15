/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * LPS node firmware.
 *
 * Copyright 2016, Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Foobar is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
 */
/* uwb_twr_anchor.c: Uwb two way ranging tag with TDMA implementation */

#include "locodeck.h"

#include <string.h>

#include "log.h"
#include "param.h"

#include "libdw1000.h"

#include "mac.h"

#include "stabilizer_types.h"
#ifdef ESTIMATOR_TYPE_kalman
#include "estimator_kalman.h"
#include "arm_math.h"
#endif

#define RX_TIMEOUT 1000

// TDMA handling
bool tdmaSynchronized;
dwTime_t frameStart;

#ifndef TDMA_NSLOTS_BITS
#warning "Number of slots for TDMA not defined! Defaulting to 1."
#define TDMA_NSLOTS_BITS 1
#endif

#ifndef TDMA_SLOT
#warning "Slot number for TDMA not defined! Defaulting to 0."
#define TDMA_SLOT 0
#endif

#define TDMA_SLOT_BITS 27

#define TDMA_FRAME_BITS (TDMA_SLOT_BITS + TDMA_NSLOTS_BITS)
#define TDMA_SLOT_LEN (1ull<<(TDMA_SLOT_BITS+1))
#define TDMA_FRAME_LEN (1ull<<(TDMA_FRAME_BITS+1))

#define TDMA_LAST_FRAME(NOW) ( NOW & ~(TDMA_FRAME_LEN-1) )


// Amount of failed ranging before the ranging value is set as wrong
#define RANGING_FAILED_TH 6

#define N_ANCHORS 6
int anchors[N_ANCHORS] = {1,2,3,4,5,6};

#define TAG_ADDRESS 8+TDMA_SLOT

// Outlier rejection
#ifdef ESTIMATOR_TYPE_kalman
  #define RANGING_HISTORY_LENGTH 32
  #define OUTLIER_TH 4
  static struct {
    float32_t history[RANGING_HISTORY_LENGTH];
    size_t ptr;
  } rangingStats[N_ANCHORS];
#endif

static uint8_t tag_address[] = {TAG_ADDRESS,0,0,0,0,0,0xcf,0xbc};
static uint8_t base_address[] = {0,0,0,0,0,0,0xcf,0xbc};

// The four packets for ranging
#define POLL 0x01   // Poll is initiated by the tag
#define ANSWER 0x02
#define FINAL 0x03
#define REPORT 0x04 // Report contains all measurement from the anchor

typedef struct {
  uint8_t pollRx[5];
  uint8_t answerTx[5];
  uint8_t finalRx[5];

  float pressure;
  float temperature;
  float asl;
  uint8_t pressure_ok;
} __attribute__((packed)) reportPayload_t;

// Timestamps for ranging
static dwTime_t poll_tx;
static dwTime_t poll_rx;
static dwTime_t answer_tx;
static dwTime_t answer_rx;
static dwTime_t final_tx;
static dwTime_t final_rx;

#define ANTENNA_OFFSET 154.6   // In meter
static const uint64_t ANTENNA_DELAY = (ANTENNA_OFFSET*499.2e6*128)/299792458.0; // In radio tick

static packet_t rxPacket;
static packet_t txPacket;
static volatile uint8_t curr_seq = 0;
static int current_anchor = 0;

static bool ranging_complete = false;

static lpsAlgoOptions_t* options;

static void txcallback(dwDevice_t *dev)
{
  dwTime_t departure;
  dwGetTransmitTimestamp(dev, &departure);
  departure.full += (options->antennaDelay / 2);

  switch (txPacket.payload[0]) {
    case POLL:
      poll_tx = departure;
      break;
    case FINAL:
      final_tx = departure;
      break;
  }
}

#define TYPE 0
#define SEQ 1

static uint32_t rxcallback(dwDevice_t *dev) {
  dwTime_t arival = { .full=0 };
  int dataLength = dwGetDataLength(dev);

  if (dataLength == 0) return 0;

  bzero(&rxPacket, MAC802154_HEADER_LENGTH);

  dwGetData(dev, (uint8_t*)&rxPacket, dataLength);

  if (memcmp(&rxPacket.destAddress, tag_address, 8)) {
    dwNewReceive(dev);
    dwSetDefaults(dev);
    dwStartReceive(dev);
    return MAX_TIMEOUT;
  }

  txPacket.destAddress = rxPacket.sourceAddress;
  txPacket.sourceAddress = rxPacket.destAddress;

  switch(rxPacket.payload[TYPE]) {
    // Tag received messages
    case ANSWER:
      if (rxPacket.payload[SEQ] != curr_seq) {
        return 0;
      }

      txPacket.payload[0] = FINAL;
      txPacket.payload[SEQ] = rxPacket.payload[SEQ];

      dwGetReceiveTimestamp(dev, &arival);
      arival.full -= (ANTENNA_DELAY/2);
      answer_rx = arival;

      dwNewTransmit(dev);
      dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2);

      dwWaitForResponse(dev, true);
      dwStartTransmit(dev);

      break;
    case REPORT:
    {
      reportPayload_t *report = (reportPayload_t *)(rxPacket.payload+2);
      double tround1, treply1, treply2, tround2, tprop_ctn, tprop;

      if (rxPacket.payload[SEQ] != curr_seq) {
        return 0;
      }


      memcpy(&poll_rx, &report->pollRx, 5);
      memcpy(&answer_tx, &report->answerTx, 5);
      memcpy(&final_rx, &report->finalRx, 5);

      tround1 = answer_rx.low32 - poll_tx.low32;
      treply1 = answer_tx.low32 - poll_rx.low32;
      tround2 = final_rx.low32 - answer_tx.low32;
      treply2 = final_tx.low32 - answer_rx.low32;

      tprop_ctn = ((tround1*tround2) - (treply1*treply2)) / (tround1 + tround2 + treply1 + treply2);

      tprop = tprop_ctn/LOCODECK_TS_FREQ;
      options->distance[current_anchor] = SPEED_OF_LIGHT * tprop;
      options->pressures[current_anchor] = report->asl;

#ifdef ESTIMATOR_TYPE_kalman
      // Outliers rejection
      rangingStats[current_anchor].ptr = (rangingStats[current_anchor].ptr + 1) % RANGING_HISTORY_LENGTH;
      float32_t mean;
      float32_t stddev;

      arm_std_f32(rangingStats[current_anchor].history, RANGING_HISTORY_LENGTH, &stddev);
      arm_mean_f32(rangingStats[current_anchor].history, RANGING_HISTORY_LENGTH, &mean);
      float32_t diff = fabsf(mean - options->distance[current_anchor]);

      rangingStats[current_anchor].history[rangingStats[current_anchor].ptr] = options->distance[current_anchor];

      if (options->combinedAnchorPositionOk && (diff < (OUTLIER_TH*stddev))) {
        distanceMeasurement_t dist;
        dist.distance = options->distance[current_anchor];
        dist.x = options->anchorPosition[current_anchor].x;
        dist.y = options->anchorPosition[current_anchor].y;
        dist.z = options->anchorPosition[current_anchor].z;
        dist.stdDev = 0.25;
        stateEstimatorEnqueueDistance(&dist);
      }
#endif

      if (current_anchor == 0) {
        // Final packet is sent by us and received by the anchor
        // We use it as synchonisation time for TDMA
        dwTime_t offset = { .full =final_tx.full - final_rx.full };
        frameStart.full = TDMA_LAST_FRAME(final_rx.full) + offset.full;
        tdmaSynchronized = true;
      }

      ranging_complete = true;

      return 0;
      break;
    }
  }
  return MAX_TIMEOUT;
}

/* Adjust time for schedule transfer by DW1000 radio. Set 9 LSB to 0 */
uint32_t adjustTxRxTime(dwTime_t *time)
{
  uint32_t added = (1<<9) - (time->low32 & ((1<<9)-1));
  time->low32 = (time->low32 & ~((1<<9)-1)) + (1<<9);
  return added;
}

/* Calculate the transmit time for a given timeslot in the current frame */
static dwTime_t transmitTimeForSlot(int slot)
{
  dwTime_t transmitTime = { .full = 0 };
  // Calculate start of the slot
  transmitTime.full = frameStart.full + slot*TDMA_SLOT_LEN;

  // DW1000 can only schedule time with 9 LSB at 0, adjust for it
  adjustTxRxTime(&transmitTime);
  return transmitTime;
}

static void initiateRanging(dwDevice_t *dev)
{
  if (tdmaSynchronized) {
    // go to next frame
    frameStart.full += TDMA_FRAME_LEN;

    current_anchor ++;
    if (current_anchor >= N_ANCHORS) {
      current_anchor = 0;
    }
  } else {
    current_anchor = 0;
  }

  base_address[0] =  anchors[current_anchor];

  dwIdle(dev);

  txPacket.payload[TYPE] = POLL;
  txPacket.payload[SEQ] = ++curr_seq;

  memcpy(&txPacket.sourceAddress, &tag_address, 8);
  memcpy(&txPacket.destAddress, &base_address, 8);


  dwNewTransmit(dev);
  dwSetDefaults(dev);
  dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2);

  if (tdmaSynchronized) {
    dwTime_t txTime = transmitTimeForSlot(TDMA_SLOT);
    dwSetTxRxTime(dev, txTime);
  }

  dwWaitForResponse(dev, true);
  dwStartTransmit(dev);
}

static uint32_t twrTagOnEvent(dwDevice_t *dev, uwbEvent_t event)
{
  switch(event) {
    case eventPacketReceived:
      return rxcallback(dev);
      break;
    case eventPacketSent:
      txcallback(dev);
      return MAX_TIMEOUT;
      break;
    case eventTimeout:  // Comes back to timeout after each ranging attempt
      if (!ranging_complete) {
        options->rangingState &= ~(1<<current_anchor);
        if (options->failedRanging[current_anchor] < RANGING_FAILED_TH) {
          options->failedRanging[current_anchor] ++;
          options->rangingState |= (1<<current_anchor);
        }
      } else {
        options->rangingState |= (1<<current_anchor);
        options->failedRanging[current_anchor] = 0;
      }
      ranging_complete = false;
      initiateRanging(dev);
      return MAX_TIMEOUT;
      break;
    case eventReceiveTimeout:
    case eventReceiveFailed:
      return 0;
      break;
    default:
      configASSERT(false);
  }

  return MAX_TIMEOUT;
}

static void twrTagInit(dwDevice_t *dev, lpsAlgoOptions_t* newOptions)
{
  tdmaSynchronized = false;

  options = newOptions;


  // Initialize the packet in the TX buffer
  MAC80215_PACKET_INIT(txPacket, MAC802154_TYPE_DATA);
  txPacket.pan = 0xbccf;

  // onEvent is going to be called with eventTimeout which will start ranging
}

uwbAlgorithm_t uwbTwrTagAlgorithm = {
  .init = twrTagInit,
  .onEvent = twrTagOnEvent,
};
