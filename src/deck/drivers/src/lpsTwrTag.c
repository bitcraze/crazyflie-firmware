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
/* uwb_twr_anchor.c: Uwb two way ranging anchor implementation */


#include <string.h>

#include "lpsTwrTag.h"

#include "log.h"
#include "param.h"

#include "mac.h"

#include "stabilizer_types.h"
#ifdef ESTIMATOR_TYPE_kalman
#include "estimator_kalman.h"
#include "arm_math.h"
#endif

#define RX_TIMEOUT 1000

// Amount of failed ranging before the ranging value is set as wrong
#define RANGING_FAILED_TH 6

#define N_ANCHORS 6
static const int anchors[N_ANCHORS] = {1,2,3,4,5,6};

#define TAG_ADDRESS 8

// Outlier rejection
#ifdef ESTIMATOR_TYPE_kalman
  #define RANGING_HISTORY_LENGTH 32
  #define OUTLIER_TH 4
  static struct {
    float32_t history[RANGING_HISTORY_LENGTH];
    size_t ptr;
  } rangingStats[N_ANCHORS];
#endif

static const uint8_t tag_address[] = {TAG_ADDRESS,0,0,0,0,0,0xcf,0xbc};

static const uint8_t baseAddressInitialValues[] = {0, 0, 0, 0, 0, 0, 0xcf, 0xbc};
static uint8_t base_address[8];

// The anchor position can be set using parameters
// As an option you can set a static position in this file and set
// anchorPositionOk to enable sending the anchor rangings to the Kalman filter
static point_t anchorPosition[N_ANCHORS];
static bool anchorPositionOk = false;

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

static const double C = 299792458.0;       // Speed of light
static const double tsfreq = 499.2e6 * 128;  // Timestamp counter frequency

#define ANTENNA_OFFSET 154.6   // In meter
static const uint64_t ANTENNA_DELAY = (ANTENNA_OFFSET*499.2e6*128)/299792458.0; // In radio tick

static packet_t txPacket;
static volatile uint8_t curr_seq = 0;
static int current_anchor = 0;

static float distance[N_ANCHORS];
static float pressures[N_ANCHORS];
static int failedRanging[N_ANCHORS];
static volatile uint16_t rangingState = 0;
static bool ranging_complete = false;

static void txcallback(dwDevice_t *dev)
{
  dwTime_t departure;
  dwGetTransmitTimestamp(dev, &departure);
  departure.full += (ANTENNA_DELAY/2);

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

  packet_t rxPacket;
  memset(&rxPacket, 0, MAC802154_HEADER_LENGTH);

  dwGetData(dev, (uint8_t*)&rxPacket, dataLength);

  if (memcmp(rxPacket.destAddress, tag_address, 8)) {
    dwNewReceive(dev);
    dwSetDefaults(dev);
    dwStartReceive(dev);
    return MAX_TIMEOUT;
  }

  memcpy(txPacket.destAddress, rxPacket.sourceAddress, 8);
  memcpy(txPacket.sourceAddress, rxPacket.destAddress, 8);

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

      tprop = tprop_ctn/tsfreq;
      distance[current_anchor] = C * tprop;
      pressures[current_anchor] = report->asl;

#ifdef ESTIMATOR_TYPE_kalman
      // Ouliers rejection
      rangingStats[current_anchor].ptr = (rangingStats[current_anchor].ptr + 1) % RANGING_HISTORY_LENGTH;
      float32_t mean;
      float32_t stddev;

      arm_std_f32(rangingStats[current_anchor].history, RANGING_HISTORY_LENGTH, &stddev);
      arm_mean_f32(rangingStats[current_anchor].history, RANGING_HISTORY_LENGTH, &mean);
      float32_t diff = fabsf(mean-distance[current_anchor]);

      rangingStats[current_anchor].history[rangingStats[current_anchor].ptr] = distance[current_anchor];

      if (anchorPositionOk && (diff < (OUTLIER_TH*stddev))) {
        distanceMeasurement_t dist;
        dist.distance = distance[current_anchor];
        dist.x = anchorPosition[current_anchor].x;
        dist.y = anchorPosition[current_anchor].y;
        dist.z = anchorPosition[current_anchor].z;
        dist.stdDev = 0.25;
        stateEstimatorEnqueueDistance(&dist);
      }
#endif

      ranging_complete = true;

      return 0;
      break;
    }
  }
  return MAX_TIMEOUT;
}

void initiateRanging(dwDevice_t *dev)
{
  current_anchor ++;
  if (current_anchor >= N_ANCHORS) {
    current_anchor = 0;
  }

  base_address[0] =  anchors[current_anchor];

  dwIdle(dev);

  txPacket.payload[TYPE] = POLL;
  txPacket.payload[SEQ] = ++curr_seq;

  memcpy(txPacket.sourceAddress, tag_address, 8);
  memcpy(txPacket.destAddress, base_address, 8);

  dwNewTransmit(dev);
  dwSetDefaults(dev);
  dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2);

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
        rangingState &= ~(1<<current_anchor);
        if (failedRanging[current_anchor] < RANGING_FAILED_TH) {
          failedRanging[current_anchor] ++;
          rangingState |= (1<<current_anchor);
        }
      } else {
        rangingState |= (1<<current_anchor);
        failedRanging[current_anchor] = 0;
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

static void twrTagInit(dwDevice_t *dev)
{
  // Initialize the packet in the TX buffer
  MAC80215_PACKET_INIT(txPacket, MAC802154_TYPE_DATA);
  txPacket.pan = 0xbccf;

  memcpy(base_address, baseAddressInitialValues, sizeof(base_address));
  memset(anchorPosition, 0, sizeof(anchorPosition));
  anchorPositionOk = false;

  memset(&poll_tx, 0, sizeof(poll_tx));
  memset(&poll_rx, 0, sizeof(poll_rx));
  memset(&answer_tx, 0, sizeof(answer_tx));
  memset(&answer_rx, 0, sizeof(answer_rx));
  memset(&final_tx, 0, sizeof(final_tx));
  memset(&final_rx, 0, sizeof(final_rx));

  memset(&txPacket, 0, sizeof(txPacket));
  curr_seq = 0;
  current_anchor = 0;

  rangingState = 0;
  ranging_complete = false;

  memset(distance, 0, sizeof(distance));
  memset(pressures, 0, sizeof(pressures));
  memset(failedRanging, 0, sizeof(failedRanging));
}

uwbAlgorithm_t uwbTwrTagAlgorithm = {
  .init = twrTagInit,
  .onEvent = twrTagOnEvent,
};

LOG_GROUP_START(ranging)
LOG_ADD(LOG_FLOAT, distance1, &distance[0])
LOG_ADD(LOG_FLOAT, distance2, &distance[1])
LOG_ADD(LOG_FLOAT, distance3, &distance[2])
LOG_ADD(LOG_FLOAT, distance4, &distance[3])
LOG_ADD(LOG_FLOAT, distance5, &distance[4])
LOG_ADD(LOG_FLOAT, distance6, &distance[5])
LOG_ADD(LOG_FLOAT, distance7, &distance[6])
LOG_ADD(LOG_FLOAT, distance8, &distance[7])
LOG_ADD(LOG_FLOAT, pressure1, &pressures[0])
LOG_ADD(LOG_FLOAT, pressure2, &pressures[1])
LOG_ADD(LOG_FLOAT, pressure3, &pressures[2])
LOG_ADD(LOG_FLOAT, pressure4, &pressures[3])
LOG_ADD(LOG_FLOAT, pressure5, &pressures[4])
LOG_ADD(LOG_FLOAT, pressure6, &pressures[5])
LOG_ADD(LOG_FLOAT, pressure7, &pressures[6])
LOG_ADD(LOG_FLOAT, pressure8, &pressures[7])
LOG_ADD(LOG_UINT16, state, &rangingState)
LOG_GROUP_STOP(ranging)

PARAM_GROUP_START(anchorpos)
PARAM_ADD(PARAM_FLOAT, anchor0x, &anchorPosition[0].x)
PARAM_ADD(PARAM_FLOAT, anchor0y, &anchorPosition[0].y)
PARAM_ADD(PARAM_FLOAT, anchor0z, &anchorPosition[0].z)
PARAM_ADD(PARAM_FLOAT, anchor1x, &anchorPosition[1].x)
PARAM_ADD(PARAM_FLOAT, anchor1y, &anchorPosition[1].y)
PARAM_ADD(PARAM_FLOAT, anchor1z, &anchorPosition[1].z)
PARAM_ADD(PARAM_FLOAT, anchor2x, &anchorPosition[2].x)
PARAM_ADD(PARAM_FLOAT, anchor2y, &anchorPosition[2].y)
PARAM_ADD(PARAM_FLOAT, anchor2z, &anchorPosition[2].z)
PARAM_ADD(PARAM_FLOAT, anchor3x, &anchorPosition[3].x)
PARAM_ADD(PARAM_FLOAT, anchor3y, &anchorPosition[3].y)
PARAM_ADD(PARAM_FLOAT, anchor3z, &anchorPosition[3].z)
PARAM_ADD(PARAM_FLOAT, anchor4x, &anchorPosition[4].x)
PARAM_ADD(PARAM_FLOAT, anchor4y, &anchorPosition[4].y)
PARAM_ADD(PARAM_FLOAT, anchor4z, &anchorPosition[4].z)
PARAM_ADD(PARAM_FLOAT, anchor5x, &anchorPosition[5].x)
PARAM_ADD(PARAM_FLOAT, anchor5y, &anchorPosition[5].y)
PARAM_ADD(PARAM_FLOAT, anchor5z, &anchorPosition[5].z)
PARAM_ADD(PARAM_UINT8, enable, &anchorPositionOk)
PARAM_GROUP_STOP(anchorpos)
