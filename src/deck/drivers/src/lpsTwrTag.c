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
#include <math.h>

#include "lpsTwrTag.h"
#include "lpsTdma.h"

#include "FreeRTOS.h"
#include "task.h"

#include "log.h"
#include "crtp_localization_service.h"

#include "stabilizer_types.h"
#include "estimator_kalman.h"
#include "arm_math.h"

// Outlier rejection
#define RANGING_HISTORY_LENGTH 32
#define OUTLIER_TH 4
static struct {
  float32_t history[RANGING_HISTORY_LENGTH];
  size_t ptr;
} rangingStats[LOCODECK_NR_OF_ANCHORS];

// Rangin statistics
static uint8_t rangingPerSec[LOCODECK_NR_OF_ANCHORS];
static uint8_t rangingSuccessRate[LOCODECK_NR_OF_ANCHORS];
// Used to calculate above values
static uint8_t succededRanging[LOCODECK_NR_OF_ANCHORS];
static uint8_t failedRanging[LOCODECK_NR_OF_ANCHORS];

// Timestamps for ranging
static dwTime_t poll_tx;
static dwTime_t poll_rx;
static dwTime_t answer_tx;
static dwTime_t answer_rx;
static dwTime_t final_tx;
static dwTime_t final_rx;

static packet_t txPacket;
static volatile uint8_t curr_seq = 0;
static int current_anchor = 0;

static bool ranging_complete = false;
static bool lpp_transaction = false;

static lpsLppShortPacket_t lppShortPacket;

static lpsAlgoOptions_t* options;

// TDMA handling
static bool tdmaSynchronized;
static dwTime_t frameStart;

static bool rangingOk;

static void txcallback(dwDevice_t *dev)
{
  dwTime_t departure;
  dwGetTransmitTimestamp(dev, &departure);
  departure.full += (options->antennaDelay / 2);

  switch (txPacket.payload[0]) {
    case LPS_TWR_POLL:
      poll_tx = departure;
      break;
    case LPS_TWR_FINAL:
      final_tx = departure;
      break;
  }
}


static uint32_t rxcallback(dwDevice_t *dev) {
  dwTime_t arival = { .full=0 };
  int dataLength = dwGetDataLength(dev);

  if (dataLength == 0) return 0;

  packet_t rxPacket;
  memset(&rxPacket, 0, MAC802154_HEADER_LENGTH);

  dwGetData(dev, (uint8_t*)&rxPacket, dataLength);

  if (rxPacket.destAddress != options->tagAddress) {
    dwNewReceive(dev);
    dwSetDefaults(dev);
    dwStartReceive(dev);
    return MAX_TIMEOUT;
  }

  txPacket.destAddress = rxPacket.sourceAddress;
  txPacket.sourceAddress = rxPacket.destAddress;

  switch(rxPacket.payload[LPS_TWR_TYPE]) {
    // Tag received messages
    case LPS_TWR_ANSWER:
      if (rxPacket.payload[LPS_TWR_SEQ] != curr_seq) {
        return 0;
      }

      if (dataLength - MAC802154_HEADER_LENGTH > 3) {
        if (rxPacket.payload[LPS_TWR_LPP_HEADER] == LPP_HEADER_SHORT_PACKET) {
          int srcId = -1;

          for (int i=0; i<LOCODECK_NR_OF_ANCHORS; i++) {
            if (rxPacket.sourceAddress == options->anchorAddress[i]) {
              srcId = i;
              break;
            }
          }

          if (srcId >= 0) {
            lpsHandleLppShortPacket(srcId, &rxPacket.payload[LPS_TWR_LPP_TYPE],
                                    dataLength - MAC802154_HEADER_LENGTH - 3);
          }
        }
      }

      txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_FINAL;
      txPacket.payload[LPS_TWR_SEQ] = rxPacket.payload[LPS_TWR_SEQ];

      dwGetReceiveTimestamp(dev, &arival);
      arival.full -= (options->antennaDelay / 2);
      answer_rx = arival;

      dwNewTransmit(dev);
      dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2);

      dwWaitForResponse(dev, true);
      dwStartTransmit(dev);

      break;
    case LPS_TWR_REPORT:
    {
      lpsTwrTagReportPayload_t *report = (lpsTwrTagReportPayload_t *)(rxPacket.payload+2);
      double tround1, treply1, treply2, tround2, tprop_ctn, tprop;

      if (rxPacket.payload[LPS_TWR_SEQ] != curr_seq) {
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

      tprop = tprop_ctn / LOCODECK_TS_FREQ;
      options->distance[current_anchor] = SPEED_OF_LIGHT * tprop;
      options->pressures[current_anchor] = report->asl;

      // Outliers rejection
      rangingStats[current_anchor].ptr = (rangingStats[current_anchor].ptr + 1) % RANGING_HISTORY_LENGTH;
      float32_t mean;
      float32_t stddev;

      arm_std_f32(rangingStats[current_anchor].history, RANGING_HISTORY_LENGTH, &stddev);
      arm_mean_f32(rangingStats[current_anchor].history, RANGING_HISTORY_LENGTH, &mean);
      float32_t diff = fabsf(mean - options->distance[current_anchor]);

      rangingStats[current_anchor].history[rangingStats[current_anchor].ptr] = options->distance[current_anchor];

      rangingOk = true;

      if ((options->combinedAnchorPositionOk || options->anchorPosition[current_anchor].timestamp) &&
          (diff < (OUTLIER_TH*stddev))) {
        distanceMeasurement_t dist;
        dist.distance = options->distance[current_anchor];
        dist.x = options->anchorPosition[current_anchor].x;
        dist.y = options->anchorPosition[current_anchor].y;
        dist.z = options->anchorPosition[current_anchor].z;
        dist.stdDev = 0.25;
        estimatorKalmanEnqueueDistance(&dist);
      }

      if (options->useTdma && current_anchor == 0) {
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
static uint32_t adjustTxRxTime(dwTime_t *time)
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
  if (!options->useTdma || tdmaSynchronized) {
    if (options->useTdma) {
      // go to next TDMA frame
      frameStart.full += TDMA_FRAME_LEN;
    }

    current_anchor ++;
    if (current_anchor >= LOCODECK_NR_OF_ANCHORS) {
      current_anchor = 0;
    }
  } else {
    current_anchor = 0;
  }

  dwIdle(dev);

  txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_POLL;
  txPacket.payload[LPS_TWR_SEQ] = ++curr_seq;

  txPacket.sourceAddress = options->tagAddress;
  txPacket.destAddress = options->anchorAddress[current_anchor];

  dwNewTransmit(dev);
  dwSetDefaults(dev);
  dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2);

  if (options->useTdma && tdmaSynchronized) {
    dwTime_t txTime = transmitTimeForSlot(options->tdmaSlot);
    dwSetTxRxTime(dev, txTime);
  }

  dwWaitForResponse(dev, true);
  dwStartTransmit(dev);
}

static void sendLppShort(dwDevice_t *dev, lpsLppShortPacket_t *packet)
{
  dwIdle(dev);

  txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_LPP_SHORT;
  memcpy(&txPacket.payload[LPS_TWR_SEND_LPP_PAYLOAD], packet->data, packet->length);

  txPacket.sourceAddress = options->tagAddress;
  txPacket.destAddress = options->anchorAddress[packet->dest];

  dwNewTransmit(dev);
  dwSetDefaults(dev);
  dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+1+packet->length);

  dwWaitForResponse(dev, false);
  dwStartTransmit(dev);
}

static uint32_t twrTagOnEvent(dwDevice_t *dev, uwbEvent_t event)
{
  static uint32_t statisticStartTick = 0;

  if (statisticStartTick == 0) {
    statisticStartTick = xTaskGetTickCount();
  }

  switch(event) {
    case eventPacketReceived:
      return rxcallback(dev);
      break;
    case eventPacketSent:
      txcallback(dev);

      if (lpp_transaction) {
        return 0;
      }
      return MAX_TIMEOUT;
      break;
    case eventTimeout:  // Comes back to timeout after each ranging attempt
      if (!ranging_complete && !lpp_transaction) {
        options->rangingState &= ~(1<<current_anchor);
        if (options->failedRanging[current_anchor] < options->rangingFailedThreshold) {
          options->failedRanging[current_anchor] ++;
          options->rangingState |= (1<<current_anchor);
        }

        locSrvSendRangeFloat(current_anchor, NAN);
        failedRanging[current_anchor]++;
      } else {
        options->rangingState |= (1<<current_anchor);
        options->failedRanging[current_anchor] = 0;

        locSrvSendRangeFloat(current_anchor, options->distance[current_anchor]);
        succededRanging[current_anchor]++;
      }

      // Handle ranging statistic
      if (xTaskGetTickCount() > (statisticStartTick+1000)) {
        statisticStartTick = xTaskGetTickCount();

        for (int i=0; i<LOCODECK_NR_OF_ANCHORS; i++) {
          rangingPerSec[i] = failedRanging[i] + succededRanging[i];
          if (rangingPerSec[i] > 0) {
            rangingSuccessRate[i] = 100.0f*(float)succededRanging[i] / (float)rangingPerSec[i];
          } else {
            rangingSuccessRate[i] = 0.0f;
          }

          failedRanging[i] = 0;
          succededRanging[i] = 0;
        }
      }


      if (lpsGetLppShort(&lppShortPacket)) {
        lpp_transaction = true;
        sendLppShort(dev, &lppShortPacket);
      } else {
        lpp_transaction = false;
        ranging_complete = false;
        initiateRanging(dev);
      }
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

static void twrTagInit(dwDevice_t *dev, lpsAlgoOptions_t* algoOptions)
{
  options = algoOptions;

  // Initialize the packet in the TX buffer
  memset(&txPacket, 0, sizeof(txPacket));
  MAC80215_PACKET_INIT(txPacket, MAC802154_TYPE_DATA);
  txPacket.pan = 0xbccf;

  memset(&poll_tx, 0, sizeof(poll_tx));
  memset(&poll_rx, 0, sizeof(poll_rx));
  memset(&answer_tx, 0, sizeof(answer_tx));
  memset(&answer_rx, 0, sizeof(answer_rx));
  memset(&final_tx, 0, sizeof(final_tx));
  memset(&final_rx, 0, sizeof(final_rx));

  curr_seq = 0;
  current_anchor = 0;

  options->rangingState = 0;
  ranging_complete = false;

  tdmaSynchronized = false;

  memset(options->distance, 0, sizeof(options->distance));
  memset(options->pressures, 0, sizeof(options->pressures));
  memset(options->failedRanging, 0, sizeof(options->failedRanging));

  dwSetReceiveWaitTimeout(dev, TWR_RECEIVE_TIMEOUT);

  dwCommitConfiguration(dev);

  rangingOk = false;
}

static bool isRangingOk()
{
  return rangingOk;
}

uwbAlgorithm_t uwbTwrTagAlgorithm = {
  .init = twrTagInit,
  .onEvent = twrTagOnEvent,
  .isRangingOk = isRangingOk,
};

LOG_GROUP_START(twr)
LOG_ADD(LOG_UINT8, rangingSuccessRate0, &rangingSuccessRate[0])
LOG_ADD(LOG_UINT8, rangingPerSec0, &rangingPerSec[0])
LOG_ADD(LOG_UINT8, rangingSuccessRate1, &rangingSuccessRate[1])
LOG_ADD(LOG_UINT8, rangingPerSec1, &rangingPerSec[1])
LOG_ADD(LOG_UINT8, rangingSuccessRate2, &rangingSuccessRate[2])
LOG_ADD(LOG_UINT8, rangingPerSec2, &rangingPerSec[2])
LOG_ADD(LOG_UINT8, rangingSuccessRate3, &rangingSuccessRate[3])
LOG_ADD(LOG_UINT8, rangingPerSec3, &rangingPerSec[3])
LOG_ADD(LOG_UINT8, rangingSuccessRate4, &rangingSuccessRate[4])
LOG_ADD(LOG_UINT8, rangingPerSec4, &rangingPerSec[4])
LOG_ADD(LOG_UINT8, rangingSuccessRate5, &rangingSuccessRate[5])
LOG_ADD(LOG_UINT8, rangingPerSec5, &rangingPerSec[5])
LOG_GROUP_STOP(twr)
