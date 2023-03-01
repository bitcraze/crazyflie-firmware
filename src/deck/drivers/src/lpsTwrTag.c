/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
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
#include "estimator.h"
#include "cf_math.h"

#include "physicalConstants.h"
#include "configblock.h"
#include "lpsTdma.h"
#include "static_mem.h"

// Config
static lpsTwrAlgoOptions_t defaultOptions = {
   .tagAddress = 0xbccf000000000008,
   .anchorAddress = {
     0xbccf000000000000,
     0xbccf000000000001,
     0xbccf000000000002,
     0xbccf000000000003,
     0xbccf000000000004,
     0xbccf000000000005,
 #if LOCODECK_NR_OF_TWR_ANCHORS > 6
     0xbccf000000000006,
 #endif
 #if LOCODECK_NR_OF_TWR_ANCHORS > 7
     0xbccf000000000007,
 #endif
   },
   .antennaDelay = LOCODECK_ANTENNA_DELAY,
   .rangingFailedThreshold = 6,

   .combinedAnchorPositionOk = false,

 #ifdef LPS_TDMA_ENABLE
   .useTdma = true,
   .tdmaSlot = TDMA_SLOT,
 #endif

   // To set a static anchor position from startup, uncomment and modify the
   // following code:
 //   .anchorPosition = {
 //     {timestamp: 1, x: 0.99, y: 1.49, z: 1.80},
 //     {timestamp: 1, x: 0.99, y: 3.29, z: 1.80},
 //     {timestamp: 1, x: 4.67, y: 2.54, z: 1.80},
 //     {timestamp: 1, x: 0.59, y: 2.27, z: 0.20},
 //     {timestamp: 1, x: 4.70, y: 3.38, z: 0.20},
 //     {timestamp: 1, x: 4.70, y: 1.14, z: 0.20},
 //   },
 //
 //   .combinedAnchorPositionOk = true,
};

typedef struct {
  float distance[LOCODECK_NR_OF_TWR_ANCHORS];
  float pressures[LOCODECK_NR_OF_TWR_ANCHORS];
  int failedRanging[LOCODECK_NR_OF_TWR_ANCHORS];
} twrState_t;

static twrState_t state;
static lpsTwrAlgoOptions_t* options = &defaultOptions;

// Outlier rejection
#define RANGING_HISTORY_LENGTH 32
#define OUTLIER_TH 4
NO_DMA_CCM_SAFE_ZERO_INIT static struct {
  float32_t history[RANGING_HISTORY_LENGTH];
  size_t ptr;
} rangingStats[LOCODECK_NR_OF_TWR_ANCHORS];

// Rangin statistics
static uint8_t rangingPerSec[LOCODECK_NR_OF_TWR_ANCHORS];
static uint8_t rangingSuccessRate[LOCODECK_NR_OF_TWR_ANCHORS];
// Used to calculate above values
static uint8_t succededRanging[LOCODECK_NR_OF_TWR_ANCHORS];
static uint8_t failedRanging[LOCODECK_NR_OF_TWR_ANCHORS];

// Timestamps for ranging
static dwTime_t poll_tx;
static dwTime_t poll_rx;
static dwTime_t answer_tx;
static dwTime_t answer_rx;
static dwTime_t final_tx;
static dwTime_t final_rx;

static packet_t txPacket;
static volatile uint8_t curr_seq = 0;
static uint8_t current_anchor = 0;

static bool ranging_complete = false;
static bool lpp_transaction = false;

static lpsLppShortPacket_t lppShortPacket;

// TDMA handling
static bool tdmaSynchronized;
static dwTime_t frameStart;

static bool rangingOk;

static void lpsHandleLppShortPacket(const uint8_t srcId, const uint8_t *data);

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

          for (int i=0; i<LOCODECK_NR_OF_TWR_ANCHORS; i++) {
            if (rxPacket.sourceAddress == options->anchorAddress[i]) {
              srcId = i;
              break;
            }
          }

          if (srcId >= 0) {
            lpsHandleLppShortPacket(srcId, &rxPacket.payload[LPS_TWR_LPP_TYPE]);
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
      state.distance[current_anchor] = SPEED_OF_LIGHT * tprop;
      state.pressures[current_anchor] = report->asl;

      // Outliers rejection
      rangingStats[current_anchor].ptr = (rangingStats[current_anchor].ptr + 1) % RANGING_HISTORY_LENGTH;
      float32_t mean;
      float32_t stddev;

      arm_std_f32(rangingStats[current_anchor].history, RANGING_HISTORY_LENGTH, &stddev);
      arm_mean_f32(rangingStats[current_anchor].history, RANGING_HISTORY_LENGTH, &mean);
      float32_t diff = fabsf(mean - state.distance[current_anchor]);

      rangingStats[current_anchor].history[rangingStats[current_anchor].ptr] = state.distance[current_anchor];

      rangingOk = true;

      if ((options->combinedAnchorPositionOk || options->anchorPosition[current_anchor].timestamp) &&
          (diff < (OUTLIER_TH*stddev))) {
        distanceMeasurement_t dist;
        dist.distance = state.distance[current_anchor];
        dist.x = options->anchorPosition[current_anchor].x;
        dist.y = options->anchorPosition[current_anchor].y;
        dist.z = options->anchorPosition[current_anchor].z;
        dist.anchorId = current_anchor;
        dist.stdDev = 0.25;
        estimatorEnqueueDistance(&dist);
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
    if (current_anchor >= LOCODECK_NR_OF_TWR_ANCHORS) {
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
      {
        uint16_t rangingState = locoDeckGetRangingState();
        if (!ranging_complete && !lpp_transaction) {
          rangingState &= ~(1<<current_anchor);
          if (state.failedRanging[current_anchor] < options->rangingFailedThreshold) {
            state.failedRanging[current_anchor] ++;
            rangingState |= (1<<current_anchor);
          }

          locSrvSendRangeFloat(current_anchor, NAN);
          failedRanging[current_anchor]++;
        } else {
          rangingState |= (1<<current_anchor);
          state.failedRanging[current_anchor] = 0;

          locSrvSendRangeFloat(current_anchor, state.distance[current_anchor]);
          succededRanging[current_anchor]++;
        }
        locoDeckSetRangingState(rangingState);
      }

      // Handle ranging statistic
      if (xTaskGetTickCount() > (statisticStartTick+1000)) {
        statisticStartTick = xTaskGetTickCount();

        for (int i=0; i<LOCODECK_NR_OF_TWR_ANCHORS; i++) {
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

// Loco Posisioning Protocol (LPP) handling
static void lpsHandleLppShortPacket(const uint8_t srcId, const uint8_t *data)
{
  uint8_t type = data[0];

  if (type == LPP_SHORT_ANCHORPOS) {
    if (srcId < LOCODECK_NR_OF_TWR_ANCHORS) {
      struct lppShortAnchorPos_s *newpos = (struct lppShortAnchorPos_s*)&data[1];
      options->anchorPosition[srcId].timestamp = xTaskGetTickCount();
      options->anchorPosition[srcId].x = newpos->x;
      options->anchorPosition[srcId].y = newpos->y;
      options->anchorPosition[srcId].z = newpos->z;
    }
  }
}

static void updateTagTdmaSlot(lpsTwrAlgoOptions_t * options)
{
  if (options->tdmaSlot < 0) {
    uint64_t radioAddress = configblockGetRadioAddress();
    int nslot = 1;
    for (int i=0; i<CONFIG_DECK_LOCO_TDMA_SLOTS; i++) {
      nslot *= 2;
    }
    options->tdmaSlot = radioAddress % nslot;
  }
  options->tagAddress += options->tdmaSlot;
}


static void twrTagInit(dwDevice_t *dev)
{
  updateTagTdmaSlot(options);

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

  locoDeckSetRangingState(0);
  ranging_complete = false;

  tdmaSynchronized = false;

  memset(state.distance, 0, sizeof(state.distance));
  memset(state.pressures, 0, sizeof(state.pressures));
  memset(state.failedRanging, 0, sizeof(state.failedRanging));

  dwSetReceiveWaitTimeout(dev, TWR_RECEIVE_TIMEOUT);

  dwCommitConfiguration(dev);

  rangingOk = false;
}

static bool isRangingOk()
{
  return rangingOk;
}

void uwbTwrTagSetOptions(lpsTwrAlgoOptions_t* newOptions) {
  options = newOptions;
}

float lpsTwrTagGetDistance(const uint8_t anchorId) {
  return state.distance[anchorId];
}

static bool getAnchorPosition(const uint8_t anchorId, point_t* position) {
  if (anchorId < LOCODECK_NR_OF_TWR_ANCHORS) {
    *position = options->anchorPosition[anchorId];
    return true;
  }

  return false;
}

static uint8_t getAnchorIdList(uint8_t unorderedAnchorList[], const int maxListSize) {
  for (int i = 0; i < LOCODECK_NR_OF_TWR_ANCHORS; i++) {
    unorderedAnchorList[i] = i;
  }

  return LOCODECK_NR_OF_TWR_ANCHORS;
}

static uint8_t getActiveAnchorIdList(uint8_t unorderedAnchorList[], const int maxListSize) {
  uint8_t count = 0;

  for (int i = 0; i < LOCODECK_NR_OF_TWR_ANCHORS; i++) {
    if (state.failedRanging[i] < options->rangingFailedThreshold) {
      unorderedAnchorList[count] = i;
      count++;
    }
  }

  return count;
}

uwbAlgorithm_t uwbTwrTagAlgorithm = {
  .init = twrTagInit,
  .onEvent = twrTagOnEvent,
  .isRangingOk = isRangingOk,
  .getAnchorPosition = getAnchorPosition,
  .getAnchorIdList = getAnchorIdList,
  .getActiveAnchorIdList = getActiveAnchorIdList,
};

/**
 * Log group for Two Way Ranging data
 */
LOG_GROUP_START(twr)

/**
 * @brief Successful ranging ratio with anchor 0 [%]
 */
LOG_ADD(LOG_UINT8, rangingSuccessRate0, &rangingSuccessRate[0])

/**
 * @brief Ranging attempt rate with anchor 0 [1/s]
 */
LOG_ADD(LOG_UINT8, rangingPerSec0, &rangingPerSec[0])

/**
 * @brief Successful ranging ratio with anchor 1 [%]
 */
LOG_ADD(LOG_UINT8, rangingSuccessRate1, &rangingSuccessRate[1])

/**
 * @brief Ranging attempt rate with anchor 1 [1/s]
 */
LOG_ADD(LOG_UINT8, rangingPerSec1, &rangingPerSec[1])

/**
 * @brief Successful ranging ratio with anchor 2 [%]
 */
LOG_ADD(LOG_UINT8, rangingSuccessRate2, &rangingSuccessRate[2])

/**
 * @brief Ranging attempt rate with anchor 2 [1/s]
 */
LOG_ADD(LOG_UINT8, rangingPerSec2, &rangingPerSec[2])

/**
 * @brief Successful ranging ratio with anchor 3 [%]
 */
LOG_ADD(LOG_UINT8, rangingSuccessRate3, &rangingSuccessRate[3])

/**
 * @brief Ranging attempt rate with anchor 3 [1/s]
 */
LOG_ADD(LOG_UINT8, rangingPerSec3, &rangingPerSec[3])

/**
 * @brief Successful ranging ratio with anchor 4 [%]
 */
LOG_ADD(LOG_UINT8, rangingSuccessRate4, &rangingSuccessRate[4])

/**
 * @brief Ranging attempt rate with anchor 4 [1/s]
 */
LOG_ADD(LOG_UINT8, rangingPerSec4, &rangingPerSec[4])

/**
 * @brief Successful ranging ratio with anchor 5 [%]
 */
LOG_ADD(LOG_UINT8, rangingSuccessRate5, &rangingSuccessRate[5])

/**
 * @brief Ranging attempt rate with anchor 5 [1/s]
 */
LOG_ADD(LOG_UINT8, rangingPerSec5, &rangingPerSec[5])
LOG_GROUP_STOP(twr)

/**
 * Log group for distances (ranges) to anchors aquired by Two Way Ranging (TWR)
 */
LOG_GROUP_START(ranging)
#if (LOCODECK_NR_OF_TWR_ANCHORS > 0)
/**
 * @brief Distance to anchor 0 [m]
 */
LOG_ADD(LOG_FLOAT, distance0, &state.distance[0])
#endif

#if (LOCODECK_NR_OF_TWR_ANCHORS > 1)
/**
 * @brief Distance to anchor 1 [m]
 */
LOG_ADD(LOG_FLOAT, distance1, &state.distance[1])
#endif

#if (LOCODECK_NR_OF_TWR_ANCHORS > 2)
/**
 * @brief Distance to anchor 2 [m]
 */
LOG_ADD(LOG_FLOAT, distance2, &state.distance[2])
#endif

#if (LOCODECK_NR_OF_TWR_ANCHORS > 3)
/**
 * @brief Distance to anchor 3 [m]
 */
LOG_ADD(LOG_FLOAT, distance3, &state.distance[3])
#endif

#if (LOCODECK_NR_OF_TWR_ANCHORS > 4)
/**
 * @brief Distance to anchor 4 [m]
 */
LOG_ADD(LOG_FLOAT, distance4, &state.distance[4])
#endif

#if (LOCODECK_NR_OF_TWR_ANCHORS > 5)
/**
 * @brief Distance to anchor 5 [m]
 */
LOG_ADD(LOG_FLOAT, distance5, &state.distance[5])
#endif

#if (LOCODECK_NR_OF_TWR_ANCHORS > 6)
/**
 * @brief Distance to anchor 6 [m]
 */
LOG_ADD(LOG_FLOAT, distance6, &state.distance[6])
#endif

#if (LOCODECK_NR_OF_TWR_ANCHORS > 7)
/**
 * @brief Distance to anchor 7 [m]
 */
LOG_ADD(LOG_FLOAT, distance7, &state.distance[7])
#endif

#if (LOCODECK_NR_OF_TWR_ANCHORS > 0)
LOG_ADD(LOG_FLOAT, pressure0, &state.pressures[0])
#endif
#if (LOCODECK_NR_OF_TWR_ANCHORS > 1)
LOG_ADD(LOG_FLOAT, pressure1, &state.pressures[1])
#endif
#if (LOCODECK_NR_OF_TWR_ANCHORS > 2)
LOG_ADD(LOG_FLOAT, pressure2, &state.pressures[2])
#endif
#if (LOCODECK_NR_OF_TWR_ANCHORS > 3)
LOG_ADD(LOG_FLOAT, pressure3, &state.pressures[3])
#endif
#if (LOCODECK_NR_OF_TWR_ANCHORS > 4)
LOG_ADD(LOG_FLOAT, pressure4, &state.pressures[4])
#endif
#if (LOCODECK_NR_OF_TWR_ANCHORS > 5)
LOG_ADD(LOG_FLOAT, pressure5, &state.pressures[5])
#endif
#if (LOCODECK_NR_OF_TWR_ANCHORS > 6)
LOG_ADD(LOG_FLOAT, pressure6, &state.pressures[6])
#endif
#if (LOCODECK_NR_OF_TWR_ANCHORS > 7)
LOG_ADD(LOG_FLOAT, pressure7, &state.pressures[7])
#endif
LOG_GROUP_STOP(ranging)
