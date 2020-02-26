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
 * lpsTdoa2Tag.c is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with lpsTdoa2Tag.c.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "log.h"
#include "param.h"
#include "lpsTdoa2Tag.h"

#include "stabilizer_types.h"
#include "cfassert.h"

#include "estimator.h"

#include "physicalConstants.h"
#include "tdoaEngineInstance.h"

#if ANCHOR_STORAGE_COUNT < LOCODECK_NR_OF_TDOA2_ANCHORS
  #error "Tdoa engine storage is too small"
#endif
#if REMOTE_ANCHOR_DATA_COUNT < LOCODECK_NR_OF_TDOA2_ANCHORS
  #error "Tdoa engine storage is too small"
#endif

// Config
static lpsTdoa2AlgoOptions_t defaultOptions = {
   .anchorAddress = {
     0xbccf000000000000,
     0xbccf000000000001,
     0xbccf000000000002,
     0xbccf000000000003,
     0xbccf000000000004,
     0xbccf000000000005,
     0xbccf000000000006,
     0xbccf000000000007,
   },
};

static lpsTdoa2AlgoOptions_t* options = &defaultOptions;

// State
typedef struct {
  uint32_t anchorStatusTimeout;
} history_t;

static uint8_t previousAnchor;
// Holds data for the latest packet from all anchors
static history_t history[LOCODECK_NR_OF_TDOA2_ANCHORS];


// LPP packet handling
static lpsLppShortPacket_t lppPacket;
static bool lppPacketToSend;
static int lppPacketSendTryCounter;

static void lpsHandleLppShortPacket(const uint8_t srcId, const uint8_t *data, tdoaAnchorContext_t* anchorCtx);

// Log data
static float logUwbTdoaDistDiff[LOCODECK_NR_OF_TDOA2_ANCHORS];
static float logClockCorrection[LOCODECK_NR_OF_TDOA2_ANCHORS];
static uint16_t logAnchorDistance[LOCODECK_NR_OF_TDOA2_ANCHORS];

static bool rangingOk;

// The default receive time in the anchors for messages from other anchors is 0
// and is overwritten with the actual receive time when a packet arrives.
// That is, if no message was received the rx time will be 0.
static bool isValidTimeStamp(const int64_t anchorRxTime) {
  return anchorRxTime != 0;
}

static bool isConsecutiveIds(const uint8_t previousAnchor, const uint8_t currentAnchor) {
  return (((previousAnchor + 1) & 0x07) == currentAnchor);
}

static void updateRemoteData(tdoaAnchorContext_t* anchorCtx, const rangePacket2_t* packet) {
  const uint8_t anchorId = tdoaStorageGetId(anchorCtx);
  for (uint8_t i = 0; i < LOCODECK_NR_OF_TDOA2_ANCHORS; i++) {
    if (anchorId != i) {
      uint8_t remoteId = i;
      int64_t remoteRxTime = packet->timestamps[i];
      uint8_t remoteSeqNr = packet->sequenceNrs[i] & 0x7f;

      if (isValidTimeStamp(remoteRxTime)) {
        tdoaStorageSetRemoteRxTime(anchorCtx, remoteId, remoteRxTime, remoteSeqNr);
      }

      bool hasDistance = (packet->distances[i] != 0);
      if (hasDistance) {
        int64_t tof = packet->distances[i];
        if (isValidTimeStamp(tof)) {
          tdoaStorageSetTimeOfFlight(anchorCtx, remoteId, tof);

          if (isConsecutiveIds(previousAnchor, anchorId)) {
            logAnchorDistance[anchorId] = packet->distances[previousAnchor];
          }
        }
      }
    }
  }
}

static void handleLppPacket(const int dataLength, const packet_t* rxPacket, tdoaAnchorContext_t* anchorCtx) {
  const int32_t payloadLength = dataLength - MAC802154_HEADER_LENGTH;
  const int32_t startOfLppDataInPayload = LPS_TDOA2_LPP_HEADER;
  const int32_t lppDataLength = payloadLength - startOfLppDataInPayload;

  if (lppDataLength > 0) {
    const uint8_t lppPacketHeader = rxPacket->payload[LPS_TDOA2_LPP_HEADER];
    if (lppPacketHeader == LPP_HEADER_SHORT_PACKET) {
      int srcId = -1;

      for (int i=0; i < LOCODECK_NR_OF_TDOA2_ANCHORS; i++) {
        if (rxPacket->sourceAddress == options->anchorAddress[i]) {
          srcId = i;
          break;
        }
      }

      if (srcId >= 0) {
        lpsHandleLppShortPacket(srcId, &rxPacket->payload[LPS_TDOA2_LPP_TYPE], anchorCtx);
      }
    }
  }
}

// Send an LPP packet, the radio will automatically go back in RX mode
static void sendLppShort(dwDevice_t *dev, lpsLppShortPacket_t *packet)
{
  static packet_t txPacket;
  dwIdle(dev);

  MAC80215_PACKET_INIT(txPacket, MAC802154_TYPE_DATA);

  txPacket.payload[LPS_TDOA2_TYPE_INDEX] = LPP_HEADER_SHORT_PACKET;
  memcpy(&txPacket.payload[LPS_TDOA2_SEND_LPP_PAYLOAD_INDEX], packet->data, packet->length);

  txPacket.pan = 0xbccf;
  txPacket.sourceAddress = 0xbccf000000000000 | 0xff;
  txPacket.destAddress = options->anchorAddress[packet->dest];

  dwNewTransmit(dev);
  dwSetDefaults(dev);
  dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+1+packet->length);

  dwWaitForResponse(dev, true);
  dwStartTransmit(dev);
}

static bool rxcallback(dwDevice_t *dev) {
  tdoaStats_t* stats = &tdoaEngineState.stats;
  STATS_CNT_RATE_EVENT(&stats->packetsReceived);

  int dataLength = dwGetDataLength(dev);
  packet_t rxPacket;

  dwGetData(dev, (uint8_t*)&rxPacket, dataLength);
  const rangePacket2_t* packet = (rangePacket2_t*)rxPacket.payload;

  bool lppSent = false;
  if (packet->type == PACKET_TYPE_TDOA2) {
    const uint8_t anchor = rxPacket.sourceAddress & 0xff;

    // Check if we need to send the current LPP packet
    if (lppPacketToSend && lppPacket.dest == anchor) {
      sendLppShort(dev, &lppPacket);
      lppSent = true;
    }

    dwTime_t arrival = {.full = 0};
    dwGetReceiveTimestamp(dev, &arrival);

    if (anchor < LOCODECK_NR_OF_TDOA2_ANCHORS) {
      uint32_t now_ms = T2M(xTaskGetTickCount());

      const int64_t rxAn_by_T_in_cl_T = arrival.full;
      const int64_t txAn_in_cl_An = packet->timestamps[anchor];
      const uint8_t seqNr = packet->sequenceNrs[anchor] & 0x7f;

      tdoaAnchorContext_t anchorCtx;
      tdoaEngineGetAnchorCtxForPacketProcessing(&tdoaEngineState, anchor, now_ms, &anchorCtx);
      updateRemoteData(&anchorCtx, packet);
      tdoaEngineProcessPacket(&tdoaEngineState, &anchorCtx, txAn_in_cl_An, rxAn_by_T_in_cl_T);
      tdoaStorageSetRxTxData(&anchorCtx, rxAn_by_T_in_cl_T, txAn_in_cl_An, seqNr);

      logClockCorrection[anchor] = tdoaStorageGetClockCorrection(&anchorCtx);

      previousAnchor = anchor;

      handleLppPacket(dataLength, &rxPacket, &anchorCtx);

      rangingOk = true;
    }
  }

  return lppSent;
}

static void setRadioInReceiveMode(dwDevice_t *dev) {
  dwNewReceive(dev);
  dwSetDefaults(dev);
  dwStartReceive(dev);
}

static uint32_t onEvent(dwDevice_t *dev, uwbEvent_t event) {
  switch(event) {
    case eventPacketReceived:
      if (rxcallback(dev)) {
        lppPacketToSend = false;
      } else {
        setRadioInReceiveMode(dev);

        // Discard lpp packet if we cannot send it for too long
        if (++lppPacketSendTryCounter >= TDOA2_LPP_PACKET_SEND_TIMEOUT) {
          lppPacketToSend = false;
        }
      }

      if (!lppPacketToSend) {
        // Get next lpp packet
        lppPacketToSend = lpsGetLppShort(&lppPacket);
        lppPacketSendTryCounter = 0;
      }
      break;
    case eventTimeout:
      setRadioInReceiveMode(dev);
      break;
    case eventReceiveTimeout:
      setRadioInReceiveMode(dev);
      break;
    case eventPacketSent:
      // Service packet sent, the radio is back to receive automatically
      break;
    default:
      ASSERT_FAILED();
  }

  uint32_t now = xTaskGetTickCount();
  uint16_t rangingState = 0;
  for (int anchor = 0; anchor < LOCODECK_NR_OF_TDOA2_ANCHORS; anchor++) {
    if (now < history[anchor].anchorStatusTimeout) {
      rangingState |= (1 << anchor);
    }
  }
  locoDeckSetRangingState(rangingState);

  return MAX_TIMEOUT;
}


static void sendTdoaToEstimatorCallback(tdoaMeasurement_t* tdoaMeasurement, const uint8_t idA, const uint8_t idB) {
  estimatorEnqueueTDOA(tdoaMeasurement);

  #ifdef LPS_2D_POSITION_HEIGHT
  // If LPS_2D_POSITION_HEIGHT is defined we assume that we are doing 2D positioning.
  // LPS_2D_POSITION_HEIGHT contains the height (Z) that the tag will be located at
  heightMeasurement_t heightData;
  heightData.timestamp = xTaskGetTickCount();
  heightData.height = LPS_2D_POSITION_HEIGHT;
  heightData.stdDev = 0.0001;
  estimatorEnqueueAbsoluteHeight(&heightData);
  #endif

  if (isConsecutiveIds(idA, idB)) {
    logUwbTdoaDistDiff[idB] = tdoaMeasurement->distanceDiff;
  }
}


static void Initialize(dwDevice_t *dev) {
  uint32_t now_ms = T2M(xTaskGetTickCount());
  tdoaEngineInit(&tdoaEngineState, now_ms, sendTdoaToEstimatorCallback, LOCODECK_TS_FREQ, TdoaEngineMatchingAlgorithmYoungest);

  previousAnchor = 0;

  lppPacketToSend = false;

  locoDeckSetRangingState(0);
  dwSetReceiveWaitTimeout(dev, TDOA2_RECEIVE_TIMEOUT);

  dwCommitConfiguration(dev);

  rangingOk = false;
}

static bool isRangingOk()
{
  return rangingOk;
}

static bool getAnchorPosition(const uint8_t anchorId, point_t* position) {
  tdoaAnchorContext_t anchorCtx;
  uint32_t now_ms = T2M(xTaskGetTickCount());

  bool contextFound = tdoaStorageGetAnchorCtx(tdoaEngineState.anchorInfoArray, anchorId, now_ms, &anchorCtx);
  if (contextFound) {
    tdoaStorageGetAnchorPosition(&anchorCtx, position);
    return true;
  }

  return false;
}

static uint8_t getAnchorIdList(uint8_t unorderedAnchorList[], const int maxListSize) {
  return tdoaStorageGetListOfAnchorIds(tdoaEngineState.anchorInfoArray, unorderedAnchorList, maxListSize);
}

static uint8_t getActiveAnchorIdList(uint8_t unorderedAnchorList[], const int maxListSize) {
  uint32_t now_ms = T2M(xTaskGetTickCount());
  return tdoaStorageGetListOfActiveAnchorIds(tdoaEngineState.anchorInfoArray, unorderedAnchorList, maxListSize, now_ms);
}

// Loco Posisioning Protocol (LPP) handling
static void lpsHandleLppShortPacket(const uint8_t srcId, const uint8_t *data, tdoaAnchorContext_t* anchorCtx)
{
  uint8_t type = data[0];

  if (type == LPP_SHORT_ANCHORPOS) {
    if (srcId < LOCODECK_NR_OF_TDOA2_ANCHORS) {
      struct lppShortAnchorPos_s *newpos = (struct lppShortAnchorPos_s*)&data[1];
      tdoaStorageSetAnchorPosition(anchorCtx, newpos->x, newpos->y, newpos->z);
    }
  }
}

uwbAlgorithm_t uwbTdoa2TagAlgorithm = {
  .init = Initialize,
  .onEvent = onEvent,
  .isRangingOk = isRangingOk,
  .getAnchorPosition = getAnchorPosition,
  .getAnchorIdList = getAnchorIdList,
  .getActiveAnchorIdList = getActiveAnchorIdList,
};

void lpsTdoa2TagSetOptions(lpsTdoa2AlgoOptions_t* newOptions) {
  options = newOptions;
}

LOG_GROUP_START(tdoa2)
LOG_ADD(LOG_FLOAT, d7-0, &logUwbTdoaDistDiff[0])
LOG_ADD(LOG_FLOAT, d0-1, &logUwbTdoaDistDiff[1])
LOG_ADD(LOG_FLOAT, d1-2, &logUwbTdoaDistDiff[2])
LOG_ADD(LOG_FLOAT, d2-3, &logUwbTdoaDistDiff[3])
LOG_ADD(LOG_FLOAT, d3-4, &logUwbTdoaDistDiff[4])
LOG_ADD(LOG_FLOAT, d4-5, &logUwbTdoaDistDiff[5])
LOG_ADD(LOG_FLOAT, d5-6, &logUwbTdoaDistDiff[6])
LOG_ADD(LOG_FLOAT, d6-7, &logUwbTdoaDistDiff[7])

LOG_ADD(LOG_FLOAT, cc0, &logClockCorrection[0])
LOG_ADD(LOG_FLOAT, cc1, &logClockCorrection[1])
LOG_ADD(LOG_FLOAT, cc2, &logClockCorrection[2])
LOG_ADD(LOG_FLOAT, cc3, &logClockCorrection[3])
LOG_ADD(LOG_FLOAT, cc4, &logClockCorrection[4])
LOG_ADD(LOG_FLOAT, cc5, &logClockCorrection[5])
LOG_ADD(LOG_FLOAT, cc6, &logClockCorrection[6])
LOG_ADD(LOG_FLOAT, cc7, &logClockCorrection[7])

LOG_ADD(LOG_UINT16, dist7-0, &logAnchorDistance[0])
LOG_ADD(LOG_UINT16, dist0-1, &logAnchorDistance[1])
LOG_ADD(LOG_UINT16, dist1-2, &logAnchorDistance[2])
LOG_ADD(LOG_UINT16, dist2-3, &logAnchorDistance[3])
LOG_ADD(LOG_UINT16, dist3-4, &logAnchorDistance[4])
LOG_ADD(LOG_UINT16, dist4-5, &logAnchorDistance[5])
LOG_ADD(LOG_UINT16, dist5-6, &logAnchorDistance[6])
LOG_ADD(LOG_UINT16, dist6-7, &logAnchorDistance[7])
LOG_GROUP_STOP(tdoa2)
