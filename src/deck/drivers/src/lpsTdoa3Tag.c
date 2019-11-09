/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie firmware.
 *
 * Copyright 2018, Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * lpsTdoa3Tag.c is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with lpsTdoa3Tag.c. If not, see <http://www.gnu.org/licenses/>.
 */


/*

The tag is assumed to move around in a large system of anchors. Any anchor ids
can be used, and the same anchor id can even be used by multiple anchors as long
as they are not visible in the same area. It is assumed that the anchor density
is evenly distributed in the covered volume and that 5-20 anchors are visible
in every point. The tag is attached to a physical object and the expected
velocity is a few m/s, this means that anchors are within range for a time
period of seconds.

The implementation must handle
1. An infinite number of anchors, where around 20 are visible at one time
2. Any anchor ids
3. Dynamically changing visibility of anchors over time
4. Random TX times from anchors with possible packet collisions and packet loss

*/

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "lpsTdoa3Tag.h"
#include "tdoaEngine.h"
#include "tdoaStats.h"
#include "estimator.h"

#include "libdw1000.h"
#include "mac.h"

#define DEBUG_MODULE "TDOA3"
#include "debug.h"
#include "cfassert.h"
#include "log.h"
#include "param.h"

// Positions for sent LPP packets
#define LPS_TDOA3_TYPE 0
#define LPS_TDOA3_SEND_LPP_PAYLOAD 1

#define PACKET_TYPE_TDOA3 0x30

#define TDOA3_RECEIVE_TIMEOUT 10000

typedef struct {
  uint8_t type;
  uint8_t seq;
  uint32_t txTimeStamp;
  uint8_t remoteCount;
} __attribute__((packed)) rangePacketHeader3_t;

typedef struct {
  uint8_t id;
  uint8_t seq;
  uint32_t rxTimeStamp;
  uint16_t distance;
} __attribute__((packed)) remoteAnchorDataFull_t;

typedef struct {
  uint8_t id;
  uint8_t seq;
  uint32_t rxTimeStamp;
} __attribute__((packed)) remoteAnchorDataShort_t;

typedef struct {
  rangePacketHeader3_t header;
  uint8_t remoteAnchorData;
} __attribute__((packed)) rangePacket3_t;


// Outgoing LPP packet
static lpsLppShortPacket_t lppPacket;

static bool rangingOk;

static tdoaEngineState_t engineState;


static bool isValidTimeStamp(const int64_t anchorRxTime) {
  return anchorRxTime != 0;
}

static int updateRemoteData(tdoaAnchorContext_t* anchorCtx, const void* payload) {
  const rangePacket3_t* packet = (rangePacket3_t*)payload;
  const void* anchorDataPtr = &packet->remoteAnchorData;
  for (uint8_t i = 0; i < packet->header.remoteCount; i++) {
    remoteAnchorDataFull_t* anchorData = (remoteAnchorDataFull_t*)anchorDataPtr;

    uint8_t remoteId = anchorData->id;
    int64_t remoteRxTime = anchorData->rxTimeStamp;
    uint8_t remoteSeqNr = anchorData->seq & 0x7f;

    if (isValidTimeStamp(remoteRxTime)) {
      tdoaStorageSetRemoteRxTime(anchorCtx, remoteId, remoteRxTime, remoteSeqNr);
    }

    bool hasDistance = ((anchorData->seq & 0x80) != 0);
    if (hasDistance) {
      int64_t tof = anchorData->distance;
      if (isValidTimeStamp(tof)) {
        tdoaStorageSetTimeOfFlight(anchorCtx, remoteId, tof);

        uint8_t anchorId = tdoaStorageGetId(anchorCtx);
        tdoaStats_t* stats = &engineState.stats;
        if (anchorId == stats->anchorId && remoteId == stats->remoteAnchorId) {
          stats->tof = (uint16_t)tof;
        }
      }

      anchorDataPtr += sizeof(remoteAnchorDataFull_t);
    } else {
      anchorDataPtr += sizeof(remoteAnchorDataShort_t);
    }
  }

  return (uint8_t*)anchorDataPtr - (uint8_t*)packet;
}

static void handleLppShortPacket(tdoaAnchorContext_t* anchorCtx, const uint8_t *data, const int length) {
  uint8_t type = data[0];

  if (type == LPP_SHORT_ANCHORPOS) {
    struct lppShortAnchorPos_s *newpos = (struct lppShortAnchorPos_s*)&data[1];
    tdoaStorageSetAnchorPosition(anchorCtx, newpos->x, newpos->y, newpos->z);
  }
}

static void handleLppPacket(const int dataLength, int rangePacketLength, const packet_t* rxPacket, tdoaAnchorContext_t* anchorCtx) {
  const int32_t payloadLength = dataLength - MAC802154_HEADER_LENGTH;
  const int32_t startOfLppDataInPayload = rangePacketLength;
  const int32_t lppDataLength = payloadLength - startOfLppDataInPayload;
  const int32_t lppTypeInPayload = startOfLppDataInPayload + 1;

  if (lppDataLength > 0) {
    const uint8_t lppPacketHeader = rxPacket->payload[startOfLppDataInPayload];
    if (lppPacketHeader == LPP_HEADER_SHORT_PACKET) {
      const int32_t lppTypeAndPayloadLength = lppDataLength - 1;
      handleLppShortPacket(anchorCtx, &rxPacket->payload[lppTypeInPayload], lppTypeAndPayloadLength);
    }
  }
}

static void rxcallback(dwDevice_t *dev) {
  tdoaStats_t* stats = &engineState.stats;
  STATS_CNT_RATE_EVENT(&stats->packetsReceived);

  int dataLength = dwGetDataLength(dev);
  packet_t rxPacket;

  dwGetData(dev, (uint8_t*)&rxPacket, dataLength);
  const uint8_t anchorId = rxPacket.sourceAddress & 0xff;

  dwTime_t arrival = {.full = 0};
  dwGetReceiveTimestamp(dev, &arrival);
  const int64_t rxAn_by_T_in_cl_T = arrival.full;

  const rangePacket3_t* packet = (rangePacket3_t*)rxPacket.payload;
  if (packet->header.type == PACKET_TYPE_TDOA3) {
    const int64_t txAn_in_cl_An = packet->header.txTimeStamp;;
    const uint8_t seqNr = packet->header.seq;

    tdoaAnchorContext_t anchorCtx;
    uint32_t now_ms = T2M(xTaskGetTickCount());

    tdoaEngineGetAnchorCtxForPacketProcessing(&engineState, anchorId, now_ms, &anchorCtx);
    int rangeDataLength = updateRemoteData(&anchorCtx, packet);
    tdoaEngineProcessPacket(&engineState, &anchorCtx, txAn_in_cl_An, rxAn_by_T_in_cl_T);
    tdoaStorageSetRxTxData(&anchorCtx, rxAn_by_T_in_cl_T, txAn_in_cl_An, seqNr);
    handleLppPacket(dataLength, rangeDataLength, &rxPacket, &anchorCtx);

    rangingOk = true;
  }
}

static void setRadioInReceiveMode(dwDevice_t *dev) {
  dwNewReceive(dev);
  dwSetDefaults(dev);
  dwStartReceive(dev);
}

static void sendLppShort(dwDevice_t *dev, lpsLppShortPacket_t *packet)
{
  static packet_t txPacket;
  dwIdle(dev);

  MAC80215_PACKET_INIT(txPacket, MAC802154_TYPE_DATA);

  txPacket.payload[LPS_TDOA3_TYPE] = LPP_HEADER_SHORT_PACKET;
  memcpy(&txPacket.payload[LPS_TDOA3_SEND_LPP_PAYLOAD], packet->data, packet->length);

  txPacket.pan = 0xbccf;
  txPacket.sourceAddress = 0xbccf000000000000 | 0xff;
  txPacket.destAddress = 0xbccf000000000000 | packet->dest;

  dwNewTransmit(dev);
  dwSetDefaults(dev);
  dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+1+packet->length);

  dwStartTransmit(dev);
}

static bool sendLpp(dwDevice_t *dev) {
  bool lppPacketToSend = lpsGetLppShort(&lppPacket);
  if (lppPacketToSend) {
    sendLppShort(dev, &lppPacket);
    return true;
  }

  return false;
}

static uint32_t onEvent(dwDevice_t *dev, uwbEvent_t event) {
  switch(event) {
    case eventPacketReceived:
      rxcallback(dev);
      break;
    case eventTimeout:
      break;
    case eventReceiveTimeout:
      break;
    case eventPacketSent:
      // Service packet sent, the radio is back to receive automatically
      break;
    default:
      ASSERT_FAILED();
  }

  if(!sendLpp(dev)) {
    setRadioInReceiveMode(dev);
  }

  uint32_t now_ms = T2M(xTaskGetTickCount());
  tdoaStatsUpdate(&engineState.stats, now_ms);

  return MAX_TIMEOUT;
}

static void sendTdoaToEstimatorCallback(tdoaMeasurement_t* tdoaMeasurement) {
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
}

static bool getAnchorPosition(const uint8_t anchorId, point_t* position) {
  tdoaAnchorContext_t anchorCtx;
  uint32_t now_ms = T2M(xTaskGetTickCount());

  bool contextFound = tdoaStorageGetAnchorCtx(engineState.anchorInfoArray, anchorId, now_ms, &anchorCtx);
  if (contextFound) {
    tdoaStorageGetAnchorPosition(&anchorCtx, position);
    return true;
  }

  return false;
}

static uint8_t getAnchorIdList(uint8_t unorderedAnchorList[], const int maxListSize) {
  return tdoaStorageGetListOfAnchorIds(engineState.anchorInfoArray, unorderedAnchorList, maxListSize);
}

static uint8_t getActiveAnchorIdList(uint8_t unorderedAnchorList[], const int maxListSize) {
  uint32_t now_ms = T2M(xTaskGetTickCount());
  return tdoaStorageGetListOfActiveAnchorIds(engineState.anchorInfoArray, unorderedAnchorList, maxListSize, now_ms);
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static void Initialize(dwDevice_t *dev) {
  uint32_t now_ms = T2M(xTaskGetTickCount());
  tdoaEngineInit(&engineState, now_ms, sendTdoaToEstimatorCallback, LOCODECK_TS_FREQ);

  #ifdef LPS_2D_POSITION_HEIGHT
  DEBUG_PRINT("2D positioning enabled at %f m height\n", LPS_2D_POSITION_HEIGHT);
  #endif

  dwSetReceiveWaitTimeout(dev, TDOA3_RECEIVE_TIMEOUT);

  dwCommitConfiguration(dev);

  rangingOk = false;
}
#pragma GCC diagnostic pop

static bool isRangingOk()
{
  return rangingOk;
}

uwbAlgorithm_t uwbTdoa3TagAlgorithm = {
  .init = Initialize,
  .onEvent = onEvent,
  .isRangingOk = isRangingOk,
  .getAnchorPosition = getAnchorPosition,
  .getAnchorIdList = getAnchorIdList,
  .getActiveAnchorIdList = getActiveAnchorIdList,
};


LOG_GROUP_START(tdoa3)
STATS_CNT_RATE_LOG_ADD(stRx, &engineState.stats.packetsReceived)
STATS_CNT_RATE_LOG_ADD(stEst, &engineState.stats.packetsToEstimator)
STATS_CNT_RATE_LOG_ADD(stTime, &engineState.stats.timeIsGood)
STATS_CNT_RATE_LOG_ADD(stFound, &engineState.stats.suitableDataFound)
STATS_CNT_RATE_LOG_ADD(stCc, &engineState.stats.clockCorrection)
STATS_CNT_RATE_LOG_ADD(stHit, &engineState.stats.contextHitCount)
STATS_CNT_RATE_LOG_ADD(stMiss, &engineState.stats.contextMissCount)

LOG_ADD(LOG_FLOAT, cc, &engineState.stats.clockCorrection)
LOG_ADD(LOG_UINT16, tof, &engineState.stats.tof)
LOG_ADD(LOG_FLOAT, tdoa, &engineState.stats.tdoa)
LOG_GROUP_STOP(tdoa3)

PARAM_GROUP_START(tdoa3)
PARAM_ADD(PARAM_UINT8, logId, &engineState.stats.newAnchorId)
PARAM_ADD(PARAM_UINT8, logOthrId, &engineState.stats.newRemoteAnchorId)
PARAM_GROUP_STOP(tdoa3)
