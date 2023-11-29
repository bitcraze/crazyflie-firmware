/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie firmware.
 *
 * Copyright 2018-2023, Bitcraze AB
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


Hybrid Mode

In Hybrid Mode, the tag is not only passively listening for packets, but is
also transmitting, which enables Two Way Ranging with peers in the network.
The default behaviour is to send the acquired distance data to the estimator
for improved position estimation.

*/

#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"

#include "lpsTdoa3Tag.h"
#include "tdoaEngineInstance.h"
#include "tdoaStats.h"
#include "estimator.h"

#include "libdw1000.h"
#include "mac.h"

#include "param.h"
#include "autoconf.h"
#include "cf_math.h"

#include "physicalConstants.h"
#include "statsCnt.h"
#include "log.h"

#define DEBUG_MODULE "TDOA3"
#include "debug.h"
#include "cfassert.h"

// Positions for sent LPP packets
#define LPS_TDOA3_TYPE 0
#define LPS_TDOA3_SEND_LPP_PAYLOAD 1

#define PACKET_TYPE_TDOA3 0x30

#define TDOA3_RECEIVE_TIMEOUT 50000

#ifdef CONFIG_DECK_LOCO_TDOA3_HYBRID_MODE
// The delay required for the radio to be ready to transmit
#define TX_DELAY_TIME_S ( 500e-6 )
#define TX_DELAY_TIME (uint64_t)( TX_DELAY_TIME_S * LOCODECK_TS_FREQ )

#define LPP_HEADER 0
#define LPP_TYPE (LPP_HEADER + 1)
#define LPP_PAYLOAD (LPP_HEADER + 2)

#define MAX_NR_OF_ANCHORS_IN_TX 8

#define STATS_INTERVAL 500

#define SYSTEM_TX_FREQ 400.0f
#define ANCHOR_MAX_TX_FREQ 50.0f
// We need a lower limit of minimum tx rate. The TX timestamp in the protocol is
// only 32 bits (equal to 67 ms) and we want to avoid double wraps of the TX counter.
// To have some margin set the lowest tx frequency to 20 Hz (= 50 ms)
#define ANCHOR_MIN_TX_FREQ 20.0f
#define ID_COUNT 256

// Short LPP packets for position
#define SHORT_LPP 0xF0
#define LPP_SHORT_ANCHOR_POSITION 0x01

// Log ids for estimated position
static logVarId_t estimatedPosLogX;
static logVarId_t estimatedPosLogY;
static logVarId_t estimatedPosLogZ;

static const locoAddress_t base_address = 0xcfbc;

static void processTwoWayRanging(tdoaAnchorContext_t* anchorCtx, const uint32_t now_ms, const uint64_t txAn_in_cl_An, const uint64_t rxAn_by_T_in_cl_T);

typedef enum {
  txOwnPosNot = 0,
  txOwnPosEstimated,
  txOwnPosLocked
} TxOwnPosition;
#endif

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

static struct {
  float tdoaStdDev;
  bool isReceivingPackets;

  // Outgoing LPP packet
  lpsLppShortPacket_t lppPacket;

  bool isTdoaActive;

#ifdef CONFIG_DECK_LOCO_TDOA3_HYBRID_MODE

  // Hybrid mode transmission information
  int anchorId;
  uint8_t txSeqNr;
  uint32_t txT_in_cl_T; // In UWB clock ticks
  uint32_t latestTransmissionTime_ms; // System clock
  uint32_t nextTxTick;  // in system clock ticks

  uint32_t averageTxDelay;
  uint32_t nextTxDelayEvaluationTime_ms;
  uint32_t rxCount[ID_COUNT];

  // Transmit packets if true
  bool isTwrActive;

  // Indicates if position information should be transmitted when sending packets.
  // If no position information is transmitted, the CF can use TWR for positioning itself only while other CFs can not
  // use the transmitted packets for positioning.
  // If position is transmitted other CFs can use the packets for positioning.
  TxOwnPosition twrSendPosition;
  struct lppShortAnchorPos_s twrTxPos;

  // Std dev for TWR samples sent to the estimator
  float twrStdDev;

  // If TWR data should be used for position estimation
  bool useTwrForPositionEstimation;

  // Maximum acceptable age of time of flight information to be used in transmissions. The ToF information is used by
  // other CFs for TDoA positioning. A longer time is acceptable if the transmitting CF is not moving.
  uint32_t maxAgeOfTof_ms;

  statsCntRateLogger_t cntPacketsTransmitted;
  statsCntRateLogger_t cntTwrSeqNrOk;
  statsCntRateLogger_t cntTwrToEstimator;

  // Logging of hybrid mode distances
  uint8_t logDistAnchorId;
  float logDistance;
#endif
} ctx;

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
        tdoaStorageSetRemoteTimeOfFlight(anchorCtx, remoteId, tof);

        uint8_t anchorId = tdoaStorageGetId(anchorCtx);
        tdoaStats_t* stats = &tdoaEngineState.stats;
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
  tdoaStats_t* stats = &tdoaEngineState.stats;
  STATS_CNT_RATE_EVENT(&stats->packetsReceived);

  int dataLength = dwGetDataLength(dev);
  packet_t rxPacket;

  dwGetData(dev, (uint8_t*)&rxPacket, dataLength);
  const uint8_t anchorId = rxPacket.sourceAddress & 0xff;

#ifdef CONFIG_DECK_LOCO_TDOA3_HYBRID_MODE
  ctx.rxCount[anchorId]++;
#endif

  dwTime_t arrival = {.full = 0};
  dwGetReceiveTimestamp(dev, &arrival);
  const int64_t rxAn_by_T_in_cl_T = arrival.full;

  const rangePacket3_t* packet = (rangePacket3_t*)rxPacket.payload;
  if (packet->header.type == PACKET_TYPE_TDOA3) {
    const int64_t txAn_in_cl_An = packet->header.txTimeStamp;
    const uint8_t seqNr = packet->header.seq & 0x7f;

    uint32_t now_ms = T2M(xTaskGetTickCount());
    tdoaAnchorContext_t anchorCtx;
    tdoaEngineGetAnchorCtxForPacketProcessing(&tdoaEngineState, anchorId, now_ms, &anchorCtx);
    int rangeDataLength = updateRemoteData(&anchorCtx, packet);

#ifdef CONFIG_DECK_LOCO_TDOA3_HYBRID_MODE
    const bool doExcludeId = ctx.isTwrActive;
    const uint8_t excludedId = ctx.anchorId;
    const bool timeIsGood = tdoaEngineProcessPacketFiltered(&tdoaEngineState, &anchorCtx, txAn_in_cl_An, rxAn_by_T_in_cl_T, doExcludeId, excludedId);
#else
    const bool doExcludeId = false;
    const uint8_t excludedId = 0;
    tdoaEngineProcessPacketFiltered(&tdoaEngineState, &anchorCtx, txAn_in_cl_An, rxAn_by_T_in_cl_T, doExcludeId, excludedId);
#endif

    tdoaStorageSetRxTxData(&anchorCtx, rxAn_by_T_in_cl_T, txAn_in_cl_An, seqNr);
    handleLppPacket(dataLength, rangeDataLength, &rxPacket, &anchorCtx);

#ifdef CONFIG_DECK_LOCO_TDOA3_HYBRID_MODE
    if (ctx.isTwrActive) {
      if (timeIsGood) {
        processTwoWayRanging(&anchorCtx, now_ms, txAn_in_cl_An, rxAn_by_T_in_cl_T);
      }
    }
#endif

    ctx.isReceivingPackets = true;
  }
}

static void setRadioInReceiveMode(dwDevice_t *dev) {
  dwNewReceive(dev);
  dwSetDefaults(dev);
  dwStartReceive(dev);
}

#ifdef CONFIG_DECK_LOCO_TDOA3_HYBRID_MODE

static void processTwoWayRanging(tdoaAnchorContext_t* anchorCtx, const uint32_t now_ms, const uint64_t txAn_in_cl_An, const uint64_t rxAn_by_T_in_cl_T) {
  // We assume updateRemoteData() has been called before this function
  // and that the remote data from the current packet is in the storage, as we read data from the storage.

  const uint32_t MAX_TIME_SINCE_TRANSMISSION_MS = 100;
  const bool isLatestTransmisionTimeClose = ((now_ms - ctx.latestTransmissionTime_ms) < MAX_TIME_SINCE_TRANSMISSION_MS);
  if (isLatestTransmisionTimeClose) {
    uint8_t latestSeqReceivedByRemote = 255;
    int64_t rxT_by_An_in_cl_An = 0;
    if (tdoaStorageGetRemoteRxTimeSeqNr(anchorCtx, ctx.anchorId, &rxT_by_An_in_cl_An, &latestSeqReceivedByRemote)) {
      STATS_CNT_RATE_EVENT(&ctx.cntTwrSeqNrOk);
      const bool isLatestTxPacketReturnedFromRemote = (ctx.txSeqNr == latestSeqReceivedByRemote);
      if (isLatestTxPacketReturnedFromRemote) {
        const double clockCorrection = tdoaStorageGetClockCorrection(anchorCtx);
        int64_t t_in_anchor_T = (int64_t)(clockCorrection * tdoaEngineTruncateToAnchorTimeStamp(txAn_in_cl_An - rxT_by_An_in_cl_An));
        int64_t t_since_tx_T = tdoaEngineTruncateToAnchorTimeStamp(rxAn_by_T_in_cl_T - ctx.txT_in_cl_T);
        int64_t tof_T = (t_since_tx_T - t_in_anchor_T) / 2;

        float distance = SPEED_OF_LIGHT * (tof_T - LOCODECK_ANTENNA_DELAY) / LOCODECK_TS_FREQ;
        tdoaStorageSetTimeOfFlight(anchorCtx, tof_T, now_ms);

        if (tdoaStorageGetId(anchorCtx) == ctx.logDistAnchorId) {
          ctx.logDistance = distance;
        }

        point_t position;
        if (tdoaStorageGetAnchorPosition(anchorCtx, &position)) {
          distanceMeasurement_t measurement = {
            .distance = distance,
            .stdDev = ctx.twrStdDev,
            .x = position.x,
            .y = position.y,
            .z = position.z,
          };

          if (ctx.useTwrForPositionEstimation) {
            estimatorEnqueueDistance(&measurement);
            STATS_CNT_RATE_EVENT(&ctx.cntTwrToEstimator);
          }
        }
      }
    }
  }
}

static int countSeenAnchorsAndClearCounters() {
  int anchorsCount = 0;

  for (int i = 0; i < ID_COUNT; i++) {
    if (ctx.rxCount[i] != 0) {
      anchorsCount++;
      ctx.rxCount[i] = 0;
    }
  }

  return anchorsCount;
}

static uint32_t updateAverageTxDelay(const uint32_t now_ms) {
  if (now_ms > ctx.nextTxDelayEvaluationTime_ms) {
    const uint32_t evalutationPeriod_ms = 500;
    int anchorsCount = countSeenAnchorsAndClearCounters();

    // Set the TX rate based on the number of transmitting anchors around us
    // Aim for 400 messages/s. Up to 8 anchors: 50 Hz / anchor
    float freq = SYSTEM_TX_FREQ / (anchorsCount + 1);
    if (freq > ANCHOR_MAX_TX_FREQ) {
      freq = ANCHOR_MAX_TX_FREQ;
    }
    if (freq < ANCHOR_MIN_TX_FREQ) {
      freq = ANCHOR_MIN_TX_FREQ;
    }
    ctx.averageTxDelay = 1000.0f / freq;

    ctx.nextTxDelayEvaluationTime_ms = now_ms + evalutationPeriod_ms;
  }

  return ctx.averageTxDelay;
}

static uint32_t randomizeDelayToNextTx(const uint32_t now_ms) {
  const uint32_t interval = 10;

  uint32_t averageTxDelay = updateAverageTxDelay(now_ms);

  uint32_t r = rand();
  uint32_t delay = averageTxDelay + r % interval - interval / 2;

  return delay;
}

static dwTime_t findTransmitTimeAsSoonAsPossible(dwDevice_t *dev) {
  dwTime_t transmitTime = { .full = 0 };
  dwGetSystemTimestamp(dev, &transmitTime);

  // Add some extra time to make sure the radio is ready to transmit, taking into account that this task may
  // be halted for a while. This time also includes time for the preamble (128 * 1017.63e-9 s).
  transmitTime.full += TX_DELAY_TIME;
  return transmitTime;
}

static int populateTxData(rangePacket3_t *rangePacket, const uint32_t now_ms) {
  // rangePacket->header.type already populated
  rangePacket->header.seq = ctx.txSeqNr;
  rangePacket->header.txTimeStamp = ctx.txT_in_cl_T;

  uint8_t remoteAnchorCount = 0;
  uint8_t* anchorDataPtr = &rangePacket->remoteAnchorData;

  // Consider a more clever selection of which anchors to include as remote data.
  // This implementation will give a somewhat randomized set but can probably be improved
  uint8_t ids[MAX_NR_OF_ANCHORS_IN_TX];
  uint8_t anchorCount = tdoaStorageGetListOfActiveAnchorIds(tdoaEngineState.anchorInfoArray, ids, MAX_NR_OF_ANCHORS_IN_TX, now_ms);

  for (uint8_t i = 0; i < anchorCount; i++) {
    remoteAnchorDataFull_t* anchorData = (remoteAnchorDataFull_t*) anchorDataPtr;

    uint32_t now_ms = T2M(xTaskGetTickCount());

    uint8_t id = ids[i];
    tdoaAnchorContext_t anchorCtx;
    tdoaStorageGetAnchorCtx(tdoaEngineState.anchorInfoArray, id, now_ms, &anchorCtx);

    anchorData->id = id;
    anchorData->seq = tdoaStorageGetSeqNr(&anchorCtx);
    anchorData->rxTimeStamp = tdoaStorageGetRxTime(&anchorCtx);

    uint64_t tof = tdoaStorageGetTimeOfFlight(&anchorCtx, now_ms - ctx.maxAgeOfTof_ms);
    if (tof > 0 && tof <= 0xfffful) {
      anchorData->distance = tof;
      anchorDataPtr += sizeof(remoteAnchorDataFull_t);
      anchorData->seq |= 0x80;  // Set bit to indicate tof is included
    } else {
      anchorDataPtr += sizeof(remoteAnchorDataShort_t);
    }

    remoteAnchorCount++;
  }
  rangePacket->header.remoteCount = remoteAnchorCount;

  return (uint8_t*)anchorDataPtr - (uint8_t*)rangePacket;
}

// Set TX data in the radio TX buffer
static void setTxData(dwDevice_t *dev, const uint32_t now_ms)
{
  static packet_t txPacket;
  static bool firstEntry = true;
  static int lppLength = 0;

  if (firstEntry) {
    MAC80215_PACKET_INIT(txPacket, MAC802154_TYPE_DATA);
    txPacket.destAddress = (base_address & 0xffffffffffffff00) | 0xff;
    txPacket.payload[0] = PACKET_TYPE_TDOA3;

    firstEntry = false;
  }

  txPacket.sourceAddress = (base_address & 0xffffffffffffff00) | ctx.anchorId;

  int rangePacketSize = populateTxData((rangePacket3_t *)txPacket.payload, now_ms);

  if (ctx.twrSendPosition != txOwnPosLocked) {
      // Store current estimated position
      ctx.twrTxPos.x = logGetFloat(estimatedPosLogX);
      ctx.twrTxPos.y = logGetFloat(estimatedPosLogY);
      ctx.twrTxPos.z = logGetFloat(estimatedPosLogZ);
  }

  if (ctx.twrSendPosition != txOwnPosNot) {
    txPacket.payload[rangePacketSize + LPP_HEADER] = SHORT_LPP;
    txPacket.payload[rangePacketSize + LPP_TYPE] = LPP_SHORT_ANCHOR_POSITION;

    struct lppShortAnchorPos_s *pos = (struct lppShortAnchorPos_s*) &txPacket.payload[rangePacketSize + LPP_PAYLOAD];
    *pos = ctx.twrTxPos;

    lppLength = 2 + sizeof(struct lppShortAnchorPos_s);
  } else {
    lppLength = 0;
  }

  dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH + rangePacketSize + lppLength);
}


// Setup the radio to send a ranging packet
static void setupTx(dwDevice_t *dev, const uint32_t now_ms) {
  dwTime_t txTime = findTransmitTimeAsSoonAsPossible(dev);
  ctx.txT_in_cl_T = txTime.low32;
  ctx.txSeqNr = (ctx.txSeqNr + 1) & 0x7f;

  dwIdle(dev);

  setTxData(dev, now_ms);

  dwNewTransmit(dev);
  dwSetDefaults(dev);
  dwSetTxRxTime(dev, txTime);

  dwStartTransmit(dev);
}
#endif

static void sendLppShort(dwDevice_t *dev, lpsLppShortPacket_t *packet) {
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
  bool lppPacketToSend = lpsGetLppShort(&ctx.lppPacket);
  if (lppPacketToSend) {
    sendLppShort(dev, &ctx.lppPacket);
    return true;
  }

  return false;
}

static uint32_t startNextEvent(dwDevice_t *dev, const uint32_t now) {
  uint32_t timeout = 500;

  bool isTxPending = sendLpp(dev);

#ifdef CONFIG_DECK_LOCO_TDOA3_HYBRID_MODE
  if (ctx.isTwrActive) {
    if (!isTxPending) {
      if (ctx.nextTxTick < now) {
        const uint32_t now_ms = T2M(now);
        setupTx(dev, now_ms);
        isTxPending = true;
        ctx.latestTransmissionTime_ms = now_ms;
        STATS_CNT_RATE_EVENT(&ctx.cntPacketsTransmitted);

        uint32_t newDelay = randomizeDelayToNextTx(now_ms);
        ctx.nextTxTick = now + newDelay;
      }
    }

    timeout = ctx.nextTxTick - now;
  }
#endif

  if (!isTxPending) {
    setRadioInReceiveMode(dev);
  }

  return timeout;
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
    case eventReceiveFailed:
      break;
    case eventPacketSent:
      break;
    default:
      ASSERT_FAILED();
  }

  uint32_t now = xTaskGetTickCount();
  tdoaStatsUpdate(&tdoaEngineState.stats, T2M(now));

  uint32_t timeout = startNextEvent(dev, now);
  return timeout;
}

static void sendTdoaToEstimatorCallback(tdoaMeasurement_t* tdoaMeasurement) {
  if (ctx.isTdoaActive) {
    // Override the default standard deviation set by the TDoA engine.
    tdoaMeasurement->stdDev = ctx.tdoaStdDev;

    estimatorEnqueueTDOA(tdoaMeasurement);

    #ifdef CONFIG_DECK_LOCO_2D_POSITION
    heightMeasurement_t heightData;
    heightData.timestamp = xTaskGetTickCount();
    heightData.height = DECK_LOCO_2D_POSITION_HEIGHT;
    heightData.stdDev = 0.0001;
    estimatorEnqueueAbsoluteHeight(&heightData);
    #endif
  }
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

static void Initialize(dwDevice_t *dev) {
  uint32_t now_ms = T2M(xTaskGetTickCount());
  tdoaEngineInit(&tdoaEngineState, now_ms, sendTdoaToEstimatorCallback, LOCODECK_TS_FREQ, TdoaEngineMatchingAlgorithmRandom);

  #ifdef CONFIG_DECK_LOCO_2D_POSITION
  DEBUG_PRINT("2D positioning enabled at %f m height\n", DECK_LOCO_2D_POSITION_HEIGHT);
  #endif

  dwSetReceiveWaitTimeout(dev, TDOA3_RECEIVE_TIMEOUT);

  dwCommitConfiguration(dev);

  ctx.tdoaStdDev = TDOA_ENGINE_MEASUREMENT_NOISE_STD;
  ctx.isTdoaActive = true;
  ctx.isReceivingPackets = false;

#ifdef CONFIG_DECK_LOCO_TDOA3_HYBRID_MODE
  ctx.anchorId = 254;
  ctx.txSeqNr = 0;
  ctx.latestTransmissionTime_ms = 0;
  ctx.nextTxTick = 0;
  ctx.isTwrActive = false;
  ctx.twrSendPosition = txOwnPosNot;
  ctx.twrStdDev = 0.25;
  ctx.useTwrForPositionEstimation = false;
  ctx.maxAgeOfTof_ms = 200;

  ctx.averageTxDelay = 1000.0f / ANCHOR_MAX_TX_FREQ;
  ctx.nextTxDelayEvaluationTime_ms = 0;

  // Get log ids to acquire the current estimated position
  estimatedPosLogX = logGetVarId("stateEstimate", "x");
  estimatedPosLogY = logGetVarId("stateEstimate", "y");
  estimatedPosLogZ = logGetVarId("stateEstimate", "z");

  STATS_CNT_RATE_INIT(&ctx.cntPacketsTransmitted, STATS_INTERVAL);
  STATS_CNT_RATE_INIT(&ctx.cntTwrSeqNrOk, STATS_INTERVAL);
  STATS_CNT_RATE_INIT(&ctx.cntTwrToEstimator, STATS_INTERVAL);
#endif
}

static bool isRangingOk() {
  return ctx.isReceivingPackets;
}

uwbAlgorithm_t uwbTdoa3TagAlgorithm = {
  .init = Initialize,
  .onEvent = onEvent,
  .isRangingOk = isRangingOk,
  .getAnchorPosition = getAnchorPosition,
  .getAnchorIdList = getAnchorIdList,
  .getActiveAnchorIdList = getActiveAnchorIdList,
};

#ifdef CONFIG_DECK_LOCO_TDOA3_HYBRID_MODE
LOG_GROUP_START(tdoa3)
  /**
   * @brief Transmission rate of TWR packets in hybrid mode [packets/s]
   */
  STATS_CNT_RATE_LOG_ADD(hmTx, &ctx.cntPacketsTransmitted)

  /**
   * @brief Rate of received TWR packets with matching seq nr in hybrid mode [packets/s]
   */
  STATS_CNT_RATE_LOG_ADD(hmSeqOk, &ctx.cntTwrSeqNrOk)

  /**
   * @brief Rate of TWR measurements that are sent to the estimator in hybrid mode [samples/s]
   */
  STATS_CNT_RATE_LOG_ADD(hmEst, &ctx.cntTwrToEstimator)

  /**
   * @brief Measured distance to the anchor selected by the tdoa3.hmAnchLog parameter in hybrid mode [m]
   */
  LOG_ADD(LOG_FLOAT, hmDist, &ctx.logDistance)
LOG_GROUP_STOP(tdoa3)
#endif

PARAM_GROUP_START(tdoa3)
/**
 * @brief The measurement noise to use when sending TDoA measurements to the estimator.
 */
PARAM_ADD(PARAM_FLOAT, stddev, &ctx.tdoaStdDev)

#ifdef CONFIG_DECK_LOCO_TDOA3_HYBRID_MODE
  /**
   * @brief Anchor id used when transmitting packets in hybrid mode. Don't use 255 as this means broadcast and is
   * received by all nodes.
   */
  PARAM_ADD(PARAM_UINT8, hmId, &ctx.anchorId)

  /**
   * @brief If non-zero use TDoA for position estimation in hybrid mode
   */
  PARAM_ADD(PARAM_UINT8, hmTdoa, &ctx.isTdoaActive)

  /**
   * @brief If non-zero transmit TWR packets in hybrid mode
   */
  PARAM_ADD(PARAM_UINT8, hmTwr, &ctx.isTwrActive)

  /**
   * @brief Include position information in transmitted TWR packets for other CFs to use for positioning
   * 0 = do not send TWR packets
   * 1 = use the current estimated position
   * 2 = use the estimated position that was sampled when this parameter was set to 2
   */
  PARAM_ADD(PARAM_UINT8, hmTwrTXPos, &ctx.twrSendPosition)

  /**
   * @brief If non-zero use data from the TWR process for position estimation in hybrid mode. Requires hmTwr to be enabled.
   */
  PARAM_ADD(PARAM_UINT8, hmTwrEstPos, &ctx.useTwrForPositionEstimation)

  /**
   * @brief Max age of tof to include in transmitted packets.
   */
  PARAM_ADD(PARAM_UINT32, hmTofAge, &ctx.maxAgeOfTof_ms)

  /**
   * @brief Select an anchor id to use for logging distance to a specific anchor. The distance is available in the
   * tdoa3.hmDist log variable. Used in hybrid mode.
   */
  PARAM_ADD(PARAM_UINT8, hmAnchLog, &ctx.logDistAnchorId)

  /**
   * @brief The measurement noise to use when sending TWR measurements to the estimator in hybrid mode
   */
  PARAM_ADD(PARAM_FLOAT, twrStd, &ctx.twrStdDev)
#endif
PARAM_GROUP_STOP(tdoa3)
