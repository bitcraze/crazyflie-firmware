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

#include "lpsTdoa3Tag.h"
#include "lpsTdoaTagEngine.h"
#include "lpsTdoaTagStats.h"

#include "libdw1000.h"
#include "mac.h"

#define DEBUG_MODULE "TDOA3"
#include "debug.h"
#include "cfassert.h"


// Positions for sent LPP packets
#define LPS_TDOA3_TYPE 0
#define LPS_TDOA3_SEND_LPP_PAYLOAD 1

#define TDOA3_LPP_PACKET_SEND_TIMEOUT (LOCODECK_NR_OF_ANCHORS * 5)

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


static lpsAlgoOptions_t* options;

// Outgoing LPP packet
static lpsLppShortPacket_t lppPacket;


// TODO krri Find better way to communicate system state to the client. Currently only supports 8 anchors
static void updateRangingState() {
  options->rangingState = 0;
//  for (int anchor = 0; anchor < LOCODECK_NR_OF_TDOA2_ANCHORS; anchor++) {
//    if (now < history[anchor].anchorStatusTimeout) {
//      options->rangingState |= (1 << anchor);
//    }
//  }
}

static bool rangingOk;


static bool isValidTimeStamp(const int64_t anchorRxTime) {
  return anchorRxTime != 0;
}

static int updateRemoteData(anchorInfo_t* anchorCtx, const void* payload) {
  const rangePacket3_t* packet = (rangePacket3_t*)payload;
  const void* anchorDataPtr = &packet->remoteAnchorData;
  for (uint8_t i = 0; i < packet->header.remoteCount; i++) {
    remoteAnchorDataFull_t* anchorData = (remoteAnchorDataFull_t*)anchorDataPtr;

    uint8_t remoteId = anchorData->id;
    int64_t remoteRxTime = anchorData->rxTimeStamp;
    uint8_t remoteSeqNr = anchorData->seq & 0x7f;

    if (isValidTimeStamp(remoteRxTime)) {
      tdoaEngineSetRemoteRxTime(anchorCtx, remoteId, remoteRxTime, remoteSeqNr);
    }

    bool hasDistance = ((anchorData->seq & 0x80) != 0);
    if (hasDistance) {
      int64_t tof = anchorData->distance;
      if (isValidTimeStamp(tof)) {
        tdoaEngineSetTimeOfFlight(anchorCtx, remoteId, tof);

        uint8_t anchorId = tdoaEngineGetId(anchorCtx);
        if (anchorId == lpsTdoaStats.anchorId && remoteId == lpsTdoaStats.remoteAnchorId) {
          lpsTdoaStats.tof = (uint16_t)tof;
        }
      }

      anchorDataPtr += sizeof(remoteAnchorDataFull_t);
    } else {
      anchorDataPtr += sizeof(remoteAnchorDataShort_t);
    }
  }

  return (uint8_t*)anchorDataPtr - (uint8_t*)packet;
}

static void handleLppShortPacket(anchorInfo_t* anchorCtx, const uint8_t *data, const int length) {
  uint8_t type = data[0];

  if (type == LPP_SHORT_ANCHORPOS) {
    struct lppShortAnchorPos_s *newpos = (struct lppShortAnchorPos_s*)&data[1];
    tdoaEngineSetAnchorPosition(anchorCtx, newpos->x, newpos->y, newpos->z);
  }
}

static void handleLppPacket(const int dataLength, int rangePacketLength, const packet_t* rxPacket, anchorInfo_t* anchorCtx) {
  const int32_t payloadLength = dataLength - MAC802154_HEADER_LENGTH;
  const int32_t startOfLppDataInPayload = rangePacketLength;
  const int32_t lppDataLength = payloadLength - startOfLppDataInPayload;
  const int32_t lppTypeInPayload = startOfLppDataInPayload + 1;

  if (lppDataLength > 0) {
    const uint8_t lppPacketHeader = rxPacket->payload[startOfLppDataInPayload];
    if (lppPacketHeader == LPP_HEADER_SHORT_PACKET) {
      const int32_t lppTypeAndPayloadLength = lppDataLength - 1;
      handleLppShortPacket(anchorCtx, &rxPacket->payload[lppTypeInPayload], lppTypeAndPayloadLength);

      // TODO krri Find better solution for communicating system state to the client
      // Send it to the "old" path to log anchor 0 - 7 positions to the client.
      lpsHandleLppShortPacket(tdoaEngineGetId(anchorCtx), &rxPacket->payload[lppTypeInPayload], lppTypeAndPayloadLength);
    }
  }
}

static void rxcallback(dwDevice_t *dev) {
  lpsTdoaStats.packetsReceived++;

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

    anchorInfo_t* anchorCtx = getAnchorCtxForPacketProcessing(anchorId);
    if (anchorCtx) {
      int rangeDataLength = updateRemoteData(anchorCtx, packet);
      tdoaEngineProcessPacket(anchorCtx, txAn_in_cl_An, rxAn_by_T_in_cl_T);
      tdoaEngineSetRxTxData(anchorCtx, rxAn_by_T_in_cl_T, txAn_in_cl_An, seqNr);
      handleLppPacket(dataLength, rangeDataLength, &rxPacket, anchorCtx);
    }

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

  lpsTdoaStatsUpdate();
  updateRangingState();

  return MAX_TIMEOUT;
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static void Initialize(dwDevice_t *dev, lpsAlgoOptions_t* algoOptions) {
  DEBUG_PRINT("TDoA 3 initialized\n");

  options = algoOptions;
  options->rangingState = 0;

  tdoaEngineInit();
  lpsTdoaStatsInit();

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
};
