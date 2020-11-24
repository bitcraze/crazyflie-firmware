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

/*
    lpsTdoa3Tag.c

    Created on :  Nov., 24, 2020
        Author : Wenda Zhao
        Email  : wenda.zhao@robotics.utias.utoronto.ca
    TDOA3_plus, add inter-drone ranging and info. exchange together with TDOA3 meas. from anchors
*/
#include <string.h>
#include <stdlib.h>   // [change] for rand() function 
#include "FreeRTOS.h"
#include "task.h"

#include "lpsTdoa3Tag.h"
#include "tdoaEngineInstance.h"
#include "tdoaStats.h"
#include "estimator.h"

#include "libdw1000.h"
#include "mac.h"

#define DEBUG_MODULE "TDOA3"
#include "debug.h"
#include "cfassert.h"
// [Change]
#include "lpsTdoa4Tag.h"
#include "log.h"

// Positions for sent LPP packets
#define LPS_TDOA3_TYPE 0
#define LPS_TDOA3_SEND_LPP_PAYLOAD 1
#define PACKET_TYPE_TDOA3 0x30
#define TDOA3_RECEIVE_TIMEOUT 10000

// ----------- [Change] ----------- //
// tdoa4 protocol version
#define PACKET_TYPE_TDOA4 0x60
#define LPP_HEADER 0
#define LPP_TYPE (LPP_HEADER + 1)
#define LPP_PAYLOAD (LPP_HEADER + 2)
static int Agent_Id;            // global variable, defined in lpsTdoa4Tag.c
// Useful constants
static const uint8_t base_address[] = {0,0,0,0,0,0,0xcf,0xbc};
// ------------------------------- //

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
/* ----------------------------- Data structure used for tdoa4 feature ----------------------------- */
// define it's own data structure for packets from agents (to avoid code confusion)
typedef struct {
    uint8_t type;
    uint8_t seq;
    uint32_t txTimeStamp;
    uint8_t remoteCount;
} __attribute__((packed)) rangePacketHeader4_t;

typedef struct {
    rangePacketHeader4_t header;
    uint8_t remoteAgentData;
} __attribute__((packed)) rangePacket4_t;

typedef struct {
    uint8_t id;
    uint8_t seq;
    uint32_t rxTimeStamp;
    uint16_t distance;
} __attribute__((packed)) remoteAgentDataFull_t;

typedef struct {
    uint8_t id;
    uint8_t seq;
    uint32_t rxTimeStamp;
} __attribute__((packed)) remoteAgentDataShort_t;

// Agent context
typedef struct {
    uint8_t id;
    bool isUsed;
    uint8_t seqNr;
    uint32_t rxTimeStamp;
    uint32_t txTimeStamp;
    uint16_t distance;
    uint32_t distanceUpdateTime;
    bool isDataGoodForTransmission;

    double clockCorrection;
    int clockCorrectionBucket;
} agentContext_t;
// This context struct contains all the required global values of the inter-agent algorithm
static struct ctx_s {
    int agentId;    //[change] anchorId->agentId
    // Information about latest transmitted packet
    uint8_t seqNr;
    uint32_t txTime; // In UWB clock ticks

    // Next transmit time in system clock ticks
    uint32_t nextTxTick;
    int averageTxDelay; // ms

    // List of ids to transmit in remote data section
    uint8_t remoteTxId[REMOTE_TX_MAX_COUNT];
    uint8_t remoteTxIdCount;

    // The list of agents to transmit and store is updated at regular intervals
    uint32_t nextAgentListUpdate;

    // Remote agent data
    uint8_t agentCtxLookup[ID_COUNT];
    agentContext_t agentCtx[AGENT_STORAGE_COUNT];
    uint8_t agentRxCount[ID_COUNT];
} ctx;

// lppShortAnchorPos_s is defined in locodeck.h, here we define a new msg for TDoA4 
// [New] lpp packet (transmission data): limitation is 11 float num
struct lppShortAgent_s {
  float position[3];
  float quaternion[4];
  float imu[6];
} __attribute__((packed));
// [New] Define a struct containing the info of remote "anchor" --> agent
// global variable
static struct remoteAgentInfo_s{
    int remoteAgentID;           // source Agent 
    int destAgentID;             // destination Agent
    bool hasDistance;
    struct lppShortAgent_s Pose;
    double ranging;
}remoteAgentInfo;                //[re-design]
/* ------------------------------------------------------------------------------------------------- */
//log parameters
static float log_range;    // distance is uint16_t
/* ----------------------------- Function used for tdoa4 feature ----------------------------- */
static agentContext_t* getContext(uint8_t agentId) {
  uint8_t slot = ctx.agentCtxLookup[agentId];

  if (slot == ID_WITHOUT_CONTEXT) {
    return 0;
  }
  return &ctx.agentCtx[slot];
}

static void clearAgentRxCount() {
  memset(&ctx.agentRxCount, 0, ID_COUNT);
}

static void removeAgentContextsNotInList(const uint8_t* id, const uint8_t count) {
  for (int i = 0; i < AGENT_STORAGE_COUNT; i++) {
    agentContext_t* agentCtx = &ctx.agentCtx[i];
    if (agentCtx->isUsed) {
      const uint8_t ctxId = agentCtx->id;
      bool found = false;
      for (int j = 0; j < count; j++) {
        if (id[j] == ctxId) {
          found = true;
          break;
        }
      }

      if (!found) {
        ctx.agentCtxLookup[ctxId] = ID_WITHOUT_CONTEXT;
        agentCtx->isUsed = false;
      }
    }
  }
}

static void createAgentContext(const uint8_t id) {
  if (ctx.agentCtxLookup[id] != ID_WITHOUT_CONTEXT) {
    // Already has a context, we're done
    return;
  }

  for (uint8_t i = 0; i < AGENT_STORAGE_COUNT; i++) {
    agentContext_t* agentCtx = &ctx.agentCtx[i];
    if (!agentCtx->isUsed) {
      ctx.agentCtxLookup[id] = i;

      memset(agentCtx, 0, sizeof(agentContext_t));
      agentCtx->id = id;
      agentCtx->isUsed = true;

      break;
    }
  }
}

static void createAgentContextsInList(const uint8_t* id, const uint8_t count) {
  for (uint8_t i = 0; i < count; i++) {
    createAgentContext(id[i]);
  }
}

static void purgeData() { // clear the data
  uint32_t now = xTaskGetTickCount();
  uint32_t acceptedCreationTime = now - DISTANCE_VALIDITY_PERIOD;

  for (int i = 0; i < AGENT_STORAGE_COUNT; i++) {
    agentContext_t* agentCtx = &ctx.agentCtx[i];
    if (agentCtx->isUsed) {
      if (agentCtx->distanceUpdateTime < acceptedCreationTime) {
        agentCtx->distance = 0;
        agentCtx->clockCorrection = 0.0;
        agentCtx->clockCorrectionBucket = 0;
      }
    }
  }
}

// This function is called at regular intervals to update lists containing data
// about which anchors to store and add to outgoing messages. This
// update might take some time but this should not be a problem since the TX
// times are randomized anyway. The intention is that we could plug in clever
// algorithms here that optimizes which anchors to use.
static void updateAgentLists() {
  // Randomize which anchors to use

  static uint8_t availableId[ID_COUNT];
  static bool availableUsed[ID_COUNT];
  memset(availableId, 0, sizeof(availableId));
  memset(availableUsed, 0, sizeof(availableUsed));
  int availableCount = 0;

  static uint8_t ctxts[AGENT_STORAGE_COUNT];
  memset(ctxts, 0, sizeof(ctxts));

  // Collect all anchors we have got a message from
  for (int i = 0; i < ID_COUNT; i++) {
    if (ctx.agentRxCount[i] != 0) {
      availableId[availableCount++] = i;
    }
  }

  // Out of all anchors that we have received messages from, pick two
  // randomized subsets for storage and TX ids
  uint8_t remoteTXIdIndex = 0;
  uint8_t contextIndex = 0;
  for (int i = 0; i < AGENT_STORAGE_COUNT; i++) {
    int start = rand() % availableCount;
    // Scan forward until we find an anchor
    for (int j = start; j < (start + availableCount); j++) {
      const int index = j % availableCount;
      if (!availableUsed[index]) {

        const int id = availableId[index];
        if (remoteTXIdIndex < REMOTE_TX_MAX_COUNT) {
          ctx.remoteTxId[remoteTXIdIndex++] = id;
        }
        if (contextIndex < AGENT_STORAGE_COUNT) {
          ctxts[contextIndex++] = id;
        }
        availableUsed[index] = true;
        break;
      }
    }
  }

  removeAgentContextsNotInList(ctxts, contextIndex);
  createAgentContextsInList(ctxts, contextIndex);

  ctx.remoteTxIdCount = remoteTXIdIndex;

  clearAgentRxCount();
  
  // [Note]: organize the msg freq between agents
  // Set the TX rate based on the number of transmitting agents around us
  // Aim for 400 messages/s. Up to 8 agents: 50 Hz / anchor
  float freq = SYSTEM_TX_FREQ / (availableCount + 1);
  if (freq > (float) AGENT_MAX_TX_FREQ) {  //[change]: add (float)
    freq = AGENT_MAX_TX_FREQ;
  }
  if (freq < (float) AGENT_MIN_TX_FREQ) { //[change]: add (float)
    freq = AGENT_MIN_TX_FREQ;
  }
  ctx.averageTxDelay = 1000.0f / freq;

  purgeData();
}
/* Adjust time for schedule transfer by DW1000 radio. Set 9 LSB to 0 and round up */
static void adjustTxRxTime(dwTime_t *time)
{
  time->full = (time->full & ~((1 << 9) - 1)) + (1 << 9);
}

static dwTime_t findTransmitTimeAsSoonAsPossible(dwDevice_t *dev)
{
  dwTime_t transmitTime = { .full = 0 };
  dwGetSystemTimestamp(dev, &transmitTime);

  // Add guard and preamble time
  transmitTime.full += TDMA_GUARD_LENGTH;
  transmitTime.full += PREAMBLE_LENGTH;

  // And some extra
  transmitTime.full += TDMA_EXTRA_LENGTH;

  // DW1000 can only schedule time with 9 LSB at 0, adjust for it
  adjustTxRxTime(&transmitTime);

  return transmitTime;
}

static double calculateClockCorrection(agentContext_t* agentCtx, int remoteTxSeqNr, uint32_t remoteTx, uint32_t rx)
{
  double result = 0.0d;

  // Assigning to uint32_t truncates the diffs and takes care of wrapping clocks
  uint32_t tickCountRemote = remoteTx - agentCtx->txTimeStamp;
  uint32_t tickCountLocal = rx - agentCtx->rxTimeStamp;

  if (tickCountRemote != 0) {
    result = (double)tickCountLocal / (double)tickCountRemote;
  }

  return result;
}

static uint16_t calculateDistance(agentContext_t* agentCtx, int remoteRxSeqNr, uint32_t remoteTx, uint32_t remoteRx, uint32_t rx)
{
  // Check that the remote received seq nr is our latest tx seq nr
  if (remoteRxSeqNr == ctx.seqNr && agentCtx->clockCorrection > 0.0d) {
    uint32_t localTime = rx - ctx.txTime;
    uint32_t remoteTime = (uint32_t)((double)(remoteTx - remoteRx) * agentCtx->clockCorrection);
    uint32_t distance = (localTime - remoteTime) / 2;

    return distance & 0xfffful;
  } else {
    return 0;
  }
}
// [change] use the data structure defined for packets from agents
static bool extractFromPacket(const rangePacket4_t* rangePacket, uint32_t* remoteRx, uint8_t* remoteRxSeqNr) {
  const void* agentDataPtr = &rangePacket->remoteAgentData;
  // loop over all the remote agents' info
    for (uint8_t i = 0; i < rangePacket->header.remoteCount; i++) {
        remoteAgentDataFull_t* agentData = (remoteAgentDataFull_t*)agentDataPtr;
        // if the radio packet is sending to this agent --> to twr
        const uint8_t id = agentData->id;
        if (id == ctx.agentId) {
        *remoteRxSeqNr = agentData->seq & 0x7f;
        *remoteRx = agentData->rxTimeStamp;
        return true;
        }
        // else --> move the pointer away from the distance msg
        // currently the other agents distance msg is not used 
        bool hasDistance = ((agentData->seq & 0x80) != 0);
        if (hasDistance) {
        agentDataPtr += sizeof(remoteAgentDataFull_t);
        } else {
        agentDataPtr += sizeof(remoteAgentDataShort_t);
        }
    }
    return false;
}

static void fillClockCorrectionBucket(agentContext_t* agentCtx) {
    if (agentCtx->clockCorrectionBucket < CLOCK_CORRECTION_BUCKET_MAX) {
      agentCtx->clockCorrectionBucket++;
    }
}

static bool emptyClockCorrectionBucket(agentContext_t* agentCtx) {
    if (agentCtx->clockCorrectionBucket > 0) {
      agentCtx->clockCorrectionBucket--;
      return false;
    }
    return true;
}

static bool updateClockCorrection(agentContext_t* agentCtx, double clockCorrection) {
  const double diff = clockCorrection - agentCtx->clockCorrection;
  bool sampleIsAccepted = false;

  if (-CLOCK_CORRECTION_ACCEPTED_NOISE < diff && diff < CLOCK_CORRECTION_ACCEPTED_NOISE) {
    // LP filter
    agentCtx->clockCorrection = agentCtx->clockCorrection * (1.0d - CLOCK_CORRECTION_FILTER) + clockCorrection * CLOCK_CORRECTION_FILTER;

    fillClockCorrectionBucket(agentCtx);
    sampleIsAccepted = true;
  } else {
    if (emptyClockCorrectionBucket(agentCtx)) {
      if (CLOCK_CORRECTION_SPEC_MIN < clockCorrection && clockCorrection < CLOCK_CORRECTION_SPEC_MAX) {
        agentCtx->clockCorrection = clockCorrection;
      }
    }
  }

  return sampleIsAccepted;
}

//[New]: get the LPP transmitted data --- tdoa4
static void handleLppShortPacket_tdoa4(const uint8_t *data, const int length) {
  uint8_t type = data[0];
  if (type == LPP_SHORT_AGENT_INFO) {
    struct lppShortAgent_s *pose = (struct lppShortAgent_s*)&data[1];
    // printf("Position data is: (%f,%f,%f) \r\n", pos->x, pos->y, pos->z);
    // save and use the remote angent data
    remoteAgentInfo.Pose.position[0] = pose->position[0];
    remoteAgentInfo.Pose.position[1] = pose->position[1];
    remoteAgentInfo.Pose.position[2] = pose->position[2];
    remoteAgentInfo.Pose.quaternion[0] = pose->quaternion[0];
    remoteAgentInfo.Pose.quaternion[1] = pose->quaternion[1];
    remoteAgentInfo.Pose.quaternion[2] = pose->quaternion[2];
    remoteAgentInfo.Pose.quaternion[3] = pose->quaternion[3];
    }
}
//[New]: handle LPP packet --- tdoa4
static void handleLppPacket_tdoa4(const int dataLength, int rangePacketLength, const packet_t* rxPacket) {
    const int32_t payloadLength = dataLength - MAC802154_HEADER_LENGTH;
    const int32_t startOfLppDataInPayload = rangePacketLength;
    const int32_t lppDataLength = payloadLength - startOfLppDataInPayload;
    const int32_t lppTypeInPayload = startOfLppDataInPayload + 1;
    //   printf("payloadLentgh is %d\r\n",(int)payloadLength);
    //   printf("startOfLppDataInPayload is %d\r\n",(int)startOfLppDataInPayload);
    if (lppDataLength > 0) {
        const uint8_t lppPacketHeader = rxPacket->payload[startOfLppDataInPayload];
        if (lppPacketHeader == LPP_HEADER_SHORT_PACKET) {
            const int32_t lppTypeAndPayloadLength = lppDataLength - 1;
            // extract the remote agent info for usage
            handleLppShortPacket_tdoa4(&rxPacket->payload[lppTypeInPayload], lppTypeAndPayloadLength);
      }
    }
}

// [New]: Update the remote agent info, also get the rangeDataLength --> for LPP packet
// [Note]: store the remote agent info. for tdoa (among agents) computation
static int updateRemoteAgentData(const void* payload){
    const rangePacket4_t* packet = (rangePacket4_t*)payload;
    const void* agentDataPtr = &packet->remoteAgentData;
    // loop over all remote agent packe info, should save the remote agent data
    for(uint8_t i = 0; i<packet->header.remoteCount; i++){
        remoteAgentDataFull_t* agentData = (remoteAgentDataFull_t*)agentDataPtr;
        /* comment out unused value*/
        // uint8_t remoteId = anchorData->id;
        // int64_t remoteRxTime = anchorData->rxTimeStamp;
        // uint8_t remoteSeqNr = anchorData->seq & 0x7f;

        /* ----------------- store receive msg time ----------------- */
        // if (isValidTimeStamp(remoteRxTime)) {
        //    tdoaStorageSetRemoteRxTime(anchorCtx, remoteId, remoteRxTime, remoteSeqNr);
        // }
        /* ----------------- --------------------------- ----------------- */
        bool hasDistance = ((agentData->seq & 0x80) != 0);
        if (hasDistance) {
        /* comment out unused value*/
        // int64_t tof = anchorData->distance;

        /* ----------------- store the remote agent info ----------------- */
        // if (isValidTimeStamp(tof)) {
        //     tdoaStorageSetTimeOfFlight(anchorCtx, remoteId, tof);

        //     uint8_t anchorId = tdoaStorageGetId(anchorCtx);
        //     tdoaStats_t* stats = &engineState.stats;
        //     if (anchorId == stats->anchorId && remoteId == stats->remoteAnchorId) {
        //     stats->tof = (uint16_t)tof;
        //     }
        // }
        /* ----------------- --------------------------- ----------------- */
        agentDataPtr += sizeof(remoteAgentDataFull_t);
        } else {
        agentDataPtr += sizeof(remoteAgentDataShort_t);
        }
    }
    return (uint8_t*)agentDataPtr - (uint8_t*)packet;
}

//[note]: get range data from message (TDoA4)
static void handleRangePacket(const uint32_t rxTime, const packet_t* rxPacket, const int dataLength)
{
  //[change] packet code is slightly different 
  //     in CF: locoAddress_t sourceAddress =>  uint64_t sourceAddress
  // in anchor: uint8_t sourceAddress[8]
  // similar to destAddress
  const uint8_t remoteAgentId = rxPacket->sourceAddress;
  ctx.agentRxCount[remoteAgentId]++;
  agentContext_t* agentCtx = getContext(remoteAgentId);
  if (agentCtx) {
    const rangePacket4_t* rangePacket = (rangePacket4_t *)rxPacket->payload;

    uint32_t remoteTx = rangePacket->header.txTimeStamp;
    uint8_t remoteTxSeqNr = rangePacket->header.seq;

    double clockCorrection = calculateClockCorrection(agentCtx, remoteTxSeqNr, remoteTx, rxTime);
    if (updateClockCorrection(agentCtx, clockCorrection)) {
      agentCtx->isDataGoodForTransmission = true;

      uint32_t remoteRx = 0;
      uint8_t remoteRxSeqNr = 0;
      bool dataFound = extractFromPacket(rangePacket, &remoteRx, &remoteRxSeqNr);
      if (dataFound) {
        //[note]: here is the range distance data!
        uint16_t distance = calculateDistance(agentCtx, remoteRxSeqNr, remoteTx, remoteRx, rxTime);
        // TODO krri Remove outliers in distances
        if (distance > MIN_TOF) {
          agentCtx->distance = distance;
          agentCtx->distanceUpdateTime = xTaskGetTickCount();
        //[note]: log time of flight of inter-drone ranging
          float M_PER_TICK = 0.0046917639786157855;
          log_range = (float) distance * M_PER_TICK - (float)ANTENNA_OFFSET;
        }
      }
    } else {
      agentCtx->isDataGoodForTransmission = false;
    }
    // [change]
    rangingOk = agentCtx->isDataGoodForTransmission;
    agentCtx->rxTimeStamp = rxTime;
    agentCtx->seqNr = remoteTxSeqNr;
    agentCtx->txTimeStamp = remoteTx;
    // [New] get transmitted info. Position + quaternion
    int rangeDataLength = updateRemoteAgentData(rangePacket); 
    handleLppPacket_tdoa4(dataLength, rangeDataLength, rxPacket);
  }
}

// Send information 
static int populateTxData(rangePacket4_t *rangePacket)
{
  // rangePacket->header.type already populated
  rangePacket->header.seq = ctx.seqNr;
  rangePacket->header.txTimeStamp = ctx.txTime;

  uint8_t remoteAgentCount = 0;
  uint8_t* agentDataPtr = &rangePacket->remoteAgentData;
  for (uint8_t i = 0; i < ctx.remoteTxIdCount; i++) {
    remoteAgentDataFull_t* agentData = (remoteAgentDataFull_t*) agentDataPtr;

    uint8_t id = ctx.remoteTxId[i];
    agentContext_t* agentCtx = getContext(id);

    if (agentCtx->isDataGoodForTransmission) {
      agentData->id = id;
      agentData->seq = agentCtx->seqNr;
      agentData->rxTimeStamp = agentCtx->rxTimeStamp;

      if (agentCtx->distance > 0) {
        agentData->distance = agentCtx->distance;
        agentDataPtr += sizeof(remoteAgentDataFull_t);
        agentData->seq |= 0x80;
      } else {
        agentDataPtr += sizeof(remoteAgentDataShort_t);
      }

      remoteAgentCount++;
    }
  }
  rangePacket->header.remoteCount = remoteAgentCount;

  return (uint8_t*)agentDataPtr - (uint8_t*)rangePacket;
}
// Set TX data in the radio TX buffer for sendind: sourceAddress, destAddress, LPP.position
static void setTxData(dwDevice_t *dev)
{
  static packet_t txPacket;
  static bool firstEntry = true;
  static int lppLength = 0;

  if (firstEntry) {
    MAC80215_PACKET_INIT(txPacket, MAC802154_TYPE_DATA);
    // [change]: add '&' in front of txPacket 
    memcpy(&txPacket.sourceAddress, base_address, 8);
    txPacket.sourceAddress = ctx.agentId;
    memcpy(&txPacket.destAddress, base_address, 8);
    txPacket.destAddress = 0xff;
    txPacket.payload[0] = PACKET_TYPE_TDOA4;

    firstEntry = false;
  }
    int rangePacketSize = populateTxData((rangePacket4_t *)txPacket.payload);
    // LPP anchor position is currently sent in all packets
    txPacket.payload[rangePacketSize + LPP_HEADER] = SHORT_LPP;
    txPacket.payload[rangePacketSize + LPP_TYPE] = LPP_SHORT_AGENT_INFO;

    struct lppShortAgent_s *pos = (struct lppShortAgent_s*) &txPacket.payload[rangePacketSize + LPP_PAYLOAD];
    // test with dummy positions, quater, imu. For more agents and anchors, 
    // LPP packet size will be limited
    float dummy_pos[3] = {(float)AGENT_ID+(float)0.15, (float)AGENT_ID+(float)0.25, (float)AGENT_ID+(float)0.35};
    float dummy_quater[4] = {(float)AGENT_ID+(float)0.015, (float)AGENT_ID+(float)0.025, (float)AGENT_ID+(float)0.035, (float)AGENT_ID+(float)0.045};
    float dummy_imu[6] = {(float)AGENT_ID+(float)0.115, (float)AGENT_ID+(float)0.225, 
                          (float)AGENT_ID+(float)0.335, (float)AGENT_ID+(float)0.445,
                          (float)AGENT_ID+(float)0.555, (float)AGENT_ID+(float)0.665};
    memcpy(pos->position, dummy_pos, 3 * sizeof(float));
    memcpy(pos->quaternion, dummy_quater, 4 * sizeof(float));
    memcpy(pos->imu, dummy_imu, 6 * sizeof(float) );
    lppLength = 2 + sizeof(struct lppShortAgent_s);

  dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH + rangePacketSize + lppLength);
}

static uint32_t randomizeDelayToNextTx()
{
  const uint32_t interval = 10;

  uint32_t r = rand();
  uint32_t delay = ctx.averageTxDelay + r % interval - interval / 2;

  return delay;
}

// Setup the radio to transmit mode 
// originally named setupTx
static void setRadioInTransmitMode(dwDevice_t *dev)
{
  dwTime_t txTime = findTransmitTimeAsSoonAsPossible(dev);
  ctx.txTime = txTime.low32;
  ctx.seqNr = (ctx.seqNr + 1) & 0x7f;

  setTxData(dev);

  dwNewTransmit(dev);
  dwSetDefaults(dev);
  dwSetTxRxTime(dev, txTime);

  dwStartTransmit(dev);
}

static void setRadioInReceiveMode(dwDevice_t *dev) {
    dwNewReceive(dev);
    dwSetDefaults(dev);
    dwStartReceive(dev);
}

// [Note]: transmit TWR
static uint32_t startNextEvent(dwDevice_t *dev, uint32_t now)
{
  dwIdle(dev);

  if (ctx.nextTxTick < now) {
    uint32_t newDelay = randomizeDelayToNextTx();
    ctx.nextTxTick = now + M2T(newDelay);

    setRadioInTransmitMode(dev);
  } else {
    // set the radio into receive mode
    setRadioInReceiveMode(dev);
    // setupRx(dev);
  }

  return ctx.nextTxTick - now;
}
/* ------------------------------------ End for tdoa4 functions -------------------------------------- */


/* ------------------------------------- Functions for tdoa3 ---------------------------------------- */
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

// ---------------------------access the position in the packet-------------------------------- //
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

//[New]: moved from lpp.c in anchor code
// used for switch back to TDOA4
static void lppHandleShortPacket(uint8_t *data, size_t length)
{
    if (length < 1) return;
    int type  = data[0];

    switch(type) {
        case LPP_SHORT_MODE:
        { // used to switch Agent mode
            struct lppShortMode_s* modeInfo = (struct lppShortMode_s*)&data[1];
            //   DEBUG_PRINT("Switch mode!!!!! \n");
            //   // Set new mode
            //   DEBUG_PRINT("MODE is %d\n",(int)modeInfo->mode);
            //   DEBUG_PRINT("TDoA3 is %d\n",(int)LPP_SHORT_MODE_TDOA3);
            if (modeInfo->mode == LPP_SHORT_MODE_TWR) {
                MODE = lpsMode_TWR;
            } else if (modeInfo->mode == LPP_SHORT_MODE_TDOA2) {
                MODE = lpsMode_TDoA2;
            } else if (modeInfo->mode == LPP_SHORT_MODE_TDOA3) {
                // DEBUG_PRINT("Set mode to be tdoa3!!!!! \n");
                MODE = lpsMode_TDoA3;
            }else if (modeInfo->mode == LPP_SHORT_MODE_TDOA4) {
                MODE = lpsMode_TDoA4;
            }
            break;
        }
    }
}
/* ----------------------------- Callback function for tdoa3 ------------------------------------ */
static void rxcallback_tdoa3(dwDevice_t *dev) {
  tdoaStats_t* stats = &tdoaEngineState.stats;
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
    const uint8_t seqNr = packet->header.seq & 0x7f;;

    tdoaAnchorContext_t anchorCtx;
    uint32_t now_ms = T2M(xTaskGetTickCount());

    tdoaEngineGetAnchorCtxForPacketProcessing(&tdoaEngineState, anchorId, now_ms, &anchorCtx);
    int rangeDataLength = updateRemoteData(&anchorCtx, packet);
    tdoaEngineProcessPacket(&tdoaEngineState, &anchorCtx, txAn_in_cl_An, rxAn_by_T_in_cl_T);
    tdoaStorageSetRxTxData(&anchorCtx, rxAn_by_T_in_cl_T, txAn_in_cl_An, seqNr);
    handleLppPacket(dataLength, rangeDataLength, &rxPacket, &anchorCtx);

    rangingOk = true;
  }
  // [Change] switch mode back to TDOA4
  else if(rxPacket.payload[0] == SHORT_LPP){
        if ((int)rxPacket.destAddress == Agent_Id) {  // the lpp is sent to this Agent. 
        // DEBUG_PRINT("Receive the correct Short LPP packet !!!!!!!!!!!!\n");
        // testing switch time//
        // xStart_s = T2M(xTaskGetTickCount());
        lppHandleShortPacket(&rxPacket.payload[1], dataLength - MAC802154_HEADER_LENGTH - 1);
        }
  }

}
/* ----------------------------- Ends for tdoa3 functions ------------------------------------ */


/* ----------------------------- Main function after receiving a packet ----------------------------- */
// static int idx = 0;    // for testing time
// static int xStart=0;   // xStart used in different if loop, need to be static
// int xStart_s=0;        // testing for switching mode, used in lpsTdoa3Tag.c
static void handleRxPacket(dwDevice_t *dev)
{
    // int xEnd=0; int xDifference=0;

    static packet_t rxPacket;
    dwTime_t rxTime = { .full = 0 };

    dwGetRawReceiveTimestamp(dev, &rxTime);
    dwCorrectTimestamp(dev, &rxTime);

    int dataLength = dwGetDataLength(dev);
    rxPacket.payload[0] = 0;
    dwGetData(dev, (uint8_t*)&rxPacket, dataLength);

    if (dataLength == 0) {
        return;
    }
    // DEBUG_PRINT("Receive radio packet \n");
    switch(rxPacket.payload[0]) {
        case PACKET_TYPE_TDOA3: 
            rxcallback_tdoa3(dev);     // compute tdoa3 
            break;
        case PACKET_TYPE_TDOA4:        //[change]
            // original code
            handleRangePacket(rxTime.low32, &rxPacket, dataLength);   //[note] get range
            break;
        case SHORT_LPP:  //[New]: use SHORT_LPP to change mode
            // [Note] Need the (int) in front of rxPacket.destAddress
            if ((int)rxPacket.destAddress == ctx.agentId) {  // the lpp is sent to this Agent. 
                lppHandleShortPacket(&rxPacket.payload[1], dataLength - MAC802154_HEADER_LENGTH - 1);
            }
            break;
        default:
            // Do nothing
            break;
    }
}
/* ------------------------------------------------- End ------------------------------------------------- */

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
        handleRxPacket(dev);   //[change]
        //   rxcallback(dev);
      break;
    case eventTimeout:
        setRadioInReceiveMode(dev);   // [change]
      break;
    case eventReceiveTimeout:
        setRadioInReceiveMode(dev);   // [change]
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
  tdoaStatsUpdate(&tdoaEngineState.stats, now_ms);
    //[change]
    // --- tdoa4 --- //
    if (now_ms> ctx.nextAgentListUpdate){     
        updateAgentLists();  
        ctx.nextAgentListUpdate = now_ms + AGENT_LIST_UPDATE_INTERVAL;
    }

    uint32_t timeout_ms = startNextEvent(dev,now_ms);
    return timeout_ms;
    // --- end --- //
    // return MAX_TIMEOUT;    // original return time. Does this return matters?
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

// ----------------------------------- Change ---------------------------------------- //
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static void Initialize(dwDevice_t *dev) {
  uint32_t now_ms = T2M(xTaskGetTickCount());
  tdoaEngineInit(&tdoaEngineState, now_ms, sendTdoaToEstimatorCallback, LOCODECK_TS_FREQ, TdoaEngineMatchingAlgorithmRandom);

  #ifdef LPS_2D_POSITION_HEIGHT
  DEBUG_PRINT("2D positioning enabled at %f m height\n", LPS_2D_POSITION_HEIGHT);
  #endif

  dwSetReceiveWaitTimeout(dev, TDOA3_RECEIVE_TIMEOUT);

  dwCommitConfiguration(dev);
  rangingOk = false;

    // ----------------------- initialize Agent info. ----------------------- // 
    // manually set the Agent ID
    ctx.agentId = AGENT_ID;   // Agent ID
    ctx.seqNr = 0;
    ctx.txTime = 0;
    ctx.nextTxTick = 0;
    ctx.averageTxDelay = 1000.0 / AGENT_MAX_TX_FREQ;
    ctx.remoteTxIdCount = 0;
    ctx.nextAgentListUpdate = 0;

    memset(&ctx.agentCtxLookup, ID_WITHOUT_CONTEXT, ID_COUNT);
    for (int i = 0; i < AGENT_STORAGE_COUNT; i++) {
        ctx.agentCtx[i].isUsed = false;
    }
    clearAgentRxCount();
    srand(ctx.agentId);
}
#pragma GCC diagnostic pop
// ------------------------------------ End ------------------------------------ //

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

// [Note]: for debugging to see the tdoa packet rate
LOG_GROUP_START(tdoa3)
LOG_ADD(LOG_UINT16, stRx, &tdoaEngineState.stats.packetsReceived)
LOG_ADD(LOG_UINT16, stEst, &tdoaEngineState.stats.packetsToEstimator)
LOG_ADD(LOG_UINT16, stHit, &tdoaEngineState.stats.contextHitCount)
LOG_ADD(LOG_UINT16, stMiss, &tdoaEngineState.stats.contextMissCount)
// LOG_ADD(LOG_UINT16, stCc, &tdoaEngineState.stats.clockCorrectionCount)
// LOG_ADD(LOG_FLOAT, cc, &tdoaEngineState.stats.clockCorrection)
// LOG_ADD(LOG_UINT16, tof, &tdoaEngineState.stats.tof)
LOG_ADD(LOG_FLOAT, tdoa, &tdoaEngineState.stats.tdoa)
LOG_GROUP_STOP(tdoa3)

LOG_GROUP_START(tdoa4)
LOG_ADD(LOG_FLOAT, inter_range, &log_range)
LOG_GROUP_STOP(tdoa4)

