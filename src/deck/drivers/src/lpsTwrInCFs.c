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

/* lpsTwrInterCFs.c: Uwb two way ranging inter multiple crazyflies
    reference: https://arxiv.org/abs/2003.05853
    usage example:  crazyflies with address ending by E5, E6, E7, E...
                    set NUM_CFs to the number of crazyflies
                    make LPS_TWRINCFS_ENABLE=1 */


#include <string.h>

#include "lpsTwrInCFs.h"

#include "FreeRTOS.h"
#include "task.h"

#include "log.h"
#include "physicalConstants.h"
#include "configblock.h"

// #include "estimator_kalman.h"

#define ANTENNA_OFFSET 154.6   // In meter
#define basicAddr 0xbccf000000000000
static uint8_t selfID; // selfID = last_number_of_radio_address - 5
static locoAddress_t selfAddress;
static const uint64_t antennaDelay = (ANTENNA_OFFSET*499.2e6*128)/299792458.0; // In radio tick

typedef struct {
  uint16_t distance[NUM_CFs];
} swarmInfo_t;
static swarmInfo_t state;

// Timestamps for ranging
static dwTime_t poll_tx;
static dwTime_t poll_rx;
static dwTime_t answer_tx;
static dwTime_t answer_rx;
static dwTime_t final_tx;
static dwTime_t final_rx;

static packet_t txPacket;
static bool rangingOk;

// Communication logic
static bool current_mode_trans; // is or not in transmitting mode
static uint8_t current_receiveID; // transmitting to which UWB

static bool checkTurn; // check if the receiving UWB turns into transmitting mode
static uint32_t checkTurnTick = 0;

// Median filter for distance ranging (size=3)
typedef struct {
  uint16_t distance_history[3];
  uint8_t index_inserting;
} median_data_t;
static median_data_t median_data[NUM_CFs];

static uint16_t median_filter_3(uint16_t *data)
{
  uint16_t middle;
  if ((data[0] <= data[1]) && (data[0] <= data[2])){
    middle = (data[1] <= data[2]) ? data[1] : data[2];
  }
  else if((data[1] <= data[0]) && (data[1] <= data[2])){
    middle = (data[0] <= data[2]) ? data[0] : data[2];
  }
  else{
    middle = (data[0] <= data[1]) ? data[0] : data[1];
  }
  return middle;
}
#define ABS(a) ((a) > 0 ? (a) : -(a))

static void txcallback(dwDevice_t *dev)
{
  dwTime_t departure;
  dwGetTransmitTimestamp(dev, &departure);
  departure.full += (antennaDelay / 2);

  if(current_mode_trans){
    switch (txPacket.payload[0]) {
      case LPS_TWR_POLL:
        poll_tx = departure;
        break;
      case LPS_TWR_FINAL:
        final_tx = departure;
        break;
      case LPS_TWR_REPORT+1:
        if( (current_receiveID == 0) || (current_receiveID-1 == selfID) ){
          current_mode_trans = false;
          dwIdle(dev);
          dwSetReceiveWaitTimeout(dev, 10000);
          dwNewReceive(dev);
          dwSetDefaults(dev);
          dwStartReceive(dev);
          checkTurn = true;
          checkTurnTick = xTaskGetTickCount();
        }else{
          current_receiveID = current_receiveID - 1;
        }
        break;
    }
  }else{
    switch (txPacket.payload[0]) {
      case LPS_TWR_ANSWER:
        answer_tx = departure;
        break;
      case LPS_TWR_REPORT:
        break;
    }
  }
}


static void rxcallback(dwDevice_t *dev) {
  dwTime_t arival = { .full=0 };
  int dataLength = dwGetDataLength(dev);
  if (dataLength == 0) return;

  packet_t rxPacket;
  memset(&rxPacket, 0, MAC802154_HEADER_LENGTH);
  dwGetData(dev, (uint8_t*)&rxPacket, dataLength);
  if (rxPacket.destAddress != selfAddress) {
    if(current_mode_trans){
      current_mode_trans = false;
      dwIdle(dev);
      dwSetReceiveWaitTimeout(dev, 10000);
    }
    dwNewReceive(dev);
    dwSetDefaults(dev);
    dwStartReceive(dev);
    return;
  }

  txPacket.destAddress = rxPacket.sourceAddress;
  txPacket.sourceAddress = rxPacket.destAddress;

  if(current_mode_trans){
    switch(rxPacket.payload[LPS_TWR_TYPE]) {
      case LPS_TWR_ANSWER:
      {
        txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_FINAL;
        txPacket.payload[LPS_TWR_SEQ] = rxPacket.payload[LPS_TWR_SEQ];
        dwGetReceiveTimestamp(dev, &arival);
        arival.full -= (antennaDelay / 2);
        answer_rx = arival;
        dwNewTransmit(dev);
        dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2);
        dwWaitForResponse(dev, true);
        dwStartTransmit(dev);
        break;
      }
      case LPS_TWR_REPORT:
      {
        lpsTwrInterCFsReportPayload_t *report = (lpsTwrInterCFsReportPayload_t *)(rxPacket.payload+2);
        double tround1, treply1, treply2, tround2, tprop_ctn, tprop;
        memcpy(&poll_rx, &report->pollRx, 5);
        memcpy(&answer_tx, &report->answerTx, 5);
        memcpy(&final_rx, &report->finalRx, 5);
        tround1 = answer_rx.low32 - poll_tx.low32;
        treply1 = answer_tx.low32 - poll_rx.low32;
        tround2 = final_rx.low32 - answer_tx.low32;
        treply2 = final_tx.low32 - answer_rx.low32;
        tprop_ctn = ((tround1*tround2) - (treply1*treply2)) / (tround1 + tround2 + treply1 + treply2);
        tprop = tprop_ctn / LOCODECK_TS_FREQ;
        uint16_t calcDist = (uint16_t)(1000 * (SPEED_OF_LIGHT * tprop + 1));
        if(calcDist!=0){
          uint16_t medianDist = median_filter_3(median_data[current_receiveID].distance_history);
          if (ABS(medianDist-calcDist)>500)
            state.distance[current_receiveID] = medianDist;
          else
            state.distance[current_receiveID] = calcDist;
          median_data[current_receiveID].index_inserting++;
          if(median_data[current_receiveID].index_inserting==3)
            median_data[current_receiveID].index_inserting = 0;
          median_data[current_receiveID].distance_history[median_data[current_receiveID].index_inserting] = calcDist;        
          rangingOk = true;
        }

        lpsTwrInterCFsReportPayload_t *report2 = (lpsTwrInterCFsReportPayload_t *)(txPacket.payload+2);
        txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_REPORT+1;
        txPacket.payload[LPS_TWR_SEQ] = rxPacket.payload[LPS_TWR_SEQ];
        report2->distance = calcDist;
        dwNewTransmit(dev);
        dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2+sizeof(lpsTwrInterCFsReportPayload_t));
        dwWaitForResponse(dev, true);
        dwStartTransmit(dev);  
        break;
      }
    }
  }else{
    switch(rxPacket.payload[LPS_TWR_TYPE]) {
      case LPS_TWR_POLL:
      {
        txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_ANSWER;
        txPacket.payload[LPS_TWR_SEQ] = rxPacket.payload[LPS_TWR_SEQ];
        dwGetReceiveTimestamp(dev, &arival);
        arival.full -= (antennaDelay / 2);
        poll_rx = arival;
        dwNewTransmit(dev);
        dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2);
        dwWaitForResponse(dev, true);
        dwStartTransmit(dev);
        break;
      }
      case LPS_TWR_FINAL:
      {
        lpsTwrInterCFsReportPayload_t *report = (lpsTwrInterCFsReportPayload_t *)(txPacket.payload+2);
        dwGetReceiveTimestamp(dev, &arival);
        arival.full -= (antennaDelay / 2);
        final_rx = arival;
        txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_REPORT;
        txPacket.payload[LPS_TWR_SEQ] = rxPacket.payload[LPS_TWR_SEQ];
        memcpy(&report->pollRx, &poll_rx, 5);
        memcpy(&report->answerTx, &answer_tx, 5);
        memcpy(&report->finalRx, &final_rx, 5);
        dwNewTransmit(dev);
        dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2+sizeof(lpsTwrInterCFsReportPayload_t));
        dwWaitForResponse(dev, true);
        dwStartTransmit(dev);
        break;
      }
      case (LPS_TWR_REPORT+1):
      {
        lpsTwrInterCFsReportPayload_t *report2 = (lpsTwrInterCFsReportPayload_t *)(rxPacket.payload+2);
        uint8_t rangingID = (uint8_t)(rxPacket.sourceAddress & 0xFF);
        if((report2->distance)!=0){
          // received distance has large noise
          uint16_t calcDist = report2->distance;
          uint16_t medianDist = median_filter_3(median_data[rangingID].distance_history);
          if (ABS(medianDist-calcDist)>500)
            state.distance[rangingID] = medianDist;
          else
            state.distance[rangingID] = calcDist;
          median_data[rangingID].index_inserting++;
          if(median_data[rangingID].index_inserting==3)
            median_data[rangingID].index_inserting = 0;
          median_data[rangingID].distance_history[median_data[rangingID].index_inserting] = calcDist;
        }
        rangingOk = true;
        uint8_t fromID = (uint8_t)(rxPacket.sourceAddress & 0xFF);
        if( selfID == fromID + 1 || selfID == 0 ){
          current_mode_trans = true;
          dwIdle(dev);
          dwSetReceiveWaitTimeout(dev, 1000);
          if(selfID == NUM_CFs-1)
            current_receiveID = 0;
          else
            current_receiveID = NUM_CFs - 1;
          if(selfID == 0)
            current_receiveID = NUM_CFs - 2; // immediate problem
          txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_POLL;
          txPacket.payload[LPS_TWR_SEQ] = 0;
          txPacket.sourceAddress = selfAddress;
          txPacket.destAddress = basicAddr + current_receiveID;
          dwNewTransmit(dev);
          dwSetDefaults(dev);
          dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2);
          dwWaitForResponse(dev, true);
          dwStartTransmit(dev);
        }else{
          dwNewReceive(dev);
          dwSetDefaults(dev);
          dwStartReceive(dev);
        }
        break;
      }
    }
  }
}

static uint32_t twrTagOnEvent(dwDevice_t *dev, uwbEvent_t event)
{
  switch(event) {
    case eventPacketReceived:
      rxcallback(dev);
      checkTurn = false;
      break;
    case eventPacketSent:
      txcallback(dev);
      break;
    case eventTimeout:  // Comes back to timeout after each ranging attempt
    case eventReceiveTimeout:
    case eventReceiveFailed:
      if (current_mode_trans==true)
      {
        txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_POLL;
        txPacket.payload[LPS_TWR_SEQ] = 0;
        txPacket.sourceAddress = selfAddress;
        txPacket.destAddress = basicAddr + current_receiveID;
        dwNewTransmit(dev);
        dwSetDefaults(dev);
        dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2);
        dwWaitForResponse(dev, true);
        dwStartTransmit(dev);
      }else
      {
        if(xTaskGetTickCount() > checkTurnTick + 20) // > 20ms
        {
          if(checkTurn == true){
            current_mode_trans = true;
            dwIdle(dev);
            dwSetReceiveWaitTimeout(dev, 1000);
            txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_POLL;
            txPacket.payload[LPS_TWR_SEQ] = 0;
            txPacket.sourceAddress = selfAddress;
            txPacket.destAddress = basicAddr + current_receiveID;
            dwNewTransmit(dev);
            dwSetDefaults(dev);
            dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2);
            dwWaitForResponse(dev, true);
            dwStartTransmit(dev);
            checkTurn = false;
            break;
          }
        }
        dwNewReceive(dev);
	      dwSetDefaults(dev);
        dwStartReceive(dev);
      }     
      break;
    default:
      configASSERT(false);
  }
  return MAX_TIMEOUT;
}

static void twrTagInit(dwDevice_t *dev)
{
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

  selfID = (uint8_t)(((configblockGetRadioAddress()) & 0x000000000f) - 5);
  selfAddress = basicAddr + selfID;

  // Communication logic between each UWB
  if(selfID==0)
  {
    current_receiveID = NUM_CFs-1;
    current_mode_trans = true;
    dwSetReceiveWaitTimeout(dev, 1000);
  }
  else
  {
    // current_receiveID = 0;
    current_mode_trans = false;
    dwSetReceiveWaitTimeout(dev, 10000);
  }

  for (int i = 0; i < NUM_CFs; i++) {
    median_data[i].index_inserting = 0;
  }

  checkTurn = false;
  rangingOk = false;
}

static bool isRangingOk()
{
  return rangingOk;
}

static bool getAnchorPosition(const uint8_t anchorId, point_t* position) {
  if (anchorId < NUM_CFs-1)
    return true;
  else
    return false;
}

static uint8_t getAnchorIdList(uint8_t unorderedAnchorList[], const int maxListSize) {
  for (int i = 0; i < NUM_CFs-1; i++) {
    unorderedAnchorList[i] = i;
  }
  return NUM_CFs-1;
}

static uint8_t getActiveAnchorIdList(uint8_t unorderedAnchorList[], const int maxListSize) {
  uint8_t count = 0;
  for (int i = 0; i < NUM_CFs-1; i++) {
      unorderedAnchorList[count] = i;
      count++;
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

LOG_GROUP_START(ranging)
LOG_ADD(LOG_UINT16, distance0, &state.distance[0])
LOG_ADD(LOG_UINT16, distance1, &state.distance[1])
LOG_ADD(LOG_UINT16, distance2, &state.distance[2])
LOG_ADD(LOG_UINT16, distance3, &state.distance[3])
LOG_ADD(LOG_UINT16, distance4, &state.distance[4])
LOG_GROUP_STOP(ranging)