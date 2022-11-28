#include <stdint.h>
#include <math.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "system.h"

#include "autoconf.h"
#include "debug.h"
#include "log.h"
#include "assert.h"
#include "adhocdeck.h"
#include "ranging_struct.h"
#include "swarm_ranging.h"

static uint16_t MY_UWB_ADDRESS;

static QueueHandle_t rxQueue;
static Ranging_Table_Set_t rangingTableSet;
static UWB_Message_Listener_t listener;
static TaskHandle_t uwbRangingTxTaskHandle = 0;
static TaskHandle_t uwbRangingRxTaskHandle = 0;

static Timestamp_Tuple_t TfBuffer[Tf_BUFFER_POOL_SIZE] = {0};
static int TfBufferIndex = 0;
static int rangingSeqNumber = 1;

static logVarId_t idVelocityX, idVelocityY, idVelocityZ;
static float velocity;

int16_t distanceTowards[RANGING_TABLE_SIZE + 1] = {0};

void rangingRxCallback(void *parameters) {
//  DEBUG_PRINT("rangingRxCallback \n");

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  UWB_Packet_t *packet = (UWB_Packet_t *) parameters;

  dwTime_t rxTime;
  dwt_readrxtimestamp((uint8_t *) &rxTime.raw);
  Ranging_Message_With_Timestamp_t rxMessageWithTimestamp;
  rxMessageWithTimestamp.rxTime = rxTime;
  Ranging_Message_t *rangingMessage = (Ranging_Message_t *) packet->payload;
  rxMessageWithTimestamp.rangingMessage = *rangingMessage;

  xQueueSendFromISR(rxQueue, &rxMessageWithTimestamp, &xHigherPriorityTaskWoken);
}

void rangingTxCallback(void *parameters) {
  dwTime_t txTime;
  dwt_readtxtimestamp((uint8_t *) &txTime.raw);
  TfBufferIndex++;
  TfBufferIndex %= Tf_BUFFER_POOL_SIZE;
  TfBuffer[TfBufferIndex].seqNumber = rangingSeqNumber;
  TfBuffer[TfBufferIndex].timestamp = txTime;
}

int16_t getDistance(uint16_t neighborAddress) {
  assert(neighborAddress < RANGING_TABLE_SIZE);
  return distanceTowards[neighborAddress];
}

static void uwbRangingTxTask(void *parameters) {
  systemWaitStart();
  // TODO check below UART2 related code
//#ifdef CONFIG_DECK_ADHOCDECK_USE_UART2_PINS
//  while (!isUWBStart) {
//    vTaskDelay(500);
//  }
//#endif
  /* velocity log variable id */
  idVelocityX = logGetVarId("stateEstimate", "vx");
  idVelocityY = logGetVarId("stateEstimate", "vy");
  idVelocityZ = logGetVarId("stateEstimate", "vz");

  UWB_Packet_t txPacketCache;
  txPacketCache.header.type = RANGING;
//  txPacketCache.header.mac = ? TODO init mac header
  while (true) {
    int msgLen = generateRangingMessage((Ranging_Message_t *) &txPacketCache.payload);
    txPacketCache.header.length = sizeof (Packet_Header_t) + msgLen;
    uwbSendPacketBlock(&txPacketCache);
    vTaskDelay(TX_PERIOD_IN_MS);
  }
}

static void uwbRangingRxTask(void *parameters) {
  systemWaitStart();
  // TODO check below UART2 related code
//#ifdef CONFIG_DECK_ADHOCDECK_USE_UART2_PINS
//  while (!isUWBStart) {
//    vTaskDelay(500);
//  }
//#endif
//  while (rxQueue == 0) {
//    DEBUG_PRINT("rxQueue for RangingRxTask is not init\n");
//    vTaskDelay(M2T(1000));
//  }
  Ranging_Message_With_Timestamp_t rxPacketCache;

  while (true) {
    if (xQueueReceive(rxQueue, &rxPacketCache, portMAX_DELAY)) {
//      DEBUG_PRINT("uwbRangingRxTask: received ranging message \n");
      processRangingMessage(&rxPacketCache);
    }
  }
}

void rangingInit() {
  MY_UWB_ADDRESS = getUWBAddress();
//  DEBUG_PRINT("MY_UWB_ADDRESS = %d \n", MY_UWB_ADDRESS);
  rxQueue = xQueueCreate(RANGING_RX_QUEUE_SIZE, RANGING_RX_QUEUE_ITEM_SIZE);
  rangingTableSetInit(&rangingTableSet);

  listener.type = RANGING;
  listener.rxQueue = NULL; // handle rxQueue in swarm_ranging.c instead of adhocdeck.c
  listener.rxCb = rangingRxCallback;
  listener.txCb = rangingTxCallback;
  uwbRegisterListener(&listener);

  idVelocityX = logGetVarId("stateEstimate", "vx");
  idVelocityY = logGetVarId("stateEstimate", "vy");
  idVelocityZ = logGetVarId("stateEstimate", "vz");

  xTaskCreate(uwbRangingTxTask, ADHOC_DECK_RANGING_TX_TASK_NAME, 4 * configMINIMAL_STACK_SIZE, NULL,
              ADHOC_DECK_TASK_PRI, &uwbRangingTxTaskHandle); // TODO optimize STACK SIZE
  xTaskCreate(uwbRangingRxTask, ADHOC_DECK_RANGING_RX_TASK_NAME, 4 * configMINIMAL_STACK_SIZE, NULL,
              ADHOC_DECK_TASK_PRI, &uwbRangingRxTaskHandle); // TODO optimize STACK SIZE
}

int16_t computeDistance(Timestamp_Tuple_t Tp, Timestamp_Tuple_t Rp,
                        Timestamp_Tuple_t Tr, Timestamp_Tuple_t Rr,
                        Timestamp_Tuple_t Tf, Timestamp_Tuple_t Rf) {

  int64_t tRound1, tReply1, tRound2, tReply2, diff1, diff2, tprop_ctn;
  tRound1 = (Rr.timestamp.full - Tp.timestamp.full + MAX_TIMESTAMP) % MAX_TIMESTAMP;
  tReply1 = (Tr.timestamp.full - Rp.timestamp.full + MAX_TIMESTAMP) % MAX_TIMESTAMP;
  tRound2 = (Rf.timestamp.full - Tr.timestamp.full + MAX_TIMESTAMP) % MAX_TIMESTAMP;
  tReply2 = (Tf.timestamp.full - Rr.timestamp.full + MAX_TIMESTAMP) % MAX_TIMESTAMP;
  diff1 = tRound1 - tReply1;
  diff2 = tRound2 - tReply2;
  tprop_ctn = (diff1 * tReply2 + diff2 * tReply1 + diff2 * diff1) / (tRound1 + tRound2 + tReply1 + tReply2);
  int16_t distance = (int16_t) tprop_ctn * 0.4691763978616;

  bool isErrorOccurred = false;
  if (distance > 1000 || distance < 0) {
    DEBUG_PRINT("isErrorOccurred\n");
    isErrorOccurred = true;
  }

  if (tRound2 < 0 || tReply2 < 0) {
    DEBUG_PRINT("tRound2 < 0 || tReply2 < 0\n");
    isErrorOccurred = true;
  }

  if (isErrorOccurred) {
    return 0;
  }

  return distance;
}

void processRangingMessage(Ranging_Message_With_Timestamp_t *rangingMessageWithTimestamp) {
  Ranging_Message_t *rangingMessage = &rangingMessageWithTimestamp->rangingMessage;
  uint16_t neighborAddress = rangingMessage->header.srcAddress;
  set_index_t neighborIndex = findInRangingTableSet(&rangingTableSet, neighborAddress);

  /* handle new neighbor */
  if (neighborIndex == -1) {
    if (rangingTableSet.freeQueueEntry == -1) {
      /* ranging table set is full, ignore this ranging message */
      return;
    }
    Ranging_Table_t table;
    rangingTableInit(&table, neighborAddress);
    neighborIndex = rangingTableSetInsert(&rangingTableSet, &table);
  }

  Ranging_Table_t *neighborRangingTable = &rangingTableSet.setData[neighborIndex].data;
  Ranging_Table_Tr_Rr_Buffer_t *neighborTrRrBuffer = &neighborRangingTable->TrRrBuffer;

  /* update Re */
  neighborRangingTable->Re.timestamp = rangingMessageWithTimestamp->rxTime;
  neighborRangingTable->Re.seqNumber = rangingMessage->header.msgSequence;

  /* update Tr and Rr */
  Timestamp_Tuple_t neighborTr = rangingMessage->header.lastTxTimestamp;
  if (neighborTr.timestamp.full && neighborTrRrBuffer->candidates[neighborTrRrBuffer->cur].Rr.timestamp.full
      && neighborTr.seqNumber == neighborTrRrBuffer->candidates[neighborTrRrBuffer->cur].Rr.seqNumber) {
    rangingTableBufferUpdate(&neighborRangingTable->TrRrBuffer,
                             neighborTr,
                             neighborTrRrBuffer->candidates[neighborTrRrBuffer->cur].Rr);
  }

  /* update Rf */
  Timestamp_Tuple_t neighborRf = {.timestamp.full = 0};
  if (rangingMessage->header.filter & (1 << (getUWBAddress() % 16))) {
    /* retrieve body unit */
    uint8_t bodyUnitCount = (rangingMessage->header.msgLength - sizeof(Ranging_Message_Header_t)) / sizeof(Body_Unit_t);
    for (int i = 0; i < bodyUnitCount; i++) {
      if (rangingMessage->bodyUnits[i].address == getUWBAddress()) {
        neighborRf = rangingMessage->bodyUnits[i].timestamp;
        break;
      }
    }
  }

  if (neighborRf.timestamp.full) {
    neighborRangingTable->Rf = neighborRf;
    // TODO it is possible that can not find corresponding Tf
    /* find corresponding Tf in TfBuffer */
    for (int i = 0; i < Tf_BUFFER_POOL_SIZE; i++) {
      if (TfBuffer[i].seqNumber == neighborRf.seqNumber) {
        neighborRangingTable->Tf = TfBuffer[i];
      }
    }

    Ranging_Table_Tr_Rr_Candidate_t Tr_Rr_Candidate = rangingTableBufferGetCandidate(&neighborRangingTable->TrRrBuffer,
                                                                                     neighborRangingTable->Tf);
    /* try to compute distance */
    if (Tr_Rr_Candidate.Tr.timestamp.full && Tr_Rr_Candidate.Rr.timestamp.full &&
        neighborRangingTable->Tp.timestamp.full && neighborRangingTable->Rp.timestamp.full &&
        neighborRangingTable->Tf.timestamp.full && neighborRangingTable->Rf.timestamp.full) {
      int16_t distance = computeDistance(neighborRangingTable->Tp, neighborRangingTable->Rp,
                                         Tr_Rr_Candidate.Tr, Tr_Rr_Candidate.Rr,
                                         neighborRangingTable->Tf, neighborRangingTable->Rf);
      if (distance > 0) {
        neighborRangingTable->distance = distance;
        distanceTowards[neighborRangingTable->neighborAddress] = distance;
        DEBUG_PRINT("distance to %d = %d \n", neighborAddress, distance);
      } else {
        DEBUG_PRINT("distance is not updated since some error occurs");
      }
    }
  }

  /* Tp <- Tf, Rp <- Rf */
  if (neighborRangingTable->Tf.timestamp.full && neighborRangingTable->Rf.timestamp.full) {
    rangingTableShift(neighborRangingTable);
  }

  /* update Rr */
  neighborTrRrBuffer->candidates[neighborTrRrBuffer->cur].Rr = neighborRangingTable->Re;

  /* update expiration time */
  neighborRangingTable->expirationTime = xTaskGetTickCount() + M2T(RANGING_TABLE_HOLD_TIME);

  neighborRangingTable->state = RECEIVED;
}

int generateRangingMessage(Ranging_Message_t *rangingMessage) {
#ifdef ENABLE_BUS_BOARDING_SCHEME
  sortRangingTableSet(&rangingTableSet);
#endif
  int8_t bodyUnitNumber = 0;
  rangingSeqNumber++;
  int curSeqNumber = rangingSeqNumber;
  rangingMessage->header.filter = 0;
  /* generate message body */
  for (set_index_t index = rangingTableSet.fullQueueEntry; index != -1;
       index = rangingTableSet.setData[index].next) {
    Ranging_Table_t *table = &rangingTableSet.setData[index].data;
    if (bodyUnitNumber >= MAX_BODY_UNIT_NUMBER) {
      break;
    }
    if (table->state == RECEIVED) {
      rangingMessage->bodyUnits[bodyUnitNumber].address = table->neighborAddress;
      /* It is possible that Re is not the newest timestamp, because the newest may be in rxQueue
       * waiting to be handled.
       */
      rangingMessage->bodyUnits[bodyUnitNumber].timestamp = table->Re;
      bodyUnitNumber++;
      table->state = TRANSMITTED;
      rangingMessage->header.filter |= 1 << (table->neighborAddress % 16);
    }
  }
  /* generate message header */
  rangingMessage->header.srcAddress = MY_UWB_ADDRESS;
  rangingMessage->header.msgLength = sizeof(Ranging_Message_Header_t) + sizeof(Body_Unit_t) * bodyUnitNumber;
  rangingMessage->header.msgSequence = curSeqNumber;
  rangingMessage->header.lastTxTimestamp = TfBuffer[TfBufferIndex];
  float velocityX = logGetFloat(idVelocityX);
  float velocityY = logGetFloat(idVelocityY);
  float velocityZ = logGetFloat(idVelocityZ);
  velocity = sqrt(pow(velocityX, 2) + pow(velocityY, 2) + pow(velocityZ, 2));
  /* velocity in cm/s */
  rangingMessage->header.velocity = (short) (velocity * 100);
  return rangingMessage->header.msgLength;
}

LOG_GROUP_START(Ranging)
        LOG_ADD(LOG_INT16, distTo1, distanceTowards + 1)
        LOG_ADD(LOG_INT16, distTo2, distanceTowards + 2)
        LOG_ADD(LOG_INT16, distTo3, distanceTowards + 3)
        LOG_ADD(LOG_INT16, distTo4, distanceTowards + 4)
        LOG_ADD(LOG_INT16, distTo5, distanceTowards + 5)
        LOG_ADD(LOG_INT16, distTo6, distanceTowards + 6)
        LOG_ADD(LOG_INT16, distTo7, distanceTowards + 7)
        LOG_ADD(LOG_INT16, distTo8, distanceTowards + 8)
        LOG_ADD(LOG_INT16, distTo8, distanceTowards + 9)
LOG_GROUP_STOP(Ranging)
