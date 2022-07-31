#include "ranging_struct.h"
#include "adhocdeck.h"
#include <stdio.h>
#include <string.h>
#include "task.h"
#include "debug.h"

void rangingTableBufferInit(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer) {
  rangingTableBuffer->index = 0;
  Timestamp_Tuple_t empty = {.seqNumber = 0, .timestamp.full = 0};
  for (set_index_t i = 0; i < Tr_Rr_BUFFER_SIZE; i++) {
    rangingTableBuffer->candidates[i].Tr = empty;
    rangingTableBuffer->candidates[i].Rr = empty;
    rangingTableBuffer->candidates[i].Tf_SeqNumber = 0;
  }
}

void rangingTableBufferUpdateTimestamp(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer, Timestamp_Tuple_t Tr, Timestamp_Tuple_t Rr) {
  rangingTableBuffer->candidates[rangingTableBuffer->index].Tr = Tr;
  rangingTableBuffer->candidates[rangingTableBuffer->index].Rr = Rr;
}

void rangingTableBufferUpdateTimestampPredecessors(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer, Timestamp_Tuple_t Tr, Timestamp_Tuple_t Rr) {
  set_index_t index = rangingTableBuffer->index;
  for (int count = 0; count < Tr_Rr_BUFFER_SIZE; count++) {
    if (Tr.seqNumber == rangingTableBuffer->candidates[index].Tr.seqNumber + 1) {
      rangingTableBuffer->candidates[index].Tr = Tr;
      rangingTableBuffer->candidates[index].Rr = Rr;
    }
    if (index == 0) {
      index = Tr_Rr_BUFFER_SIZE - 1;
    } else {
      index--;
    }
  }
}

void rangingTableBufferUpdateSeqNumber(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer, uint16_t Tf_SeqNumber) {
  rangingTableBuffer->candidates[rangingTableBuffer->index].Tf_SeqNumber = Tf_SeqNumber;
}

void rangingTableBufferUpdate(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer, Timestamp_Tuple_t Tr, Timestamp_Tuple_t Rr, uint16_t Tf_SeqNumber) {
  rangingTableBufferUpdateSeqNumber(rangingTableBuffer, Tf_SeqNumber);
  rangingTableBufferUpdateTimestamp(rangingTableBuffer, Tr, Rr);
}

void rangingTableBufferShift(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer) {
  set_index_t pre = rangingTableBuffer->index;
  rangingTableBuffer->index = (rangingTableBuffer->index + 1) % Tr_Rr_BUFFER_SIZE;
  set_index_t cur = rangingTableBuffer->index;
  rangingTableBuffer->candidates[cur].Tr = rangingTableBuffer->candidates[pre].Tr;
  rangingTableBuffer->candidates[cur].Rr = rangingTableBuffer->candidates[pre].Rr;
  rangingTableBuffer->candidates[cur].Tf_SeqNumber = 0;
}

Ranging_Table_Tr_Rr_Candidate_t rangingTableBufferGetCandidate(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer, Timestamp_Tuple_t Tf) {
  set_index_t index = rangingTableBuffer->index;
  Ranging_Table_Tr_Rr_Candidate_t candidate = {.Rr.timestamp.full = 0, .Tr.timestamp.full = 0, .Tf_SeqNumber = 0};
  uint64_t rightBound = Tf.timestamp.full % MAX_TIMESTAMP;
  for (int count = 0; count < Tr_Rr_BUFFER_SIZE; count++) {
    if (rangingTableBuffer->candidates[index].Rr.timestamp.full % MAX_TIMESTAMP < rightBound) {
      candidate.Tr = rangingTableBuffer->candidates[index].Tr;
      candidate.Rr = rangingTableBuffer->candidates[index].Rr;
      candidate.Tf_SeqNumber = rangingTableBuffer->candidates[index].Tf_SeqNumber;
      break;
    }

    if (index == 0) {
      index = Tr_Rr_BUFFER_SIZE - 1;
    } else {
      index--;
    }
  }
  return candidate;
}

Ranging_Table_Tr_Rr_Candidate_t rangingTableBufferGetLatestCandidate(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer) {
  return rangingTableBuffer->candidates[rangingTableBuffer->index];
}

void rangingTableInit(Ranging_Table_t *rangingTable, address_t address) {
  memset(rangingTable, 0, sizeof(Ranging_Table_t));
  rangingTable->neighborAddress = address;
  rangingTable->period = TX_PERIOD_IN_MS;
  rangingTable->nextDeliveryTime = xTaskGetTickCount() + rangingTable->period;
  rangingTable->expirationTime = xTaskGetTickCount() + M2T(RANGING_TABLE_HOLD_TIME);
  rangingTable->state = RECEIVED;
  rangingTableBufferInit(&rangingTable->TrRrBuffer);
}

void rangingTableShift(Ranging_Table_t *rangingTable) {
  rangingTable->Rp = rangingTable->Rf;
  rangingTable->Tp = rangingTable->Tf;
  rangingTable->Rr = rangingTable->Re;

  rangingTable->Rf.timestamp.full = 0;
  rangingTable->Rf.seqNumber = 0;
  rangingTable->Tf.timestamp.full = 0;
  rangingTable->Tf.seqNumber = 0;
  rangingTable->Tr.timestamp.full = 0;
  rangingTable->Tr.seqNumber = 0;
}

//TODO add semaphore to protect ranging table structure.
static set_index_t rangingTableSetMalloc(
    Ranging_Table_Set_t *rangingTableSet) {
  if (rangingTableSet->freeQueueEntry == -1) {
    DEBUG_PRINT("Ranging Table Set is FULL, malloc failed.\n");
    return -1;
  } else {
    set_index_t candidate = rangingTableSet->freeQueueEntry;
    rangingTableSet->freeQueueEntry =
        rangingTableSet->setData[candidate].next;
    // insert to full queue
    set_index_t temp = rangingTableSet->fullQueueEntry;
    rangingTableSet->fullQueueEntry = candidate;
    rangingTableSet->setData[candidate].next = temp;
    return candidate;
  }
}

static bool rangingTableSetFree(Ranging_Table_Set_t *rangingTableSet,
                                   set_index_t item_index) {
  if (-1 == item_index) {
    return true;
  }
  // delete from full queue
  set_index_t pre = rangingTableSet->fullQueueEntry;
  if (item_index == pre) {
    rangingTableSet->fullQueueEntry = rangingTableSet->setData[pre].next;
    // insert into empty queue
    rangingTableSet->setData[item_index].next =
        rangingTableSet->freeQueueEntry;
    rangingTableSet->freeQueueEntry = item_index;
    rangingTableSet->size = rangingTableSet->size - 1;
    return true;
  } else {
    while (pre != -1) {
      if (rangingTableSet->setData[pre].next == item_index) {
        rangingTableSet->setData[pre].next =
            rangingTableSet->setData[item_index].next;
        // insert into empty queue
        rangingTableSet->setData[item_index].next =
            rangingTableSet->freeQueueEntry;
        rangingTableSet->freeQueueEntry = item_index;
        rangingTableSet->size = rangingTableSet->size - 1;
        return true;
      }
      pre = rangingTableSet->setData[pre].next;
    }
  }
  return false;
}

void rangingTableSetInit(Ranging_Table_Set_t *rangingTableSet) {
  set_index_t i;
  for (i = 0; i < RANGING_TABLE_SIZE - 1; i++) {
    rangingTableSet->setData[i].next = i + 1;
  }
  rangingTableSet->setData[i].next = -1;
  rangingTableSet->freeQueueEntry = 0;
  rangingTableSet->fullQueueEntry = -1;
  rangingTableSet->size = 0;
}

set_index_t rangingTableSetInsert(Ranging_Table_Set_t *rangingTableSet,
                                     Ranging_Table_t *table) {
  set_index_t candidate = rangingTableSetMalloc(rangingTableSet);
  if (candidate != -1) {
    memcpy(&rangingTableSet->setData[candidate].data, table,
           sizeof(Ranging_Table_t));
    rangingTableSet->size++;
  }
  return candidate;
}

set_index_t findInRangingTableSet(Ranging_Table_Set_t *rangingTableSet,
                                      address_t addr) {
  set_index_t iter = rangingTableSet->fullQueueEntry;
  while (iter != -1) {
    Ranging_Table_Set_Item_t cur = rangingTableSet->setData[iter];
    if (cur.data.neighborAddress == addr) {
      break;
    }
    iter = cur.next;
  }
  return iter;
}

bool deleteRangingTableByIndex(Ranging_Table_Set_t *rangingTableSet,
                                   set_index_t index) {
  return rangingTableSetFree(rangingTableSet, index);
}

void printRangingTable(Ranging_Table_t *table) {
  // DEBUG_PRINT("Rp = %2x%8lx, Tr = %2x%8lx, Rf = %2x%8lx, \n",
  //        table->Rp.timestamp.high8, table->Rp.timestamp.low32,
  //        table->Tr.timestamp.high8, table->Tr.timestamp.low32,
  //        table->Rf.timestamp.high8, table->Rf.timestamp.low32);
  // DEBUG_PRINT("Tp = %2x%8lx, Rr = %2x%8lx, Tf = %2x%8lx, Re = %2x%8lx, \n",
  //        table->Tp.timestamp.high8, table->Tp.timestamp.low32,
  //        table->Rr.timestamp.high8, table->Rr.timestamp.low32,
  //        table->Tf.timestamp.high8, table->Tf.timestamp.low32,
  //        table->Re.timestamp.high8, table->Re.timestamp.low32);
  // DEBUG_PRINT("====\n");
  // DEBUG_PRINT("Rp = %llu, Tr = %llu, Rf = %llu, \n",
  //        table->Rp.timestamp.full,
  //        table->Tr.timestamp.full, 
  //        table->Rf.timestamp.full);
  // DEBUG_PRINT("Tp = %llu, Rr = %llu, Tf = %llu, Re = %llu, \n",
  //        table->Tp.timestamp.full,
  //        table->Rr.timestamp.full,
  //        table->Tf.timestamp.full,
  //        table->Re.timestamp.full);
  // DEBUG_PRINT("====\n");
  DEBUG_PRINT("Rp = %u, Tr = %u, Rf = %u, \n",
         table->Rp.seqNumber,
         table->Tr.seqNumber, 
         table->Rf.seqNumber);
  DEBUG_PRINT("Tp = %u, Rr = %u, Tf = %u, Re = %u, \n",
         table->Tp.seqNumber,
         table->Rr.seqNumber,
         table->Tf.seqNumber,
         table->Re.seqNumber);
  DEBUG_PRINT("====\n");
}

void printRangingTableSet(Ranging_Table_Set_t *rangingTableSet) {
  for (set_index_t index = rangingTableSet->fullQueueEntry; index != -1;
       index = rangingTableSet->setData[index].next) {
    printRangingTable(&rangingTableSet->setData[index].data);
  }
}

bool rangingTableSetClearExpire(Ranging_Table_Set_t *rangingTableSet) {
  set_index_t candidate = rangingTableSet->fullQueueEntry;
  Time_t now = xTaskGetTickCount();
  bool has_changed = false;
  while (candidate != -1) {
    Ranging_Table_Set_Item_t temp = rangingTableSet->setData[candidate];
    if (temp.data.expirationTime < now) {
      set_index_t next_index = temp.next;
      rangingTableSetFree(rangingTableSet, candidate);
      candidate = next_index;
      has_changed = true;
      continue;
    }
    candidate = temp.next;
  }
  return has_changed;
}

void sortRangingTableSet(Ranging_Table_Set_t *rangingTableSet) {
  if (rangingTableSet->fullQueueEntry == -1) {
    return;
  }
  set_index_t new_head = rangingTableSet->fullQueueEntry;
  set_index_t cur = rangingTableSet->setData[new_head].next;
  rangingTableSet->setData[new_head].next = -1;
  set_index_t next = -1;
  while (cur != -1) {
    next = rangingTableSet->setData[cur].next;
    if (rangingTableSet->setData[cur].data.nextDeliveryTime <=
        rangingTableSet->setData[new_head].data.nextDeliveryTime) {
      rangingTableSet->setData[cur].next = new_head;
      new_head = cur;
    } else {
      set_index_t start = rangingTableSet->setData[new_head].next;
      set_index_t pre = new_head;
      while (start != -1 &&
             rangingTableSet->setData[cur].data.nextDeliveryTime >
                 rangingTableSet->setData[start].data.nextDeliveryTime) {
        pre = start;
        start = rangingTableSet->setData[start].next;
      }
      rangingTableSet->setData[cur].next = start;
      rangingTableSet->setData[pre].next = cur;
    }
    cur = next;
  }
  rangingTableSet->fullQueueEntry = new_head;
}

void printRangingMessage(Ranging_Message_t *rangingMessage) {
  DEBUG_PRINT("msgLength=%u, msgSequence=%d, srcAddress=%u, velocity=%d\n, last_tx_timestamp_seq=%u, lastTxTimestamp=%2x%8lx\n",
      rangingMessage->header.msgLength,
      rangingMessage->header.msgSequence,
      rangingMessage->header.srcAddress, rangingMessage->header.velocity,
      rangingMessage->header.lastTxTimestamp.seqNumber,
      rangingMessage->header.lastTxTimestamp.timestamp.high8,
      rangingMessage->header.lastTxTimestamp.timestamp.low32);

  if (rangingMessage->header.msgLength - sizeof(Ranging_Message_Header_t) == 0) {
    return;
  }
  int body_unit_number = (rangingMessage->header.msgLength - sizeof(Ranging_Message_Header_t)) / sizeof(Body_Unit_t);
  if (body_unit_number >= MAX_BODY_UNIT_NUMBER) {
    DEBUG_PRINT("===printRangingMessage: wrong body unit number occurs===\n");
    return;
  }
  for (int i = 0; i < body_unit_number; i++) {
    DEBUG_PRINT("body_unit_address=%u, body_unit_seq=%u\n",
           rangingMessage->bodyUnits[i].address,
           rangingMessage->bodyUnits[i].timestamp.seqNumber);
    DEBUG_PRINT("body_unit_timestamp=%2x%8lx\n",
           rangingMessage->bodyUnits[i].timestamp.timestamp.high8,
           rangingMessage->bodyUnits[i].timestamp.timestamp.low32);
  }
}