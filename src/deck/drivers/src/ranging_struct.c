#include "ranging_struct.h"

#include <stdio.h>
#include <string.h>
#include "task.h"

//TODO add semaphore to protect ranging table structure.
static set_index_t ranging_table_set_malloc(
    Ranging_Table_Set_t *rangingTableSet) {
  if (rangingTableSet->freeQueueEntry == -1) {
    printf("Ranging Table Set is FULL, malloc failed.\r\n");
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

static bool ranging_table_set_free(Ranging_Table_Set_t *rangingTableSet,
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
  set_index_t candidate = ranging_table_set_malloc(rangingTableSet);
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

bool deleteRangingTupleByIndex(Ranging_Table_Set_t *rangingTableSet,
                                   set_index_t index) {
  return ranging_table_set_free(rangingTableSet, index);
}

void printRangingTableTuple(Ranging_Table_t *table) {
  // printf("Rp = %2x%8lx, Tr = %2x%8lx, Rf = %2x%8lx, \r\n",
  //        table->Rp.timestamp.high8, table->Rp.timestamp.low32,
  //        table->Tr.timestamp.high8, table->Tr.timestamp.low32,
  //        table->Rf.timestamp.high8, table->Rf.timestamp.low32);
  // printf("Tp = %2x%8lx, Rr = %2x%8lx, Tf = %2x%8lx, Re = %2x%8lx, \r\n",
  //        table->Tp.timestamp.high8, table->Tp.timestamp.low32,
  //        table->Rr.timestamp.high8, table->Rr.timestamp.low32,
  //        table->Tf.timestamp.high8, table->Tf.timestamp.low32,
  //        table->Re.timestamp.high8, table->Re.timestamp.low32);
  // printf("====\r\n");
  // printf("Rp = %llu, Tr = %llu, Rf = %llu, \r\n",
  //        table->Rp.timestamp.full,
  //        table->Tr.timestamp.full, 
  //        table->Rf.timestamp.full);
  // printf("Tp = %llu, Rr = %llu, Tf = %llu, Re = %llu, \r\n",
  //        table->Tp.timestamp.full,
  //        table->Rr.timestamp.full,
  //        table->Tf.timestamp.full,
  //        table->Re.timestamp.full);
  // printf("====\r\n");
  printf("Rp = %u, Tr = %u, Rf = %u, \r\n",
         table->Rp.seqNumber,
         table->Tr.seqNumber, 
         table->Rf.seqNumber);
  printf("Tp = %u, Rr = %u, Tf = %u, Re = %u, \r\n",
         table->Tp.seqNumber,
         table->Rr.seqNumber,
         table->Tf.seqNumber,
         table->Re.seqNumber);
  printf("====\r\n");
}

void printRangingTable(Ranging_Table_Set_t *rangingTableSet) {
  for (set_index_t index = rangingTableSet->fullQueueEntry; index != -1;
       index = rangingTableSet->setData[index].next) {
    printRangingTableTuple(&rangingTableSet->setData[index].data);
  }
}

bool rangingTableClearExpire(Ranging_Table_Set_t *rangingTableSet) {
  set_index_t candidate = rangingTableSet->fullQueueEntry;
  Time_t now = xTaskGetTickCount();
  bool has_changed = false;
  while (candidate != -1) {
    Ranging_Table_Set_Item_t temp = rangingTableSet->setData[candidate];
    if (temp.data.expirationTime < now) {
      set_index_t next_index = temp.next;
      ranging_table_set_free(rangingTableSet, candidate);
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
  printf("msgLength=%u, msgSequence=%d, srcAddress=%u, velocity=%d\r\n, last_tx_timestamp_seq=%u, lastTxTimestamp=%2x%8lx\r\n",
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
    printf("===printRangingMessage: wrong body unit number occurs===\r\n");
    return;
  }
  for (int i = 0; i < body_unit_number; i++) {
    printf("body_unit_address=%u, body_unit_seq=%u\r\n",
           rangingMessage->bodyUnits[i].address,
           rangingMessage->bodyUnits[i].timestamp.seqNumber);
    printf("body_unit_timestamp=%2x%8lx\r\n",
           rangingMessage->bodyUnits[i].timestamp.timestamp.high8,
           rangingMessage->bodyUnits[i].timestamp.timestamp.low32);
  }
}