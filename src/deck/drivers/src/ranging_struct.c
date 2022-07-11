#include "ranging_struct.h"

#include <stdio.h>

#include "task.h"

static set_index_t ranging_table_set_malloc(
    Ranging_Table_Set_t *ranging_table_set) {
  if (ranging_table_set->free_queue_entry == -1) {
    printf("Ranging Table Set is FULL, malloc failed.\r\n");
    return -1;
  } else {
    set_index_t candidate = ranging_table_set->free_queue_entry;
    ranging_table_set->free_queue_entry =
        ranging_table_set->set_data[candidate].next;
    // insert to full queue
    set_index_t temp = ranging_table_set->full_queue_entry;
    ranging_table_set->full_queue_entry = candidate;
    ranging_table_set->set_data[candidate].next = temp;
    return candidate;
  }
}

static bool ranging_table_set_free(Ranging_Table_Set_t *ranging_table_set,
                                   set_index_t item_index) {
  if (-1 == item_index) {
    return true;
  }
  // delete from full queue
  set_index_t pre = ranging_table_set->full_queue_entry;
  if (item_index == pre) {
    ranging_table_set->full_queue_entry = ranging_table_set->set_data[pre].next;
    // insert into empty queue
    ranging_table_set->set_data[item_index].next =
        ranging_table_set->free_queue_entry;
    ranging_table_set->free_queue_entry = item_index;
    ranging_table_set->size = ranging_table_set->size - 1;
    return true;
  } else {
    while (pre != -1) {
      if (ranging_table_set->set_data[pre].next == item_index) {
        ranging_table_set->set_data[pre].next =
            ranging_table_set->set_data[item_index].next;
        // insert into empty queue
        ranging_table_set->set_data[item_index].next =
            ranging_table_set->free_queue_entry;
        ranging_table_set->free_queue_entry = item_index;
        ranging_table_set->size = ranging_table_set->size - 1;
        return true;
      }
      pre = ranging_table_set->set_data[pre].next;
    }
  }
  return false;
}

void ranging_table_set_init(Ranging_Table_Set_t *ranging_table_set) {
  set_index_t i;
  for (i = 0; i < RANGING_TABLE_SIZE - 1; i++) {
    ranging_table_set->set_data[i].next = i + 1;
  }
  ranging_table_set->set_data[i].next = -1;
  ranging_table_set->free_queue_entry = 0;
  ranging_table_set->full_queue_entry = -1;
  ranging_table_set->size = 0;
}

set_index_t ranging_table_set_insert(Ranging_Table_Set_t *ranging_table_set,
                                     Ranging_Table_t *table) {
  set_index_t candidate = ranging_table_set_malloc(ranging_table_set);
  if (candidate != -1) {
    memcpy(&ranging_table_set->set_data[candidate].data, table,
           sizeof(Ranging_Table_t));
    ranging_table_set->size++;
  }
  return candidate;
}

set_index_t find_in_ranging_table_set(Ranging_Table_Set_t *ranging_table_set,
                                      address_t addr) {
  set_index_t iter = ranging_table_set->full_queue_entry;
  while (iter != -1) {
    Ranging_Table_Set_Item_t cur = ranging_table_set->set_data[iter];
    if (cur.data.neighbor_address == addr) {
      break;
    }
    iter = cur.next;
  }
  return iter;
}

bool delete_ranging_tuple_by_index(Ranging_Table_Set_t *ranging_table_set,
                                   set_index_t index) {
  return ranging_table_set_free(ranging_table_set, index);
}

void print_ranging_table_tuple(Ranging_Table_t *table) {
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
         table->Rp.sequence_number,
         table->Tr.sequence_number, 
         table->Rf.sequence_number);
  printf("Tp = %u, Rr = %u, Tf = %u, Re = %u, \r\n",
         table->Tp.sequence_number,
         table->Rr.sequence_number,
         table->Tf.sequence_number,
         table->Re.sequence_number);
  printf("====\r\n");
}

void print_ranging_table(Ranging_Table_Set_t *ranging_table_set) {
  for (set_index_t index = ranging_table_set->full_queue_entry; index != -1;
       index = ranging_table_set->set_data[index].next) {
    print_ranging_table_tuple(&ranging_table_set->set_data[index].data);
  }
}

bool ranging_table_clear_expire(Ranging_Table_Set_t *ranging_table_set) {
  set_index_t candidate = ranging_table_set->full_queue_entry;
  Time_t now = xTaskGetTickCount();
  bool has_changed = false;
  while (candidate != -1) {
    Ranging_Table_Set_Item_t temp = ranging_table_set->set_data[candidate];
    if (temp.data.expiration_time < now) {
      set_index_t next_index = temp.next;
      ranging_table_set_free(ranging_table_set, candidate);
      candidate = next_index;
      has_changed = true;
      continue;
    }
    candidate = temp.next;
  }
  return has_changed;
}

void sort_ranging_table_set(Ranging_Table_Set_t *ranging_table_set) {
  if (ranging_table_set->full_queue_entry == -1) {
    return;
  }
  set_index_t new_head = ranging_table_set->full_queue_entry;
  set_index_t cur = ranging_table_set->set_data[new_head].next;
  ranging_table_set->set_data[new_head].next = -1;
  set_index_t next = -1;
  while (cur != -1) {
    next = ranging_table_set->set_data[cur].next;
    if (ranging_table_set->set_data[cur].data.next_delivery_time <=
        ranging_table_set->set_data[new_head].data.next_delivery_time) {
      ranging_table_set->set_data[cur].next = new_head;
      new_head = cur;
    } else {
      set_index_t start = ranging_table_set->set_data[new_head].next;
      set_index_t pre = new_head;
      while (start != -1 &&
             ranging_table_set->set_data[cur].data.next_delivery_time >
                 ranging_table_set->set_data[start].data.next_delivery_time) {
        pre = start;
        start = ranging_table_set->set_data[start].next;
      }
      ranging_table_set->set_data[cur].next = start;
      ranging_table_set->set_data[pre].next = cur;
    }
    cur = next;
  }
  ranging_table_set->full_queue_entry = new_head;
}

void print_ranging_message(Ranging_Message_t *ranging_message) {
  printf("message_length=%u, message_sequence=%d, source_address=%u, velocity=%d\r\n, last_tx_timestamp_seq=%u, last_tx_timestamp=%2x%8lx\r\n",
      ranging_message->header.message_length,
      ranging_message->header.message_sequence,
      ranging_message->header.source_address, ranging_message->header.velocity,
      ranging_message->header.last_tx_timestamp.sequence_number,
      ranging_message->header.last_tx_timestamp.timestamp.high8,
      ranging_message->header.last_tx_timestamp.timestamp.low32);

  if (ranging_message->header.message_length - sizeof(Ranging_Message_Header_t) == 0) {
    return;
  }
  int body_unit_number = (ranging_message->header.message_length - sizeof(Ranging_Message_Header_t)) / sizeof(Body_Unit_t);
  if (body_unit_number >= MAX_BODY_UNIT_NUMBER) {
    printf("===print_ranging_message: wrong body unit number occurs===\r\n");
    return;
  }
  for (int i = 0; i < body_unit_number; i++) {
    printf("body_unit_address=%u, body_unit_seq=%u\r\n",
           ranging_message->body_units[i].address,
           ranging_message->body_units[i].timestamp.sequence_number);
    printf("body_unit_timestamp=%2x%8lx\r\n",
           ranging_message->body_units[i].timestamp.timestamp.high8,
           ranging_message->body_units[i].timestamp.timestamp.low32);
  }
}