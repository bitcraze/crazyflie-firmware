#include "flooding_struct.h"
#include "ranging_struct.h"
#include "adhocdeck.h"
#include <string.h>
#include "debug.h"

extern uint16_t floodingCheckTable[FLOODING_CHECK_TABLE_SIZE];

// /* Flooding Check Table Operations */
// void floodingCheckTableInit(Flooding_Check_Table_t *floodingCheckTable, uint16_t address, uint16_t sequence) {
//     memset(floodingCheckTable, 0, sizeof(Flooding_Check_Table_t));
//     floodingCheckTable->srcAddress = address;
//     floodingCheckTable->srcAddress = sequence;
// }

// static set_index_t floodingCheckTableSetMalloc(Flooding_Check_Table_Set_t *floodingCheckTableSet) {
//     if (floodingCheckTableSet->freeQueueEntry == -1) {
//         // When there is no space left, the last element in the queue will be removed and the new 
//         // element will be inserted into the queue header
//         set_index_t candidate = floodingCheckTableSet->fullQueueEntry;
//         while (floodingCheckTableSet->setData[floodingCheckTableSet->setData[candidate].next].next != -1) {
//             candidate = floodingCheckTableSet->setData[candidate].next;
//         }
//         set_index_t tmp = floodingCheckTableSet->fullQueueEntry;
//         floodingCheckTableSet->fullQueueEntry = floodingCheckTableSet->setData[candidate].next;
//         floodingCheckTableSet->setData[floodingCheckTableSet->fullQueueEntry].next = tmp;
//         candidate = floodingCheckTableSet->fullQueueEntry;
//         return candidate;
//     }
//     else {
//         set_index_t candidate = floodingCheckTableSet->freeQueueEntry;
//         floodingCheckTableSet->freeQueueEntry = floodingCheckTableSet->setData[candidate].next;
//         set_index_t tmp = floodingCheckTableSet->fullQueueEntry;
//         floodingCheckTableSet->fullQueueEntry = candidate;
//         floodingCheckTableSet->setData[candidate].next = tmp;
//         return candidate;
//     }
// }

// void floodingCheckTableSetInit(Flooding_Check_Table_Set_t *floodingCheckTableSet) {
//     set_index_t i;
//     for(i = 0; i < FLOODING_CHECK_TABLE_SIZE - 1; i++){
//         floodingCheckTableSet->setData[i].next = i + 1;
//     }
//     floodingCheckTableSet->setData[i].next = -1;
//     floodingCheckTableSet->freeQueueEntry = 0;
//     floodingCheckTableSet->fullQueueEntry = -1;
//     floodingCheckTableSet->size = 0;
// }

// set_index_t floodingCheckTableSetInsert(Flooding_Check_Table_Set_t *floodingCheckTableSet,
//                                         Flooding_Check_Table_t *floodingCheckTable){
//     set_index_t candidate = floodingCheckTableSetMalloc(floodingCheckTableSet);
//     // candidate will never be -1
//     memcpy(&floodingCheckTableSet->setData[candidate].data, floodingCheckTable,
//            sizeof(Flooding_Check_Table_t));
//     floodingCheckTableSet->size++;
//     return candidate;
// }

// set_index_t findInFloodingCheckTableSet(Flooding_Check_Table_Set_t *floodingCheckTableSet,
//                                         uint16_t address) {
//     set_index_t iter = floodingCheckTableSet->fullQueueEntry;
//     while(iter != -1) {
//         if (floodingCheckTableSet->setData[iter].data.srcAddress == address) {
//             break;;
//         }
//         iter = floodingCheckTableSet->setData[iter].next;
//     }
//     return iter;
// }

// void sortFloodingCheckTableSet(Flooding_Check_Table_Set_t *floodingCheckTableSet,
//                                set_index_t objectIndex) {
//     set_index_t cur = floodingCheckTableSet->fullQueueEntry;
//     if (objectIndex == floodingCheckTableSet->fullQueueEntry) {
//         // If the head is the object, return directly
//         return;
//     }
//     while (floodingCheckTableSet->setData[cur].next != -1) {
//         // If the object is detected, the object will be arranged at the top of the queue
//         set_index_t iter = floodingCheckTableSet->setData[cur].next;
//         if (objectIndex == iter) {
//             floodingCheckTableSet->setData[cur].next = floodingCheckTableSet->setData[iter].next;
//             floodingCheckTableSet->setData[iter].next = floodingCheckTableSet->fullQueueEntry;
//             floodingCheckTableSet->fullQueueEntry = iter;
//             return;
//         }
//         cur = floodingCheckTableSet->setData[cur].next;
//     }
// }

/* Flooding Topology Table Operations */
void floodingTopologyTableInit(Flooding_Topology_Table_t *floodingTopologyTable, uint16_t srcAddress, uint16_t dstAddress, int16_t distance) {
    memset(floodingTopologyTable, 0, sizeof(Flooding_Topology_Table_t));
    floodingTopologyTable->srcAddress = srcAddress;
    floodingTopologyTable->dstAddress = dstAddress;
    floodingTopologyTable->distance = distance;
    floodingTopologyTable->expirationTime = xTaskGetTickCount() + M2T(FLOODING_TOPOLOGY_TABLE_HOLD_TIME);
}

void floodingTopologyTableUpdate(Flooding_Topology_Table_t *floodingTopologyTable, int16_t distance) {
    floodingTopologyTable->distance = distance;
    floodingTopologyTable->expirationTime = xTaskGetTickCount() + M2T(FLOODING_TOPOLOGY_TABLE_HOLD_TIME);
}

static set_index_t floodingTopologyTableSetMalloc(Flooding_Topology_Table_set_t *floodingTopologyTableSet) {
    if (floodingTopologyTableSet->freeQueueEntry == -1) {
        DEBUG_PRINT("Flooding Topology Table Set is FULL, malloc failed.\n");
        return -1;
    } else {
        set_index_t candidate = floodingTopologyTableSet->freeQueueEntry;
        floodingTopologyTableSet->freeQueueEntry = floodingTopologyTableSet->setData[candidate].next;
        set_index_t tmp = floodingTopologyTableSet->fullQueueEntry;
        floodingTopologyTableSet->fullQueueEntry = candidate;
        floodingTopologyTableSet->setData[candidate].next = tmp;
        return candidate;
    }
}

void floodingTopologyTableSetInit(Flooding_Topology_Table_set_t *floodingTopologyTableSet) {
    set_index_t i;
    for (i = 0; i < FLOODING_TOPOLOGY_TABLE_SIZE - 1; i++) {
        floodingTopologyTableSet->setData[i].next = i + 1;
    }
    floodingTopologyTableSet->setData[i].next = -1;
    floodingTopologyTableSet->freeQueueEntry = 0;
    floodingTopologyTableSet->fullQueueEntry = -1;
    floodingTopologyTableSet->size = 0;
}

set_index_t floodingTopologyTableSetInsert(Flooding_Topology_Table_set_t *floodingTopologyTableSet,
                                           uint16_t srcAddress, uint16_t dstAddress, int16_t distance) {
    set_index_t candidate = floodingTopologyTableSetMalloc(floodingTopologyTableSet);
    // DEBUG_PRINT("Insert: %d\n", candidate);
    if (candidate != -1) {
        floodingTopologyTableSet->setData[candidate].data.srcAddress = srcAddress;
        floodingTopologyTableSet->setData[candidate].data.dstAddress = dstAddress;
        floodingTopologyTableSet->setData[candidate].data.distance = distance;
        floodingTopologyTableSet->setData[candidate].data.expirationTime = xTaskGetTickCount() + M2T(FLOODING_TOPOLOGY_TABLE_HOLD_TIME);
        floodingTopologyTableSet->size++;
    }
    return candidate;
}

set_index_t findInFloodingTopologyTableSet(Flooding_Topology_Table_set_t *floodingTopologyTableSet,
                                           uint16_t srcAddress, uint16_t dstAddress) {
    set_index_t candidate = floodingTopologyTableSet->fullQueueEntry;
    while (candidate != -1) {
        Flooding_Topology_Table_Set_Item_t item = floodingTopologyTableSet->setData[candidate];
        if ((item.data.srcAddress == srcAddress) && (item.data.dstAddress == dstAddress)) {
            break;
        }
        candidate = item.next;
    }
    return candidate;
}

static void floodingTopologyTableSetFree(Flooding_Topology_Table_set_t *floodingTopologyTableSet,
                                         set_index_t itemIndex, set_index_t preItemIndex) {
    if (itemIndex == -1) return;
    if (itemIndex == preItemIndex) {
        floodingTopologyTableSet->fullQueueEntry = floodingTopologyTableSet->setData[itemIndex].next;
        floodingTopologyTableSet->setData[itemIndex].next = floodingTopologyTableSet->freeQueueEntry;
        floodingTopologyTableSet->freeQueueEntry = itemIndex;
        floodingTopologyTableSet->size = floodingTopologyTableSet->size - 1;
        // DEBUG_PRINT("Expire %u", floodingTopologyTableSet->setData[itemIndex].data.srcAddress);
        return;
    } else {
        floodingTopologyTableSet->setData[preItemIndex].next = floodingTopologyTableSet->setData[itemIndex].next;
        floodingTopologyTableSet->setData[itemIndex].next = floodingTopologyTableSet->freeQueueEntry;
        floodingTopologyTableSet->freeQueueEntry = itemIndex;
        floodingTopologyTableSet->size = floodingTopologyTableSet->size - 1;
        // DEBUG_PRINT("Expire %u", floodingTopologyTableSet->setData[itemIndex].data.srcAddress);
    }
    return;
}

void floodingTopologyTableSetClearExpire(Flooding_Topology_Table_set_t *floodingTopologyTableSet) {
    set_index_t candidate = floodingTopologyTableSet->fullQueueEntry;
    set_index_t preItemIndex = floodingTopologyTableSet->fullQueueEntry;
    Time_t curTime = xTaskGetTickCount();
    while (candidate != -1) {
        // Suspect bug
        Flooding_Topology_Table_Set_Item_t item = floodingTopologyTableSet->setData[candidate];
        if (item.data.expirationTime < curTime) {
            // DEBUG_PRINT("Delete: %d\n", candidate);
            // check table set to 0, since the neighbor is expired
            floodingCheckTable[item.data.srcAddress] = 0;
            set_index_t nextIndex = item.next;
            // Have completed
            floodingTopologyTableSetFree(floodingTopologyTableSet, candidate, preItemIndex);
            if (candidate == preItemIndex) {
                preItemIndex = nextIndex;
            }
            candidate = nextIndex;
            continue;
        }
        preItemIndex = candidate;
        candidate = item.next;
    }
}

void floodingTopologyTableSetUpdate(Flooding_Topology_Table_set_t *floodingTopologyTableSet, 
                                   uint16_t srcAddress, uint16_t dstAddress, int16_t distance) {
    // Flooding_Topology_Table_t table;
    // floodingTopologyTableInit(&table, srcAddress, dstAddress, distance);

    set_index_t index = findInFloodingTopologyTableSet(floodingTopologyTableSet, srcAddress, dstAddress);
    if (index == -1) {
        // DEBUG_PRINT("New topology table insert.\n");
        floodingTopologyTableSetInsert(floodingTopologyTableSet, srcAddress, dstAddress, distance);
    } else {
        // DEBUG_PRINT("Update: %d\n", index);
        floodingTopologyTableUpdate(&floodingTopologyTableSet->setData[index].data, distance);
    }
}

static void printFloodingTopologyTable(Flooding_Topology_Table_t *floodingTopologyTable) {
    DEBUG_PRINT("SRC: %u, ", floodingTopologyTable->srcAddress);
    DEBUG_PRINT("DST: %u, ", floodingTopologyTable->dstAddress);
    DEBUG_PRINT("DIS: %u\n", floodingTopologyTable->distance);
}

void printFloodingTopologyTableSet(Flooding_Topology_Table_set_t *floodingTopologyTableSet) {
    set_index_t index = -1;
    for (index = floodingTopologyTableSet->fullQueueEntry; index != -1; index = floodingTopologyTableSet->setData[index].next) {
        printFloodingTopologyTable(&floodingTopologyTableSet->setData[index].data);
    }
    DEBUG_PRINT("\n");
}
