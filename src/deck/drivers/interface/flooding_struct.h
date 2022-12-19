#ifndef __FLOODING_STRUCT_H__
#define __FLOODING_STRUCT_H__

#include "FreeRTOS.h"
#include "dwTypes.h"
#include "adhocdeck.h"
#include "ranging_struct.h"

#define MAX_FLOODING_BODY_UNIT_NUMBER 30
#define FLOODING_CHECK_TABLE_SIZE 10
#define FLOODING_TOPOLOGY_TABLE_SIZE 60
#define FLOODING_TOPOLOGY_TABLE_HOLD_TIME 10000

typedef portTickType Time_t;
typedef short set_index_t;

/* Flooding Package */
/* Flooding Body Unit */
typedef struct {
    uint16_t srcAddress; // 2 byte
    uint16_t dstAddress; // 2 byte
    int16_t distance;// 2 byte
} __attribute__((packed)) Flooding_Body_Unit_t; // 6 byte

/* Flooding Message Header */
typedef struct {
    uint16_t srcAddress; // 2 byte
    uint16_t msgSequence; // 2 byte
    uint16_t msgLength; // 2 byte
    uint8_t timeToLive; // 1 byte
} __attribute__((packed)) Flooding_Message_Header_t; // 7 byte

/* Flooding Message */
typedef struct {
    Flooding_Message_Header_t header; // 7 byte
    Flooding_Body_Unit_t bodyUnits[MAX_FLOODING_BODY_UNIT_NUMBER]; // 6 * MAX_FLOODING_BODY_UNIT_NUMBER byte
} __attribute__((packed)) Flooding_Message_t; // 7 + 6 * MAX_FLOODING_BODY_UNIT_NUMBER byte

/* Flooding Table */
/* Flooding Check Table */

// typedef struct {
//     uint16_t srcAddress;
//     uint16_t msgSequence;
// } __attribute__((packed)) Flooding_Check_Table_t;

// void floodingCheckTableInit(Flooding_Check_Table_t *floodingCheckTable, uint16_t address, uint16_t sequence);

// /* Flooding Check Table Set Item */
// typedef struct {
//     set_index_t next;
//     Flooding_Check_Table_t data;
// } __attribute__((packed)) Flooding_Check_Table_Set_Item_t;

// /* Flooding Check Table Set */
// typedef struct {
//     Flooding_Check_Table_Set_Item_t setData[FLOODING_CHECK_TABLE_SIZE];
//     set_index_t freeQueueEntry;
//     set_index_t fullQueueEntry;
//     int size;
// } Flooding_Check_Table_Set_t;

// /* Flooding Check Table Set Operations */
// void floodingCheckTableSetInit(Flooding_Check_Table_Set_t *floodingCheckTableSet);

// set_index_t floodingCheckTableSetInsert(Flooding_Check_Table_Set_t *floodingCheckTableSet,
//                                         Flooding_Check_Table_t *floodingCheckTable);

// set_index_t findInFloodingCheckTableSet(Flooding_Check_Table_Set_t *floodingCheckTableSet,
//                                         uint16_t address);

// void sortFloodingCheckTableSet(Flooding_Check_Table_Set_t *floodingCheckTableSet,
//                                         set_index_t objectIndex);

/* Flooding Topology Table */
typedef struct {
    uint16_t srcAddress;
    uint16_t dstAddress;
    int16_t distance;
    Time_t expirationTime;
} __attribute__((packed)) Flooding_Topology_Table_t;

void floodingTopologyTableInit(Flooding_Topology_Table_t *floodingTopologyTable, uint16_t srcAddress, uint16_t dstAddress, int16_t distance);
void floodingTopologyTableUpdate(Flooding_Topology_Table_t *floodingTopologyTable, int16_t distance);

/* Flooding Topology Table Set Item */
typedef struct {
    set_index_t next;
    Flooding_Topology_Table_t data;
} __attribute__((packed)) Flooding_Topology_Table_Set_Item_t;

/* Flooding Topology Table Set */
typedef struct {
    Flooding_Topology_Table_Set_Item_t setData[FLOODING_TOPOLOGY_TABLE_SIZE];
    set_index_t freeQueueEntry;
    set_index_t fullQueueEntry;
    int size;
} Flooding_Topology_Table_set_t;

/* Flooding Topology Table Set Operations */
void floodingTopologyTableSetInit(Flooding_Topology_Table_set_t *floodingTopologyTableSet);

set_index_t floodingTopologyTableSetInsert(Flooding_Topology_Table_set_t *floodingTopologyTableSet,
                                           uint16_t srcAddress, uint16_t dstAddress, int16_t distance);

set_index_t findInFloodingTopologyTableSet(Flooding_Topology_Table_set_t *floodingTopologyTableSet,
                                           uint16_t srcAddress, uint16_t dstAddress);

void floodingTopologyTableSetClearExpire(Flooding_Topology_Table_set_t *floodingTopologyTableSet);

void floodingTopologyTableSetUpdate(Flooding_Topology_Table_set_t *floodingTopologyTableSet, uint16_t srcAddress, uint16_t dstAddress, int16_t distance);

void printFloodingTopologyTableSet(Flooding_Topology_Table_set_t *floodingTopologyTableSet);

#endif
