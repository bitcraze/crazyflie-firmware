#include <stdbool.h>

#include "FreeRTOS.h"
#include "dwTypes.h"
#define MAX_NEIGHBOR_SIZE 5
#define RANGING_TABLE_SIZE MAX_NEIGHBOR_SIZE
// #define MAX_BODY_UNIT_NUMBER \
//   (FRAME_LEN_MAX - sizeof(Ranging_Message_Header_t)) / sizeof(Body_Unit_t)
#define MAX_BODY_UNIT_NUMBER 5
#define RANGING_TABLE_HOLD_TIME 10000

typedef uint16_t address_t;
typedef portTickType Time_t;
typedef short set_index_t;

/* Timestamp Tuple */
typedef struct {
  uint16_t seqNumber; // 2 byte
  dwTime_t timestamp; // 8 byte
} __attribute__((packed)) Timestamp_Tuple_t; // 10 byte

/* Body Unit */
typedef struct {
  address_t address; // 2 byte
  Timestamp_Tuple_t timestamp; // 10 byte
} __attribute__((packed)) Body_Unit_t; // 12 byte

/* Ranging Message Header*/
typedef struct {
  address_t srcAddress; // 2 byte
  uint16_t msgSequence; // 2 byte
  Timestamp_Tuple_t lastTxTimestamp; // 10 byte
  short velocity; // 2 byte 
  uint16_t msgLength; // 2 byte
} __attribute__((packed)) Ranging_Message_Header_t; // 18 byte

/* Ranging Message */
typedef struct {
  Ranging_Message_Header_t header; // 18 byte
  Body_Unit_t bodyUnits[MAX_NEIGHBOR_SIZE]; // 12 byte * MAX_NEIGHBOR_SIZE
} __attribute__((packed)) Ranging_Message_t; // 18 + 12 byte * MAX_NEIGHBOR_SIZE

/* Ranging Message With RX Timestamp, used in RX Queue */
typedef struct {
  Ranging_Message_t rangingMessage;
  dwTime_t rxTime;
} __attribute__((packed)) Ranging_Message_With_Timestamp_t;

/* Ranging Table
  +------+------+------+------+------+
  |  Rp  |  Tr  |  Rf  |  P   |  tn  |
  +------+------+------+------+------+
  |  Tp  |  Rr  |  Tf  |  Re  |  ts  |
  +------+------+------+------+------+
*/
typedef struct {
  address_t neighborAddress;

  Timestamp_Tuple_t Rp;
  Timestamp_Tuple_t Tp;
  Timestamp_Tuple_t Rr;
  Timestamp_Tuple_t Tr;
  Timestamp_Tuple_t Rf;
  Timestamp_Tuple_t Tf;
  Timestamp_Tuple_t Re;

  Time_t period;
  Time_t nextDeliveryTime;
  Time_t expirationTime;
  int16_t distance;
} __attribute__((packed)) Ranging_Table_t;

typedef struct {
  set_index_t next;
  Ranging_Table_t data;
} __attribute__((packed)) Ranging_Table_Set_Item_t;

/* Ranging Table Set*/
typedef struct {
  Ranging_Table_Set_Item_t setData[RANGING_TABLE_SIZE];
  set_index_t freeQueueEntry;
  set_index_t fullQueueEntry;
  int size;
} Ranging_Table_Set_t;

Ranging_Table_Set_t rangingTableSet;

/*Ranging Table Set Operations*/
void rangingTableSetInit(Ranging_Table_Set_t *rangingTableSet);

set_index_t rangingTableSetInsert(Ranging_Table_Set_t *rangingTableSet,
                                     Ranging_Table_t *table);

set_index_t findInRangingTableSet(Ranging_Table_Set_t *rangingTableSet,
                                      address_t addr);

bool deleteRangingTupleByIndex(Ranging_Table_Set_t *rangingTableSet,
                                   set_index_t index);

void printRangingTableTuple(Ranging_Table_t *tuple);

void printRangingTable(Ranging_Table_Set_t *rangingTableSet);

void printRangingMessage(Ranging_Message_t *rangingMessage);

bool rangingTableClearExpire(Ranging_Table_Set_t *rangingTableSet);

void sortRangingTableSet(Ranging_Table_Set_t *rangingTableSet);