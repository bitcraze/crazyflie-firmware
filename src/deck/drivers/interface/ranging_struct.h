#include <stdbool.h>

#include "FreeRTOS.h"
#include "dwTypes.h"


#define MAX_BODY_UNIT_NUMBER 30
// #define MAX_BODY_UNIT_NUMBER (FRAME_LEN_MAX - sizeof(Ranging_Message_Header_t)) / sizeof(Body_Unit_t)
#define RANGING_TABLE_SIZE 60
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
  Body_Unit_t bodyUnits[MAX_BODY_UNIT_NUMBER]; // 12 byte * MAX_NEIGHBOR_SIZE
} __attribute__((packed)) Ranging_Message_t; // 18 + 12 byte * MAX_NEIGHBOR_SIZE

/* Ranging Message With RX Timestamp, used in RX Queue */
typedef struct {
  Ranging_Message_t rangingMessage;
  dwTime_t rxTime;
} __attribute__((packed)) Ranging_Message_With_Timestamp_t;

#define Tr_Rr_BUFFER_SIZE 3

typedef struct {
  Timestamp_Tuple_t Tr;
  Timestamp_Tuple_t Rr;
  uint16_t Tf_SeqNumber;
} __attribute__((packed)) Ranging_Table_Tr_Rr_Candidate_t;

/* Tr and Rr candidate buffer for each Ranging Table */
typedef struct {
  set_index_t index; // Always point to oldest data
  Ranging_Table_Tr_Rr_Candidate_t candidates[Tr_Rr_BUFFER_SIZE];
} __attribute__((packed)) Ranging_Table_Tr_Rr_Buffer_t;

/* Tr_Rr Buffer Operations */
void rangingTableBufferInit(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer);
void rangingTableBufferUpdate(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer, Timestamp_Tuple_t Tr, Timestamp_Tuple_t Rr, uint16_t Tf_SeqNumber);
void rangingTableBufferUpdateTimestamp(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer, Timestamp_Tuple_t Tr, Timestamp_Tuple_t Rr);
void rangingTableBufferUpdateTimestampPredecessor(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer, Timestamp_Tuple_t Tr, Timestamp_Tuple_t Rr);
void rangingTableBufferUpdateSeqNumber(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer, uint16_t Tf_SeqNumber);
void rangingTableBufferShift(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer);
Ranging_Table_Tr_Rr_Candidate_t rangingTableBufferGetCandidate(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer, Timestamp_Tuple_t neighborRf);
Ranging_Table_Tr_Rr_Candidate_t rangingTableBufferGetLatestCandidate(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer);

typedef enum {
  RESERVED = 0, 
  TRANSMITTED = 1, 
  RECEIVED = 2,
} RANGING_TABLE_STATE;

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

  Ranging_Table_Tr_Rr_Buffer_t TrRrBuffer;

  Time_t period;
  Time_t nextDeliveryTime;
  Time_t expirationTime;
  int16_t distance;

  RANGING_TABLE_STATE state;
} __attribute__((packed)) Ranging_Table_t;

void rangingTableInit(Ranging_Table_t *rangingTable, address_t address);
void rangingTableShift(Ranging_Table_t *rangingTable);

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
                                     Ranging_Table_t *rangingTable);

set_index_t findInRangingTableSet(Ranging_Table_Set_t *rangingTableSet,
                                      address_t address);

bool deleteRangingTableByIndex(Ranging_Table_Set_t *rangingTableSet,
                                   set_index_t index);

bool rangingTableSetClearExpire(Ranging_Table_Set_t *rangingTableSet);

void sortRangingTableSet(Ranging_Table_Set_t *rangingTableSet);

void printRangingTable(Ranging_Table_t *rangingTable);

void printRangingTableSet(Ranging_Table_Set_t *rangingTableSet);

void printRangingMessage(Ranging_Message_t *rangingMessage);
