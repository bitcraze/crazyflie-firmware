#ifndef __OLSR_STRUCT_H__
#define __OLSR_STRUCT_H__
#include "FreeRTOS.h"
#include "semphr.h"
#include <queue.h>
#include"adhocdeck.h"
#include <string.h>

/*
*********************Recv&SendQueue*************************
*/

void olsrStructInitAll(dwDevice_t *dev);

/*
*********************NeighborSet*************************
*/
typedef portTickType olsrTime_t;
typedef uint16_t olsrAddr_t;
typedef short olsrDist_t;
typedef short setIndex_t;

#define TOPOLOGY_SET_SIZE 30
#define LINK_SET_SIZE 30
#define MPR_SET_SIZE 30
#define DUPLICATE_SET_SIZE 30
#define TIMESTAMP_SET_SIZE 30
#define MPR_SELECTOR_SET_SIZE 30
#define NEIGHBOR_SET_SIZE 30
#define TWO_HOP_NEIGHBOR_SET_SIZE 30
#define ROUTING_SET_SIZE 30

SemaphoreHandle_t olsrNeighborSetLock;
typedef enum
{
  WILL_NEVER   = 0,
  WILL_LOW     = 1,
  WILL_DEFAULT = 3,
  WILL_HIGH    = 6,
  WILL_ALWAYS  = 7,
} olsrWillingness_t;

typedef enum 
{
  STATUS_NOT_SYM = 0,
  STATUS_SYM = 1,
} olsrNeighborType_t;

typedef struct
{
  olsrAddr_t m_neighborAddr;
  olsrNeighborType_t m_status;
  bool m_isAdvertised;
  olsrWillingness_t m_willingness;
} olsrNeighborTuple_t;

typedef struct 
{
  olsrNeighborTuple_t data;
  setIndex_t next;
} olsrNeighborSetItem_t;
/*
*********************Neighbor2Set*************************
*/
SemaphoreHandle_t olsrTwoHopNeighborSetLock;
typedef struct
{
  olsrAddr_t m_neighborAddr;
  olsrAddr_t m_twoHopNeighborAddr;
  olsrTime_t m_expirationTime; //need fix name
} olsrTwoHopNeighborTuple_t;

typedef struct
{
  olsrTwoHopNeighborTuple_t data;
  setIndex_t next;
} olsrTwoHopNeighborSetItem_t;


/*
*********************LinkSet*************************
*/
SemaphoreHandle_t olsrLinkSetLock;
typedef struct
{
  olsrAddr_t m_localAddr;
  olsrAddr_t m_neighborAddr;
  /// The link is considered bidirectional until this time.
  olsrTime_t m_symTime;
  /// The link is considered unidirectional until this time.
  olsrTime_t m_asymTime;
  olsrTime_t m_expirationTime;
} olsrLinkTuple_t;

typedef struct 
{
  olsrLinkTuple_t data;
  setIndex_t next;
} olsrLinkSetItem_t;


/*
*********************MprSet*************************
*/

SemaphoreHandle_t olsrMprSetLock;
typedef struct 
{
  olsrAddr_t m_addr;
} olsrMprTuple_t;

typedef struct 
{
  olsrMprTuple_t data;
  setIndex_t next;
} olsrMprSetItem_t;
/*
*********************MprSelectorSet*************************
*/

SemaphoreHandle_t olsrMprSelectorSetLock;

typedef struct 
{
  olsrAddr_t m_addr;
  olsrTime_t m_expirationTime;
} olsrMprSelectorTuple_t;

typedef struct 
{
  olsrMprSelectorTuple_t data;
  setIndex_t next;
} olsrMprSelectorSetItem_t;

/*
*********************TopologySet*************************
*/
SemaphoreHandle_t olsrTopologySetLock;
typedef struct 
{
  olsrAddr_t m_destAddr;
  olsrAddr_t m_lastAddr;
  uint16_t m_seqenceNumber;
  olsrDist_t m_distance;
  olsrTime_t m_expirationTime;
} olsrTopologyTuple_t;

typedef struct 
{
  olsrTopologyTuple_t data;
  setIndex_t next;
} olsrTopologySetItem_t;
/*
*********************DuplicateSet*************************
*/
SemaphoreHandle_t olsrDuplicateSetLock;
typedef struct
{
  olsrAddr_t m_addr;
  uint16_t m_seqenceNumber;
  bool m_retransmitted;
  olsrTime_t m_expirationTime;
} olsrDuplicateTuple_t;

typedef struct 
{
  olsrDuplicateTuple_t data;
  setIndex_t next;
} olsrDuplicateSetItem_t;
/*
*********************TimestampSet*************************
*/

typedef struct 
{
  uint16_t m_seqenceNumber;
  dwTime_t m_timestamp;
} olsrTimestampTuple_t;

/*
+------+------+------+------+
|  Rp  |  Tr  |  Rf  |      |
+------+------+------+------+
|  Tp  |  Rr  |  Tf  |  Re  |
+------+------+------+------+
*/
typedef struct {
  olsrAddr_t m_tsAddress; // neighbor address
  // tick from dw1000
  olsrTimestampTuple_t Rp;
  olsrTimestampTuple_t Tp;
  olsrTimestampTuple_t Rr;
  olsrTimestampTuple_t Tr;
  olsrTimestampTuple_t Rf;
  olsrTimestampTuple_t Tf;
  olsrTimestampTuple_t Re;
  // tick from stm32
  olsrTime_t m_period;
  olsrTime_t m_nextDeliveryTime;
  olsrTime_t m_expiration;
  int16_t m_distance;
} olsrRangingTuple_t;

typedef struct 
{
  olsrRangingTuple_t data;
  setIndex_t next;
} olsrRangingTableItem_t;

typedef struct {
  packet_t pkt;
  olsrTimestampTuple_t ots;
}packetWithTimestamp_t;

/*
*********************RoutingTable*************************
*/
typedef struct 
{
  olsrAddr_t m_destAddr;
  olsrAddr_t m_nextAddr;
  olsrDist_t m_distance;
  olsrTime_t m_expirationTime;
} olsrRoutingTuple_t;

typedef struct 
{
  olsrRoutingTuple_t data;
  setIndex_t next;
} olsrRoutingSetItem_t;

/*
********************Set Definition*******************
*/

typedef struct
{
  olsrRoutingSetItem_t setData[ROUTING_SET_SIZE];
  setIndex_t freeQueueEntry;
  setIndex_t fullQueueEntry;
  int size;
} olsrRoutingSet_t;


typedef struct 
{
  olsrTopologySetItem_t setData[TOPOLOGY_SET_SIZE];
  setIndex_t freeQueueEntry;
  setIndex_t fullQueueEntry;
} olsrTopologySet_t;


typedef struct
{
  olsrLinkSetItem_t setData[LINK_SET_SIZE];
  setIndex_t freeQueueEntry; //pos0
  setIndex_t fullQueueEntry; //-1
} olsrLinkSet_t;


typedef struct 
{
  olsrNeighborSetItem_t setData[NEIGHBOR_SET_SIZE];
  setIndex_t freeQueueEntry;
  setIndex_t fullQueueEntry; 
} olsrNeighborSet_t;


typedef struct 
{
  olsrTwoHopNeighborSetItem_t setData[TWO_HOP_NEIGHBOR_SET_SIZE];
  setIndex_t freeQueueEntry;
  setIndex_t fullQueueEntry;
} olsrTwoHopNeighborSet_t;


typedef struct
{
  olsrDuplicateSetItem_t setData[DUPLICATE_SET_SIZE];
  setIndex_t freeQueueEntry;
  setIndex_t fullQueueEntry;
} olsrDuplicateSet_t;


typedef struct 
{
  olsrMprSetItem_t setData[MPR_SET_SIZE];
  setIndex_t freeQueueEntry;
  setIndex_t fullQueueEntry;
} olsrMprSet_t;


typedef struct 
{
  olsrMprSelectorSetItem_t setData[MPR_SELECTOR_SET_SIZE];
  setIndex_t freeQueueEntry;
  setIndex_t fullQueueEntry; 
} olsrMprSelectorSet_t;



typedef struct
{
  olsrRangingTableItem_t setData[TIMESTAMP_SET_SIZE];
  setIndex_t freeQueueEntry;
  setIndex_t fullQueueEntry;
} olsrRangingTable_t;


olsrTopologySet_t olsrTopologySet;

olsrNeighborSet_t olsrNeighborSet;

olsrLinkSet_t olsrLinkSet;

olsrTwoHopNeighborSet_t olsrTwoHopNeighborSet;

olsrDuplicateSet_t olsrDuplicateSet;

olsrMprSet_t olsrMprSet;

olsrMprSelectorSet_t olsrMprSelectorSet;

olsrRoutingSet_t olsrRoutingSet;

olsrRangingTable_t olsrRangingTable;

/*linkSet*/
setIndex_t olsrInsertToLinkSet(olsrLinkSet_t *linkSet, const olsrLinkTuple_t *item);

setIndex_t olsrFindInLinkByAddr(olsrLinkSet_t *linkSet, const olsrAddr_t addr);

void olsrPrintLinkSet(olsrLinkSet_t *linkSet);

setIndex_t olsrFindSymLinkTuple(olsrLinkSet_t *linkSet,olsrAddr_t sender,olsrTime_t now);

void olsrDelLinkTupleByPos(olsrLinkSet_t *linkSet,setIndex_t pos);

/* twoHopNeighbor*/
void olsrTwoHopNeighborSetInit(olsrTwoHopNeighborSet_t *twoHopNeighborSet);

setIndex_t olsrFindTwoHopNeighborTuple(olsrTwoHopNeighborSet_t *twoHopNeighborSet,\
                                       olsrAddr_t neighborAddr,\
                                       olsrAddr_t twoHopNeighborAddr);

setIndex_t olsrInsertToTwoHopNeighborSet(olsrTwoHopNeighborSet_t* twoHopNeighborSet,\
                                        const olsrTwoHopNeighborTuple_t* tuple);

setIndex_t olsrEraseTwoHopNeighborTuple(olsrTwoHopNeighborSet_t* twoHopNeighborSet,\
                                  olsrAddr_t neighborAddr,\
                                  olsrAddr_t twoHopNeighborAddr);
setIndex_t olsrEraseTwoHopNeighborTupleByTuple(olsrTwoHopNeighborSet_t *twoHopNeighborSet,\
                                        olsrTwoHopNeighborTuple_t *tuple);

void olsrEraseTwoHopNeighborTupleByNeighborAddr(olsrTwoHopNeighborSet_t *twoHopNeighborSet,\
                                  olsrAddr_t neighborAddr);

void olsrDelTwoHopNeighborTupleByPos(olsrTwoHopNeighborSet_t *twoHopNeighborSet,setIndex_t pos);

void olsrPrintTwoHopNeighborSet(olsrTwoHopNeighborSet_t *twoHopNeighborSet);

/* Neighbor*/
setIndex_t olsrFindNeighborByAddr(olsrNeighborSet_t* neighborSet,\
                                  olsrAddr_t addr);

setIndex_t olsrInsertToNeighborSet(olsrNeighborSet_t* neighborSet,\
                                  const olsrNeighborTuple_t* tuple);

void olsrNeighborSetInit(olsrNeighborSet_t *neighborSet);

void olsrPrintNeighborSet(olsrNeighborSet_t *neighborSet);


bool olsrDelNeighborByAddr(olsrNeighborSet_t *neighborSet,olsrAddr_t addr);

/*topologySet */
setIndex_t olsrInsertToTopologySet(olsrTopologySet_t *topologySet,\
                              const olsrTopologyTuple_t *tcTuple);

setIndex_t olsrFindNewerTopologyTuple(olsrTopologySet_t *topologyset,\
                                      olsrAddr_t originator,\
                                      uint16_t ansn);

void olsrEraseOlderTopologyTuples(olsrTopologySet_t *topologyset,\
                                  olsrAddr_t originator,\
                                  uint16_t ansn);

setIndex_t olsrFindTopologyTuple(olsrTopologySet_t *topologyset,\
                                 olsrAddr_t destAddr,\
                                 olsrAddr_t lastAddr);

setIndex_t olsrFindInDuplicateSet(olsrDuplicateSet_t *duplicateSet,\
                                  olsrAddr_t originator,uint16_t seq);

void olsrDelTopologyTupleByPos(setIndex_t pos);

void olsrPrintTopologySet(olsrTopologySet_t *topologyset);

/*mpr*/
void olsrMprSetInit(olsrMprSet_t *mprSet);

bool olsrFindMprByAddr(olsrMprSet_t *mprSet,olsrAddr_t addr);

setIndex_t olsrInsertToMprSet(olsrMprSet_t *MprSet,const olsrMprTuple_t *item);

void olsrPrintMprSet(olsrMprSet_t *mprSet);


/*ms*/
setIndex_t olsrInsertToMprSelectorSet(olsrMprSelectorSet_t *mprSelectorSet,\
                                      const olsrMprSelectorTuple_t *item);

setIndex_t olsrFindInMprSelectorSet(olsrMprSelectorSet_t *mprSelectorSet, olsrAddr_t addr);

bool olsrMprSelectorSetIsEmpty();

bool olsrEraseMprSelectorTuples(olsrMprSelectorSet_t *mprSelectorSet, olsrAddr_t addr);

void olsrDelMprSelectorTupleByPos(olsrMprSelectorSet_t *mprSelectorSet,setIndex_t pos);

void olsrPrintMprSelectorSet(olsrMprSelectorSet_t *mprSelectorSet);

/*duplicate*/

setIndex_t olsrInsertDuplicateTuple(olsrDuplicateSet_t *duplicateSet, const olsrDuplicateTuple_t *tuple);

bool olsrDuplicateSetClearExpire();

void olsrPrintDuplicateSet(olsrDuplicateSet_t *duplicateSet);

/*routing table*/
void olsrRoutingSetInit(olsrRoutingSet_t *routingSet);

bool olsrRoutingSetInsert(olsrRoutingSet_t *routingSet,olsrRoutingTuple_t *tuple);

olsrAddr_t olsrFindInRoutingTable(olsrRoutingSet_t *routingSet,olsrAddr_t destAddr);

void olsrPrintRoutingSet(olsrRoutingSet_t *routingSet);

void olsrRoutingSetCopy(olsrRoutingSet_t *dest,olsrRoutingSet_t *source);

/*ranging table*/
void olsrRangingTableInit(olsrRangingTable_t *rangingTable);

bool olsrRangingTableInsert(olsrRangingTable_t *rangingTable, olsrRangingTuple_t *tuple);

olsrAddr_t olsrFindInRangingTable(olsrRangingTable_t *rangingTable, olsrAddr_t addr);

bool olsrDelRangingTupleByAddr(olsrRangingTable_t *rangingTable, setIndex_t addr);

void olsrPrintRangingTable(olsrRangingTable_t *rangingTable);

bool olsrRangingTableClearExpire(olsrRangingTable_t *rangingTable);

void olsrSortRangingTable(olsrRangingTable_t *rangingTable);

#endif //__OLSR_STRUCT_H__