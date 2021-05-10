#ifndef __OLSR_ALGO_H__
#define __OLSR_ALGO_H__

#include "olsrPacket.h"
#include "adHocApp.h"

#define OLSR_HELLO_INTERVAL 2000
#define OLSR_NEIGHB_HOLD_TIME (3*OLSR_HELLO_INTERVAL)
#define OLSR_TC_INTERVAL 5000
#define TS_INTERVAL 20 //must be in range: 20 - 500
#define TS_INTERVAL_MIN 20 //default 20
#define TS_INTERVAL_MAX 500 //default 500
#define TS_OTSPOOL_MAXSIZE (4*TS_INTERVAL_MAX/TS_INTERVAL)
#define OLSR_TOP_HOLD_TIME (3*OLSR_TC_INTERVAL)
#define OLSR_DUP_HOLD_TIME 10000
#define OLSR_ROUTING_SET_HOLD_TIME 10000
#define OLSR_RANGING_TABLE_HOLD_TIME 10000
#define OLSR_DUP_CLEAR_INTERVAL 30000
#define OLSR_LINK_CLEAR_INTERVAL 5000
#define OLSR_MPR_SELECTOR_CLEAR_INTERVAL 6000

#define OLSR_NEIGHBOR2HOP_CLEAR_INTERVAL 5000

#define OLSR_MS_CLEAR_INTERVAL 4000

#define OLSR_TOP_CLEAR_INTERVAL 3000

/// Unspecified link type.
#define OLSR_UNSPEC_LINK        0
/// Asymmetric link type.
#define OLSR_ASYM_LINK          1
/// Symmetric link type.
#define OLSR_SYM_LINK           2
/// Lost link type.
#define OLSR_LOST_LINK          3

/// Not neighbor type.
#define OLSR_NOT_NEIGH          0
/// Symmetric neighbor type.
#define OLSR_SYM_NEIGH          1
/// Asymmetric neighbor type.
#define OLSR_MPR_NEIGH          2



#define MAX_TIMESTAMP 1099511627776 //2**40

// void olsr_hello_task(void *ptr);


void olsrHelloTask(void *ptr);
void olsrSendTask(void *ptr);
void olsrRecvTask(void *ptr);
void olsrTcTask(void *ptr);
void olsrTsTask(void *ptr);
void olsrDeviceInit(dwDevice_t *dev);
void olsrRxCallback(dwDevice_t *dev);
void olsrTxCallback(dwDevice_t *dev);
void olsrDupTupleTimerExpireTask(void *ptr);
void olsrLinkTupleTimerExpireTask(void *ptr);
void olsrNbTwoHopTupleTimerExpireTask(void *ptr);
void olsrMprSelectorTupleTimerExpireTask(void *ptr);
void olsrTopologyTupleTimerExpireTask(void *ptr);
void olsrPacketLossTask(void *ptr);
void olsrPacketLossCallBack(dwDevice_t *dev);
void olsrSendData(olsrAddr_t sourceAddr,AdHocPort sourcePort,olsrAddr_t destAddr, AdHocPort destPort,uint16_t portSeq, uint8_t data[],uint8_t length);
olsrWeight_t distanceToWeight(int16_t distance);
int16_t getDistanceFromAddr(olsrAddr_t addr);
#endif //__OLSR_ALGO_H__
