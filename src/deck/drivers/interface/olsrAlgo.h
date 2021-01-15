#ifndef __OLSR_ALGO_H__
#define __OLSR_ALGO_H__

#include "olsrPacket.h"

// void olsr_hello_task(void *ptr);



void olsrHelloTask(void *ptr);
void olsrSendTask(void *ptr);
void olsrRecvTask(void *ptr);
void olsrTcTask(void *ptr);
void olsr_ts_task(void *ptr);
void olsrDeviceInit(dwDevice_t *dev);
void olsrRxCallback(dwDevice_t *dev);
void olsrDupTupleTimerExpireTask(void *ptr);
void olsrLinkTupleTimerExpireTask(void *ptr);
void olsrNbTwoHopTupleTimerExpireTask(void *ptr);
void olsrMprSelectorTupleTimerExpireTask(void *ptr);
void olsrTopologyTupleTimerExpireTask(void *ptr);


#endif //__OLSR_ALGO_H__
