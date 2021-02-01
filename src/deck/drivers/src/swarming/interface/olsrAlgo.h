#ifndef __OLSR_ALGO_H__
#define __OLSR_ALGO_H__

#include "olsrPacket.h"
#include "adHocApp.h"

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

#endif //__OLSR_ALGO_H__
