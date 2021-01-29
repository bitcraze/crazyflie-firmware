#include "libdw1000.h"
#include "FreeRTOS.h"
#include <string.h>
#include <semphr.h>
#include "task.h"
#include "olsrDebug.h"
#include "mac.h"
#include "olsrAlgo.h"
#include "olsrStruct.h"
#include "olsrPacket.h"
#include "adHocOnBoardSim.h"
#include "log.h"
#include "math.h"
#include "stdlib.h"

#define ANTENNA_OFFSET 154.0   // In meter
//const
const int antennaDelay = (ANTENNA_OFFSET * 499.2e6 * 128) / 299792458.0; // In radio tick


#define OLSR_HELLO_INTERVAL 2000
#define OLSR_NEIGHB_HOLD_TIME (3*OLSR_HELLO_INTERVAL)
#define OLSR_TC_INTERVAL 5000
#define TS_INTERVAL 200 //must be in range: 20 - 500
#define TS_INTERVAL_MIN 20 //default 20
#define TS_INTERVAL_MAX 500 //default 500
#define TS_OTSPOOL_MAXSIZE (4*TS_INTERVAL_MAX/TS_INTERVAL)
#define OLSR_TOP_HOLD_TIME (3*OLSR_TC_INTERVAL)
#define OLSR_DUP_HOLD_TIME 10000
#define OLSR_ROUTING_SET_HOLD_TIME 10000
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



//
#define OLSR_SIM 1

static dwDevice_t* dwm;
extern uint16_t myAddress;
static xQueueHandle g_olsrSendQueue;
static xQueueHandle g_olsrRecvQueue;
static uint16_t g_staticMessageSeq = 0;
static uint16_t g_ansn = 0;
static SemaphoreHandle_t olsrMessageSeqLock;
static SemaphoreHandle_t olsrAnsnLock;
static SemaphoreHandle_t olsrAllSetLock;
// static bool m_linkTupleTimerFirstTime  = true;

static uint16_t idVelocityX;
static uint16_t idVelocityY;
static uint16_t idVelocityZ;
static float velocity;
static int16_t distanceTowards[10];
int jitter = 0;

int olsr_ts_otspool_idx = 0;
olsrTimestampTuple_t olsr_ts_otspool[TS_OTSPOOL_MAXSIZE] = {0};
packet_t txpacket = {0}; // todo : 如果命名为txPacket的话会和LpsTwrTag.c文件里面定义的static类型变量重名
packetWithTimestamp_t rxPacketWts = {0};
packet_t *rxPacket = &rxPacketWts.pkt;

//TODO define packet and message struct once, save space
//debugging, to be deleted
//TODO delete testDataLength2send
bool olsrMprSelectorTupleTimerExpire();
bool olsrLinkTupleClearExpire();
bool olsrNbTwoHopTupleTimerExpire();
bool olsrTopologyTupleTimerExpire();

//rxcallback

static void olsrSendQueueInit()
{
  g_olsrSendQueue = xQueueCreate(15,sizeof(olsrMessage_t));
  DEBUG_PRINT_OLSR_SYSTEM("SEND_QUEUE_INIT_SUCCESSFUL\n");
}
static void olsrRecvQueueInit()
{
  g_olsrRecvQueue = xQueueCreate(15,sizeof(packetWithTimestamp_t));
  DEBUG_PRINT_OLSR_SYSTEM("RECV_QUEUE_INIT_SUCCESSFUL\n");
}
void olsrDeviceInit(dwDevice_t *dev){
  dwm = dev;
  olsrMessageSeqLock = xSemaphoreCreateMutex();
  olsrAnsnLock = xSemaphoreCreateMutex();
  olsrAllSetLock = xSemaphoreCreateMutex();
  olsrSendQueueInit();
  olsrRecvQueueInit();
  DEBUG_PRINT_OLSR_SYSTEM("Device init finish\n");
}
/**
 * @brief this function is used to get SeqNumber(Message global)
 * @return uint16_t MessageSeqNumber(0->2^16-1)
**/
static uint16_t getSeqNumber()
{
  uint16_t retVal= 0;
  xSemaphoreTake(olsrMessageSeqLock,portMAX_DELAY);
  retVal = g_staticMessageSeq++;
  xSemaphoreGive(olsrMessageSeqLock);
  return retVal;
}

void olsrPacketLossCallBack(dwDevice_t *dev)
{
    //packet_t rxPacket;
    static int count = 0;
    static int notOlsr = 0;
    unsigned int dataLength = dwGetDataLength(dwm);
    if(dataLength==0){
				return;
		}
    memset(rxPacket,0,sizeof(packet_t));
    dwGetData(dwm,(uint8_t*)rxPacket,dataLength);
    if(rxPacket->fcf_s.type!=MAC802154_TYPE_OLSR){
        // DEBUG_PRINT_OLSR_RECEIVE("Mac_T not OLSR!\n");
        notOlsr++;
				return;
		}
    count++;
    int p;
    memcpy(&p,&rxPacket->payload,sizeof(int));
    DEBUG_PRINT_OLSR_RECEIVE("receive %d not olsr,success receive %d/%d(RecvcountGT,RecvCount)\n",notOlsr,count,p);
}

void olsrRxCallback(dwDevice_t *dev) {
  //packet_t rxPacket;
  DEBUG_PRINT_OLSR_SYSTEM("rxCallBack\n");
  unsigned int dataLength = dwGetDataLength(dwm);
  if (dataLength == 0) {
    DEBUG_PRINT_OLSR_RECEIVE("DataLen=0!\n");
    return;
  }
  memset(rxPacket, 0, sizeof(packet_t));

  dwGetData(dwm, (uint8_t *) rxPacket, dataLength);
  dwTime_t *arrival = &rxPacketWts.ots.m_timestamp;
  dwGetReceiveTimestamp(dev, arrival);
  arrival->full -= (antennaDelay / 2);
  //DEBUG_PRINT_OLSR_TS("arrival=%2x%8lx\n", arrival->high8, arrival->low32);

  olsrPacketHeader_t *olsrPacketHeader = (olsrPacketHeader_t *) &rxPacket->payload;
  rxPacketWts.ots.m_seqenceNumber = olsrPacketHeader->m_packetSeq;

  //DEBUG_PRINT_OLSR_RECEIVE("dwPkt:Len=%d,fcf=%X,type=%d\n", dataLength, rxPacket->fcf, rxPacket->fcf_s.type);
  // DEBUG_PRINT_OLSR_RECEIVE("dataLength=%d\n", dataLength);
  // DEBUG_PRINT_OLSR_RECEIVE("fcf: %X\n", rxPacket.fcf);
  if (rxPacket->fcf_s.type != MAC802154_TYPE_OLSR) {
    // DEBUG_PRINT_OLSR_RECEIVE("Mac_T not OLSR!\n");
    return;
  }
#ifdef DEBUG_OLSR_RECEIVE_DECODEHDR
    locoAddress_t u64 = 0;
DEBUG_PRINT_OLSR_RECEIVE("dataLength=%d\n", dataLength);
DEBUG_PRINT_OLSR_RECEIVE("fcf: %X\n", rxPacket.fcf);
DEBUG_PRINT_OLSR_RECEIVE("seq: %X\n", rxPacket.seq);
DEBUG_PRINT_OLSR_RECEIVE("pan: %X\n", rxPacket.pan);
u64=rxPacket.destAddress;
DEBUG_PRINT_OLSR_RECEIVE("dst: %X%X%X\n", u64>>32, u64, u64);
u64=rxPacket.sourceAddress;
DEBUG_PRINT_OLSR_RECEIVE("src: %X%X%X\n", u64>>32, u64, u64);
#endif
  xQueueSend(g_olsrRecvQueue, &rxPacketWts, portMAX_DELAY);
}

void olsrTxCallback(dwDevice_t *dev) {
  DEBUG_PRINT_OLSR_SYSTEM("txCallBack\n");
  if (txpacket.fcf_s.reserved != 1) {
    return; // packet includes no ts message
  }
  olsrPacket_t *txOlsrPkt = (olsrPacket_t *) txpacket.payload;

  dwTime_t departure = {.full = 0};
  dwGetTransmitTimestamp(dev, &departure);
  departure.full += (antennaDelay / 2);
  //DEBUG_PRINT_OLSR_TS("departure=%2x%8lx\n", departure.high8, departure.low32);
  olsr_ts_otspool_idx++;
  olsr_ts_otspool_idx %= TS_OTSPOOL_MAXSIZE;
  olsr_ts_otspool[olsr_ts_otspool_idx].m_seqenceNumber = txOlsrPkt->m_packetHeader.m_packetSeq;
  olsr_ts_otspool[olsr_ts_otspool_idx].m_timestamp = departure;
  DEBUG_PRINT_OLSR_SEND("otspool update [%d] with %d.\n", olsr_ts_otspool_idx, txOlsrPkt->m_packetHeader.m_packetSeq);
}

//packet process

void olsrProcessTs(const olsrMessage_t* tsMsg, const olsrTimestampTuple_t *rxOTS){

}

static void incrementAnsn()
{
  xSemaphoreTake(olsrAnsnLock,portMAX_DELAY);
  g_ansn++;
  xSemaphoreGive(olsrAnsnLock);
}
static uint16_t getAnsn()
{
  xSemaphoreTake(olsrAnsnLock,portMAX_DELAY); 
  uint16_t retVal = g_ansn;
  xSemaphoreGive(olsrAnsnLock);
  return retVal;
}

static void addNeighborTuple(olsrNeighborSet_t* neighborSet ,const olsrNeighborTuple_t* tuple)
{
  olsrInsertToNeighborSet(neighborSet,tuple);
  incrementAnsn();
}

static void addDuplicateTuple(olsrDuplicateSet_t *duplicateSet, const olsrDuplicateTuple_t *tuple)
{
  olsrInsertDuplicateTuple(duplicateSet,tuple);
}

static void addTwoHopNeighborTuple(const olsrTwoHopNeighborTuple_t* tuple)
{
  olsrInsertToTwoHopNeighborSet(&olsrTwoHopNeighborSet,tuple);
}

static void addMprSelectorTuple(const olsrMprSelectorTuple_t * tuple)
{
  olsrInsertToMprSelectorSet(&olsrMprSelectorSet,tuple);
  incrementAnsn();
}

static void addTopologyTuple(olsrTopologySet_t* topologySet ,const olsrTopologyTuple_t *tuple)
{
  olsrInsertToTopologySet(topologySet,tuple);
}

static void linkTupleAdded(olsrLinkTuple_t *tuple,uint8_t willingness)
{
  // Creates associated neighbor tuple
  olsrNeighborTuple_t nbTuple;
  nbTuple.m_neighborAddr = tuple->m_neighborAddr;
  nbTuple.m_willingness = willingness;
  olsrTime_t now = xTaskGetTickCount();
  if(tuple->m_symTime >= now)
    {
      nbTuple.m_status = STATUS_SYM;
    }
  else
    {
      nbTuple.m_status = STATUS_NOT_SYM;
    }
  if(olsrFindNeighborByAddr(&olsrNeighborSet,tuple->m_neighborAddr) == -1)
    {
      addNeighborTuple(&olsrNeighborSet,&nbTuple);
    }
}
static void linkTupleUpdated(olsrLinkTuple_t *tuple,uint8_t willingness)
{
  //// Each time a link tuple changes, the associated neighbor tuple must be recomputed
  setIndex_t neighborTuple = olsrFindNeighborByAddr(&olsrNeighborSet,\
                                                    tuple->m_neighborAddr);
  if(neighborTuple==-1)
    {
      linkTupleAdded(tuple,willingness);
      neighborTuple = olsrFindNeighborByAddr(&olsrNeighborSet,\
                                            tuple->m_neighborAddr);
    }
  
  if(neighborTuple!=-1)
    {
      bool hasSymmetricLink = false;
      setIndex_t linkTupleIter = olsrLinkSet.fullQueueEntry;
      while(linkTupleIter!=-1)
        {
          if(olsrLinkSet.setData[linkTupleIter].data.m_neighborAddr == tuple->m_neighborAddr &&\
          olsrLinkSet.setData[linkTupleIter].data.m_symTime >= xTaskGetTickCount())
            {
              hasSymmetricLink = true;
              break;
            }
          linkTupleIter = olsrLinkSet.setData[linkTupleIter].next;
        }
      if(hasSymmetricLink)
        {
          olsrNeighborSet.setData[neighborTuple].data.m_status = STATUS_SYM;
        }
      else
        {
          olsrNeighborSet.setData[neighborTuple].data.m_status = STATUS_NOT_SYM;
        }
      
    }
  else
    {
      DEBUG_PRINT_OLSR_LINK("bad alloc in linkTupleUpdated func!\n");
    }
  

}
static void linkSensing(const olsrMessage_t* helloMsg)
{
  configASSERT(helloMsg->m_messageHeader.m_vTime>0);
  setIndex_t linkTuple = olsrFindInLinkByAddr(&olsrLinkSet, helloMsg->m_messageHeader.m_originatorAddress);
  olsrTime_t now = xTaskGetTickCount();
  bool created = false, updated = false;
  if(linkTuple == -1)
    {
      DEBUG_PRINT_OLSR_LINK("not found Addr %d in linkSet\n", helloMsg->m_messageHeader.m_originatorAddress);
      olsrLinkTuple_t newLinkTuple;
      newLinkTuple.m_localAddr = myAddress;
      newLinkTuple.m_neighborAddr = helloMsg->m_messageHeader.m_originatorAddress;
      newLinkTuple.m_symTime = now - M2T(1000);
      newLinkTuple.m_expirationTime = now + helloMsg->m_messageHeader.m_vTime;
      linkTuple = olsrInsertToLinkSet(&olsrLinkSet,&newLinkTuple);
      if(linkTuple==-1)
        {
          DEBUG_PRINT_OLSR_LINK("can not malloc from link set by Function[linkSensing]\n");
          return;
        }
      created = true;
    }
  else
    {
      updated = true;
    }
  olsrLinkSet.setData[linkTuple].data.m_asymTime = now +helloMsg->m_messageHeader.m_vTime;
  olsrHelloMessage_t* helloMsgBody = (olsrHelloMessage_t*)(helloMsg->m_messagePayload);
  for(uint8_t i = 0;i < helloMsgBody->m_helloHeader.m_linkMessageNumber;i++)
    {
      uint8_t lt = helloMsgBody->m_linkMessage[i].m_linkCode & 0x03;
      uint8_t nt = (helloMsgBody->m_linkMessage[i].m_linkCode >> 2 ) & 0x03;

      if((lt == OLSR_SYM_LINK && nt == OLSR_NOT_NEIGH)\
      ||(nt != OLSR_SYM_NEIGH && nt != OLSR_MPR_NEIGH\
      && nt != OLSR_NOT_NEIGH))
        {
          continue;
        }
      if(helloMsgBody->m_linkMessage[i].m_addresses == myAddress)
        {
          if(lt == OLSR_LOST_LINK)
            {
              DEBUG_PRINT_OLSR_LINK("this addr %d is lost link\n",helloMsgBody->m_linkMessage[i].m_addresses);
              olsrLinkSet.setData[linkTuple].data.m_symTime = now - M2T(1000);
              updated = true;
            }
          else if(lt == OLSR_SYM_LINK ||lt == OLSR_ASYM_LINK)
            {
              olsrLinkSet.setData[linkTuple].data.m_symTime = now +helloMsg->m_messageHeader.m_vTime;
              olsrLinkSet.setData[linkTuple].data.m_expirationTime = olsrLinkSet.setData[linkTuple].data.m_symTime+OLSR_NEIGHB_HOLD_TIME;
              updated = true;
            }
          else
            {
              DEBUG_PRINT_OLSR_LINK("BAD LINK");
            }  
        }
      else
        {
          DEBUG_PRINT_OLSR_LINK("this %d is not equal to myaddress\n",helloMsgBody->m_linkMessage[i].m_addresses);
        }      
    }
  olsrLinkSet.setData[linkTuple].data.m_expirationTime = olsrLinkSet.setData[linkTuple].data.m_asymTime > \
                                                olsrLinkSet.setData[linkTuple].data.m_expirationTime? \
                                                olsrLinkSet.setData[linkTuple].data.m_asymTime:\
                                                olsrLinkSet.setData[linkTuple].data.m_expirationTime;
  if(updated)
    {
      linkTupleUpdated(&olsrLinkSet.setData[linkTuple].data,helloMsgBody->m_helloHeader.m_willingness);
      DEBUG_PRINT_OLSR_LINK("update\n");
    }
  if(created)
    {
      linkTupleAdded(&olsrLinkSet.setData[linkTuple].data,helloMsgBody->m_helloHeader.m_willingness); // same addr?
      //del
    }
}

void populateNeighborSet(const olsrMessage_t* helloMsg)
{
  setIndex_t nbTuple = olsrFindNeighborByAddr(&olsrNeighborSet,
                                              helloMsg->m_messageHeader.m_originatorAddress);
  if(nbTuple != -1)
    {
      olsrNeighborSet.setData[nbTuple].data.m_willingness = ((olsrHelloMessageHeader_t *)helloMsg->m_messagePayload)->m_willingness;
      DEBUG_PRINT_OLSR_NEIGHBOR("populate successful\n");
    }
}

void populateTwoHopNeighborSet(const olsrMessage_t* helloMsg)
{
  olsrTime_t now = xTaskGetTickCount();
  olsrAddr_t sender = helloMsg->m_messageHeader.m_originatorAddress;
  setIndex_t linkTuple = olsrFindInLinkByAddr(&olsrLinkSet, sender);
  if(linkTuple != -1)
    {
      olsrHelloMessage_t* helloMsgBody = (olsrHelloMessage_t*)helloMsg->m_messagePayload;
      for(int i = 0;i<helloMsgBody->m_helloHeader.m_linkMessageNumber;i++)
        {
          uint8_t nbType = (helloMsgBody->m_linkMessage[i].m_linkCode >> 2) & 0x3;
          olsrAddr_t candidate = helloMsgBody->m_linkMessage[i].m_addresses;
          if(nbType == OLSR_SYM_NEIGH || nbType == OLSR_MPR_NEIGH)
            {
              if(candidate == myAddress)
                {
                  continue;
                }
              setIndex_t twoHopNeighborTuple = olsrFindTwoHopNeighborTuple(&olsrTwoHopNeighborSet,sender,candidate);
              if(twoHopNeighborTuple != -1)
                {
                  olsrTwoHopNeighborSet.setData[twoHopNeighborTuple].data.m_expirationTime = now+\
                                                    helloMsg->m_messageHeader.m_vTime;
                }
              else
                {
                  olsrTwoHopNeighborTuple_t newTuple;
                  newTuple.m_neighborAddr = sender;
                  newTuple.m_twoHopNeighborAddr = candidate;
                  newTuple.m_expirationTime = now+helloMsg->m_messageHeader.m_vTime;
                  addTwoHopNeighborTuple(&newTuple);
                }
            }
          else if(nbType == OLSR_NOT_NEIGH)
            {
              olsrEraseTwoHopNeighborTuple(&olsrTwoHopNeighborSet, sender,candidate);
            }
          else
            {
              DEBUG_PRINT_OLSR_NEIGHBOR2("bad neighbor type in func [PopulateTwoHopNeighborSet]\n");
            }
          
        }
    }
  else
    {
      DEBUG_PRINT_OLSR_NEIGHBOR2("can not found link tuple\n");
    }
}

void populateMprSelectorSet(const olsrMessage_t* helloMsg)
{
  olsrTime_t now = xTaskGetTickCount();
  olsrAddr_t sender =  helloMsg->m_messageHeader.m_originatorAddress;
  olsrHelloMessage_t* helloBody = (olsrHelloMessage_t *)helloMsg->m_messagePayload;
  for(int i = 0;i< helloBody->m_helloHeader.m_linkMessageNumber;i++)
    {
      uint8_t nt = helloBody->m_linkMessage[i].m_linkCode >> 2;
      if(nt == OLSR_MPR_NEIGH && helloBody->m_linkMessage[i].m_addresses == myAddress)
        {
          setIndex_t candidate = olsrFindInMprSelectorSet(&olsrMprSelectorSet,sender);
          if(candidate == -1)
            {
              olsrMprSelectorTuple_t new;
              new.m_addr = sender;
              new.m_expirationTime = now + helloMsg->m_messageHeader.m_vTime;
              addMprSelectorTuple(&new);
            }
          else
            {
              olsrMprSelectorSet.setData[candidate].data.m_expirationTime = now + helloMsg->m_messageHeader.m_vTime;
            }
        }
    }

}
void coverTwoHopNeighbors(olsrAddr_t addr, olsrTwoHopNeighborSet_t *twoHopNeighborSet)
{
  setIndex_t n2NeedToEraseIt = twoHopNeighborSet->fullQueueEntry;
  while(n2NeedToEraseIt != -1)
    {
      olsrTwoHopNeighborTuple_t item = twoHopNeighborSet->setData[n2NeedToEraseIt].data;
      if(item.m_neighborAddr == addr)
        {
          n2NeedToEraseIt = olsrEraseTwoHopNeighborTupleByTuple(twoHopNeighborSet,&item);
          continue;
        }
      n2NeedToEraseIt = twoHopNeighborSet->setData[n2NeedToEraseIt].next;
    }  
}
void mprCompute()
{
  olsrMprSetInit(&olsrMprSet);
  olsrNeighborSet_t N;
  olsrNeighborSetInit(&N);

  setIndex_t itForOlsrNeighborSet = olsrNeighborSet.fullQueueEntry;
  while(itForOlsrNeighborSet != -1)
    {
      if(olsrNeighborSet.setData[itForOlsrNeighborSet].data.m_status == STATUS_SYM)
        {
          olsrInsertToNeighborSet(&N,&olsrNeighborSet.setData[itForOlsrNeighborSet].data);
        }
      itForOlsrNeighborSet = olsrNeighborSet.setData[itForOlsrNeighborSet].next;
    }
  
  olsrTwoHopNeighborSet_t N2;
  olsrTwoHopNeighborSetInit(&N2);

  setIndex_t itForTwoHopNeighborSet = olsrTwoHopNeighborSet.fullQueueEntry;
  while(itForTwoHopNeighborSet != -1)
    {
      olsrTwoHopNeighborSetItem_t twoHopNTuple = olsrTwoHopNeighborSet.setData[itForTwoHopNeighborSet];
      //two hop neighbor can not equal to myself
      //TODO bianli fangshi check
      if(twoHopNTuple.data.m_twoHopNeighborAddr == myAddress)
        {
          itForTwoHopNeighborSet = twoHopNTuple.next;
          continue;
        }
      
      bool ok = false;
      setIndex_t itForN = N.fullQueueEntry;
      while(itForN != -1)
        {
          if(N.setData[itForN].data.m_neighborAddr == twoHopNTuple.data.m_neighborAddr)
            {
              // N.setData[itForN].data.m_willingness == WILL_NEVER? ok=false: ok=true;
              ok = true;
              break;
            }
          itForN = N.setData[itForN].next;
        }
      if(!ok)
        {
          itForTwoHopNeighborSet = twoHopNTuple.next;
          DEBUG_PRINT_OLSR_MPR("464 continue\n");
          continue;
        }
      //this two hop neighbor can not be one hop neighbor
      itForN = N.fullQueueEntry;
      while(itForN != -1)
        {
          if(N.setData[itForN].data.m_neighborAddr == twoHopNTuple.data.m_twoHopNeighborAddr) //A-B A-C B-C
            {
              ok = false;
              break;
            }
            itForN = N.setData[itForN].next;
        }

      if(ok)
        {
          olsrInsertToTwoHopNeighborSet(&N2,&twoHopNTuple.data);
        }

      itForTwoHopNeighborSet = twoHopNTuple.next;
    }
  olsrAddr_t coveredTwoHopNeighbors[TWO_HOP_NEIGHBOR_SET_SIZE];
  for(int i =0;i<TWO_HOP_NEIGHBOR_SET_SIZE;i++)
    {
      coveredTwoHopNeighbors[i] = 0;
    }
  int lenthOfCoveredTwoHopNeighbor = 0;
    //find the unique pair of two hop neighborN2
  setIndex_t n2It = N2.fullQueueEntry;
   //A-B-C  A-Q-C  A-E-F 
  while(n2It != -1)
    {
      bool onlyOne = true;
      setIndex_t otherN2It = N2.fullQueueEntry;
      while(otherN2It != -1)
        {
          if(N2.setData[otherN2It].data.m_twoHopNeighborAddr == N2.setData[n2It].data.m_twoHopNeighborAddr &&N2.setData[otherN2It].data.m_neighborAddr != N2.setData[n2It].data.m_neighborAddr)
            {
              onlyOne = false;
              break;
            }
          otherN2It = N2.setData[otherN2It].next;
        }
      if(onlyOne&&!olsrFindMprByAddr(&olsrMprSet, N2.setData[n2It].data.m_neighborAddr))//TODO if 
        {
          // DEBUG_PRINT_OLSR_MPR("this addr:% is matched a isolatied node in two hop\n",N2.setData[n2It].data.m_neighborAddr);
          olsrMprTuple_t item;
          item.m_addr = N2.setData[n2It].data.m_neighborAddr;
          olsrInsertToMprSet(&olsrMprSet,&item);
          
          setIndex_t twoHopNIt = N2.fullQueueEntry;
          while(twoHopNIt != -1)
            {
              if(N2.setData[twoHopNIt].data.m_neighborAddr == N2.setData[n2It].data.m_neighborAddr)
                {
                  coveredTwoHopNeighbors[lenthOfCoveredTwoHopNeighbor++] = N2.setData[twoHopNIt].data.m_twoHopNeighborAddr;
                }
              twoHopNIt = N2.setData[twoHopNIt].next;
            }
        }
      n2It = N2.setData[n2It].next;
    }

  // //erase 
  DEBUG_PRINT_OLSR_MPR("lenthOfCoveredTwoHopNeighbor is %d\n",lenthOfCoveredTwoHopNeighbor);
  setIndex_t itForEraseFromN2 = N2.fullQueueEntry;
  while(itForEraseFromN2 != -1)
    {
      bool find = false;
      for(int i=0;i<lenthOfCoveredTwoHopNeighbor;i++)
        {
          if(coveredTwoHopNeighbors[i] == N2.setData[itForEraseFromN2].data.m_twoHopNeighborAddr)
            {
              find = true;
              break;
            }
        }
      if(find)
        {
          setIndex_t tmp = itForEraseFromN2;
          itForEraseFromN2 = olsrEraseTwoHopNeighborTupleByTuple(&N2,&N2.setData[itForEraseFromN2].data);
          if(tmp != itForEraseFromN2) continue;
        }
      itForEraseFromN2 = N2.setData[itForEraseFromN2].next;
    } //TODO 有一些冗余
  while(N2.fullQueueEntry != -1) //N N2 A-B-C A-Q-C A-G-
    {
      DEBUG_PRINT_OLSR_NEIGHBOR2("in mpr cmpute !!!\n");
      olsrPrintTwoHopNeighborSet(&N2);
      int maxR = 0;
      setIndex_t maxNeighborTuple = -1;
      uint8_t num[NEIGHBOR_SET_SIZE];
      for(int i=0;i<NEIGHBOR_SET_SIZE;i++)
        {
          num[i] = 0;
        }
      setIndex_t nbIt = N.fullQueueEntry;
      while(nbIt != -1)
        {
          setIndex_t nbTwoHopIt = N2.fullQueueEntry;
          uint8_t count = 0;
          while(nbTwoHopIt != -1)
            {
              if(N2.setData[nbTwoHopIt].data.m_neighborAddr 
                == N.setData[nbIt].data.m_neighborAddr)
              {
                count++;
              }
              nbTwoHopIt = N2.setData[nbTwoHopIt].next;
            }
          num[nbIt] = count;
          nbIt = N.setData[nbIt].next;
        }
      for(int i = 0;i<NEIGHBOR_SET_SIZE;i++)
        {
          if(num[i]>maxR)
            {
              maxR = num[i];
              maxNeighborTuple = i;
            }
        }
      if(maxNeighborTuple != -1)
        {
          olsrMprTuple_t tmp;
          tmp.m_addr = N.setData[maxNeighborTuple].data.m_neighborAddr;
          olsrInsertToMprSet(&olsrMprSet,&tmp);
          coverTwoHopNeighbors(tmp.m_addr,&N2);
        }
    }

}
static void olsrSetExpire()
{
  olsrLinkTupleClearExpire();//
  olsrNbTwoHopTupleTimerExpire();
  olsrMprSelectorTupleTimerExpire();
  olsrTopologyTupleTimerExpire();
}
void olsrPrintAll()
{
  olsrPrintLinkSet(&olsrLinkSet);
  olsrPrintNeighborSet(&olsrNeighborSet);
  olsrPrintTwoHopNeighborSet(&olsrTwoHopNeighborSet);
  olsrPrintMprSet(&olsrMprSet);
  olsrPrintTopologySet(&olsrTopologySet);
  olsrPrintMprSelectorSet(&olsrMprSelectorSet);
  olsrPrintRoutingSet(&olsrRoutingSet);
  // olsrPrintDuplicateSet(&olsrDuplicateSet);
}
void olsrProcessHello(const olsrMessage_t* helloMsg)
{
  linkSensing(helloMsg);
  populateNeighborSet(helloMsg);
  populateTwoHopNeighborSet(helloMsg); 
  mprCompute();
  populateMprSelectorSet(helloMsg);
  olsrPrintAll();
}
//
void olsrProcessTc(const olsrMessage_t* tcMsg)
{
  olsrPrintLinkSet(&olsrLinkSet);
  olsrTime_t now = xTaskGetTickCount();
  olsrAddr_t originator = tcMsg->m_messageHeader.m_originatorAddress;
  olsrAddr_t sender = tcMsg->m_messageHeader.m_relayAddress;
  olsrTopologyMessage_t* tcBody = (olsrTopologyMessage_t *)tcMsg->m_messagePayload;
  uint16_t ansn = tcBody->m_ansn;

  setIndex_t linkTupleIndex = olsrFindSymLinkTuple(&olsrLinkSet,sender,now);
  if(linkTupleIndex == -1)
    {
      DEBUG_PRINT_OLSR_TC("not from sym link.\n");
      return;
    }
  setIndex_t topologyTupleIndex = olsrFindNewerTopologyTuple(&olsrTopologySet,originator,ansn);
  if(topologyTupleIndex != -1)
    {
      DEBUG_PRINT_OLSR_TC("not the newerst one\n");
      return;
    }
  olsrEraseOlderTopologyTuples(&olsrTopologySet,originator,ansn);
  uint8_t count = (tcMsg->m_messageHeader.m_messageSize - sizeof(olsrMessageHeader_t) - 2)/ \
                  sizeof(olsrTopologyMessageUint_t);
  for(int i = 0;i < count ;i++)
    {
      olsrAddr_t destAddr =  tcBody->m_content[i].m_address;
      setIndex_t topologyIt = olsrFindTopologyTuple(&olsrTopologySet,destAddr,originator);
      if(topologyIt != -1)
        {
          olsrTopologySet.setData[topologyIt].data.m_expirationTime = now + tcMsg->m_messageHeader.m_vTime;
        }
      else
        {
          olsrTopologyTuple_t topologyTuple;
          topologyTuple.m_destAddr = destAddr;
          topologyTuple.m_lastAddr = originator;
          topologyTuple.m_seqenceNumber = ansn;
          topologyTuple.m_expirationTime = now + tcMsg->m_messageHeader.m_vTime;
          topologyTuple.m_distance = tcBody->m_content[i].m_distance;
          addTopologyTuple(&olsrTopologySet,&topologyTuple);
        }
    }
}

void olsrProcessData(olsrMessage_t* msg)
{
  olsrDataMessage_t* dataMsg = (olsrDataMessage_t *)msg->m_messagePayload;
  if(dataMsg->m_dataHeader.m_nextHop != myAddress)
    {
      DEBUG_PRINT_OLSR_ROUTING("m_nextHop!=myAddress\n");
      return;
    }
  if(msg->m_messageHeader.m_destinationAddress == myAddress)
    {
      //up
      if(adHocPortIsUsed(dataMsg->m_dataHeader.m_destPort))
        {
          DEBUG_PRINT_OLSR_ROUTING("add DataMessage to Queue\n");
          adHocAddToQueue(dataMsg->m_dataHeader.m_destPort,dataMsg);
        }
      else
        {
          DEBUG_PRINT_OLSR_ROUTING("this port is not used in data Process\n");
          return;
        }
    }
  else
    {
      olsrAddr_t nextHop = olsrFindInRoutingTable(&olsrRoutingSet,msg->m_messageHeader.m_destinationAddress);
      if(nextHop != -1)
        {
          dataMsg->m_dataHeader.m_nextHop = nextHop; 
        }
      else
        {
          DEBUG_PRINT_OLSR_ROUTING("can not find next hop\n");
          return;
        }
      xQueueSend(g_olsrSendQueue,msg,portMAX_DELAY);
    }
}
void forwardDefault(olsrMessage_t* olsrMessage, setIndex_t duplicateIndex)
{
  olsrTime_t now = xTaskGetTickCount();
  olsrMessageHeader_t* msgHeader = &olsrMessage->m_messageHeader;
  olsrAddr_t sender = msgHeader->m_relayAddress;
  uint16_t seq = msgHeader->m_messageSeq;
  setIndex_t symIndex = olsrFindSymLinkTuple(&olsrLinkSet,sender,now);
  if(symIndex == -1)
    {
      DEBUG_PRINT_OLSR_FORWARD("not from sym link\n");
      return;
    }
  if(msgHeader->m_timeToLive > 1)
    {
      setIndex_t mprSelIndex = olsrFindInMprSelectorSet(&olsrMprSelectorSet, sender);
      if(mprSelIndex != -1)
        {
          msgHeader->m_timeToLive--;
          msgHeader->m_hopCount++;
          msgHeader->m_relayAddress = myAddress;
          xQueueSend(g_olsrSendQueue,olsrMessage,portMAX_DELAY);
          DEBUG_PRINT_OLSR_FORWARD("forward successful\n");
        }
    }
  if(duplicateIndex != -1)
    {
      olsrDuplicateSet.setData[duplicateIndex].data.m_expirationTime = now + OLSR_DUP_HOLD_TIME;
    }
  else
    {
      olsrDuplicateTuple_t newDup;
      newDup.m_addr = sender;
      newDup.m_expirationTime = now + OLSR_DUP_HOLD_TIME;
      newDup.m_seqenceNumber = seq;
      newDup.m_retransmitted = true;
      addDuplicateTuple(&olsrDuplicateSet, &newDup);
    }
}
//switch to tc|hello|ts process
void olsrPrintPacket(const packet_t* rxPacket)
{
  olsrPacket_t* olsrPacket = (olsrPacket_t*)rxPacket->payload;
  int lengthPacket = olsrPacket->m_packetHeader.m_packetLength;
  DEBUG_PRINT_OLSR_HELLO("packet Length: %d\n",lengthPacket);
  void* olsrMessage = (void *)olsrPacket->m_packetPayload;
  unsigned int index = 4;
  //TODO 一个packet里有俩个helloMessage
  while(index < lengthPacket)
  {
    olsrMessage_t *olsrIndex = (olsrMessage_t *)olsrMessage;
    if(olsrIndex->m_messageHeader.m_messageType==HELLO_MESSAGE)
      {
        DEBUG_PRINT_OLSR_HELLO("hello msg from %d\n",olsrIndex->m_messageHeader.m_originatorAddress);
        olsrHelloMessage_t* helloMessage = (olsrHelloMessage_t *)olsrIndex->m_messagePayload;
        uint8_t length = helloMessage->m_helloHeader.m_linkMessageNumber;
        DEBUG_PRINT_OLSR_HELLO("linkNumber:%d\n",length);
        for(int i=0;i<length;i++)
          {
            DEBUG_PRINT_OLSR_HELLO("link %d :addr:%d ,link code is %d\n",i,helloMessage->m_linkMessage[i].m_addresses,helloMessage->m_linkMessage[i].m_linkCode);
          }
        index+=(sizeof(olsrHelloMessageHeader_t) +length*sizeof(olsrLinkMessage_t)+16);
        olsrMessage+=(sizeof(olsrHelloMessageHeader_t) +length*sizeof(olsrLinkMessage_t)+16);
      }
    else if(olsrIndex->m_messageHeader.m_messageType==TC_MESSAGE)
      {
        DEBUG_PRINT_OLSR_TC("this message is tc msg\n");
        olsrTopologyMessage_t *tcMessage = (olsrTopologyMessage_t *) olsrIndex->m_messagePayload;
        unsigned int count = (olsrIndex->m_messageHeader.m_messageSize- sizeof(olsrMessageHeader_t) - 2)/ \
                    sizeof(olsrTopologyMessageUint_t);
        for(int i = 0;i < count; i++)
          {
            DEBUG_PRINT("%d: address %d .distance is %d",i,tcMessage->m_content[i].m_address,tcMessage->m_content[i].m_distance);
          }
      }
  }
  DEBUG_PRINT_OLSR_HELLO("leave PrintPacket function\n");
}
void olsrRoutingTableComputation2()
{
  olsrRoutingSet_t tmpRoutingSet;
  olsrRoutingSetInit(&tmpRoutingSet);

  setIndex_t neighborIt = olsrNeighborSet.fullQueueEntry;
  while(neighborIt != -1)
  {
    if(olsrNeighborSet.setData[neighborIt].data.m_status == STATUS_SYM)
    {
      setIndex_t linkIt = olsrLinkSet.fullQueueEntry;
      while(linkIt != -1)
      {
        if(olsrNeighborSet.setData[neighborIt].data.m_neighborAddr == olsrLinkSet.setData[linkIt].data.m_neighborAddr)
        {
          olsrRoutingTuple_t add;
          add.m_destAddr = olsrLinkSet.setData[linkIt].data.m_neighborAddr;
          add.m_distance = 1;
          add.m_nextAddr = olsrLinkSet.setData[linkIt].data.m_neighborAddr;
          olsrRoutingSetInsert(&tmpRoutingSet,&add);
        }
        linkIt = olsrLinkSet.setData[linkIt].next;
      }
    }
    neighborIt = olsrNeighborSet.setData[neighborIt].next;
  }

  setIndex_t neighbor2It = olsrTwoHopNeighborSet.fullQueueEntry;
  while(neighbor2It != -1)
  {
    bool is_n1 = false;
    bool corresponds = false;

    setIndex_t neighborIt = olsrNeighborSet.fullQueueEntry;
    while(neighborIt != -1)
    {
      if(olsrNeighborSet.setData[neighborIt].data.m_neighborAddr == olsrTwoHopNeighborSet.setData[neighbor2It].data.m_twoHopNeighborAddr)
      {
        is_n1 = true;
        break;
      }
      if(olsrNeighborSet.setData[neighborIt].data.m_neighborAddr == olsrTwoHopNeighborSet.setData[neighbor2It].data.m_neighborAddr 
      && olsrNeighborSet.setData[neighborIt].data.m_willingness != WILL_NEVER)
      {
        corresponds = true;
      }
      neighborIt = olsrNeighborSet.setData[neighborIt].next;
    }
    if(is_n1)
      continue;

    if(!corresponds)
      continue;


    olsrRoutingTuple_t* route = NULL;
    setIndex_t routingIt = tmpRoutingSet.fullQueueEntry;
    while(routingIt != -1)
    {
      // Avoid processing twice the same...
      if(tmpRoutingSet.setData[routingIt].data.m_destAddr == olsrTwoHopNeighborSet.setData[neighbor2It].data.m_twoHopNeighborAddr)
      {
        route = NULL;
        break;
      }

      if(tmpRoutingSet.setData[routingIt].data.m_destAddr == olsrTwoHopNeighborSet.setData[neighbor2It].data.m_neighborAddr)
      {
        route = (olsrRoutingTuple_t*)(&(tmpRoutingSet.setData[routingIt].data));
      }
      routingIt = tmpRoutingSet.setData[routingIt].next;
    }

    if(!route)
      continue;

    olsrRoutingTuple_t add;
    add.m_destAddr = olsrTwoHopNeighborSet.setData[neighbor2It].data.m_twoHopNeighborAddr;
    add.m_nextAddr = route->m_nextAddr;
    add.m_distance = 2;

    olsrRoutingSetInsert(&tmpRoutingSet,&add);

    neighbor2It = olsrTwoHopNeighborSet.setData[neighbor2It].next;

  }


  int h = 2;
  bool something_inserted = true;
  while(something_inserted)
  {
    const int old_size = tmpRoutingSet.size;


    setIndex_t topologyIt = olsrTopologySet.fullQueueEntry;
    while(topologyIt != -1)
    {
      bool exist1 = false;

      setIndex_t routingIt = tmpRoutingSet.fullQueueEntry;
      while(routingIt != -1)
      {
        if(tmpRoutingSet.setData[routingIt].data.m_destAddr == olsrTopologySet.setData[topologyIt].data.m_destAddr)
        {
          exist1 = true;
          break;
        }
        routingIt = tmpRoutingSet.setData[routingIt].next;
      }

      if(exist1)
        continue;

      olsrRoutingTuple_t* route = NULL;
      bool exist2 = false;
      //setIndex_t routingIt = tmpRoutingSet.fullQueueEntry;
      while(routingIt != -1)
      {
        if(tmpRoutingSet.setData[routingIt].data.m_destAddr == olsrTopologySet.setData[topologyIt].data.m_lastAddr
        && tmpRoutingSet.setData[routingIt].data.m_distance == h)
        {
          route = (olsrRoutingTuple_t*)&(tmpRoutingSet.setData[routingIt].data);
          exist2 = true;
          break;
        }
        routingIt = tmpRoutingSet.setData[routingIt].next;
      }

      if(!exist2 || !route)
        continue;

      olsrRoutingTuple_t add;
      add.m_destAddr = olsrTopologySet.setData[topologyIt].data.m_destAddr;
      add.m_nextAddr = route->m_nextAddr;
      add.m_distance = h + 1;

      olsrRoutingSetInsert(&tmpRoutingSet,&add);
      topologyIt = olsrTopologySet.setData[topologyIt].next;
    }


    h++;
    something_inserted = tmpRoutingSet.size > old_size;
  
  }
  olsrRoutingSetCopy(&olsrRoutingSet,&tmpRoutingSet);
}
void olsrRoutingTableComputation()
{
  olsrRoutingSet_t tmpRoutingSet;
  olsrRoutingSetInit(&tmpRoutingSet);

  olsrTime_t now = xTaskGetTickCount();
  olsrRoutingTuple_t self;
  self.m_destAddr = myAddress;
  self.m_distance = 0;
  self.m_nextAddr = myAddress;
  self.m_expirationTime = now + OLSR_ROUTING_SET_HOLD_TIME;
  olsrRoutingSetInsert(&tmpRoutingSet,&self);

  setIndex_t neighborIt = olsrNeighborSet.fullQueueEntry;
  while(neighborIt != -1)
    {
      self.m_destAddr = olsrNeighborSet.setData[neighborIt].data.m_neighborAddr;
      self.m_nextAddr = olsrNeighborSet.setData[neighborIt].data.m_neighborAddr;
      self.m_distance = 1;
      olsrRoutingSetInsert(&tmpRoutingSet,&self);
      neighborIt = olsrNeighborSet.setData[neighborIt].next;
    }
  bool somethingChanged = true;

  
  while(somethingChanged)
    { 
      somethingChanged = false;  
      setIndex_t topologyIt = olsrTopologySet.fullQueueEntry; 
      while(topologyIt != -1)
        {
          olsrTopologySetItem_t tmpTc = olsrTopologySet.setData[topologyIt];
          
          setIndex_t routingIt = tmpRoutingSet.fullQueueEntry;
          setIndex_t routeCanLinkToTc = -1;
          bool isFound = false;
          olsrDist_t length = 0;
          while(routingIt != -1)
            {
              if(tmpRoutingSet.setData[routingIt].data.m_destAddr == tmpTc.data.m_lastAddr)
                {
                  isFound = true;
                  length = tmpRoutingSet.setData[routingIt].data.m_distance + tmpTc.data.m_distance;
                  routeCanLinkToTc = routingIt;
                  break;
                }
              routingIt = tmpRoutingSet.setData[routingIt].next;
            }
          if(!isFound)
            {
              topologyIt = tmpTc.next;
              continue;
            }
          routingIt = tmpRoutingSet.fullQueueEntry;
          bool isFoundOldRoute = false;
          while(routingIt != -1)
            {
              if(tmpRoutingSet.setData[routingIt].data.m_destAddr == tmpTc.data.m_destAddr)
                {
                  isFoundOldRoute = true;
                  if(tmpRoutingSet.setData[routingIt].data.m_distance > length)
                    {
                      tmpRoutingSet.setData[routingIt].data.m_distance = length;
                      tmpRoutingSet.setData[routingIt].data.m_nextAddr = tmpRoutingSet.setData[routeCanLinkToTc].data.m_nextAddr;
                      somethingChanged = true;
                    }
                  break;
                }
              routingIt = tmpRoutingSet.setData[routingIt].next;
            }
          if(!isFoundOldRoute)
            {
              self.m_destAddr = tmpTc.data.m_destAddr;
              self.m_distance = length;
              self.m_nextAddr = tmpRoutingSet.setData[routeCanLinkToTc].data.m_nextAddr;
              olsrRoutingSetInsert(&tmpRoutingSet,&self);
              somethingChanged = true;
            }
          topologyIt = tmpTc.next;
        }
    }
    olsrRoutingSetCopy(&olsrRoutingSet,&tmpRoutingSet);
}
void olsrPacketDispatch(const packetWithTimestamp_t * rxPacketWts)
{
  DEBUG_PRINT_OLSR_HELLO("PACKET_DISPATCH\n");  
  //need to add a condition whether recvive a packet from self
  olsrTimestampTuple_t *rx_otimestamp = &rxPacketWts->ots;
  olsrPacket_t *olsrPacket = (olsrPacket_t *) rxPacket->payload;
  int lengthOfPacket = olsrPacket->m_packetHeader.m_packetLength;
  int index = sizeof(olsrPacket->m_packetHeader);
  void *message = (void *)olsrPacket->m_packetPayload;

  xSemaphoreTake(olsrAllSetLock,portMAX_DELAY);
  olsrSetExpire();
  while(index<lengthOfPacket)
    {
      olsrMessageHeader_t* messageHeader = (olsrMessageHeader_t*)message;
      olsrMessageType_t type = messageHeader->m_messageType;

      #ifdef OLSR_SIM
      if(checkItCanReceive(messageHeader->m_relayAddress,myAddress)!=1)
        {
          DEBUG_PRINT_OLSR_SIM("%d to %d is can not accept\n",messageHeader->m_relayAddress,myAddress);
          xSemaphoreGive(olsrAllSetLock);
          return ;
        }
        DEBUG_PRINT_OLSR_SIM("%d to %d is can  accept\n",messageHeader->m_relayAddress,myAddress);
      #endif

      if(type!=TS_MESSAGE&&(messageHeader->m_originatorAddress == myAddress ||messageHeader->m_timeToLive ==0))
        {
          index += messageHeader->m_messageSize;
          message += messageHeader->m_messageSize;
          continue;
        }
      bool doForward = true;
      setIndex_t duplicated = olsrFindInDuplicateSet(&olsrDuplicateSet,messageHeader->m_originatorAddress,\
                                                     messageHeader->m_messageSeq);
      if(duplicated == -1)
        {
          switch (type) 
            {
            case HELLO_MESSAGE:
                DEBUG_PRINT_OLSR_HELLO("recv a hello\n");
                olsrProcessHello((olsrMessage_t*)message);
                break;
            case TC_MESSAGE:
                DEBUG_PRINT_OLSR_RECEIVE("recv a TC\n");
                olsrProcessTc((olsrMessage_t*)message);
                break;
            case DATA_MESSAGE:
                DEBUG_PRINT_OLSR_RECEIVE("DATA_MESSAGE\n");
                olsrProcessData((olsrMessage_t*)message);
                break;
            case TS_MESSAGE:
                DEBUG_PRINT_OLSR_RECEIVE("TS_MESSAGE\n");
                olsrProcessTs((olsrMessage_t*)message, rx_otimestamp);
                break;
            default:
                DEBUG_PRINT_OLSR_RECEIVE("WRONG MESSAGE\n");
                break;
            }
        }
      else
        {
          doForward = false;
        }

      if(doForward)
        {
          if(type == TC_MESSAGE)
            {
              forwardDefault((olsrMessage_t *)message,duplicated);
            }
        }

      index += messageHeader->m_messageSize;
      message += messageHeader->m_messageSize;
    }
    olsrRoutingTableComputation();
	  // olsrRoutingTableComputation2();
  	xSemaphoreGive(olsrAllSetLock);
    
}

/**
 * @brief this funcition is used to send Hello Message
 * 
 **/

void olsrSendHello()
{
  olsrMessage_t msg; //100
  //message header initial
  memset(&msg,0,sizeof(msg));
  msg.m_messageHeader.m_messageType = HELLO_MESSAGE;
  msg.m_messageHeader.m_vTime = OLSR_NEIGHB_HOLD_TIME;
  msg.m_messageHeader.m_messageSize = sizeof(olsrMessageHeader_t);
  msg.m_messageHeader.m_originatorAddress = myAddress;
  msg.m_messageHeader.m_destinationAddress = 0;
  msg.m_messageHeader.m_relayAddress = myAddress;  
  msg.m_messageHeader.m_timeToLive = 0xff;
  msg.m_messageHeader.m_hopCount = 0;
  msg.m_messageHeader.m_messageSeq = getSeqNumber();
  //hello message
  olsrHelloMessage_t helloMessage;//84
  helloMessage.m_helloHeader.m_hTime = OLSR_HELLO_INTERVAL; //hello's header on packet
  helloMessage.m_helloHeader.m_willingness = WILL_ALWAYS;
  helloMessage.m_helloHeader.m_linkMessageNumber = 0;

  //loop
  setIndex_t linkTupleIndex = olsrLinkSet.fullQueueEntry;//2
  olsrTime_t now = xTaskGetTickCount();//4
  while(linkTupleIndex!=-1)
    {
      if(!(olsrLinkSet.setData[linkTupleIndex].data.m_localAddr == myAddress &&\
      olsrLinkSet.setData[linkTupleIndex].data.m_expirationTime >= now))
        {
          linkTupleIndex = olsrLinkSet.setData[linkTupleIndex].next;
          continue;
        }
      uint8_t linkType, nbType = 0xff;

      if(olsrLinkSet.setData[linkTupleIndex].data.m_symTime>=now)
        {
          linkType = OLSR_SYM_LINK;//2
        }
      else if(olsrLinkSet.setData[linkTupleIndex].data.m_asymTime>=now)
        {
          linkType = OLSR_ASYM_LINK;//1
        }
      else
        {
          linkType = OLSR_LOST_LINK;//3
        }
      if(olsrFindMprByAddr(&olsrMprSet, olsrLinkSet.setData[linkTupleIndex].data.m_neighborAddr))//reg+3.4
        {
           nbType = OLSR_MPR_NEIGH;//2
        }
      else
        {
          bool ok = false;
          setIndex_t neighborTupleIndex = olsrNeighborSet.fullQueueEntry;
          while(neighborTupleIndex!=-1)
            {
              if(olsrNeighborSet.setData[neighborTupleIndex].data.m_neighborAddr ==\
              olsrLinkSet.setData[linkTupleIndex].data.m_neighborAddr) // this linkTuple Addr is in NighborSet
                {
                  if(olsrNeighborSet.setData[neighborTupleIndex].data.m_status == STATUS_SYM)
                    {
                      nbType = OLSR_SYM_NEIGH; //is a sym neighbor 1
                    }
                  else if(olsrNeighborSet.setData[neighborTupleIndex].data.m_status == STATUS_NOT_SYM)
                    {
                      nbType = OLSR_NOT_NEIGH; // is not a sym neghbor 0
                    }
                  else
                    {
                      DEBUG_PRINT_OLSR_HELLO("There is a neighbor tuple with an unknown status!\n");
                    }
                  ok = true;
                  break;
                }
              neighborTupleIndex = olsrNeighborSet.setData[neighborTupleIndex].next;
            }
          if(!ok)
            {
              linkTupleIndex = olsrLinkSet.setData[linkTupleIndex].next;
              continue;
            }
        } //TODO -1 in queue will be not dropped
        olsrLinkMessage_t linkMessage;//6
        linkMessage.m_linkCode = (linkType & 0x03) | ((nbType << 2) & 0x0f);
        linkMessage.m_addressUsedSize = 1;
        linkMessage.m_addresses = olsrLinkSet.setData[linkTupleIndex].data.m_neighborAddr;
        if(helloMessage.m_helloHeader.m_linkMessageNumber==LINK_MESSAGE_MAX_NUM) break;
        helloMessage.m_linkMessage[helloMessage.m_helloHeader.m_linkMessageNumber++] = linkMessage;
        linkTupleIndex = olsrLinkSet.setData[linkTupleIndex].next;
    }
  uint16_t writeSize = sizeof(olsrHelloMessageHeader_t)+helloMessage.m_helloHeader.m_linkMessageNumber*\
                       sizeof(olsrLinkMessage_t);
  msg.m_messageHeader.m_messageSize +=  writeSize;                 
  memcpy(msg.m_messagePayload,&helloMessage,writeSize);
  xQueueSend(g_olsrSendQueue,&msg,portMAX_DELAY);
}


void olsrSendTc()
{
  olsrMessage_t msg;
  msg.m_messageHeader.m_messageType = TC_MESSAGE;
  msg.m_messageHeader.m_vTime = OLSR_TOP_HOLD_TIME;
  msg.m_messageHeader.m_messageSize = sizeof(olsrMessageHeader_t);
  msg.m_messageHeader.m_originatorAddress = myAddress;
  msg.m_messageHeader.m_destinationAddress = 0;
  msg.m_messageHeader.m_relayAddress = myAddress;  
  msg.m_messageHeader.m_timeToLive = 0xff;
  msg.m_messageHeader.m_hopCount = 0;
  msg.m_messageHeader.m_messageSeq = getSeqNumber();

  olsrTopologyMessage_t tcMsg;
  tcMsg.m_ansn = getAnsn();
  
  setIndex_t mprSelectorIt = olsrMprSelectorSet.fullQueueEntry;
  uint8_t pos = 0;
  while(mprSelectorIt != -1 && pos<TC_PAYLOAD_MAX_NUM)
    {
      olsrMprSelectorSetItem_t tmp = olsrMprSelectorSet.setData[mprSelectorIt];
      tcMsg.m_content[pos].m_address = tmp.data.m_addr;
      tcMsg.m_content[pos++].m_distance = 1;
      mprSelectorIt = tmp.next;
    }
  memcpy(msg.m_messagePayload,&tcMsg,2+pos*sizeof(olsrTopologyMessageUint_t));
  msg.m_messageHeader.m_messageSize+=(2+pos*sizeof(olsrTopologyMessageUint_t));
  xQueueSend(g_olsrSendQueue,&msg,portMAX_DELAY);
}

void olsrSendData(olsrAddr_t sourceAddr,AdHocPort sourcePort,\
                  olsrAddr_t destAddr, AdHocPort destPort,\
                  uint16_t portSeq, uint8_t data[],uint8_t length)
{
  if(length>DATA_PAYLOAD_MAX_NUM) return;

  olsrMessage_t msg;
  msg.m_messageHeader.m_messageType = DATA_MESSAGE;
  msg.m_messageHeader.m_vTime = OLSR_TOP_HOLD_TIME;
  msg.m_messageHeader.m_messageSize = sizeof(olsrMessageHeader_t);
  msg.m_messageHeader.m_originatorAddress = sourceAddr;
  msg.m_messageHeader.m_destinationAddress = destAddr;

  olsrAddr_t nextHop = olsrFindInRoutingTable(&olsrRoutingSet,destAddr);
  if(nextHop == -1)
    {
      DEBUG_PRINT_OLSR_ROUTING("can not find next hop\n");
      return;
    }
  msg.m_messageHeader.m_relayAddress = myAddress;  
  msg.m_messageHeader.m_timeToLive = 0xff;
  msg.m_messageHeader.m_hopCount = 0;
  msg.m_messageHeader.m_messageSeq = getSeqNumber(); 

  olsrDataMessage_t dataMsg;
  dataMsg.m_dataHeader.m_sourcePort = sourcePort;
  dataMsg.m_dataHeader.m_nextHop = nextHop;
  dataMsg.m_dataHeader.m_destPort =  destPort;
  dataMsg.m_dataHeader.m_seq = portSeq;
  dataMsg.m_dataHeader.m_size = sizeof(olsrDataMessageHeader_t)+length;
  memcpy(dataMsg.m_payload,data,length);
  memcpy(msg.m_messagePayload,&dataMsg,sizeof(olsrDataMessageHeader_t)+length);
  msg.m_messageHeader.m_messageSize+=sizeof(olsrDataMessageHeader_t)+length;

  xQueueSend(g_olsrSendQueue,&msg,portMAX_DELAY);
}

olsrTime_t olsrSendTs() {
  olsrMessage_t tsMsg = {0};
  olsrTime_t nextSendTime = xTaskGetTickCount() + M2T(TS_INTERVAL_MAX) + TS_INTERVAL_MIN;
  olsrTimestampTuple_t *txOTS = olsr_ts_otspool + olsr_ts_otspool_idx;

  // generate header
  olsrTsMessageHeader_t *tsMsgHeader = (olsrTsMessageHeader_t *) &tsMsg.m_messageHeader;
  tsMsgHeader->m_messageType = TS_MESSAGE;
  tsMsgHeader->m_messageSize = sizeof(olsrTsMessageHeader_t);
  tsMsgHeader->m_originatorAddress = myAddress;
  tsMsgHeader->m_messageSeq = getSeqNumber();
  tsMsgHeader->m_dwTimeHigh8 = txOTS->m_timestamp.high8;
  tsMsgHeader->m_dwTimeLow32 = txOTS->m_timestamp.low32;
  tsMsgHeader->m_seq4TSsend = txOTS->m_seqenceNumber;
  float velocityX = logGetFloat(idVelocityX);
  float velocityY = logGetFloat(idVelocityY);
  float velocityZ = logGetFloat(idVelocityZ);
  velocity = sqrt(pow(velocityX,2)+pow(velocityY,2)+pow(velocityZ,2));
  tsMsgHeader->m_velocity = (short) (velocity * 100);

  // generate bodyunit
  uint8_t *msgPtr = (uint8_t *) &tsMsg + sizeof(olsrTsMessageHeader_t);
  uint8_t *msgPtrEnd = (uint8_t *) &tsMsg + MESSAGE_MAX_LENGTH;
  olsrTsMessageBodyUnit_t *tsMsgBodyUnit = (olsrTsMessageBodyUnit_t *) msgPtr;
  for (olsrRangingTableItem_t t = olsrRangingTable.setData[olsrRangingTable.fullQueueEntry]; t.next != -1; t = olsrRangingTable.setData[t.next]) {
    if (t.data.m_nextDeliveryTime <= xTaskGetTickCount() + TS_INTERVAL_MIN && t.data.Re.m_timestamp.full) {
      tsMsgBodyUnit->m_tsAddr = t.data.m_tsAddress;
      tsMsgBodyUnit->m_sequence = t.data.Re.m_seqenceNumber;
      tsMsgBodyUnit->m_dwTimeLow32 = t.data.Re.m_timestamp.low32;
      tsMsgBodyUnit->m_dwTimeHigh8 = t.data.Re.m_timestamp.high8;
      tsMsgHeader->m_messageSize += sizeof(olsrTsMessageBodyUnit_t);
      tsMsgBodyUnit++;
      t.data.Re.m_seqenceNumber = 0;
      t.data.Re.m_timestamp.full = 0;
    }
    jitter = (int) (rand() / (float) RAND_MAX * 9) - 4;// the rand part should not exceed TS_INTERVAL_MIN/2
    jitter = 0;//TODO remove after debug
    t.data.m_nextDeliveryTime = xTaskGetTickCount() + t.data.m_period + jitter;
    if (t.data.m_nextDeliveryTime < nextSendTime) {
      nextSendTime = t.data.m_nextDeliveryTime;
    }
    xQueueSend(g_olsrSendQueue, &tsMsg, portMAX_DELAY);
    return nextSendTime;
  }
}

void olsrNeighborLoss(olsrAddr_t addr[],uint8_t length)
{
  for(int i = 0; i < length; i++)
    {
      olsrEraseTwoHopNeighborTupleByNeighborAddr(&olsrTwoHopNeighborSet,addr[i]);
      olsrEraseMprSelectorTuples(&olsrMprSelectorSet,addr[i]);
    }
  // mprCompute();
}
bool olsrLinkTupleClearExpire() // 
{
   /*
  link->nb->nb2->mpr->mprs
  */
  setIndex_t candidate = olsrLinkSet.fullQueueEntry;
  olsrTime_t now = xTaskGetTickCount();
  bool isChange = false;
  olsrAddr_t expireSymVec[LINK_SET_SIZE];
  for(int i=0;i<LINK_SET_SIZE;i++)
    {
      expireSymVec[i]=0;
    }
  uint8_t length = 0;
  while(candidate != -1)
    {
      olsrLinkSetItem_t tmp = olsrLinkSet.setData[candidate];
      if(tmp.data.m_expirationTime < now)
        {
          olsrDelNeighborByAddr(&olsrNeighborSet,tmp.data.m_neighborAddr);
          setIndex_t delItem = candidate;
          candidate = tmp.next;
          olsrDelLinkTupleByPos(&olsrLinkSet,delItem);
          isChange = true;
          continue;
        }
      else if(tmp.data.m_symTime < now)
        {
          expireSymVec[length++] = tmp.data.m_neighborAddr;
        }
      candidate = tmp.next;
    }
  if(length > 0)
    {
      olsrNeighborLoss(expireSymVec, length);
    }
  return isChange;
}

bool olsrTopologyTupleTimerExpire()
{
  setIndex_t candidate = olsrTopologySet.fullQueueEntry;
  olsrTime_t now = xTaskGetTickCount();
  bool isChange = false;
  while(candidate != -1)
    {
      olsrTopologySetItem_t tmp = olsrTopologySet.setData[candidate];
      if(tmp.data.m_expirationTime < now)
        {
          setIndex_t delItem = candidate;
          candidate = tmp.next;
          olsrDelTopologyTupleByPos(delItem);
          isChange = true;
          continue;
        }
      candidate = tmp.next;
    }
  return isChange;
}

bool olsrNbTwoHopTupleTimerExpire()
{
  setIndex_t candidate = olsrTwoHopNeighborSet.fullQueueEntry;
  olsrTime_t now = xTaskGetTickCount();
  bool isChange = false;
  while(candidate != -1)
    {
      olsrTwoHopNeighborSetItem_t tmp = olsrTwoHopNeighborSet.setData[candidate];
      // DEBUG_PRINT_OLSR_NEIGHBOR2("ex:%ld,now:%ld\n",tmp.data.m_expirationTime,now);
      if(tmp.data.m_expirationTime < now)
        {
          setIndex_t delItem = candidate;
          candidate = tmp.next;
          olsrDelTwoHopNeighborTupleByPos(&olsrTwoHopNeighborSet,delItem);
          isChange = true;
          continue;
        }
      candidate = tmp.next;
    }
  // DEBUG_PRINT_OLSR_NEIGHBOR2("clear in expire\n");
  if(isChange){
    // DEBUG_PRINT_OLSR_NEIGHBOR2("cleared morethan one tuple\n");
  }
  return isChange; 
}

bool olsrMprSelectorTupleTimerExpire()
{
  setIndex_t candidate = olsrMprSelectorSet.fullQueueEntry;
  olsrTime_t now = xTaskGetTickCount();
  bool isChange = false;
  while(candidate != -1)
    {
      olsrMprSelectorSetItem_t tmp = olsrMprSelectorSet.setData[candidate];
      if(tmp.data.m_expirationTime < now)
        {
          setIndex_t delItem = candidate;
          candidate = tmp.next;
          olsrDelMprSelectorTupleByPos(&olsrMprSelectorSet,delItem);
          isChange = true;
          continue;
        }
      candidate = tmp.next;
    }
  return isChange;   
}
// all task defination
void olsrHelloTask(void *ptr)
{
  while (true)
  {
      /* code */
      vTaskDelay(M2T(OLSR_HELLO_INTERVAL));
      xSemaphoreTake(olsrAllSetLock,portMAX_DELAY);
      olsrLinkTupleClearExpire();
      olsrSendHello();
      xSemaphoreGive(olsrAllSetLock);
  }
}

void olsrTcTask(void *ptr)
{
  while(true)
  {
    xSemaphoreTake(olsrAllSetLock,portMAX_DELAY);
    if(!olsrMprSelectorSetIsEmpty())
      {
        olsrSendTc();
        DEBUG_PRINT_OLSR_TC("Send TC yes\n");
      }
    else
      {
        DEBUG_PRINT_OLSR_TC("Not sending any TC, no one selected me as MPR.\n");
      }
    xSemaphoreGive(olsrAllSetLock);
    vTaskDelay(M2T(OLSR_TC_INTERVAL));
  }
}


// void olsrSendTask(void *ptr)
// {
//   packet_t txPacket = {0};
//   MAC80215_PACKET_INIT(txPacket, MAC802154_TYPE_OLSR);
//   dwm = (dwDevice_t *)ptr;
//   olsrPacket_t olsrPacket = {0};
//   olsrMessage_t olsrMessageCache = {0};
//   while(true)
//     {
//       if(xQueueReceive(g_olsrSendQueue, &olsrMessageCache, 0)==pdTRUE)
//         {
//           olsrPacket.m_packetHeader.m_packetLength = sizeof(olsrPacket.m_packetHeader)+olsrMessageCache.m_messageHeader.m_messageSize;
//           memcpy(&olsrPacket.m_packetPayload,&olsrMessageCache,sizeof(olsrMessageCache));
//           memcpy(&txPacket.payload,&olsrPacket,sizeof(olsrPacket));
//       //transmit
//           dwNewTransmit(dwm);
//           dwSetDefaults(dwm);
//           dwWaitForResponse(dwm, true);
//           dwReceivePermanently(dwm, true);
//           dwSetData(dwm, (uint8_t *)&txPacket,MAC802154_HEADER_LENGTH+olsrPacket.m_packetHeader.m_packetLength);
//           dwStartTransmit(dwm);
//         }
//       vTaskDelay(500);
//     }
// }
void olsrPacketLossTask(void *ptr)
{
  //packet_t txPacket = {0};
  MAC80215_PACKET_INIT(txpacket, MAC802154_TYPE_OLSR);
  int countSend = 1;
  while(1)
    {

      dwNewTransmit(dwm);
      dwSetDefaults(dwm);
      dwWaitForResponse(dwm, true);
      dwReceivePermanently(dwm, true);
      memcpy(txpacket.payload,&countSend,sizeof(int));
      dwSetData(dwm, (uint8_t *)&txpacket,MAC802154_HEADER_LENGTH+4);
      dwStartTransmit(dwm);
      DEBUG_PRINT_OLSR_SEND("send %d packet\n",countSend);
      countSend++;
      vTaskDelay(50);
    }  
}

void olsrTsTask(void *ptr) {
  idVelocityX = logGetVarId("stateEstimate", "vx");
  idVelocityY = logGetVarId("stateEstimate", "vy");
  idVelocityZ = logGetVarId("stateEstimate", "vz");
  while (true) {
    xSemaphoreTake(olsrAllSetLock, portMAX_DELAY);
    olsrTime_t nextSendTime = olsrSendTs();
    olsrTime_t currentTime = xTaskGetTickCount();
    if (nextSendTime < currentTime + M2T(TS_INTERVAL_MIN)) {
      nextSendTime = currentTime + M2T(TS_INTERVAL_MIN);
    }
    xSemaphoreGive(olsrAllSetLock);
    vTaskDelay(nextSendTime - currentTime);
  }
}

void olsrSendTask(void *ptr)
{
    bool hasOlsrMessageCache =false;
		TickType_t timeToWaitForSendQueue;
    olsrMessage_t olsrMessageCache = {0};
    //packet_t txPacket = {0};
  	MAC80215_PACKET_INIT(txpacket, MAC802154_TYPE_OLSR);
    dwm = (dwDevice_t *)ptr;
    olsrPacket_t *olsrPacket = (olsrPacket_t*)txpacket.payload;
    //task loop
    while(true){
      uint8_t *writePosition = olsrPacket->m_packetPayload;
		  timeToWaitForSendQueue = portMAX_DELAY;
      while(hasOlsrMessageCache || xQueueReceive(g_olsrSendQueue, &olsrMessageCache, timeToWaitForSendQueue)){
				timeToWaitForSendQueue = 0;
        hasOlsrMessageCache = false;
        configASSERT(olsrMessageCache.m_messageHeader.m_messageSize <= MESSAGE_MAX_LENGTH);
        if(olsrMessageCache.m_messageHeader.m_messageType!=TS_MESSAGE && 0==olsrMessageCache.m_messageHeader.m_timeToLive) 
          continue;
        if((writePosition-olsrPacket->m_packetPayload)+olsrMessageCache.m_messageHeader.m_messageSize>MESSAGE_MAX_LENGTH){
          hasOlsrMessageCache = true;
          break;
        }
        memcpy(writePosition,&olsrMessageCache,olsrMessageCache.m_messageHeader.m_messageSize);
        writePosition += olsrMessageCache.m_messageHeader.m_messageSize;
				if(olsrMessageCache.m_messageHeader.m_messageType==TS_MESSAGE) 
						txpacket.fcf_s.reserved = 1;
      }
      ASSERT(writePosition-(uint8_t *)olsrPacket>sizeof(olsrPacketHeader_t));
      olsrPacket->m_packetHeader.m_packetLength = writePosition-(uint8_t *)olsrPacket;
      olsrPacket->m_packetHeader.m_packetSeq = getSeqNumber();
      //transmit
      dwNewTransmit(dwm);
      dwSetDefaults(dwm);
      dwWaitForResponse(dwm, true);
      dwReceivePermanently(dwm, true);
      dwSetData(dwm, (uint8_t *)&txpacket,MAC802154_HEADER_LENGTH+olsrPacket->m_packetHeader.m_packetLength);
      dwStartTransmit(dwm);
      DEBUG_PRINT_OLSR_SEND("PktSend!Len:%d\n",MAC802154_HEADER_LENGTH+olsrPacket->m_packetHeader.m_packetLength);
    }
}
// packet_t dwPacket = {0};
// bool hasMessageCache = false;
// olsrMessage_t olsrMessageCache = {0};
// void olsrSendTask(void *ptr)
// {
//     //pointer initialize 
//     //task loop
//   memset(&olsrMessageCache,0,sizeof(olsrMessage_t));
//   while(true)
//     { 
//       memset(&dwPacket,0,sizeof(packet_t));
//       MAC80215_PACKET_INIT(dwPacket, MAC802154_TYPE_OLSR);
//       olsrPacket_t *olsrPacket = (olsrPacket_t *)dwPacket.payload;
//       olsrMessage_t *messages = (olsrMessage_t *)(olsrPacket->m_packetPayload);
//       uint8_t *writePosition = (uint8_t *)messages;
//       if(hasMessageCache)
//         {
//             configASSERT(olsrMessageCache.m_messageHeader.m_messageSize <= MESSAGE_MAX_LENGTH);
//             memcpy(writePosition,&olsrMessageCache,olsrMessageCache.m_messageHeader.m_messageSize);
//             writePosition += olsrMessageCache.m_messageHeader.m_messageSize;
//             hasMessageCache = false;
//         }
//       while(xQueueReceive(g_olsrSendQueue, &olsrMessageCache, 0))
//         {
//           configASSERT(olsrMessageCache.m_messageHeader.m_messageSize <= MESSAGE_MAX_LENGTH);
//           if(0==olsrMessageCache.m_messageHeader.m_timeToLive) break;
//           if(writePosition+olsrMessageCache.m_messageHeader.m_messageSize-(uint8_t *)messages>MESSAGE_MAX_LENGTH)
//             {
//               hasMessageCache = true;
//               DEBUG_PRINT_OLSR_SEND("break\n");
//               break;
//             }
//           else
//             {
//               memcpy(writePosition,&olsrMessageCache,olsrMessageCache.m_messageHeader.m_messageSize);
//               writePosition += (uint8_t)olsrMessageCache.m_messageHeader.m_messageSize;
//             }
//         }
//       if(writePosition-(uint8_t *)olsrPacket==sizeof(olsrPacketHeader_t))
//         {
//           vTaskDelay(200);
//           continue;
//         }
//       olsrPacket->m_packetHeader.m_packetLength = (uint16_t)(writePosition-(uint8_t *)olsrPacket);
//           // olsrPacket_t *p = (olsrPacket_t *)dwPacket.payload;
//           // olsrMessage_t *q = (olsrMessage_t *)p->m_packetPayload;
//           // olsrHelloMessage_t *k = (olsrHelloMessage_t *)q->m_messagePayload;
//           // DEBUG_PRINT_OLSR_SEND("last send legth %d\n",p->m_packetHeader.m_packetLength);
//           // DEBUG_PRINT_OLSR_SEND("last send message length %d\n",q->m_messageHeader.m_messageSize);
//           // DEBUG_PRINT_OLSR_SEND("last send HelloMsg usd leth %d\n",k->m_helloHeader.m_linkMessageNumber);
//           //transmit
//       DEBUG_PRINT_OLSR_SEND("when SEND\n");
//       olsrPrintPacket(&dwPacket);
//       dwNewTransmit(dwm);
//       dwSetDefaults(dwm);
//       dwWaitForResponse(dwm, true);
//       dwReceivePermanently(dwm, true);
//       dwSetData(dwm, (uint8_t *)&dwPacket,MAC802154_HEADER_LENGTH+olsrPacket->m_packetHeader.m_packetLength);
//       dwStartTransmit(dwm);
//       vTaskDelay(200);
//       DEBUG_PRINT_OLSR_SEND("send successful\n");
//     }
// }
void olsrRecvTask(void *ptr) {
  DEBUG_PRINT_OLSR_RECEIVE("RECV TASK START\n");
  //packet_t recvPacket;
  while (true) {
//    // DEBUG_PRINT_OLSR_RECEIVE("to take a packet from q\n");
//    if (xQueueReceive(g_olsrRecvQueue, &recvPacket, 0) == pdTRUE) {
//      // DEBUG_PRINT_OLSR_RECEIVE("got a packet from q\n");
//      olsrPacketDispatch(&recvPacket);
//    }
//    vTaskDelay(50);

    // DEBUG_PRINT_OLSR_RECEIVE("to take a packet from q\n");
    if (xQueueReceive(g_olsrRecvQueue, &rxPacketWts, 0) == pdTRUE) {
      // DEBUG_PRINT_OLSR_RECEIVE("got a packet from q\n");
      olsrPacketDispatch(&rxPacketWts);
    }
    vTaskDelay(50);
  }
}
