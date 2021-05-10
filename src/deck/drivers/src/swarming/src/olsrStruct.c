#define DEBUG_MODULE "OLSR"

#include "olsrStruct.h"
#include <semphr.h>

#include "mac.h"
#include "olsrDebug.h"
#include "olsrPacket.h"



/*
************************QueueInitFunctions********************
*/


/*
************************TopologySetFunctions********************
*/


static void olsrTopologySetInit(olsrTopologySet_t *topologySet)
{
  setIndex_t i;
  for(i=0; i < TOPOLOGY_SET_SIZE-1; i++)
    {
      topologySet->setData[i].next = i+1;
    }
  topologySet->setData[i].next = -1;
  topologySet->freeQueueEntry = 0;
  topologySet->fullQueueEntry = -1;
}

static setIndex_t olsrTopologySetMalloc(olsrTopologySet_t *topologySet)
{
  if(topologySet->freeQueueEntry==-1)
    {
      DEBUG_PRINT_OLSR_SET("Full of sets!!!! can not malloc!!!\n");
      return -1;
    }
  else
    { 
      setIndex_t candidate = topologySet->freeQueueEntry;
      topologySet->freeQueueEntry = topologySet->setData[candidate].next;
      //insert to full queue
      setIndex_t tmp = topologySet->fullQueueEntry;
      topologySet->fullQueueEntry = candidate;
      topologySet->setData[candidate].next = tmp;
      return candidate;
    }
}

static bool olsrTopologySetFree(olsrTopologySet_t *topologySet,\
                                setIndex_t delItem)
{
  if(-1==delItem) return true;
  //del from full queue
  setIndex_t pre = topologySet->fullQueueEntry;
  if(delItem == pre)
    {
      topologySet->fullQueueEntry = topologySet->setData[pre].next;
      //insert to empty queue
      topologySet->setData[delItem].next = topologySet->freeQueueEntry;
      topologySet->freeQueueEntry = delItem;
      return true;
    }
  else 
    {
      while(pre!=-1)
        {
          if(topologySet->setData[pre].next==delItem)
            {
              topologySet->setData[pre].next = topologySet->setData[delItem].next;
              //insert to empty queue
              topologySet->setData[delItem].next = topologySet->freeQueueEntry;
              topologySet->freeQueueEntry = delItem;
              return true;
            }
          pre = topologySet->setData[pre].next;
        }
    }
    return false;
}

setIndex_t olsrInsertToTopologySet(olsrTopologySet_t *topologySet,\
                             const olsrTopologyTuple_t *tcTuple)
{
  setIndex_t candidate = olsrTopologySetMalloc(topologySet); 
  if(candidate != -1)
    {
      memcpy(&topologySet->setData[candidate].data,tcTuple,sizeof(olsrTopologyTuple_t));
    }
  else
    {
      DEBUG_PRINT_OLSR_TC("bad alloc in function[olsrInsertToTopologySet]\n");
    }
  return candidate;
}


void olsrDelTopologyTupleByPos(setIndex_t pos)
{
  olsrTopologySetFree(&olsrTopologySet ,pos);
}

setIndex_t olsrFindNewerTopologyTuple(olsrTopologySet_t *topologyset,\
                                      olsrAddr_t originator,\
                                      uint16_t ansn)
{
  setIndex_t candidate = topologyset->fullQueueEntry;
  while(candidate != -1)
    {
      olsrTopologySetItem_t item = topologyset->setData[candidate];
      if(item.data.m_lastAddr == originator && item.data.m_seqenceNumber > ansn)
        {
          break;
        }
      candidate = item.next;
    }
  return candidate;
}

void olsrEraseOlderTopologyTuples(olsrTopologySet_t *topologyset,\
                                  olsrAddr_t originator,\
                                  uint16_t ansn)
{
  setIndex_t it = topologyset->fullQueueEntry;
  while(it != -1)
    {
      olsrTopologySetItem_t item = topologyset->setData[it];
      if(item.data.m_lastAddr == originator && item.data.m_seqenceNumber < ansn)
        {
          setIndex_t del = it;
          it = item.next;
          olsrTopologySetFree(topologyset,del);
          continue;
        }
      it = item.next;
    }
}

setIndex_t olsrFindTopologyTuple(olsrTopologySet_t *topologyset,\
                                 olsrAddr_t destAddr,\
                                 olsrAddr_t lastAddr)
{
  setIndex_t candidate = topologyset->fullQueueEntry;
  while(candidate != -1)
    {
      olsrTopologySetItem_t item = topologyset->setData[candidate];
      if(item.data.m_lastAddr == lastAddr && item.data.m_destAddr == destAddr)
        {
          break;
        }
      candidate = item.next;
    }
  return candidate;
}

void olsrPrintTopologySet(olsrTopologySet_t *topologyset)
{
  setIndex_t candidate = topologyset->fullQueueEntry;
  while(candidate != -1)
    {
      olsrTopologySetItem_t tmp = topologyset->setData[candidate];
      DEBUG_PRINT_OLSR_TOPOLOGY("TOPOLOGYSET: last:%d,dest:%d,weight:%f\n",tmp.data.m_lastAddr,tmp.data.m_destAddr,tmp.data.m_weight);
      candidate = tmp.next;
    }
}
/*
************************LinkSetFunctions********************
*/

static void olsrLinkSetInit(olsrLinkSet_t *linkSet)
{
  setIndex_t i;
  for(i=0; i < LINK_SET_SIZE-1; i++)
    {
      linkSet->setData[i].next = i+1;
    }
  linkSet->setData[i].next = -1;
  linkSet->freeQueueEntry = 0;
  linkSet->fullQueueEntry = -1;
}
/**
 * @brief 
 * @return
 **/
static setIndex_t olsrLinkSetMalloc(olsrLinkSet_t *linkSet)
{
  // xSemaphoreTake(olsrLinkEmptySetLock, portMAX_DELAY);
  if(linkSet->freeQueueEntry ==-1)
    {
      DEBUG_PRINT_OLSR_SET("Full of sets!!!! can not malloc!!!\n");
      // xSemaphoreGive(olsrLinkEmptySetLock);
      return -1;
    }
  else
    { 
      setIndex_t candidate = linkSet->freeQueueEntry;   // 0 2 3 5 6 -1 freequeentey
      linkSet->freeQueueEntry = linkSet->setData[candidate].next; //  1 4 -1 fullqueueentry
      // xSemaphoreGive(olsrLinkEmptySetLock);
      //insert tlinkSet->
      // xSemaphoreTake(olsrLinkFullSetLock, portMAX_DELAY);
      setIndex_t tmp = linkSet->fullQueueEntry; // -1
      linkSet->fullQueueEntry = candidate; // 0 -1
      linkSet->setData[candidate].next = tmp;
      // xSemaphoreGive(olsrLinkFullSetLock);
      return candidate;
    }
}

static bool olsrLinkSetFree(olsrLinkSet_t *linkSet,setIndex_t delItem)
{
  if(-1==delItem) return true;
  //del from full queue
  // xSemaphoreTake(olsrLinkFullSetLock, portMAX_DELAY);
  setIndex_t pre = linkSet->fullQueueEntry;
  if(delItem == pre)
    {
      linkSet->fullQueueEntry =linkSet->setData[pre].next;
      // xSemaphoreGive(olsrLinkFullSetLock);
      //insert to empty queue
      // xSemaphoreTake(olsrLinkEmptySetLock,portMAX_DELAY);
      linkSet->setData[delItem].next = linkSet->freeQueueEntry;
      linkSet->freeQueueEntry = delItem;
      // xSemaphoreGive(olsrLinkEmptySetLock);
      return true;
    }
  else 
    {
      while(pre!=-1)
        {
          if(linkSet->setData[pre].next==delItem)
            {
              linkSet->setData[pre].next = linkSet->setData[delItem].next;//next.next
              // xSemaphoreGive(olsrLinkFullSetLock);
              //insert to empty queue
              // xSemaphoreTake(olsrLinkEmptySetLock,portMAX_DELAY);
              linkSet->setData[delItem].next = linkSet->freeQueueEntry;
              linkSet->freeQueueEntry = delItem;
              // xSemaphoreGive(olsrLinkEmptySetLock);
              return true;
            }
          pre = linkSet->setData[pre].next;
        }
    }
    return false;
}
void olsrDelLinkTupleByPos(olsrLinkSet_t *linkset,setIndex_t pos)
{
  olsrLinkSetFree(linkset ,pos);
}
setIndex_t olsrFindInLinkByAddr(olsrLinkSet_t *linkSet,const olsrAddr_t addr)
{
  setIndex_t it = linkSet->fullQueueEntry;
  while(it!=-1)
    {
      if(linkSet->setData[it].data.m_neighborAddr==addr)
        {
          break;
        }
      it = linkSet->setData[it].next;
    }
  return it;
}
setIndex_t olsrInsertToLinkSet(olsrLinkSet_t *linkSet,const olsrLinkTuple_t *item)
{
  setIndex_t candidate = olsrLinkSetMalloc(linkSet);
  if(candidate!=-1)
    {
      memcpy(&linkSet->setData[candidate].data,item,sizeof(olsrLinkTuple_t));
      
    }
  else
    {
      DEBUG_PRINT_OLSR_LINK("bad alloc whan insert by function [olsrInsertToLinkSet]\n");
    }
  return candidate;
}

void olsrPrintLinkSet(olsrLinkSet_t *linkSet)
{
  setIndex_t it = linkSet->fullQueueEntry;
  if(it == -1)
    {
      DEBUG_PRINT_OLSR_LINK("linkSet is empty!\n");
    }
  while(it != -1)
    {
      olsrLinkSetItem_t tmp = linkSet->setData[it];
      DEBUG_PRINT_OLSR_LINK("linkSet: localAddr is %u, neighborAddr is %u ,symtime is %u\n",tmp.data.m_localAddr,tmp.data.m_neighborAddr,tmp.data.m_symTime);
      it = tmp.next;
    }
}

setIndex_t olsrFindSymLinkTuple(olsrLinkSet_t *linkSet,olsrAddr_t sender,olsrTime_t now)
{
  setIndex_t candidate = linkSet->fullQueueEntry;
  DEBUG_PRINT_OLSR_TC("sender is :%d,now is %d\n",sender,now);
  while(candidate != -1)
    {
      olsrLinkSetItem_t tmp = linkSet->setData[candidate];
      if(tmp.data.m_neighborAddr == sender && tmp.data.m_symTime > now)
        {
          break;
        }
      candidate = tmp.next;
    }
  return candidate;
}


/*
************************NeighborSetFunctions********************
*/


void olsrNeighborSetInit(olsrNeighborSet_t *neighborSet)
{
  setIndex_t i;
  for(i=0; i < NEIGHBOR_SET_SIZE-1; i++)
    {
      neighborSet->setData[i].next = i+1;
    }
  neighborSet->setData[i].next = -1;
  neighborSet->freeQueueEntry = 0;
  neighborSet->fullQueueEntry = -1;
}

static setIndex_t olsrNeighborSetMalloc(olsrNeighborSet_t *neighborSet)
{
  // xSemaphoreTake(olsrNeighborEmptySetLock, portMAX_DELAY);
  if(neighborSet->freeQueueEntry==-1)
    {
      DEBUG_PRINT_OLSR_SET("Full of sets!!!! can not malloc!!!\n");
      // xSemaphoreGive(olsrNeighborEmptySetLock);
      return -1;
    }
  else
    { 
      setIndex_t candidate = neighborSet->freeQueueEntry;
      neighborSet->freeQueueEntry = neighborSet->setData[candidate].next;
      // xSemaphoreGive(olsrNeighborEmptySetLock);
      //insert to full queue
      // xSemaphoreTake(olsrNeighborFullSetLock, portMAX_DELAY);
      setIndex_t tmp = neighborSet->fullQueueEntry;
      neighborSet->fullQueueEntry = candidate;
      neighborSet->setData[candidate].next = tmp;
      // xSemaphoreGive(olsrNeighborFullSetLock);
      return candidate;
    }
}

static bool olsrNeighborSetFree(olsrNeighborSet_t *neighborSet, \
                                setIndex_t delItem)
{
  if(-1==delItem) return true;
  //del from full queue
  // xSemaphoreTake(olsrNeighborFullSetLock, portMAX_DELAY);
  setIndex_t pre = neighborSet->fullQueueEntry;
  if(delItem == pre)
    {
      neighborSet->fullQueueEntry = neighborSet->setData[pre].next;
      // xSemaphoreGive(olsrNeighborFullSetLock);
      //insert to empty queue
      // xSemaphoreTake(olsrNeighborEmptySetLock,portMAX_DELAY);
      neighborSet->setData[delItem].next = neighborSet->freeQueueEntry;
      neighborSet->freeQueueEntry = delItem;
      // xSemaphoreGive(olsrNeighborEmptySetLock);
      return true;
    }
  else 
    {
      while(pre!=-1)
        {
          if(neighborSet->setData[pre].next==delItem)
            {
              neighborSet->setData[pre].next = neighborSet->setData[delItem].next;
              // xSemaphoreGive(olsrNeighborFullSetLock);
              //insert to empty queue
              // xSemaphoreTake(olsrNeighborEmptySetLock,portMAX_DELAY);
              neighborSet->setData[delItem].next = neighborSet->freeQueueEntry;
              neighborSet->freeQueueEntry = delItem;
              // xSemaphoreGive(olsrNeighborEmptySetLock);
              return true;
            }
          pre = neighborSet->setData[pre].next;
        }
    }
    return false;
}

bool olsrDelNeighborByAddr(olsrNeighborSet_t *neighborSet,olsrAddr_t addr)
{
  setIndex_t candidate = olsrFindNeighborByAddr(neighborSet,addr);
  if(candidate != -1)
    {
      olsrNeighborSetFree(neighborSet,candidate);
      return true;
    }
  return false;
}

setIndex_t olsrFindNeighborByAddr(olsrNeighborSet_t *neighborSet,\
                                  olsrAddr_t addr)
{
  setIndex_t it = neighborSet->fullQueueEntry;
  while(it!=-1)
    {
      if(neighborSet->setData[it].data.m_neighborAddr==addr)
        {
          break;
        }
      it = neighborSet->setData[it].next;
    }
  return it;
}

setIndex_t olsrInsertToNeighborSet(olsrNeighborSet_t *neighborSet, const olsrNeighborTuple_t* tuple)
{
  setIndex_t candidate = olsrNeighborSetMalloc(neighborSet);
  if(candidate != -1)
    {
      memcpy(&neighborSet->setData[candidate].data,tuple,sizeof(olsrNeighborTuple_t));
    }
  else
    {
      DEBUG_PRINT_OLSR_NEIGHBOR("bad alloc whan insert by function [olsrInsertToNeighborSet]\n");
    }
  return candidate;
}

void olsrPrintNeighborSet(olsrNeighborSet_t *neighborSet)
{
  setIndex_t it = neighborSet->fullQueueEntry;
  if(it == -1)
    {
      DEBUG_PRINT_OLSR_NEIGHBOR("neighborSet is empty\n");
    }
  while(it != -1)
    {
      olsrNeighborSetItem_t tmp = neighborSet->setData[it];
      DEBUG_PRINT_OLSR_NEIGHBOR("neighborSet: neighborAddr is %d, status is %d(not sym is 0, sym is 1)\n",tmp.data.m_neighborAddr,tmp.data.m_status);
      it = tmp.next;
    }
}
/*
************************TwoNeighborSetFunctions********************
*/


void olsrTwoHopNeighborSetInit(olsrTwoHopNeighborSet_t *twoHopNeighborSet)
{
  setIndex_t i;
  for(i=0; i < TWO_HOP_NEIGHBOR_SET_SIZE-1; i++)
    {
      twoHopNeighborSet->setData[i].next = i+1;
    }
  twoHopNeighborSet->setData[i].next = -1;
  twoHopNeighborSet->freeQueueEntry = 0;
  twoHopNeighborSet->fullQueueEntry = -1;
}

static setIndex_t olsrTwoHopNeighborSetMalloc(olsrTwoHopNeighborSet_t *twoHopNeighborSet)
{
  // xSemaphoreTake(olsrNeighborEmptySetLock, portMAX_DELAY);
  if(twoHopNeighborSet->freeQueueEntry==-1)
    {
      DEBUG_PRINT_OLSR_SET("Full of sets!!!! can not malloc!!!\n");
      // xSemaphoreGive(olsrNeighborEmptySetLock);
      return -1;
    }
  else
    { 
      setIndex_t candidate = twoHopNeighborSet->freeQueueEntry;
      twoHopNeighborSet->freeQueueEntry = twoHopNeighborSet->setData[candidate].next;
      // xSemaphoreGive(olsrNeighborEmptySetLock);
      //insert to full queue
      // xSemaphoreTake(olsrNeighborFullSetLock, portMAX_DELAY);
      setIndex_t tmp = twoHopNeighborSet->fullQueueEntry;
      twoHopNeighborSet->fullQueueEntry = candidate;
      twoHopNeighborSet->setData[candidate].next = tmp;
      // xSemaphoreGive(olsrNeighborFullSetLock);
      return candidate;
    }
}

static bool olsrTwoHopNeighborSetFree(olsrTwoHopNeighborSet_t *twoHopNeighborSet,\
                                      setIndex_t delItem)
{
  if(-1==delItem) return true;
  //del from full queue
  // xSemaphoreTake(olsrNeighborFullSetLock, portMAX_DELAY);
  setIndex_t pre = twoHopNeighborSet->fullQueueEntry;
  if(delItem == pre)
    {
      twoHopNeighborSet->fullQueueEntry = twoHopNeighborSet->setData[pre].next;
      // xSemaphoreGive(olsrNeighborFullSetLock);
      //insert to empty queue
      // xSemaphoreTake(olsrNeighborEmptySetLock,portMAX_DELAY);
      twoHopNeighborSet->setData[delItem].next = twoHopNeighborSet->freeQueueEntry;
      twoHopNeighborSet->freeQueueEntry = delItem;
      // xSemaphoreGive(olsrNeighborEmptySetLock);
      return true;
    }
  else 
    {
      while(pre!=-1)
        {
          if(twoHopNeighborSet->setData[pre].next==delItem)
            {
              twoHopNeighborSet->setData[pre].next = twoHopNeighborSet->setData[delItem].next;
              // xSemaphoreGive(olsrNeighborFullSetLock);
              //insert to empty queue
              // xSemaphoreTake(olsrNeighborEmptySetLock,portMAX_DELAY);
              twoHopNeighborSet->setData[delItem].next = twoHopNeighborSet->freeQueueEntry;
              twoHopNeighborSet->freeQueueEntry = delItem;
              // xSemaphoreGive(olsrNeighborEmptySetLock);
              return true;
            }
          pre = twoHopNeighborSet->setData[pre].next;
        }
    }
    return false;
}
setIndex_t olsrFindTwoHopNeighborTuple(olsrTwoHopNeighborSet_t *twoHopNeighborSet,\
                                       olsrAddr_t neighborAddr,\
                                       olsrAddr_t twoHopNeighborAddr)
{
  setIndex_t candidate = twoHopNeighborSet->fullQueueEntry;
  while(candidate != -1)
    {
      if(twoHopNeighborSet->setData[candidate].data.m_neighborAddr == neighborAddr\
      &&twoHopNeighborSet->setData[candidate].data.m_twoHopNeighborAddr == twoHopNeighborAddr)
        {
          break;
        }
      candidate = twoHopNeighborSet->setData[candidate].next;
    }
  return candidate;
}


setIndex_t olsrInsertToTwoHopNeighborSet(olsrTwoHopNeighborSet_t *twoHopNeighborSet,\
                                         const olsrTwoHopNeighborTuple_t* tuple)
{
  setIndex_t candidate = olsrTwoHopNeighborSetMalloc(twoHopNeighborSet);
  if(candidate != -1)
    {
      memcpy(&twoHopNeighborSet->setData[candidate].data,tuple,sizeof(olsrTwoHopNeighborTuple_t));
    }
  else
    {
      DEBUG_PRINT_OLSR_NEIGHBOR2("bad alloc.\n");
    }
  return candidate;
}

setIndex_t olsrEraseTwoHopNeighborTuple(olsrTwoHopNeighborSet_t *twoHopNeighborSet,\
                                  olsrAddr_t neighborAddr, \
                                  olsrAddr_t twoHopNeighborAddr)
{
  setIndex_t candidate = olsrFindTwoHopNeighborTuple(twoHopNeighborSet,neighborAddr,twoHopNeighborAddr);
  setIndex_t nextIt = twoHopNeighborSet->setData[candidate].next;
  if(candidate != -1)
    {
      if(olsrTwoHopNeighborSetFree(twoHopNeighborSet,candidate) != false)
        {
          return nextIt;
        }
    }
  return candidate;
}

void olsrDelTwoHopNeighborTupleByPos(olsrTwoHopNeighborSet_t *twoHopNeighborSet,setIndex_t pos)
{
  olsrTwoHopNeighborSetFree(twoHopNeighborSet ,pos);
}

void olsrEraseTwoHopNeighborTupleByNeighborAddr(olsrTwoHopNeighborSet_t *twoHopNeighborSet,\
                                  olsrAddr_t neighborAddr)
{
  setIndex_t candidate = twoHopNeighborSet->fullQueueEntry;
  while(candidate != -1)
    {
      olsrTwoHopNeighborSetItem_t tmp = twoHopNeighborSet->setData[candidate];
      if(tmp.data.m_neighborAddr == neighborAddr)
        {
          setIndex_t delItem = candidate;
          candidate = tmp.next;
          olsrTwoHopNeighborSetFree(&olsrTwoHopNeighborSet,delItem);
          continue;
        }
      candidate = tmp.next;
    }
}

setIndex_t olsrEraseTwoHopNeighborTupleByTuple(olsrTwoHopNeighborSet_t *twoHopNeighborSet,\
                                        olsrTwoHopNeighborTuple_t *tuple)
{
  setIndex_t candidate = olsrFindTwoHopNeighborTuple(twoHopNeighborSet,tuple->m_neighborAddr,tuple->m_twoHopNeighborAddr);
  setIndex_t nextIt = twoHopNeighborSet->setData[candidate].next;
  if(candidate != -1)
    {
      if(olsrTwoHopNeighborSetFree(twoHopNeighborSet,candidate) != false)
        {
          return nextIt;
        }
    }
  return candidate; 
}

void olsrPrintTwoHopNeighborSet(olsrTwoHopNeighborSet_t *twoHopNeighborSet)
{
  setIndex_t candidate = twoHopNeighborSet->fullQueueEntry;
  if(candidate == -1)
    {
       DEBUG_PRINT_OLSR_NEIGHBOR2("2HopNeighborSet is empty!\n");
    }
  while (candidate != -1)
  {
    olsrTwoHopNeighborSetItem_t tmp = twoHopNeighborSet->setData[candidate];
    #ifdef USER_ROUTING
    DEBUG_PRINT_OLSR_NEIGHBOR2("2HopNeighborSet: neighbor:%d ,2hopNeighbor: %d,weight:%f\n",tmp.data.m_neighborAddr,tmp.data.m_twoHopNeighborAddr,tmp.data.m_weight);
    #else
    DEBUG_PRINT_OLSR_NEIGHBOR2("2HopNeighborSet: neighbor:%d ,2hopNeighbor: %d\n",tmp.data.m_neighborAddr,tmp.data.m_twoHopNeighborAddr);
    #endif
    candidate = tmp.next;
  }
}
/*
************************MprSetFunctions********************
*/

void olsrMprSetInit(olsrMprSet_t *mprSet)
{
  setIndex_t i;
  for(i=0; i < MPR_SET_SIZE-1; i++)
    {
      mprSet->setData[i].next = i+1;
    }
  mprSet->setData[i].next = -1;
  mprSet->freeQueueEntry = 0;
  mprSet->fullQueueEntry = -1;
}

static setIndex_t olsrMprSetMalloc(olsrMprSet_t *mprSet)
{
  if(mprSet->freeQueueEntry==-1)
    {
      DEBUG_PRINT_OLSR_SET("Full of sets!!!! can not malloc!!!\n");
      return -1;
    }
  else
    { 
      setIndex_t candidate = mprSet->freeQueueEntry;
      mprSet->freeQueueEntry = mprSet->setData[candidate].next;
      //insert to full queue
      setIndex_t tmp = mprSet->fullQueueEntry;
      mprSet->fullQueueEntry = candidate;
      mprSet->setData[candidate].next = tmp;
      return candidate;
    }
}


setIndex_t olsrInsertToMprSet(olsrMprSet_t *MprSet,const olsrMprTuple_t *item)
{
  setIndex_t candidate = olsrMprSetMalloc(MprSet);
  if(candidate!=-1)
    {
      memcpy(&MprSet->setData[candidate].data,item,sizeof(olsrMprTuple_t));
    }
  else
    {
      DEBUG_PRINT_OLSR_LINK("bad alloc whan insert by function [olsrInsertToMprSet]\n");
    }
  return candidate;
}

bool olsrFindMprByAddr(olsrMprSet_t *mprSet,\
                       olsrAddr_t addr)
{
  bool isFound = false;
  setIndex_t it = mprSet->fullQueueEntry;
  while(it!=-1)
    {
      if(mprSet->setData[it].data.m_addr==addr)
        {
          isFound = true;
          break;
        }
      it = mprSet->setData[it].next;
    }
  return isFound; 
  //TODO checkout lock all
}

void olsrPrintMprSet(olsrMprSet_t *mprSet)
{
  setIndex_t candidate = mprSet->fullQueueEntry;
  while(candidate != -1)
    {
      olsrMprSetItem_t tmp = mprSet->setData[candidate];
      DEBUG_PRINT_OLSR_MPR("MPRSET: %d\n",tmp.data.m_addr);
      candidate = tmp.next;
    }
}
/*
***********************MprSelectorSetFunctions********************
*/
void olsrMprSelectorSetInit(olsrMprSelectorSet_t *mprSelectorSet)
{
  setIndex_t i;
  for(i=0; i < MPR_SELECTOR_SET_SIZE-1; i++)
    {
      mprSelectorSet->setData[i].next = i+1;
    }
  mprSelectorSet->setData[i].next = -1;
  mprSelectorSet->freeQueueEntry = 0;
  mprSelectorSet->fullQueueEntry = -1;
}

static setIndex_t olsrMprSelectorSetMalloc(olsrMprSelectorSet_t *mprSelectorSet)
{
  if(mprSelectorSet->freeQueueEntry==-1)
    {
      DEBUG_PRINT_OLSR_SET("Full of sets!!!! can not malloc!!!\n");
      return -1;
    }
  else
    { 
      setIndex_t candidate = mprSelectorSet->freeQueueEntry;
      mprSelectorSet->freeQueueEntry = mprSelectorSet->setData[candidate].next;
      //insert to full queue
      setIndex_t tmp = mprSelectorSet->fullQueueEntry;
      mprSelectorSet->fullQueueEntry = candidate;
      mprSelectorSet->setData[candidate].next = tmp;
      return candidate;
    }
}

static bool olsrMprSelectorSetFree(olsrMprSelectorSet_t *mprSelectorSet, setIndex_t delItem)
{
  if(-1==delItem) return true;
  //del from full queue
  setIndex_t pre = mprSelectorSet->fullQueueEntry;
  if(delItem == pre)
    {
      mprSelectorSet->fullQueueEntry = mprSelectorSet->setData[pre].next;
      //insert to empty queue
      mprSelectorSet->setData[delItem].next = mprSelectorSet->freeQueueEntry;
      mprSelectorSet->freeQueueEntry = delItem;
      return true;
    }
  else 
    {
      while(pre!=-1)
        {
          if(mprSelectorSet->setData[pre].next==delItem)
            {
              mprSelectorSet->setData[pre].next = mprSelectorSet->setData[delItem].next;
              //insert to empty queue
              mprSelectorSet->setData[delItem].next = mprSelectorSet->freeQueueEntry;
              mprSelectorSet->freeQueueEntry = delItem;
              return true;
            }
          pre = mprSelectorSet->setData[pre].next;
        }
    }
    return false;
}

setIndex_t olsrInsertToMprSelectorSet(olsrMprSelectorSet_t *mprSelectorSet,const olsrMprSelectorTuple_t *item)
{
  setIndex_t candidate = olsrMprSelectorSetMalloc(mprSelectorSet);
  if(candidate!=-1)
    {
      memcpy(&mprSelectorSet->setData[candidate].data,item,sizeof(olsrMprSelectorTuple_t));
    }
  else
    {
      DEBUG_PRINT_OLSR_LINK("bad alloc whan insert by function [olsrInsertToMprSelectorSet]\n");
    }
  return candidate;
}

setIndex_t olsrFindInMprSelectorSet(olsrMprSelectorSet_t *mprSelectorSet, olsrAddr_t addr)
{
  setIndex_t it = mprSelectorSet->fullQueueEntry;
  while(it != -1)
    {
      if(mprSelectorSet->setData[it].data.m_addr == addr) 
        {
          break;
        }
      it = mprSelectorSet->setData[it].next;
    }
  return it;
}

bool olsrMprSelectorSetIsEmpty()
{
  return (olsrMprSelectorSet.fullQueueEntry == -1);
}

bool olsrEraseMprSelectorTuples(olsrMprSelectorSet_t *mprSelectorSet, olsrAddr_t addr)
{
  setIndex_t candidate = mprSelectorSet->fullQueueEntry;
  while(candidate != -1)
    {
      olsrMprSelectorSetItem_t tmp = mprSelectorSet->setData[candidate];
      if(tmp.data.m_addr == addr)
        {
          if(olsrMprSelectorSetFree(mprSelectorSet,candidate))
            {
              return true;
            }
        }
      candidate = tmp.next; 
    }
  return false;
}

void olsrDelMprSelectorTupleByPos(olsrMprSelectorSet_t *mprSelectorSet,setIndex_t pos)
{
  olsrMprSelectorSetFree(mprSelectorSet, pos);
}

void olsrPrintMprSelectorSet(olsrMprSelectorSet_t *mprSelectorSet)
{
  setIndex_t candidate = mprSelectorSet->fullQueueEntry;
  while(candidate != -1)
    {
      olsrMprSelectorSetItem_t tmp = mprSelectorSet->setData[candidate];
      DEBUG_PRINT_OLSR_MS("MPR_SEL_SET: add :%d\n",tmp.data.m_addr);
      candidate = tmp.next;
    }
}
/*
************************DuplicateSetFunctions********************
*/


void olsrDuplicateSetInit(olsrDuplicateSet_t *duplicateSet)
{
  setIndex_t i;
  for(i=0; i < DUPLICATE_SET_SIZE-1; i++)
    {
      duplicateSet->setData[i].next = i+1;
    }
  duplicateSet->setData[i].next = -1;
  duplicateSet->freeQueueEntry = 0;
  duplicateSet->fullQueueEntry = -1;
}

static setIndex_t olsrDuplicateSetMalloc(olsrDuplicateSet_t *duplicateSet)
{
  if(duplicateSet->freeQueueEntry==-1)
    {
      DEBUG_PRINT_OLSR_SET("Full of sets!!!! can not malloc!!!\n");
      return -1;
    }
  else
    { 
      setIndex_t candidate = duplicateSet->freeQueueEntry;
      duplicateSet->freeQueueEntry = duplicateSet->setData[candidate].next;
      //insert to full queue
      setIndex_t tmp = duplicateSet->fullQueueEntry;
      duplicateSet->fullQueueEntry = candidate;
      duplicateSet->setData[candidate].next = tmp;
      return candidate;
    }
}

static bool olsrDuplicateSetFree(olsrDuplicateSet_t *duplicateSet, setIndex_t delItem)
{
  if(-1==delItem) return true;
  //del from full queue
  setIndex_t pre = duplicateSet->fullQueueEntry;
  if(delItem == pre)
    {
      duplicateSet->fullQueueEntry = duplicateSet->setData[pre].next;
      //insert to empty queue
      duplicateSet->setData[delItem].next = duplicateSet->freeQueueEntry;
      duplicateSet->freeQueueEntry = delItem;
      return true;
    }
  else 
    {
      while(pre!=-1)
        {
          if(duplicateSet->setData[pre].next==delItem)
            {
              duplicateSet->setData[pre].next = duplicateSet->setData[delItem].next;
              //insert to empty queue
              duplicateSet->setData[delItem].next = duplicateSet->freeQueueEntry;
              duplicateSet->freeQueueEntry = delItem;
              return true;
            }
          pre = duplicateSet->setData[pre].next;
        }
    }
    return false;
}

setIndex_t olsrFindInDuplicateSet(olsrDuplicateSet_t *duplicateSet,olsrAddr_t originator,uint16_t seq)
{
  setIndex_t duplicateIt = duplicateSet->fullQueueEntry;
  while(duplicateIt != -1)
    {
      olsrDuplicateSetItem_t item = duplicateSet->setData[duplicateIt];
      if(item.data.m_addr == originator && item.data.m_seqenceNumber == seq)
        {
          break;
        }
      duplicateIt = item.next;
    }
  return duplicateIt;
}

setIndex_t olsrInsertDuplicateTuple(olsrDuplicateSet_t *duplicateSet,const olsrDuplicateTuple_t *tuple)
{
  setIndex_t duplicateIt = olsrDuplicateSetMalloc(duplicateSet);
  if(duplicateIt != -1)
    {
      memcpy(&duplicateSet->setData[duplicateIt].data,tuple,sizeof(olsrDuplicateTuple_t));
    }
  else
    {
      DEBUG_PRINT_OLSR_DUPLICATE("bad alloc in olsrInsertDuplicateTuple");
    }
  return duplicateIt;
}

bool olsrDuplicateSetClearExpire()
{
  setIndex_t candidate = olsrDuplicateSet.fullQueueEntry;
  olsrTime_t now = xTaskGetTickCount();
  bool isChange = false;
  while(candidate != -1)
    {
      olsrDuplicateSetItem_t tmp = olsrDuplicateSet.setData[candidate];
      if(tmp.data.m_expirationTime < now)
        {
          setIndex_t nextIt = tmp.next;
          olsrDuplicateSetFree(&olsrDuplicateSet,candidate);
          candidate = nextIt;
          isChange = true;
          continue;
        }
      candidate = tmp.next;
    }
  return isChange;
}

void olsrPrintDuplicateSet(olsrDuplicateSet_t *duplicateSet)
{
  setIndex_t candidate = duplicateSet->fullQueueEntry;
  while(candidate != -1)
    {
      olsrDuplicateSetItem_t tmp = duplicateSet->setData[candidate];
      DEBUG_PRINT_OLSR_DUPLICATE("DUPSET:addr:%d,seq:%d\n",tmp.data.m_addr,tmp.data.m_seqenceNumber);
      candidate = tmp.next;
    }
}


/*
************************RoutingSetFunction********************
*/

void olsrRoutingSetInit(olsrRoutingSet_t *routingSet)
{
  setIndex_t i;
  for(i=0; i < DUPLICATE_SET_SIZE-1; i++)
    {
      routingSet->setData[i].next = i+1;
    }
  routingSet->setData[i].next = -1;
  routingSet->freeQueueEntry = 0;
  routingSet->fullQueueEntry = -1;
  routingSet->size = 0;
}

static setIndex_t olsrRoutingSetMalloc(olsrRoutingSet_t *routingSet)
{
  if(routingSet->freeQueueEntry==-1)
    {
      DEBUG_PRINT_OLSR_SET("Full of sets!!!! can not malloc!!!\n");
      return -1;
    }
  else
    { 
      setIndex_t candidate = routingSet->freeQueueEntry;
      routingSet->freeQueueEntry = routingSet->setData[candidate].next;
      //insert to full queue
      setIndex_t tmp = routingSet->fullQueueEntry;
      routingSet->fullQueueEntry = candidate;
      routingSet->setData[candidate].next = tmp;
      return candidate;
    }
}

static bool olsrRoutingSetFree(olsrRoutingSet_t *routingSet, setIndex_t delItem)
{
  if(-1==delItem) return true;
  //del from full queue
  setIndex_t pre = routingSet->fullQueueEntry;
  if(delItem == pre)
    {
      routingSet->fullQueueEntry = routingSet->setData[pre].next;
      //insert to empty queue
      routingSet->setData[delItem].next = routingSet->freeQueueEntry;
      routingSet->freeQueueEntry = delItem;
	  routingSet->size = routingSet->size - 1;
      return true;
    }
  else 
    {
      while(pre!=-1)
        {
          if(routingSet->setData[pre].next==delItem)
            {
              routingSet->setData[pre].next = routingSet->setData[delItem].next;
              //insert to empty queue
              routingSet->setData[delItem].next = routingSet->freeQueueEntry;
              routingSet->freeQueueEntry = delItem;
			  routingSet->size = routingSet->size - 1;
              return true;
            }
          pre = routingSet->setData[pre].next;
        }
    }
    return false;
}
bool olsrRoutingSetInsert(olsrRoutingSet_t *routingSet,olsrRoutingTuple_t *tuple)
{
  setIndex_t candidate = olsrRoutingSetMalloc(routingSet);
  if(candidate != -1)
    {
      memcpy(&routingSet->setData[candidate].data,tuple,sizeof(olsrRoutingTuple_t));
	  routingSet->size++;
      return true;
    }
  else
    {
      return false;
    }
}

// todo : 应该返回setIndex_t类型
setIndex_t olsrFindInRoutingTable(olsrRoutingSet_t *routingSet,olsrAddr_t destAddr)
{
  setIndex_t it = routingSet->fullQueueEntry;
  while(it != -1)
    {
      olsrRoutingSetItem_t routeNode = routingSet->setData[it];
      if(routeNode.data.m_destAddr == destAddr)
        {
          return it;
        }
      it = routeNode.next;
    }
  return it;
}

void olsrPrintRoutingSet(olsrRoutingSet_t *routingSet)
{
  setIndex_t it = routingSet->fullQueueEntry;
  while(it != -1)
    {
      olsrRoutingSetItem_t routeNode = routingSet->setData[it];
      DEBUG_PRINT_OLSR_APP("Dest:%d,Next:%d,weight%f\n",routeNode.data.m_destAddr,routeNode.data.m_nextAddr,routeNode.data.m_weight);
      it = routeNode.next;
    } 
}

void olsrRoutingSetCopy(olsrRoutingSet_t *dest,olsrRoutingSet_t *source)
{
  memcpy(dest,source,sizeof(olsrRoutingSet_t));
}

/*
************************RangingSetFunction********************
*/

void olsrRangingTableInit(olsrRangingTable_t *rangingTable) {
  setIndex_t i;
  for (i = 0; i < TIMESTAMP_SET_SIZE - 1; i++) {
    rangingTable->setData[i].next = i + 1;
  }
  rangingTable->setData[i].next = -1;
  rangingTable->freeQueueEntry = 0;
  rangingTable->fullQueueEntry = -1;
  rangingTable->size = 0;
}

static setIndex_t olsrRangingTableMalloc(olsrRangingTable_t *rangingTable) {
  if (rangingTable->freeQueueEntry == -1) {
    DEBUG_PRINT_OLSR_SET("Full of sets!!!! can not malloc!!!\n");
    return -1;
  } else {
    setIndex_t candidate = rangingTable->freeQueueEntry;
    rangingTable->freeQueueEntry = rangingTable->setData[candidate].next;
    //insert to full queue
    setIndex_t tmp = rangingTable->fullQueueEntry;
    rangingTable->fullQueueEntry = candidate;
    rangingTable->setData[candidate].next = tmp;
    return candidate;
  }
}

static bool olsrRangingTableFree(olsrRangingTable_t *rangingTable, setIndex_t delItem) {
  if (-1 == delItem) {
    return true;
  }
  //del from full queue
  setIndex_t pre = rangingTable->fullQueueEntry;
  if (delItem == pre) {
    rangingTable->fullQueueEntry = rangingTable->setData[pre].next;
    //insert to empty queue
    rangingTable->setData[delItem].next = rangingTable->freeQueueEntry;
    rangingTable->freeQueueEntry = delItem;
    rangingTable->size = rangingTable->size - 1;
    return true;
  } else {
    while (pre != -1) {
      if (rangingTable->setData[pre].next == delItem) {
        rangingTable->setData[pre].next = rangingTable->setData[delItem].next;
        //insert to empty queue
        rangingTable->setData[delItem].next = rangingTable->freeQueueEntry;
        rangingTable->freeQueueEntry = delItem;
        rangingTable->size = rangingTable->size - 1;
        return true;
      }
      pre = rangingTable->setData[pre].next;
    }
  }
  return false;
}

setIndex_t olsrRangingTableInsert(olsrRangingTable_t *rangingTable, olsrRangingTuple_t *tuple) {
  setIndex_t candidate = olsrRangingTableMalloc(rangingTable);
  if (candidate != -1) {
    memcpy(&rangingTable->setData[candidate].data, tuple, sizeof(olsrRangingTuple_t));
    rangingTable->size++;
  }
  return candidate;
}

setIndex_t olsrFindInRangingTable(olsrRangingTable_t *rangingTable, olsrAddr_t addr) {
  setIndex_t it = rangingTable->fullQueueEntry;
  while (it != -1) {
    olsrRangingTableItem_t rangingNode = rangingTable->setData[it];
    if (rangingNode.data.m_tsAddress == addr) {
      break;
    }
    it = rangingNode.next;
  }
  return it;
}

void olsrPrintRangingTableTuple(olsrRangingTuple_t *tuple) {
  /*
+------+------+------+------+
|  Rp  |  Tr  |  Rf  |      |
+------+------+------+------+
|  Tp  |  Rr  |  Tf  |  Re  |
+------+------+------+------+
*/
//  DEBUG_PRINT_OLSR_TS("Rp:(%u)%llu\tTr:(%u)%llu\tRf:(%u)%llu\n",
//                      tuple->Rp.m_seqenceNumber, tuple->Rp.m_timestamp.full,
//                      tuple->Tr.m_seqenceNumber, tuple->Tr.m_timestamp.full,
//                      tuple->Rf.m_seqenceNumber, tuple->Rf.m_timestamp.full);
//  DEBUG_PRINT_OLSR_TS("Tp:(%u)%llu\tRr:(%u)%llu\tTf:(%u)%llu\tRe:(%u)%llu\n",
//                      tuple->Tp.m_seqenceNumber, tuple->Tp.m_timestamp.full,
//                      tuple->Rr.m_seqenceNumber, tuple->Rr.m_timestamp.full,
//                      tuple->Tf.m_seqenceNumber, tuple->Tf.m_timestamp.full,
//                      tuple->Re.m_seqenceNumber, tuple->Re.m_timestamp.full);
  DEBUG_PRINT_OLSR_TS("addr:%u, nextDeliveryTime:%lu \n", tuple->m_tsAddress, tuple->m_nextDeliveryTime);
}

void olsrPrintRangingTable(olsrRangingTable_t *rangingTable) {
  for (setIndex_t i = rangingTable->fullQueueEntry; i != -1; i = rangingTable->setData[i].next) {
    olsrPrintRangingTableTuple(&rangingTable->setData[i].data);
  }
}

bool olsrDelRangingTupleByAddr(olsrRangingTable_t *rangingTable, setIndex_t addr) {
  return olsrRangingTableFree(rangingTable, addr);
}

bool olsrRangingTableClearExpire(olsrRangingTable_t *rangingTable) {
  setIndex_t candidate = rangingTable->fullQueueEntry;
  olsrTime_t now = xTaskGetTickCount();
  bool isChange = false;
  while(candidate != -1)
  {
    olsrRangingTableItem_t tmp = rangingTable->setData[candidate];
    if(tmp.data.m_expiration < now)
    {
      setIndex_t nextIt = tmp.next;
      DEBUG_PRINT_OLSR_TS("neighbor %u expiration occurred!\n", rangingTable->setData[candidate].data.m_tsAddress);
      olsrRangingTableFree(rangingTable, candidate);
      candidate = nextIt;
      isChange = true;
      continue;
    }
    candidate = tmp.next;
  }
  return isChange;
}

void olsrSortRangingTable(olsrRangingTable_t *rangingTable) {
  if (rangingTable->fullQueueEntry == -1) {
    return;
  }
  setIndex_t newHead = rangingTable->fullQueueEntry;
  setIndex_t cur = rangingTable->setData[newHead].next;
  rangingTable->setData[newHead].next = -1;
  setIndex_t next = -1;
  while (cur != -1) {
    next = rangingTable->setData[cur].next;
    if (rangingTable->setData[cur].data.m_nextDeliveryTime <= rangingTable->setData[newHead].data.m_nextDeliveryTime) {
      rangingTable->setData[cur].next = newHead;
      newHead = cur;
    } else {
      setIndex_t start = rangingTable->setData[newHead].next;
      setIndex_t pre = newHead;
      while (start != -1 && rangingTable->setData[cur].data.m_nextDeliveryTime
          > rangingTable->setData[start].data.m_nextDeliveryTime) {
        pre = start;
        start = rangingTable->setData[start].next;
      }
      rangingTable->setData[cur].next = start;
      rangingTable->setData[pre].next = cur;
    }
    cur = next;
  }
  rangingTable->fullQueueEntry = newHead;
}
/*
************************CommonFunctions********************
*/












static void olsrSetEntryInit()
{
  olsrTopologySetLock = xSemaphoreCreateMutex();
  olsrLinkSetLock = xSemaphoreCreateMutex();
  olsrNeighborSetLock = xSemaphoreCreateMutex();
  olsrMprSetLock = xSemaphoreCreateMutex();
  olsrTwoHopNeighborSetLock = xSemaphoreCreateMutex();
  olsrMprSelectorSetLock = xSemaphoreCreateMutex();
  olsrDuplicateSetLock = xSemaphoreCreateMutex();
}


void olsrStructInitAll(dwDevice_t *dev)
{
  DEBUG_PRINT_OLSR_SYSTEM("start init struct\n");
  olsrSetEntryInit();
  olsrTopologySetInit(&olsrTopologySet);
  olsrLinkSetInit(&olsrLinkSet);
  olsrNeighborSetInit(&olsrNeighborSet);
  olsrTwoHopNeighborSetInit(&olsrTwoHopNeighborSet);
  olsrMprSetInit(&olsrMprSet);
  olsrDuplicateSetInit(&olsrDuplicateSet);
  olsrMprSelectorSetInit(&olsrMprSelectorSet);
  olsrRoutingSetInit(&olsrRoutingSet);
  olsrRangingTableInit(&olsrRangingTable);
}
