/**
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * crtp.c - CrazyRealtimeTransferProtocol stack
 */

#include <stdbool.h>
#include <errno.h>

/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#include "uart.h"

#include "config.h"

#include "crtp.h"
#include "info.h"
#include "cfassert.h"

static bool isInit;

static int nopFunc(void);
static struct crtpLinkOperations nopLink = {
  .setEnable         = (void*) nopFunc,
  .sendPacket        = (void*) nopFunc,
  .receivePacket     = (void*) nopFunc,
}; 

static struct crtpLinkOperations *link = &nopLink;

static xQueueHandle  tmpQueue;

static xQueueHandle  rxQueue;

#define CRTP_NBR_OF_PORTS 16
#define CRTP_TX_QUEUE_SIZE 20
#define CRTP_RX_QUEUE_SIZE 2

static void crtpTxTask(void *param);
static void crtpRxTask(void *param);

static xQueueHandle queues[CRTP_NBR_OF_PORTS];
static volatile CrtpCallback callbacks[CRTP_NBR_OF_PORTS];

void crtpInit(void)
{
  if(isInit)
    return;

  tmpQueue = xQueueCreate(CRTP_TX_QUEUE_SIZE, sizeof(CRTPPacket));
  rxQueue = xQueueCreate(CRTP_RX_QUEUE_SIZE, sizeof(CRTPPacket));
  /* Start Rx/Tx tasks */
  xTaskCreate(crtpTxTask, (const signed char * const)"CRTP-Tx",
              configMINIMAL_STACK_SIZE, NULL, /*priority*/2, NULL);
  xTaskCreate(crtpRxTask, (const signed char * const)"CRTP-Rx",
              configMINIMAL_STACK_SIZE, NULL, /*priority*/2, NULL);
  
  isInit = true;
}

bool crtpTest(void)
{
  return isInit;
}

void crtpInitTaskQueue(CRTPPort portId)
{
  ASSERT(queues[portId] == NULL);
  
  queues[portId] = xQueueCreate(1, sizeof(CRTPPacket));
}

int crtpReceivePacket(CRTPPort portId, CRTPPacket *p)
{
  ASSERT(queues[portId]);
  ASSERT(p);
    
  return xQueueReceive(queues[portId], p, 0);
}

int crtpReceivePacketBlock(CRTPPort portId, CRTPPacket *p)
{
  ASSERT(queues[portId]);
  ASSERT(p);
  
  return xQueueReceive(queues[portId], p, portMAX_DELAY);
}


int crtpReceivePacketWait(CRTPPort portId, CRTPPacket *p, int wait) {
  ASSERT(queues[portId]);
  ASSERT(p);
  
  return xQueueReceive(queues[portId], p, M2T(wait));
}

void crtpTxTask(void *param)
{
  CRTPPacket p;

  while (TRUE)
  {
    if (xQueueReceive(tmpQueue, &p, portMAX_DELAY) == pdTRUE)
    {
      link->sendPacket(&p);
    }
  }
}

void crtpRxTask(void *param)
{
  CRTPPacket p;
  static unsigned int droppedPacket=0;

  while (TRUE)
  {
    if (!link->receivePacket(&p))
    {
      if(queues[p.port])
      {
        // TODO: If full, remove one packet and then send
        xQueueSend(queues[p.port], &p, 0);
      } else {
        droppedPacket++;
      }
      
      if(callbacks[p.port])
        callbacks[p.port](&p);  //Dangerous?
    }
  }
}

void crtpRegisterPortCB(int port, CrtpCallback cb)
{
  if (port>CRTP_NBR_OF_PORTS)
    return;
  
  callbacks[port] = cb;
}

int crtpSendPacket(CRTPPacket *p)
{
  ASSERT(p); 

  return xQueueSend(tmpQueue, p, 0);
}

int crtpSendPacketBlock(CRTPPacket *p)
{
  ASSERT(p); 

  return xQueueSend(tmpQueue, p, portMAX_DELAY);
}

int crtpReset(void)
{
  xQueueReset(tmpQueue);
  if (link->reset) {
    link->reset();
  }

  return 0;
}

bool crtpIsConnected(void)
{
  if (link->isConnected)
    return link->isConnected();
  return true;
}

void crtpPacketReveived(CRTPPacket *p)
{
  portBASE_TYPE xHigherPriorityTaskWoken;

  xHigherPriorityTaskWoken = pdFALSE;
  xQueueSendFromISR(rxQueue, p, &xHigherPriorityTaskWoken);
}

void crtpSetLink(struct crtpLinkOperations * lk)
{
  if(link)
    link->setEnable(false);

  if (lk)
    link = lk;
  else
    link = &nopLink;

  link->setEnable(true);
}

static int nopFunc(void)
{
  return ENETDOWN;
}
