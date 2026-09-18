/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2012 BitCraze AB
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
 * usblink.c
 */

#include <stdbool.h>
#include <string.h>

#include "config.h"
#include "usblink.h"
#include "crtp.h"
#include "configblock.h"
#include "ledseq.h"
#include "pm.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "queuemonitor.h"
#include "semphr.h"
#include "static_mem.h"

#include "usb.h"

static bool isInit = false;
static xQueueHandle crtpPacketDelivery;
STATIC_MEM_QUEUE_ALLOC(crtpPacketDelivery, 16, sizeof(CRTPPacket));
static USBPacket sendStage;

static int usblinkSendPacket(CRTPPacket *p);
static int usblinkSetEnable(bool enable);
static int usblinkReceivePacket(CRTPPacket *p);

static struct crtpLinkOperations usblinkOp =
{
  .setEnable         = usblinkSetEnable,
  .sendPacket        = usblinkSendPacket,
  .receivePacket     = usblinkReceivePacket,
};

xQueueHandle usblinkGetCrtpDeliveryQueue(void)
{
  return crtpPacketDelivery;
}

static int usblinkReceivePacket(CRTPPacket *p)
{
  if (xQueueReceive(crtpPacketDelivery, p, M2T(100)) == pdTRUE)
  {
    usbResumeRx();
    ledseqRun(&seq_linkUp);
    return 0;
  }

  return -1;
}

static int usblinkSendPacket(CRTPPacket *p)
{
  ASSERT(p->size < SYSLINK_MTU);
  ASSERT(p->size <= CRTP_MAX_DATA_SIZE);

  sendStage.size = p->size + 1;
  sendStage.data[0] = p->header;
  memcpy(&sendStage.data[1], p->data, p->size);

  ledseqRun(&seq_linkDown);

  return usbSendData(&sendStage);
}

static int usblinkSetEnable(bool enable)
{
  return 0;
}

/*
 * Public functions
 */

void usblinkInit()
{
  if(isInit)
    return;

  // Queue must exist before usbInit, since the OUT-EP ISR writes into it.
  crtpPacketDelivery = STATIC_MEM_QUEUE_CREATE(crtpPacketDelivery);
  DEBUG_QUEUE_MONITOR_REGISTER(crtpPacketDelivery);

  usbInit();

  isInit = true;
}

bool usblinkTest()
{
  return isInit;
}

struct crtpLinkOperations * usblinkGetLink()
{
  return &usblinkOp;
}
