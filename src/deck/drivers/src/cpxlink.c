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
 * syslink.c: nRF24L01 implementation of the CRTP link
 */

#include <stdbool.h>
#include <string.h>

#include "config.h"
#include "cpxlink.h"
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

#include "aideck.h"
#include "aideck-router.h"
#include "cpxlink.h"
#include "aideck-router.h"
#include "debug.h"

static bool isInit = false;

static bool clientIsConnected = false;

static int cpxlinkSendPacket(CRTPPacket *p);
static int cpxlinkSetEnable(bool enable);
static int cpxlinkReceivePacket(CRTPPacket *p);
static bool cpxlinkIsConnected(void);

static struct crtpLinkOperations cpxlinkOp =
{
  .setEnable         = cpxlinkSetEnable,
  .sendPacket        = cpxlinkSendPacket,
  .receivePacket     = cpxlinkReceivePacket,
  .isConnected       = cpxlinkIsConnected
};

static int cpxlinkReceivePacket(CRTPPacket *p)
{
  if (aideckReceiveCRTPPacket(p) == pdTRUE)
  {
    ledseqRun(&seq_linkUp);
    return 0;
  }

  return -1;
}

static int cpxlinkSendPacket(CRTPPacket *p)
{
  ledseqRun(&seq_linkDown);

  return aideckSendCRTPPacket(p);
}

static int cpxlinkSetEnable(bool enable)
{
  return 0;
}

static bool cpxlinkIsConnected(void) {
  return clientIsConnected;
}

void cpxlinkInit()
{
  if(isInit)
    return;

  isInit = true;
}

bool cpxlinkTest()
{
  return isInit;
}

void cpxLinkSetClientConnected(bool isConnected) {
  clientIsConnected = isConnected;
}

struct crtpLinkOperations * cpxlinkGetLink()
{
  return &cpxlinkOp;
}
