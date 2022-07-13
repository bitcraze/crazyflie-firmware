/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2022 Bitcraze AB
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
 */

/* CRTP link which enables sending packets over CPX */

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

#include "cpx.h"
#include "cpx_internal_router.h"
#include "cpxlink.h"
#include "debug.h"

static bool isInit = false;

static bool clientIsConnected = false;

static CPXPacket_t cpxTx;
static CPXPacket_t cpxRx;

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
  if (cpxInternalRouterReceiveCRTP(&cpxRx) == pdTRUE)
  {

    p->size = cpxRx.dataLength - 1;
    p->header = cpxRx.data[0];
    memcpy(p->data, &cpxRx.data[1], cpxRx.dataLength - 1);

    ledseqRun(&seq_linkUp);
    return 0;
  }

  return -1;
}

static int cpxlinkSendPacket(CRTPPacket *p)
{
  ledseqRun(&seq_linkDown);

  cpxInitRoute(CPX_T_STM32, CPX_T_WIFI_HOST, CPX_F_CRTP, &cpxTx.route);

  memcpy(&cpxTx.data, p->raw, p->size + 1);
  cpxTx.dataLength = p->size + 1;
  cpxSendPacketBlocking(&cpxTx);

  return true;
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

void cpxLinkSetConnected(bool isConnected) {
  clientIsConnected = isConnected;
}

struct crtpLinkOperations * cpxlinkGetLink()
{
  return &cpxlinkOp;
}
