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
 * syslink.c: Communication between NRF51 and STM32
 */
#define DEBUG_MODULE "SL"

#include <stdbool.h>
#include <string.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

#include "config.h"
#include "autoconf.h"
#include "debug.h"
#include "syslink.h"
#include "radiolink.h"
#include "uart_syslink.h"
#include "configblock.h"
#include "pm.h"
#include "ow.h"
#include "static_mem.h"
#include "system.h"
#include "param.h"

static bool isInit = false;
static uint8_t sendBuffer[SYSLINK_MTU + 6];

static void syslinkRouteIncommingPacket(SyslinkPacket *slp);

static xTimerHandle debugTimer;
static uint8_t triggerDebugProbe;
static void debugHandler(xTimerHandle timer);
static void debugSyslinkReceive(SyslinkPacket *slp);

static xSemaphoreHandle syslinkAccess;

STATIC_MEM_TASK_ALLOC_STACK_NO_DMA_CCM_SAFE(syslinkTask, SYSLINK_TASK_STACKSIZE);

/* Syslink task, handles communication between nrf and stm and dispatch messages
 */
static void syslinkTask(void *param)
{
  SyslinkPacket slp;
  while(1)
  {
    uartslkGetPacketBlocking(&slp);
    syslinkRouteIncommingPacket(&slp);
  }
}

static void syslinkRouteIncommingPacket(SyslinkPacket *slp)
{
  uint8_t groupType;

  groupType = slp->type & SYSLINK_GROUP_MASK;

  switch (groupType)
  {
    case SYSLINK_RADIO_GROUP:
      radiolinkSyslinkDispatch(slp);
      break;
    case SYSLINK_PM_GROUP:
      pmSyslinkUpdate(slp);
      break;
    case SYSLINK_OW_GROUP:
      owSyslinkReceive(slp);
      break;
    case SYSLINK_SYS_GROUP:
      systemSyslinkReceive(slp);
      break;
    case SYSLINK_DEBUG_GROUP:
      debugSyslinkReceive(slp);
      break;
    default:
      DEBUG_PRINT("Unknown packet:%X.\n", slp->type);
      break;
  }
}

/*
 * Public functions
 */

void syslinkInit()
{
  if(isInit) {
    return;
  }

  vSemaphoreCreateBinary(syslinkAccess);

  STATIC_MEM_TASK_CREATE(syslinkTask, syslinkTask, SYSLINK_TASK_NAME, NULL, SYSLINK_TASK_PRI);

  debugTimer = xTimerCreate( "syslinkTimer", M2T(1000), pdTRUE, NULL, debugHandler );
  xTimerStart(debugTimer, M2T(1000));

  isInit = true;
}

bool syslinkTest()
{
  return isInit;
}

bool isSyslinkUp()
{
  return isInit;
}


int syslinkSendPacket(SyslinkPacket *slp)
{
  int i = 0;
  int dataSize;
  uint8_t cksum[2] = {0};

  xSemaphoreTake(syslinkAccess, portMAX_DELAY);

  ASSERT(slp->length <= SYSLINK_MTU);

  sendBuffer[0] = SYSLINK_START_BYTE1;
  sendBuffer[1] = SYSLINK_START_BYTE2;
  sendBuffer[2] = slp->type;
  sendBuffer[3] = slp->length;

  memcpy(&sendBuffer[4], slp->data, slp->length);
  dataSize = slp->length + 6;
  // Calculate checksum delux
  for (i = 2; i < dataSize - 2; i++)
  {
    cksum[0] += sendBuffer[i];
    cksum[1] += cksum[0];
  }
  sendBuffer[dataSize-2] = cksum[0];
  sendBuffer[dataSize-1] = cksum[1];

  uartslkSendDataDmaBlocking(dataSize, sendBuffer);

  xSemaphoreGive(syslinkAccess);

  return 0;
}

static void debugHandler(xTimerHandle timer) {
  static SyslinkPacket txPacket;

  if (triggerDebugProbe) {
    triggerDebugProbe = 0;

    uartSyslinkDumpDebugProbe();
    DEBUG_PRINT("Syslink NRF debug probe initialized\n");

    txPacket.type = SYSLINK_DEBUG_PROBE;
    txPacket.length = 0;
    syslinkSendPacket(&txPacket);
  }
}

static void debugSyslinkReceive(SyslinkPacket *slp) {
  if (slp->type == SYSLINK_DEBUG_PROBE) {
    DEBUG_PRINT("NRF Address received: %d\n", slp->data[0]);
    DEBUG_PRINT("NRF Chan received: %d\n", slp->data[1]);
    DEBUG_PRINT("NRF Rate received: %d\n", slp->data[2]);
    DEBUG_PRINT("NRF Dropped: %d\n", slp->data[3]);
    DEBUG_PRINT("NRF uart error code: %d\n", slp->data[4]);
    DEBUG_PRINT("NRF uart error count: %d\n", slp->data[5]);
    DEBUG_PRINT("NRF uart checksum 1 fail count: %d\n", slp->data[6]);
    DEBUG_PRINT("NRF uart checksum 2 fail count: %d\n", slp->data[7]);
  }
}

PARAM_GROUP_START(syslink)
/**
 * @brief Trigger syslink debug probe in the NRF by setting to 1
 */
PARAM_ADD(PARAM_UINT8, probe, &triggerDebugProbe)
PARAM_GROUP_STOP(syslink)
