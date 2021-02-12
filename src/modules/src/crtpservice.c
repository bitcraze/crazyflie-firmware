/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2021 Bitcraze AB
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
 * crtpservice.c - Implements low level services for CRTP
 */

#include <stdbool.h>
#include <string.h>

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"

#include "crtp.h"
#include "crtpservice.h"
#include "static_mem.h"
#include "param.h"


typedef enum {
  linkEcho   = 0x00,
  linkSource = 0x01,
  linkSink   = 0x02,
} LinkNbr;


static bool isInit=false;
static uint16_t echoDelay=0;
STATIC_MEM_TASK_ALLOC_STACK_NO_DMA_CCM_SAFE(crtpSrvTask, CRTP_SRV_TASK_STACKSIZE);

static void crtpSrvTask(void*);

void crtpserviceInit(void)
{
  if (isInit)
    return;

  //Start the task
  STATIC_MEM_TASK_CREATE(crtpSrvTask, crtpSrvTask, CRTP_SRV_TASK_NAME, NULL, CRTP_SRV_TASK_PRI);

  isInit = true;
}

bool crtpserviceTest(void)
{
  return isInit;
}

static void crtpSrvTask(void* prm)
{
  static CRTPPacket p;

  crtpInitTaskQueue(CRTP_PORT_LINK);

  while(1) {
    crtpReceivePacketBlock(CRTP_PORT_LINK, &p);

    switch (p.channel)
    {
      case linkEcho:
        if (echoDelay > 0) {
          vTaskDelay(M2T(echoDelay));
        }
        crtpSendPacketBlock(&p);
        break;
      case linkSource:
        p.size = CRTP_MAX_DATA_SIZE;
        bzero(p.data, CRTP_MAX_DATA_SIZE);
        strcpy((char*)p.data, "Bitcraze Crazyflie");
        crtpSendPacketBlock(&p);
        break;
      case linkSink:
        /* Ignore packet */
        break;
      default:
        break;
    }
  }
}

PARAM_GROUP_START(crtpsrv)
PARAM_ADD(PARAM_UINT16, echoDelay, &echoDelay)
PARAM_GROUP_STOP(crtpsrv)
