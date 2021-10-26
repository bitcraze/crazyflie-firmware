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
 * platformservice.c - Implements platform services for CRTP
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"

#include "config.h"
#include "crtp.h"
#include "platformservice.h"
#include "syslink.h"
#include "version.h"
#include "platform.h"
#include "app_channel.h"
#include "static_mem.h"

static bool isInit=false;
STATIC_MEM_TASK_ALLOC_STACK_NO_DMA_CCM_SAFE(platformSrvTask, PLATFORM_SRV_TASK_STACKSIZE);

typedef enum {
  platformCommand   = 0x00,
  versionCommand    = 0x01,
  appChannel        = 0x02,
} Channel;

typedef enum {
  setContinousWave   = 0x00,
} PlatformCommand;

typedef enum {
  getProtocolVersion = 0x00,
  getFirmwareVersion = 0x01,
  getDeviceTypeName  = 0x02,
} VersionCommand;

static void platformSrvTask(void*);
static void platformCommandProcess(uint8_t command, uint8_t *data);
static void versionCommandProcess(CRTPPacket *p);

void platformserviceInit(void)
{
  if (isInit)
    return;

  appchannelInit();

  //Start the task
  STATIC_MEM_TASK_CREATE(platformSrvTask, platformSrvTask, PLATFORM_SRV_TASK_NAME, NULL, PLATFORM_SRV_TASK_PRI);

  isInit = true;
}

bool platformserviceTest(void)
{
  return isInit;
}

static void platformSrvTask(void* prm)
{
  static CRTPPacket p;

  crtpInitTaskQueue(CRTP_PORT_PLATFORM);

  while(1) {
    crtpReceivePacketBlock(CRTP_PORT_PLATFORM, &p);

    switch (p.channel)
    {
      case platformCommand:
        platformCommandProcess(p.data[0], &p.data[1]);
        crtpSendPacketBlock(&p);
        break;
      case versionCommand:
        versionCommandProcess(&p);
        break;
      case appChannel:
        appchannelIncomingPacket(&p);
        break;
      default:
        break;
    }
  }
}

static void platformCommandProcess(uint8_t command, uint8_t *data)
{
  static SyslinkPacket slp;

  switch (command) {
    case setContinousWave:
      slp.type = SYSLINK_RADIO_CONTWAVE;
      slp.length = 1;
      slp.data[0] = data[0];
      syslinkSendPacket(&slp);
      break;
    default:
      break;
  }
}

int platformserviceSendAppchannelPacket(CRTPPacket *p)
{
  p->port = CRTP_PORT_PLATFORM;
  p->channel = appChannel;
  return crtpSendPacket(p);
}

int platformserviceSendAppchannelPacketBlock(CRTPPacket *p)
{
  p->port = CRTP_PORT_PLATFORM;
  p->channel = appChannel;
  return crtpSendPacketBlock(p);
}

static void versionCommandProcess(CRTPPacket *p)
{
  switch (p->data[0]) {
    case getProtocolVersion:
      *(int*)&p->data[1] = PROTOCOL_VERSION;
      p->size = 5;
      crtpSendPacketBlock(p);
      break;
    case getFirmwareVersion:
      strncpy((char*)&p->data[1], V_STAG, CRTP_MAX_DATA_SIZE-1);
      p->size = (strlen(V_STAG)>CRTP_MAX_DATA_SIZE-1)?CRTP_MAX_DATA_SIZE:strlen(V_STAG)+1;
      crtpSendPacketBlock(p);
      break;
    case getDeviceTypeName:
      {
      const char* name = platformConfigGetDeviceTypeName();
      strncpy((char*)&p->data[1], name, CRTP_MAX_DATA_SIZE-1);
      p->size = (strlen(name)>CRTP_MAX_DATA_SIZE-1)?CRTP_MAX_DATA_SIZE:strlen(name)+1;
      crtpSendPacketBlock(p);
      }
      break;
    default:
      break;
  }
}
