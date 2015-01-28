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
 * crtpservice.c - Implements low level services for CRTP
 */

#include <stdbool.h>

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>

#include "crtp.h"
#include "platformservice.h"
#include "syslink.h"

static bool isInit=false;

typedef enum {
  platformCommand   = 0x00,
} Channel;

typedef enum {
  setContinousWave   = 0x00,
} PlatformCommand;

void platformserviceHandler(CRTPPacket *p);
static void platformCommandProcess(uint8_t command, uint8_t *data);

void platformserviceInit(void)
{
  if (isInit)
    return;

  // Register a callback to service the Platform port
  crtpRegisterPortCB(CRTP_PORT_PLATFORM, platformserviceHandler);
  
  isInit = true;
}

bool platformserviceTest(void)
{
  return isInit;
}

void platformserviceHandler(CRTPPacket *p)
{
  switch (p->channel)
  {
    case platformCommand:
      platformCommandProcess(p->data[0], &p->data[1]);
      crtpSendPacket(p);
      break;
    default:
      break;
  } 
}

static void platformCommandProcess(uint8_t command, uint8_t *data)
{
  SyslinkPacket slp;

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
