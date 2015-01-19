/**
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2015 Bitcraze AB
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
 * mem.c - Provies compatibility with Crazyflie 2.0 memory port. This
 *         will always return 0 memories and no information is requested.
 */

#include <stdbool.h>

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"

#include "crtp.h"
#include "mem.h"

static bool isInit=false;

#define SETTINGS_CH     0

#define CMD_GET_NBR     1

void memHandler(CRTPPacket *p);

static void memSettingsProcess(CRTPPacket *p);

void memInit(void)
{
  if (isInit)
    return;

  // Register a callback to service the memory port
  crtpRegisterPortCB(CRTP_PORT_MEM, memHandler);
  
  isInit = true;
}

bool memTest(void)
{
  return isInit;
}

void memHandler(CRTPPacket *p)
{
  switch (p->channel)
  {
    case SETTINGS_CH:
      memSettingsProcess(p);
      break;
    default:
      break;
  } 
}

static void memSettingsProcess(CRTPPacket *p)
{
  switch (p->data[0])
  {
    case CMD_GET_NBR:
      p->header = CRTP_HEADER(CRTP_PORT_MEM, SETTINGS_CH);
      p->size = 2;
      p->data[0] = CMD_GET_NBR;
      p->data[1] = 0;
      crtpSendPacket(p);
      break;
  }
}
