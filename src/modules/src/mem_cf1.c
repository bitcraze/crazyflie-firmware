/**
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
 * mem.c: Memory module. Handles one-wire and eeprom memory functions over crtp link.
 */

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"

#include "mem.h"
#include "crtp.h"

//Private functions
static void memTask(void * prm);
static void memSettingsProcess(int command);
static void createNbrResponse(CRTPPacket* p);

static bool isInit = false;
static CRTPPacket p;

void memInit(void)
{
  if(isInit)
    return;

  isInit = true;

  //Start the mem task
  xTaskCreate(memTask, MEM_TASK_NAME,
              MEM_TASK_STACKSIZE, NULL, MEM_TASK_PRI, NULL);
}

bool memTest(void)
{
  return isInit;
}

void memTask(void * param)
{
	crtpInitTaskQueue(CRTP_PORT_MEM);
	
	while(1)
	{
		crtpReceivePacketBlock(CRTP_PORT_MEM, &p);

		switch (p.channel)
		{
      case MEM_SETTINGS_CH:
        memSettingsProcess(p.data[0]);
        break;
      default:
        break;
		}
	}
}

void memSettingsProcess(int command)
{
  switch (command)
  {
    case MEM_CMD_GET_NBR:
      createNbrResponse(&p);
      crtpSendPacket(&p);
      break;
  }
}

void createNbrResponse(CRTPPacket* p)
{
  p->header = CRTP_HEADER(CRTP_PORT_MEM, MEM_SETTINGS_CH);
  p->size = 2;
  p->data[0] = MEM_CMD_GET_NBR;
  p->data[1] = 0;
}
