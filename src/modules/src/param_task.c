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
 * param_task.c - Crazy parameter system task source file.
 */

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"

#include "config.h"
#include "param_task.h"
#include "param_logic.h"
#include "debug.h"
#include "static_mem.h"

#include <string.h>

#if 0
#define PARAM_DEBUG(fmt, ...) DEBUG_PRINT("D/param " fmt, ## __VA_ARGS__)
#define PARAM_ERROR(fmt, ...) DEBUG_PRINT("E/param " fmt, ## __VA_ARGS__)
#else
#define PARAM_DEBUG(...)
#define PARAM_ERROR(...)
#endif

//Private functions
static void paramTask(void * prm);

static bool isInit = false;
static CRTPPacket p;

STATIC_MEM_TASK_ALLOC(paramTask, PARAM_TASK_STACKSIZE);


void paramInit(void)
{
  if(isInit) {
    return;
  }

  paramLogicInit();
  paramLogicStorageInit();

  //Start the param task
  STATIC_MEM_TASK_CREATE(paramTask, paramTask, PARAM_TASK_NAME, NULL, PARAM_TASK_PRI);

  isInit = true;
}

bool paramTest(void)
{
  return isInit;
}

void paramTask(void * prm)
{
	crtpInitTaskQueue(CRTP_PORT_PARAM);

	while(1) {
		crtpReceivePacketBlock(CRTP_PORT_PARAM, &p);

		if (p.channel==TOC_CH)
		  paramTOCProcess(&p, p.data[0]);
	  else if (p.channel==READ_CH)
		  paramReadProcess(&p);
		else if (p.channel==WRITE_CH)
		  paramWriteProcess(&p);
    else if (p.channel==MISC_CH) {
      switch (p.data[0]) {
        case MISC_SETBYNAME:
          paramSetByName(&p);
          break;
        case MISC_GET_EXTENDED_TYPE:
          paramGetExtendedType(&p);
          break;
        case MISC_PERSISTENT_STORE:
          paramPersistentStore(&p);
          break;
        case MISC_PERSISTENT_GET_STATE:
          paramPersistentGetState(&p);
          break;
        case MISC_PERSISTENT_CLEAR:
          paramPersistentClear(&p);
          break;
        case MISC_GET_DEFAULT_VALUE:
          paramGetDefaultValue(&p);
          break;
        default:
          break;
      }
    }
	}
}
