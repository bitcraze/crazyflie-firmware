/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * LPS node firmware.
 *
 * Copyright 2019, Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Foobar is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
 */
/* app_handler.c: App layer handling function implementation */

#include <stdbool.h>
#include <stdint.h>

#include "motors.h"
#include "bmi088_defs.h"
#include "bmi088.h"
#include "zranger2.h"
#include "deck.h"
#include "deck_core.h"
#include "log.h"
#include "FreeRTOS.h"
#include "task.h"

#include "system.h"
#include "static_mem.h"

#include "app.h"

#ifndef APP_STACKSIZE
#define APP_STACKSIZE 400
#endif

#ifndef APP_PRIORITY
#define APP_PRIORITY 0
#endif

static bool isInit = false;

STATIC_MEM_TASK_ALLOC(appTask, APP_STACKSIZE);

static void appTask(void *param);

void appMain(){

	//uint8_t y = 0;
	//uint16_t len = 8;
	unsigned long delay = 1*1000;
	unsigned long thrust = 65536;//maximum thrust capable of the motors
	
//	for (int ix = 0; ix < 2; ix++){
//		motorsTest();
//		for (int ix2 = 0; ix2 < delay; ix2++);
//	}
	logInit();//gain access to log info

	vTaskDelay(delay);

	motorsInit(&motorMapDefaultBrushed[NBR_OF_MOTORS]);
	vTaskDelay(M2T(MOTORS_TEST_ON_TIME_MS));//FUNCTION OF FREERTOS
	

	vTaskDelay(M2T(delay));

	deckInit();//Enables use of Decks
	
	struct deckInfo_s* z = (struct deckInfo_s *)deckInfo(0);//zranger deck struct

	zRanger2Init(z);

	int zRang = logGetVarId("range", "zrange");


	while (1) {//testing the sensors
		if (logGetUint(zRang) <300) {//if the drone is less than 300 mm off the ground the propellers spin
			motorsSetRatio(0, .2*thrust);
			motorsSetRatio(1, .2*thrust);
			motorsSetRatio(2, .2*thrust);
			motorsSetRatio(3, .2*thrust);
		}
	}

	vTaskDelay(M2T(1*delay));//runs engines for delay milliseconds seconds
	motorsSetRatio(0, 0);
	motorsSetRatio(1, 0);
	motorsSetRatio(2, 0);
	motorsSetRatio(3, 0);
	
}

void __attribute__((weak)) appInit()
{
  if (isInit) {
    return;
  }

  STATIC_MEM_TASK_CREATE(appTask, appTask, "app", NULL, APP_PRIORITY);
  isInit = true;
}

static void appTask(void *param)
{
  systemWaitStart();

  appMain();

  while(1) {
    vTaskDelay(portMAX_DELAY);
  }
}
