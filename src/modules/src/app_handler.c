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
#include "zranger2.h"
#include "stabilizer.h"
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

	unsigned long delay = 1*1000;//one second in milliseconds
	unsigned long thrust = 65536;//maximum thrust capable of the motors
	
	logInit();//gain access to log info

	vTaskDelay(delay);

	stateEstimatorInit(getStateEstimator());//struct necessary for initializing the gyro sensors

	vTaskDelay(delay);

	stabilizerInit(getStateEstimator());

	vTaskDelay(delay);

	motorsInit(&motorMapDefaultBrushed[NBR_OF_MOTORS]);
	vTaskDelay(M2T(MOTORS_TEST_ON_TIME_MS));//FUNCTION OF FREERTOS

	deckInit();//Enables use of Decks
	
	struct deckInfo_s* z = (struct deckInfo_s *)deckInfo(0);//zranger deck struct

	zRanger2Init(z);//initializes the zranger

	int zRang = logGetVarId("range", "zrange");//measures the height of the crazyflie
	int roll = logGetVarId("stabilizer","roll");//measures roll of the crazyflie
	int pitch = logGetVarId("stabilizer", "pitch");//measures pitch of the crazyflie

	unsigned long thrust1, thrust2, thrust3, thrust4 = 0;//the thrust each motor will output


	while (1) {//testing the sensors

		thrust1 = 0;
		thrust2 = 0;
		thrust3 = 0;
		thrust4 = 0;

		if (logGetUint(zRang) < 500) {
			thrust1 = .5*thrust;
			thrust2 = .5*thrust;
			thrust3 = .5*thrust;
			thrust4 = .5*thrust;
		}
		if (logGetFloat(roll) > 15 && logGetFloat(roll) < 160) {//if the drone is less than 300 mm off the ground the propellers spin
			thrust1 += .3*thrust;
			thrust2 += .3*thrust;
			thrust3 += 0;
			thrust4 += 0;
		}
		if (logGetFloat(roll) < -15 && logGetFloat(roll) > -160) {//if the drone is less than 300 mm off the ground the propellers spin
			thrust1 += 0;
			thrust2 += 0;
			thrust3 += .3*thrust;
			thrust4 += .3*thrust;
		}
		if (logGetFloat(pitch) < -15 && logGetFloat(pitch) > -160){
			thrust1 += .3*thrust;
			thrust2 += 0;
			thrust3 += 0;
			thrust4 += .3*thrust;
		}
		if (logGetFloat(pitch) > 15 && logGetFloat(pitch) < 160){
			thrust1 += 0;
			thrust2 += .3*thrust;
			thrust3 += .3*thrust;
			thrust4 += 0;
		}
		if (logGetFloat(roll) <= -160 || logGetFloat(roll) >= 160){
			thrust1 = 0*thrust;
			thrust2 = 0*thrust;
			thrust3 = 0*thrust;
			thrust4 = 0*thrust;
			return;
		}
		motorsSetRatio(0, thrust1);
		motorsSetRatio(1, thrust2);
		motorsSetRatio(2, thrust3);
		motorsSetRatio(3, thrust4);
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
