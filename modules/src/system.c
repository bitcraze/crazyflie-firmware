/*
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
 * system.c - Top level module implementation
 */
#define DEBUG_MODULE "SYS"

#include <stdbool.h>

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "debug.h"
#include "version.h"
#include "config.h"
#include "param.h"
#include "log.h"
#include "ledseq.h"
#include "pm.h"

#include "config.h"
#include "system.h"
#include "configblock.h"
#include "worker.h"
#include "freeRTOSdebug.h"
#include "uart_syslink.h"
#include "comm.h"
#include "stabilizer.h"
#include "commander.h"
#include "neopixelring.h"
#include "console.h"
#include "usb.h"
#include "expbrd.h"
#include "mem.h"

/* Private variable */
static bool selftestPassed;
static bool canFly;
static bool isInit;

/* System wide synchronisation */
xSemaphoreHandle canStartMutex;

/* Private functions */
static void systemTask(void *arg);

/* Public functions */
void systemLaunch(void)
{
  xTaskCreate(systemTask, (const signed char * const)SYSTEM_TASK_NAME,
              SYSTEM_TASK_STACKSIZE, NULL,
              SYSTEM_TASK_PRI, NULL);

}

//This must be the first module to be initialized!
void systemInit(void)
{
  if(isInit)
    return;

  canStartMutex = xSemaphoreCreateMutex();
  xSemaphoreTake(canStartMutex, portMAX_DELAY);

  configblockInit();
  workerInit();
  //adcInit();
  ledseqInit();
  pmInit();
    
  isInit = true;
}

bool systemTest()
{
  bool pass=isInit;
  
  //pass &= adcTest();
  pass &= ledseqTest();
  pass &= pmTest();
  pass &= workerTest();
  
  return pass;
}

/* Private functions implementation */

void systemTask(void *arg)
{
  bool pass = true;
  
  ledInit();
  ledSet(CHG_LED, 1);

  uartInit();
  //Init the high-levels modules
  systemInit();

#ifndef USE_RADIOLINK_CRTP
#ifdef UART_OUTPUT_TRACE_DATA
  //debugInitTrace();
#endif
#ifdef ENABLE_UART
//  uartInit();
#endif
#endif //ndef USE_RADIOLINK_CRTP

  commInit();

  DEBUG_PRINT("----------------------------\n");
  DEBUG_PRINT("Crazyflie is up and running!\n");
  DEBUG_PRINT("Build %s:%s (%s) %s\n", V_SLOCAL_REVISION,
              V_SREVISION, V_STAG, (V_MODIFIED)?"MODIFIED":"CLEAN");
  DEBUG_PRINT("I am 0x%X%X%X and I have %dKB of flash!\n",
              *((int*)(MCU_ID_ADDRESS+8)), *((int*)(MCU_ID_ADDRESS+4)),
              *((int*)(MCU_ID_ADDRESS+0)), *((short*)(MCU_FLASH_SIZE_ADDRESS)));

  commanderInit();
  stabilizerInit();
  expbrdInit();
  memInit();
  
  //Test the modules
  pass &= systemTest();
  pass &= configblockTest();
  pass &= commTest();
  pass &= commanderTest();
  pass &= stabilizerTest();
  pass &= expbrdTest();
  pass &= memTest();
  
  //Start the firmware
  if(pass)
  {
    selftestPassed = 1;
    systemStart();
    ledseqRun(SYS_LED, seq_alive);
    ledseqRun(LINK_LED, seq_testPassed);
  }
  else
  {
    selftestPassed = 0;
    if (systemTest())
    {
      while(1)
      {
        ledseqRun(SYS_LED, seq_testPassed); //Red passed == not passed!
        vTaskDelay(M2T(2000));
        // System can be forced to start by setting the param to 1 from the cfclient
        if (selftestPassed)
        {
	        DEBUG_PRINT("Start forced.\n");
          systemStart();
          break;
        }
      }
    }
    else
    {
      ledInit();
      ledSet(SYS_LED, true);
    }
  }
  DEBUG_PRINT("Free heap: %d bytes\n", xPortGetFreeHeapSize());
  
  workerLoop();
  
  //Should never reach this point!
  while(1)
    vTaskDelay(portMAX_DELAY);
}


/* Global system variables */
void systemStart()
{
  xSemaphoreGive(canStartMutex);
}

void systemWaitStart(void)
{
  //This permits to guarantee that the system task is initialized before other
  //tasks waits for the start event.
  while(!isInit)
    vTaskDelay(2);

  xSemaphoreTake(canStartMutex, portMAX_DELAY);
  xSemaphoreGive(canStartMutex);
}

void systemSetCanFly(bool val)
{
  canFly = val;
}

bool systemCanFly(void)
{
  return canFly;
}

void vApplicationIdleHook( void )
{
  extern size_t debugPrintTCBInfo(void);
  static uint32_t timeToPrint = M2T(5000);

  if (xTaskGetTickCount() - timeToPrint > M2T(10000))
  {
    timeToPrint = xTaskGetTickCount();
    debugPrintTCBInfo();
  }
  // Enter sleep mode
//  { __asm volatile ("wfi"); }
}

/*System parameters (mostly for test, should be removed from here) */
PARAM_GROUP_START(cpu)
PARAM_ADD(PARAM_UINT16 | PARAM_RONLY, flash, MCU_FLASH_SIZE_ADDRESS)
PARAM_ADD(PARAM_UINT32 | PARAM_RONLY, id0, MCU_ID_ADDRESS+0)
PARAM_ADD(PARAM_UINT32 | PARAM_RONLY, id1, MCU_ID_ADDRESS+4)
PARAM_ADD(PARAM_UINT32 | PARAM_RONLY, id2, MCU_ID_ADDRESS+8)
PARAM_GROUP_STOP(cpu)

PARAM_GROUP_START(system)
PARAM_ADD(PARAM_INT8, selftestPassed, &selftestPassed)
PARAM_GROUP_STOP(sytem)

/* Loggable variables */
LOG_GROUP_START(sys)
LOG_ADD(LOG_INT8, canfly, &canFly)
LOG_GROUP_STOP(sys)
