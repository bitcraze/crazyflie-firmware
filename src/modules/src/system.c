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
#include "platform.h"
#include "storage.h"
#include "configblock.h"
#include "worker.h"
#include "freeRTOSdebug.h"
#include "uart_syslink.h"
#include "uart1.h"
#include "uart2.h"
#include "comm.h"
#include "stabilizer.h"
#include "commander.h"
#include "console.h"
#include "usblink.h"
#include "mem.h"
#include "proximity.h"
#include "watchdog.h"
#include "queuemonitor.h"
#include "buzzer.h"
#include "sound.h"
#include "sysload.h"
#include "estimator_kalman.h"
#include "deck.h"
#include "extrx.h"
#include "app.h"
#include "static_mem.h"
#include "peer_localization.h"
#include "cfassert.h"
#include "i2cdev.h"

#ifndef START_DISARMED
#define ARM_INIT true
#else
#define ARM_INIT false
#endif

/* Private variable */
static bool selftestPassed;
static bool armed = ARM_INIT;
static bool forceArm;
static bool isInit;

STATIC_MEM_TASK_ALLOC(systemTask, SYSTEM_TASK_STACKSIZE);

/* System wide synchronisation */
xSemaphoreHandle canStartMutex;
static StaticSemaphore_t canStartMutexBuffer;

/* Private functions */
static void systemTask(void *arg);

/* Public functions */
void systemLaunch(void)
{
  STATIC_MEM_TASK_CREATE(systemTask, systemTask, SYSTEM_TASK_NAME, NULL, SYSTEM_TASK_PRI);
}

// This must be the first module to be initialized!
void systemInit(void)
{
  if(isInit)
    return;

  canStartMutex = xSemaphoreCreateMutexStatic(&canStartMutexBuffer);
  xSemaphoreTake(canStartMutex, portMAX_DELAY);

  usblinkInit();
  sysLoadInit();

  /* Initialized here so that DEBUG_PRINT (buffered) can be used early */
  debugInit();
  crtpInit();
  consoleInit();

  DEBUG_PRINT("----------------------------\n");
  DEBUG_PRINT("%s is up and running!\n", platformConfigGetDeviceTypeName());

  if (V_PRODUCTION_RELEASE) {
    DEBUG_PRINT("Production release %s\n", V_STAG);
  } else {
    DEBUG_PRINT("Build %s:%s (%s) %s\n", V_SLOCAL_REVISION,
                V_SREVISION, V_STAG, (V_MODIFIED)?"MODIFIED":"CLEAN");
  }
  DEBUG_PRINT("I am 0x%08X%08X%08X and I have %dKB of flash!\n",
              *((int*)(MCU_ID_ADDRESS+8)), *((int*)(MCU_ID_ADDRESS+4)),
              *((int*)(MCU_ID_ADDRESS+0)), *((short*)(MCU_FLASH_SIZE_ADDRESS)));

  configblockInit();
  storageInit();
  workerInit();
  adcInit();
  ledseqInit();
  pmInit();
  buzzerInit();
  peerLocalizationInit();

#ifdef APP_ENABLED
  appInit();
#endif

  isInit = true;
}

bool systemTest()
{
  bool pass=isInit;

  pass &= ledseqTest();
  pass &= pmTest();
  pass &= workerTest();
  pass &= buzzerTest();
  return pass;
}

/* Private functions implementation */

void systemTask(void *arg)
{
  bool pass = true;

  ledInit();
  ledSet(CHG_LED, 1);

#ifdef DEBUG_QUEUE_MONITOR
  queueMonitorInit();
#endif

#ifdef ENABLE_UART1
  uart1Init(9600);
#endif
#ifdef ENABLE_UART2
  uart2Init(115200);
#endif

  initUsecTimer();
  i2cdevInit(I2C3_DEV);
  i2cdevInit(I2C1_DEV);

  //Init the high-levels modules
  systemInit();
  commInit();
  commanderInit();

  StateEstimatorType estimator = anyEstimator;
  estimatorKalmanTaskInit();
  deckInit();
  estimator = deckGetRequiredEstimator();
  stabilizerInit(estimator);
  if (deckGetRequiredLowInterferenceRadioMode() && platformConfigPhysicalLayoutAntennasAreClose())
  {
    platformSetLowInterferenceRadioMode();
  }
  soundInit();
  memInit();

#ifdef PROXIMITY_ENABLED
  proximityInit();
#endif

  //Test the modules
  DEBUG_PRINT("About to run tests in system.c.\n");
  if (systemTest() == false) {
    pass = false;
    DEBUG_PRINT("system [FAIL]\n");
  }
  if (configblockTest() == false) {
    pass = false;
    DEBUG_PRINT("configblock [FAIL]\n");
  }
  if (storageTest() == false) {
    pass = false;
    DEBUG_PRINT("storage [FAIL]\n");
  }
  if (commTest() == false) {
    pass = false;
    DEBUG_PRINT("comm [FAIL]\n");
  }
  if (commanderTest() == false) {
    pass = false;
    DEBUG_PRINT("commander [FAIL]\n");
  }
  if (stabilizerTest() == false) {
    pass = false;
    DEBUG_PRINT("stabilizer [FAIL]\n");
  }
  if (estimatorKalmanTaskTest() == false) {
    pass = false;
    DEBUG_PRINT("estimatorKalmanTask [FAIL]\n");
  }
  if (deckTest() == false) {
    pass = false;
    DEBUG_PRINT("deck [FAIL]\n");
  }
  if (soundTest() == false) {
    pass = false;
    DEBUG_PRINT("sound [FAIL]\n");
  }
  if (memTest() == false) {
    pass = false;
    DEBUG_PRINT("mem [FAIL]\n");
  }
  if (watchdogNormalStartTest() == false) {
    pass = false;
    DEBUG_PRINT("watchdogNormalStart [FAIL]\n");
  }
  if (cfAssertNormalStartTest() == false) {
    pass = false;
    DEBUG_PRINT("cfAssertNormalStart [FAIL]\n");
  }
  if (peerLocalizationTest() == false) {
    pass = false;
    DEBUG_PRINT("peerLocalization [FAIL]\n");
  }

  //Start the firmware
  if(pass)
  {
    DEBUG_PRINT("Self test passed!\n");
    selftestPassed = 1;
    systemStart();
    soundSetEffect(SND_STARTUP);
    ledseqRun(&seq_alive);
    ledseqRun(&seq_testPassed);
  }
  else
  {
    selftestPassed = 0;
    if (systemTest())
    {
      while(1)
      {
        ledseqRun(&seq_testFailed);
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
#ifndef DEBUG
  watchdogInit();
#endif
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

void systemSetArmed(bool val)
{
  armed = val;
}

bool systemIsArmed()
{

  return armed || forceArm;
}

void vApplicationIdleHook( void )
{
  static uint32_t tickOfLatestWatchdogReset = M2T(0);

  portTickType tickCount = xTaskGetTickCount();

  if (tickCount - tickOfLatestWatchdogReset > M2T(WATCHDOG_RESET_PERIOD_MS))
  {
    tickOfLatestWatchdogReset = tickCount;
    watchdogReset();
  }

  // Enter sleep mode. Does not work when debugging chip with SWD.
  // Currently saves about 20mA STM32F405 current consumption (~30%).
#ifndef DEBUG
  { __asm volatile ("wfi"); }
#endif
}

/**
 * This parameter group contain read-only parameters pertaining to the CPU
 * in the Crazyflie.
 *
 * These could be used to identify an unique quad.
 */
PARAM_GROUP_START(cpu)

/**
 * @brief Size in kB of the device flash memory
 */
PARAM_ADD_CORE(PARAM_UINT16 | PARAM_RONLY, flash, MCU_FLASH_SIZE_ADDRESS)

/**
 * @brief Byte `0 - 3` of device unique id
 */
PARAM_ADD_CORE(PARAM_UINT32 | PARAM_RONLY, id0, MCU_ID_ADDRESS+0)

/**
 * @brief Byte `4 - 7` of device unique id
 */
PARAM_ADD_CORE(PARAM_UINT32 | PARAM_RONLY, id1, MCU_ID_ADDRESS+4)

/**
 * @brief Byte `8 - 11` of device unique id
 */
PARAM_ADD_CORE(PARAM_UINT32 | PARAM_RONLY, id2, MCU_ID_ADDRESS+8)

PARAM_GROUP_STOP(cpu)

PARAM_GROUP_START(system)

/**
 * @brief All tests passed when booting
 */
PARAM_ADD_CORE(PARAM_INT8 | PARAM_RONLY, selftestPassed, &selftestPassed)

/**
 * @brief Set to nonzero to force system to be armed
 */
PARAM_ADD(PARAM_INT8, forceArm, &forceArm)

PARAM_GROUP_STOP(sytem)

/**
 *  System loggable variables to check different system states.
 */
LOG_GROUP_START(sys)
/**
 * @brief If zero, arming system is preventing motors to start
 */
LOG_ADD(LOG_INT8, armed, &armed)
LOG_GROUP_STOP(sys)
