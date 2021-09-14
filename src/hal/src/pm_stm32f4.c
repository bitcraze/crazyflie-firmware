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
 * pm.c - Power Management driver and functions.
 */

#include "stm32fxxx.h"
#include <string.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"

#include "config.h"
#include "system.h"
#include "pm.h"
#include "led.h"
#include "log.h"
#include "ledseq.h"
#include "commander.h"
#include "sound.h"
#include "deck.h"
#include "static_mem.h"
#include "worker.h"

typedef struct _PmSyslinkInfo
{
  union
  {
    uint8_t flags;
    struct
    {
      uint8_t chg    : 1;
      uint8_t pgood  : 1;
      uint8_t unused : 6;
    };
  };
  float vBat;
  float chargeCurrent;
#ifdef PM_SYSTLINK_INLCUDE_TEMP
  float temp;
#endif
}  __attribute__((packed)) PmSyslinkInfo;

static float     batteryVoltage;
static uint16_t  batteryVoltageMV;
static float     batteryVoltageMin = 6.0;
static float     batteryVoltageMax = 0.0;

static float     extBatteryVoltage;
static uint16_t  extBatteryVoltageMV;
static deckPin_t extBatVoltDeckPin;
static bool      isExtBatVoltDeckPinSet = false;
static float     extBatVoltMultiplier;
static float     extBatteryCurrent;
static deckPin_t extBatCurrDeckPin;
static bool      isExtBatCurrDeckPinSet = false;
static float     extBatCurrAmpPerVolt;

#ifdef PM_SYSTLINK_INLCUDE_TEMP
// nRF51 internal temp
static float    temp;
#endif

static uint32_t batteryLowTimeStamp;
static uint32_t batteryCriticalLowTimeStamp;
static bool isInit;
static PMStates pmState;
static PmSyslinkInfo pmSyslinkInfo;

static uint8_t batteryLevel;

static void pmSetBatteryVoltage(float voltage);

const static float bat671723HS25C[10] =
{
  3.00, // 00%
  3.78, // 10%
  3.83, // 20%
  3.87, // 30%
  3.89, // 40%
  3.92, // 50%
  3.96, // 60%
  4.00, // 70%
  4.04, // 80%
  4.10  // 90%
};

STATIC_MEM_TASK_ALLOC(pmTask, PM_TASK_STACKSIZE);

void pmInit(void)
{
  if(isInit) {
    return;
  }

  STATIC_MEM_TASK_CREATE(pmTask, pmTask, PM_TASK_NAME, NULL, PM_TASK_PRI);

  isInit = true;

  pmSyslinkInfo.vBat = 3.7f;
  pmSetBatteryVoltage(pmSyslinkInfo.vBat); //TODO remove
}

bool pmTest(void)
{
  return isInit;
}

/**
 * Sets the battery voltage and its min and max values
 */
static void pmSetBatteryVoltage(float voltage)
{
  batteryVoltage = voltage;
  batteryVoltageMV = (uint16_t)(voltage * 1000);
  if (batteryVoltageMax < voltage)
  {
    batteryVoltageMax = voltage;
  }
  if (batteryVoltageMin > voltage)
  {
    batteryVoltageMin = voltage;
  }
}

/**
 * Shutdown system
 */
static void pmSystemShutdown(void)
{
#ifdef ACTIVATE_AUTO_SHUTDOWN
  systemRequestShutdown();
#endif
}

/**
 * Returns a number from 0 to 9 where 0 is completely discharged
 * and 9 is 90% charged.
 */
static int32_t pmBatteryChargeFromVoltage(float voltage)
{
  int charge = 0;

  if (voltage < bat671723HS25C[0])
  {
    return 0;
  }
  if (voltage > bat671723HS25C[9])
  {
    return 9;
  }
  while (voltage >  bat671723HS25C[charge])
  {
    charge++;
  }

  return charge;
}


float pmGetBatteryVoltage(void)
{
  return batteryVoltage;
}

float pmGetBatteryVoltageMin(void)
{
  return batteryVoltageMin;
}

float pmGetBatteryVoltageMax(void)
{
  return batteryVoltageMax;
}

/*
 * When a module wants to register a callback to be called on shutdown they
 * call pmRegisterGracefulShutdownCallback(graceful_shutdown_callback_t),
 * with a function they which to be run at shutdown. We currently support
 * GRACEFUL_SHUTDOWN_MAX_CALLBACKS number of callbacks to be registred.
 */
#define GRACEFUL_SHUTDOWN_MAX_CALLBACKS 5
static int graceful_shutdown_callbacks_index;
static graceful_shutdown_callback_t graceful_shutdown_callbacks[GRACEFUL_SHUTDOWN_MAX_CALLBACKS];

/*
 * Please take care in your callback, do not take to long time the nrf
 * will not wait for you, it will shutdown.
 */
bool pmRegisterGracefulShutdownCallback(graceful_shutdown_callback_t cb)
{
  // To many registered allready! Increase limit if you think you are important
  // enough!
  if (graceful_shutdown_callbacks_index >= GRACEFUL_SHUTDOWN_MAX_CALLBACKS) {
    return false;
  }

  graceful_shutdown_callbacks[graceful_shutdown_callbacks_index] = cb;
  graceful_shutdown_callbacks_index += 1;

  return true;
}

/*
 * Iterate through all registered shutdown callbacks and call them one after
 * the other, when all is done, send the ACK back to nrf to allow power off.
 */
static void pmGracefulShutdown()
{
  for (int i = 0; i < graceful_shutdown_callbacks_index; i++) {
    graceful_shutdown_callback_t callback = graceful_shutdown_callbacks[i];

    callback();
  }

  SyslinkPacket slp = {
    .type = SYSLINK_PM_SHUTDOWN_ACK,
  };

  syslinkSendPacket(&slp);
}

void pmSyslinkUpdate(SyslinkPacket *slp)
{
  if (slp->type == SYSLINK_PM_BATTERY_STATE) {
    memcpy(&pmSyslinkInfo, &slp->data[0], sizeof(pmSyslinkInfo));
    pmSetBatteryVoltage(pmSyslinkInfo.vBat);
#ifdef PM_SYSTLINK_INLCUDE_TEMP
    temp = pmSyslinkInfo.temp;
#endif
  } else if (slp->type == SYSLINK_PM_SHUTDOWN_REQUEST) {
    workerSchedule(pmGracefulShutdown, NULL);
  }
}

void pmSetChargeState(PMChargeStates chgState)
{
  // TODO: Send syslink packafe with charge state
}

PMStates pmUpdateState()
{
  PMStates state;
  bool isCharging = pmSyslinkInfo.chg;
  bool isPgood = pmSyslinkInfo.pgood;
  uint32_t batteryLowTime;

  batteryLowTime = xTaskGetTickCount() - batteryLowTimeStamp;

  if (isPgood && !isCharging)
  {
    state = charged;
  }
  else if (isPgood && isCharging)
  {
    state = charging;
  }
  else if (!isPgood && !isCharging && (batteryLowTime > PM_BAT_LOW_TIMEOUT))
  {
    state = lowPower;
  }
  else
  {
    state = battery;
  }

  return state;
}

void pmEnableExtBatteryCurrMeasuring(const deckPin_t pin, float ampPerVolt)
{
  extBatCurrDeckPin = pin;
  isExtBatCurrDeckPinSet = true;
  extBatCurrAmpPerVolt = ampPerVolt;
}

float pmMeasureExtBatteryCurrent(void)
{
  float current;

  if (isExtBatCurrDeckPinSet)
  {
    current = analogReadVoltage(extBatCurrDeckPin) * extBatCurrAmpPerVolt;
  }
  else
  {
    current = 0.0;
  }

  return current;
}

void pmEnableExtBatteryVoltMeasuring(const deckPin_t pin, float multiplier)
{
  extBatVoltDeckPin = pin;
  isExtBatVoltDeckPinSet = true;
  extBatVoltMultiplier = multiplier;
}

float pmMeasureExtBatteryVoltage(void)
{
  float voltage;

  if (isExtBatVoltDeckPinSet)
  {
    voltage = analogReadVoltage(extBatVoltDeckPin) * extBatVoltMultiplier;
  }
  else
  {
    voltage = 0.0;
  }

  return voltage;
}

bool pmIsBatteryLow(void) {
  return (pmState == lowPower);
}

bool pmIsChargerConnected(void) {
  return (pmState == charging) || (pmState == charged);
}

bool pmIsCharging(void) {
  return (pmState == charging);
}

// return true if battery discharging
bool pmIsDischarging(void) {
  return (pmState == lowPower) || (pmState == battery);
}

void pmTask(void *param)
{
  PMStates pmStateOld = battery;
  uint32_t tickCount;

  vTaskSetApplicationTaskTag(0, (void*)TASK_PM_ID_NBR);

  tickCount = xTaskGetTickCount();
  batteryLowTimeStamp = tickCount;
  batteryCriticalLowTimeStamp = tickCount;

  pmSetChargeState(charge500mA);
  systemWaitStart();

  while(1)
  {
    vTaskDelay(100);
    tickCount = xTaskGetTickCount();

    extBatteryVoltage = pmMeasureExtBatteryVoltage();
    extBatteryVoltageMV = (uint16_t)(extBatteryVoltage * 1000);
    extBatteryCurrent = pmMeasureExtBatteryCurrent();
    batteryLevel = pmBatteryChargeFromVoltage(pmGetBatteryVoltage()) * 10;

    if (pmGetBatteryVoltage() > PM_BAT_LOW_VOLTAGE)
    {
      batteryLowTimeStamp = tickCount;
    }
    if (pmGetBatteryVoltage() > PM_BAT_CRITICAL_LOW_VOLTAGE)
    {
      batteryCriticalLowTimeStamp = tickCount;
    }

    pmState = pmUpdateState();

    if (pmState != pmStateOld)
    {
      // Actions on state change
      switch (pmState)
      {
        case charged:
          ledseqStop(&seq_charging);
          ledseqRunBlocking(&seq_charged);
          soundSetEffect(SND_BAT_FULL);
          break;
        case charging:
          ledseqStop(&seq_lowbat);
          ledseqStop(&seq_charged);
          ledseqRunBlocking(&seq_charging);
          soundSetEffect(SND_USB_CONN);
          break;
        case lowPower:
          ledseqRunBlocking(&seq_lowbat);
          soundSetEffect(SND_BAT_LOW);
          break;
        case battery:
          ledseqRunBlocking(&seq_charging);
          ledseqRun(&seq_charged);
          soundSetEffect(SND_USB_DISC);
          break;
        default:
          break;
      }
      pmStateOld = pmState;
    }
    // Actions during state
    switch (pmState)
    {
      case charged:
        break;
      case charging:
        {
          // Charge level between 0.0 and 1.0
          float chargeLevel = pmBatteryChargeFromVoltage(pmGetBatteryVoltage()) / 10.0f;
          ledseqSetChargeLevel(chargeLevel);
        }
        break;
      case lowPower:
        {
          uint32_t batteryCriticalLowTime;

          batteryCriticalLowTime = tickCount - batteryCriticalLowTimeStamp;
          if (batteryCriticalLowTime > PM_BAT_CRITICAL_LOW_TIMEOUT)
          {
            pmSystemShutdown();
          }
        }
        break;
      case battery:
        {
          if ((commanderGetInactivityTime() > PM_SYSTEM_SHUTDOWN_TIMEOUT))
          {
            pmSystemShutdown();
          }
        }
        break;
      default:
        break;
    }
  }
}

/**
 * Power management log variables.
 */
LOG_GROUP_START(pm)
/**
 * @brief Battery voltage [V]
 */
LOG_ADD_CORE(LOG_FLOAT, vbat, &batteryVoltage)
/**
 * @brief Battery voltage [mV]
 */
LOG_ADD(LOG_UINT16, vbatMV, &batteryVoltageMV)
/**
 * @brief BigQuad external voltage measurement [V]
 */
LOG_ADD(LOG_FLOAT, extVbat, &extBatteryVoltage)
/**
 * @brief BigQuad external voltage measurement [mV]
 */
LOG_ADD(LOG_UINT16, extVbatMV, &extBatteryVoltageMV)
/**
 * @brief BigQuad external current measurement [V]
 */
LOG_ADD(LOG_FLOAT, extCurr, &extBatteryCurrent)
/**
 * @brief Battery charge current [A]
 */
LOG_ADD(LOG_FLOAT, chargeCurrent, &pmSyslinkInfo.chargeCurrent)
/**
 * @brief State of power management
 *
 * | State | Meaning   | \n
 * | -     | -         | \n
 * | 0     | Battery   | \n
 * | 1     | Charging  | \n
 * | 2     | Charged   | \n
 * | 3     | Low power | \n
 * | 4     | Shutdown  | \n
 */
LOG_ADD_CORE(LOG_INT8, state, &pmState)
/**
 * @brief Battery charge level [%]
 */
LOG_ADD_CORE(LOG_UINT8, batteryLevel, &batteryLevel)
#ifdef PM_SYSTLINK_INCLUDE_TEMP
/**
 * @brief Temperature from nrf51 [degrees]
 */
LOG_ADD(LOG_FLOAT, temp, &temp)
#endif
LOG_GROUP_STOP(pm)
