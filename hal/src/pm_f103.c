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
#include "semphr.h"

#include "config.h"
#include "system.h"
#include "pm.h"
#include "led.h"
#include "log.h"
#include "adc.h"
#include "ledseq.h"
#include "commander.h"
#include "nrf24link.h"

static float    batteryVoltage;
static float    batteryVoltageMin = 6.0;
static float    batteryVoltageMax = 0.0;
static int32_t  batteryVRawFilt = PM_BAT_ADC_FOR_3_VOLT;
static int32_t  batteryVRefRawFilt = PM_BAT_ADC_FOR_1p2_VOLT;
static uint32_t batteryLowTimeStamp;
static uint32_t batteryCriticalLowTimeStamp;
static bool isInit;
static PMStates pmState;

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

LOG_GROUP_START(pm)
LOG_ADD(LOG_FLOAT, vbat, &batteryVoltage)
LOG_ADD(LOG_INT8, state, &pmState)
LOG_GROUP_STOP(pm)

void pmInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  if(isInit)
    return;

  RCC_AHBPeriphClockCmd(PM_GPIO_IN_PGOOD_PERIF | PM_GPIO_IN_CHG_PERIF |
                         PM_GPIO_SYSOFF_PERIF | PM_GPIO_EN1_PERIF | 
                         PM_GPIO_EN2_PERIF | PM_GPIO_BAT_PERIF, ENABLE);

  // Configure PM PGOOD pin (Power good)
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Pin = PM_GPIO_IN_PGOOD;
  GPIO_Init(PM_GPIO_IN_PGOOD_PORT, &GPIO_InitStructure);
  // Configure PM CHG pin (Charge)
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Pin = PM_GPIO_IN_CHG;
  GPIO_Init(PM_GPIO_IN_CHG_PORT, &GPIO_InitStructure);
  // Configure PM EN2 pin
  GPIO_InitStructure.GPIO_Pin = PM_GPIO_EN2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_Init(PM_GPIO_EN2_PORT, &GPIO_InitStructure);
  // Configure PM EN1 pin
  GPIO_InitStructure.GPIO_Pin = PM_GPIO_EN1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_Init(PM_GPIO_EN1_PORT, &GPIO_InitStructure);
  // Configure PM SYSOFF pin
  GPIO_InitStructure.GPIO_Pin = PM_GPIO_SYSOFF;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_Init(PM_GPIO_SYSOFF_PORT, &GPIO_InitStructure);
  // Configure battery ADC pin
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Pin = PM_GPIO_BAT;
  GPIO_Init(PM_GPIO_BAT_PORT, &GPIO_InitStructure);
  
  xTaskCreate(pmTask, (const signed char * const)PM_TASK_NAME,
              PM_TASK_STACKSIZE, NULL, PM_TASK_PRI, NULL);
  
  isInit = true;

  pmSetBatteryVoltage(3.7f); //TODO remove
}

bool pmTest(void)
{
  return isInit;
}

/**
 * IIR low pass filter the samples.
 */
static int16_t pmBatteryIIRLPFilter(uint16_t in, int32_t* filt)
{
  int32_t inScaled;
  int32_t filttmp = *filt;
  int16_t out;

  // Shift to keep accuracy
  inScaled = in << PM_BAT_IIR_SHIFT;
  // Calculate IIR filter
  filttmp = filttmp + (((inScaled-filttmp) >> 8) * PM_BAT_IIR_LPF_ATT_FACTOR);
  // Scale and round
  out = (filttmp >> 8) + ((filttmp & (1 << (PM_BAT_IIR_SHIFT - 1))) >> (PM_BAT_IIR_SHIFT - 1));
  *filt = filttmp;

  return out;
}

/**
 * Sets the battery voltage and its min and max values
 */
static void pmSetBatteryVoltage(float voltage)
{
  batteryVoltage = voltage;
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
  GPIO_SetBits(PM_GPIO_SYSOFF_PORT, PM_GPIO_SYSOFF);
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

void pmBatteryUpdate(AdcGroup* adcValues)
{
  float vBat;
  int16_t vBatRaw;
  int16_t vBatRefRaw;

  vBatRaw = pmBatteryIIRLPFilter(adcValues->vbat.val, &batteryVRawFilt);
  vBatRefRaw = pmBatteryIIRLPFilter(adcValues->vbat.vref, &batteryVRefRawFilt);

  vBat = adcConvertToVoltageFloat(vBatRaw, vBatRefRaw) * PM_BAT_DIVIDER;
  pmSetBatteryVoltage(vBat);
}

void pmSetChargeState(PMChargeStates chgState)
{
  switch (chgState)
  {
    case charge100mA:
      GPIO_ResetBits(PM_GPIO_EN1_PORT, PM_GPIO_EN1);
      GPIO_ResetBits(PM_GPIO_EN2_PORT, PM_GPIO_EN2);
      break;
    case charge500mA:
      GPIO_SetBits(PM_GPIO_EN1_PORT, PM_GPIO_EN1);
      GPIO_ResetBits(PM_GPIO_EN2_PORT, PM_GPIO_EN2);
      break;
    case chargeMax:
      GPIO_ResetBits(PM_GPIO_EN1_PORT, PM_GPIO_EN1);
      GPIO_SetBits(PM_GPIO_EN2_PORT, PM_GPIO_EN2);
      break;
  }
}

PMStates pmUpdateState()
{
  PMStates state;
  bool isCharging = !GPIO_ReadInputDataBit(PM_GPIO_IN_CHG_PORT, PM_GPIO_IN_CHG);
  bool isPgood = !GPIO_ReadInputDataBit(PM_GPIO_IN_PGOOD_PORT, PM_GPIO_IN_PGOOD);
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


// return true if battery discharging
bool pmIsDischarging(void) {
    PMStates pmState;
    pmState = pmUpdateState();
    return (pmState == lowPower )|| (pmState == battery);
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

  vTaskDelay(1000);

  while(1)
  {
    vTaskDelay(100);
    tickCount = xTaskGetTickCount();

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
          ledseqStop(LED_GREEN, seq_charging);
          ledseqRun(LED_GREEN, seq_charged);
          systemSetCanFly(false);
          break;
        case charging:
          ledseqStop(LED_RED, seq_lowbat);
          ledseqStop(LED_GREEN, seq_charged);
          ledseqRun(LED_GREEN, seq_charging);
          systemSetCanFly(false);
          //Due to voltage change radio must be restarted
          nrf24linkReInit();
          break;
        case lowPower:
          ledseqRun(LED_RED, seq_lowbat);
          systemSetCanFly(true);
          break;
        case battery:
          ledseqStop(LED_GREEN, seq_charging);
          ledseqStop(LED_GREEN, seq_charged);
          systemSetCanFly(true);
          //Due to voltage change radio must be restarted
          nrf24linkReInit();
          break;
        default:
          systemSetCanFly(true);
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
          uint32_t onTime;

          onTime = pmBatteryChargeFromVoltage(pmGetBatteryVoltage()) *
                   (LEDSEQ_CHARGE_CYCLE_TIME / 10);
          ledseqSetTimes(seq_charging, onTime, LEDSEQ_CHARGE_CYCLE_TIME - onTime);
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
