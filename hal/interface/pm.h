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
 * pm.h - Power Management driver and functions.
 */

#ifndef PM_H_
#define PM_H_
#include "adc.h"

// Power managment pins
#define PM_GPIO_SYSOFF_PERIF    RCC_APB2Periph_GPIOA
#define PM_GPIO_SYSOFF_PORT     GPIOA
#define PM_GPIO_SYSOFF          GPIO_Pin_1

#define PM_GPIO_EN1_PERIF       RCC_APB2Periph_GPIOC
#define PM_GPIO_EN1_PORT        GPIOC
#define PM_GPIO_EN1             GPIO_Pin_13

#define PM_GPIO_EN2_PERIF       RCC_APB2Periph_GPIOA
#define PM_GPIO_EN2_PORT        GPIOA
#define PM_GPIO_EN2             GPIO_Pin_2

#define PM_GPIO_IN_CHG_PERIF    RCC_APB2Periph_GPIOB
#define PM_GPIO_IN_CHG_PORT     GPIOB
#define PM_GPIO_IN_CHG          GPIO_Pin_2

#define PM_GPIO_IN_PGOOD_PERIF  RCC_APB2Periph_GPIOC
#define PM_GPIO_IN_PGOOD_PORT   GPIOC
#define PM_GPIO_IN_PGOOD        GPIO_Pin_15

// Power managment pins
#define PM_GPIO_BAT_PERIF       RCC_APB2Periph_GPIOA
#define PM_GPIO_BAT_PORT        GPIOA
#define PM_GPIO_BAT             GPIO_Pin_3

#define PM_BAT_CRITICAL_LOW_VOLTAGE   3.0
#define PM_BAT_CRITICAL_LOW_TIMEOUT   M2T(1000 * 5) // 5 sec
#define PM_BAT_LOW_VOLTAGE            3.2
#define PM_BAT_LOW_TIMEOUT            M2T(1000 * 5) // 5 sec

#define PM_BAT_DIVIDER                (float)(3.0)
#define PM_BAT_ADC_FOR_3_VOLT         (int32_t)(((3.0 / PM_BAT_DIVIDER) / 2.8) * 4096)
#define PM_BAT_ADC_FOR_1p2_VOLT       (int32_t)(((1.2 / PM_BAT_DIVIDER) / 2.8) * 4096)

#define PM_SYSTEM_SHUTDOWN_TIMEOUT    M2T(1000 * 60 * 5) // 5 min

#define PM_BAT_IIR_SHIFT     8
/**
 * Set PM_BAT_WANTED_LPF_CUTOFF_HZ to the wanted cut-off freq in Hz.
 */
#define PM_BAT_WANTED_LPF_CUTOFF_HZ   1

/**
 * Attenuation should be between 1 to 256.
 *
 * f0 = fs / 2*pi*attenuation.
 * attenuation = fs / 2*pi*f0
 */
#define PM_BAT_IIR_LPF_ATTENUATION (int)(ADC_SAMPLING_FREQ / (int)(2 * 3.1415 * PM_BAT_WANTED_LPF_CUTOFF_HZ))
#define PM_BAT_IIR_LPF_ATT_FACTOR  (int)((1<<PM_BAT_IIR_SHIFT) / PM_BAT_IIR_LPF_ATTENUATION)

typedef enum
{
  battery,
  charging,
  charged,
  lowPower,
  shutDown,
} PMStates;

typedef enum
{
  charge100mA,
  charge500mA,
  chargeMax,
} PMChargeStates;

void pmInit(void);

bool pmTest(void);

/**
 * Power management task
 */
void pmTask(void *param);

void pmSetChargeState(PMChargeStates chgState);

/**
 * Returns the battery voltage i volts as a float
 */
float pmGetBatteryVoltage(void);

/**
 * Returns the min battery voltage i volts as a float
 */
float pmGetBatteryVoltageMin(void);

/**
 * Returns the max battery voltage i volts as a float
 */
float pmGetBatteryVoltageMax(void);

/**
 * Updates and calculates battery values.
 * Should be called for every new adcValues sample.
 */
void pmBatteryUpdate(AdcGroup* adcValues);

#endif /* PM_H_ */
