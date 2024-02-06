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

#include "autoconf.h"
#include "adc.h"
#include "syslink.h"
#include "deck.h"


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

typedef enum
{
  USBNone,
  USB500mA,
  USBWallAdapter,
} PMUSBPower;

typedef void (*graceful_shutdown_callback_t)();

void pmInit(void);

bool pmTest(void);

/**
 * Power management task
 */
void pmTask(void *param);

void pmSetChargeState(PMChargeStates chgState);
void pmSyslinkUpdate(SyslinkPacket *slp);

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

/**
 * Returns true if the battery is below its low capacity threshold for an
 * extended period of time.
 */
bool pmIsBatteryLow(void);

/**
 * Returns true if the charger is currently connected
 */
bool pmIsChargerConnected(void);

/**
 * Returns true if the battery is currently charging
 */
bool pmIsCharging(void);

/**
 * Returns true if the battery is currently in use
 */
bool pmIsDischarging(void);

/**
 * Enable or disable external battery voltage measuring.
 */
void pmEnableExtBatteryVoltMeasuring(const deckPin_t pin, float multiplier);

/**
 * Measure an external voltage.
 */
float pmMeasureExtBatteryVoltage(void);

/**
 * Enable or disable external battery current measuring.
 */
void pmEnableExtBatteryCurrMeasuring(const deckPin_t pin, float ampPerVolt);

/**
 * Measure an external current.
 */
float pmMeasureExtBatteryCurrent(void);

/*
 * Ignore charging/charge state in the PM state machine.
 * This can be useful if a platform doesn't have a charger.
 */
void pmIgnoreChargedState(bool ignore);

/**
 * Register a callback to be run when the NRF51 signals shutdown
 */
bool pmRegisterGracefulShutdownCallback(graceful_shutdown_callback_t cb);

#endif /* PM_H_ */
