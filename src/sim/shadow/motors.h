/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2026 Bitcraze AB
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
 * motors.h - Shadow header for the Simmyflie.
 *
 * The mainline motors.h includes the STM32 device headers and describes the
 * motor timers and pins. This header keeps the part of its contract that the
 * compiled files use, implemented by src/sim/src/drivers/motors.c:
 * - motorsInit() takes `const void **` instead of `const MotorPerifDef **`.
 *   No compiled file looks inside the motor map, and C links by symbol name,
 *   so this is link-compatible.
 * - MOTORS_TIM_BEEP_CLK_FREQ is a literal: mainline's TIM_CLOCK_HZ / 5.
 *
 * See docs/development/simmyflie/index.md for the Shadow header rules.
 */
#ifndef __MOTORS_H__
#define __MOTORS_H__

#include <stdbool.h>
#include <stdint.h>

#include "config.h"
#include "autoconf.h"

#define NBR_OF_MOTORS 4
#define MOTOR_M1  0
#define MOTOR_M2  1
#define MOTOR_M3  2
#define MOTOR_M4  3

#define MOTORS_RPM_INVALID (UINT16_MAX)

/* Test defines -- just the two health.c's propeller-test failure path
 * references (dead code by default, see above). */
#define MOTORS_TEST_ON_TIME_MS    50
#define MOTORS_TEST_DELAY_TIME_MS 150

/* Sound defines -- just enough of real motors.h's tone table for
 * health.c's beep-frequency arithmetic (dead code by default, see above). */
#define A4    440
#define A5    880
#define F5    698
#define D5    587
#define MOTORS_TIM_BEEP_CLK_FREQ 16800000UL /* real motors.h: TIM_CLOCK_HZ/5 */

typedef struct {
  uint16_t onPeriodMsec;
  uint16_t offPeriodMsec;
  uint16_t varianceMeasurementStartMsec;
  uint16_t onPeriodPWMRatioProp;
  uint16_t onPeriodPWMRatioBat;
} MotorHealthTestDef;

extern const uint16_t testsound[NBR_OF_MOTORS];

void motorsInit(const void **motorMapSelect);
bool motorsTest(void);
void motorsSetRatio(uint32_t id, uint16_t ratio);
uint16_t motorsGetRatio(uint32_t id);
uint16_t motorsGetRPM(uint32_t motorId);
void motorsStop(void);
void motorsResetESCs(void);
void motorsBurstDshot(void);
void motorsBeep(int id, bool enable, uint16_t frequency, uint16_t ratio);
const MotorHealthTestDef* motorsGetHealthTestSettings(uint32_t id);
float motorsCompensateBatteryVoltage(uint32_t id, float iThrust, float supplyVoltage);

#endif // __MOTORS_H__
