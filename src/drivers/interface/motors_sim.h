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
 * motors_sim.h - declares motors_sim.c's public surface for
 * CONFIG_PLATFORM_SIM (Simmyflie).
 *
 * Not "motors.h": that header pulls in the STM32 hardware chain directly
 * (stm32fxxx.h) plus MotorPerifDef's ST peripheral-register fields (see
 * motors_sim.c's own header comment). motorsInit()'s motor-map argument is
 * narrowed from motors.h's real `const MotorPerifDef**` to `const void**`,
 * same precedent as Phase 4.0's i2cdevInit(void*) -- link-compatible with
 * real callers since C linking matches symbol names only, not per-TU
 * parameter types. Every other signature matches current mainline motors.h
 * exactly (verified against stabilizer.c's setMotorRatios()), which is what
 * should let Phase 4.8's stabilizer.c call this stub unmodified.
 *
 * Single source of truth for these signatures -- both motors_sim.c (the
 * definitions) and every consumer (main_sim.c; and, via
 * src/config/sim/hw_shims/motors.h, stabilizer.c/health.c -- see Phase 4.8)
 * include this header instead of hand-typing matching forward declarations.
 *
 * Phase 4.8 additions (motorsResetESCs/motorsCompensateBatteryVoltage/
 * motorsBeep/motorsGetHealthTestSettings/testsound/NBR_OF_MOTORS/
 * MOTOR_M1..M4/MotorHealthTestDef): the rest of real motors.h's surface that
 * stabilizer.c/health.c reference. All narrower-than-mainline stubs --
 * motorsCompensateBatteryVoltage() is a pass-through (no battery-voltage
 * model exists in sim, matching pm_sim.c's constant-voltage scope),
 * motorsBeep()/motorsResetESCs() are no-ops (no speaker/ESC-reset pin in
 * sim), motorsGetHealthTestSettings() returns one fixed timing/ratio set
 * (health.c's propeller/battery self-test is dead code in sim by default --
 * startPropTest/startBatTest params are 0 unless a client sets them -- so
 * the values are cosmetic, not modeled).
 */
#ifndef __MOTORS_SIM_H__
#define __MOTORS_SIM_H__

#include <stdbool.h>
#include <stdint.h>

#define NBR_OF_MOTORS 4
#define MOTOR_M1  0
#define MOTOR_M2  1
#define MOTOR_M3  2
#define MOTOR_M4  3

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
void motorsStop(void);
void motorsResetESCs(void);
void motorsBeep(int id, bool enable, uint16_t frequency, uint16_t ratio);
const MotorHealthTestDef* motorsGetHealthTestSettings(uint32_t id);
float motorsCompensateBatteryVoltage(uint32_t id, float iThrust, float supplyVoltage);

#endif // __MOTORS_SIM_H__
