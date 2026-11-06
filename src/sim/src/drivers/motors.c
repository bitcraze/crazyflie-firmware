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
 * motors.c - Replacement of motors.h for the Simmyflie: the Actuator shim.
 *
 * motorsSetRatio() stores the ratio the firmware writes and passes it on,
 * unchanged, to the Physics engine manager with
 * physicsEngineManagerSetMotorRatio(). motorsStop() does the same with four
 * zeros. Passing a ratio on never advances the true state: the Sensor shim
 * steps the physics once per tick. The shim holds no physical knowledge of
 * motors and never calls the physics engine.
 *
 * The "motor" log group has the m1-m4 ratios of the mainline motors.c, but not
 * the RPM entries: there is no ESC to report them. motorsGetRPM() returns a
 * constant that passes the supervisor's RPM checks.
 *
 * The rest of the motors.h functions that mainline code calls have no
 * hardware to act on: motorsCompensateBatteryVoltage() passes the thrust
 * through (the sim battery voltage is constant), motorsBeep(),
 * motorsResetESCs() and motorsBurstDshot() do nothing, and
 * motorsGetHealthTestSettings() returns one fixed set of values for health.c's
 * propeller and battery tests.
 */

#include <stdbool.h>
#include <stdint.h>

#include "log.h"
#include "motors.h"
#include "physics_engine_manager.h"

// Constant RPM reported by motorsGetRPM(). Must lie inside the supervisor's
// arming window (CONFIG_MOTORS_ARMING_RPM_MIN..MAX, default 1000..3500) and
// at or above CONFIG_MOTORS_RPM_NOT_RESPONDING_THRESHOLD (default 500).
#define MOTORS_RPM 2000

const uint16_t testsound[NBR_OF_MOTORS] = {A4, A5, F5, D5};

static const MotorHealthTestDef healthTestSettings = {
  .onPeriodMsec = 50,
  .offPeriodMsec = 950,
  .varianceMeasurementStartMsec = 20,
  .onPeriodPWMRatioProp = 5000,
  .onPeriodPWMRatioBat = 5000,
};

static bool isInit = false;
static uint16_t motor_ratios[NBR_OF_MOTORS] = {0, 0, 0, 0};

void motorsInit(const void **motorMapSelect)
{
  (void)motorMapSelect;

  for (int i = 0; i < NBR_OF_MOTORS; i++) {
    motor_ratios[i] = 0;
  }
  isInit = true;
}

bool motorsTest(void)
{
  return isInit;
}

void motorsSetRatio(uint32_t id, uint16_t ratio)
{
  if (id < NBR_OF_MOTORS) {
    motor_ratios[id] = ratio;
    physicsEngineManagerSetMotorRatio(id, ratio);
  }
}

uint16_t motorsGetRatio(uint32_t id)
{
  if (id < NBR_OF_MOTORS) {
    return motor_ratios[id];
  }
  return 0;
}

uint16_t motorsGetRPM(uint32_t motorId)
{
  /* No ESC telemetry in sim: a constant that passes both of supervisor.c's
   * RPM checks (arming window and motors-not-responding), regardless of the
   * commanded ratio and of the true rotor speeds (trueState.rotor1..4). */
  if (motorId >= NBR_OF_MOTORS) {
    return MOTORS_RPM_INVALID;
  }
  return MOTORS_RPM;
}

void motorsStop(void)
{
  for (uint32_t i = 0; i < NBR_OF_MOTORS; i++) {
    motor_ratios[i] = 0;
    physicsEngineManagerSetMotorRatio(i, 0);
  }
}

void motorsResetESCs(void)
{
  /* No-op: only meaningful for brushless motors with an ESC reset pin. */
}

void motorsBurstDshot(void)
{
  /* No-op: motorsSetRatio() already applies the ratio directly, so there's
   * no separate hardware burst step to model. */
}

void motorsBeep(int id, bool enable, uint16_t frequency, uint16_t ratio)
{
  /* No-op: no speaker in sim. */
  (void)id;
  (void)enable;
  (void)frequency;
  (void)ratio;
}

const MotorHealthTestDef* motorsGetHealthTestSettings(uint32_t id)
{
  (void)id;
  return &healthTestSettings;
}

float motorsCompensateBatteryVoltage(uint32_t id, float iThrust, float supplyVoltage)
{
  /* The sim battery voltage is constant (see pm.c): pass through. */
  (void)id;
  (void)supplyVoltage;
  return iThrust;
}

/**
 * Motor output related log variables.
 */
LOG_GROUP_START(motor)
/**
 * @brief Motor power (PWM value) for M1 [0 - UINT16_MAX]
 */
LOG_ADD_CORE(LOG_UINT16, m1, &motor_ratios[MOTOR_M1])
/**
 * @brief Motor power (PWM value) for M2 [0 - UINT16_MAX]
 */
LOG_ADD_CORE(LOG_UINT16, m2, &motor_ratios[MOTOR_M2])
/**
 * @brief Motor power (PWM value) for M3 [0 - UINT16_MAX]
 */
LOG_ADD_CORE(LOG_UINT16, m3, &motor_ratios[MOTOR_M3])
/**
 * @brief Motor power (PWM value) for M4 [0 - UINT16_MAX]
 */
LOG_ADD_CORE(LOG_UINT16, m4, &motor_ratios[MOTOR_M4])
LOG_GROUP_STOP(motor)
