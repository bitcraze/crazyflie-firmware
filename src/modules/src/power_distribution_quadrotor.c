/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2022 Bitcraze AB
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
 * power_distribution_quadrotor.c - Crazyflie stock power distribution code
 */


#include "power_distribution.h"

#include <limits.h>
#include <stdint.h>
#include <string.h>
#include "debug.h"
#include "log.h"
#include "param.h"
#include "num.h"
#include "autoconf.h"
#include "config.h"
#include "math.h"
#include "platform_defaults.h"

#if (!defined(CONFIG_MOTORS_REQUIRE_ARMING) || (CONFIG_MOTORS_REQUIRE_ARMING == 0)) && defined(CONFIG_MOTORS_DEFAULT_IDLE_THRUST) && (CONFIG_MOTORS_DEFAULT_IDLE_THRUST > 0)
    #error "CONFIG_MOTORS_REQUIRE_ARMING must be defined and not set to 0 if CONFIG_MOTORS_DEFAULT_IDLE_THRUST is greater than 0"
#endif
#ifndef CONFIG_MOTORS_DEFAULT_IDLE_THRUST
#  define DEFAULT_IDLE_THRUST 0
#else
#  define DEFAULT_IDLE_THRUST CONFIG_MOTORS_DEFAULT_IDLE_THRUST
#endif

static uint32_t idleThrust = DEFAULT_IDLE_THRUST;

int powerDistributionMotorType(uint32_t id)
{
  return 1;
}

uint16_t powerDistributionStopRatio(uint32_t id)
{
  return 0;
}

void powerDistributionInit(void)
{
  #if (!defined(CONFIG_MOTORS_REQUIRE_ARMING) || (CONFIG_MOTORS_REQUIRE_ARMING == 0))
  if(idleThrust > 0) {
    DEBUG_PRINT("WARNING: idle thrust will be overridden with value 0. Autoarming can not be on while idle thrust is higher than 0. If you want to use idle thust please use use arming\n");
  }
  #endif
}

bool powerDistributionTest(void)
{
  bool pass = true;
  return pass;
}

/**
 * @brief Saturate a motor PWM duty command to the safe hardware range.
 *
 * Motor PWM ratios are 16-bit values written out to the motor driver / ESC
 * (via motorsSetRatio). Every path that produces a final duty must land in
 * [minThrust, UINT16_MAX]; the lower bound enforces idle thrust for
 * brushless spin-up, the upper bound prevents timer/DSHOT overflow.
 */
static uint16_t saturateMotorPwm(int32_t thrust, uint32_t minThrust) {
  if (thrust < (int32_t)minThrust) {
    return (uint16_t)minThrust;
  }
  if (thrust > (int32_t)UINT16_MAX) {
    return UINT16_MAX;
  }
  return (uint16_t)thrust;
}

/**
 * @brief Clamp a normalized force request to [0, 1] before scaling to PWM.
 *
 * Used by every allocation path that already speaks in normalized units so
 * out-of-range controller output cannot produce a duty above UINT16_MAX
 * before powerDistributionCap runs.
 */
static float clampNormalizedForce(float f) {
  if (f < 0.0f) {
    return 0.0f;
  }
  if (f > 1.0f) {
    return 1.0f;
  }
  return f;
}

/**
 * @brief Map a per-motor force in Newtons to an uncapped thrust integer.
 *
 * Negative force is treated as zero thrust. Values above THRUST_MAX are
 * allowed so powerDistributionCap can apply coordinated multi-motor
 * reduction (prioritize attitude over peak thrust). The result is saturated
 * to INT32_MAX so float overflow cannot poison the cap arithmetic.
 */
static int32_t forceToThrustUncapped(float motorForce) {
  if (motorForce < 0.0f) {
    motorForce = 0.0f;
  }

  /* THRUST_MAX is the nominal full-scale force (N) for one motor. */
  const float scaled = motorForce / THRUST_MAX * (float)UINT16_MAX;
  if (scaled >= (float)INT32_MAX) {
    return INT32_MAX;
  }
  return (int32_t)scaled;
}

static void powerDistributionLegacy(const control_t *control, motors_thrust_uncapped_t* motorThrustUncapped)
{
  int16_t r = control->roll / 2.0f;
  int16_t p = control->pitch / 2.0f;

  motorThrustUncapped->motors.m1 = control->thrust - r + p + control->yaw;
  motorThrustUncapped->motors.m2 = control->thrust - r - p - control->yaw;
  motorThrustUncapped->motors.m3 = control->thrust + r - p + control->yaw;
  motorThrustUncapped->motors.m4 = control->thrust + r + p - control->yaw;
}

static void powerDistributionForceTorque(const control_t *control, motors_thrust_uncapped_t* motorThrustUncapped) {
  static float motorForces[STABILIZER_NR_OF_MOTORS];

  const float arm = 0.707106781f * ARM_LENGTH;
  const float rollPart = 0.25f / arm * control->torqueX;
  const float pitchPart = 0.25f / arm * control->torqueY;
  const float thrustPart = 0.25f * control->thrustSi; // N (per rotor)
  const float yawPart = 0.25f * control->torqueZ / THRUST2TORQUE;

  motorForces[0] = thrustPart - rollPart - pitchPart - yawPart;
  motorForces[1] = thrustPart - rollPart + pitchPart + yawPart;
  motorForces[2] = thrustPart + rollPart + pitchPart - yawPart;
  motorForces[3] = thrustPart + rollPart - pitchPart + yawPart;

  for (int motorIndex = 0; motorIndex < STABILIZER_NR_OF_MOTORS; motorIndex++) {
    /* Lower-bound only here: values above full-scale stay uncapped so
     * powerDistributionCap can reduce all motors together (attitude first).
     * Final duty is still saturated to UINT16_MAX in saturateMotorPwm(). */
    motorThrustUncapped->list[motorIndex] = forceToThrustUncapped(motorForces[motorIndex]);
  }
}

/**
 * @brief Allows for direct control of motor power with clipping
 *
 * This function applies clipping to the motor values, which is different to
 * the "capping" behaviour found in powerDistributionForceTorque() - which
 * instead prioritizes stability rather than thrust.
 */
static void powerDistributionForce(const control_t *control, motors_thrust_uncapped_t* motorThrustUncapped) {
  for (int i = 0; i < STABILIZER_NR_OF_MOTORS; i++) {
    const float f = clampNormalizedForce(control->normalizedForces[i]);
    motorThrustUncapped->list[i] = (int32_t)(f * (float)UINT16_MAX);
  }
}

void powerDistribution(const control_t *control, motors_thrust_uncapped_t* motorThrustUncapped)
{
  switch (control->controlMode) {
    case controlModeLegacy:
      powerDistributionLegacy(control, motorThrustUncapped);
      break;
    case controlModeForceTorque:
      powerDistributionForceTorque(control, motorThrustUncapped);
      break;
    case controlModeForce:
      powerDistributionForce(control, motorThrustUncapped);
      break;
    default:
      // Nothing here
      break;
  }
}

bool powerDistributionCap(const motors_thrust_uncapped_t* motorThrustBatCompUncapped, motors_thrust_pwm_t* motorPwm)
{
  const int32_t maxAllowedThrust = UINT16_MAX;
  bool isCapped = false;

  // Find highest thrust
  int32_t highestThrustFound = 0;
  for (int motorIndex = 0; motorIndex < STABILIZER_NR_OF_MOTORS; motorIndex++)
  {
    const int32_t thrust = motorThrustBatCompUncapped->list[motorIndex];
    if (thrust > highestThrustFound)
    {
      highestThrustFound = thrust;
    }
  }

  int32_t reduction = 0;
  if (highestThrustFound > maxAllowedThrust)
  {
    reduction = highestThrustFound - maxAllowedThrust;
    isCapped = true;
  }

  const uint32_t minThrust = powerDistributionGetIdleThrust();
  for (int motorIndex = 0; motorIndex < STABILIZER_NR_OF_MOTORS; motorIndex++)
  {
    /* Coordinated reduction keeps relative torques; saturateMotorPwm is the
     * last line of defence so a duty out of [min, UINT16_MAX] can never be
     * handed to motorsSetRatio / the PWM hardware. */
    const int32_t thrustCappedUpper = motorThrustBatCompUncapped->list[motorIndex] - reduction;
    motorPwm->list[motorIndex] = saturateMotorPwm(thrustCappedUpper, minThrust);
  }

  return isCapped;
}

uint32_t powerDistributionGetIdleThrust()
{
  int32_t thrust = idleThrust;
  #if (!defined(CONFIG_MOTORS_REQUIRE_ARMING) || (CONFIG_MOTORS_REQUIRE_ARMING == 0))
    thrust = 0;
  #endif
  return thrust;
}

float powerDistributionGetMaxThrust() {
  return STABILIZER_NR_OF_MOTORS * THRUST_MAX;
}

/**
 * Power distribution parameters
 */
PARAM_GROUP_START(powerDist)
/**
 * @brief Motor thrust to set at idle (default: 0)
 *
 * This is often needed for brushless motors as
 * it takes time to start up the motor. Then a
 * common value is between 3000 - 6000.
 */
PARAM_ADD_CORE(PARAM_UINT32 | PARAM_PERSISTENT, idleThrust, &idleThrust)
PARAM_GROUP_STOP(powerDist)
