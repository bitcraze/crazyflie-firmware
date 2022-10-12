/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2022 Bitcraze AB & Flapper Drones (https:\\flapper-drones.com)
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
 * power_distribution_flapper.c - Power distribution code for the Flapper Nimble+
 */
#define DEBUG_MODULE "PWR_DIST"

#include "power_distribution.h"

#include <string.h>
#include "log.h"
#include "param.h"
#include "num.h"
#include "autoconf.h"
#include "config.h"

#include "debug.h"
#include "math.h"

#ifndef CONFIG_MOTORS_DEFAULT_IDLE_THRUST
#  define DEFAULT_IDLE_THRUST 0
#else
#  define DEFAULT_IDLE_THRUST CONFIG_MOTORS_DEFAULT_IDLE_THRUST
#endif

static uint32_t idleThrust = DEFAULT_IDLE_THRUST;

struct flapperConfig_s {
  uint8_t pitchServoNeutral;
  uint8_t yawServoNeutral;
  int8_t rollBias;
  uint16_t maxThrust;
};

struct flapperConfig_s flapperConfig = {
  .pitchServoNeutral = 50,
  .yawServoNeutral = 50,
  .rollBias = 0,
  .maxThrust = 60000,
};

static float thrust;
static uint16_t act_max = 65535;

static float pitch_ampl = 0.4f; // 1 = full servo stroke

#if CONFIG_POWER_DISTRIBUTION_FLAPPER_REVB
  uint32_t idPitch = 1;
  uint32_t idYaw = 2;
#else
  uint32_t idPitch = 0;
  uint32_t idYaw = 2;
#endif

static uint8_t limitServoNeutral(uint8_t value)
{
  if(value > 75)
  {
    value = 75;
  }
  else if(value < 25)
  {
    value = 25;
  }

  return (uint8_t)value;
}

static int8_t limitRollBias(uint8_t value)
{
  if(value > 25)
  {
    value = 25;
  }
  else if(value < -25)
  {
    value = -25;
  }

  return (uint8_t)value;
}

int powerDistributionMotorType(uint32_t id)
{
  int type = 0;
  if (id == (idPitch || idYaw)) type = 1;
  return type;
}

uint16_t powerDistributionStopRatio(uint32_t id)
{
  uint16_t stopRatio = 0;
  if (id==idPitch) stopRatio = flapperConfig.pitchServoNeutral*act_max/100.0f;
  else if (id==idYaw) stopRatio = flapperConfig.yawServoNeutral*act_max/100.0f;
  return stopRatio;
}
  
void powerDistributionInit(void)
{
  #if CONFIG_POWER_DISTRIBUTION_FLAPPER_REVB
    DEBUG_PRINT("Using Flapper power distribution | PCB revB (2021)\n");
  #else
    DEBUG_PRINT("Using Flapper power distribution | PCB revD (2022) or newer\n");
  #endif
  
}

bool powerDistributionTest(void)
{
  bool pass = true;
  return pass;
}

#define limitThrust(VAL) limitUint16(VAL)

void powerDistribution(motors_thrust_t* motorPower, const control_t *control)
{
  thrust = fmin(control->thrust, flapperConfig.maxThrust);
  
  flapperConfig.pitchServoNeutral=limitServoNeutral(flapperConfig.pitchServoNeutral);
  flapperConfig.yawServoNeutral=limitServoNeutral(flapperConfig.yawServoNeutral);
  flapperConfig.rollBias=limitRollBias(flapperConfig.rollBias);

  #if CONFIG_POWER_DISTRIBUTION_FLAPPER_REVB
    motorPower->m2 = limitThrust(flapperConfig.pitchServoNeutral*act_max/100.0f + pitch_ampl*control->pitch); // pitch servo
    motorPower->m3 = limitThrust(flapperConfig.yawServoNeutral*act_max/100.0f - control->yaw); // yaw servo
    motorPower->m1 = limitThrust( 0.5f * control->roll + thrust * (1.0f + flapperConfig.rollBias/100.0f) ); // left motor
    motorPower->m4 = limitThrust(-0.5f * control->roll + thrust * (1.0f - flapperConfig.rollBias/100.0f) ); // right motor
    
    if (motorPower->m1 < idleThrust) {
      motorPower->m1 = idleThrust;
    }
    if (motorPower->m4 < idleThrust) {
      motorPower->m4 = idleThrust;
    }
  #else
    motorPower->m1 = limitThrust(flapperConfig.pitchServoNeutral*act_max/100.0f + pitch_ampl*control->pitch); // pitch servo
    motorPower->m3 = limitThrust(flapperConfig.yawServoNeutral*act_max/100.0f - control->yaw); // yaw servo
    motorPower->m2 = limitThrust( 0.5f * control->roll + thrust * (1.0f + flapperConfig.rollBias/100.0f) ); // left motor
    motorPower->m4 = limitThrust(-0.5f * control->roll + thrust * (1.0f - flapperConfig.rollBias/100.0f) ); // right motor
    
    if (motorPower->m2 < idleThrust) {
      motorPower->m2 = idleThrust;
    }
    if (motorPower->m4 < idleThrust) {
      motorPower->m4 = idleThrust;
    }
  #endif
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

/**
 *
 * Flapper Drone configration parameters
 */
PARAM_GROUP_START(flapper)
/**
 * @brief Roll bias <-25%; 25%> (default 0%)
 *
 * This parameter can be used if uneven performance of the left and right flapping mechanaisms and/or wings
 * is observed, which in flight results in a drift in roll/sideways flight. Positive values make the drone roll 
 * more to the right, negative values to the left.
 */
PARAM_ADD(PARAM_INT8 | PARAM_PERSISTENT, motBiasRoll, &flapperConfig.rollBias)
/**
 * @brief Pitch servo neutral <25%; 75%> (default 50%)
 *
 * The parameter sets the neutral position of the pitch servo, such that the left and right wing-pairs are
 * aligned when observed from the side. If in flight you observe too much drift forward (nose down) increase the value
 * and vice versa if the drift is backward (nose up).
 */
PARAM_ADD(PARAM_UINT8 | PARAM_PERSISTENT, servPitchNeutr, &flapperConfig.pitchServoNeutral)
/**
 * @brief Yaw servo neutral <25%; 75%> (default 50%)
 *
 * The parameter sets the neutral position of the yaw servo, such that the yaw control arm is pointed spanwise. If in flight 
 * you observe drift in the clock-wise direction, increase this parameter and vice-versa if the drift is counter-clock-wise.
 */
PARAM_ADD(PARAM_UINT8 | PARAM_PERSISTENT, servYawNeutr, &flapperConfig.yawServoNeutral)
/**
 * @brief Yaw servo neutral <25%; 75%> (default 50%)
 *
 * The parameter sets the neutral position of the yaw servo, such that the yaw control arm is pointed spanwise. If in flight 
 * you observe drift in the clock-wise direction, increase this parameter and vice-versa if the drift is counter-clock-wise.
 */
PARAM_ADD(PARAM_UINT16 | PARAM_PERSISTENT, flapperMaxThrust, &flapperConfig.maxThrust)

PARAM_GROUP_STOP(flapper)
