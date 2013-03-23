/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
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
 *
 */
#include <stdbool.h>
 
#include "stm32f10x_conf.h"
#include "FreeRTOS.h"
#include "task.h"

#include "controller.h"
#include "pid.h"
#include "param.h"
/*
#define TRUNCATE_SINT16(out, in) \
  {\
    if (in > INT16_MAX) out = (int16_t)INT16_MAX;\
    else if (in < INT16_MIN) out = (int16_t)INT16_MIN;\
    else out = (int16_t)in;\
  }
*/

//Fancier version
#define TRUNCATE_SINT16(out, in) (out = (in<INT16_MIN)?INT16_MIN:((in>INT16_MAX)?INT16_MAX:in) )

//Better semantic
#define SATURATE_SINT16(in) ( (in<INT16_MIN)?INT16_MIN:((in>INT16_MAX)?INT16_MAX:in) )

PidObject pidRollRate;
PidObject pidPitchRate;
PidObject pidYawRate;
PidObject pidRoll;
PidObject pidPitch;
PidObject pidYaw;

int16_t rollOutput;
int16_t pitchOutput;
int16_t yawOutput;

static bool isInit;

PARAM_GROUP_START(attitudepid)
PARAM_ADD(PARAM_FLOAT, kp_roll, &pidRoll.kp)
PARAM_ADD(PARAM_FLOAT, ki_roll, &pidRoll.ki)
PARAM_ADD(PARAM_FLOAT, kd_roll, &pidRoll.kd)
PARAM_ADD(PARAM_FLOAT, kp_pitch, &pidPitch.kp)
PARAM_ADD(PARAM_FLOAT, ki_pitch, &pidPitch.ki)
PARAM_ADD(PARAM_FLOAT, kd_pitch, &pidPitch.kd)
PARAM_ADD(PARAM_FLOAT, kp_yaw, &pidYaw.kp)
PARAM_ADD(PARAM_FLOAT, ki_yaw, &pidYaw.ki)
PARAM_ADD(PARAM_FLOAT, kd_yaw, &pidYaw.kd)
PARAM_GROUP_STOP(attitidepid)

PARAM_GROUP_START(ratepid)
PARAM_ADD(PARAM_FLOAT, kp_roll, &pidRollRate.kp)
PARAM_ADD(PARAM_FLOAT, ki_roll, &pidRollRate.ki)
PARAM_ADD(PARAM_FLOAT, kd_roll, &pidRollRate.kd)
PARAM_ADD(PARAM_FLOAT, kp_pitch, &pidPitchRate.kp)
PARAM_ADD(PARAM_FLOAT, ki_pitch, &pidPitchRate.ki)
PARAM_ADD(PARAM_FLOAT, kd_pitch, &pidPitchRate.kd)
PARAM_ADD(PARAM_FLOAT, kp_yaw, &pidYawRate.kp)
PARAM_ADD(PARAM_FLOAT, ki_yaw, &pidYawRate.ki)
PARAM_ADD(PARAM_FLOAT, kd_yaw, &pidYawRate.kd)
PARAM_GROUP_STOP(ratepid)

void controllerInit()
{
  if(isInit)
    return;
  
  //TODO: get parameters from configuration manager instead
  pidInit(&pidRollRate, 0, PID_ROLL_RATE_KP, PID_ROLL_RATE_KI, PID_ROLL_RATE_KD);
  pidInit(&pidPitchRate, 0, PID_PITCH_RATE_KP, PID_PITCH_RATE_KI, PID_PITCH_RATE_KD);
  pidInit(&pidYawRate, 0, PID_YAW_RATE_KP, PID_YAW_RATE_KI, PID_YAW_RATE_KD);
  pidSetIntegralLimit(&pidRollRate, PID_ROLL_RATE_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidPitchRate, PID_PITCH_RATE_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidYawRate, PID_YAW_RATE_INTEGRATION_LIMIT);

  pidInit(&pidRoll, 0, PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD);
  pidInit(&pidPitch, 0, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD);
  pidInit(&pidYaw, 0, PID_YAW_KP, PID_YAW_KI, PID_YAW_KD);
  pidSetIntegralLimit(&pidRoll, PID_ROLL_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidPitch, PID_PITCH_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidYaw, PID_YAW_INTEGRATION_LIMIT);
  
  isInit = true;
}

bool controllerTest()
{
  return isInit;
}

void controllerCorrectRatePID(
       float rollRateActual, float pitchRateActual, float yawRateActual,
       float rollRateDesired, float pitchRateDesired, float yawRateDesired)
{
  pidSetDesired(&pidRollRate, rollRateDesired);
  TRUNCATE_SINT16(rollOutput, pidUpdate(&pidRollRate, rollRateActual, TRUE));

  pidSetDesired(&pidPitchRate, pitchRateDesired);
  TRUNCATE_SINT16(pitchOutput, pidUpdate(&pidPitchRate, pitchRateActual, TRUE));

  pidSetDesired(&pidYawRate, yawRateDesired);
  TRUNCATE_SINT16(yawOutput, pidUpdate(&pidYawRate, yawRateActual, TRUE));
}

void controllerCorrectAttitudePID(
       float eulerRollActual, float eulerPitchActual, float eulerYawActual,
       float eulerRollDesired, float eulerPitchDesired, float eulerYawDesired,
       float* rollRateDesired, float* pitchRateDesired, float* yawRateDesired)
{
  pidSetDesired(&pidRoll, eulerRollDesired);
  *rollRateDesired = pidUpdate(&pidRoll, eulerRollActual, TRUE);

  // Update PID for pitch axis
  pidSetDesired(&pidPitch, eulerPitchDesired);
  *pitchRateDesired = pidUpdate(&pidPitch, eulerPitchActual, TRUE);

  // Update PID for yaw axis
  float yawError;
  yawError = eulerYawDesired - eulerYawActual;
  if (yawError > 180.0)
    yawError -= 360.0;
  else if (yawError < -180.0)
    yawError += 360.0;
  pidSetError(&pidYaw, yawError);
  *yawRateDesired = pidUpdate(&pidYaw, eulerYawActual, FALSE);
}

void controllerResetAllPID(void)
{
  pidReset(&pidRoll);
  pidReset(&pidPitch);
  pidReset(&pidYaw);
}

void controllerGetActuatorOutput(int16_t* roll, int16_t* pitch, int16_t* yaw)
{
  *roll = rollOutput;
  *pitch = pitchOutput;
  *yaw = yawOutput;
}
