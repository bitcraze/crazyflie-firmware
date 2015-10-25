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
 
#include "FreeRTOS.h"

#include "controller.h"
#include "pid.h"
#include "param.h"
#include "imu.h"

static inline int16_t saturateSignedInt16(float in)
{
  // don't use INT16_MIN, because later we may negate it, which won't work for that value.
  if (in > INT16_MAX)
    return INT16_MAX;
  else if (in < -INT16_MAX)
    return -INT16_MAX;
  else
    return (int16_t)in;
}

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

void controllerInit()
{
  if(isInit)
    return;
  
  //TODO: get parameters from configuration manager instead
  pidInit(&pidRollRate, 0, PID_ROLL_RATE_KP, PID_ROLL_RATE_KI, PID_ROLL_RATE_KD, IMU_UPDATE_DT);
  pidInit(&pidPitchRate, 0, PID_PITCH_RATE_KP, PID_PITCH_RATE_KI, PID_PITCH_RATE_KD, IMU_UPDATE_DT);
  pidInit(&pidYawRate, 0, PID_YAW_RATE_KP, PID_YAW_RATE_KI, PID_YAW_RATE_KD, IMU_UPDATE_DT);
  pidSetIntegralLimit(&pidRollRate, PID_ROLL_RATE_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidPitchRate, PID_PITCH_RATE_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidYawRate, PID_YAW_RATE_INTEGRATION_LIMIT);

  pidInit(&pidRoll, 0, PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD, IMU_UPDATE_DT);
  pidInit(&pidPitch, 0, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, IMU_UPDATE_DT);
  pidInit(&pidYaw, 0, PID_YAW_KP, PID_YAW_KI, PID_YAW_KD, IMU_UPDATE_DT);
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
  rollOutput = saturateSignedInt16(pidUpdate(&pidRollRate, rollRateActual, true));

  pidSetDesired(&pidPitchRate, pitchRateDesired);
  pitchOutput = saturateSignedInt16(pidUpdate(&pidPitchRate, pitchRateActual, true));

  pidSetDesired(&pidYawRate, yawRateDesired);
  yawOutput = saturateSignedInt16(pidUpdate(&pidYawRate, yawRateActual, true));
}

void controllerCorrectAttitudePID(
       float eulerRollActual, float eulerPitchActual, float eulerYawActual,
       float eulerRollDesired, float eulerPitchDesired, float eulerYawDesired,
       float* rollRateDesired, float* pitchRateDesired, float* yawRateDesired)
{
  pidSetDesired(&pidRoll, eulerRollDesired);
  *rollRateDesired = pidUpdate(&pidRoll, eulerRollActual, true);

  // Update PID for pitch axis
  pidSetDesired(&pidPitch, eulerPitchDesired);
  *pitchRateDesired = pidUpdate(&pidPitch, eulerPitchActual, true);

  // Update PID for yaw axis
  float yawError;
  yawError = eulerYawDesired - eulerYawActual;
  if (yawError > 180.0)
    yawError -= 360.0;
  else if (yawError < -180.0)
    yawError += 360.0;
  pidSetError(&pidYaw, yawError);
  *yawRateDesired = pidUpdate(&pidYaw, eulerYawActual, false);
}

void controllerResetAllPID(void)
{
  pidReset(&pidRoll);
  pidReset(&pidPitch);
  pidReset(&pidYaw);
  pidReset(&pidRollRate);
  pidReset(&pidPitchRate);
  pidReset(&pidYawRate);
}

void controllerGetActuatorOutput(int16_t* roll, int16_t* pitch, int16_t* yaw)
{
  *roll = rollOutput;
  *pitch = pitchOutput;
  *yaw = yawOutput;
}

PARAM_GROUP_START(pid_attitude)
PARAM_ADD(PARAM_FLOAT, roll_kp, &pidRoll.kp)
PARAM_ADD(PARAM_FLOAT, roll_ki, &pidRoll.ki)
PARAM_ADD(PARAM_FLOAT, roll_kd, &pidRoll.kd)
PARAM_ADD(PARAM_FLOAT, pitch_kp, &pidPitch.kp)
PARAM_ADD(PARAM_FLOAT, pitch_ki, &pidPitch.ki)
PARAM_ADD(PARAM_FLOAT, pitch_kd, &pidPitch.kd)
PARAM_ADD(PARAM_FLOAT, yaw_kp, &pidYaw.kp)
PARAM_ADD(PARAM_FLOAT, yaw_ki, &pidYaw.ki)
PARAM_ADD(PARAM_FLOAT, yaw_kd, &pidYaw.kd)
PARAM_GROUP_STOP(pid_attitude)

PARAM_GROUP_START(pid_rate)
PARAM_ADD(PARAM_FLOAT, roll_kp, &pidRollRate.kp)
PARAM_ADD(PARAM_FLOAT, roll_ki, &pidRollRate.ki)
PARAM_ADD(PARAM_FLOAT, roll_kd, &pidRollRate.kd)
PARAM_ADD(PARAM_FLOAT, pitch_kp, &pidPitchRate.kp)
PARAM_ADD(PARAM_FLOAT, pitch_ki, &pidPitchRate.ki)
PARAM_ADD(PARAM_FLOAT, pitch_kd, &pidPitchRate.kd)
PARAM_ADD(PARAM_FLOAT, yaw_kp, &pidYawRate.kp)
PARAM_ADD(PARAM_FLOAT, yaw_ki, &pidYawRate.ki)
PARAM_ADD(PARAM_FLOAT, yaw_kd, &pidYawRate.kd)
PARAM_GROUP_STOP(pid_rate)
