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
 * attitude_pid_controller.c: Attitude controller using PID correctors
 */
#include <stdbool.h>

#include "stabilizer_types.h"

#include "attitude_controller.h"
#include "pid.h"
#include "param.h"
#include "log.h"
#include "commander.h"
#include "platform_defaults.h"


static bool attFiltEnable = ATTITUDE_LPF_ENABLE;
static bool rateFiltEnable = ATTITUDE_RATE_LPF_ENABLE;
static float attFiltCutoff = ATTITUDE_LPF_CUTOFF_FREQ;
static float omxFiltCutoff = ATTITUDE_ROLL_RATE_LPF_CUTOFF_FREQ;
static float omyFiltCutoff = ATTITUDE_PITCH_RATE_LPF_CUTOFF_FREQ;
static float omzFiltCutoff = ATTITUDE_YAW_RATE_LPF_CUTOFF_FREQ;
static float yawMaxDelta = YAW_MAX_DELTA;

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

PidObject pidRollRate = {
  .kp = PID_ROLL_RATE_KP,
  .ki = PID_ROLL_RATE_KI,
  .kd = PID_ROLL_RATE_KD,
  .kff = PID_ROLL_RATE_KFF,
};

PidObject pidPitchRate = {
  .kp = PID_PITCH_RATE_KP,
  .ki = PID_PITCH_RATE_KI,
  .kd = PID_PITCH_RATE_KD,
  .kff = PID_PITCH_RATE_KFF,
};

PidObject pidYawRate = {
  .kp = PID_YAW_RATE_KP,
  .ki = PID_YAW_RATE_KI,
  .kd = PID_YAW_RATE_KD,
  .kff = PID_YAW_RATE_KFF,
};

PidObject pidRoll = {
  .kp = PID_ROLL_KP,
  .ki = PID_ROLL_KI,
  .kd = PID_ROLL_KD,
  .kff = PID_ROLL_KFF,
};

PidObject pidPitch = {
  .kp = PID_PITCH_KP,
  .ki = PID_PITCH_KI,
  .kd = PID_PITCH_KD,
  .kff = PID_PITCH_KFF,
};

PidObject pidYaw = {
  .kp = PID_YAW_KP,
  .ki = PID_YAW_KI,
  .kd = PID_YAW_KD,
  .kff = PID_YAW_KFF,
};

static int16_t rollOutput;
static int16_t pitchOutput;
static int16_t yawOutput;

static bool isInit;

void attitudeControllerInit(const float updateDt)
{
  if(isInit)
    return;

  //TODO: get parameters from configuration manager instead - now (partly) implemented
  pidInit(&pidRollRate,  0, pidRollRate.kp,  pidRollRate.ki,  pidRollRate.kd,
       pidRollRate.kff,  updateDt, ATTITUDE_RATE, omxFiltCutoff, rateFiltEnable);
  pidInit(&pidPitchRate, 0, pidPitchRate.kp, pidPitchRate.ki, pidPitchRate.kd,
       pidPitchRate.kff, updateDt, ATTITUDE_RATE, omyFiltCutoff, rateFiltEnable);
  pidInit(&pidYawRate,   0, pidYawRate.kp,   pidYawRate.ki,   pidYawRate.kd,
       pidYawRate.kff,   updateDt, ATTITUDE_RATE, omzFiltCutoff, rateFiltEnable);

  pidSetIntegralLimit(&pidRollRate,  PID_ROLL_RATE_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidPitchRate, PID_PITCH_RATE_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidYawRate,   PID_YAW_RATE_INTEGRATION_LIMIT);

  pidInit(&pidRoll,  0, pidRoll.kp,  pidRoll.ki,  pidRoll.kd,  pidRoll.kff,  updateDt,
      ATTITUDE_RATE, attFiltCutoff, attFiltEnable);
  pidInit(&pidPitch, 0, pidPitch.kp, pidPitch.ki, pidPitch.kd, pidPitch.kff, updateDt,
      ATTITUDE_RATE, attFiltCutoff, attFiltEnable);
  pidInit(&pidYaw,   0, pidYaw.kp,   pidYaw.ki,   pidYaw.kd,   pidYaw.kff,   updateDt,
      ATTITUDE_RATE, attFiltCutoff, attFiltEnable);

  pidSetIntegralLimit(&pidRoll,  PID_ROLL_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidPitch, PID_PITCH_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidYaw,   PID_YAW_INTEGRATION_LIMIT);

  isInit = true;
}

bool attitudeControllerTest()
{
  return isInit;
}

void attitudeControllerCorrectRatePID(
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

void attitudeControllerCorrectAttitudePID(
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
  if (yawError > 180.0f)
    yawError -= 360.0f;
  else if (yawError < -180.0f)
    yawError += 360.0f;
  pidSetError(&pidYaw, yawError);
  *yawRateDesired = pidUpdate(&pidYaw, eulerYawActual, false);
}

void attitudeControllerResetRollAttitudePID(void)
{
    pidReset(&pidRoll);
}

void attitudeControllerResetPitchAttitudePID(void)
{
    pidReset(&pidPitch);
}

void attitudeControllerResetAllPID(void)
{
  pidReset(&pidRoll);
  pidReset(&pidPitch);
  pidReset(&pidYaw);
  pidReset(&pidRollRate);
  pidReset(&pidPitchRate);
  pidReset(&pidYawRate);
}

void attitudeControllerGetActuatorOutput(int16_t* roll, int16_t* pitch, int16_t* yaw)
{
  *roll = rollOutput;
  *pitch = pitchOutput;
  *yaw = yawOutput;
}

float attitudeControllerGetYawMaxDelta(void)
{
  return yawMaxDelta;
}

/**
 *  Log variables of attitude PID controller
 */ 
LOG_GROUP_START(pid_attitude)
/**
 * @brief Proportional output roll
 */
LOG_ADD(LOG_FLOAT, roll_outP, &pidRoll.outP)
/**
 * @brief Integral output roll
 */
LOG_ADD(LOG_FLOAT, roll_outI, &pidRoll.outI)
/**
 * @brief Derivative output roll
 */
LOG_ADD(LOG_FLOAT, roll_outD, &pidRoll.outD)
/**
 * @brief Feedforward output roll
 */
LOG_ADD(LOG_FLOAT, roll_outFF, &pidRoll.outFF)
/**
 * @brief Proportional output pitch
 */
LOG_ADD(LOG_FLOAT, pitch_outP, &pidPitch.outP)
/**
 * @brief Integral output pitch
 */
LOG_ADD(LOG_FLOAT, pitch_outI, &pidPitch.outI)
/**
 * @brief Derivative output pitch
 */
LOG_ADD(LOG_FLOAT, pitch_outD, &pidPitch.outD)
/**
 * @brief Feedforward output pitch
 */
LOG_ADD(LOG_FLOAT, pitch_outFF, &pidPitch.outFF)
/**
 * @brief Proportional output yaw
 */
LOG_ADD(LOG_FLOAT, yaw_outP, &pidYaw.outP)
/**
 * @brief Intergal output yaw
 */
LOG_ADD(LOG_FLOAT, yaw_outI, &pidYaw.outI)
/**
 * @brief Derivative output yaw
 */
LOG_ADD(LOG_FLOAT, yaw_outD, &pidYaw.outD)
/**
 * @brief Feedforward output yaw
 */
LOG_ADD(LOG_FLOAT, yaw_outFF, &pidYaw.outFF)
LOG_GROUP_STOP(pid_attitude)

/**
 *  Log variables of attitude rate PID controller
 */
LOG_GROUP_START(pid_rate)
/**
 * @brief Proportional output roll rate
 */
LOG_ADD(LOG_FLOAT, roll_outP, &pidRollRate.outP)
/**
 * @brief Integral output roll rate
 */
LOG_ADD(LOG_FLOAT, roll_outI, &pidRollRate.outI)
/**
 * @brief Derivative output roll rate
 */
LOG_ADD(LOG_FLOAT, roll_outD, &pidRollRate.outD)
/**
 * @brief Feedforward output roll rate
 */
LOG_ADD(LOG_FLOAT, roll_outFF, &pidRollRate.outFF)
/**
 * @brief Proportional output pitch rate
 */
LOG_ADD(LOG_FLOAT, pitch_outP, &pidPitchRate.outP)
/**
 * @brief Integral output pitch rate
 */
LOG_ADD(LOG_FLOAT, pitch_outI, &pidPitchRate.outI)
/**
 * @brief Derivative output pitch rate
 */
LOG_ADD(LOG_FLOAT, pitch_outD, &pidPitchRate.outD)
/**
 * @brief Feedforward output pitch rate
 */
LOG_ADD(LOG_FLOAT, pitch_outFF, &pidPitchRate.outFF)
/**
 * @brief Proportional output yaw rate
 */
LOG_ADD(LOG_FLOAT, yaw_outP, &pidYawRate.outP)
/**
 * @brief Integral output yaw rate
 */
LOG_ADD(LOG_FLOAT, yaw_outI, &pidYawRate.outI)
/**
 * @brief Derivative output yaw rate
 */
LOG_ADD(LOG_FLOAT, yaw_outD, &pidYawRate.outD)
/**
 * @brief Feedforward output yaw rate
 */
LOG_ADD(LOG_FLOAT, yaw_outFF, &pidYawRate.outFF)
LOG_GROUP_STOP(pid_rate)

/**
 * Tuning settings for the gains of the PID
 * controller for the attitude of the Crazyflie which consists
 * of the Yaw Pitch and Roll 
 */
PARAM_GROUP_START(pid_attitude)
/**
 * @brief Proportional gain for the PID roll controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, roll_kp, &pidRoll.kp)
/**
 * @brief Integral gain for the PID roll controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, roll_ki, &pidRoll.ki)
/**
 * @brief Derivative gain for the PID roll controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, roll_kd, &pidRoll.kd)
/**
 * @brief Feedforward gain for the PID roll controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, roll_kff, &pidRoll.kff)
/**
 * @brief Proportional gain for the PID pitch controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, pitch_kp, &pidPitch.kp)
/**
 * @brief Integral gain for the PID pitch controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, pitch_ki, &pidPitch.ki)
/**
 * @brief Derivative gain for the PID pitch controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, pitch_kd, &pidPitch.kd)
/**
 * @brief Feedforward gain for the PID pitch controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, pitch_kff, &pidPitch.kff)
/**
 * @brief Proportional gain for the PID yaw controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yaw_kp, &pidYaw.kp)
/**
 * @brief Integral gain for the PID yaw controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yaw_ki, &pidYaw.ki)
/**
 * @brief Derivative gain for the PID yaw controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yaw_kd, &pidYaw.kd)
/**
 * @brief Feedforward gain for the PID yaw controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yaw_kff, &pidYaw.kff)
/**
 * @brief If nonzero, yaw setpoint can only be set within +/- yawMaxDelta from the current yaw
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yawMaxDelta, &yawMaxDelta)
/**
 * @brief Low pass filter enable
 */
PARAM_ADD(PARAM_INT8 | PARAM_PERSISTENT, attFiltEn, &attFiltEnable)
/**
 * @brief Low pass filter cut-off frequency (Hz)
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, attFiltCut, &attFiltCutoff)
PARAM_GROUP_STOP(pid_attitude)

/**
 * Tuning settings for the gains of the PID controller for the rate angles of
 * the Crazyflie, which consists of the yaw, pitch and roll rates 
 */
PARAM_GROUP_START(pid_rate)
/**
 * @brief Proportional gain for the PID roll rate controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, roll_kp, &pidRollRate.kp)
/**
 * @brief Integral gain for the PID roll rate controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, roll_ki, &pidRollRate.ki)
/**
 * @brief Derivative gain for the PID roll rate controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, roll_kd, &pidRollRate.kd)
/**
 * @brief Feedforward gain for the PID roll rate controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, roll_kff, &pidRollRate.kff)
/**
 * @brief Proportional gain for the PID pitch rate controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, pitch_kp, &pidPitchRate.kp)
/**
 * @brief Integral gain for the PID pitch rate controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, pitch_ki, &pidPitchRate.ki)
/**
 * @brief Derivative gain for the PID pitch rate controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, pitch_kd, &pidPitchRate.kd)
/**
 * @brief Feedforward gain for the PID pitch rate controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, pitch_kff, &pidPitchRate.kff)
/**
 * @brief Proportional gain for the PID yaw rate controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yaw_kp, &pidYawRate.kp)
/**
 * @brief Integral gain for the PID yaw rate controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yaw_ki, &pidYawRate.ki)
/**
 * @brief Derivative gain for the PID yaw rate controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yaw_kd, &pidYawRate.kd)
/**
 * @brief Feedforward gain for the PID yaw rate controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yaw_kff, &pidYawRate.kff)
/**
 * @brief Low pass filter enable
 */
PARAM_ADD(PARAM_INT8 | PARAM_PERSISTENT, rateFiltEn, &rateFiltEnable)
/**
 * @brief Low pass filter cut-off frequency, roll axis (Hz)
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, omxFiltCut, &omxFiltCutoff)
/**
 * @brief Low pass filter cut-off frequency, pitch axis (Hz)
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, omyFiltCut, &omyFiltCutoff)
/**
 * @brief Low pass filter cut-off frequency, yaw axis (Hz)
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, omzFiltCut, &omzFiltCutoff)
PARAM_GROUP_STOP(pid_rate)
