/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2016 Bitcraze AB
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
 * position_estimator_pid.c: PID-based implementation of the position controller
 */

#include <math.h>
#include "num.h"

#include "log.h"
#include "param.h"
#include "pid.h"
#include "num.h"
#include "position_controller.h"
#include "platform_defaults.h"


struct pidAxis_s {
  PidObject pid;

  stab_mode_t previousMode;
  float setpoint;

  float output;
};

struct this_s {
  struct pidAxis_s pidVX;
  struct pidAxis_s pidVY;
  struct pidAxis_s pidVZ;

  struct pidAxis_s pidX;
  struct pidAxis_s pidY;
  struct pidAxis_s pidZ;

  uint16_t thrustBase; // approximate throttle needed when in perfect hover. More weight/older battery can use a higher value
  uint16_t thrustMin;  // Minimum thrust value to output
};

// Maximum roll/pitch angle permited
static float rLimit = PID_VEL_ROLL_MAX;
static float pLimit = PID_VEL_PITCH_MAX;
static float rpLimitOverhead = 1.10f;
// Velocity maximums
static float xVelMax = PID_POS_VEL_X_MAX;
static float yVelMax = PID_POS_VEL_Y_MAX;
static float zVelMax = PID_POS_VEL_Z_MAX;
static float velMaxOverhead = 1.10f;

static const float thrustScale = 1000.0f;

#define DT (float)(1.0f/POSITION_RATE)
static bool posFiltEnable = PID_POS_XY_FILT_ENABLE;
static bool velFiltEnable = PID_VEL_XY_FILT_ENABLE;
static float posFiltCutoff = PID_POS_XY_FILT_CUTOFF;
static float velFiltCutoff = PID_VEL_XY_FILT_CUTOFF;
static bool posZFiltEnable = PID_POS_Z_FILT_ENABLE;
static bool velZFiltEnable = PID_VEL_Z_FILT_ENABLE;
static float posZFiltCutoff = PID_POS_Z_FILT_CUTOFF;
#if CONFIG_CONTROLLER_PID_IMPROVED_BARO_Z_HOLD
static float velZFiltCutoff = PID_VEL_Z_FILT_CUTOFF_BARO_Z_HOLD;
#else
static float velZFiltCutoff = PID_VEL_Z_FILT_CUTOFF;
#endif

#ifndef UNIT_TEST
static struct this_s this = {
  .pidVX = {
    .pid = {
      .kp = PID_VEL_X_KP,
      .ki = PID_VEL_X_KI,
      .kd = PID_VEL_X_KD,
      .kff = PID_VEL_X_KFF,
    },
    .pid.dt = DT,
  },

  .pidVY = {
    .pid = {
      .kp = PID_VEL_Y_KP,
      .ki = PID_VEL_Y_KI,
      .kd = PID_VEL_Y_KD,
      .kff = PID_VEL_Y_KFF,
    },
    .pid.dt = DT,
  },
  #if CONFIG_CONTROLLER_PID_IMPROVED_BARO_Z_HOLD
    .pidVZ = {
      .pid = {
        .kp = PID_VEL_Z_KP_BARO_Z_HOLD,
        .ki = PID_VEL_Z_KI_BARO_Z_HOLD,
        .kd = PID_VEL_Z_KD_BARO_Z_HOLD,
        .kff = PID_VEL_Z_KFF_BARO_Z_HOLD,
      },
      .pid.dt = DT,
    },
  #else
    .pidVZ = {
      .pid = {
        .kp = PID_VEL_Z_KP,
        .ki = PID_VEL_Z_KI,
        .kd = PID_VEL_Z_KD,
        .kff = PID_VEL_Z_KFF,
      },
      .pid.dt = DT,
    },
  #endif
  .pidX = {
    .pid = {
      .kp = PID_POS_X_KP,
      .ki = PID_POS_X_KI,
      .kd = PID_POS_X_KD,
      .kff = PID_POS_X_KFF,
    },
    .pid.dt = DT,
  },

  .pidY = {
    .pid = {
      .kp = PID_POS_Y_KP,
      .ki = PID_POS_Y_KI,
      .kd = PID_POS_Y_KD,
      .kff = PID_POS_Y_KFF,
    },
    .pid.dt = DT,
  },

  .pidZ = {
    .pid = {
      .kp = PID_POS_Z_KP,
      .ki = PID_POS_Z_KI,
      .kd = PID_POS_Z_KD,
      .kff = PID_POS_Z_KFF,
    },
    .pid.dt = DT,
  },
  #if CONFIG_CONTROLLER_PID_IMPROVED_BARO_Z_HOLD
    .thrustBase = PID_VEL_THRUST_BASE_BARO_Z_HOLD,
  #else
    .thrustBase = PID_VEL_THRUST_BASE,
  #endif
  .thrustMin  = PID_VEL_THRUST_MIN,
};
#endif

void positionControllerInit()
{
  pidInit(&this.pidX.pid, this.pidX.setpoint, this.pidX.pid.kp, this.pidX.pid.ki, this.pidX.pid.kd,
      this.pidX.pid.kff, this.pidX.pid.dt, POSITION_RATE, posFiltCutoff, posFiltEnable);
  pidInit(&this.pidY.pid, this.pidY.setpoint, this.pidY.pid.kp, this.pidY.pid.ki, this.pidY.pid.kd,
      this.pidY.pid.kff, this.pidY.pid.dt, POSITION_RATE, posFiltCutoff, posFiltEnable);
  pidInit(&this.pidZ.pid, this.pidZ.setpoint, this.pidZ.pid.kp, this.pidZ.pid.ki, this.pidZ.pid.kd,
      this.pidZ.pid.kff, this.pidZ.pid.dt, POSITION_RATE, posZFiltCutoff, posZFiltEnable);

  pidInit(&this.pidVX.pid, this.pidVX.setpoint, this.pidVX.pid.kp, this.pidVX.pid.ki, this.pidVX.pid.kd,
      this.pidVX.pid.kff, this.pidVX.pid.dt, POSITION_RATE, velFiltCutoff, velFiltEnable);
  pidInit(&this.pidVY.pid, this.pidVY.setpoint, this.pidVY.pid.kp, this.pidVY.pid.ki, this.pidVY.pid.kd,
      this.pidVY.pid.kff, this.pidVY.pid.dt, POSITION_RATE, velFiltCutoff, velFiltEnable);
  pidInit(&this.pidVZ.pid, this.pidVZ.setpoint, this.pidVZ.pid.kp, this.pidVZ.pid.ki, this.pidVZ.pid.kd,
      this.pidVZ.pid.kff, this.pidVZ.pid.dt, POSITION_RATE, velZFiltCutoff, velZFiltEnable);
}

static float runPid(float input, struct pidAxis_s *axis, float setpoint, float dt) {
  axis->setpoint = setpoint;

  pidSetDesired(&axis->pid, axis->setpoint);
  return pidUpdate(&axis->pid, input, true);
}


float state_body_x, state_body_y, state_body_vx, state_body_vy;

void positionController(float* thrust, attitude_t *attitude, const setpoint_t *setpoint,
                                                             const state_t *state)
{
  this.pidX.pid.outputLimit = xVelMax * velMaxOverhead;
  this.pidY.pid.outputLimit = yVelMax * velMaxOverhead;
  // The ROS landing detector will prematurely trip if
  // this value is below 0.5
  this.pidZ.pid.outputLimit = fmaxf(zVelMax, 0.5f)  * velMaxOverhead;

  float cosyaw = cosf(state->attitude.yaw * (float)M_PI / 180.0f);
  float sinyaw = sinf(state->attitude.yaw * (float)M_PI / 180.0f);

  float setp_body_x = setpoint->position.x * cosyaw + setpoint->position.y * sinyaw;
  float setp_body_y = -setpoint->position.x * sinyaw + setpoint->position.y * cosyaw;

  state_body_x = state->position.x * cosyaw + state->position.y * sinyaw;
  state_body_y = -state->position.x * sinyaw + state->position.y * cosyaw;

  float globalvx = setpoint->velocity.x;
  float globalvy = setpoint->velocity.y;

  //X, Y
  Axis3f setpoint_velocity;
  setpoint_velocity.x = setpoint->velocity.x;
  setpoint_velocity.y = setpoint->velocity.y;
  setpoint_velocity.z = setpoint->velocity.z;
  if (setpoint->mode.x == modeAbs) {
    setpoint_velocity.x = runPid(state_body_x, &this.pidX, setp_body_x, DT);
  } else if (!setpoint->velocity_body) {
    setpoint_velocity.x = globalvx * cosyaw + globalvy * sinyaw;
  }
  if (setpoint->mode.y == modeAbs) {
    setpoint_velocity.y = runPid(state_body_y, &this.pidY, setp_body_y, DT);
  } else if (!setpoint->velocity_body) {
    setpoint_velocity.y = globalvy * cosyaw - globalvx * sinyaw;
  }
  if (setpoint->mode.z == modeAbs) {
    setpoint_velocity.z = runPid(state->position.z, &this.pidZ, setpoint->position.z, DT);
  }

  velocityController(thrust, attitude, &setpoint_velocity, state);
}

void velocityController(float* thrust, attitude_t *attitude, const Axis3f* setpoint_velocity,
                                                             const state_t *state)
{
  this.pidVX.pid.outputLimit = pLimit * rpLimitOverhead;
  this.pidVY.pid.outputLimit = rLimit * rpLimitOverhead;
  // Set the output limit to the maximum thrust range
  this.pidVZ.pid.outputLimit = (UINT16_MAX / 2 / thrustScale);
  //this.pidVZ.pid.outputLimit = (this.thrustBase - this.thrustMin) / thrustScale;

  float cosyaw = cosf(state->attitude.yaw * (float)M_PI / 180.0f);
  float sinyaw = sinf(state->attitude.yaw * (float)M_PI / 180.0f);
  state_body_vx = state->velocity.x * cosyaw + state->velocity.y * sinyaw;
  state_body_vy = -state->velocity.x * sinyaw + state->velocity.y * cosyaw;

  // Roll and Pitch
  attitude->pitch = -runPid(state_body_vx, &this.pidVX, setpoint_velocity->x, DT);
  attitude->roll = -runPid(state_body_vy, &this.pidVY, setpoint_velocity->y, DT);

  attitude->roll  = constrain(attitude->roll,  -rLimit, rLimit);
  attitude->pitch = constrain(attitude->pitch, -pLimit, pLimit);

  // Thrust
  float thrustRaw = runPid(state->velocity.z, &this.pidVZ, setpoint_velocity->z, DT);
  // Scale the thrust and add feed forward term
  *thrust = thrustRaw*thrustScale + this.thrustBase;
  // Check for minimum thrust
  if (*thrust < this.thrustMin) {
    *thrust = this.thrustMin;
  }
    // saturate
  *thrust = constrain(*thrust, 0, UINT16_MAX);
}

void positionControllerResetAllPID()
{
  pidReset(&this.pidX.pid);
  pidReset(&this.pidY.pid);
  pidReset(&this.pidZ.pid);
  pidReset(&this.pidVX.pid);
  pidReset(&this.pidVY.pid);
  pidReset(&this.pidVZ.pid);
}

void positionControllerResetAllfilters() {
  filterReset(&this.pidX.pid, POSITION_RATE, posFiltCutoff, posFiltEnable);
  filterReset(&this.pidY.pid, POSITION_RATE, posFiltCutoff, posFiltEnable);
  filterReset(&this.pidZ.pid, POSITION_RATE, posZFiltCutoff, posZFiltEnable);
  filterReset(&this.pidVX.pid, POSITION_RATE, velFiltCutoff, velFiltEnable);
  filterReset(&this.pidVY.pid, POSITION_RATE, velFiltCutoff, velFiltEnable);
  filterReset(&this.pidVZ.pid, POSITION_RATE, velZFiltCutoff, velZFiltEnable);
}

/**
 * Log variables of the PID position controller
 *
 * Note: rename to posCtrlPID ?
 */
LOG_GROUP_START(posCtl)

/**
 * @brief PID controller target desired body-yaw-aligned velocity x [m/s]
 *
 * Note: Same as stabilizer log
 */
LOG_ADD(LOG_FLOAT, targetVX, &this.pidVX.pid.desired)
/**
 * @brief PID controller target desired body-yaw-aligned velocity y [m/s]
 *
 * Note: Same as stabilizer log
 */
LOG_ADD(LOG_FLOAT, targetVY, &this.pidVY.pid.desired)
/**
 * @brief PID controller target desired velocity z [m/s]
 *
 * Note: Same as stabilizer log
 */
LOG_ADD(LOG_FLOAT, targetVZ, &this.pidVZ.pid.desired)
/**
 * @brief PID controller target desired body-yaw-aligned position x [m]
 *
 * Note: Same as stabilizer log
 */
LOG_ADD(LOG_FLOAT, targetX, &this.pidX.pid.desired)
/**
 * @brief PID controller target desired body-yaw-aligned position y [m]
 *
 * Note: Same as stabilizer log
 */
LOG_ADD(LOG_FLOAT, targetY, &this.pidY.pid.desired)
/**
 * @brief PID controller target desired global position z [m]
 *
 * Note: Same as stabilizer log
 */
LOG_ADD(LOG_FLOAT, targetZ, &this.pidZ.pid.desired)

/**
 * @brief PID state body-yaw-aligned velocity x [m/s]
 *
 */
LOG_ADD(LOG_FLOAT, bodyVX, &state_body_vx)
/**
 * @brief PID state body-yaw-aligned velocity y [m/s]
 *
 */
LOG_ADD(LOG_FLOAT, bodyVY, &state_body_vy)
/**
 * @brief PID state body-yaw-aligned position x [m]
 *
 */
LOG_ADD(LOG_FLOAT, bodyX, &state_body_x)
/**
 * @brief PID state body-yaw-aligned position y [m]
 *
 */
LOG_ADD(LOG_FLOAT, bodyY, &state_body_y)

/**
 * @brief PID proportional output position x
 */
LOG_ADD(LOG_FLOAT, Xp, &this.pidX.pid.outP)
/**
 * @brief PID integral output position x
 */
LOG_ADD(LOG_FLOAT, Xi, &this.pidX.pid.outI)
/**
 * @brief PID derivative output position x
 */
LOG_ADD(LOG_FLOAT, Xd, &this.pidX.pid.outD)
/**
 * @brief PID feedforward output position x
 */
LOG_ADD(LOG_FLOAT, Xff, &this.pidX.pid.outFF)

/**
 * @brief PID proportional output position y
 */
LOG_ADD(LOG_FLOAT, Yp, &this.pidY.pid.outP)
/**
 * @brief PID integral output position y
 */
LOG_ADD(LOG_FLOAT, Yi, &this.pidY.pid.outI)
/**
 * @brief PID derivative output position y
 */
LOG_ADD(LOG_FLOAT, Yd, &this.pidY.pid.outD)
/**
 * @brief PID feedforward output position y
 */
LOG_ADD(LOG_FLOAT, Yff, &this.pidY.pid.outFF)

/**
 * @brief PID proportional output position z
 */
LOG_ADD(LOG_FLOAT, Zp, &this.pidZ.pid.outP)
/**
 * @brief PID integral output position z
 */
LOG_ADD(LOG_FLOAT, Zi, &this.pidZ.pid.outI)
/**
 * @brief PID derivative output position z
 */
LOG_ADD(LOG_FLOAT, Zd, &this.pidZ.pid.outD)
/**
 * @brief PID feedforward output position z
 */
LOG_ADD(LOG_FLOAT, Zff, &this.pidZ.pid.outFF)

/**
 * @brief PID proportional output velocity x
 */
LOG_ADD(LOG_FLOAT, VXp, &this.pidVX.pid.outP)
/**
 * @brief PID integral output velocity x
 */
LOG_ADD(LOG_FLOAT, VXi, &this.pidVX.pid.outI)
/**
 * @brief PID derivative output velocity x
 */
LOG_ADD(LOG_FLOAT, VXd, &this.pidVX.pid.outD)
/**
 * @brief PID feedforward output velocity x
 */
LOG_ADD(LOG_FLOAT, VXff, &this.pidVX.pid.outFF)

/**
 * @brief PID proportional output velocity y
 */
LOG_ADD(LOG_FLOAT, VYp, &this.pidVY.pid.outP)
/**
 * @brief PID integral output velocity y
 */
LOG_ADD(LOG_FLOAT, VYi, &this.pidVY.pid.outI)
/**
 * @brief PID derivative output velocity y
 */
LOG_ADD(LOG_FLOAT, VYd, &this.pidVY.pid.outD)
/**
 * @brief PID feedforward output velocity y
 */
LOG_ADD(LOG_FLOAT, VYff, &this.pidVY.pid.outFF)

/**
 * @brief PID proportional output velocity z
 */
LOG_ADD(LOG_FLOAT, VZp, &this.pidVZ.pid.outP)
/**
 * @brief PID integral output velocity z
 */
LOG_ADD(LOG_FLOAT, VZi, &this.pidVZ.pid.outI)
/**
 * @brief PID integral output velocity z
 */
LOG_ADD(LOG_FLOAT, VZd, &this.pidVZ.pid.outD)
/**
 * @brief PID feedforward output velocity z
 */
LOG_ADD(LOG_FLOAT, VZff, &this.pidVZ.pid.outFF)

LOG_GROUP_STOP(posCtl)

/**
 * Tuning settings for the gains of the PID
 * controller for the velocity of the Crazyflie ¨
 * in the body-yaw-aligned X & Y and global Z directions.
 */
PARAM_GROUP_START(velCtlPid)
/**
 * @brief Proportional gain for the velocity PID in the body-yaw-aligned X direction
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, vxKp, &this.pidVX.pid.kp)
/**
 * @brief Integral gain for the velocity PID in the body-yaw-aligned X direction
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, vxKi, &this.pidVX.pid.ki)
/**
 * @brief Derivative gain for the velocity PID in the body-yaw-aligned X direction
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, vxKd, &this.pidVX.pid.kd)
/**
 * @brief Feedforward gain for the velocity PID in the body-yaw-aligned X direction (in degrees per m/s)
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, vxKFF, &this.pidVX.pid.kff)

/**
 * @brief Proportional gain for the velocity PID in the body-yaw-aligned Y direction
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, vyKp, &this.pidVY.pid.kp)
/**
 * @brief Integral gain for the velocity PID in the body-yaw-aligned Y direction
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, vyKi, &this.pidVY.pid.ki)
/**
 * @brief Derivative gain for the velocity PID in the body-yaw-aligned Y direction
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, vyKd, &this.pidVY.pid.kd)
/**
 * @brief Feedforward gain for the velocity PID in the body-yaw-aligned Y direction (in degrees per m/s)
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, vyKFF, &this.pidVY.pid.kff)

/**
 * @brief Proportional gain for the velocity PID in the global Z direction
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, vzKp, &this.pidVZ.pid.kp)
/**
 * @brief Integral gain for the velocity PID in the global Z direction
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, vzKi, &this.pidVZ.pid.ki)
/**
 * @brief Derivative gain for the velocity PID in the global Z direction
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, vzKd, &this.pidVZ.pid.kd)
/**
 * @brief Feedforward gain for the velocity PID in the global direction (in degrees per m/s)
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, vzKFF, &this.pidVZ.pid.kff)

PARAM_GROUP_STOP(velCtlPid)

/**
 * Tuning settings for the gains of the PID
 * controller for the position of the Crazyflie ¨
 * in the body-yaw-aligned X & Y and global Z directions.
 */
PARAM_GROUP_START(posCtlPid)
/**
 * @brief Proportional gain for the position PID in the body-yaw-aligned X direction
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, xKp, &this.pidX.pid.kp)
/**
 * @brief Integral gain for the position PID in the body-yaw-aligned X direction
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, xKi, &this.pidX.pid.ki)
/**
 * @brief Derivative gain for the position PID in the body-yaw-aligned X direction
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, xKd, &this.pidX.pid.kd)
/**
 * @brief Feedforward gain for the position PID in the body-yaw-aligned X direction
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, xKff, &this.pidX.pid.kff)

/**
 * @brief Proportional gain for the position PID in the body-yaw-aligned Y direction
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yKp, &this.pidY.pid.kp)
/**
 * @brief Integral gain for the position PID in the body-yaw-aligned Y direction
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yKi, &this.pidY.pid.ki)
/**
 * @brief Derivative gain for the position PID in the body-yaw-aligned Y direction
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yKd, &this.pidY.pid.kd)
/**
 * @brief Feedforward gain for the position PID in the body-yaw-aligned Y direction
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yKff, &this.pidY.pid.kff)

/**
 * @brief Proportional gain for the position PID in the global Z direction
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, zKp, &this.pidZ.pid.kp)
/**
 * @brief Integral gain for the position PID in the global Z direction
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, zKi, &this.pidZ.pid.ki)
/**
 * @brief Derivative gain for the position PID in the global Z direction
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, zKd, &this.pidZ.pid.kd)
/**
 * @brief Feedforward gain for the position PID in the body-yaw-aligned Z direction
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, zKff, &this.pidZ.pid.kff)

/**
 * @brief Approx. thrust needed for hover
 */
PARAM_ADD(PARAM_UINT16 | PARAM_PERSISTENT, thrustBase, &this.thrustBase)
/**
 * @brief Min. thrust value to output
 */
PARAM_ADD(PARAM_UINT16 | PARAM_PERSISTENT, thrustMin, &this.thrustMin)

/**
 * @brief Roll absolute limit
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, rLimit,  &rLimit)
/**
 * @brief Pitch absolute limit
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, pLimit,  &pLimit)
/**
 * @brief Maximum body-yaw-aligned X velocity
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, xVelMax, &xVelMax)
/**
 * @brief Maximum body-yaw-aligned Y velocity
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yVelMax, &yVelMax)
/**
 * @brief Maximum Z Velocity
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, zVelMax,  &zVelMax)

PARAM_GROUP_STOP(posCtlPid)
