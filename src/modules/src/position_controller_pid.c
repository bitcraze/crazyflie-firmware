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

#include "commander.h"
#include "log.h"
#include "param.h"
#include "pid.h"
#include "num.h"
#include "position_controller.h"

struct pidInit_s {
  float kp;
  float ki;
  float kd;
};

struct pidAxis_s {
  PidObject pid;

  struct pidInit_s init;
  mode_t previousMode;
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
static float rpLimit  = 20;
// Velocity maximums
static float xyVelMax = 1.0f;
static float zVelMax  = 1.0f;
static float velMaxOverhead = 1.10f;

#define DT (float)(1.0f/POSITION_RATE)
#define POSITION_LPF_CUTOFF_FREQ 20.0f

#ifndef UNIT_TEST
static struct this_s this = {
  .pidVX = {
    .init = {
      .kp = 30,
      .ki = 0,
      .kd = 0,
    },
    .pid.dt = DT,
  },

  .pidVY = {
    .init = {
      .kp = 30,
      .ki = 0,
      .kd = 0,
    },
    .pid.dt = DT,
  },

  .pidVZ = {
    .init = {
      .kp = 25,
      .ki = 15,
      .kd = 0,
    },
    .pid.dt = DT,
  },

  .pidX = {
    .init = {
      .kp = 2.0f,
      .ki = 0,
      .kd = 0,
    },
    .pid.dt = DT,
  },

  .pidY = {
    .init = {
      .kp = 2.0f,
      .ki = 0,
      .kd = 0,
    },
    .pid.dt = DT,
  },

  .pidZ = {
    .init = {
      .kp = 2.0f,
      .ki = 0,
      .kd = 0.01f,
    },
    .pid.dt = DT,
  },

  .thrustBase = 36000,
  .thrustMin  = 20000,
};
#endif

void positionControllerInit()
{
  pidInit(&this.pidX.pid, this.pidX.setpoint, this.pidX.init.kp, this.pidX.init.ki, this.pidX.init.kd,
      this.pidX.pid.dt, POSITION_RATE, POSITION_LPF_CUTOFF_FREQ);
  pidInit(&this.pidY.pid, this.pidY.setpoint, this.pidY.init.kp, this.pidY.init.ki, this.pidY.init.kd,
      this.pidY.pid.dt, POSITION_RATE, POSITION_LPF_CUTOFF_FREQ);
  pidInit(&this.pidZ.pid, this.pidZ.setpoint, this.pidZ.init.kp, this.pidZ.init.ki, this.pidZ.init.kd,
      this.pidZ.pid.dt, POSITION_RATE, POSITION_LPF_CUTOFF_FREQ);

  pidInit(&this.pidVX.pid, this.pidVX.setpoint, this.pidVX.init.kp, this.pidVX.init.ki, this.pidVX.init.kd,
      this.pidVX.pid.dt, POSITION_RATE, POSITION_LPF_CUTOFF_FREQ);
  pidInit(&this.pidVY.pid, this.pidVY.setpoint, this.pidVY.init.kp, this.pidVY.init.ki, this.pidVY.init.kd,
      this.pidVY.pid.dt, POSITION_RATE, POSITION_LPF_CUTOFF_FREQ);
  pidInit(&this.pidVZ.pid, this.pidVZ.setpoint, this.pidVZ.init.kp, this.pidVZ.init.ki, this.pidVZ.init.kd,
      this.pidVZ.pid.dt, POSITION_RATE, POSITION_LPF_CUTOFF_FREQ);

  this.pidX.pid.errorMax = xyVelMax * velMaxOverhead;
  this.pidY.pid.errorMax = xyVelMax * velMaxOverhead;
  this.pidZ.pid.errorMax = zVelMax  * velMaxOverhead;
}

static float runPid(float input, struct pidAxis_s *axis, float setpoint, float dt) {
  axis->setpoint = setpoint;

  pidSetDesired(&axis->pid, axis->setpoint);
  return pidUpdate(&axis->pid, input, true);
}

void positionController(float* thrust, attitude_t *attitude, const state_t *state,
                                                             const setpoint_t *setpoint)
{
  // X, Y
  pidSetDesired(&this.pidVX.pid, runPid(state->position.x, &this.pidX, setpoint->position.x, DT));
  pidSetDesired(&this.pidVY.pid, runPid(state->position.y, &this.pidY, setpoint->position.y, DT));
  pidSetDesired(&this.pidVZ.pid, runPid(state->position.z, &this.pidZ, setpoint->position.z, DT));

  velocityController(thrust, attitude, state, setpoint);
}

void velocityController(float* thrust, attitude_t *attitude, const state_t *state,
                                                             const setpoint_t *setpoint)
{
  // Roll and Pitch
  float rollRaw  = pidUpdate(&this.pidVX.pid, state->velocity.x, true);
  float pitchRaw = pidUpdate(&this.pidVY.pid, state->velocity.y, true);

  float yawRad = state->attitude.yaw * (float)M_PI / 180;
  attitude->pitch = -(rollRaw  * cosf(yawRad)) - (pitchRaw * sinf(yawRad));
  attitude->roll  = -(pitchRaw * cosf(yawRad)) + (rollRaw  * sinf(yawRad));

  // Check for roll/pitch limits and prevent I from accumulating when capped
  if (abs(attitude->roll) > rpLimit || abs(attitude->pitch) > rpLimit) {
      this.pidVX.pid.iCapped = true;
      this.pidVY.pid.iCapped = true;
  } else {
      this.pidVX.pid.iCapped = false;
      this.pidVY.pid.iCapped = false;
  }

  attitude->roll  = constrain(attitude->roll,  -rpLimit, rpLimit);
  attitude->pitch = constrain(attitude->pitch, -rpLimit, rpLimit);

  // Thrust
  float thrustRaw = pidUpdate(&this.pidVZ.pid, state->velocity.z, true);
  // Scale the thrust and add feed forward term
  *thrust = thrustRaw*1000 + this.thrustBase;
  // Check for minimum thrust, prevent I from accumulating when capped
  if (*thrust < this.thrustMin) {
    *thrust = this.thrustMin;
    this.pidVZ.pid.iCapped = true;
  } else {
    this.pidVZ.pid.iCapped = false;
  }
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

LOG_GROUP_START(posCtl)

LOG_ADD(LOG_FLOAT, targetVX, &this.pidVX.pid.desired)
LOG_ADD(LOG_FLOAT, targetVY, &this.pidVY.pid.desired)
LOG_ADD(LOG_FLOAT, targetVZ, &this.pidVZ.pid.desired)

LOG_ADD(LOG_FLOAT, targetX, &this.pidX.pid.desired)
LOG_ADD(LOG_FLOAT, targetY, &this.pidY.pid.desired)
LOG_ADD(LOG_FLOAT, targetZ, &this.pidZ.pid.desired)

LOG_ADD(LOG_FLOAT, Xp, &this.pidX.pid.outP)
LOG_ADD(LOG_FLOAT, Xi, &this.pidX.pid.outI)
LOG_ADD(LOG_FLOAT, Xd, &this.pidX.pid.outD)

LOG_ADD(LOG_FLOAT, Yp, &this.pidY.pid.outP)
LOG_ADD(LOG_FLOAT, Yi, &this.pidY.pid.outI)
LOG_ADD(LOG_FLOAT, Yd, &this.pidY.pid.outD)

LOG_ADD(LOG_FLOAT, Zp, &this.pidZ.pid.outP)
LOG_ADD(LOG_FLOAT, Zi, &this.pidZ.pid.outI)
LOG_ADD(LOG_FLOAT, Zd, &this.pidZ.pid.outD)

LOG_ADD(LOG_FLOAT, VZp, &this.pidVZ.pid.outP)
LOG_ADD(LOG_FLOAT, VZi, &this.pidVZ.pid.outI)
LOG_ADD(LOG_FLOAT, VZd, &this.pidVZ.pid.outD)

LOG_GROUP_STOP(posCtl)

PARAM_GROUP_START(velCtlPid)

PARAM_ADD(PARAM_FLOAT, vxKp, &this.pidVX.pid.kp)
PARAM_ADD(PARAM_FLOAT, vxKi, &this.pidVX.pid.ki)
PARAM_ADD(PARAM_FLOAT, vxKd, &this.pidVX.pid.kd)

PARAM_ADD(PARAM_FLOAT, vyKp, &this.pidVY.pid.kp)
PARAM_ADD(PARAM_FLOAT, vyKi, &this.pidVY.pid.ki)
PARAM_ADD(PARAM_FLOAT, vyKd, &this.pidVY.pid.kd)

PARAM_ADD(PARAM_FLOAT, vzKp, &this.pidVZ.pid.kp)
PARAM_ADD(PARAM_FLOAT, vzKi, &this.pidVZ.pid.ki)
PARAM_ADD(PARAM_FLOAT, vzKd, &this.pidVZ.pid.kd)

PARAM_GROUP_STOP(velCtlPid)

PARAM_GROUP_START(posCtlPid)

PARAM_ADD(PARAM_FLOAT, xKp, &this.pidX.pid.kp)
PARAM_ADD(PARAM_FLOAT, xKi, &this.pidX.pid.ki)
PARAM_ADD(PARAM_FLOAT, xKd, &this.pidX.pid.kd)

PARAM_ADD(PARAM_FLOAT, yKp, &this.pidY.pid.kp)
PARAM_ADD(PARAM_FLOAT, yKi, &this.pidY.pid.ki)
PARAM_ADD(PARAM_FLOAT, yKd, &this.pidY.pid.kd)

PARAM_ADD(PARAM_FLOAT, zKp, &this.pidZ.pid.kp)
PARAM_ADD(PARAM_FLOAT, zKi, &this.pidZ.pid.ki)
PARAM_ADD(PARAM_FLOAT, zKd, &this.pidZ.pid.kd)

PARAM_ADD(PARAM_UINT16, thrustBase, &this.thrustBase)
PARAM_ADD(PARAM_UINT16, thrustMin, &this.thrustMin)

PARAM_ADD(PARAM_FLOAT, rpLimit, &rpLimit)

PARAM_GROUP_STOP(posCtlPid)
