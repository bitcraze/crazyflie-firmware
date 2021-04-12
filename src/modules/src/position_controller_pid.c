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
static float rpLimit  = 30; // global control
static float rLimit  = 30; // in-body control
static float pLimit  = 30;
static float rpLimitOverhead = 1.10f;
// Velocity maximums
static float xyVelMax = 1.0f; // global control
static float xBodyVelMax = 1.0f; // in-body control
static float yBodyVelMax = 1.0f;
static float zVelMax  = 1.0f;
static float velMaxOverhead = 1.10f;

static const float thrustScale = 1000.0f;

static float kFFx = 10.0; // feedforward term for x direction [deg / m/s]
static float kFFy = 10.0; // feedforward term for x direction [deg / m/s]

float bank_roll = 0.0f; // for logging & debugging
float bank_pitch = 0.0f;

float setpointx = 0.0f;
float setpointy = 0.0f;
float setpointvx = 0.0f;
float setpointvy = 0.0f;

#define DT (float)(1.0f/POSITION_RATE)
#define POSITION_LPF_CUTOFF_FREQ 5.0f
#define POSITION_LPF_ENABLE false
#define VELOCITY_LPF_CUTOFF_FREQ 10.0f
#define VELOCITY_LPF_ENABLE true

#define ZPOSITION_LPF_CUTOFF_FREQ 5.0f
#define ZPOSITION_LPF_ENABLE false
#define ZVELOCITY_LPF_CUTOFF_FREQ 10.0f
#define ZVELOCITY_LPF_ENABLE true

#define POSITION_CONTROL_IN_BODY true
#define POSITION_CONTROL_SINGLE_LOOP false

bool singleLoop = POSITION_CONTROL_SINGLE_LOOP;
bool posFiltEnable = POSITION_LPF_ENABLE;
bool velFiltEnable = VELOCITY_LPF_ENABLE;
float posFiltCutoff = POSITION_LPF_CUTOFF_FREQ;
float velFiltCutoff = VELOCITY_LPF_CUTOFF_FREQ;
bool posZFiltEnable = ZPOSITION_LPF_ENABLE;
bool velZFiltEnable = ZVELOCITY_LPF_ENABLE;
float posZFiltCutoff = ZPOSITION_LPF_CUTOFF_FREQ;
float velZFiltCutoff = ZVELOCITY_LPF_CUTOFF_FREQ;

#ifndef UNIT_TEST
static struct this_s this = {
  .pidVX = {
    .init = {
      .kp = 0.0f,
      .ki = 0.0f,
      .kd = 0.0f,
    },
    .pid.dt = DT,
  },

  .pidVY = {
    .init = {
      .kp = 0.0f,
      .ki = 0.0f,
      .kd = 0.0f,
    },
    .pid.dt = DT,
  },

  .pidVZ = {
    .init = {
      .kp = 0,
      .ki = 0,
      .kd = 0,
    },
    .pid.dt = DT,
  },

  .pidX = {
    .init = {
      .kp = 32.0f,
      .ki = 0,
      .kd = 8.0f,
    },
    .pid.dt = DT,
  },

  .pidY = {
    .init = {
      .kp = 17.0f,
      .ki = 0,
      .kd = 10.0f,
    },
    .pid.dt = DT,
  },

  .pidZ = {
    .init = {
      .kp = 62.5f,
      .ki = 6.0f,
      .kd = 12.5f,
    },
    .pid.dt = DT,
  },

  .thrustBase = 27000,
  .thrustMin  = 15000,
};
#endif

void positionControllerInit()
{
  // Enabling filters to be tuned as config. parameters
  pidInit(&this.pidX.pid, this.pidX.setpoint, this.pidX.init.kp, this.pidX.init.ki, this.pidX.init.kd,
      this.pidX.pid.dt, POSITION_RATE, posFiltCutoff, posFiltEnable);
  pidInit(&this.pidY.pid, this.pidY.setpoint, this.pidY.init.kp, this.pidY.init.ki, this.pidY.init.kd,
      this.pidY.pid.dt, POSITION_RATE, posFiltCutoff, posFiltEnable);
  pidInit(&this.pidZ.pid, this.pidZ.setpoint, this.pidZ.init.kp, this.pidZ.init.ki, this.pidZ.init.kd,
      this.pidZ.pid.dt, POSITION_RATE, posFiltCutoff, posZFiltEnable);

  pidInit(&this.pidVX.pid, this.pidVX.setpoint, this.pidVX.init.kp, this.pidVX.init.ki, this.pidVX.init.kd,
      this.pidVX.pid.dt, POSITION_RATE, velFiltCutoff, velFiltEnable);
  pidInit(&this.pidVY.pid, this.pidVY.setpoint, this.pidVY.init.kp, this.pidVY.init.ki, this.pidVY.init.kd,
      this.pidVY.pid.dt, POSITION_RATE, velFiltCutoff, velFiltEnable);
  pidInit(&this.pidVZ.pid, this.pidVZ.setpoint, this.pidVZ.init.kp, this.pidVZ.init.ki, this.pidVZ.init.kd,
      this.pidVZ.pid.dt, POSITION_RATE, velFiltCutoff, velZFiltEnable);
}

static float runPid(float input, struct pidAxis_s *axis, float setpoint, float dt) {
  axis->setpoint = setpoint;

  pidSetDesired(&axis->pid, axis->setpoint);
  return pidUpdate(&axis->pid, input, true);
}

void positionControllerInBodySingleLoop(float* thrust, attitude_t *attitude, setpoint_t *setpoint,
                                                             const state_t *state)
{
  float cosyaw = cosf(state->attitude.yaw * (float)M_PI / 180.0f);
  float sinyaw = sinf(state->attitude.yaw * (float)M_PI / 180.0f);
  
  this.pidX.pid.outputLimit = pLimit * rpLimitOverhead;
  this.pidY.pid.outputLimit = rLimit * rpLimitOverhead;
  this.pidZ.pid.outputLimit = (UINT16_MAX / 2 / thrustScale);

  float setp_body_x = setpoint->position.x * cosyaw + setpoint->position.y * sinyaw;
  float setp_body_y = -setpoint->position.x * sinyaw + setpoint->position.y * cosyaw;

  float state_body_x = state->position.x * cosyaw + state->position.y * sinyaw;
  float state_body_y = -state->position.x * sinyaw + state->position.y * cosyaw;
  
  attitude->pitch = -runPid(state_body_x, &this.pidX, setp_body_x, DT);
  attitude->roll = -runPid(state_body_y, &this.pidY, setp_body_y, DT);
  float thrustRaw = runPid(state->position.z, &this.pidZ, setpoint->position.z, DT);

  attitude->roll  = constrain(attitude->roll,  -rLimit, rLimit);
  attitude->pitch = constrain(attitude->pitch, -pLimit, pLimit);

  bank_roll = attitude->roll; // for logging & debugging only
  bank_pitch = attitude->pitch;
  
  // Scale the thrust and add feed forward term
  *thrust = thrustRaw*thrustScale + this.thrustBase;
  // Check for minimum thrust
  if (*thrust < this.thrustMin) {
    *thrust = this.thrustMin;
  }
  // saturate
  *thrust = constrain(*thrust, 0, UINT16_MAX);
}

void positionControllerInBody(float* thrust, attitude_t *attitude, setpoint_t *setpoint,
                                                             const state_t *state)
{
  this.pidX.pid.outputLimit = xBodyVelMax * velMaxOverhead;
  this.pidY.pid.outputLimit = yBodyVelMax * velMaxOverhead;
  this.pidZ.pid.outputLimit = zVelMax * velMaxOverhead;
  // // The ROS landing detector will prematurely trip if
  // // this value is below 0.5
  // this.pidZ.pid.outputLimit = fmaxf(zVelMax, 0.5f)  * velMaxOverhead;
  
  float cosyaw = cosf(state->attitude.yaw * (float)M_PI / 180.0f);
  float sinyaw = sinf(state->attitude.yaw * (float)M_PI / 180.0f);

  float setp_body_x = setpoint->position.x * cosyaw + setpoint->position.y * sinyaw;
  float setp_body_y = -setpoint->position.x * sinyaw + setpoint->position.y * cosyaw;

  float state_body_x = state->position.x * cosyaw + state->position.y * sinyaw;
  float state_body_y = -state->position.x * sinyaw + state->position.y * cosyaw;
    
  float globalvx = setpoint->velocity.x;
  float globalvy = setpoint->velocity.y;

  //X, Y
  if (setpoint->mode.x == modeAbs) {
    setpoint->velocity.x = runPid(state_body_x, &this.pidX, setp_body_x, DT);
  } else if (!setpoint->velocity_body) {
    setpoint->velocity.x = globalvx * cosyaw + globalvy * sinyaw;
  }
  if (setpoint->mode.y == modeAbs) {
    setpoint->velocity.y = runPid(state_body_y, &this.pidY, setp_body_y, DT);
    globalvx = setpoint->velocity.x*cosyaw - setpoint->velocity.y*sinyaw;
    globalvy = setpoint->velocity.x*sinyaw + setpoint->velocity.y*cosyaw;
  } else if (!setpoint->velocity_body) {
    setpoint->velocity.y = globalvy * cosyaw - globalvx * sinyaw;
  }
  if (setpoint->mode.z == modeAbs) {
    setpoint->velocity.z = runPid(state->position.z, &this.pidZ, setpoint->position.z, DT);
  }

  setpointvx = globalvx;
  setpointvy = globalvy;

  velocityControllerInBody(thrust, attitude, setpoint, state);
}

void positionControllerInGlobal(float* thrust, attitude_t *attitude, setpoint_t *setpoint,
                                                             const state_t *state)
{
  this.pidX.pid.outputLimit = xyVelMax * velMaxOverhead;
  this.pidY.pid.outputLimit = xyVelMax * velMaxOverhead;
  // The ROS landing detector will prematurely trip if
  // this value is below 0.5
  this.pidZ.pid.outputLimit = fmaxf(zVelMax, 0.5f)  * velMaxOverhead;

  float cosyaw = cosf(state->attitude.yaw * (float)M_PI / 180.0f);
  float sinyaw = sinf(state->attitude.yaw * (float)M_PI / 180.0f);
  float bodyvx = setpoint->velocity.x;
  float bodyvy = setpoint->velocity.y;

  // X, Y
  if (setpoint->mode.x == modeAbs) {
    setpoint->velocity.x = runPid(state->position.x, &this.pidX, setpoint->position.x, DT);
  } else if (setpoint->velocity_body) {
    setpoint->velocity.x = bodyvx * cosyaw - bodyvy * sinyaw;
  }
  if (setpoint->mode.y == modeAbs) {
    setpoint->velocity.y = runPid(state->position.y, &this.pidY, setpoint->position.y, DT);
  } else if (setpoint->velocity_body) {
    setpoint->velocity.y = bodyvy * cosyaw + bodyvx * sinyaw;
  }
  if (setpoint->mode.z == modeAbs) {
    setpoint->velocity.z = runPid(state->position.z, &this.pidZ, setpoint->position.z, DT);
  }

  setpointvx = setpoint->velocity.x;
  setpointvy = setpoint->velocity.y;

  velocityController(thrust, attitude, setpoint, state);
}

void velocityController(float* thrust, attitude_t *attitude, setpoint_t *setpoint,
                                                             const state_t *state)
{
  this.pidVX.pid.outputLimit = rpLimit * rpLimitOverhead;
  this.pidVY.pid.outputLimit = rpLimit * rpLimitOverhead;
  // Set the output limit to the maximum thrust range
  this.pidVZ.pid.outputLimit = (UINT16_MAX / 2 / thrustScale);
  //this.pidVZ.pid.outputLimit = (this.thrustBase - this.thrustMin) / thrustScale;

  // Roll and Pitch
  float rollRaw  = runPid(state->velocity.x, &this.pidVX, setpoint->velocity.x, DT) + kFFx*setpoint->velocity.x;
  float pitchRaw = runPid(state->velocity.y, &this.pidVY, setpoint->velocity.y, DT) + kFFy*setpoint->velocity.y;

  float yawRad = state->attitude.yaw * (float)M_PI / 180;
  attitude->pitch = -(rollRaw  * cosf(yawRad)) - (pitchRaw * sinf(yawRad));
  attitude->roll  = -(pitchRaw * cosf(yawRad)) + (rollRaw  * sinf(yawRad));

  attitude->roll  = constrain(attitude->roll,  -rpLimit, rpLimit);
  attitude->pitch = constrain(attitude->pitch, -rpLimit, rpLimit);

  // Thrust
  float thrustRaw = runPid(state->velocity.z, &this.pidVZ, setpoint->velocity.z, DT);
  // Scale the thrust and add feed forward term
  *thrust = thrustRaw*thrustScale + this.thrustBase;
  // Check for minimum thrust
  if (*thrust < this.thrustMin) {
    *thrust = this.thrustMin;
  }
}

void velocityControllerInBody(float* thrust, attitude_t *attitude, setpoint_t *setpoint,
                                                             const state_t *state)
{
  this.pidVX.pid.outputLimit = pLimit * rpLimitOverhead;
  this.pidVY.pid.outputLimit = rLimit * rpLimitOverhead;
  // Set the output limit to the maximum thrust range
  this.pidVZ.pid.outputLimit = (UINT16_MAX / 2 / thrustScale);
  //this.pidVZ.pid.outputLimit = (this.thrustBase - this.thrustMin) / thrustScale;

  float cosyaw = cosf(state->attitude.yaw * (float)M_PI / 180.0f);
  float sinyaw = sinf(state->attitude.yaw * (float)M_PI / 180.0f);
  float state_body_vx = state->velocity.x * cosyaw + state->velocity.y * sinyaw;
  float state_body_vy = -state->velocity.x * sinyaw + state->velocity.y * cosyaw;

  // Roll and Pitch
  attitude->pitch = -runPid(state_body_vx, &this.pidVX, setpoint->velocity.x, DT) - kFFx*setpoint->velocity.x;
  attitude->roll = -runPid(state_body_vy, &this.pidVY, setpoint->velocity.y, DT) - kFFy*setpoint->velocity.y;



  attitude->roll  = constrain(attitude->roll,  -rLimit, rLimit);
  attitude->pitch = constrain(attitude->pitch, -pLimit, pLimit);

  // Thrust
  float thrustRaw = runPid(state->velocity.z, &this.pidVZ, setpoint->velocity.z, DT);
  // Scale the thrust and add feed forward term
  *thrust = thrustRaw*thrustScale + this.thrustBase;
  // Check for minimum thrust
  if (*thrust < this.thrustMin) {
    *thrust = this.thrustMin;
  }
    // saturate
  *thrust = constrain(*thrust, 0, UINT16_MAX);
}

void positionController(float* thrust, attitude_t *attitude, setpoint_t *setpoint,
                                                             const state_t *state)
{
  setpointx = setpoint->position.x;
  setpointy = setpoint->position.y;
  
  if (POSITION_CONTROL_IN_BODY){
    if (singleLoop) positionControllerInBodySingleLoop(thrust, attitude, setpoint, state);
    else positionControllerInBody(thrust, attitude, setpoint, state);
    
  }
  else positionControllerInGlobal(thrust, attitude, setpoint, state);
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

// LOG_ADD(LOG_FLOAT, targetVX, &this.pidVX.pid.desired)
// LOG_ADD(LOG_FLOAT, targetVY, &this.pidVY.pid.desired)
LOG_ADD(LOG_FLOAT, targetVZ, &this.pidVZ.pid.desired)

// LOG_ADD(LOG_FLOAT, targetX, &this.pidX.pid.desired)
// LOG_ADD(LOG_FLOAT, targetY, &this.pidY.pid.desired)
LOG_ADD(LOG_FLOAT, targetZ, &this.pidZ.pid.desired)

LOG_ADD(LOG_FLOAT, targetVX, &setpointvx)
LOG_ADD(LOG_FLOAT, targetVY, &setpointvy)

LOG_ADD(LOG_FLOAT, targetX, &setpointx)
LOG_ADD(LOG_FLOAT, targetY, &setpointy)

LOG_ADD(LOG_FLOAT, Xp, &this.pidX.pid.outP)
LOG_ADD(LOG_FLOAT, Xi, &this.pidX.pid.outI)
LOG_ADD(LOG_FLOAT, Xd, &this.pidX.pid.outD)

LOG_ADD(LOG_FLOAT, Yp, &this.pidY.pid.outP)
LOG_ADD(LOG_FLOAT, Yi, &this.pidY.pid.outI)
LOG_ADD(LOG_FLOAT, Yd, &this.pidY.pid.outD)

LOG_ADD(LOG_FLOAT, Zp, &this.pidZ.pid.outP)
LOG_ADD(LOG_FLOAT, Zi, &this.pidZ.pid.outI)
LOG_ADD(LOG_FLOAT, Zd, &this.pidZ.pid.outD)

LOG_ADD(LOG_FLOAT, VXp, &this.pidVX.pid.outP)
LOG_ADD(LOG_FLOAT, VXi, &this.pidVX.pid.outI)
LOG_ADD(LOG_FLOAT, VXd, &this.pidVX.pid.outD)

LOG_ADD(LOG_FLOAT, VZp, &this.pidVZ.pid.outP)
LOG_ADD(LOG_FLOAT, VZi, &this.pidVZ.pid.outI)
LOG_ADD(LOG_FLOAT, VZd, &this.pidVZ.pid.outD)

// LOG_ADD(LOG_FLOAT, VXp, &bank_pitch)
// LOG_ADD(LOG_FLOAT, VZp, &bank_roll)

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

PARAM_ADD(PARAM_FLOAT, vxKFF, &kFFx)
PARAM_ADD(PARAM_FLOAT, vyKFF, &kFFy)

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

PARAM_ADD(PARAM_FLOAT, rpLimit,  &rpLimit)
PARAM_ADD(PARAM_FLOAT, rLimit,  &rLimit)
PARAM_ADD(PARAM_FLOAT, pLimit,  &pLimit)
PARAM_ADD(PARAM_FLOAT, xyVelMax, &xyVelMax)
PARAM_ADD(PARAM_FLOAT, zVelMax,  &zVelMax)
PARAM_ADD(PARAM_FLOAT, xBodyVelMax, &xBodyVelMax)
PARAM_ADD(PARAM_FLOAT, yBodyVelMax, &yBodyVelMax)

PARAM_ADD(PARAM_INT8, singleLoop, &singleLoop)


PARAM_GROUP_STOP(posCtlPid)

PARAM_GROUP_START(posVelFilt)

PARAM_ADD(PARAM_INT8, posFiltEn, &posFiltEnable)
PARAM_ADD(PARAM_FLOAT, posFiltCut, &posFiltCutoff)
PARAM_ADD(PARAM_INT8, velFiltEn, &velFiltEnable)
PARAM_ADD(PARAM_FLOAT, velFiltCut, &velFiltCutoff)
PARAM_ADD(PARAM_INT8, posZFiltEn, &posZFiltEnable)
PARAM_ADD(PARAM_FLOAT, posZFiltCut, &posZFiltCutoff)
PARAM_ADD(PARAM_INT8, velZFiltEn, &velZFiltEnable)
PARAM_ADD(PARAM_FLOAT, velZFiltCut, &velZFiltCutoff)

PARAM_GROUP_STOP(posVelFilt)
