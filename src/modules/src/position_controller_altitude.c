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
 * position_estimator_altitude.c: Altitude-only position estimator
 */

#include <math.h>

#include "commander.h"
#include "log.h"
#include "param.h"
#include "pid.h"
#include "num.h"
#include "position_controller.h"

struct state_s {
  PidObject pid;

  float targetZ;    // Target altitude

  float pidInitKp;
  float pidInitKi;
  float pidInitKd;

  uint16_t thrustBase; // approximate throttle needed when in perfect hover. More weight/older battery can use a higher value
  float thrust;      // The thrust from regulator, expressed as an offset to thrustBase
};

static struct state_s state = {
  .targetZ = -1,

  .pidInitKp = 30000.0,
  .pidInitKi = 0.0,
  .pidInitKd = 10000.0,

  .thrustBase = 32000,
};

static void positionPidControllerUpdateInternal(uint16_t* actuatorThrust, const estimate_t* estimate, float dt, struct state_s* state);
static void positionControllerSetZTargetInternal(const setpointZ_t* setpoint, float dt, struct state_s* state);

void positionControllerUpdate(uint16_t* actuatorThrust, const estimate_t* estimate, float dt) {
  positionPidControllerUpdateInternal(actuatorThrust, estimate, dt, &state);
}

void positionControllerSetZTarget(const setpointZ_t* setpoint, float dt) {
  positionControllerSetZTargetInternal(setpoint, dt, &state);
}

static void positionControllerSetZTargetInternal(const setpointZ_t* setpoint, float dt, struct state_s* state) {
  state->targetZ = setpoint->z;
  if (setpoint->isUpdate) {
    pidSetDesired(&state->pid, state->targetZ);
  } else {
    pidInit(&state->pid, state->targetZ, state->pidInitKp, state->pidInitKi, state->pidInitKd, dt);
  }
}

static void positionPidControllerUpdateInternal(uint16_t* actuatorThrust, const estimate_t* estimate, float dt, struct state_s* state) {
  state->thrust = pidUpdate(&state->pid, estimate->position.z, true);
  *actuatorThrust = limitUint16(state->thrustBase + state->thrust);
}


LOG_GROUP_START(posCtlAlt)
LOG_ADD(LOG_FLOAT, p, &state.pid.outP)
LOG_ADD(LOG_FLOAT, i, &state.pid.outI)
LOG_ADD(LOG_FLOAT, d, &state.pid.outD)
LOG_ADD(LOG_FLOAT, thrust, &state.thrust)

LOG_ADD(LOG_FLOAT, targetZ, &state.targetZ)
LOG_GROUP_STOP(posCtlAlt)

PARAM_GROUP_START(posCtlAlt)
PARAM_ADD(PARAM_FLOAT, pidInitKp, &state.pidInitKp)
PARAM_ADD(PARAM_FLOAT, pidInitKi, &state.pidInitKi)
PARAM_ADD(PARAM_FLOAT, pidInitKd, &state.pidInitKd)
PARAM_ADD(PARAM_UINT16, thrustBase, &state.thrustBase)
PARAM_GROUP_STOP(posCtlAlt)
