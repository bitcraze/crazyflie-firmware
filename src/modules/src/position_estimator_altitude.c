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

#include "log.h"
#include "param.h"
#include "position_estimator.h"

struct state_s {
  float estimatedZ; // The current Z estimate, has same offset as asl
  float estAlpha;
  float velocityFactor;
};

static struct state_s state = {
  .estimatedZ = 0.0,
  .estAlpha = 0.99,
  .velocityFactor = 1.0,
};

static void positionEstimateInternal(estimate_t* estimate, float asl, float velocityZ, float dt, struct state_s* state);

void positionEstimate(estimate_t* estimate, float asl, float velocityZ, float dt) {
  positionEstimateInternal(estimate, asl, velocityZ, dt, &state);
}

static void positionEstimateInternal(estimate_t* estimate, float asl, float velocityZ, float dt, struct state_s* state) {
  state->estimatedZ = state->estAlpha * state->estimatedZ +
                     (1.0 - state->estAlpha) * asl +
                     state->velocityFactor * velocityZ * dt;

  estimate->position.x = 0.0;
  estimate->position.y = 0.0;
  estimate->position.z = state->estimatedZ;
}


LOG_GROUP_START(posEstimatorAlt)
LOG_ADD(LOG_FLOAT, estimatedZ, &state.estimatedZ)
LOG_GROUP_STOP(posEstimatorAlt)

PARAM_GROUP_START(posEst)
PARAM_ADD(PARAM_FLOAT, estAlpha, &state.estAlpha)
PARAM_ADD(PARAM_FLOAT, velFactor, &state.velocityFactor)
PARAM_GROUP_STOP(posEstimatorAlt)
