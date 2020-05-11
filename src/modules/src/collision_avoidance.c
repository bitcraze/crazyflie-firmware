/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2020 Bitcraze AB
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
 * collision_avoidance.c - Collision avoidance for multiple Crazyflies.
 *
 * Original author: James A. Preiss, University of Southern California, 2020.
 */

#include <float.h>
#include <string.h>  // for memset
#include <stdio.h>  // for debugging. TODO: remove

#include "collision_avoidance.h"
#include "peer_localization.h"


// See header for API comments.


// We must convert from stabilizer types to cmath3d types to get the library of
// math operations.
static struct vec vec2svec(struct vec3_s v)
{
  return mkvec(v.x, v.y, v.z);
}
static struct vec3_s svec2vec(struct vec v)
{
  struct vec3_s vv = { .x = v.x, .y = v.y, .z = v.z, };
  return vv;
}

void collisionAvoidanceUpdateSetpointCore(
  collision_avoidance_params_t const *params,
  collision_avoidance_state_t *collisionState,
  int nOthers,
  float const *otherPositions,
  float *workspace,
  setpoint_t *setpoint, sensorData_t const *sensorData, state_t const *state)
{
  //
  // Part 1: Construct the polytope inequalities in A, b.
  //

  int const nRows = nOthers + 6;
  float *A = workspace;
  float *B = workspace + 3 * nRows;
  float *projectionWorkspace = workspace + 4 * nRows;

  // Compute the cell in a stretched coordinate system for downwash awareness.
  // See header for details.
  struct vec const radiiInv = veltrecip(params->ellipsoidRadii);
  struct vec const ourPos = vec2svec(state->position);

  // We could do some optimizations here to reduce the number of rows, like
  // leave out very far away neighbors. It would help the average case
  // complexity, but not the worst case - so we skip it for now.
  for (int i = 0; i < nOthers; ++i) {
    struct vec peerPos = vloadf(otherPositions + 3 * i);
    struct vec const toPeerStretched = veltmul(vsub(peerPos, ourPos), radiiInv);
    float const dist = vmag(toPeerStretched);
    struct vec const a = vdiv(veltmul(toPeerStretched, radiiInv), dist);
    float const b = dist / 2.0f - 1.0f;
    float scale = 1.0f / vmag(a);
    vstoref(vscl(scale, a), A + 3 * i);
    B[i] = scale * b;
  }

  // Add the bounding box polytope faces. We also use the box faces to enforce
  // max speed in the infinity-norm.
  float const maxDist = params->horizonSecs * params->maxSpeed;

  memset(A + 3 * nOthers, 0, 18 * sizeof(float));

  for (int dim = 0; dim < 3; ++dim) {
    float boxMax = vindex(params->bboxMax, dim) - vindex(ourPos, dim);
    A[3 * (nOthers + dim) + dim] = 1.0f;
    B[nOthers + dim] = fminf(maxDist, boxMax);

    float boxMin = vindex(params->bboxMin, dim) - vindex(ourPos, dim);
    A[3 * (nOthers + dim + 3) + dim] = -1.0f;
    B[nOthers + dim + 3] = fmaxf(-maxDist, -boxMin);
  }

  //
  // Part 2: Use the constructed polytope to modify the setpoint.
  //

  struct vec setPos = vec2svec(setpoint->position);
  struct vec setVel = vec2svec(setpoint->velocity);
  struct vec setPosRelative = vsub(setPos, ourPos);

  // Since the polytope coordinates are centered on our current position,
  // projecting the zero vector in velocity control mode is used to determine
  // if we are inside our own buffered cell or not.
  struct vec toProject = (setpoint->mode.x == modeVelocity) ? vzero() : setPosRelative;
  struct vec setPosRelativeProjected = vprojectpolytope(
    toProject,
    A, B, projectionWorkspace, nRows,
    params->voronoiProjectionTolerance,
    params->voronoiProjectionMaxIters
  );

  float const inPolytopeTolerance = 10.0f * params->voronoiProjectionTolerance;

  if (!vinpolytope(setPosRelativeProjected, A, B, nRows, inPolytopeTolerance)) {
    // If the projection algorithm failed to converge, then either
    //   1) The problem is infeasible, or
    //   2) The problem is somehow badly conditioned.
    // Case 2) is still effectively infeasible -- we're in a real-time system
    // with a fixed computation time budget, and we've already spent it.
    //
    // There's not a lot we can do if our buffered cell is empty. It means
    // someone failed to stay within their cell in the past. We choose to stay
    // a fixed position for as long as the cell is empty.
    setVel = vzero();
    if (!visnan(collisionState->lastFeasibleSetPosition)) {
      setPos = collisionState->lastFeasibleSetPosition;
    }
    else {
      // This case should never happen as long as collision avoidance is
      // initialized in a collision-free state, but we do something reasonable
      // just in case.
      setPos = ourPos;
      collisionState->lastFeasibleSetPosition = ourPos;
    }
  }
  else if (setpoint->mode.x == modeVelocity) {
    // Interpret the setpoint to mean "fly with this velocity".

    if (!vinpolytope(setPosRelativeProjected, A, B, nRows, inPolytopeTolerance)) {
      // If our current location is not within our cell, then we should forget
      // about the original goal velocity and try to move towards our cell.
      setVel = vclampnorm(setPosRelativeProjected, params->maxSpeed);
    }
    else {
      // Otherwise, if goal velocity would take us outside our cell during the
      // planning horizon, clamp it.
      float const scale = rayintersectpolytope(vzero(), setVel, A, B, nRows, NULL);
      if (scale < 1.0f)  {
        setVel = vscl(scale, setVel);
      }
    }
    collisionState->lastFeasibleSetPosition = ourPos;
  }
  else if (setpoint->mode.x == modeAbs && veq(setVel, vzero())) {
    // Position, but no velocity. Interpret as waypoint, not trajectory
    // tracking. "Go to this position and stop".
    setPos = vadd(ourPos, setPosRelativeProjected);
    collisionState->lastFeasibleSetPosition = setPos;
  }
  else if (setpoint->mode.x == modeAbs) {
    // Position with nonzero velocity. Interpret as trajectory tracking.
    if (vneq(setPosRelative, setPosRelativeProjected)) {
      // Set position is not within our cell. The situation has likely diverged
      // from the original plan, and the nonzero velocity is probably pointing
      // further away from our cell. Therefore, degrade to the v == 0 behavior.
      setPos = vadd(ourPos, setPosRelativeProjected);
      setVel = vzero();
    }
    else {
      // Set position is within our cell. In case velocity would take us
      // outside the cell within the planning horizon, scale it appropriately.
      // See github issue #567 for more detailed discussion of this behavior.
      float const scale = rayintersectpolytope(setPosRelative, setVel, A, B, nRows, NULL);
      if (scale < 1.0f)  {
        setVel = vscl(scale, setVel);
      }
      // leave setPos unchanged.
    }
    collisionState->lastFeasibleSetPosition = setPos;
  }
  else {
    // Unsupported control mode, do nothing.
  }

  setpoint->position = svec2vec(setPos);
  setpoint->velocity = svec2vec(setVel);
}


//
// Everything below this comment will only be compiled in a firware build made
// with the standard Makefile. Everything depending on FreeRTOS, ARM, params,
// etc. must go here.
//
#ifdef CRAZYFLIE_FW

#include "FreeRTOS.h"
#include "task.h"

#include "param.h"


static uint8_t collisionAvoidanceEnable = 0;

static collision_avoidance_params_t params = {
  .ellipsoidRadii = { .x = 0.125, .y = 0.125, .z = 0.75 },
  .bboxMin = { .x = -FLT_MAX, .y = -FLT_MAX, .z = -FLT_MAX },
  .bboxMax = { .x = FLT_MAX, .y = FLT_MAX, .z = FLT_MAX },
  .horizonSecs = 1.0f,
  .maxSpeed = 5.0f,
  .maxPeerLocAgeMillis = 5000,
  .voronoiProjectionTolerance = 1e-4,
  .voronoiProjectionMaxIters = 100,
};

static collision_avoidance_state_t collisionState = {
  .lastFeasibleSetPosition = { .x = NAN, .y = NAN, .z = NAN },
};

// Each face of the Voronoi cell is defined by a linear inequality a^T x <= b.
// The algorithm for projecting a point into a convex polytope requires 3 more
// floats of working space per face. The six extra faces come from the overall
// flight area bounding box.
#define MAX_CELL_ROWS (PEER_LOCALIZATION_MAX_NEIGHBORS + 6)
static float workspace[7 * MAX_CELL_ROWS];


void collisionAvoidanceUpdateSetpoint(
  setpoint_t *setpoint, sensorData_t const *sensorData, state_t const *state)
{
  if (!collisionAvoidanceEnable) {
    return;
  }

  TickType_t const time = xTaskGetTickCount();
  bool doAgeFilter = params.maxPeerLocAgeMillis >= 0;

  // Counts the actual number of neighbors after we filter stale measurements.
  int nOthers = 0;

  for (int i = 0; i < PEER_LOCALIZATION_MAX_NEIGHBORS; ++i) {

    peerLocalizationOtherPosition_t const *otherPos = peerLocalizationGetPositionByIdx(i);
    int age = time - otherPos->pos.timestamp;

    if (otherPos == NULL ||
        otherPos->id == 0 ||
        (doAgeFilter && (age > params.maxPeerLocAgeMillis))) {
      continue;
    }

    workspace[3 * nOthers + 0] = otherPos->pos.x;
    workspace[3 * nOthers + 1] = otherPos->pos.y;
    workspace[3 * nOthers + 2] = otherPos->pos.z;
    ++nOthers;
  }

  collisionAvoidanceUpdateSetpointCore(&params, &collisionState, nOthers, workspace, workspace, setpoint, sensorData, state);
}


PARAM_GROUP_START(collisionAvoidance)

PARAM_ADD(PARAM_UINT8, enable, &collisionAvoidanceEnable)

PARAM_ADD(PARAM_FLOAT, ellipsoidX, &params.ellipsoidRadii.x)
PARAM_ADD(PARAM_FLOAT, ellipsoidY, &params.ellipsoidRadii.y)
PARAM_ADD(PARAM_FLOAT, ellipsoidZ, &params.ellipsoidRadii.z)

PARAM_ADD(PARAM_FLOAT, bboxMinX, &params.bboxMin.x)
PARAM_ADD(PARAM_FLOAT, bboxMinY, &params.bboxMin.y)
PARAM_ADD(PARAM_FLOAT, bboxMinZ, &params.bboxMin.z)

PARAM_ADD(PARAM_FLOAT, bboxMaxX, &params.bboxMax.x)
PARAM_ADD(PARAM_FLOAT, bboxMaxY, &params.bboxMax.y)
PARAM_ADD(PARAM_FLOAT, bboxMaxZ, &params.bboxMax.z)

PARAM_ADD(PARAM_FLOAT, horizon, &params.horizonSecs)
PARAM_ADD(PARAM_FLOAT, maxSpeed, &params.maxSpeed)
PARAM_ADD(PARAM_INT32, maxPeerLocAge, &params.maxPeerLocAgeMillis)

// For now, leave out the parameters of the Voronoi projection algorithm.
// They should probably be tuned once with experiments and then hard-coded.

PARAM_GROUP_STOP(collisionAvoidance)

#endif  // CRAZYFLIE_FW
