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

#include <string.h>  // for memset
#include <stdio.h>  // for debugging. TODO: remove

#include "collision_avoidance.h"


// See header for API comments.


struct vec collisionAvoidanceVoronoi(
  collision_avoidance_params_t const *params,
  struct vec position,
  struct vec const *otherPositions,
  int nOthers,
  float *work)
{
  struct vec toGoal = vsub(params->goal, position);
  float const goalDist = vmag(toGoal);
  struct vec const goalDir = vdiv(toGoal, goalDist);

  // Compute a "sidestep" - in case a neighbor is directly blocking the path
  // to our goal, we turn a little to the right in the XY plane instead of
  // trying to move directly towards the goal.

  float worstInner = 0.0f;

  for (int i = 0; i < nOthers; i++) {
    struct vec const toNeighbor = vsub(otherPositions[i], position);
    float const neighborDist = vmag(toNeighbor);
    struct vec const neighborDir = vdiv(toNeighbor, neighborDist);

    float const inner = vdot(neighborDir, goalDir);
    if (inner > 0.0f && neighborDist <= goalDist) {
      worstInner = fmaxf(worstInner, inner);
    }
    // Otherwise, neighbor is not blocking our path to goal.
  }

  if (worstInner != 0.0f) {
    struct vec const sidestepDir = vnormalize(vcross(goalDir, mkvec(0.0f, 0.0f, 1.0f)));
    // The size of sidestep to take if we are almost touching the neighbor.
    float const deadlockSidestep = 2.0f * params->ellipsoidRadii.x;
    // Scaling by inner^4 makes sidestep strength fall off quickly if neighbor
    // isn't exactly in between us and the goal.
    float const sidestepAmount = fsqr(fsqr(worstInner)) * fmaxf(goalDist, deadlockSidestep);
    toGoal = vadd(toGoal, vscl(sidestepAmount, sidestepDir));
  }

  // Construct the buffered Voronoi cell polytope inequalites using the
  // provided scratch space. Last 6 rows are for bounding box.
  float *B = work;
  float *A = work + (nOthers + 6);
  float *work2 = work + 4 * (nOthers + 6);

  struct vec radiiInv = veltrecip(params->ellipsoidRadii);

  // Add the neighbor polytope faces. The equal-distance separating planes are
  // computed in the stretched coordinate system defined by the collision
  // ellipsoid radii.
  for (int i = 0; i < nOthers; ++i) {
    struct vec const toNeighborStretched = veltmul(
      vsub(otherPositions[i], position),
      radiiInv);
    float const dist = vmag(toNeighborStretched);
    struct vec const a = vdiv(veltmul(toNeighborStretched, radiiInv), dist);
    float const b = dist / 2.0f - 1.0f;
    float scale = 1.0f / vmag(a);
    vstoref(vscl(scale, a), A + 3 * i);
    B[i] = scale * b;
  }

  // Add the bounding box polytope faces.
  memset(A + 3 * nOthers, 0, 18 * sizeof(float));
  for (int dim = 0; dim < 3; ++dim) {
    A[3 * (nOthers + dim) + dim] = 1.0f;
    B[nOthers + dim] = vindex(params->bboxMax, dim) - vindex(position, dim);
    A[3 * (nOthers + dim + 3) + dim] = -1.0f;
    B[nOthers + dim + 3] = -(vindex(params->bboxMin, dim) - vindex(position, dim));
  }

  struct vec move = vprojectpolytope(toGoal, A, B, work2, nOthers + 6, 1e-6);
  struct vec clamped = vclampnorm(move, params->maxSpeed);
  return clamped;
}
