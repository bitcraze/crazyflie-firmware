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
 * collision_avoidance.h - Collision avoidance for multiple Crazyflies.
 *
 * Original author: James A. Preiss, University of Southern California, 2020.
 */
#ifndef __COLLISION_AVOIDANCE_H__
#define __COLLISION_AVOIDANCE_H__


#include "math3d.h"
#include "stabilizer_types.h"


typedef struct collision_avoidance_params_s
{
  // The Crazyflie's boundary for collision checking is a tall ellipsoid.
  // This accounts for the downwash effect: Due to the fast-moving stream of
  // air produced by the rotors, the safe distance to pass underneath another
  // rotorcraft is much further than the safe distance to pass to the side.
  struct vec ellipsoidRadii;

  // The minimal and maximal corners of the bounding box in which we fly.
  struct vec bboxMin;
  struct vec bboxMax;

  // The BVC algorithm will ensure that we stay within our Voronoi cell
  // for this duration. Higher values lead to more conservative behavior.
  float horizonSecs;

  // Maximum speed (meters/second) we will command. This is currently enforced
  // loosely with an infinity-norm box, so be conservative.
  float maxSpeed;

  // If peer localization measurements are older than this, ignore them.
  // If negative, never ignore.
  int maxPeerLocAgeMillis;

  // Tolerance for projecting position vectors into our Voronoi cell.
  // Units are roughly Euclidean distance in meters, but not exactly.
  float voronoiProjectionTolerance;

  // Max number of iterations for projecting into our Voronoi cell.
  // Using Dykstra's algorithm.
  int voronoiProjectionMaxIters;

} collision_avoidance_params_t;


typedef struct collision_avoidance_state_s
{
  // In case our Voronoi cell becomes empty, due to e.g. a disturbance, we will
  // attempt to stop and stay in place. To avoid self-compounding drift , we
  // must keep track of a fixed position value rather than using the current
  // state as a setpoint.
  struct vec lastFeasibleSetPosition;

} collision_avoidance_state_t;


// Mutates the setpoint.
//
// To facilitate compiling and testing on a PC, we take neighbor positions via
// array instead of having the implementation call peer_localization functions
// directly. On the other hand, we wish to use the minimum possible amount of
// memory. Therefore, we allow the input and output arrays to overlap. If
// otherPositions == workspace, this function will still work correctly, but it
// will overwrite the contents of otherPositions.
//
void collisionAvoidanceUpdateSetpointCore(
  collision_avoidance_params_t const *params,
  collision_avoidance_state_t *collisionState,
  int nOthers,
  float const *otherPositions,
  float *workspace,
  setpoint_t *setpoint, sensorData_t const *sensorData, state_t const *state);

// For ease of use, in a firmware build we include this wrapper that handles
// the interaction with peer_localization and gets all parameter values via the
// param system.
#ifdef CRAZYFLIE_FW

void collisionAvoidanceUpdateSetpoint(
  setpoint_t *setpoint, sensorData_t const *sensorData, state_t const *state);
#endif

#endif //__COLLISION_AVOIDANCE_H__
