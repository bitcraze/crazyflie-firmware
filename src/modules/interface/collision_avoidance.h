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


typedef struct collision_avoidance_params_s
{
  // The point in 3D we are trying to reach.
  struct vec goal;

  // The radii of our axis-aligned collision ellipsoid.
  struct vec ellipsoidRadii;

  // The minimal and maximal corners of the bounding box of the entire space.
  struct vec bboxMin;
  struct vec bboxMax;

  // Commanded velocities will stay within our Voronoi cell for this duration.
  float horizonSecs;

  // Maximum speed (meters/second) we will command.
  float maxSpeed;

} collision_avoidance_params_t;

// Args:
//   params: Parameters struct, above.
//   position: Our position.
//   otherPositions: Array of other agents' positions.
//   nOthers: Number of elements in otherPositions.
//   work: Scratch space of length at least 7 * (nOthers + 6) floats.
//     Needed to avoid dynamic memory allocation within.
//
// Returns:
//   Commanded velocity vector.
struct vec collisionAvoidanceVoronoi(
  collision_avoidance_params_t const *params,
  struct vec position,
  struct vec const *otherPositions,
  int nOthers,
  float *work);

#endif //__COLLISION_AVOIDANCE_H__
