/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2018 Bitcraze AB
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
 * outlierFilter.c: Outlier rejection filter for the LPS system
 */

#include <math.h>
#include "outlierFilter.h"

static bool isDistanceDiffSmallerThanDistanceBetweenAnchors(tdoaMeasurement_t* tdoa);
static double distanceSq(const point_t* a, const point_t* b);
static double sq(double a) {return a * a;}


bool outlierFilterValidateTdoa(tdoaMeasurement_t* tdoa, point_t* estimatedPosition) {
  return isDistanceDiffSmallerThanDistanceBetweenAnchors(tdoa);
}

void outlierFilterReset() {
  // Nothing here
}

static bool isDistanceDiffSmallerThanDistanceBetweenAnchors(tdoaMeasurement_t* tdoa) {  
  double anchorDistanceSq = distanceSq(&tdoa->anchorPosition[0], &tdoa->anchorPosition[1]);
  double distanceDiffSq = sq(tdoa->distanceDiff);
  return (distanceDiffSq < anchorDistanceSq);
}

static double distanceSq(const point_t* a, const point_t* b) {
  return sq(a->x - b->x) + sq(a->y - b->y) + sq(a->z - b->z);
}
