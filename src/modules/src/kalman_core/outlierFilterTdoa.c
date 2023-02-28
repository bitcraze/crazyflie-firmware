/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2023 Bitcraze AB
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
#include "outlierFilterTdoa.h"
#include "stabilizer_types.h"
#include "log.h"
#include "debug.h"


static bool isDistanceDiffSmallerThanDistanceBetweenAnchors(const tdoaMeasurement_t* tdoa);


// The step TDoA outlier filter is deprecated and will be removed

#define BUCKET_ACCEPTANCE_LEVEL 3
#define MAX_BUCKET_FILL 10
#define FILTER_CLOSE_DELAY_COUNT 30

static float acceptanceLevel = 0.0;
static float errorDistance;
static int filterCloseDelayCounter = 0;
static int previousFilterIndex = 0;

typedef struct {
  float acceptanceLevel;
  int bucket;
} filterLevel_t;

#define FILTER_LEVELS 5
#define FILTER_NONE FILTER_LEVELS
filterLevel_t filterLevels[FILTER_LEVELS] = {
  {.acceptanceLevel = 0.4},
  {.acceptanceLevel = 0.8},
  {.acceptanceLevel = 1.2},
  {.acceptanceLevel = 1.6},
  {.acceptanceLevel = 2.0},
};


static float distanceSq(const point_t* a, const point_t* b);
static float sq(float a) {return a * a;}
static void addToBucket(filterLevel_t* filter);
static void removeFromBucket(filterLevel_t* filter);
static int updateBuckets(float errorDistance);

bool outlierFilterValidateTdoaSteps(const tdoaMeasurement_t* tdoa, const float error, const vector_t* jacobian, const point_t* estPos) {
  bool sampleIsGood = false;

  if (isDistanceDiffSmallerThanDistanceBetweenAnchors(tdoa)) {
    float errorBaseDistance = sqrtf(powf(jacobian->x, 2) + powf(jacobian->y, 2) + powf(jacobian->z, 2));
    errorDistance = fabsf(error / errorBaseDistance);

    int filterIndex = updateBuckets(errorDistance);

    if (filterIndex > previousFilterIndex) {
      filterCloseDelayCounter = FILTER_CLOSE_DELAY_COUNT;
    } else if (filterIndex < previousFilterIndex) {
      if (filterCloseDelayCounter > 0) {
        filterCloseDelayCounter--;
        filterIndex = previousFilterIndex;
      }
    }
    previousFilterIndex = filterIndex;

    if (filterIndex == FILTER_NONE) {
      // Lost tracking, open up to let the kalman filter converge
      acceptanceLevel = 100.0;
      sampleIsGood = true;
    } else {
      acceptanceLevel = filterLevels[filterIndex].acceptanceLevel;
      if (errorDistance < acceptanceLevel) {
        sampleIsGood = true;
      }
    }
  }

  return sampleIsGood;
}

// Simple TDoA outlier filter
bool outlierFilterValidateTdoaSimple(const tdoaMeasurement_t* tdoa) {
  return isDistanceDiffSmallerThanDistanceBetweenAnchors(tdoa);
}


// Integrator TDoA outlier filter -------------------------------------
// This filter uses an "integrator" to track the ratio of good/samples. Samples with an errors (measurement - predicted)
// larger than the acceptance level will be discarded. The acceptance level is based on the integrator, with a
// hysteresis to avoid rapid changes. When the filter is open, all samples are let through, usually at startup to let
// the kalman filter converge. When the filter is closed, only samples with an error < the acceptance level, this should
// be most of the time.
// The acceptance level is based on the standard deviation used for the tdoa samples, which should be based on the
// noise level in the system.


// The maximum size of the integrator. This size determines the time [in ms] needed to open/close the filter.
static const float INTEGRATOR_SIZE = 300.0f;

// The level when the filter open up to let all samples through
static const float INTEGRATOR_FORCE_OPEN_LEVEL = INTEGRATOR_SIZE * 0.1f;

// The level when the filter closes again
static const float INTEGRATOR_RESUME_ACTION_LEVEL = INTEGRATOR_SIZE * 0.9f;

static float integrator;
static uint32_t latestUpdateMs;
static bool isFilterOpen = true;


bool outlierFilterValidateTdoaIntegrator(const tdoaMeasurement_t* tdoa, const float error, const uint32_t nowMs) {
  // The accepted error when the filter is closed
  const float acceptedDistance = tdoa->stdDev * 2.5f;

  // The level used to determine if a sample is added or removed from the integrator
  const float integratorTriggerDistance = tdoa->stdDev * 2.0f;


  bool sampleIsGood = false;

  // Discard samples that are physically impossible, most likely measurement error
  if (isDistanceDiffSmallerThanDistanceBetweenAnchors(tdoa)) {
    uint32_t dtMs = nowMs - latestUpdateMs;
    // Limit dt to minimize the impact on the integrator if we have not received samples for a long time (or at start up)
    dtMs = fminf(dtMs, INTEGRATOR_SIZE / 10.0f);

    if (fabsf(error) < integratorTriggerDistance) {
      integrator += dtMs;
      integrator = fminf(integrator, INTEGRATOR_SIZE);
    } else {
      integrator -= dtMs;
      integrator = fmaxf(integrator, 0.0f);
    }

    if (isFilterOpen) {
      // The filter is open, let all samples through
      sampleIsGood = true;

      if (integrator > INTEGRATOR_RESUME_ACTION_LEVEL) {
        // We have recovered and converged, close the filter again
        isFilterOpen = false;
      }
    } else {
      // The filter is closed, let samples with a small error through
      sampleIsGood = (fabsf(error) < acceptedDistance);

      if (integrator < INTEGRATOR_FORCE_OPEN_LEVEL) {
        // We have got lots of outliers lately, the kalman filter may have diverged. Open up to try to recover
        isFilterOpen = true;
      }
    }

    latestUpdateMs = nowMs;
  }

  return sampleIsGood;
}


static bool isDistanceDiffSmallerThanDistanceBetweenAnchors(const tdoaMeasurement_t* tdoa) {
  float anchorDistanceSq = distanceSq(&tdoa->anchorPositions[0], &tdoa->anchorPositions[1]);
  float distanceDiffSq = sq(tdoa->distanceDiff);
  return (distanceDiffSq < anchorDistanceSq);
}

static float distanceSq(const point_t* a, const point_t* b) {
  return sq(a->x - b->x) + sq(a->y - b->y) + sq(a->z - b->z);
}


static void addToBucket(filterLevel_t* filter) {
  if (filter->bucket < MAX_BUCKET_FILL) {
    filter->bucket++;
  }
}

static void removeFromBucket(filterLevel_t* filter) {
  if (filter->bucket > 0) {
    filter->bucket--;
  }
}

static int updateBuckets(float errorDistance) {
  int filterIndex = FILTER_NONE;

  for (int i = FILTER_LEVELS - 1; i >= 0; i--) {
    filterLevel_t* filter = &filterLevels[i];

    if (errorDistance < filter->acceptanceLevel) {
      removeFromBucket(filter);
    } else {
      addToBucket(filter);
    }

    if (filter->bucket < BUCKET_ACCEPTANCE_LEVEL) {
      filterIndex = i;
    }
  }

  return filterIndex;
}

LOG_GROUP_START(outlierf)
  LOG_ADD(LOG_INT32, bucket0, &filterLevels[0].bucket)
  LOG_ADD(LOG_INT32, bucket1, &filterLevels[1].bucket)
  LOG_ADD(LOG_INT32, bucket2, &filterLevels[2].bucket)
  LOG_ADD(LOG_INT32, bucket3, &filterLevels[3].bucket)
  LOG_ADD(LOG_INT32, bucket4, &filterLevels[4].bucket)
  LOG_ADD(LOG_FLOAT, accLev, &acceptanceLevel)
  LOG_ADD(LOG_FLOAT, errD, &errorDistance)
LOG_GROUP_STOP(outlierf)
