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
#include "autoconf.h"
#include "outlierFilterTdoa.h"
#include "stabilizer_types.h"

#ifdef CONFIG_DEBUG_TDOA_QUALITY
#include "log.h"
#endif


static bool isDistanceDiffSmallerThanDistanceBetweenAnchors(const tdoaMeasurement_t* tdoa);

#ifdef CONFIG_DEBUG_TDOA_QUALITY
// Static mirror of the most recently validated filter state, for logging only.
// The actual state lives in the active estimator's OutlierFilterTdoaState_t
// instance; this captures a snapshot in the validate function so it can be
// surfaced as cheap LOG vars without changing the filter API or behavior.
static uint8_t logFilterOpen = 1;
static float logIntegrator = 0.0f;
#endif

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


void outlierFilterTdoaReset(OutlierFilterTdoaState_t* this) {
  this->integrator = 0.0f;
  this->isFilterOpen = true;
  this->latestUpdateMs = 0;
}

bool outlierFilterTdoaValidateIntegrator(OutlierFilterTdoaState_t* this, const tdoaMeasurement_t* tdoa, const float error, const uint32_t nowMs) {
  // The accepted error when the filter is closed
  const float acceptedDistance = tdoa->stdDev * 2.5f;

  // The level used to determine if a sample is added or removed from the integrator
  const float integratorTriggerDistance = tdoa->stdDev * 2.0f;


  bool sampleIsGood = false;

  // Discard samples that are physically impossible, most likely measurement error
  if (isDistanceDiffSmallerThanDistanceBetweenAnchors(tdoa)) {
    uint32_t dtMs = nowMs - this->latestUpdateMs;
    // Limit dt to minimize the impact on the integrator if we have not received samples for a long time (or at start up)
    dtMs = fminf(dtMs, INTEGRATOR_SIZE / 10.0f);

    if (fabsf(error) < integratorTriggerDistance) {
      this->integrator += dtMs;
      this->integrator = fminf(this->integrator, INTEGRATOR_SIZE);
    } else {
      this->integrator -= dtMs;
      this->integrator = fmaxf(this->integrator, 0.0f);
    }

    if (this->isFilterOpen) {
      // The filter is open, let all samples through
      sampleIsGood = true;

      if (this->integrator > INTEGRATOR_RESUME_ACTION_LEVEL) {
        // We have recovered and converged, close the filter again
        this->isFilterOpen = false;
      }
    } else {
      // The filter is closed, let samples with a small error through
      sampleIsGood = (fabsf(error) < acceptedDistance);

      if (this->integrator < INTEGRATOR_FORCE_OPEN_LEVEL) {
        // We have got lots of outliers lately, the kalman filter may have diverged. Open up to try to recover
        this->isFilterOpen = true;
      }
    }

    this->latestUpdateMs = nowMs;
  }

#ifdef CONFIG_DEBUG_TDOA_QUALITY
  // Snapshot current state for logging (D6 divergence diagnostics).
  logFilterOpen = this->isFilterOpen ? 1 : 0;
  logIntegrator = this->integrator;
#endif

  return sampleIsGood;
}

static float sq(float a) {return a * a;}

static float distanceSq(const point_t* a, const point_t* b) {
  return sq(a->x - b->x) + sq(a->y - b->y) + sq(a->z - b->z);
}

static bool isDistanceDiffSmallerThanDistanceBetweenAnchors(const tdoaMeasurement_t* tdoa) {
  float anchorDistanceSq = distanceSq(&tdoa->anchorPositions[0], &tdoa->anchorPositions[1]);
  float distanceDiffSq = sq(tdoa->distanceDiff);
  return (distanceDiffSq < anchorDistanceSq);
}

#ifdef CONFIG_DEBUG_TDOA_QUALITY
/**
 * TDoA outlier-filter integrator state, a live snapshot for divergence
 * diagnostics (D6). The integrator tracks the recent ratio of good samples
 * (0..INTEGRATOR_SIZE, ~0..300); the filter opens to let all samples through
 * when the estimate may have diverged and closes once it has converged again.
 * Enabled by CONFIG_DEBUG_TDOA_QUALITY.
 */
LOG_GROUP_START(tdoaFilter)
  /**
   * @brief Outlier filter open (1) / closed (0) state. Open means all samples
   *        pass; a sustained open state indicates possible divergence.
   */
  LOG_ADD(LOG_UINT8, filterOpen, &logFilterOpen)
  /**
   * @brief Outlier filter integrator level (~0..300). Low values trend toward
   *        opening the filter, high values toward closing it.
   */
  LOG_ADD(LOG_FLOAT, integ, &logIntegrator)
LOG_GROUP_STOP(tdoaFilter)
#endif
