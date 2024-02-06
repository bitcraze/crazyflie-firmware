/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2021 Bitcraze AB
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
 * estimator.h - State estimator interface
 */
#pragma once

#include "autoconf.h"
#include "stabilizer_types.h"

typedef enum {
  StateEstimatorTypeAutoSelect = 0,
  StateEstimatorTypeComplementary,
#ifdef CONFIG_ESTIMATOR_KALMAN_ENABLE
  StateEstimatorTypeKalman,
#endif
#ifdef CONFIG_ESTIMATOR_UKF_ENABLE
  StateEstimatorTypeUkf,
#endif
#ifdef CONFIG_ESTIMATOR_OOT
  StateEstimatorTypeOutOfTree,
#endif
  StateEstimatorType_COUNT,
} StateEstimatorType;

typedef enum {
  MeasurementTypeTDOA,
  MeasurementTypePosition,
  MeasurementTypePose,
  MeasurementTypeDistance,
  MeasurementTypeTOF,
  MeasurementTypeAbsoluteHeight,
  MeasurementTypeFlow,
  MeasurementTypeYawError,
  MeasurementTypeSweepAngle,
  MeasurementTypeGyroscope,
  MeasurementTypeAcceleration,
  MeasurementTypeBarometer,
} MeasurementType;

typedef struct
{
  MeasurementType type;
  union
  {
    tdoaMeasurement_t tdoa;
    positionMeasurement_t position;
    poseMeasurement_t pose;
    distanceMeasurement_t distance;
    tofMeasurement_t tof;
    heightMeasurement_t height;
    flowMeasurement_t flow;
    yawErrorMeasurement_t yawError;
    sweepAngleMeasurement_t sweepAngle;
    gyroscopeMeasurement_t gyroscope;
    accelerationMeasurement_t acceleration;
    barometerMeasurement_t barometer;
  } data;
} measurement_t;

void stateEstimatorInit(StateEstimatorType estimator);
bool stateEstimatorTest(void);
void stateEstimatorSwitchTo(StateEstimatorType estimator);
void stateEstimator(state_t *state, const stabilizerStep_t stabilizerStep);
StateEstimatorType stateEstimatorGetType(void);
const char* stateEstimatorGetName();

// Support to incorporate additional sensors into the state estimate via the following functions
void estimatorEnqueue(const measurement_t *measurement);

// These helper functions simplify the caller code, but cause additional memory copies
static inline void estimatorEnqueueTDOA(const tdoaMeasurement_t *tdoa)
{
  measurement_t m;
  m.type = MeasurementTypeTDOA;
  m.data.tdoa = *tdoa;
  estimatorEnqueue(&m);
}

static inline void estimatorEnqueuePosition(const positionMeasurement_t *position)
{
  measurement_t m;
  m.type = MeasurementTypePosition;
  m.data.position = *position;
  estimatorEnqueue(&m);
}

static inline void estimatorEnqueuePose(const poseMeasurement_t *pose)
{
  measurement_t m;
  m.type = MeasurementTypePose;
  m.data.pose = *pose;
  estimatorEnqueue(&m);
}

static inline void estimatorEnqueueDistance(const distanceMeasurement_t *distance)
{
  measurement_t m;
  m.type = MeasurementTypeDistance;
  m.data.distance = *distance;
  estimatorEnqueue(&m);
}

static inline void estimatorEnqueueTOF(const tofMeasurement_t *tof)
{
  measurement_t m;
  m.type = MeasurementTypeTOF;
  m.data.tof = *tof;
  estimatorEnqueue(&m);
}

static inline void estimatorEnqueueAbsoluteHeight(const heightMeasurement_t *height)
{
  measurement_t m;
  m.type = MeasurementTypeAbsoluteHeight;
  m.data.height = *height;
  estimatorEnqueue(&m);
}

static inline void estimatorEnqueueFlow(const flowMeasurement_t *flow)
{
  measurement_t m;
  m.type = MeasurementTypeFlow;
  m.data.flow = *flow;
  estimatorEnqueue(&m);
}

static inline void estimatorEnqueueYawError(const yawErrorMeasurement_t *yawError)
{
  measurement_t m;
  m.type = MeasurementTypeYawError;
  m.data.yawError = *yawError;
  estimatorEnqueue(&m);
}

static inline void estimatorEnqueueSweepAngles(const sweepAngleMeasurement_t *sweepAngle)
{
  measurement_t m;
  m.type = MeasurementTypeSweepAngle;
  m.data.sweepAngle = *sweepAngle;
  estimatorEnqueue(&m);
}

// Helper function for state estimators
bool estimatorDequeue(measurement_t *measurement);

#ifdef CONFIG_ESTIMATOR_OOT
void estimatorOutOfTreeInit(void);
bool estimatorOutOfTreeTest(void);
void estimatorOutOfTree(state_t *state, const stabilizerStep_t stabilizerStep);
#endif
