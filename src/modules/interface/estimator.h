/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
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
#ifndef __ESTIMATOR_H__
#define __ESTIMATOR_H__

#include "stabilizer_types.h"

typedef enum {
  anyEstimator = 0,
  complementaryEstimator,
  kalmanEstimator,
  StateEstimatorTypeCount,
} StateEstimatorType;

void stateEstimatorInit(StateEstimatorType estimator);
bool stateEstimatorTest(void);
void stateEstimator(state_t *state, sensorData_t *sensors, control_t *control, const uint32_t tick);
StateEstimatorType getStateEstimator(void);
const char* stateEstimatorGetName();

#endif //__ESTIMATOR_H__
