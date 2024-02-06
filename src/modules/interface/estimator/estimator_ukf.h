/**
 *                                             __
 *    __  __ ____      ____  __  __ __________/ /_  __   _____________
 *   / /_/ / ___/____ / __ `/ / / /´__  / ___/ __ \/ /  / / ___/ __  /
 *  / __  (__  )/___// /_/ / /_/ / /_/ (__  ) /_/ / /__/ / /  / /_/ /
 * /_/ /_/____/      \__,_/_____/\__  /____/\____/______/_/   \__  /
 *                              /____/                       /____/
 * Crazyflie Project
 *
 * Copyright (C) 2019-2022 University of Applied Sciences Augsburg
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
 * ============================================================================
 * Error-State Unscented Kalman Filter
 * ============================================================================
 * 
 * The error-state unscented Kalman filter implemented in this file is based
 * on the paper:
 * 
 * BIBTEX ENTRIES:
      @INPROCEEDINGS{9841385,
      author={Kefferpütz, Klaus and McGuire, Kimberly},
      booktitle={2022 25th International Conference on Information Fusion (FUSION)}, 
      title={Error-State Unscented Kalman-Filter for UAV Indoor Navigation}, 
      year={2022},
      volume={},
      number={},
      pages={01-08},
      doi={10.23919/FUSION49751.2022.9841385}}
 * 
 * ============================================================================
 * 
 * Authored by Klaus Kefferpütz, December 2022
 *
 * ============================================================================
 *
 * MAJOR CHANGELOG:
 * 2021.09.03 Initial Commit
 *
 */

#ifndef __ERROR_ESTIMATOR_UKF_H__
#define __ERROR_ESTIMATOR_UKF_H__

#include <stdint.h>
#include "stabilizer_types.h"

void errorEstimatorUkfInit(void);
bool errorEstimatorUkfTest(void);
void errorEstimatorUkf(state_t *state, const uint32_t tick);

void errorEstimatorUkfTaskInit();
bool errorEstimatorUkfTaskTest();

void errorEstimatorUkfGetEstimatedPos(point_t* pos);

/**
 * Copies 9 floats representing the current state rotation matrix
 */
void errorEstimatorUkfGetEstimatedRot(float * rotationMatrix);

#endif // __ERROR_ESTIMATOR_UKF_H__
