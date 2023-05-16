/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
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
 * stabilizer.h: Stabilizer orchestrator
 */
#ifndef STABILIZER_H_
#define STABILIZER_H_

#include <stdbool.h>
#include <stdint.h>

#include "estimator.h"

/**
 * Initialize the stabilizer subsystem and launch the stabilizer loop task.
 * The stabilizer loop task will wait on systemWaitStart() before running.
 */
void stabilizerInit(StateEstimatorType estimator);

/**
 * Test the stabilizer subsystem. Calls test for all the stabilizer related
 * sensors.
 * @return True if all test has passed. False otherwise.
 */
bool stabilizerTest(void);

#endif /* STABILIZER_H_ */
