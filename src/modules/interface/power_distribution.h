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
 * power_distribution.h - Interface to stabilizer power distribution
 */
#ifndef __POWER_DISTRIBUTION_H__
#define __POWER_DISTRIBUTION_H__

#include "stabilizer_types.h"


void powerDistributionInit(void);
bool powerDistributionTest(void);

/**
 * @brief Calculate the power (thrust) of each motor based on the output from the controller
 *
 * @param control Data from the controller
 * @param motorThrustUncapped The desired thrust
 */
void powerDistribution(const control_t *control, motors_thrust_uncapped_t* motorThrustUncapped);

/**
 * @brief Cap the thrust for the motors when out side of the valid range [0 - UINT16_MAX]. The platform specific
 * implementation can chose to cap the trust in a way that provides graceful degradation, for instance prioritizing
 * attitude over thrust.
 *
 * @param motorThrustBatCompUncapped The desired thrust for the motors
 * @param motorPwm The capped thrust
 */
void powerDistributionCap(const motors_thrust_uncapped_t* motorThrustBatCompUncapped, motors_thrust_pwm_t* motorPwm);

/**
 * Returns a 1 when motor 'id' gives thrust, returns 0 otherwise
 */
int powerDistributionMotorType(uint32_t id);

/**
 * Returns the stop ratio of the motor 'id'
 */
uint16_t powerDistributionStopRatio(uint32_t id);

#endif //__POWER_DISTRIBUTION_H__
