/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--'  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2025 Bitcraze AB
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
 */

#pragma once

/**
 * @file kalman_core_params_defaults.h
 * @brief Single source of truth for Kalman core parameter default values
 *
 * This header defines default parameter values as a macro to avoid duplication.
 * The macro is used in two places:
 * 1. estimator_kalman.c - Static initializer for firmware (avoids overwriting persistent params)
 * 2. kalman_core.c - kalmanCoreDefaultParams() function for Python bindings
 */

#include "platform_defaults.h"

// Process noise defaults depend on configuration
#ifdef CONFIG_ESTIMATOR_KALMAN_GENERAL_PURPOSE
#define KALMAN_CORE_PROC_NOISE_DEFAULTS \
  .procNoiseAcc_xy = 0.5f, \
  .procNoiseAcc_z = 0.5f
#else
#define KALMAN_CORE_PROC_NOISE_DEFAULTS \
  .procNoiseAcc_xy = 0.5f, \
  .procNoiseAcc_z = 1.0f
#endif

#ifdef CONFIG_STIMATOR_KALMAN_INITIAL_YAW_STD
#define KALMAN_INITIAL_YAW_STD (CONFIG_STIMATOR_KALMAN_INITIAL_YAW_STD / 1000.0f)
#else
#define KALMAN_INITIAL_YAW_STD 0.01f
#endif

/**
 * @brief Macro containing default values for kalmanCoreParams_t
 *
 * Use as a designated initializer, e.g.:
 *   kalmanCoreParams_t params = { KALMAN_CORE_DEFAULT_PARAMS_INIT };
 */
#define KALMAN_CORE_DEFAULT_PARAMS_INIT \
  /* Initial variances, uncertain of position, but know we're stationary and roughly flat */ \
  .stdDevInitialPosition_xy = 100, \
  .stdDevInitialPosition_z = 1, \
  .stdDevInitialVelocity = 0.01, \
  .stdDevInitialAttitude_rollpitch = 0.01, \
  .stdDevInitialAttitude_yaw = KALMAN_INITIAL_YAW_STD, \
  \
  KALMAN_CORE_PROC_NOISE_DEFAULTS, \
  .procNoiseVel = 0, \
  .procNoisePos = 0, \
  .procNoiseAtt = 0, \
  .measNoiseBaro = 2.0f,           /* meters */ \
  .measNoiseGyro_rollpitch = 0.1f, /* radians per second */ \
  .measNoiseGyro_yaw = 0.1f,       /* radians per second */ \
  \
  .initialX = 0.0, \
  .initialY = 0.0, \
  .initialZ = 0.0, \
  /* Initial yaw of the Crazyflie in radians. */ \
  /* 0 --- facing positive X */ \
  /* PI / 2 --- facing positive Y */ \
  /* PI --- facing negative X */ \
  /* 3 * PI / 2 --- facing negative Y */ \
  .initialYaw = 0.0, \
  \
  /* Roll/pitch/yaw zero reversion is on by default. Will be overridden by estimatorKalmanInit() if requested by the deck. */ \
  .attitudeReversion = 0.001f, \
  \
  .dragB_x = DRAG_B_X, \
  .dragB_y = DRAG_B_Y, \
  .dragB_z = DRAG_B_Z, \
  \
  .cop_x = CENTER_OF_PRESSURE_X, \
  .cop_y = CENTER_OF_PRESSURE_Y, \
  .cop_z = CENTER_OF_PRESSURE_Z

