/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2022 Bitcraze AB
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
 *
 * platform_defaults_cf2.h - platform specific default values for the cf2 platform
 */

#pragma once

#ifndef __INCLUDED_FROM_PLATFORM_DEFAULTS__
    #pragma GCC error "Do not include this file directly, include platform_defaults.h instead."
#endif

// Defines for default values in the cf2 platform

// Default values for battery limits
#define DEFAULT_BAT_LOW_VOLTAGE                   3.2f
#define DEFAULT_BAT_CRITICAL_LOW_VOLTAGE          3.0f
#define DEFAULT_BAT_LOW_DURATION_TO_TRIGGER_SEC   5

// Default value for system shutdown in minutes after radio silence.
// Requires kbuild config ENABLE_AUTO_SHUTDOWN to be activated.
#define DEFAULT_SYSTEM_SHUTDOWN_TIMEOUT_MIN       5

// Drone physical constants
// m
#define ARM_LENGTH  0.046f
// kg
#define CF_MASS     0.03454f
// thrust coefficients
#if (CONFIG_THRUST_BAT_COMPENSATION_TYPE == 1)    // 2.1+ propellers
#define VMOTOR2THRUST0  -0.02476537915958403f
#define VMOTOR2THRUST1  0.06523793527519485f
#define VMOTOR2THRUST2  -0.026792504967750107f
#define VMOTOR2THRUST3  0.006776789303971145f
#define THRUST_MIN      0.02f
#define THRUST_MAX      0.1125f
#define THRUST2TORQUE   0.005964552f // TODO, value is for the legacy propellers and old battery compensation
#elif (CONFIG_THRUST_BAT_COMPENSATION_TYPE == 2)  // Thrust upgrade kit
#define VMOTOR2THRUST0  -0.03978221591250353f
#define VMOTOR2THRUST1  0.10979738851226176f
#define VMOTOR2THRUST2  -0.05545304285403245f
#define VMOTOR2THRUST3  0.016215002062640885f
#define THRUST_MIN      0.03f
#define THRUST_MAX      0.1625f
#define THRUST2TORQUE   0.005964552f // TODO, value is for the legacy propellers and old battery compensation
#else                                             // default case, legacy propellers
#define VMOTOR2THRUST0  -0.014830744918356092f
#define VMOTOR2THRUST1  0.04724465241828281f
#define VMOTOR2THRUST2  -0.01847364358025878f
#define VMOTOR2THRUST3  0.005960923942142f
#define THRUST_MIN      0.02f
#define THRUST_MAX      0.1125f
#define THRUST2TORQUE   0.005964552f
#endif

// Default PID gains
#define PID_ROLL_RATE_KP  250.0
#define PID_ROLL_RATE_KI  500.0
#define PID_ROLL_RATE_KD  2.5
#define PID_ROLL_RATE_KFF 0.0
#define PID_ROLL_RATE_INTEGRATION_LIMIT    33.3

#define PID_PITCH_RATE_KP  250.0
#define PID_PITCH_RATE_KI  500.0
#define PID_PITCH_RATE_KD  2.5
#define PID_PITCH_RATE_KFF 0.0
#define PID_PITCH_RATE_INTEGRATION_LIMIT   33.3

#define PID_YAW_RATE_KP  120.0
#define PID_YAW_RATE_KI  16.7
#define PID_YAW_RATE_KD  0.0
#define PID_YAW_RATE_KFF 0.0
#define PID_YAW_RATE_INTEGRATION_LIMIT     166.7

#define PID_ROLL_KP  6.0
#define PID_ROLL_KI  3.0
#define PID_ROLL_KD  0.0
#define PID_ROLL_KFF 0.0
#define PID_ROLL_INTEGRATION_LIMIT    20.0

#define PID_PITCH_KP  6.0
#define PID_PITCH_KI  3.0
#define PID_PITCH_KD  0.0
#define PID_PITCH_KFF 0.0
#define PID_PITCH_INTEGRATION_LIMIT   20.0

#define PID_YAW_KP  6.0
#define PID_YAW_KI  1.0
#define PID_YAW_KD  0.35
#define PID_YAW_KFF 0.0
#define PID_YAW_INTEGRATION_LIMIT     360.0

#define PID_VEL_X_KP 25.0f
#define PID_VEL_X_KI 1.0f
#define PID_VEL_X_KD 0.0f
#define PID_VEL_X_KFF 0.0f

#define PID_VEL_Y_KP 25.0f
#define PID_VEL_Y_KI 1.0f
#define PID_VEL_Y_KD 0.0f
#define PID_VEL_Y_KFF 0.0f

#define PID_VEL_Z_KP 25.0f
#define PID_VEL_Z_KI 15.0f
#define PID_VEL_Z_KD 0.0f
#define PID_VEL_Z_KFF 0.0f

#define PID_VEL_Z_KP_BARO_Z_HOLD 3.0f
#define PID_VEL_Z_KI_BARO_Z_HOLD 1.0f
#define PID_VEL_Z_KD_BARO_Z_HOLD 1.5f
#define PID_VEL_Z_KFF_BARO_Z_HOLD 0.0f

#define PID_VEL_ROLL_MAX 20.0f
#define PID_VEL_PITCH_MAX 20.0f
#define PID_VEL_THRUST_BASE 36000.0f
#define PID_VEL_THRUST_BASE_BARO_Z_HOLD 38000.0f
#define PID_VEL_THRUST_MIN 20000.0f

#define PID_POS_X_KP 2.0f
#define PID_POS_X_KI 0.0f
#define PID_POS_X_KD 0.0f
#define PID_POS_X_KFF 0.0f

#define PID_POS_Y_KP 2.0f
#define PID_POS_Y_KI 0.0f
#define PID_POS_Y_KD 0.0f
#define PID_POS_Y_KFF 0.0f

#define PID_POS_Z_KP 2.0f
#define PID_POS_Z_KI 0.5f
#define PID_POS_Z_KD 0.0f
#define PID_POS_Z_KFF 0.0f

#define PID_POS_VEL_X_MAX 1.0f
#define PID_POS_VEL_Y_MAX 1.0f
#define PID_POS_VEL_Z_MAX 1.0f

#if defined(CONFIG_DECK_BIGQUAD) && defined(MOTORS_REQUIRE_ARMING)
    #define CONFIG_MOTORS_DEFAULT_IDLE_THRUST 7000
#endif
