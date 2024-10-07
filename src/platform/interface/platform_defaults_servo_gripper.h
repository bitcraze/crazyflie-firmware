/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2022 Bitcraze AB & Flapper Drones (https://flapper-drones.com)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY},{without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * platform_defaults_flapper.h - platform specific default values for the Flapper Nimble+
 */

#pragma once

#ifndef __INCLUDED_FROM_PLATFORM_DEFAULTS__
    #pragma GCC error "Do not include this file directly, include platform_defaults.h instead."
#endif

// Defines for default values in the flapper platform

// Default values for battery limits
#define DEFAULT_BAT_LOW_VOLTAGE                   6.4f
#define DEFAULT_BAT_CRITICAL_LOW_VOLTAGE          6.0f
#define DEFAULT_BAT_LOW_DURATION_TO_TRIGGER_SEC   5

// Default value for system shutdown in minutes after radio silence.
// Requires kbuild config ENABLE_AUTO_SHUTDOWN to be activated.
#define DEFAULT_SYSTEM_SHUTDOWN_TIMEOUT_MIN       5

// Default PID gains
#define PID_ROLL_RATE_KP  130.0
#define PID_ROLL_RATE_KI  0.0
#define PID_ROLL_RATE_KD  0.0
#define PID_ROLL_RATE_KFF 0.0
#define PID_ROLL_RATE_INTEGRATION_LIMIT    33.3

#define PID_PITCH_RATE_KP  250.0
#define PID_PITCH_RATE_KI  0.0
#define PID_PITCH_RATE_KD  0.0
#define PID_PITCH_RATE_KFF 0.0
#define PID_PITCH_RATE_INTEGRATION_LIMIT   33.3

#define PID_YAW_RATE_KP  200.0
#define PID_YAW_RATE_KI  950.0
#define PID_YAW_RATE_KD  0.0
#define PID_YAW_RATE_KFF 150.0
#define PID_YAW_RATE_INTEGRATION_LIMIT     166.7

#define PID_ROLL_KP  4.0
#define PID_ROLL_KI  20.0
#define PID_ROLL_KD  0.3
#define PID_ROLL_KFF 0.0
#define PID_ROLL_INTEGRATION_LIMIT    20.0

#define PID_PITCH_KP  12.0
#define PID_PITCH_KI  40.0
#define PID_PITCH_KD  0.4
#define PID_PITCH_KFF 0.0
#define PID_PITCH_INTEGRATION_LIMIT   20.0

#define PID_YAW_KP  30.0
#define PID_YAW_KI  0.0
#define PID_YAW_KD  1.0
#define PID_YAW_KFF 0.0
#define PID_YAW_INTEGRATION_LIMIT     360.0

#define PID_VEL_X_KP 25.0f
#define PID_VEL_X_KI 1.0f
#define PID_VEL_X_KD 0.0f
#define PID_VEL_X_KFF 10.0f

#define PID_VEL_Y_KP 15.0f
#define PID_VEL_Y_KI 1.0f
#define PID_VEL_Y_KD 0.0f
#define PID_VEL_Y_KFF 5.0f

#define PID_VEL_Z_KP 12.5f
#define PID_VEL_Z_KI 0.5f
#define PID_VEL_Z_KD 0.0f
#define PID_VEL_Z_KFF 0.0f

#define PID_VEL_Z_KP_BARO_Z_HOLD 4.0f
#define PID_VEL_Z_KI_BARO_Z_HOLD 3.0f
#define PID_VEL_Z_KD_BARO_Z_HOLD 1.0f
#define PID_VEL_Z_KFF_BARO_Z_HOLD 0.0f

#define PID_VEL_ROLL_MAX 20.0f
#define PID_VEL_PITCH_MAX 20.0f
#define PID_VEL_THRUST_BASE 40000.0f
#define PID_VEL_THRUST_BASE_BARO_Z_HOLD 40000.0f
#define PID_VEL_THRUST_MIN 20000.0f

#define PID_POS_X_KP 1.5f
#define PID_POS_X_KI 0.0f
#define PID_POS_X_KD 0.0f
#define PID_POS_X_KFF 0.0f

#define PID_POS_Y_KP 1.5f
#define PID_POS_Y_KI 0.0f
#define PID_POS_Y_KD 0.0f
#define PID_POS_Y_KFF 0.0f

#define PID_POS_Z_KP 5.0f
#define PID_POS_Z_KI 0.5f
#define PID_POS_Z_KD 0.0f
#define PID_POS_Z_KFF 0.0f

#define PID_POS_VEL_X_MAX 1.0f
#define PID_POS_VEL_Y_MAX 1.0f
#define PID_POS_VEL_Z_MAX 1.0f

// PID filter configuration
#define ATTITUDE_ROLL_RATE_LPF_CUTOFF_FREQ 12.5f
#define ATTITUDE_PITCH_RATE_LPF_CUTOFF_FREQ 12.5f
#define ATTITUDE_YAW_RATE_LPF_CUTOFF_FREQ 3.0f
#define ATTITUDE_RATE_LPF_ENABLE true
#define PID_VEL_XY_FILT_CUTOFF 10.0f
#define PID_VEL_Z_FILT_CUTOFF 10.0f

// IMU alignment
//////////////////////////////////
#if CONFIG_POWER_DISTRIBUTION_FLAPPER_REVB
    #define IMU_PHI                   0.0f
    #define IMU_THETA                 90.0f
    #define IMU_PSI                   180.0f
#else
    #define IMU_PHI                   0.0f
    #define IMU_THETA                -90.0f
    #define IMU_PSI                   180.0f
#endif

// Disable tumble check //
//////////////////////////
#define SUPERVISOR_TUMBLE_CHECK_ENABLE false

#define YAW_MAX_DELTA 30.0

#ifdef CONFIG_CONTROLLER_MLESO

#define PID_FEEDBACK_ROLL_KP  1.0
#define PID_FEEDBACK_ROLL_KI  7.5
#define PID_FEEDBACK_ROLL_KD  0.0
#define PID_FEEDBACK_ROLL_KFF 0.0
#define PID_FEEDBACK_ROLL_INTEGRATION_LIMIT    160

#define PID_FEEDBACK_PITCH_KP  0.75
#define PID_FEEDBACK_PITCH_KI  25
#define PID_FEEDBACK_PITCH_KD  0.0
#define PID_FEEDBACK_PITCH_KFF 0.0
#define PID_FEEDBACK_PITCH_INTEGRATION_LIMIT   160

#define PID_FEEDBACK_YAW_KP  0
#define PID_FEEDBACK_YAW_KI  -0.003
#define PID_FEEDBACK_YAW_KD  0.0
#define PID_FEEDBACK_YAW_KFF 0
#define PID_FEEDBACK_YAW_INTEGRATION_LIMIT     166.7

#define K_R {{1.325f, 0, 0, 0},\
    {0, 9.43f, 0, 0},\
    {0, 0, 33.6f, 0},\
    {0, 0, 0, 1.762f}}

#define A_REF {{-10, 0, 0,0,-250,0,-5,0,0},\
    {0,-10,0,250,0,0,0,-5,0},\
    {0,0,-30,0,-3,0,0,0,-1},\
    {0,0.03,0,-4,0,0,0,-0.2,0},\
    {-0.003,0,0,0,-3,0,0.2,0,0},\
    {0,0,0,0,0,-1.5,0,0,0},\
    {1,0,0,0,0,0,0,0,0},\
    {0,1,0,0,0,0,0,0,0},\
    {0,0,1,0,0,0,0,0,0},}

#define B_P {{-472.5283,	472.5283,	0,	-6.0254},\
    {-1.0435,	-1.0435,	-78.5132,	0},\
    {-34.0026,	34.0026,	0,	-234.4742},\
    {-0.5545,	-0.5545,	-0.0036212,	0},\
    {-0.0071,	-0.0071,	0.14639,	0.01057},\
    {-0.39834,	0.39834,	0, 	0.00793},\
    {0,0,0,0},\
    {0,0,0,0},\
    {0,0,0,0},}

#define L0_EST {{-250, 0, 0, 0, 0, 0, 0, 0, 0},\
    {0, -250.000000000000, 0, 0, 0, 0, 0, 0, 0},\
    {0, 0, -250, 0, 0, 0, 0, 0, 0},\
    {0, 0, 0, -250, 0, 0, 0, 0, 0},\
    {0, 0, 0, 0, -250, 0, 0, 0, 0},\
    {0, 0, 0, 0, 0, -250.000000000000, 0, 0, 0},\
    {0, 0, 0, 0, 0, 0, -250, 0, 0},\
    {0, 0, 0, 0, 0, 0, 0, -250, 0},\
    {0, 0, 0, 0, 0, 0, 0, 0, -250},}

#define M0_EST {{0.26502, -0.0104, -0.0068212, 225.4546, 0, -0.0104, 0, 0, 0},\
    {0.26502, -0.0104, -0.0068224, 225.4546, 0, -0.0104, 0, 0, 0},\
    {0.002697, 3.18445, -0.000177, -5.9928, 0, -3.18445, 0, 0, 0},\
    {-0.076866, 0, 1.06819, 0, 0, 0, 0, 0, 0},\
    {0.0004176, -0.4663055, -0.011266, 4.072963, -250, 0.4663, 0, 0, 0},\
    {0.21175, 0, -0.0139, 0, 0, -250, 0, 0, 0},\
    {0, 0, 0, 0, 0, 0, -250, 0, 0},\
    {0, 0, 0, 0, 0, 0, 0, -250, 0},\
    {0, 0, 0, 0, 0, 0, 0, 0, -250},}

#endif