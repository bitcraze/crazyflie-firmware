/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2022-2024 Bitcraze AB
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
 * platform_defaults.h - platform specific default values
 */

#pragma once

#include "autoconf.h"

#define __INCLUDED_FROM_PLATFORM_DEFAULTS__

#ifdef CONFIG_PLATFORM_CF2
    #include "platform_defaults_cf2.h"
#endif
#ifdef CONFIG_PLATFORM_CF21BL
    #include "platform_defaults_cf21bl.h"
#endif
#ifdef CONFIG_PLATFORM_BOLT
    #include "platform_defaults_bolt.h"
#endif
#ifdef CONFIG_PLATFORM_TAG
    #include "platform_defaults_tag.h"
#endif
#ifdef CONFIG_PLATFORM_FLAPPER
    #include "platform_defaults_flapper.h"
#endif

// Drone physical constants
// TODO provide those in each config. Those are values of cf2 platform
#ifndef ARM_LENGTH
    // m
    #define ARM_LENGTH  0.046f
#endif
#ifndef CF_MASS
    // kg
    #define CF_MASS     0.027f
#endif
#ifndef VMOTOR2THRUST0
    #define VMOTOR2THRUST0  0.0f
#endif
#ifndef VMOTOR2THRUST1
    #define VMOTOR2THRUST1  0.0f
#endif
#ifndef VMOTOR2THRUST2
    #define VMOTOR2THRUST2  0.0f
#endif
#ifndef VMOTOR2THRUST3
    #define VMOTOR2THRUST3  0.0f
#endif
#ifndef THRUST_MIN
    #define THRUST_MIN      0.02f
#endif
#ifndef THRUST_MAX
    #define THRUST_MAX      0.1125f
#endif
#ifndef THRUST2TORQUE
    #define THRUST2TORQUE   0.005964552f
#endif

// IMU alignment on the airframe 
#ifndef IMU_PHI
    #define IMU_PHI     0.0f
#endif
#ifndef IMU_THETA
    #define IMU_THETA   0.0f
#endif
#ifndef IMU_PSI
    #define IMU_PSI     0.0f
#endif

// Attitude PID control filter settings
#ifndef ATTITUDE_LPF_CUTOFF_FREQ 
    #define ATTITUDE_LPF_CUTOFF_FREQ      15.0f
#endif
#ifndef ATTITUDE_LPF_ENABLE
    #define ATTITUDE_LPF_ENABLE false
#endif
#ifndef ATTITUDE_ROLL_RATE_LPF_CUTOFF_FREQ
    #define ATTITUDE_ROLL_RATE_LPF_CUTOFF_FREQ 30.0f
#endif
#ifndef ATTITUDE_PITCH_RATE_LPF_CUTOFF_FREQ
    #define ATTITUDE_PITCH_RATE_LPF_CUTOFF_FREQ 30.0f
#endif
#ifndef ATTITUDE_YAW_RATE_LPF_CUTOFF_FREQ
    #define ATTITUDE_YAW_RATE_LPF_CUTOFF_FREQ 30.0f
#endif
#ifndef ATTITUDE_RATE_LPF_ENABLE
    #define ATTITUDE_RATE_LPF_ENABLE false
#endif
#ifndef ATTITUDE_RATE_FF_YAW
    #define ATTITUDE_RATE_FF_YAW 0.0f
#endif
#ifndef YAW_MAX_DELTA
    #define YAW_MAX_DELTA     0.0f
#endif

#ifndef PID_POS_XY_FILT_ENABLE
    #define PID_POS_XY_FILT_ENABLE true
#endif
#ifndef PID_POS_XY_FILT_CUTOFF
    #define PID_POS_XY_FILT_CUTOFF 20.0f
#endif
#ifndef PID_POS_Z_FILT_ENABLE
    #define PID_POS_Z_FILT_ENABLE true
#endif
#ifndef PID_POS_Z_FILT_CUTOFF
    #define PID_POS_Z_FILT_CUTOFF 20.0f
#endif
#ifndef PID_VEL_XY_FILT_ENABLE
    #define PID_VEL_XY_FILT_ENABLE true
#endif
#ifndef PID_VEL_XY_FILT_CUTOFF
    #define PID_VEL_XY_FILT_CUTOFF 20.0f
#endif
#ifndef PID_VEL_Z_FILT_ENABLE
    #define PID_VEL_Z_FILT_ENABLE true
#endif
#ifndef PID_VEL_Z_FILT_CUTOFF
    #define PID_VEL_Z_FILT_CUTOFF 20.0f
#endif
#ifndef PID_VEL_Z_FILT_CUTOFF_BARO_Z_HOLD
    #define PID_VEL_Z_FILT_CUTOFF_BARO_Z_HOLD 0.7f
#endif

// Tumble detection settings
#ifndef SUPERVISOR_TUMBLE_CHECK_ENABLE
    #define SUPERVISOR_TUMBLE_CHECK_ENABLE true
#endif

// 60 degrees tilt (when stationary)
#ifndef SUPERVISOR_TUMBLE_CHECK_ACCEPTED_TILT_ACCZ
    #define SUPERVISOR_TUMBLE_CHECK_ACCEPTED_TILT_ACCZ 0.5f
#endif


#ifndef SUPERVISOR_TUMBLE_CHECK_ACCEPTED_TILT_TIME
    #define SUPERVISOR_TUMBLE_CHECK_ACCEPTED_TILT_TIME 1000
#endif

#ifndef SUPERVISOR_TUMBLE_CHECK_ACCEPTED_UPSIDEDOWN_ACCZ
    #define SUPERVISOR_TUMBLE_CHECK_ACCEPTED_UPSIDEDOWN_ACCZ -0.2f
#endif


#ifndef SUPERVISOR_TUMBLE_CHECK_ACCEPTED_UPSIDEDOWN_TIME
    #define SUPERVISOR_TUMBLE_CHECK_ACCEPTED_UPSIDEDOWN_TIME 100
#endif

// Pre-flight disarming timeout
#ifndef PREFLIGHT_TIMEOUT_MS
    #define PREFLIGHT_TIMEOUT_MS 30000
#endif

// Landing timeout before disarming
#ifndef LANDING_TIMEOUT_MS
    #define LANDING_TIMEOUT_MS 3000
#endif


// Health test parameters
#ifndef HEALTH_BRUSHED_ON_PERIOD_MSEC
    #define HEALTH_BRUSHED_ON_PERIOD_MSEC 50
#endif
#ifndef HEALTH_BRUSHED_OFF_PERIOD_MSEC
    #define HEALTH_BRUSHED_OFF_PERIOD_MSEC 950
#endif
#ifndef HEALTH_BRUSHED_VARIANCE_START_MSEC
    #define HEALTH_BRUSHED_VARIANCE_START_MSEC 0
#endif
#ifndef HEALTH_BRUSHED_PROP_ON_PERIOD_PWM_RATIO
    #define HEALTH_BRUSHED_PROP_ON_PERIOD_PWM_RATIO 0xFFFF
#endif
#ifndef HEALTH_BRUSHED_BAT_ON_PERIOD_PWM_RATIO
    #define HEALTH_BRUSHED_BAT_ON_PERIOD_PWM_RATIO 40000
#endif

#ifndef HEALTH_BRUSHLESS_ON_PERIOD_MSEC
    #define HEALTH_BRUSHLESS_ON_PERIOD_MSEC 2000
#endif
#ifndef HEALTH_BRUSHLESS_OFF_PERIOD_MSEC
    #define HEALTH_BRUSHLESS_OFF_PERIOD_MSEC 1000
#endif
#ifndef HEALTH_BRUSHLESS_VARIANCE_START_MSEC
    #define HEALTH_BRUSHLESS_VARIANCE_START_MSEC 1000
#endif


// This is the threshold for a propeller/motor to pass. It calculates the
// variance of the accelerometer X+Y when the propeller is spinning.
#ifndef HEALTH_PROPELLER_TEST_THRESHOLD
    #define HEALTH_PROPELLER_TEST_THRESHOLD  0.0f
#endif
