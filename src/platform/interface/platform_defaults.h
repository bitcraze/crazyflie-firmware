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
 * platform_defaults.h - platform specific default values
 */

#pragma once

#include "autoconf.h"

#define __INCLUDED_FROM_PLATFORM_DEFAULTS__

#ifdef CONFIG_PLATFORM_CF2
    #include "platform_defaults_cf2.h"
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
    #define PID_VEL_Z_FILT_CUTOFF_BARO_Z_HOLD 0.7 f
#endif

// Tumble detection enabled by default
#ifndef SUPERVISOR_TUMBLE_CHECK_ENABLE
    #define SUPERVISOR_TUMBLE_CHECK_ENABLE true
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


