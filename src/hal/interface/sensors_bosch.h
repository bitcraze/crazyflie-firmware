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
 * sensors_bosch.h - Sensors interface
 */
#ifndef __SENSORS_BOSCH_H__
#define __SENSORS_BOSCH_H__

#include <math.h>

#include "stm32fxxx.h"

#include "sensors.h"
#include "imu.h"

#include "vl53l0x.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "system.h"
#include "configblock.h"
#include "param.h"
#include "debug.h"
#include "imu.h"
#include "nvicconf.h"
#include "ledseq.h"
#include "sound.h"
#include "filter.h"

/* Bosch Sensortec Drivers */
#include "bmi055.h"
#include "bmi160.h"
#include "bmm150.h"
#include "bmp280.h"
#include "bstdr_comm_support.h"

#define SENSORS_READ_RATE_HZ		1000
#define SENSORS_STARTUP_TIME_MS		1000
#define SENSORS_READ_BARO_HZ		50
#define SENSORS_READ_MAG_HZ			20
#define SENSORS_DELAY_BARO			(SENSORS_READ_RATE_HZ/SENSORS_READ_BARO_HZ)
#define SENSORS_DELAY_MAG			(SENSORS_READ_RATE_HZ/SENSORS_READ_MAG_HZ)

#define SENSORS_VARIANCE_MAN_TEST_TIMEOUT M2T(1000) // Timeout in ms
#define SENSORS_MAN_TEST_LEVEL_MAX        5.0f      // Max degrees off

#define GYRO_NBR_OF_AXES            3
#define GYRO_MIN_BIAS_TIMEOUT_MS    M2T(1*1000)

// Number of samples used in variance calculation. Changing this effects the threshold
#define SENSORS_NBR_OF_BIAS_SAMPLES  512

// Variance threshold to take zero bias for gyro
#define GYRO_VARIANCE_BASE        2000
#define GYRO_VARIANCE_THRESHOLD_X (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Y (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Z (GYRO_VARIANCE_BASE)

#endif /* __SENSORS_BOSCH_H__ */
