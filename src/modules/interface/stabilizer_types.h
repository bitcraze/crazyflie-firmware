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
#ifndef __STABILIZER_TYPES_H__
#define __STABILIZER_TYPES_H__

#include <stdint.h>
#include <stdbool.h>
#include "imu_types.h"

/* Data structure used by the stabilizer subsystem.
 * All have a timestamp to be set when the data is calculated.
 */

/** Attitude in euler angle form */
typedef struct attitude_s {
  uint32_t timestamp;  // Timestamp when the data was computed

  float roll;
  float pitch;
  float yaw;
} attitude_t;

/* x,y,z vector */
struct vec3_s {
  uint32_t timestamp; // Timestamp when the data was computed

  float x;
  float y;
  float z;
};

typedef struct vec3_s point_t;
typedef struct vec3_s velocity_t;
typedef struct vec3_s acc_t;

/* Orientation as a quaternion */
typedef struct quaternion_s {
  uint32_t timestamp;

  union {
    struct {
      float q0;
      float q1;
      float q2;
      float q3;
    };
    struct {
      float x;
      float y;
      float z;
      float w;
    };
  };
} quaternion_t;

typedef struct baro_s {
  float pressure;
  float temperature;
  float asl;
} baro_t;

typedef struct sensorData_s {
  Axis3f acc;
  Axis3f gyro;
  Axis3f mag;
  baro_t baro;
  point_t position;
} sensorData_t;

typedef struct state_s {
  attitude_t attitude;
  point_t position;
  velocity_t velocity;
  acc_t acc;
} state_t;

typedef struct control_s {
  int16_t roll;
  int16_t pitch;
  int16_t yaw;
  float thrust;
} control_t;

typedef enum mode_e {
  modeDisable = 0,
  modeAbs,
  modeVelocity
} mode_t;

typedef struct setpoint_s {
  attitude_t attitude;
  attitude_t attitudeRate;
  float thrust;
  point_t position;
  velocity_t velocity;

  struct {
    mode_t x;
    mode_t y;
    mode_t z;
    mode_t roll;
    mode_t pitch;
    mode_t yaw;
  } mode;
} setpoint_t;

/** Estimate of position */
typedef struct estimate_s {
  uint32_t timestamp; // Timestamp when the data was computed

  point_t position;
} estimate_t;

/** Setpoint for althold */
typedef struct setpointZ_s {
  float z;
  bool isUpdate; // True = small update of setpoint, false = completely new
} setpointZ_t;

#include "FreeRTOS.h"
#include "task.h"

#define _RATE_SKIP_HZ(X) ((xTaskGetTickCount() % (1000/X)) != 0)

#define RATE_SKIP_500HZ() _RATE_SKIP_HZ(500)
#define RATE_SKIP_250HZ() _RATE_SKIP_HZ(250)
#define RATE_SKIP_100HZ() _RATE_SKIP_HZ(100)


#endif
