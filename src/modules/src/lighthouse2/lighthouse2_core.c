/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2023 Bitcraze AB
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
 * lighthouse2_core.c - central part of the lighthouse positioning system
 */

#include "stm32fxxx.h"
#include "FreeRTOS.h"
#include "task.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "system.h"
#include "param.h"
#include "autoconf.h"

// Uncomment next line to add extra debug log variables
#define CONFIG_DEBUG_LOG_ENABLE 1

#include "log.h"
#include "statsCnt.h"

#define DEBUG_MODULE "LH2"
#include "debug.h"

// #define USE_UART1 1
#ifdef USE_UART1
#include "uart1.h"
#else
#include "uart2.h"
#endif

#include "static_mem.h"
#include "estimator.h"

#include "lighthouse2_core.h"
#include "lighthouse_types.h"
#include "stabilizer_types.h"
#include "lighthouse_geometry.h"
#include "lighthouse_calibration.h"
#include "lighthouse_storage.h"
#include "physicalConstants.h"

typedef struct {
  bool isSyncFrame;
  uint8_t bs;
  uint8_t sweepId;
  uint8_t sensor;
  uint8_t rotationPseudoCount;
  float angle;
} lighthouse2UartFrame_t;

static lighthouse2UartFrame_t frame;

// Stats

static bool uartSynchronized = false;

#define ONE_SECOND 1000

#ifdef CONFIG_DEBUG_LOG_ENABLE
static STATS_CNT_RATE_DEFINE(serialFrameRate, ONE_SECOND);
static STATS_CNT_RATE_DEFINE(frameRate, ONE_SECOND);
static STATS_CNT_RATE_DEFINE(estimatorRate, ONE_SECOND);
#endif

#define UART_FRAME_LENGTH 7


static float stdDevAngle = 0.01;
static float stdDevAngleDiff = 0.00005;
static uint8_t logBaseStationId;
static uint8_t logSensor;
static float logAngle1;
static float logAngle2;

// The light planes in LH2 are tilted +- 30 degrees
static const float t30 = M_PI / 6;

// Sensor positions on the deck. The positions are rotated compared to the original lighthouse deck
#define SENSOR_POS_W (0.015f / 2.0f)
#define SENSOR_POS_L (0.030f / 2.0f)
static vec3d sensorDeckPositionsV2[4] = {
    {SENSOR_POS_L, -SENSOR_POS_W, 0.0},
    {SENSOR_POS_L, SENSOR_POS_W, 0.0},
    {-SENSOR_POS_L, -SENSOR_POS_W, 0.0},
    {-SENSOR_POS_L, SENSOR_POS_W, 0.0},
};


#define NR_OF_BASE_STATIONS 16
static lighthouseCalibration_t bsCalibration[NR_OF_BASE_STATIONS];
static baseStationGeometry_t bsGeometry[NR_OF_BASE_STATIONS];
static baseStationGeometryCache_t bsGeoCache[NR_OF_BASE_STATIONS];

static void useFrame(const lighthouse2UartFrame_t* frame);
static void preProcessGeometryData(mat3d bsRot, mat3d bsRotInverted, mat3d lh1Rotor2Rot, mat3d lh1Rotor2RotInverted);

void lighthouse2CoreInit() {
}

bool getUartFrameRaw(lighthouse2UartFrame_t *frame) {
  static char data[UART_FRAME_LENGTH];
  int syncCounter = 0;

  #ifdef USE_UART1
    // Wait until there is enough data available in the queue before reading
    // to optimize the CPU usage. Locking on the queue (as is done in uart2GetDataWithTimeout()) seems to take a lot
    // of time and the vTaskDelay() solution uses much less CPU.
  while (uart1bytesAvailable() < UART_FRAME_LENGTH) {
    vTaskDelay(1);
  }
  for(int i = 0; i < UART_FRAME_LENGTH; i++) {
    uart1Getchar((char*)&data[i]);
  }
  #else
  uart2GetData(UART_FRAME_LENGTH, data);
  #endif

  for(int i = 0; i < UART_FRAME_LENGTH; i++) {
    if ((unsigned char)data[i] == 0xff) {
      syncCounter += 1;
    }
  }

  memset(frame, 0, sizeof(*frame));

  frame->isSyncFrame = (syncCounter == UART_FRAME_LENGTH);

  bool isFrameValid = true;
  if (!frame->isSyncFrame) {
    uint8_t crc = 0;
    for (int i = 0; i < UART_FRAME_LENGTH - 1; i++) {
      crc ^= data[i];
    }

    const uint8_t expectedCrc = data[5];
    if (expectedCrc == crc) {
      const bool isAux = (data[0] >> 7) & 0x01;
      if (!isAux) {
        frame->bs = data[0] & 0x0f;
        frame->sensor = (data[0] >> 4) & 0x03;
        frame->sweepId = (data[0] >> 6) & 0x01;
        frame->rotationPseudoCount = data[1];
        memcpy(&frame->angle, &data[2], 4);
      }
    } else {
      isFrameValid = false;
    }
  }

  STATS_CNT_RATE_EVENT_DEBUG(&serialFrameRate);

  return isFrameValid;
}

void waitForUartSynchFrame() {
  char c;
  int syncCounter = 0;
  bool synchronized = false;

  while (!synchronized) {
    #ifdef USE_UART1
    uart1Getchar(&c);
    #else
    uart2Getchar(&c);
    #endif
    if ((unsigned char)c == 0xff) {
      syncCounter += 1;
    } else {
      syncCounter = 0;
    }
    synchronized = (syncCounter == UART_FRAME_LENGTH);
  }
}

static void initGeoAndCalibDataFromStorage() {
  lighthouseStorageVerifySetStorageVersion();

  for (uint8_t baseStation = 0; baseStation < NR_OF_BASE_STATIONS; baseStation++) {
    lighthouseStorageReadCalibDataFromStorage(baseStation, &bsCalibration[baseStation]);

    lighthouseStorageReadGeoDataFromStorage(baseStation, &bsGeometry[baseStation]);
    if (bsGeometry[baseStation].valid) {
      baseStationGeometryCache_t* cache = &bsGeoCache[baseStation];
      preProcessGeometryData(bsGeometry[baseStation].mat, cache->baseStationInvertedRotationMatrixes, cache->lh1Rotor2RotationMatrixes, cache->lh1Rotor2InvertedRotationMatrixes);
    }
  }
}


void lighthouse2CoreTask(void *param) {
  bool isUartFrameValid = false;

  #ifdef USE_UART1
  uart1Init(230400);
  ASSERT(uart1QueueMaxLength() >= UART_FRAME_LENGTH);
  #else
  uart2Init(230400);
  // ASSERT(uart2QueueMaxLength() >= UART_FRAME_LENGTH);
  #endif

  systemWaitStart();

  initGeoAndCalibDataFromStorage();

  vTaskDelay(M2T(100));

  while(1) {
    waitForUartSynchFrame();
    uartSynchronized = true;

    DEBUG_PRINT("UART synchronized\n");

    while((isUartFrameValid = getUartFrameRaw(&frame))) {
      if(!frame.isSyncFrame) {
        STATS_CNT_RATE_EVENT_DEBUG(&frameRate);
        useFrame(&frame);

        if (frame.bs == logBaseStationId) {
          if (frame.sensor == logSensor) {
            if (frame.sweepId == 0) {
              logAngle1 = frame.angle;
            } else {
              logAngle2 = frame.angle;
            }
          }
        }
      }
    }

    uartSynchronized = false;
  }
}

static void useFrame(const lighthouse2UartFrame_t* frame) {
  const uint8_t bs = frame->bs;
  if (bs < NR_OF_BASE_STATIONS && bsGeometry[bs].valid) {
    const lighthouseCalibration_t* bsCalib = &bsCalibration[bs];
    sweepAngleMeasurement_t sweepInfo;
    sweepInfo.stdDevAngle = stdDevAngle;
    sweepInfo.stdDevAngleDiff = stdDevAngleDiff;
    sweepInfo.rotorPos = &bsGeometry[bs].origin;
    sweepInfo.rotorRot = &bsGeometry[bs].mat;
    sweepInfo.rotorRotInv = &bsGeoCache[bs].baseStationInvertedRotationMatrixes;
    sweepInfo.calibrationMeasurementModel = lighthouseCalibrationMeasurementModelLh2;
    sweepInfo.baseStationId = bs;
    sweepInfo.sensorId1 = frame->sensor;
    sweepInfo.sensorId2 = 0xff; // Not uses

    sweepInfo.sensorPos1 = &sensorDeckPositionsV2[frame->sensor];

    sweepInfo.measuredSweepAngle1 = frame->angle;
    sweepInfo.calib = &bsCalib->sweep[frame->sweepId];
    sweepInfo.sweepId = frame->sweepId;
    if (frame->sweepId == 0) {
      sweepInfo.t = -t30;
    } else {
      sweepInfo.t = t30;
    }
    estimatorEnqueueSweepAngles(&sweepInfo);
    STATS_CNT_RATE_EVENT_DEBUG(&estimatorRate);
  }
}

static void preProcessGeometryData(mat3d bsRot, mat3d bsRotInverted, mat3d lh1Rotor2Rot, mat3d lh1Rotor2RotInverted) {
  // For a rotation matrix inverse and transpose is equal. Use transpose instead
  arm_matrix_instance_f32 bsRot_ = {3, 3, (float32_t *)bsRot};
  arm_matrix_instance_f32 bsRotInverted_ = {3, 3, (float32_t *)bsRotInverted};
  mat_trans(&bsRot_, &bsRotInverted_);

  // In a LH1 system, the axis of rotation of the second rotor is perpendicular to the first rotor
  mat3d secondRotorInvertedR = {
    {1, 0, 0},
    {0, 0, -1},
    {0, 1, 0}
  };
  arm_matrix_instance_f32 secondRotorInvertedR_ = {3, 3, (float32_t *)secondRotorInvertedR};
  arm_matrix_instance_f32 lh1Rotor2Rot_ = {3, 3, (float32_t *)lh1Rotor2Rot};
  mat_mult(&bsRot_, &secondRotorInvertedR_, &lh1Rotor2Rot_);

  arm_matrix_instance_f32 lh1Rotor2RotInverted_ = {3, 3, (float32_t *)lh1Rotor2RotInverted};
  mat_trans(&lh1Rotor2Rot_, &lh1Rotor2RotInverted_);
}


/**
 * Log group for the lighthouse2 positioning system
 */
LOG_GROUP_START(lighthouse2)

/**
 * @brief Rate of frames from the Lighthouse deck on the serial buss [1/s]
 */
STATS_CNT_RATE_LOG_ADD_DEBUG(serRt, &serialFrameRate)

/**
 * @brief Rate of frames from the Lighthouse deck that contains sweep data [1/s]
 */
STATS_CNT_RATE_LOG_ADD_DEBUG(frmRt, &frameRate)

STATS_CNT_RATE_LOG_ADD_DEBUG(estRt, &estimatorRate)

LOG_ADD(LOG_UINT8, comSync, &uartSynchronized)
LOG_ADD(LOG_FLOAT, ang1, &logAngle1)
LOG_ADD(LOG_FLOAT, ang2, &logAngle2)
LOG_GROUP_STOP(lighthouse2)

/**
 * Parameters and settings for the Lighthouse positioning system 2.0
 */
PARAM_GROUP_START(lighthouse2)
/**
 * @brief Standard deviation for Sweep angles
 */
PARAM_ADD(PARAM_FLOAT, angStd, &stdDevAngle)
PARAM_ADD(PARAM_FLOAT, difStd, &stdDevAngleDiff)
PARAM_ADD(PARAM_UINT8, logBs, &logBaseStationId)
PARAM_ADD(PARAM_UINT8, logSens, &logSensor)

PARAM_GROUP_STOP(lighthouse)
