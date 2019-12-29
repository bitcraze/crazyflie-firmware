/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2018-2019 Bitcraze AB
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * lighthouse.c: lighthouse tracking system receiver
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "system.h"
#include "deck.h"
#include "log.h"
#include "param.h"
#include "statsCnt.h"

#include "config.h"
#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_MODULE "LH"
#include "debug.h"
#include "uart1.h"
#include "lh_bootloader.h"

#include "pulse_processor.h"
#include "lighthouse.h"

#include "estimator.h"
#include "estimator_kalman.h"

#ifdef LH_FLASH_DECK
#include "lh_flasher.h"
#endif

#ifndef DISABLE_LIGHTHOUSE_DRIVER
  #define DISABLE_LIGHTHOUSE_DRIVER 1
#endif

baseStationGeometry_t lighthouseBaseStationsGeometry[2]  = {
{.origin = {-1.958483,  0.542299,  3.152727, }, .mat = {{0.79721498, -0.004274, 0.60368103, }, {0.0, 0.99997503, 0.00708, }, {-0.60369599, -0.005645, 0.79719502, }, }},
{.origin = {1.062398, -2.563488,  3.112367, }, .mat = {{0.018067, -0.999336, 0.031647, }, {0.76125097, 0.034269, 0.64755201, }, {-0.648206, 0.012392, 0.76136398, }, }},
};

// Uncomment if you want to force the Crazyflie to reflash the deck at each startup
// #define FORCE_FLASH true

static bool isInit = false;

#if DISABLE_LIGHTHOUSE_DRIVER == 1
  void lightHouseGeometryDataUpdated() { /* Empty by design */ }
#else

baseStationEulerAngles_t lighthouseBaseStationAngles[2];
static mat3d baseStationInvertedRotationMatrixes[2];

// Sensor positions on the deck
#define SENSOR_POS_W (0.015f / 2.0f)
#define SENSOR_POS_L (0.030f / 2.0f)
static vec3d sensorDeckPositions[4] = {
    {-SENSOR_POS_L, SENSOR_POS_W, 0.0},
    {-SENSOR_POS_L, -SENSOR_POS_W, 0.0},
    {SENSOR_POS_L, SENSOR_POS_W, 0.0},
    {SENSOR_POS_L, -SENSOR_POS_W, 0.0},
};

#ifndef FORCE_FLASH
#define FORCE_FLASH false
#endif

#define STR2(x) #x
#define STR(x) STR2(x)

#define INCBIN(name, file) \
    __asm__(".section .rodata\n" \
            ".global incbin_" STR(name) "_start\n" \
            ".align 4\n" \
            "incbin_" STR(name) "_start:\n" \
            ".incbin \"" file "\"\n" \
            \
            ".global incbin_" STR(name) "_end\n" \
            ".align 1\n" \
            "incbin_" STR(name) "_end:\n" \
            ".byte 0\n" \
            ".align 4\n" \
            STR(name) "Size:\n" \
            ".int incbin_" STR(name) "_end - incbin_" STR(name) "_start\n" \
    ); \
    extern const __attribute__((aligned(4))) void* incbin_ ## name ## _start; \
    extern const void* incbin_ ## name ## _end; \
    extern const int name ## Size; \
    static const __attribute__((used)) unsigned char* name = (unsigned char*) & incbin_ ## name ## _start; \

INCBIN(bitstream, "blobs/lighthouse.bin");

static void checkVersionAndBoot();

static pulseProcessorResult_t angles;

// Stats
static bool comSynchronized = false;

#define ONE_SECOND 1000
#define HALF_SECOND 500
static STATS_CNT_RATE_DEFINE(serialFrameRate, ONE_SECOND);
static STATS_CNT_RATE_DEFINE(frameRate, ONE_SECOND);
static STATS_CNT_RATE_DEFINE(cycleRate, ONE_SECOND);
static STATS_CNT_RATE_DEFINE(positionRate, ONE_SECOND);

static STATS_CNT_RATE_DEFINE(estBs0Rate, HALF_SECOND);
static STATS_CNT_RATE_DEFINE(estBs1Rate, HALF_SECOND);
static statsCntRateLogger_t* bsEstRates[2] = {&estBs0Rate, &estBs1Rate};

static STATS_CNT_RATE_DEFINE(bs0Rate, HALF_SECOND);
static STATS_CNT_RATE_DEFINE(bs1Rate, HALF_SECOND);
static statsCntRateLogger_t* bsRates[2] = {&bs0Rate, &bs1Rate};

static uint16_t pulseWidth[PULSE_PROCESSOR_N_SENSORS];

typedef union frame_u {
  struct {
    uint32_t timestamp:29;
    uint32_t sensor:3;
    uint16_t width;
    uint8_t sync;
  } __attribute__((packed));
  char data[7];
} __attribute__((packed)) frame_t;

static bool getFrame(frame_t *frame)
{
  int syncCounter = 0;
  for(int i=0; i<7; i++) {
    uart1Getchar(&frame->data[i]);
    if (frame->data[i] != 0) {
      syncCounter += 1;
    }
  }
  return (frame->sync == 0 || (syncCounter==7));
}

static vec3d position;
static float deltaLog;

static positionMeasurement_t ext_pos;
static float sweepStd = 0.0004;

// Method used to estimate position
// 0 = Position calculated outside the estimator using intersection point of beams.
//     Yaw error calculated outside the estimator. Position and yaw error is pushed to the
//     estimator as pre-calculated.
// 1 = Sweep angles pushed into the estimator. Yaw error calculated outside the estimator
//     and pushed to the estimator as a pre-calculated value.
static uint8_t estimationMethod = 1;

static void estimatePositionCrossingBeams(pulseProcessorResult_t* angles, int baseStation) {
  memset(&ext_pos, 0, sizeof(ext_pos));
  int sensorsUsed = 0;
  float delta;

  // Average over all sensors with valid data
  for (size_t sensor = 0; sensor < PULSE_PROCESSOR_N_SENSORS; sensor++) {
      pulseProcessorBaseStationMeasuremnt_t* bs0Measurement = &angles->sensorMeasurements[sensor].baseStatonMeasurements[0];
      pulseProcessorBaseStationMeasuremnt_t* bs1Measurement = &angles->sensorMeasurements[sensor].baseStatonMeasurements[1];

      if (bs0Measurement->validCount == PULSE_PROCESSOR_N_SWEEPS && bs1Measurement->validCount == PULSE_PROCESSOR_N_SWEEPS) {
        lighthouseGeometryGetPositionFromRayIntersection(lighthouseBaseStationsGeometry, bs0Measurement->correctedAngles, bs1Measurement->correctedAngles, position, &delta);

        deltaLog = delta;

        ext_pos.x += position[0];
        ext_pos.y += position[1];
        ext_pos.z += position[2];
        sensorsUsed++;

        STATS_CNT_RATE_EVENT(&positionRate);
      }
  }

  ext_pos.x /= sensorsUsed;
  ext_pos.y /= sensorsUsed;
  ext_pos.z /= sensorsUsed;

  // Make sure we feed sane data into the estimator
  if (isfinite(ext_pos.pos[0]) && isfinite(ext_pos.pos[1]) && isfinite(ext_pos.pos[2])) {
    ext_pos.stdDev = 0.01;
    estimatorEnqueuePosition(&ext_pos);
  }
}

static void estimatePositionSweeps(pulseProcessorResult_t* angles, int baseStation) {
  for (size_t sensor = 0; sensor < PULSE_PROCESSOR_N_SENSORS; sensor++) {
    pulseProcessorBaseStationMeasuremnt_t* bsMeasurement = &angles->sensorMeasurements[sensor].baseStatonMeasurements[baseStation];
    if (bsMeasurement->validCount == PULSE_PROCESSOR_N_SWEEPS) {
      sweepAngleMeasurement_t sweepAngles;
      sweepAngles.angleX = bsMeasurement->correctedAngles[0];
      sweepAngles.angleY = bsMeasurement->correctedAngles[1];

      if (sweepAngles.angleX != 0 && sweepAngles.angleY != 0) {
        sweepAngles.stdDevX = sweepStd;
        sweepAngles.stdDevY = sweepStd;

        sweepAngles.sensorPos = &sensorDeckPositions[sensor];

        sweepAngles.baseStationPos = &lighthouseBaseStationsGeometry[baseStation].origin;
        sweepAngles.baseStationRot = &lighthouseBaseStationsGeometry[baseStation].mat;
        sweepAngles.baseStationRotInv = &baseStationInvertedRotationMatrixes[baseStation];

        estimatorEnqueueSweepAngles(&sweepAngles);
        STATS_CNT_RATE_EVENT(bsEstRates[baseStation]);
      }
    }
  }
}

static bool estimateYawDeltaOneBaseStation(const int bs, const pulseProcessorResult_t* angles, baseStationGeometry_t baseStationGeometries[], const float cfPos[3], const float n[3], const arm_matrix_instance_f32 *RR, float *yawDelta) {
  baseStationGeometry_t* baseStationGeometry = &baseStationGeometries[bs];

  vec3d baseStationPos;
  lighthouseGeometryGetBaseStationPosition(baseStationGeometry, baseStationPos);

  vec3d rays[PULSE_PROCESSOR_N_SENSORS];
  for (int sensor = 0; sensor < PULSE_PROCESSOR_N_SENSORS; sensor++) {
    const pulseProcessorBaseStationMeasuremnt_t* bsMeasurement = &angles->sensorMeasurements[sensor].baseStatonMeasurements[bs];
    lighthouseGeometryGetRay(baseStationGeometry, bsMeasurement->correctedAngles[0], bsMeasurement->correctedAngles[1], rays[sensor]);
  }

  // Intersection points of rays and the deck
  vec3d intersectionPoints[PULSE_PROCESSOR_N_SENSORS];
  for (int sensor = 0; sensor < PULSE_PROCESSOR_N_SENSORS; sensor++) {
    bool exists = lighthouseGeometryIntersectionPlaneVector(baseStationPos, rays[sensor], cfPos, n, intersectionPoints[sensor]);
    if (! exists) {
      return false;
    }
  }

  // Calculate positions of sensors. Rotate relative postiions using the rotation matrix and add current position
  vec3d sensorPoints[PULSE_PROCESSOR_N_SENSORS];
  for (int sensor = 0; sensor < PULSE_PROCESSOR_N_SENSORS; sensor++) {
    lighthouseGeometryGetSensorPosition(cfPos, RR, sensorDeckPositions[sensor], sensorPoints[sensor]);
  }

  // Calculate diagonals (sensors 0 - 3 and 1 - 2) for intersection and sensor points
  vec3d ipv1 = {intersectionPoints[3][0] - intersectionPoints[0][0], intersectionPoints[3][1] - intersectionPoints[0][1], intersectionPoints[3][2] - intersectionPoints[0][2]};
  vec3d ipv2 = {intersectionPoints[2][0] - intersectionPoints[1][0], intersectionPoints[2][1] - intersectionPoints[1][1], intersectionPoints[2][2] - intersectionPoints[1][2]};
  vec3d spv1 = {sensorPoints[3][0] - sensorPoints[0][0], sensorPoints[3][1] - sensorPoints[0][1], sensorPoints[3][2] - sensorPoints[0][2]};
  vec3d spv2 = {sensorPoints[2][0] - sensorPoints[1][0], sensorPoints[2][1] - sensorPoints[1][1], sensorPoints[2][2] - sensorPoints[1][2]};

  // Calculate yaw delta for the two diagonals and average
  float yawDelta1, yawDelta2;
  if (lighthouseGeometryYawDelta(ipv1, spv1, n, &yawDelta1) && lighthouseGeometryYawDelta(ipv2, spv2, n, &yawDelta2)) {
    *yawDelta = (yawDelta1 + yawDelta2) / 2.0f;
    return true;
   } else {
    *yawDelta = 0.0f;
    return false;
  }
}

static void estimateYaw(pulseProcessorResult_t* angles, int baseStation) {
  // TODO Most of these calculations should be moved into the estimator instead. It is a
  // bit dirty to get the state from the kalman filer here and calculate the yaw error outside
  // the estimator, but it will do for now.

  // Get data from the current estimated state
  point_t cfPosP;
  estimatorKalmanGetEstimatedPos(&cfPosP);
  vec3d cfPos = {cfPosP.x, cfPosP.y, cfPosP.z};

  // Rotation matrix
  float R[3][3];
  estimatorKalmanGetEstimatedRot((float*)R);
  arm_matrix_instance_f32 RR = {3, 3, (float*)R};

  // Normal to the deck: (0, 0, 1), rotated using the rotation matrix
  const vec3d n = {R[0][2], R[1][2], R[2][2]};

  // Calculate yaw delta using only one base station for now
  float yawDelta;
  if (estimateYawDeltaOneBaseStation(baseStation, angles, lighthouseBaseStationsGeometry, cfPos, n, &RR, &yawDelta)) {
    yawErrorMeasurement_t yawDeltaMeasurement = {.yawError = yawDelta, .stdDev = 0.01};
    estimatorEnqueueYawError(&yawDeltaMeasurement);
  }
}

static void estimatePoseCrossingBeams(pulseProcessorResult_t* angles, int baseStation) {
  estimatePositionCrossingBeams(angles, baseStation);
  estimateYaw(angles, baseStation);
}

static void estimatePoseSweeps(pulseProcessorResult_t* angles, int baseStation) {
  estimatePositionSweeps(angles, baseStation);
  estimateYaw(angles, baseStation);
}

static void invertRotationMatrix(mat3d rot, mat3d inverted) {
  // arm_mat_inverse_f32() alters the original matrix in the process, must make a copy to work from
  float bs_r_tmp[3][3];
  memcpy(bs_r_tmp, (float32_t *)rot, sizeof(bs_r_tmp));
  arm_matrix_instance_f32 basestation_rotation_matrix_tmp = {3, 3, (float32_t *)bs_r_tmp};

  arm_matrix_instance_f32 basestation_rotation_matrix_inv = {3, 3, (float32_t *)inverted};
  arm_mat_inverse_f32(&basestation_rotation_matrix_tmp, &basestation_rotation_matrix_inv);
}

static void usePulseResultCrossingBeams(pulseProcessor_t *appState, pulseProcessorResult_t* angles, int basestation, int axis) {
  if (basestation == 1 && axis == sweepDirection_y) {
    STATS_CNT_RATE_EVENT(&cycleRate);

    pulseProcessorApplyCalibration(appState, angles, 0);
    pulseProcessorApplyCalibration(appState, angles, 1);

    estimatePoseCrossingBeams(angles, 1);

    pulseProcessorClear(angles, 0);
    pulseProcessorClear(angles, 1);
  }
}

static void usePulseResultSweeps(pulseProcessor_t *appState, pulseProcessorResult_t* angles, int basestation, int axis) {
  if (axis == sweepDirection_y) {
    STATS_CNT_RATE_EVENT(&cycleRate);

    pulseProcessorApplyCalibration(appState, angles, basestation);

    estimatePoseSweeps(angles, basestation);

    pulseProcessorClear(angles, basestation);
  }
}

static void usePulseResult(pulseProcessor_t *appState, pulseProcessorResult_t* angles, int basestation, int axis) {
  switch(estimationMethod) {
    case 0:
      usePulseResultCrossingBeams(appState, angles, basestation, axis);
      break;
    case 1:
      usePulseResultSweeps(appState, angles, basestation, axis);
      break;
    default:
      break;
  }
}

void lightHouseGeometryDataUpdated() {
  lighthouseGeometryCalculateAnglesFromRotationMatrix(&lighthouseBaseStationsGeometry[0],&lighthouseBaseStationAngles[0]);
  lighthouseGeometryCalculateAnglesFromRotationMatrix(&lighthouseBaseStationsGeometry[1],&lighthouseBaseStationAngles[1]);

  invertRotationMatrix(lighthouseBaseStationsGeometry[0].mat, baseStationInvertedRotationMatrixes[0]);
  invertRotationMatrix(lighthouseBaseStationsGeometry[1].mat, baseStationInvertedRotationMatrixes[1]);
}

static void lighthouseTask(void *param)
{
  bool synchronized = false;
  int syncCounter = 0;
  char c;
  static frame_t frame;
  static pulseProcessor_t ppState = {};

  int basestation;
  int axis;

  lightHouseGeometryDataUpdated();

  systemWaitStart();

#ifdef LH_FLASH_DECK
  // Flash deck bootloader using SPI (factory and recovery flashing)
  lhflashInit();
  lhflashFlashBootloader();
#endif

  // Boot the deck firmware
  checkVersionAndBoot();

  while(1) {
    // Synchronize
    syncCounter = 0;
    while (!synchronized) {

      uart1Getchar(&c);
      if (c != 0) {
        syncCounter += 1;
      } else {
        syncCounter = 0;
      }
      synchronized = syncCounter == 7;
    }

    comSynchronized = true;
    DEBUG_PRINT("Synchronized!\n");

    // Receive data until being desynchronized
    synchronized = getFrame(&frame);
    while(synchronized) {
      if (frame.sync != 0) {
        synchronized = getFrame(&frame);
        memset(pulseWidth, 0, sizeof(pulseWidth[0])*PULSE_PROCESSOR_N_SENSORS);
        continue;
      }

      STATS_CNT_RATE_EVENT(&serialFrameRate);

      pulseWidth[frame.sensor] = frame.width;

      if (pulseProcessorProcessPulse(&ppState, frame.sensor, frame.timestamp, frame.width, &angles, &basestation, &axis)) {
        STATS_CNT_RATE_EVENT(&frameRate);
        STATS_CNT_RATE_EVENT(bsRates[basestation]);
        usePulseResult(&ppState, &angles, basestation, axis);
      }

      synchronized = getFrame(&frame);
      if (frame.sync != 0) {
        synchronized = getFrame(&frame);
        continue;
      }
    }
  }
}

static void checkVersionAndBoot()
{

  uint8_t bootloaderVersion = 0;
  lhblGetVersion(&bootloaderVersion);
  DEBUG_PRINT("Lighthouse bootloader version: %d\n", bootloaderVersion);

  // Wakeup mem
  lhblFlashWakeup();
  vTaskDelay(M2T(1));

  // Checking the bitstreams are identical
  // Also decoding bitstream version for console
  static char deckBitstream[65];
  lhblFlashRead(LH_FW_ADDR, 64, (uint8_t*)deckBitstream);
  deckBitstream[64] = 0;
  int deckVersion = strtol(&deckBitstream[2], NULL, 10);
  int embeddedVersion = strtol((char*)&bitstream[2], NULL, 10);

  bool identical = true;
  for (int i=0; i<=bitstreamSize; i+=64) {
    int length = ((i+64)<bitstreamSize)?64:bitstreamSize-i;
    lhblFlashRead(LH_FW_ADDR + i, length, (uint8_t*)deckBitstream);
    if (memcmp(deckBitstream, &bitstream[i], length)) {
      DEBUG_PRINT("Fail comparing firmware\n");
      identical = false;
      break;
    }
  }

  if (identical == false || FORCE_FLASH) {
    DEBUG_PRINT("Deck has version %d and we embeed version %d\n", deckVersion, embeddedVersion);
    DEBUG_PRINT("Updating deck with embedded version!\n");

    // Erase LH deck FW
    lhblFlashEraseFirmware();

    // Flash LH deck FW
    if (lhblFlashWriteFW((uint8_t*)bitstream, bitstreamSize)) {
      DEBUG_PRINT("FW updated [OK]\n");
    } else {
      DEBUG_PRINT("FW updated [FAILED]\n");
    }
  }

  // Launch LH deck FW
  DEBUG_PRINT("Firmware version %d verified, booting deck!\n", deckVersion);
  lhblBootToFW();
}

static void lighthouseInit(DeckInfo *info)
{
  if (isInit) return;

  uart1Init(230400);
  lhblInit(I2C1_DEV);

  xTaskCreate(lighthouseTask, LIGHTHOUSE_TASK_NAME,
              2*configMINIMAL_STACK_SIZE, NULL, LIGHTHOUSE_TASK_PRI, NULL);

  isInit = true;
}


static const DeckDriver lighthouse_deck = {
  .vid = 0xBC,
  .pid = 0x10,
  .name = "bcLighthouse4",

  .usedGpio = 0,  // FIXME: set the used pins
  .requiredEstimator = kalmanEstimator,

  .init = lighthouseInit,
};


DECK_DRIVER(lighthouse_deck);

LOG_GROUP_START(lighthouse)
LOG_ADD(LOG_FLOAT, rawAngle0x, &angles.sensorMeasurements[0].baseStatonMeasurements[0].angles[0])
LOG_ADD(LOG_FLOAT, rawAngle0y, &angles.sensorMeasurements[0].baseStatonMeasurements[0].angles[1])
LOG_ADD(LOG_FLOAT, rawAngle1x, &angles.sensorMeasurements[0].baseStatonMeasurements[1].angles[0])
LOG_ADD(LOG_FLOAT, rawAngle1y, &angles.sensorMeasurements[0].baseStatonMeasurements[1].angles[1])
LOG_ADD(LOG_FLOAT, angle0x, &angles.sensorMeasurements[0].baseStatonMeasurements[0].correctedAngles[0])
LOG_ADD(LOG_FLOAT, angle0y, &angles.sensorMeasurements[0].baseStatonMeasurements[0].correctedAngles[1])
LOG_ADD(LOG_FLOAT, angle1x, &angles.sensorMeasurements[0].baseStatonMeasurements[1].correctedAngles[0])
LOG_ADD(LOG_FLOAT, angle1y, &angles.sensorMeasurements[0].baseStatonMeasurements[1].correctedAngles[1])

LOG_ADD(LOG_FLOAT, angle0x_1, &angles.sensorMeasurements[1].baseStatonMeasurements[0].correctedAngles[0])
LOG_ADD(LOG_FLOAT, angle0y_1, &angles.sensorMeasurements[1].baseStatonMeasurements[0].correctedAngles[1])
LOG_ADD(LOG_FLOAT, angle1x_1, &angles.sensorMeasurements[1].baseStatonMeasurements[1].correctedAngles[0])
LOG_ADD(LOG_FLOAT, angle1y_1, &angles.sensorMeasurements[1].baseStatonMeasurements[1].correctedAngles[1])

LOG_ADD(LOG_FLOAT, angle0x_2, &angles.sensorMeasurements[2].baseStatonMeasurements[0].correctedAngles[0])
LOG_ADD(LOG_FLOAT, angle0y_2, &angles.sensorMeasurements[2].baseStatonMeasurements[0].correctedAngles[1])
LOG_ADD(LOG_FLOAT, angle1x_2, &angles.sensorMeasurements[2].baseStatonMeasurements[1].correctedAngles[0])
LOG_ADD(LOG_FLOAT, angle1y_2, &angles.sensorMeasurements[2].baseStatonMeasurements[1].correctedAngles[1])

LOG_ADD(LOG_FLOAT, angle0x_3, &angles.sensorMeasurements[3].baseStatonMeasurements[0].correctedAngles[0])
LOG_ADD(LOG_FLOAT, angle0y_3, &angles.sensorMeasurements[3].baseStatonMeasurements[0].correctedAngles[1])
LOG_ADD(LOG_FLOAT, angle1x_3, &angles.sensorMeasurements[3].baseStatonMeasurements[1].correctedAngles[0])
LOG_ADD(LOG_FLOAT, angle1y_3, &angles.sensorMeasurements[3].baseStatonMeasurements[1].correctedAngles[1])

LOG_ADD(LOG_FLOAT, x, &position[0])
LOG_ADD(LOG_FLOAT, y, &position[1])
LOG_ADD(LOG_FLOAT, z, &position[2])

LOG_ADD(LOG_FLOAT, delta, &deltaLog)

STATS_CNT_RATE_LOG_ADD(serRt, &serialFrameRate)
STATS_CNT_RATE_LOG_ADD(frmRt, &frameRate)
STATS_CNT_RATE_LOG_ADD(cycleRt, &cycleRate)
STATS_CNT_RATE_LOG_ADD(posRt, &positionRate)
STATS_CNT_RATE_LOG_ADD(estBs0Rt, &estBs0Rate)
STATS_CNT_RATE_LOG_ADD(estBs1Rt, &estBs1Rate)

STATS_CNT_RATE_LOG_ADD(bs0Rt, &bs0Rate)
STATS_CNT_RATE_LOG_ADD(bs1Rt, &bs1Rate)


LOG_ADD(LOG_UINT16, width0, &pulseWidth[0])
#if PULSE_PROCESSOR_N_SENSORS > 1
LOG_ADD(LOG_UINT16, width1, &pulseWidth[1])
#endif
#if PULSE_PROCESSOR_N_SENSORS > 2
LOG_ADD(LOG_UINT16, width2, &pulseWidth[2])
#endif
#if PULSE_PROCESSOR_N_SENSORS > 3
LOG_ADD(LOG_UINT16, width3, &pulseWidth[3])
#endif

LOG_ADD(LOG_UINT8, comSync, &comSynchronized)
LOG_GROUP_STOP(lighthouse)

PARAM_GROUP_START(lighthouse)
PARAM_ADD(PARAM_UINT8, method, &estimationMethod)
PARAM_ADD(PARAM_FLOAT, sweepStd, &sweepStd)
PARAM_GROUP_STOP(lighthouse)

#endif // DISABLE_LIGHTHOUSE_DRIVER

PARAM_GROUP_START(deck)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bdLighthouse4, &isInit)
PARAM_GROUP_STOP(deck)
