/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 - 2020 Bitcraze AB
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
 * lighthouse_core.c - central part of the lighthouse positioning system
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
// #define CONFIG_DEBUG_LOG_ENABLE 1
#include "log.h"
#include "statsCnt.h"

#define DEBUG_MODULE "LH"
#include "debug.h"
#include "uart1.h"
#include "crtp_localization_service.h"

#include "pulse_processor.h"
#include "pulse_processor_v1.h"
#include "pulse_processor_v2.h"

#include "lighthouse_deck_flasher.h"
#include "lighthouse_position_est.h"
#include "lighthouse_core.h"

#include "lighthouse_throttle.h"

#include "lighthouse_storage.h"

#include "test_support.h"
#include "static_mem.h"

#include "lighthouse_transmit.h"

static const uint32_t MAX_WAIT_TIME_FOR_HEALTH_MS = 4000;

static pulseProcessorResult_t angles;
static lighthouseUartFrame_t frame;
static lighthouseBsIdentificationData_t bsIdentificationData;

// Stats

typedef enum {
  statusNotReceiving = 0,
  statusMissingData = 1,
  statusToEstimator = 2,
} lhSystemStatus_t;

static bool uartSynchronized = false;

#define ONE_SECOND 1000
#define HALF_SECOND 500
#define FIFTH_SECOND 200

#ifdef CONFIG_DEBUG_LOG_ENABLE
static STATS_CNT_RATE_DEFINE(serialFrameRate, ONE_SECOND);
static STATS_CNT_RATE_DEFINE(frameRate, ONE_SECOND);
static STATS_CNT_RATE_DEFINE(cycleRate, ONE_SECOND);

static STATS_CNT_RATE_DEFINE(bs0Rate, HALF_SECOND);
static STATS_CNT_RATE_DEFINE(bs1Rate, HALF_SECOND);
static statsCntRateLogger_t* bsRates[CONFIG_DECK_LIGHTHOUSE_MAX_N_BS] = {&bs0Rate, &bs1Rate};

static STATS_CNT_RATE_DEFINE(preThrottleRate, ONE_SECOND);
static STATS_CNT_RATE_DEFINE(postThrottleRate, ONE_SECOND);
#endif

// A bitmap that indicates which base stations that are available
static uint16_t baseStationAvailabledMapWs;
static uint16_t baseStationAvailabledMap;

// A bitmap that indicates which base staions that are received
static uint16_t baseStationReceivedMapWs;
static uint16_t baseStationReceivedMap;

// A bitmap that indicates which base staions that are actively used in the estimation process, that is receiving sweeps
// as well a has valid geo and calib data
static uint16_t baseStationActiveMapWs;
static uint16_t baseStationActiveMap;

// A bitmap that indicates which base stations that have received calibration data over the air
static uint16_t baseStationCalibConfirmedMap;

// A bitmap that indicates which base stations that have received calibration data that was different to what was stored in memory
static uint16_t baseStationCalibUpdatedMap;

static uint8_t calibStatusReset;

// An overall system status indicating if data is sent to the estimator
static lhSystemStatus_t systemStatus;
static lhSystemStatus_t systemStatusWs;
static lhSystemStatus_t ledInternalStatus = statusToEstimator;

static const uint32_t SYSTEM_STATUS_UPDATE_INTERVAL = FIFTH_SECOND;
static uint32_t nextUpdateTimeOfSystemStatus = 0;

static uint16_t pulseWidth[PULSE_PROCESSOR_N_SENSORS];
pulseProcessor_t lighthouseCoreState;

static lighthouseBaseStationType_t systemType = lighthouseBsTypeV2;
static lighthouseBaseStationType_t previousSystemType = lighthouseBsTypeV2;
static pulseProcessorProcessPulse_t pulseProcessorProcessPulse = pulseProcessorV2ProcessPulse;

#define UART_FRAME_LENGTH 12


static bool deckIsFlashed = false;

static void modifyBit(uint16_t *bitmap, const int index, const bool value) {
  const uint16_t mask = (1 << index);

  if (value) {
    *bitmap |= mask;
  } else {
    *bitmap &= ~mask;
  }
}

void lighthouseCoreInit() {
  lighthouseStorageInitializeSystemTypeFromStorage();
  lighthousePositionEstInit();

  for (int i = 0; i < CONFIG_DECK_LIGHTHOUSE_MAX_N_BS; i++) {
    modifyBit(&baseStationAvailabledMap, i, true);
  }
}

void lighthouseCoreLedTimer()
{
  if (deckIsFlashed){
    switch (systemStatus)
    {
      case statusNotReceiving:
        if(ledInternalStatus != systemStatus)
        {
          lighthouseCoreSetLeds(lh_led_off, lh_led_on, lh_led_off);
          ledInternalStatus = systemStatus;
        }
        break;
      case statusMissingData:
        if(ledInternalStatus != systemStatus)
        {
          lighthouseCoreSetLeds(lh_led_off, lh_led_slow_blink, lh_led_off);
          ledInternalStatus = systemStatus;
        }
        break;
      case statusToEstimator:
        if(ledInternalStatus != systemStatus)
        {
          lighthouseCoreSetLeds(lh_led_off, lh_led_off, lh_led_on);
          ledInternalStatus = systemStatus;
        }
        break;
      default:
        ASSERT(false);
    }
  }
}

static void lighthouseUpdateSystemType() {
  // Switch to new pulse processor
  switch (systemType)
  {
  case lighthouseBsTypeV1:
    pulseProcessorProcessPulse = pulseProcessorV1ProcessPulse;
    baseStationAvailabledMapWs = 3;

    break;
  case lighthouseBsTypeV2:
    pulseProcessorProcessPulse = pulseProcessorV2ProcessPulse;
    for (int i = 0; i < CONFIG_DECK_LIGHTHOUSE_MAX_N_BS; i++)
    {
      modifyBit(&baseStationAvailabledMapWs, i, true);
    }
    break;
  default:
    // Do nothing if the type is not in range, stay on the previous processor
    return;
  }

  if (previousSystemType != systemType) {
    previousSystemType = systemType;

    // Clear state
    memset(&lighthouseCoreState, 0, sizeof(lighthouseCoreState));

    // Store new system type
    lighthouseStoragePersistSystemType(systemType);
  }

}

void lighthouseCoreSetSystemType(const lighthouseBaseStationType_t type)
{
  systemType = type;
  previousSystemType = type;
  lighthouseUpdateSystemType();
}

TESTABLE_STATIC bool getUartFrameRaw(lighthouseUartFrame_t *frame) {
  static char data[UART_FRAME_LENGTH];
  int syncCounter = 0;

  for(int i = 0; i < UART_FRAME_LENGTH; i++) {
    while(!uart1GetDataWithTimeout((uint8_t*)&data[i], 2)) {
      lighthouseTransmitProcessTimeout();
    }
    if ((unsigned char)data[i] == 0xff) {
      syncCounter += 1;
    }
  }

  memset(frame, 0, sizeof(*frame));

  frame->isSyncFrame = (syncCounter == UART_FRAME_LENGTH);

  frame->data.sensor = data[0] & 0x03;
  frame->data.channelFound = (data[0] & 0x80) == 0;
  frame->data.channel = (data[0] >> 3) & 0x0f;
  frame->data.slowBit = (data[0] >> 2) & 0x01;
  memcpy(&frame->data.width, &data[1], 2);
  memcpy(&frame->data.offset, &data[3], 3);
  memcpy(&frame->data.beamData, &data[6], 3);
  memcpy(&frame->data.timestamp, &data[9], 3);

  // Offset is expressed in a 6 MHz clock, convert to the 24 MHz that is used for timestamps
  frame->data.offset *= 4;

  bool isPaddingZero = (((data[5] | data[8]) & 0xfe) == 0);
  bool isFrameValid = (isPaddingZero || frame->isSyncFrame);

  STATS_CNT_RATE_EVENT_DEBUG(&serialFrameRate);

  return isFrameValid;
}

TESTABLE_STATIC void waitForUartSynchFrame() {
  char c;
  int syncCounter = 0;
  bool synchronized = false;

  while (!synchronized) {
    uart1Getchar(&c);
    if ((unsigned char)c == 0xff) {
      syncCounter += 1;
    } else {
      syncCounter = 0;
    }
    synchronized = (syncCounter == UART_FRAME_LENGTH);
  }
}

void lighthouseCoreSetLeds(lighthouseCoreLedState_t red, lighthouseCoreLedState_t orange, lighthouseCoreLedState_t green)
{
  uint8_t commandBuffer[2];

  commandBuffer[0] = 0x01;
  commandBuffer[1] = (green<<4) | (orange<<2) | red;

  uart1SendData(2, commandBuffer);
}

bool findOtherBaseStation(const pulseProcessorResult_t* angles, const int baseStation, int* otherBaseStation) {
  for (int candidate = baseStation + 1; candidate != baseStation; candidate++) {
    if (candidate >= CONFIG_DECK_LIGHTHOUSE_MAX_N_BS) {
      candidate = 0;
    }

    // Only looking at sensor 0, assuming this is enough
    if (angles->baseStationMeasurementsLh1[candidate].sensorMeasurements[0].validCount == PULSE_PROCESSOR_N_SWEEPS) {
      *otherBaseStation = candidate;
      return true;
    }
  }

  return false;
}


// Method used to estimate position
// 0 = Position calculated outside the estimator using intersection point of beams.
//     Yaw error calculated outside the estimator. Position and yaw error is pushed to the
//     estimator as pre-calculated.
// 1 = Sweep angles pushed into the estimator. Yaw error calculated outside the estimator
//     and pushed to the estimator as a pre-calculated value.
#ifdef CONFIG_DECK_LIGHTHOUSE_AS_GROUNDTRUTH
static uint8_t estimationMethod = 0;
#else
static uint8_t estimationMethod = 1;
#endif


static void usePulseResultCrossingBeams(pulseProcessor_t *appState, pulseProcessorResult_t* angles, int baseStation) {
  pulseProcessorClearOutdated(appState, angles, baseStation);

  int otherBaseStation = 0;
  bool foundPair = false;

  switch (systemType) {
    case lighthouseBsTypeV1:
      if (baseStation == 1) {
        otherBaseStation = 0;
        foundPair = true;
      }
      break;
    case lighthouseBsTypeV2:
      foundPair = findOtherBaseStation(angles, baseStation, &otherBaseStation);
      break;
    default:
      // Nothing here
      break;
  }

  if (foundPair) {
    STATS_CNT_RATE_EVENT_DEBUG(&cycleRate);

    lighthousePositionEstimatePoseCrossingBeams(appState, angles, baseStation, otherBaseStation);

    pulseProcessorProcessed(angles, baseStation);
    pulseProcessorProcessed(angles, otherBaseStation);
  }
}

static void usePulseResultSweeps(pulseProcessor_t *appState, pulseProcessorResult_t* angles, int baseStation) {
  STATS_CNT_RATE_EVENT_DEBUG(&cycleRate);

  pulseProcessorClearOutdated(appState, angles, baseStation);

  lighthousePositionEstimatePoseSweeps(appState, angles, baseStation);

  pulseProcessorProcessed(angles, baseStation);
}

static void convertV2AnglesToV1Angles(pulseProcessorResult_t* angles) {
  for (int sensor = 0; sensor < PULSE_PROCESSOR_N_SENSORS; sensor++) {
    for (int bs = 0; bs < CONFIG_DECK_LIGHTHOUSE_MAX_N_BS; bs++) {
      pulseProcessorSensorMeasurement_t* from = &angles->baseStationMeasurementsLh2[bs].sensorMeasurements[sensor];
      pulseProcessorSensorMeasurement_t* to = &angles->baseStationMeasurementsLh1[bs].sensorMeasurements[sensor];

      if (2 == from->validCount) {
        pulseProcessorV2ConvertToV1Angles(from->correctedAngles[0], from->correctedAngles[1], to->correctedAngles);
        to->validCount = from->validCount;
      } else {
        to->validCount = 0;
      }
    }
  }
}

static void usePulseResult(pulseProcessor_t *appState, pulseProcessorResult_t* angles, int baseStation, int sweepId, const uint32_t now_ms) {
  const uint16_t baseStationBitMap = (1 << baseStation);
  baseStationReceivedMapWs |= baseStationBitMap;

  if (sweepId == sweepIdSecond) {
    const bool hasCalibrationData = pulseProcessorApplyCalibration(appState, angles, baseStation);
    if (hasCalibrationData) {
      if (lighthouseBsTypeV2 == angles->measurementType) {
        // Emulate V1 base stations, convert to V1 angles
        convertV2AnglesToV1Angles(angles);
      }

      // Send measurement to the ground
      locSrvSendLighthouseAngle(baseStation, angles);

      const bool hasGeoData = appState->bsGeometry[baseStation].valid;
      if (hasGeoData) {
        baseStationActiveMapWs |= baseStationBitMap;

        STATS_CNT_RATE_EVENT_DEBUG(&preThrottleRate);
        bool useSample = true;
        if (lighthouseBsTypeV2 == angles->measurementType) {
          useSample = throttleLh2Samples(now_ms);
        }

        if (useSample) {
          switch(estimationMethod) {
            case 0:
              usePulseResultCrossingBeams(appState, angles, baseStation);
              break;
            case 1:
              usePulseResultSweeps(appState, angles, baseStation);
              break;
            default:
              break;
          }

          STATS_CNT_RATE_EVENT_DEBUG(&postThrottleRate);
        }
      }
    }

    if (baseStationActiveMapWs != 0) {
      systemStatusWs = statusToEstimator;
    } else {
      systemStatusWs = statusMissingData;
    }
  }
}

static void useCalibrationData(pulseProcessor_t *appState) {
  for (int baseStation = 0; baseStation < CONFIG_DECK_LIGHTHOUSE_MAX_N_BS; baseStation++) {
    if (appState->ootxDecoder[baseStation].isFullyDecoded) {
      lighthouseCalibration_t newData;
      lighthouseCalibrationInitFromFrame(&newData, &appState->ootxDecoder[baseStation].frame);

      modifyBit(&baseStationCalibConfirmedMap, baseStation, true);

      lighthouseCalibration_t* currentCalibData = &appState->bsCalibration[baseStation];
      const bool currentCalibDataValid = currentCalibData->valid;
      const bool isDataDifferent = ((newData.uid != currentCalibData->uid) || (newData.valid != currentCalibDataValid));
      if (isDataDifferent) {
        DEBUG_PRINT("Got calibration from %08X on channel %d\n", (unsigned int)appState->ootxDecoder[baseStation].frame.id, baseStation + 1);
        lighthouseCoreSetCalibrationData(baseStation, &newData);
        lighthouseStoragePersistCalibDataBackground(baseStation);

        modifyBit(&baseStationCalibUpdatedMap, baseStation, currentCalibDataValid);
      }
    }
  }
}

static void processFrame(pulseProcessor_t *appState, pulseProcessorResult_t* angles, const lighthouseUartFrame_t* frame, const uint32_t now_ms) {
    int baseStation;
    int sweepId;
    bool calibDataIsDecoded = false;

    pulseWidth[frame->data.sensor] = frame->data.width;

    if (pulseProcessorProcessPulse(appState, &frame->data, angles, &baseStation, &sweepId, &calibDataIsDecoded)) {
        STATS_CNT_RATE_EVENT_DEBUG(bsRates[baseStation]);
        usePulseResult(appState, angles, baseStation, sweepId, now_ms);
    }

    if (calibDataIsDecoded) {
      useCalibrationData(appState);
    }
}

static void deckHealthCheck(pulseProcessor_t *appState, const lighthouseUartFrame_t* frame, const uint32_t now_ms) {
  if (!appState->healthDetermined) {
    if (0 == appState->healthFirstSensorTs) {
      appState->healthFirstSensorTs = now_ms;
    }

    if (0x0f == appState->healthSensorBitField) {
      appState->healthDetermined = true;
      // DEBUG_PRINT("All sensors good\n");
    } else {
      appState->healthSensorBitField |= (0x01 << frame->data.sensor);

      if ((now_ms - appState->healthFirstSensorTs) > MAX_WAIT_TIME_FOR_HEALTH_MS) {
        appState->healthDetermined = true;
        DEBUG_PRINT("Warning: not getting data from all sensors\n");
        for (int i = 0; i < PULSE_PROCESSOR_N_SENSORS; i++) {
          if (appState->healthSensorBitField & (0x1 << i)) {
            DEBUG_PRINT("  %d - OK\n", i);
          } else {
            DEBUG_PRINT("  %d - error\n", i);
          }
        }
      }
    }
  }
}

static void updateSystemStatus(const uint32_t now_ms) {
  if (now_ms > nextUpdateTimeOfSystemStatus) {
    baseStationAvailabledMap = baseStationAvailabledMapWs;

    baseStationReceivedMap = baseStationReceivedMapWs;
    baseStationReceivedMapWs = 0;

    baseStationActiveMap = baseStationActiveMapWs;
    baseStationActiveMapWs = 0;

    systemStatus = systemStatusWs;
    systemStatusWs = statusNotReceiving;

    if (calibStatusReset) {
      calibStatusReset = 0;
      baseStationCalibUpdatedMap = 0;
    }

    nextUpdateTimeOfSystemStatus = now_ms + SYSTEM_STATUS_UPDATE_INTERVAL;
  }
}

void lighthouseCoreTask(void *param) {
  bool isUartFrameValid = false;

  uart1Init(230400);
  systemWaitStart();

  lighthouseStorageVerifySetStorageVersion();
  lighthouseStorageInitializeGeoDataFromStorage();
  lighthouseStorageInitializeCalibDataFromStorage();

  if (lighthouseDeckFlasherCheckVersionAndBoot() == false) {
    DEBUG_PRINT("FPGA not booted. Lighthouse disabled!\n");
    while(1) {
      vTaskDelay(portMAX_DELAY);
    }
  }
  deckIsFlashed = true;


  vTaskDelay(M2T(100));

  memset(&bsIdentificationData, 0, sizeof(bsIdentificationData));

  while(1) {
    memset(pulseWidth, 0, sizeof(pulseWidth[0]) * PULSE_PROCESSOR_N_SENSORS);
    waitForUartSynchFrame();
    uartSynchronized = true;

    bool previousWasSyncFrame = false;

    while((isUartFrameValid = getUartFrameRaw(&frame))) {
      const uint32_t now_ms = T2M(xTaskGetTickCount());

      // If a sync frame is getting through, we are only receiving sync frames. So nothing else. Reset state
      if(frame.isSyncFrame && previousWasSyncFrame) {
          pulseProcessorAllClear(&angles);
      }
      // Now we are receiving items
      else if(!frame.isSyncFrame) {
        STATS_CNT_RATE_EVENT_DEBUG(&frameRate);
	lighthouseTransmitProcessFrame(&frame);

        deckHealthCheck(&lighthouseCoreState, &frame, now_ms);
        lighthouseUpdateSystemType();
        if (pulseProcessorProcessPulse) {
          processFrame(&lighthouseCoreState, &angles, &frame, now_ms);
        }
      }

      previousWasSyncFrame = frame.isSyncFrame;

      updateSystemStatus(now_ms);
    }

    uartSynchronized = false;
  }
}

void lighthouseCoreSetCalibrationData(const uint8_t baseStation, const lighthouseCalibration_t* calibration) {
  if (baseStation < CONFIG_DECK_LIGHTHOUSE_MAX_N_BS) {
    lighthouseCoreState.bsCalibration[baseStation] = *calibration;
    lighthousePositionCalibrationDataWritten(baseStation);
  }
}

static uint8_t pulseProcessorAnglesQualityLogger(uint32_t timestamp, void* ignored) {
  return pulseProcessorAnglesQuality();
}
static logByFunction_t pulseProcessorAnglesQualityLoggerDef = {.acquireUInt8 = pulseProcessorAnglesQualityLogger, .data = 0};

/**
 * Log group for the lighthouse positioning system
 */
LOG_GROUP_START(lighthouse)
LOG_ADD_BY_FUNCTION(LOG_UINT8, validAngles, &pulseProcessorAnglesQualityLoggerDef)

/**
 * @brief The raw V1 angle received by sensor 0 [rad]
 *
 * | Base station type | V1 |\n
 * | Base station | primary |\n
 * | Sweep | 1 |\n
 * | Sensor | 0 |\n
 */
LOG_ADD(LOG_FLOAT, rawAngle0x, &angles.baseStationMeasurementsLh1[0].sensorMeasurements[0].angles[0])

/**
 * @brief The raw V1 angle received by sensor 0 [rad]
 *
 * | Base station type | V1 |\n
 * | Base station | primary |\n
 * | Sweep | 2 |\n
 * | Sensor | 0 |\n
 */
LOG_ADD(LOG_FLOAT, rawAngle0y, &angles.baseStationMeasurementsLh1[0].sensorMeasurements[0].angles[1])

/**
 * @brief The raw V1 angle received by sensor 0 [rad]
 *
 * | Base station type | V1 |\n
 * | Base station | secondary |\n
 * | Sweep | 1 |\n
 * | Sensor | 0 |\n
 */
LOG_ADD(LOG_FLOAT, rawAngle1x, &angles.baseStationMeasurementsLh1[1].sensorMeasurements[0].angles[0])

/**
 * @brief The raw V1 angle received by sensor 0 [rad]
 *
 * | Base station type | V1 |\n
 * | Base station | secondary |\n
 * | Sweep | 2 |\n
 * | Sensor | 0 |\n
 */
LOG_ADD(LOG_FLOAT, rawAngle1y, &angles.baseStationMeasurementsLh1[1].sensorMeasurements[0].angles[1])

/**
 * @brief The V1 angle received by sensor 0, corrected using calibration data [rad]
 *
 * | Base station type | V1 |\n
 * | Base station | primary |\n
 * | Sweep | 1 |\n
 * | Sensor | 0 |\n\n
 *
 * If a base station of type V2 is used, this will contain the V2 angles converted to V1 style for the base station with channel 1.
 */
LOG_ADD(LOG_FLOAT, angle0x, &angles.baseStationMeasurementsLh1[0].sensorMeasurements[0].correctedAngles[0])

/**
 * @brief The V1 angle received by sensor 0, corrected using calibration data [rad]
 *
 * | Base station type | V1 |\n
 * | Base station | primary |\n
 * | Sweep | 2 |\n
 * | Sensor | 0 |\n\n
 *
 * If a base station of type V2 is used, this will contain the V2 angles converted to V1 style for the base station with channel 1.
 */
LOG_ADD(LOG_FLOAT, angle0y, &angles.baseStationMeasurementsLh1[0].sensorMeasurements[0].correctedAngles[1])

/**
 * @brief The angle received by sensor 0, corrected using calibration data [rad]
 *
 * | Base station type | V1 |\n
 * | Base station | secondary |\n
 * | Sweep | 1 |\n
 * | Sensor | 0 |\n\n
 *
 * If a base station of type V2 is used, this will contain the V2 angles converted to V1 style for the base station with channel 2.
 */
LOG_ADD(LOG_FLOAT, angle1x, &angles.baseStationMeasurementsLh1[1].sensorMeasurements[0].correctedAngles[0])

/**
 * @brief The angle received by sensor 0, corrected using calibration data [rad]
 *
 * | Base station type | V1 |\n
 * | Base station | secondary |\n
 * | Sweep | 2 |\n
 * | Sensor | 0 |\n\n
 *
 * If a base station of type V2 is used, this will contain the V2 angles converted to V1 style for the base station with channel 2.
 */
LOG_ADD(LOG_FLOAT, angle1y, &angles.baseStationMeasurementsLh1[1].sensorMeasurements[0].correctedAngles[1])

/**
 * @brief The angle received by sensor 1, corrected using calibration data [rad]
 *
 * | Base station type | V1 |\n
 * | Base station | primary |\n
 * | Sweep | 1 |\n
 * | Sensor | 1 |\n\n
 *
 * If a base station of type V2 is used, this will contain the V2 angles converted to V1 style for the base station with channel 1.
 */
LOG_ADD_DEBUG(LOG_FLOAT, angle0x_1, &angles.baseStationMeasurementsLh1[0].sensorMeasurements[1].correctedAngles[0])

/**
 * @brief The V1 angle received by sensor 1, corrected using calibration data [rad]
 *
 * | Base station type | V1 |\n
 * | Base station | primary |\n
 * | Sweep | 2 |\n
 * | Sensor | 1 |\n\n
 *
 * If a base station of type V2 is used, this will contain the V2 angles converted to V1 style for the base station with channel 1.
 */
LOG_ADD_DEBUG(LOG_FLOAT, angle0y_1, &angles.baseStationMeasurementsLh1[0].sensorMeasurements[1].correctedAngles[1])

/**
 * @brief The V1 angle received by sensor 1, corrected using calibration data [rad]
 *
 * | Base station type | V1 |\n
 * | Base station | secondary |\n
 * | Sweep | 1 |\n
 * | Sensor | 1 |\n\n
 *
 * If a base station of type V2 is used, this will contain the V2 angles converted to V1 style for the base station with channel 2.
 */
LOG_ADD_DEBUG(LOG_FLOAT, angle1x_1, &angles.baseStationMeasurementsLh1[1].sensorMeasurements[1].correctedAngles[0])

/**
 * @brief The V1 angle received by sensor 1, corrected using calibration data [rad]
 *
 * | Base station type | V1 |\n
 * | Base station | secondary |\n
 * | Sweep | 2 |\n
 * | Sensor | 1 |\n\n
 *
 * If a base station of type V2 is used, this will contain the V2 angles converted to V1 style for the base station with channel 2.
 */
LOG_ADD_DEBUG(LOG_FLOAT, angle1y_1, &angles.baseStationMeasurementsLh1[1].sensorMeasurements[1].correctedAngles[1])

LOG_ADD_DEBUG(LOG_FLOAT, angle0x_2, &angles.baseStationMeasurementsLh1[0].sensorMeasurements[2].correctedAngles[0])
LOG_ADD_DEBUG(LOG_FLOAT, angle0y_2, &angles.baseStationMeasurementsLh1[0].sensorMeasurements[2].correctedAngles[1])
LOG_ADD_DEBUG(LOG_FLOAT, angle1x_2, &angles.baseStationMeasurementsLh1[1].sensorMeasurements[2].correctedAngles[0])
LOG_ADD_DEBUG(LOG_FLOAT, angle1y_2, &angles.baseStationMeasurementsLh1[1].sensorMeasurements[2].correctedAngles[1])

LOG_ADD_DEBUG(LOG_FLOAT, angle0x_3, &angles.baseStationMeasurementsLh1[0].sensorMeasurements[3].correctedAngles[0])
LOG_ADD_DEBUG(LOG_FLOAT, angle0y_3, &angles.baseStationMeasurementsLh1[0].sensorMeasurements[3].correctedAngles[1])
LOG_ADD_DEBUG(LOG_FLOAT, angle1x_3, &angles.baseStationMeasurementsLh1[1].sensorMeasurements[3].correctedAngles[0])
LOG_ADD_DEBUG(LOG_FLOAT, angle1y_3, &angles.baseStationMeasurementsLh1[1].sensorMeasurements[3].correctedAngles[1])

/**
 * @brief The raw V2 angle received by sensor 0 [rad]
 *
 * | Base station type | V2 |\n
 * | Channel | 1 |\n
 * | Sweep | 1 |\n
 * | Sensor | 0 |\n
 */
LOG_ADD(LOG_FLOAT, rawAngle0xlh2, &angles.baseStationMeasurementsLh2[0].sensorMeasurements[0].angles[0])

/**
 * @brief The raw V2 angle received by sensor 0 [rad]
 *
 * | Base station type | V2 |\n
 * | Channel | 1 |\n
 * | Sweep | 2 |\n
 * | Sensor | 0 |\n
 */
LOG_ADD(LOG_FLOAT, rawAngle0ylh2, &angles.baseStationMeasurementsLh2[0].sensorMeasurements[0].angles[1])

/**
 * @brief The raw V2 angle received by sensor 0 [rad]
 *
 * | Base station type | V2 |\n
 * | Channel | 2 |\n
 * | Sweep | 1 |\n
 * | Sensor | 0 |\n
 */
LOG_ADD(LOG_FLOAT, rawAngle1xlh2, &angles.baseStationMeasurementsLh2[1].sensorMeasurements[0].angles[0])

/**
 * @brief The raw V2 angle received by sensor 0 [rad]
 *
 * | Base station type | V2 |\n
 * | Channel | 2 |\n
 * | Sweep | 2 |\n
 * | Sensor | 0 |\n
 */
LOG_ADD(LOG_FLOAT, rawAngle1ylh2, &angles.baseStationMeasurementsLh2[1].sensorMeasurements[0].angles[1])

/**
 * @brief The V2 angle received by sensor 0, corrected using calibration data [rad]
 *
 * | Base station type | V2 |\n
 * | Channel | 1 |\n
 * | Sweep | 1 |\n
 * | Sensor | 0 |\n
 */
LOG_ADD(LOG_FLOAT, angle0x_0lh2, &angles.baseStationMeasurementsLh2[0].sensorMeasurements[0].correctedAngles[0])

/**
 * @brief The V2 angle received by sensor 0, corrected using calibration data [rad]
 *
 * | Base station type | V2 |\n
 * | Channel | 1 |\n
 * | Sweep | 2 |\n
 * | Sensor | 0 |\n
 */
LOG_ADD(LOG_FLOAT, angle0y_0lh2, &angles.baseStationMeasurementsLh2[0].sensorMeasurements[0].correctedAngles[1])

/**
 * @brief The V2 angle received by sensor 0, corrected using calibration data [rad]
 *
 * | Base station type | V2 |\n
 * | Channel | 2 |\n
 * | Sweep | 1 |\n
 * | Sensor | 0 |\n
 */
LOG_ADD(LOG_FLOAT, angle1x_0lh2, &angles.baseStationMeasurementsLh2[1].sensorMeasurements[0].correctedAngles[0])

/**
 * @brief The V2 angle received by sensor 0, corrected using calibration data [rad]
 *
 * | Base station type | V2 |\n
 * | Channel | 2 |\n
 * | Sweep | 2 |\n
 * | Sensor | 0 |\n
 */
LOG_ADD(LOG_FLOAT, angle1y_0lh2, &angles.baseStationMeasurementsLh2[1].sensorMeasurements[0].correctedAngles[1])

/**
 * @brief Rate of frames from the Lighthouse deck on the serial buss [1/s]
 */
STATS_CNT_RATE_LOG_ADD_DEBUG(serRt, &serialFrameRate)

/**
 * @brief Rate of frames from the Lighthouse deck that contains sweep data [1/s]
 */
STATS_CNT_RATE_LOG_ADD_DEBUG(frmRt, &frameRate)

STATS_CNT_RATE_LOG_ADD_DEBUG(cycleRt, &cycleRate)

STATS_CNT_RATE_LOG_ADD_DEBUG(bs0Rt, &bs0Rate)
STATS_CNT_RATE_LOG_ADD_DEBUG(bs1Rt, &bs1Rate)

STATS_CNT_RATE_LOG_ADD_DEBUG(preThRt, &preThrottleRate)
STATS_CNT_RATE_LOG_ADD_DEBUG(postThRt, &postThrottleRate)

LOG_ADD_DEBUG(LOG_UINT16, width0, &pulseWidth[0])
LOG_ADD_DEBUG(LOG_UINT16, width1, &pulseWidth[1])
LOG_ADD_DEBUG(LOG_UINT16, width2, &pulseWidth[2])
LOG_ADD_DEBUG(LOG_UINT16, width3, &pulseWidth[3])

LOG_ADD(LOG_UINT8, comSync, &uartSynchronized)

/**
 * @brief Bit field indicating which base stations that are available
 *
 * The lowest bit maps to base station channel 1 and the highest to channel 16.
 */
LOG_ADD_CORE(LOG_UINT16, bsAvailable, &baseStationAvailabledMap)

/**
 * @brief Bit field indicating which base stations that are received by the lighthouse deck
 *
 * The lowest bit maps to base station channel 1 and the highest to channel 16.
 */
LOG_ADD_CORE(LOG_UINT16, bsReceive, &baseStationReceivedMap)

/**
 * @brief Bit field indicating which base stations that are providing useful data to the estimator.
 *
 * A bit will be set if there is calibration and geometry data for the base station, and sweeps are received.
 *
 * The lowest bit maps to base station channel 1 and the highest to channel 16.
 */
LOG_ADD_CORE(LOG_UINT16, bsActive, &baseStationActiveMap)

/**
 * @brief Bit field that indicates which base stations that have received calibration data that was different to what was stored in memory
 *
 * The lowest bit maps to base station channel 1 and the highest to channel 16.
 */
LOG_ADD_CORE(LOG_UINT16, bsCalUd, &baseStationCalibUpdatedMap)

/**
 * @brief Bit field that indicates which base stations that have received calibration data over the air
 *
 * The lowest bit maps to base station channel 1 and the highest to channel 16.
 */
LOG_ADD_CORE(LOG_UINT16, bsCalCon, &baseStationCalibConfirmedMap)

/**
 * @brief Overall status of the lighthouse system
 *
 * | Value | Meaning                                                                             |\n
 * | -     | -                                                                                   |\n
 * | 0     | No lighthouse base stations are recevied                                            |\n
 * | 1     | One or more base stations are received but geometry or callibration data is missing |\n
 * | 2     | Base station data is sent to the state estimator                                    |\n
 *
 */
LOG_ADD_CORE(LOG_UINT8, status, &systemStatus)
LOG_GROUP_STOP(lighthouse)

/**
 * Parameters and settings for the Lighthouse positioning system
 */
PARAM_GROUP_START(lighthouse)
/**
 * @brief Estimation Method: 0:CrossingBeam,  1:Sweep in EKF (default: 1)
 */
PARAM_ADD_CORE(PARAM_UINT8, method, &estimationMethod)
/**
 * @brief Reset calibration data status
 */
PARAM_ADD_CORE(PARAM_UINT8, bsCalibReset, &calibStatusReset)
/**
 * @brief Lighthouse baseStation version: 1: LighthouseV1, 2:LighthouseV2
 *  (default: 2)
 */
PARAM_ADD_CORE(PARAM_UINT8, systemType, &systemType)

/**
 * @brief Bit field that indicates which base stations that are supported by the system - deprecated (removed after August 2023)
 *
 * The lowest bit maps to base station channel 1 and the highest to channel 16.
 */
PARAM_ADD_CORE(PARAM_UINT16 | PARAM_RONLY, bsAvailable, &baseStationAvailabledMap)

PARAM_GROUP_STOP(lighthouse)
