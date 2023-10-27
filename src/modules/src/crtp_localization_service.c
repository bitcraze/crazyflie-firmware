/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
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
 *
 */
#include <string.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "crtp.h"
#include "crtp_localization_service.h"
#include "log.h"
#include "param.h"

#include "stabilizer_types.h"
#include "stabilizer.h"
#include "configblock.h"
#include "worker.h"
#include "autoconf.h"

#ifdef CONFIG_DECK_LIGHTHOUSE
#include "lighthouse_storage.h"
#endif

#include "locodeck.h"

#include "estimator.h"
#include "quatcompress.h"

#include "peer_localization.h"

#include "num.h"


#define NBR_OF_RANGES_IN_PACKET   5
#define NBR_OF_SWEEPS_IN_PACKET   2
#define NBR_OF_SENSOR_DIFFS_IN_PACKET   3
#define NBR_OF_BASESTATIONS   2
#define NBR_OF_

typedef enum
{
  EXT_POSITION        = 0,
  GENERIC_TYPE        = 1,
  EXT_POSITION_PACKED = 2,
} locsrvChannels_t;

typedef struct
{
  uint8_t type;
  struct
  {
    uint8_t id;
    float range;
  } __attribute__((packed)) ranges[NBR_OF_RANGES_IN_PACKET];
} __attribute__((packed)) rangePacket;

typedef struct {
  uint8_t type;
  uint8_t baseStation;
  struct {
  float sweep;
    struct {
      uint16_t angleDiff;
    } __attribute__((packed)) angleDiffs [NBR_OF_SENSOR_DIFFS_IN_PACKET];
  } __attribute__((packed)) sweeps [NBR_OF_SWEEPS_IN_PACKET];
} __attribute__((packed)) anglePacket;

// up to 4 items per CRTP packet
typedef struct {
  uint8_t id; // last 8 bit of the Crazyflie address
  int16_t x; // mm
  int16_t y; // mm
  int16_t z; // mm
} __attribute__((packed)) extPositionPackedItem;

// up to 2 items per CRTP packet
typedef struct {
  uint8_t id; // last 8 bit of the Crazyflie address
  int16_t x; // mm
  int16_t y; // mm
  int16_t z; // mm
  uint32_t quat; // compressed quaternion, see quatcompress.h
} __attribute__((packed)) extPosePackedItem;

// Struct for logging position information
static positionMeasurement_t ext_pos;
// Struct for logging pose information
static poseMeasurement_t ext_pose;

static CRTPPacket pkRange;
static uint8_t rangeIndex;
static bool enableRangeStreamFloat = false;

#ifdef CONFIG_DECK_LIGHTHOUSE
static CRTPPacket LhAngle;
#endif
static bool enableLighthouseAngleStream = false;
static float extPosStdDev = 0.01;
static float extQuatStdDev = 4.5e-3;
static bool isInit = false;
static uint8_t my_id;
static uint16_t tickOfLastPacket; // tick when last packet was received

static void locSrvCrtpCB(CRTPPacket* pk);
static void extPositionHandler(CRTPPacket* pk);
static void genericLocHandle(CRTPPacket* pk);
static void extPositionPackedHandler(CRTPPacket* pk);

static bool isEmergencyStopRequested = false;
static uint32_t emergencyStopWatchdogNotificationTick = 0;

void locSrvInit()
{
  if (isInit) {
    return;
  }

  uint64_t address = configblockGetRadioAddress();
  my_id = address & 0xFF;

  crtpRegisterPortCB(CRTP_PORT_LOCALIZATION, locSrvCrtpCB);
  isInit = true;
}

static void locSrvCrtpCB(CRTPPacket* pk)
{
  switch (pk->channel)
  {
    case EXT_POSITION:
      extPositionHandler(pk);
      break;
    case GENERIC_TYPE:
      genericLocHandle(pk);
      break;
    case EXT_POSITION_PACKED:
      extPositionPackedHandler(pk);
      break;
    default:
      break;
  }
}

static void updateLogFromExtPos()
{
  ext_pose.x = ext_pos.x;
  ext_pose.y = ext_pos.y;
  ext_pose.z = ext_pos.z;
}

static void extPositionHandler(CRTPPacket* pk) {
  const struct CrtpExtPosition* data = (const struct CrtpExtPosition*)pk->data;

  ext_pos.x = data->x;
  ext_pos.y = data->y;
  ext_pos.z = data->z;
  ext_pos.stdDev = extPosStdDev;
  ext_pos.source = MeasurementSourceLocationService;
  updateLogFromExtPos();

  estimatorEnqueuePosition(&ext_pos);
  tickOfLastPacket = xTaskGetTickCount();
}

static void extPoseHandler(const CRTPPacket* pk) {
  const struct CrtpExtPose* data = (const struct CrtpExtPose*)&pk->data[1];

  ext_pose.x = data->x;
  ext_pose.y = data->y;
  ext_pose.z = data->z;
  ext_pose.quat.x = data->qx;
  ext_pose.quat.y = data->qy;
  ext_pose.quat.z = data->qz;
  ext_pose.quat.w = data->qw;
  ext_pose.stdDevPos = extPosStdDev;
  ext_pose.stdDevQuat = extQuatStdDev;

  estimatorEnqueuePose(&ext_pose);
  tickOfLastPacket = xTaskGetTickCount();
}

static void extPosePackedHandler(const CRTPPacket* pk) {
  uint8_t numItems = (pk->size - 1) / sizeof(extPosePackedItem);
  for (uint8_t i = 0; i < numItems; ++i) {
    const extPosePackedItem* item = (const extPosePackedItem*)&pk->data[1 + i * sizeof(extPosePackedItem)];
    if (item->id == my_id) {
      ext_pose.x = item->x / 1000.0f;
      ext_pose.y = item->y / 1000.0f;
      ext_pose.z = item->z / 1000.0f;
      quatdecompress(item->quat, (float *)&ext_pose.quat.q0);
      ext_pose.stdDevPos = extPosStdDev;
      ext_pose.stdDevQuat = extQuatStdDev;
      estimatorEnqueuePose(&ext_pose);
      tickOfLastPacket = xTaskGetTickCount();
    } else {
      ext_pos.x = item->x / 1000.0f;
      ext_pos.y = item->y / 1000.0f;
      ext_pos.z = item->z / 1000.0f;
      ext_pos.stdDev = extPosStdDev;
      peerLocalizationTellPosition(item->id, &ext_pos);
    }
  }
}

static void lpsShortLppPacketHandler(CRTPPacket* pk) {
  if (pk->size >= 2) {
#ifdef CONFIG_DECK_LOCO
    bool success = lpsSendLppShort(pk->data[1], &pk->data[2], pk->size-2);
#else
    bool success = false;
#endif
    pk->port = CRTP_PORT_LOCALIZATION;
    pk->channel = GENERIC_TYPE;
    pk->size = 3;
    pk->data[0] = LPS_SHORT_LPP_PACKET;
    pk->data[2] = success?1:0;
    // This is best effort, i.e. the blocking version is not needed
    crtpSendPacket(pk);
  }
}

typedef union {
  struct {
    // A bit field indicating for which base stations to store geometry data
    uint16_t geoDataBsField;
    // A bit field indicating for which base stations to store calibration data
    uint16_t calibrationDataBsField;
  } __attribute__((packed));
  uint32_t combinedField;
} __attribute__((packed)) LhPersistArgs_t;

static void lhPersistDataWorker(void* arg) {
#ifdef CONFIG_DECK_LIGHTHOUSE
  LhPersistArgs_t* args = (LhPersistArgs_t*) &arg;

  bool result = true;

  for (int baseStation = 0; baseStation < CONFIG_DECK_LIGHTHOUSE_MAX_N_BS; baseStation++) {
    uint16_t mask = 1 << baseStation;
    bool storeGeo = (args->geoDataBsField & mask) != 0;
    bool storeCalibration = (args->calibrationDataBsField & mask) != 0;
    if (! lighthouseStoragePersistData(baseStation, storeGeo, storeCalibration)) {
      result = false;
      break;
    }
  }
#else
  bool result = false;
#endif
  CRTPPacket response = {
    .port = CRTP_PORT_LOCALIZATION,
    .channel = GENERIC_TYPE,
    .size = 2,
    .data = {LH_PERSIST_DATA, result}
  };

  crtpSendPacketBlock(&response);
}

static void lhPersistDataHandler(CRTPPacket* pk) {
  if (pk->size >= (1 + sizeof(LhPersistArgs_t))) {
    LhPersistArgs_t* args = (LhPersistArgs_t*) &pk->data[1];
    workerSchedule(lhPersistDataWorker, (void*)args->combinedField);
  }
}

static void genericLocHandle(CRTPPacket* pk)
{
  const uint8_t type = pk->data[0];
  if (pk->size < 1) return;

  switch (type) {
    case LPS_SHORT_LPP_PACKET:
      lpsShortLppPacketHandler(pk);
      break;
    case EMERGENCY_STOP:
      isEmergencyStopRequested = true;
      break;
    case EMERGENCY_STOP_WATCHDOG:
      emergencyStopWatchdogNotificationTick = xTaskGetTickCount();
      break;
    case EXT_POSE:
      extPoseHandler(pk);
      break;
    case EXT_POSE_PACKED:
      extPosePackedHandler(pk);
      break;
    case LH_PERSIST_DATA:
      lhPersistDataHandler(pk);
      break;
    default:
      // Nothing here
      break;
  }
}

static void extPositionPackedHandler(CRTPPacket* pk)
{
  uint8_t numItems = pk->size / sizeof(extPositionPackedItem);
  for (uint8_t i = 0; i < numItems; ++i) {
    const extPositionPackedItem* item = (const extPositionPackedItem*)&pk->data[i * sizeof(extPositionPackedItem)];
    ext_pos.x = item->x / 1000.0f;
    ext_pos.y = item->y / 1000.0f;
    ext_pos.z = item->z / 1000.0f;
    ext_pos.stdDev = extPosStdDev;
    ext_pos.source = MeasurementSourceLocationService;
    if (item->id == my_id) {
      updateLogFromExtPos();
      estimatorEnqueuePosition(&ext_pos);
      tickOfLastPacket = xTaskGetTickCount();
    }
    else {
      peerLocalizationTellPosition(item->id, &ext_pos);
    }
  }
}

void locSrvSendRangeFloat(uint8_t id, float range)
{
  rangePacket *rp = (rangePacket *)pkRange.data;

  ASSERT(rangeIndex <= NBR_OF_RANGES_IN_PACKET);

  if (enableRangeStreamFloat)
  {
    rp->ranges[rangeIndex].id = id;
    rp->ranges[rangeIndex].range = range;
    rangeIndex++;

    if (rangeIndex >= 5)
    {
      rp->type = RANGE_STREAM_FLOAT;
      pkRange.port = CRTP_PORT_LOCALIZATION;
      pkRange.channel = GENERIC_TYPE;
      pkRange.size = sizeof(rangePacket);
      // This is best effort, i.e. the blocking version is not needed
      crtpSendPacket(&pkRange);
      rangeIndex = 0;
    }
  }
}

#ifdef CONFIG_DECK_LIGHTHOUSE
void locSrvSendLighthouseAngle(int baseStation, pulseProcessorResult_t* angles)
{
  anglePacket *ap = (anglePacket *)LhAngle.data;

  if (enableLighthouseAngleStream) {
    ap->baseStation = baseStation;
    pulseProcessorBaseStationMeasurement_t* baseStationMeasurement = &angles->baseStationMeasurementsLh1[baseStation];

    for(uint8_t its = 0; its < NBR_OF_SWEEPS_IN_PACKET; its++) {
      float angle_first_sensor =  baseStationMeasurement->sensorMeasurements[0].correctedAngles[its];
      ap->sweeps[its].sweep = angle_first_sensor;

      for(uint8_t itd = 0; itd < NBR_OF_SENSOR_DIFFS_IN_PACKET; itd++) {
        float angle_other_sensor = baseStationMeasurement->sensorMeasurements[itd + 1].correctedAngles[its];
        uint16_t angle_diff = single2half(angle_first_sensor - angle_other_sensor);
        ap->sweeps[its].angleDiffs[itd].angleDiff = angle_diff;
      }
    }

    ap->type = LH_ANGLE_STREAM;
    LhAngle.port = CRTP_PORT_LOCALIZATION;
    LhAngle.channel = GENERIC_TYPE;
    LhAngle.size = sizeof(anglePacket);
    // This is best effort, i.e. the blocking version is not needed
    crtpSendPacket(&LhAngle);
  }
}
#endif

bool locSrvIsEmergencyStopRequested() {
  return isEmergencyStopRequested;
}

void locSrvResetEmergencyStopRequest() {
  isEmergencyStopRequested = false;
}

uint32_t locSrvGetEmergencyStopWatchdogNotificationTick() {
  return emergencyStopWatchdogNotificationTick;
}

// This logging group is deprecated (removed after August 2023)
LOG_GROUP_START(ext_pos)
  LOG_ADD(LOG_FLOAT, X, &ext_pos.x)
  LOG_ADD(LOG_FLOAT, Y, &ext_pos.y)
  LOG_ADD(LOG_FLOAT, Z, &ext_pos.z)
LOG_GROUP_STOP(ext_pos)

/**
 * Logging variables for (external) positioning data stream through ctrp
 */
LOG_GROUP_START(locSrv)
/**
 * @brief Position X measurement from external system
 */
  LOG_ADD_CORE(LOG_FLOAT, x, &ext_pose.x)
/**
 * @brief Position Y measurement from external system
 */
  LOG_ADD_CORE(LOG_FLOAT, y, &ext_pose.y)
/**
 * @brief Position Z measurement from external system
 */
  LOG_ADD_CORE(LOG_FLOAT, z, &ext_pose.z)
/**
 * @brief Quaternion x meas from an external system
 */
  LOG_ADD_CORE(LOG_FLOAT, qx, &ext_pose.quat.x)
/**
 * @brief Quaternion y meas from an external system
 */
  LOG_ADD_CORE(LOG_FLOAT, qy, &ext_pose.quat.y)
/**
 * @brief Quaternion z meas from an external system
 */
  LOG_ADD_CORE(LOG_FLOAT, qz, &ext_pose.quat.z)
/**
 * @brief Quaternion w meas from an external system
 */
  LOG_ADD_CORE(LOG_FLOAT, qw, &ext_pose.quat.w)
LOG_GROUP_STOP(locSrv)

/**
 * Logging variables for (external) positioning data stream through Compressed
 */
LOG_GROUP_START(locSrvZ)
/**
 * @brief time when data was received last (ms/ticks)
 */
  LOG_ADD_CORE(LOG_UINT16, tick, &tickOfLastPacket)  // time when data was received last (ms/ticks)
LOG_GROUP_STOP(locSrvZ)

/**
 * Service parameters for (external) positioning data stream through ctrp
 */
PARAM_GROUP_START(locSrv)
/**
 * @brief Enable CRTP stream of Loco node distance
 */
  PARAM_ADD_CORE(PARAM_UINT8, enRangeStreamFP32, &enableRangeStreamFloat)
/**
 * @brief Enable CRTP stream of Lighthouse sweep angles
 */
  PARAM_ADD_CORE(PARAM_UINT8, enLhAngleStream, &enableLighthouseAngleStream)
/**
 * @brief Standard deviation of external position
 */
  PARAM_ADD_CORE(PARAM_FLOAT, extPosStdDev, &extPosStdDev)
  /**
 * @brief Standard deviation of the quarternion data to kalman filter
 */
  PARAM_ADD_CORE(PARAM_FLOAT, extQuatStdDev, &extQuatStdDev)
PARAM_GROUP_STOP(locSrv)
