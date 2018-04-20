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

#include "locodeck.h"

#include "estimator_kalman.h"

#define NBR_OF_RANGES_IN_PACKET   5
#define DEFAULT_EMERGENCY_STOP_TIMEOUT (1 * RATE_MAIN_LOOP)

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

// up to 4 items per CRTP packet
typedef struct {
  uint8_t id; // last 8 bit of the Crazyflie address
  int16_t x; // mm
  int16_t y; // mm
  int16_t z; // mm
} __attribute__((packed)) extPositionPackedItem;

/**
 * Position data cache
 */
typedef struct
{
  struct CrtpExtPosition targetVal[2];
  bool activeSide;
  uint32_t timestamp; // FreeRTOS ticks
} ExtPositionCache;

// Struct for logging position information
static positionMeasurement_t ext_pos;
static ExtPositionCache crtpExtPosCache;
static CRTPPacket pkRange;
static uint8_t rangeIndex;
static bool enableRangeStreamFloat = false;
static bool isInit = false;
static uint8_t my_id;

static void locSrvCrtpCB(CRTPPacket* pk);
static void extPositionHandler(CRTPPacket* pk);
static void genericLocHandle(CRTPPacket* pk);
static void extPositionPackedHandler(CRTPPacket* pk);

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
    case EXT_POSITION_PACKED:
      extPositionPackedHandler(pk);
    default:
      break;
  }
}

static void extPositionHandler(CRTPPacket* pk)
{
  crtpExtPosCache.targetVal[!crtpExtPosCache.activeSide] = *((struct CrtpExtPosition*)pk->data);
  crtpExtPosCache.activeSide = !crtpExtPosCache.activeSide;
  crtpExtPosCache.timestamp = xTaskGetTickCount();
}

static void genericLocHandle(CRTPPacket* pk)
{
  uint8_t type = pk->data[0];
  if (pk->size < 1) return;

  if (type == LPS_SHORT_LPP_PACKET && pk->size >= 2) {
    bool success = lpsSendLppShort(pk->data[1], &pk->data[2], pk->size-2);

    pk->port = CRTP_PORT_LOCALIZATION;
    pk->channel = GENERIC_TYPE;
    pk->size = 3;
    pk->data[2] = success?1:0;
    crtpSendPacket(pk);
  } else if (type == EMERGENCY_STOP) {
    stabilizerSetEmergencyStop();
  } else if (type == EMERGENCY_STOP_WATCHDOG) {
    stabilizerSetEmergencyStopTimeout(DEFAULT_EMERGENCY_STOP_TIMEOUT);
  }
}

static void extPositionPackedHandler(CRTPPacket* pk)
{
  uint8_t numItems = pk->size / sizeof(extPositionPackedItem);
  for (uint8_t i = 0; i < numItems; ++i) {
    const extPositionPackedItem* item = (const extPositionPackedItem*)&pk->data[i * sizeof(extPositionPackedItem)];
    if (item->id == my_id) {
      struct CrtpExtPosition position;
      position.x = item->x / 1000.0f;
      position.y = item->y / 1000.0f;
      position.z = item->z / 1000.0f;

      crtpExtPosCache.targetVal[!crtpExtPosCache.activeSide] = position;
      crtpExtPosCache.activeSide = !crtpExtPosCache.activeSide;
      crtpExtPosCache.timestamp = xTaskGetTickCount();

      break;
    }
  }
}

bool getExtPosition(state_t *state)
{
  // Only use position information if it's valid and recent
  if ((xTaskGetTickCount() - crtpExtPosCache.timestamp) < M2T(5)) {
    // Get the updated position from the mocap
    ext_pos.x = crtpExtPosCache.targetVal[crtpExtPosCache.activeSide].x;
    ext_pos.y = crtpExtPosCache.targetVal[crtpExtPosCache.activeSide].y;
    ext_pos.z = crtpExtPosCache.targetVal[crtpExtPosCache.activeSide].z;
    ext_pos.stdDev = 0.01;
    estimatorKalmanEnqueuePosition(&ext_pos);

    return true;
  }
  return false;
}

void locSrvSendPacket(locsrv_t type, uint8_t *data, uint8_t length)
{
  CRTPPacket pk;

  ASSERT(length < CRTP_MAX_DATA_SIZE);

  pk.port = CRTP_PORT_LOCALIZATION;
  pk.channel = GENERIC_TYPE;
  memcpy(pk.data, data, length);
  crtpSendPacket(&pk);
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
      crtpSendPacket(&pkRange);
      rangeIndex = 0;
    }
  }
}

LOG_GROUP_START(ext_pos)
  LOG_ADD(LOG_FLOAT, X, &ext_pos.x)
  LOG_ADD(LOG_FLOAT, Y, &ext_pos.y)
  LOG_ADD(LOG_FLOAT, Z, &ext_pos.z)
LOG_GROUP_STOP(ext_pos)

PARAM_GROUP_START(locSrv)
PARAM_ADD(PARAM_UINT8, enRangeStreamFP32, &enableRangeStreamFloat)
PARAM_GROUP_STOP(locSrv)
