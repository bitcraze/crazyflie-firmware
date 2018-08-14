/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie firmware.
 *
 * Copyright 2018, Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * tdoaEngine.c is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with tdoaEngine.c. If not, see <http://www.gnu.org/licenses/>.
 */


/*
Data storage encapsulation for the TDoA engine
*/

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "cfassert.h"

#define DEBUG_MODULE "TDOA_STORAGE"
#include "debug.h"

#include "tdoaStorage.h"
#include "clockCorrectionEngine.h"

#define TOF_VALIDITY_PERIOD M2T(2 * 1000);
#define REMOTE_DATA_VALIDITY_PERIOD M2T(30);
#define ANCHOR_POSITION_VALIDITY_PERIOD M2T(2 * 1000);


static anchorInfo_t anchorStorage[ANCHOR_STORAGE_COUNT];

void tdoaStorageInitialize() {
  memset(anchorStorage, 0, sizeof(anchorStorage));
}

anchorInfo_t* tdoaStorageGetAnchorCtx(const uint8_t anchor) {
  // TODO krri add lookup table to avoid linear search
  for (int i = 0; i < ANCHOR_STORAGE_COUNT; i++) {
    if (anchor == anchorStorage[i].id && anchorStorage[i].isInitialized) {
      return &anchorStorage[i];
    }
  }

  return 0;
}

anchorInfo_t* tdoaStorageInitializeNewAnchorContext(const uint8_t anchor) {
  int indexToInitialize = 0;
  uint32_t now = xTaskGetTickCount();
  uint32_t oldestUpdateTime = now;

  for (int i = 0; i < ANCHOR_STORAGE_COUNT; i++) {
    if (!anchorStorage[i].isInitialized) {
      indexToInitialize = i;
      break;
    }

    if (anchorStorage[i].lastUpdateTime < oldestUpdateTime) {
      oldestUpdateTime = anchorStorage[i].lastUpdateTime;
      indexToInitialize = i;
    }
  }

  memset(&anchorStorage[indexToInitialize], 0, sizeof(anchorInfo_t));
  anchorStorage[indexToInitialize].id = anchor;
  anchorStorage[indexToInitialize].isInitialized = true;

  return &anchorStorage[indexToInitialize];
}

uint8_t tdoaStorageGetId(const anchorInfo_t* anchorCtx) {
  return anchorCtx->id;
}

int64_t tdoaStorageGetRxTime(const anchorInfo_t* anchorCtx) {
  return anchorCtx->rxTime;
}

int64_t tdoaStorageGetTxTime(const anchorInfo_t* anchorCtx) {
  return anchorCtx->txTime;
}

uint8_t tdoaStorageGetSeqNr(const anchorInfo_t* anchorCtx) {
  return anchorCtx->seqNr;
}

bool tdoaStorageGetAnchorPosition(const anchorInfo_t* anchorCtx, point_t* position) {
  uint32_t now = xTaskGetTickCount();
  int32_t validCreationTime = now - ANCHOR_POSITION_VALIDITY_PERIOD;
  if ((int32_t)anchorCtx->position.timestamp > validCreationTime) {
    position->x = anchorCtx->position.x;
    position->y = anchorCtx->position.y;
    position->z = anchorCtx->position.z;
    return true;
  }

  return false;
}

void tdoaStorageSetAnchorPosition(anchorInfo_t* anchorCtx, const float x, const float y, const float z) {
  uint32_t now = xTaskGetTickCount();

  anchorCtx->position.timestamp = now;
  anchorCtx->position.x = x;
  anchorCtx->position.y = y;
  anchorCtx->position.z = z;
}

void tdoaStorageSetRxTxData(anchorInfo_t* anchorCtx, int64_t rxTime, int64_t txTime, uint8_t seqNr) {
  uint32_t now = xTaskGetTickCount();

  anchorCtx->rxTime = rxTime;
  anchorCtx->txTime = txTime;
  anchorCtx->seqNr = seqNr;
  anchorCtx->lastUpdateTime = now;
}

  double tdoaStorageGetClockCorrection(const anchorInfo_t* anchorCtx) {
  return clockCorrectionEngine.getClockCorrection(&anchorCtx->clockCorrectionStorage);
}

int64_t tdoaStorageGetRemoteRxTime(const anchorInfo_t* anchorCtx, const uint8_t remoteAnchor) {
  for (int i = 0; i < REMOTE_ANCHOR_DATA_COUNT; i++) {
    if (remoteAnchor == anchorCtx->remoteAnchorData[i].id) {
      uint32_t now = xTaskGetTickCount();
      if (anchorCtx->remoteAnchorData[i].endOfLife > now) {
        return anchorCtx->remoteAnchorData[i].rxTime;
      }
      break;
    }
  }

  return 0;
}

void tdoaStorageSetRemoteRxTime(anchorInfo_t* anchorCtx, const uint8_t remoteAnchor, const int64_t remoteRxTime, const uint8_t remoteSeqNr) {
  int indexToUpdate = 0;
  uint32_t now = xTaskGetTickCount();
  uint32_t oldestTime = 0xFFFFFFFF;

  for (int i = 0; i < REMOTE_ANCHOR_DATA_COUNT; i++) {
    if (remoteAnchor == anchorCtx->remoteAnchorData[i].id) {
      indexToUpdate = i;
      break;
    }

    if (anchorCtx->remoteAnchorData[i].endOfLife < oldestTime) {
      oldestTime = anchorCtx->remoteAnchorData[i].endOfLife;
      indexToUpdate = i;
    }
  }

  anchorCtx->remoteAnchorData[indexToUpdate].id = remoteAnchor;
  anchorCtx->remoteAnchorData[indexToUpdate].rxTime = remoteRxTime;
  anchorCtx->remoteAnchorData[indexToUpdate].seqNr = remoteSeqNr;
  anchorCtx->remoteAnchorData[indexToUpdate].endOfLife = now + REMOTE_DATA_VALIDITY_PERIOD;
}

void tdoaStorageGetRemoteSeqNrList(const anchorInfo_t* anchorCtx, int* remoteCount, uint8_t seqNr[], uint8_t id[]) {
  int count = 0;

  uint32_t now = xTaskGetTickCount();
  for (int i = 0; i < REMOTE_ANCHOR_DATA_COUNT; i++) {
    if (anchorCtx->remoteAnchorData[i].endOfLife > now) {
      id[count] = anchorCtx->remoteAnchorData[i].id;
      seqNr[count] = anchorCtx->remoteAnchorData[i].seqNr;
      count++;
    }
  }

  *remoteCount = count;
}

int64_t tdoaStorageGetTimeOfFlight(const anchorInfo_t* anchorCtx, const uint8_t otherAnchor) {
  for (int i = 0; i < TOF_PER_ANCHOR_COUNT; i++) {
    if (otherAnchor == anchorCtx->tof[i].id) {
      uint32_t now = xTaskGetTickCount();
      if (anchorCtx->tof[i].endOfLife > now) {
        return anchorCtx->tof[i].tof;
      }
      break;
    }
  }

  return 0;
}

void tdoaStorageSetTimeOfFlight(anchorInfo_t* anchorCtx, const uint8_t remoteAnchor, const int64_t tof) {
  int indexToUpdate = 0;
  uint32_t now = xTaskGetTickCount();
  uint32_t oldestTime = 0xFFFFFFFF;

  for (int i = 0; i < TOF_PER_ANCHOR_COUNT; i++) {
    if (remoteAnchor == anchorCtx->tof[i].id) {
      indexToUpdate = i;
      break;
    }

    if (anchorCtx->tof[i].endOfLife < oldestTime) {
      oldestTime = anchorCtx->tof[i].endOfLife;
      indexToUpdate = i;
    }
  }

  anchorCtx->tof[indexToUpdate].id = remoteAnchor;
  anchorCtx->tof[indexToUpdate].tof = tof;
  anchorCtx->tof[indexToUpdate].endOfLife = now + TOF_VALIDITY_PERIOD;
}
