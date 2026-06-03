/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie firmware.
 *
 * Copyright 2024, Bitcraze AB
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
 * tdoaQuality.c - optional per-anchor TDoA quality logging for debugging.
 *
 * See tdoaQuality.h for the rationale. The whole module is compiled only when
 * CONFIG_DEBUG_TDOA_QUALITY is set (see the Kbuild), so it has zero footprint
 * otherwise.
 */

#include <string.h>
#include <math.h>

#include "tdoaQuality.h"
#include "tdoaStorage.h"
#include "log.h"

// Number of anchors exposed in the snapshot / log group. Slot index is stable:
// an anchor keeps its slot for as long as it stays active.
#define TDOA_QUALITY_ANCHOR_COUNT 8

// How often the loggable snapshot is refreshed from storage. The residual /
// reject statistics are therefore an RMS / rate over this window.
#define TDOA_QUALITY_UPDATE_INTERVAL_MS 100

// Age value written to an empty (unused) slot. Also reads as "very stale" to a
// consumer that does not special-case it.
#define TDOA_QUALITY_EMPTY_AGE 0xFFFF

// Loggable snapshot of one anchor. One per slot, published in the log group.
typedef struct {
  uint8_t id;       // Anchor id occupying this slot
  float x;          // Anchor position [m] (NaN if unknown to the firmware)
  float y;
  float z;
  float cc;         // Clock correction factor (~1.0 when healthy)
  uint16_t age;     // Time since last heard [ms], TDOA_QUALITY_EMPTY_AGE if slot empty
  float resRms;     // RMS of residual (measurement - predicted) over the window [m]
  float rejRate;    // Fraction of measurements rejected by the outlier filter (0..1)
  uint32_t accCount; // Cumulative accepted measurement count (raw, monotonic)
  uint32_t rejCount; // Cumulative rejected measurement count (raw, monotonic)
} qualityAnchorLog_t;

// Per-anchor accumulator for residual / reject statistics, keyed by anchor id.
// resSumSq/resCount cover ACCEPTED measurements only (so a rejected outlier of
// hundreds of metres can't pollute the residual RMS); rejCount/totalCount give
// the reject fraction over everything.
typedef struct {
  bool used;
  uint8_t id;
  float resSumSq;
  uint32_t resCount;
  uint32_t rejCount;
  uint32_t totalCount;
} qualityAccum_t;

// EMA weight applied to each window's residual RMS / reject rate, so a single
// bad 100 ms window does not whip the value (and the displayed health) around.
#define TDOA_QUALITY_EMA_ALPHA 0.3f

static qualityAnchorLog_t qLog[TDOA_QUALITY_ANCHOR_COUNT];
static bool slotUsed[TDOA_QUALITY_ANCHOR_COUNT];
static qualityAccum_t accum[TDOA_QUALITY_ANCHOR_COUNT];
static uint32_t nextUpdate_ms = 0;

// Find the accumulator for an anchor id, allocating a free one if needed.
// Returns NULL only when all slots are taken by other anchors.
static qualityAccum_t* findOrAllocAccum(const uint8_t id) {
  qualityAccum_t* freeSlot = NULL;
  for (int i = 0; i < TDOA_QUALITY_ANCHOR_COUNT; i++) {
    if (accum[i].used && accum[i].id == id) {
      return &accum[i];
    }
    if (!accum[i].used && freeSlot == NULL) {
      freeSlot = &accum[i];
    }
  }
  if (freeSlot != NULL) {
    freeSlot->used = true;
    freeSlot->id = id;
    freeSlot->resSumSq = 0.0f;
    freeSlot->resCount = 0;
    freeSlot->rejCount = 0;
    freeSlot->totalCount = 0;
  }
  return freeSlot;
}

static void freeAccum(const uint8_t id) {
  for (int i = 0; i < TDOA_QUALITY_ANCHOR_COUNT; i++) {
    if (accum[i].used && accum[i].id == id) {
      accum[i].used = false;
      return;
    }
  }
}

void tdoaQualityReportResidual(const uint8_t idA, const uint8_t idB, const float residual, const bool accepted) {
  // The residual is a property of the pair; attribute it to both anchors so a
  // single misbehaving anchor shows up across all the pairs it appears in.
  // Lockless on purpose: a rare race with the snapshot update only perturbs a
  // debug statistic slightly, and avoiding a critical section keeps this off
  // the estimator's hot path cheap.
  const float sq = residual * residual;
  const uint8_t ids[2] = {idA, idB};
  for (int k = 0; k < 2; k++) {
    qualityAccum_t* a = findOrAllocAccum(ids[k]);
    if (a != NULL) {
      a->totalCount++;
      if (accepted) {
        // Residual RMS reflects the quality of measurements actually used;
        // rejected outliers (which can be hundreds of metres) are excluded.
        a->resSumSq += sq;
        a->resCount++;
      } else {
        a->rejCount++;
      }
    }
  }
}

static void clearSlot(const int s) {
  freeAccum(qLog[s].id);
  memset(&qLog[s], 0, sizeof(qLog[s]));
  qLog[s].age = TDOA_QUALITY_EMPTY_AGE;
  slotUsed[s] = false;
}

static void refreshSlot(tdoaEngineState_t* state, const int s, const uint8_t id, const uint32_t now_ms) {
  qLog[s].id = id;

  tdoaAnchorContext_t ctx;
  if (tdoaStorageGetAnchorCtx(state->anchorInfoArray, id, now_ms, &ctx)) {
    point_t pos;
    if (tdoaStorageGetAnchorPosition(&ctx, &pos)) {
      qLog[s].x = pos.x;
      qLog[s].y = pos.y;
      qLog[s].z = pos.z;
    } else {
      qLog[s].x = qLog[s].y = qLog[s].z = NAN;
    }
    qLog[s].cc = (float)tdoaStorageGetClockCorrection(&ctx);

    const uint32_t age = now_ms - tdoaStorageGetLastUpdateTime(&ctx);
    qLog[s].age = (age >= TDOA_QUALITY_EMPTY_AGE) ? (TDOA_QUALITY_EMPTY_AGE - 1) : (uint16_t)age;
  }

  // Fold this window's residual RMS (accepted only) and reject fraction (over
  // the total) into the logged values with an EMA, then reset the window. The
  // EMA keeps the displayed numbers — and the health derived from them — from
  // flickering when one window happens to be noisy. Values are left untouched
  // when a window had no data, so they don't drop to zero between windows.
  qualityAccum_t* a = findOrAllocAccum(id);
  if (a != NULL) {
    if (a->resCount > 0) {
      const float windowRms = sqrtf(a->resSumSq / (float)a->resCount);
      qLog[s].resRms += TDOA_QUALITY_EMA_ALPHA * (windowRms - qLog[s].resRms);
    }
    if (a->totalCount > 0) {
      const float windowRej = (float)a->rejCount / (float)a->totalCount;
      qLog[s].rejRate += TDOA_QUALITY_EMA_ALPHA * (windowRej - qLog[s].rejRate);
    }
    // Carry the raw window counts into the slot's cumulative (monotonic)
    // accepted/rejected counters before the window accumulators are reset.
    // accCount uses resCount (accepted only); rejCount uses rejCount.
    qLog[s].accCount += a->resCount;
    qLog[s].rejCount += a->rejCount;
    a->resSumSq = 0.0f;
    a->resCount = 0;
    a->rejCount = 0;
    a->totalCount = 0;
  }
}

void tdoaQualityUpdate(tdoaEngineState_t* state, const uint32_t now_ms) {
  if (now_ms < nextUpdate_ms) {
    return;
  }
  nextUpdate_ms = now_ms + TDOA_QUALITY_UPDATE_INTERVAL_MS;

  uint8_t activeIds[TDOA_QUALITY_ANCHOR_COUNT];
  const uint8_t count = tdoaStorageGetListOfActiveAnchorIds(
      state->anchorInfoArray, activeIds, TDOA_QUALITY_ANCHOR_COUNT, now_ms);

  bool idHandled[TDOA_QUALITY_ANCHOR_COUNT];
  memset(idHandled, 0, sizeof(idHandled));

  // 1. Refresh slots that still map to an active anchor; free the rest. This
  //    keeps each anchor pinned to a stable slot across updates.
  for (int s = 0; s < TDOA_QUALITY_ANCHOR_COUNT; s++) {
    if (!slotUsed[s]) {
      continue;
    }
    int match = -1;
    for (int k = 0; k < count; k++) {
      if (!idHandled[k] && activeIds[k] == qLog[s].id) {
        match = k;
        break;
      }
    }
    if (match >= 0) {
      idHandled[match] = true;
      refreshSlot(state, s, qLog[s].id, now_ms);
    } else {
      clearSlot(s);
    }
  }

  // 2. Assign newly-active anchors to free slots.
  for (int k = 0; k < count; k++) {
    if (idHandled[k]) {
      continue;
    }
    for (int s = 0; s < TDOA_QUALITY_ANCHOR_COUNT; s++) {
      if (!slotUsed[s]) {
        slotUsed[s] = true;
        refreshSlot(state, s, activeIds[k], now_ms);
        break;
      }
    }
  }
}

/**
 * Per-anchor TDoA quality, a live snapshot of all tracked anchors.
 *
 * Each slot i (0..7) describes one anchor; id<i> is the anchor occupying the
 * slot and age<i> == 65535 marks an empty slot. A slot is stable: an anchor
 * keeps its slot while it stays active. Enabled by CONFIG_DEBUG_TDOA_QUALITY.
 */
#define QLOG_ANCHOR(i) \
  LOG_ADD(LOG_UINT8, id##i, &qLog[i].id) \
  LOG_ADD(LOG_FLOAT, x##i, &qLog[i].x) \
  LOG_ADD(LOG_FLOAT, y##i, &qLog[i].y) \
  LOG_ADD(LOG_FLOAT, z##i, &qLog[i].z) \
  LOG_ADD(LOG_FLOAT, cc##i, &qLog[i].cc) \
  LOG_ADD(LOG_UINT16, age##i, &qLog[i].age) \
  LOG_ADD(LOG_FLOAT, res##i, &qLog[i].resRms) \
  LOG_ADD(LOG_FLOAT, rej##i, &qLog[i].rejRate) \
  LOG_ADD(LOG_UINT32, accCount##i, &qLog[i].accCount) \
  LOG_ADD(LOG_UINT32, rejCount##i, &qLog[i].rejCount)

LOG_GROUP_START(tdoaQuality)
QLOG_ANCHOR(0)
QLOG_ANCHOR(1)
QLOG_ANCHOR(2)
QLOG_ANCHOR(3)
QLOG_ANCHOR(4)
QLOG_ANCHOR(5)
QLOG_ANCHOR(6)
QLOG_ANCHOR(7)
LOG_GROUP_STOP(tdoaQuality)
