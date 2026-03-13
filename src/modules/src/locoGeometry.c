/**
 * locoGeometry.c - Loco anchor geometry estimation support
 *
 * Exposes two data structures via the MEM_TYPE_LOCO_GEOMETRY memory interface
 * so a host-side Python script can estimate anchor positions automatically:
 *
 *   1. Inter-anchor ToF matrix  — for Multidimensional Scaling (MDS) to get
 *                                 the geometry in an arbitrary reference frame.
 *   2. TDoA snapshot            — raw d(CF,Ai) - d(CF,Aj) measurements taken
 *                                 while the CF sits at a user-chosen position,
 *                                 used to align the MDS result to the world frame.
 *
 * Memory layout  (MEM_TYPE_LOCO_GEOMETRY):
 *
 *   0x0000  [1 B ]  nAnchors           – active anchor count
 *   0x0001  [16 B]  anchorIds[16]      – anchor ID for matrix index i
 *   (pad to 0x0100)
 *   0x0100  [512 B] tofMatrix[16][16]  – uint16, inter-anchor ToF in UWB ticks
 *                                        (0 = no data; row = receiving anchor)
 *   0x0300  [1024B] tdoaSnapshot[16][16] – float32, d(CF,Ai)-d(CF,Aj) in metres
 *                                          (INFINITY = no data)
 *
 * Workflow:
 *   1. Power on CF anywhere — tofMatrix fills in passively within ~5 s.
 *   2. Place CF at a known position (e.g., origin). Set param locoGeo.capture=1.
 *      Wait until log locoGeo.state == 3 (done). Read tdoaSnapshot.
 *   3. Repeat step 2 for each reference position (e.g., 1 m along X).
 *   4. Host script runs MDS + alignment → anchor world-frame positions.
 *
 * NOTE: anchorInfoArray is read from a low-priority task without the
 * locodeck semaphore. Occasional stale reads are harmless because we
 * average over CAPTURE_SAMPLES iterations while the CF is stationary.
 */

#define DEBUG_MODULE "LOCO_GEO"

#include "locoGeometry.h"

#include <math.h>
#include <string.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"
#include "log.h"
#include "mem.h"
#include "param.h"
#include "physicalConstants.h"

#include "tdoaEngine.h"
#include "tdoaEngineInstance.h"
#include "tdoaStorage.h"

/* ── Constants ──────────────────────────────────────────────────────────────── */

#define MAX_ANCHORS         ANCHOR_STORAGE_COUNT   /* 16 */
#define TASK_PERIOD_MS      20
#define TASK_STACK_WORDS    600
#define CAPTURE_SAMPLES     100  /* sample rounds to average per snapshot */

/* captureState values (written by host via param, updated by task) */
#define STATE_IDLE     0
#define STATE_START    1   /* host writes this to begin a new snapshot   */
#define STATE_RUNNING  2   /* task sets this while accumulating           */
#define STATE_DONE     3   /* task sets this when snapshot is ready       */

/* ── Memory-map offsets ─────────────────────────────────────────────────────── */

#define MEM_HEADER_OFFSET   0x0000u
#define MEM_TOF_OFFSET      0x0100u
#define MEM_TDOA_OFFSET     0x0300u
#define MEM_TOTAL_SIZE      0x0700u  /* 1792 bytes */

#define HEADER_SIZE   (1u + MAX_ANCHORS)                           /*  17 B */
#define TOF_SIZE      (MAX_ANCHORS * MAX_ANCHORS * sizeof(uint16_t)) /* 512 B */
#define TDOA_SIZE     (MAX_ANCHORS * MAX_ANCHORS * sizeof(float))    /* 1024 B */

/* ── Module state ───────────────────────────────────────────────────────────── */

static uint8_t  nActiveAnchors;
static uint8_t  anchorIds[MAX_ANCHORS];
static uint16_t tofMatrix[MAX_ANCHORS][MAX_ANCHORS];
static float    tdoaSnapshot[MAX_ANCHORS][MAX_ANCHORS];

/* Accumulation buffers — only used during STATE_RUNNING */
static float    tdoaAccum[MAX_ANCHORS][MAX_ANCHORS];
static uint16_t tdoaCounts[MAX_ANCHORS][MAX_ANCHORS];

static uint8_t  captureState   = STATE_IDLE;
static uint16_t capturedSamples = 0;

/* ── TDoA computation ───────────────────────────────────────────────────────── */
/*
 * Re-implements calcTDoA() + calcDistanceDiff() from tdoaEngine.c using only
 * the public storage API. Returns d(CF, anchorAn) - d(CF, anchorAr) in metres.
 *
 *  ctxAn  — "main" anchor (An): the one whose packet was last processed
 *  ctxAr  — "other" anchor (Ar): the one we are comparing against
 */
static bool computeTdoaPair(const tdoaAnchorContext_t *ctxAn,
                             const tdoaAnchorContext_t *ctxAr,
                             float *distDiff)
{
  const uint8_t idAr = tdoaStorageGetId(ctxAr);

  /* inter-anchor ToF Ar→An (required; comes from the distance field in packets) */
  const int64_t tof_Ar_to_An = tdoaStorageGetRemoteTimeOfFlight(ctxAn, idAr);
  if (tof_Ar_to_An == 0) return false;

  /* clock correction for An */
  const double cc = tdoaStorageGetClockCorrection(ctxAn);
  if (cc <= 0.0) return false;

  /* when An last received a packet from Ar (in An's clock, 30 ms validity) */
  int64_t rxAr_by_An;
  uint8_t seqNr;
  if (!tdoaStorageGetRemoteRxTimeSeqNr(ctxAn, idAr, &rxAr_by_An, &seqNr)) return false;

  /* last stored timestamps */
  const int64_t txAn      = tdoaStorageGetTxTime(ctxAn);
  const int64_t rxAn_by_T = tdoaStorageGetRxTime(ctxAn);
  const int64_t rxAr_by_T = tdoaStorageGetRxTime(ctxAr);
  if (txAn == 0 || rxAn_by_T == 0 || rxAr_by_T == 0) return false;

  /* mirror of calcTDoA() — same formula, different call site */
  const int64_t delta =
    (int64_t)tdoaEngineTruncateToAnchorTimeStamp(
      (uint64_t)(tof_Ar_to_An +
        (int64_t)tdoaEngineTruncateToAnchorTimeStamp((uint64_t)(txAn - rxAr_by_An))));

  const int64_t timeDiff =
    (int64_t)tdoaEngineTruncateToAnchorTimeStamp((uint64_t)(rxAn_by_T - rxAr_by_T))
    - (int64_t)((double)delta * cc);

  *distDiff = (float)(SPEED_OF_LIGHT * (double)timeDiff
                      / tdoaEngineState.locodeckTsFreq);
  return true;
}

/* ── Update ToF matrix (runs every task tick) ───────────────────────────────── */

static void updateTofMatrix(uint32_t now_ms)
{
  nActiveAnchors = tdoaStorageGetListOfActiveAnchorIds(
    tdoaEngineState.anchorInfoArray, anchorIds, MAX_ANCHORS, now_ms);

  for (uint8_t i = 0; i < nActiveAnchors; i++) {
    tdoaAnchorContext_t ctxA;
    if (!tdoaStorageGetAnchorCtx(
          tdoaEngineState.anchorInfoArray, anchorIds[i], now_ms, &ctxA)) continue;

    for (uint8_t j = 0; j < nActiveAnchors; j++) {
      if (i == j) continue;
      const int64_t tof = tdoaStorageGetRemoteTimeOfFlight(&ctxA, anchorIds[j]);
      if (tof > 0 && tof <= 0xFFFF) {
        tofMatrix[i][j] = (uint16_t)tof;
      }
    }
  }
}

/* ── Accumulate one round of TDoA samples ───────────────────────────────────── */

static void accumulateTdoaSamples(uint32_t now_ms)
{
  uint8_t ids[MAX_ANCHORS];
  const uint8_t n = tdoaStorageGetListOfActiveAnchorIds(
    tdoaEngineState.anchorInfoArray, ids, MAX_ANCHORS, now_ms);

  bool gotAny = false;

  for (uint8_t i = 0; i < n; i++) {
    for (uint8_t j = 0; j < n; j++) {
      if (i == j) continue;

      tdoaAnchorContext_t ctxAn, ctxAr;
      if (!tdoaStorageGetAnchorCtx(
            tdoaEngineState.anchorInfoArray, ids[i], now_ms, &ctxAn)) continue;
      if (!tdoaStorageGetAnchorCtx(
            tdoaEngineState.anchorInfoArray, ids[j], now_ms, &ctxAr)) continue;

      float diff;
      if (computeTdoaPair(&ctxAn, &ctxAr, &diff)) {
        tdoaAccum[i][j]  += diff;
        tdoaCounts[i][j] += 1;
        gotAny = true;
      }
    }
  }

  if (gotAny) {
    capturedSamples++;
  }
}

/* ── Compute final snapshot from accumulators ───────────────────────────────── */

static void finalizeSnapshot(void)
{
  for (uint8_t i = 0; i < MAX_ANCHORS; i++) {
    for (uint8_t j = 0; j < MAX_ANCHORS; j++) {
      if (i != j && tdoaCounts[i][j] > 0) {
        tdoaSnapshot[i][j] = tdoaAccum[i][j] / (float)tdoaCounts[i][j];
      } else {
        tdoaSnapshot[i][j] = (float)INFINITY; /* sentinel: no data */
      }
    }
  }
}

/* ── FreeRTOS task ──────────────────────────────────────────────────────────── */

static void locoGeoTask(void *param)
{
  (void)param;

  while (true) {
    vTaskDelay(M2T(TASK_PERIOD_MS));

    const uint32_t now_ms = T2M(xTaskGetTickCount());

    updateTofMatrix(now_ms);

    switch (captureState) {
      case STATE_START:
        memset(tofMatrix,  0, sizeof(tofMatrix));
        memset(tdoaAccum,  0, sizeof(tdoaAccum));
        memset(tdoaCounts, 0, sizeof(tdoaCounts));
        capturedSamples = 0;
        captureState = STATE_RUNNING;
        /* fall through */

      case STATE_RUNNING:
        accumulateTdoaSamples(now_ms);
        if (capturedSamples >= CAPTURE_SAMPLES) {
          finalizeSnapshot();
          captureState = STATE_DONE;
          DEBUG_PRINT("locoGeo: snapshot done (%u anchors)\n", nActiveAnchors);
        }
        break;

      default:
        break;
    }
  }
}

/* ── Memory handler ─────────────────────────────────────────────────────────── */

static uint32_t handleGetSize(const uint8_t id)
{
  (void)id;
  return MEM_TOTAL_SIZE;
}

static bool handleRead(const uint8_t id, const uint32_t addr,
                       const uint8_t readLen, uint8_t *buf)
{
  (void)id;
  if (addr + readLen > MEM_TOTAL_SIZE) return false;

  /* Header: nAnchors (1 byte) + anchorIds[16] */
  if (addr < MEM_TOF_OFFSET) {
    uint8_t header[HEADER_SIZE];
    header[0] = nActiveAnchors;
    memcpy(&header[1], anchorIds, MAX_ANCHORS);
    const uint32_t off = addr - MEM_HEADER_OFFSET;
    if (off + readLen > HEADER_SIZE) return false;
    memcpy(buf, header + off, readLen);
    return true;
  }

  /* ToF matrix: uint16_t[16][16] */
  if (addr >= MEM_TOF_OFFSET && addr < MEM_TDOA_OFFSET) {
    const uint32_t off = addr - MEM_TOF_OFFSET;
    if (off + readLen > TOF_SIZE) return false;
    memcpy(buf, (const uint8_t *)tofMatrix + off, readLen);
    return true;
  }

  /* TDoA snapshot: float[16][16] */
  if (addr >= MEM_TDOA_OFFSET && addr < MEM_TOTAL_SIZE) {
    const uint32_t off = addr - MEM_TDOA_OFFSET;
    if (off + readLen > TDOA_SIZE) return false;
    memcpy(buf, (const uint8_t *)tdoaSnapshot + off, readLen);
    return true;
  }

  return false;
}

static const MemoryHandlerDef_t memDef = {
  .type    = MEM_TYPE_LOCO_GEOMETRY,
  .getSize = handleGetSize,
  .read    = handleRead,
  .write   = NULL,
};

/* ── Init ───────────────────────────────────────────────────────────────────── */

void locoGeometryInit(void)
{
  /* Initialise snapshot to "no data" */
  for (uint8_t i = 0; i < MAX_ANCHORS; i++) {
    for (uint8_t j = 0; j < MAX_ANCHORS; j++) {
      tdoaSnapshot[i][j] = (float)INFINITY;
    }
  }

  memoryRegisterHandler(&memDef);

  xTaskCreate(locoGeoTask, "locoGeo", TASK_STACK_WORDS, NULL,
              /*priority*/ 2, NULL);

  DEBUG_PRINT("locoGeometry: initialized\n");
}

/* ── Logs & Params ──────────────────────────────────────────────────────────── */

LOG_GROUP_START(locoGeo)
/** @brief Current state: 0=idle 1=start 2=running 3=done */
LOG_ADD(LOG_UINT8,  state,    &captureState)
/** @brief Sample rounds accumulated so far */
LOG_ADD(LOG_UINT16, samples,  &capturedSamples)
/** @brief Number of active anchors currently visible */
LOG_ADD(LOG_UINT8,  nAnchors, &nActiveAnchors)
LOG_GROUP_STOP(locoGeo)

PARAM_GROUP_START(locoGeo)
/**
 * @brief Capture trigger.
 * Write 1 to start a new TDoA snapshot at the current CF position.
 * The firmware sets this to 2 (running) then 3 (done) automatically.
 * Write 0 to reset to idle.
 */
PARAM_ADD(PARAM_UINT8, capture, &captureState)
PARAM_GROUP_STOP(locoGeo)
