#include "pulse_processor.h"

#include <string.h>
#include <math.h>
#include "test_support.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Decoding contants
// Times are expressed in a 48MHz clock
#define FRAME_LENGTH 400000    // 8.333ms
#define SWEEP_MAX_WIDTH 1024    // 20us
#define SWEEP_CENTER 192000    // 4ms
#define SYNC_BASE_WIDTH 2750
#define SYNC_DIVIDER 500
#define SYNC_MAX_SEPARATION 25000   // More than 400us (400us is 19200)
#define SYNC_SEPARATION 19200
#define SENSOR_MAX_DISPERTION 20
#define MAX_FRAME_LENGTH_NOISE 800

// Utility functions and macros
// #define TS_DIFF(X, Y) ((X-Y)&((1<<TIMESTAMP_BITWIDTH)-1))
static uint32_t TS_DIFF(uint32_t x, uint32_t y) {
  const uint32_t bitmask = (1 << TIMESTAMP_BITWIDTH) - 1;
  return (x - y) & bitmask;
}



TESTABLE_STATIC bool findSyncTime(const pulseProcessorPulse_t pulseHistory[], uint32_t *foundSyncTime);
TESTABLE_STATIC bool getSystemSyncTime(const uint32_t syncTimes[], size_t nSyncTimes, uint32_t *syncTime);

static void resetPulseHistory(pulseProcessor_t *state) {
  memset(state->pulseHistoryPtr, 0, sizeof(state->pulseHistoryPtr));
}

static void synchronize(pulseProcessor_t *state, int sensor, uint32_t timestamp, uint32_t width)
{
  state->pulseHistory[sensor][state->pulseHistoryPtr[sensor]].timestamp = timestamp;
  state->pulseHistory[sensor][state->pulseHistoryPtr[sensor]].width = width;

  state->pulseHistoryPtr[sensor] += 1;

  // As soon as one of the history buffers is full, run the synchronization algorithm!
  if (state->pulseHistoryPtr[sensor] >= PULSE_PROCESSOR_HISTORY_LENGTH) {
    static uint32_t syncTimes[PULSE_PROCESSOR_HISTORY_LENGTH];
    size_t nSyncTimes = 0;

    for (int i=0; i<PULSE_PROCESSOR_HISTORY_LENGTH; i++) {
      if (findSyncTime(state->pulseHistory[i], &syncTimes[nSyncTimes])) {
        nSyncTimes += 1;
      }

      if (getSystemSyncTime(syncTimes, nSyncTimes, &state->currentSync0)) {
        state->synchronized = true;
        state->lastSync = state->currentSync0;
        state->currentSync1 = state->currentSync0 + SYNC_SEPARATION;
        break;
      }
    }

    resetPulseHistory(state);
  }
}

static bool isSweep(pulseProcessor_t *state, unsigned int timestamp, int width)
{
  uint32_t delta = TS_DIFF(timestamp, state->lastSync);
  return ((delta > SYNC_MAX_SEPARATION) && (delta < (FRAME_LENGTH - (2*SYNC_MAX_SEPARATION)))) || (width < SWEEP_MAX_WIDTH);
}

TESTABLE_STATIC bool isSync(pulseProcessor_t *state, unsigned int timestamp, int width)
{
  uint32_t delta = TS_DIFF(timestamp, state->currentSync0);
  int deltaModulo = delta % FRAME_LENGTH;

  // We expect a modulo close to 0, detect and handle wrapping around FRAME_LENGTH
  if (deltaModulo > (FRAME_LENGTH/2)) {
    deltaModulo -= FRAME_LENGTH;
  }

  if ((deltaModulo > -1*MAX_FRAME_LENGTH_NOISE) && (deltaModulo < SYNC_MAX_SEPARATION)) {
    return true;
  }
  return false;
}

/**
 * @brief Get the Base Station Id
 * 
 * This function looks at the pulse position modulo FRAME_LENGTH in relation to
 * the latest sync0. Values ~0 means a Sync0 pulse, ~19200 means a Sync1.
 * 
 * @param state State of the pulse processor
 * @param timestamp Timestamp of the syn to process
 * @return 0 for Sync0, 1 for Sync1
 */
static int getBaseStationId(pulseProcessor_t *state, unsigned int timestamp) {
  int baseStation = 0;

  uint32_t delta = TS_DIFF(timestamp, state->currentSync0);
  int deltaModulo = delta % FRAME_LENGTH;

  // We expect a modulo close to 0, detect and handle wrapping around FRAME_LENGTH
  if (deltaModulo > (FRAME_LENGTH/2)) {
    deltaModulo -= FRAME_LENGTH;
  }

  if (deltaModulo > (SYNC_SEPARATION/2)) {
    baseStation = 1;
  }

  return baseStation;
}

static SweepDirection getAxis(int width) {
  SweepDirection result = sweepDirection_j;

  if ((((width-SYNC_BASE_WIDTH)/SYNC_DIVIDER)&0x01) != 0) {
    result = sweepDirection_k;
  }

  return result;
}

static bool isSweepActiveThisFrame(int width) {
  return (((width-SYNC_BASE_WIDTH)/SYNC_DIVIDER)&0x04) == 0;
}

static void storeSweepData(pulseProcessor_t *state, int sensor, unsigned int timestamp) {
  if (state->sweeps[sensor].state == sweepStorageStateWaiting) {
    state->sweeps[sensor].timestamp = timestamp;
    state->sweeps[sensor].state = sweepStorageStateValid;
  } else {
    state->sweeps[sensor].state = sweepStorageStateError;
  }

  state->sweepDataStored = true;
}

static void resetSweepData(pulseProcessor_t *state) {
  for (size_t sensor = 0; sensor < PULSE_PROCESSOR_N_SENSORS; sensor++) {
    state->sweeps[sensor].state = sweepStorageStateWaiting;
  }
  state->sweepDataStored = false;
}

static void resetSynchronization(pulseProcessor_t *state)
{
  resetSweepData(state);
  state->synchronized = false;
}

static bool processPreviousFrame(pulseProcessor_t *state, pulseProcessorResult_t result[], int *baseStation, int *axis) {
  bool anglesMeasured = false;

  if (state->sweepDataStored) {
    for (size_t sensor = 0; sensor < PULSE_PROCESSOR_N_SENSORS; sensor++) {
      if (state->sweeps[sensor].state == sweepStorageStateValid) {
        int delta = TS_DIFF(state->sweeps[sensor].timestamp, state->currentSync);
        if (delta < FRAME_LENGTH) {
          float angle = (delta - SWEEP_CENTER)*(float)M_PI/FRAME_LENGTH;

          *baseStation = state->currentBaseStation;
          *axis = state->currentAxis;

          result[sensor].angles[state->currentBaseStation][state->currentAxis] = angle;
          result[sensor].validCount++;

          anglesMeasured = true;
        }
      }
    }

    resetSweepData(state);
  }

  return anglesMeasured;
}

static void storeSyncData(pulseProcessor_t *state, unsigned int timestamp, unsigned int width) {
  int baseStation = getBaseStationId(state, timestamp);
  if (0 == baseStation) {
    state->currentSync0 = timestamp;
  } else {
    state->currentSync1 = timestamp;
  }

  state->lastSync = timestamp;

  if (isSweepActiveThisFrame(width)) {
    state->currentBaseStation = baseStation;
    state->currentAxis = getAxis(width);
    state->currentSync = timestamp;
  }
}

TESTABLE_STATIC bool isNewSync(uint32_t timestamp, uint32_t lastSync) {
  const uint32_t min = (1 << TIMESTAMP_BITWIDTH) - SENSOR_MAX_DISPERTION;
  const uint32_t max = SENSOR_MAX_DISPERTION;

  uint32_t diff = TS_DIFF(timestamp, lastSync);
  return (diff > max) && (diff < min);
}

static bool processSync(pulseProcessor_t *state, unsigned int timestamp, unsigned int width, pulseProcessorResult_t angles[], int *baseStation, int *axis) {
  bool anglesMeasured = false;

  if (isNewSync(timestamp, state->lastSync)) {
    if (isSync(state, timestamp, width)) {
      anglesMeasured = processPreviousFrame(state, angles, baseStation, axis);
      storeSyncData(state, timestamp, width);
    } else {
      // Expected a sync but something is wrong, re-synchronize.
      resetSynchronization(state);
    }
  }

  return anglesMeasured;
}

static bool processWhenSynchronized(pulseProcessor_t *state, int sensor, unsigned int timestamp, unsigned int width, pulseProcessorResult_t angles[], int *baseStation, int *axis) {
  bool anglesMeasured = false;

  if (isSweep(state, timestamp, width)) {
    storeSweepData(state, sensor, timestamp);
  } else {
    anglesMeasured = processSync(state, timestamp, width, angles, baseStation, axis);
  }

  return anglesMeasured;
}


bool pulseProcessorProcessPulse(pulseProcessor_t *state, int sensor, unsigned int timestamp, unsigned int width, pulseProcessorResult_t angles[], int *baseStation, int *axis)
{
  bool anglesMeasured = false;

  if (!state->synchronized) {
    synchronize(state, sensor, timestamp, width);
  } else {
    anglesMeasured = processWhenSynchronized(state, sensor, timestamp, width, angles, baseStation, axis);
  }

  return anglesMeasured;
}


/**
 * @brief Find the timestamp of a SYNC0 pulse detectable in pulseHistory
 *
 * @param pulseHistory PULSE_PROCESSOR_HISTORY_LENGTH pulses
 * @param[out] foundSyncTime Timestamp of the fist Sync0 written in this variable
 * @return true if the Sync0 was found
 * @return false if no Sync0 found
 */
TESTABLE_STATIC bool findSyncTime(const pulseProcessorPulse_t pulseHistory[], uint32_t *foundSyncTime)
{
  int nFound = 0;
  bool wasSweep = false;
  uint32_t foundTimes[2];
  
  for (int i=0; i<PULSE_PROCESSOR_HISTORY_LENGTH; i++) {
    if (wasSweep && pulseHistory[i].width > SWEEP_MAX_WIDTH) {
      foundTimes[nFound] = pulseHistory[i].timestamp;

      nFound++;
      if (nFound == 2) {
        break;
      }
    }
    
    if (pulseHistory[i].width < SWEEP_MAX_WIDTH) {
      wasSweep = true;
    } else {
      wasSweep = false;
    }
  }

  *foundSyncTime = foundTimes[0];

  uint32_t delta = TS_DIFF(foundTimes[1], foundTimes[0]);

  return (nFound == 2 && delta < (FRAME_LENGTH + MAX_FRAME_LENGTH_NOISE) && delta > (FRAME_LENGTH - MAX_FRAME_LENGTH_NOISE));
}

/**
 * @brief Get the System Sync time from sampled Sync0 time from multiple sensors
 *
 * This function places the Sync0 timestamps modulo the lighthouse V1 frame length
 * to estimate if they can possibly be coming from the same lighthouse system.
 * This allows to check that the sampling done on multiple receiving sensor is
 * consistent.
 *
 * @param syncTimes Array of Sync0 timestamps
 * @param nSyncTimes Number of timestamps in syncTimes array
 * @param[out] syncTime Pointer to the variable where the resulting sync time is written
 * @return true If an acceptable sync time could be calculated
 * @return false If the sampled Sync0 timestamps do not make sense
 */
TESTABLE_STATIC bool getSystemSyncTime(const uint32_t syncTimes[], size_t nSyncTimes, uint32_t *syncTime)
{
  if (nSyncTimes == 0) {
    return false;
  }

  // Detect if samples are wrapping
  // If the samples are wrapping, all samples bellow TIMESTAMP_MAX/2 will
  // be pushed by (1<<TIMESTAMP_BITWIDTH) to correct the wrapping
  bool isWrapping = false;
  int wref = syncTimes[0];
  for (size_t i=0; i<nSyncTimes; i++) {
    if (abs(wref - (int)syncTimes[i]) > (TIMESTAMP_MAX/2)) {
      isWrapping = true;
    }
  }

  int32_t differenceSum = 0;
  int32_t reference = syncTimes[0] % FRAME_LENGTH;
  if (isWrapping && syncTimes[0] < (TIMESTAMP_MAX/2)) {
    reference = (syncTimes[0] + (1<<TIMESTAMP_BITWIDTH)) % FRAME_LENGTH;
  }

  int minDiff = INT32_MAX;
  int maxDiff = INT32_MIN;

  for (size_t i=1; i<nSyncTimes; i++) {
    int diff;
    
    if (isWrapping && (syncTimes[i] < (TIMESTAMP_MAX/2))) {
      diff = ((syncTimes[i] + (1<<TIMESTAMP_BITWIDTH)) % FRAME_LENGTH) - reference;
    } else {
      diff = (syncTimes[i] % FRAME_LENGTH) - reference;
    }
     

    if (diff < minDiff) {
      minDiff = diff;
    }
    if (diff > maxDiff) {
      maxDiff = diff;
    }

    differenceSum += diff;
  }

  if ((maxDiff - minDiff) > MAX_FRAME_LENGTH_NOISE) {
    return false;
  }

  *syncTime = ((int)syncTimes[0] + ((int)differenceSum / (int)nSyncTimes)) & (TIMESTAMP_MAX);
  

  return true;
}
