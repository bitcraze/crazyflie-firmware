#include "pulseProcessor.h"

#include <math.h>

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
#define SENSOR_MAX_DISPERTION 10
#define MAX_FRAME_LENGTH_NOISE 40

// Utility functions and macros
#define TS_DIFF(X, Y) ((X-Y)&((1<<TIMESTAMP_BITWIDTH)-1))

// static bool resetSynchonization(pulseProcessor_t *state)
// {
//   state->synchronized = false;
//   memset(state->pulseHistoryPtr, 0, sizeof(state->pulseHistoryPtr));
// }

static void synchronize(pulseProcessor_t *state, int sensor, uint32_t timestamp, uint32_t width)
{
  state->pulseHistory[sensor][state->pulseHistoryPtr[sensor]].timestamp = timestamp;
  state->pulseHistory[sensor][state->pulseHistoryPtr[sensor]].width = width;

  state->pulseHistoryPtr[sensor] += 1;

  // As soon as one of the history buffer is full, run the syncrhonization algorithm!
  if (state->pulseHistoryPtr[sensor] == PULSE_PROCESSOR_HISTORY_LENGTH) {
    static uint32_t syncTimes[PULSE_PROCESSOR_HISTORY_LENGTH];
    int nSyncTimes = 0;

    for (int i=0; i<PULSE_PROCESSOR_HISTORY_LENGTH; i++) {
      if (findSyncTime(state->pulseHistory[i], &syncTimes[nSyncTimes])) {
        nSyncTimes += 1;
      }

      if (getSystemSyncTime(syncTimes, nSyncTimes, &state->currentSync0)) {
        state->synchronized = true;
        state->lastSync = state->currentSync0;
        state->currentSync1 = state->currentSync0 + SYNC_SEPARATION;
      }
    }
  }
}

static bool isSweep(pulseProcessor_t *state, unsigned int timestamp, int width)
{
  int delta = TS_DIFF(timestamp, state->lastSync);
  return ((delta > SYNC_MAX_SEPARATION) && (delta < (FRAME_LENGTH - (2*SYNC_MAX_SEPARATION)))) || (width < SWEEP_MAX_WIDTH);
}

static bool getAxis(int width)
{
  return (((width-SYNC_BASE_WIDTH)/SYNC_DIVIDER)&0x01) != 0;
}

static bool getSkip(int width)
{
  return (((width-SYNC_BASE_WIDTH)/SYNC_DIVIDER)&0x04) != 0;
}

bool processPulse(pulseProcessor_t *state, unsigned int timestamp, unsigned int width, float *angle, int *baseStation, int *axis)
{
  bool angleMeasured = false;

  if (!state->synchronized) {
    synchronize(state, 0, timestamp, width);
  } else {
    if (isSweep(state, timestamp, width)) {
      int delta = TS_DIFF(timestamp, state->currentSync);
      
      if (delta < FRAME_LENGTH) {
        *angle = (delta - SWEEP_CENTER)*(float)M_PI/FRAME_LENGTH;
        *baseStation = state->currentBs;
        *axis = state->currentAxis;
        angleMeasured = true;
      }

      state->currentSync = 0;
    } else {
      if (TS_DIFF(timestamp, state->lastSync) > SYNC_MAX_SEPARATION) {
        // This is sync0
        if (!getSkip(width)) {
          state->currentBs = 0;
          state->currentAxis = getAxis(width);
          state->currentSync = timestamp;
        }
      } else {
        // this is sync1
        if (!getSkip(width)) {
          state->currentBs = 1;
          state->currentAxis = getAxis(width);
          state->currentSync = timestamp;
        }
      }
      
      state->lastSync = timestamp;
    }
  }

  return angleMeasured;
}


bool findSyncTime(const pulseProcessorPulse_t pulseHistory[], uint32_t *foundSyncTime)
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

bool getSystemSyncTime(const uint32_t syncTimes[], int nSyncTimes, uint32_t *syncTime)
{
  if (nSyncTimes == 0) {
    return false;
  }

  // Detect if samples are wrapping
  // If the samples are wrapping, all samples bellow TIMESTAMP_MAX/2 will
  // be pushed by (1<<TIMESTAMP_BITWIDTH) to correct the wrapping
  bool isWrapping = false;
  int wref = syncTimes[0];
  for (int i=0; i<nSyncTimes; i++) {
    if (abs(wref - syncTimes[i]) > (TIMESTAMP_MAX/2)) {
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

  *syncTime = ((int)syncTimes[0] + (differenceSum / (nSyncTimes))) & (TIMESTAMP_MAX);
  

  return true;
}
