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
#define SENSOR_MAX_DISPERTION 10
#define MAX_FRAME_LENGTH_NOISE 40

// Utility functions and macros
#define TS_DIFF(X, Y) ((X-Y)&((1<<TIMESTAMP_BITWIDTH)-1))

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