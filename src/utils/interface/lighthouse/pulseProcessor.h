#pragma once

#include <stdbool.h>
#include <stdint.h>

#define TIMESTAMP_BITWIDTH 29



typedef struct pulseProcessor_s {
  int currentFrame;
  uint32_t lastSync;
  uint32_t currentSync;
  int currentBs;
  int currentAxis;
} pulseProcessor_t;

// If returns true, the angle, base station and direction are written
bool processPulse(pulseProcessor_t *state, unsigned int timestamp, unsigned int width, float *angle, int *baseStation, int *axis);