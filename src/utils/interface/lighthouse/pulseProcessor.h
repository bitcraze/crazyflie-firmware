#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#define PULSE_PROCESSOR_N_SENSORS 8
#define PULSE_PROCESSOR_HISTORY_LENGTH 8
#define TIMESTAMP_BITWIDTH 29
#define TIMESTAMP_MAX ((1<<TIMESTAMP_BITWIDTH)-1)


enum pulseClass_e {unknown, sync0, sync1, sweep};

typedef struct {
  uint32_t timestamp;
  int width;
} pulseProcessorPulse_t;

typedef struct pulseProcessor_s {
  bool synchronized;    // At true if we are currently syncthonized (ie. we have seen one short sweep)

  // Synchronization state
  pulseProcessorPulse_t pulseHistory[PULSE_PROCESSOR_N_SENSORS][PULSE_PROCESSOR_HISTORY_LENGTH];
  int pulseHistoryPtr[PULSE_PROCESSOR_N_SENSORS];


  // Sync pulse timestamp estimation
  uint32_t lastSync;        // Last sync seen
  uint64_t currentSyncSum;  // Sum of the timestamps of all the close-together sync
  int nSyncPulses;          // Number of sync pulses accumulated

  // Sync pulse timestamps
  uint32_t currentSync;   // Sync currently used for sweep phase measurment
  uint32_t currentSync0;  // Sync0 of the current frame
  uint32_t currentSync1;  // Sync1 of the current frame

  // Base station and axis of the current frame
  int currentBs;
  int currentAxis;

  // Sweep timestamps
  struct {
    uint32_t timestamp;
    bool valid;
    bool error;
  } sweeps[PULSE_PROCESSOR_N_SENSORS];

} pulseProcessor_t;

// If returns true, the angle, base station and direction are written
bool processPulse(pulseProcessor_t *state, unsigned int timestamp, unsigned int width, float *angle, int *baseStation, int *axis);

/**
 * @brief Find the timestamp of a SYNC0 pulse detectable in pulseHistory
 * 
 * @param pulseHistory PULSE_PROCESSOR_HISTORY_LENGTH pulses
 * @param[out] foundSyncTime Timestamp of the fist Sync0 written in this variable
 * @return true if the Sync0 was found
 * @return false if no Sync0 found
 */
bool findSyncTime(const pulseProcessorPulse_t pulseHistory[], uint32_t *foundSyncTime);

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
bool getSystemSyncTime(const uint32_t syncTimes[], int nSyncTimes, uint32_t *syncTime);
