#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "ootx_decoder.h"
#include "lighthouse_calibration.h"

#define PULSE_PROCESSOR_N_SWEEPS 2
#define PULSE_PROCESSOR_N_BASE_STATIONS 2
#define PULSE_PROCESSOR_N_SENSORS 4
#define PULSE_PROCESSOR_HISTORY_LENGTH 8
#define PULSE_PROCESSOR_TIMESTAMP_BITWIDTH 29
#define PULSE_PROCESSOR_TIMESTAMP_MAX ((1<<PULSE_PROCESSOR_TIMESTAMP_BITWIDTH)-1)


enum pulseClass_e {unknown, sync0, sync1, sweep};

typedef struct {
  uint32_t timestamp;
  int width;
} pulseProcessorPulse_t;

typedef enum {
  sweepDirection_x = 0,
  sweepDirection_y = 1
} SweepDirection;

typedef enum {
  sweepStorageStateWaiting = 0,
  sweepStorageStateValid,
  sweepStorageStateError,
} SweepStorageState_t;

typedef struct pulseProcessor_s {
  bool synchronized;    // At true if we are currently syncthonized
  int basestationsSynchronizedCount;

  // Synchronization state
  pulseProcessorPulse_t pulseHistory[PULSE_PROCESSOR_N_SENSORS][PULSE_PROCESSOR_HISTORY_LENGTH];
  int pulseHistoryIdx[PULSE_PROCESSOR_N_SENSORS];


  // Sync pulse timestamp estimation
  uint32_t lastSync;        // Last sync seen
  uint64_t currentSyncSum;  // Sum of the timestamps of all the close-together sync
  int nSyncPulses;          // Number of sync pulses accumulated

  // Sync pulse timestamps
  uint32_t currentSync;   // Sync currently used for sweep phase measurement
  uint32_t currentSync0;  // Sync0 of the current frame
  uint32_t currentSync0Width;  // Width of sync0 in the current frame
  uint32_t currentSync1Width;  // Width of sync1 in the current frame

  uint32_t currentSync0X;
  uint32_t currentSync0Y;
  uint32_t currentSync1X;
  uint32_t currentSync1Y;

  float frameWidth[2][2];

  // Base station and axis of the current frame
  int currentBaseStation;
  SweepDirection currentAxis;

  // Sweep timestamps
  struct {
    uint32_t timestamp;
    SweepStorageState_t state;
  } sweeps[PULSE_PROCESSOR_N_SENSORS];
  bool sweepDataStored;

  ootxDecoderState_t ootxDecoder0;
  ootxDecoderState_t ootxDecoder1;

  lighthouseCalibration_t bsCalibration[PULSE_PROCESSOR_N_BASE_STATIONS];
} pulseProcessor_t;

typedef struct {
  float angles[PULSE_PROCESSOR_N_SWEEPS];
  float correctedAngles[PULSE_PROCESSOR_N_SWEEPS];
  int validCount;
} pulseProcessorBaseStationMeasuremnt_t;

typedef struct {
  pulseProcessorBaseStationMeasuremnt_t baseStatonMeasurements[PULSE_PROCESSOR_N_BASE_STATIONS];
} pulseProcessorSensorMeasurement_t;

typedef struct {
  pulseProcessorSensorMeasurement_t sensorMeasurements[PULSE_PROCESSOR_N_SENSORS];
} pulseProcessorResult_t;

/**
 * @brief Process pulse data from the lighthouse
 *
 * @param state
 * @param sensor
 * @param timestamp
 * @param width
 * @param angles
 * @param baseStation
 * @param axis
 * @return true, angle, base station and direction are written
 * @return false, no valid result
 */
bool pulseProcessorProcessPulse(pulseProcessor_t *state, int sensor, unsigned int timestamp, unsigned int width, pulseProcessorResult_t* angles, int *baseStation, int *axis);

/**
 * @brief Apply calibration correction to all angles of all sensors for a particular baseStation
 *
 * @param state
 * @param angles
 * @param baseStation
 */
void pulseProcessorApplyCalibration(pulseProcessor_t *state, pulseProcessorResult_t* angles, int baseStation);

/**
 * @brief Clear result struct
 *
 * @param angles
 * @param baseStation
 */
void pulseProcessorClear(pulseProcessorResult_t* angles, int baseStation);
