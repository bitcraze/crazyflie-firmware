/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 - 2020 Bitcraze AB
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
 * pulse_processor.c - pulse decoding for lighthouse V1 base stations
 *
 */

#include "pulse_processor.h"
#include "pulse_processor_v1.h"
#include "pulse_processor_v2.h"
#include "cf_math.h"
#include "autoconf.h"


/**
 * @brief Apply calibration data to the raw angles and write it to the correctedAngles member.
 * If no calibration data is available, the raw angles are simply copied to correctedAngles.
 *
 * @param state State that contains the calibration data
 * @param angles The raw and calibrated angles
 * @param baseStation The base station in question
 */
bool pulseProcessorApplyCalibration(pulseProcessor_t *state, pulseProcessorResult_t* angles, int baseStation){
  const lighthouseCalibration_t* calibrationData = &state->bsCalibration[baseStation];
  const bool doApplyCalibration = calibrationData->valid;

  pulseProcessorBaseStationMeasurement_t* bsMeasurement = &angles->baseStationMeasurementsLh1[baseStation];
  if (lighthouseBsTypeV2 == angles->measurementType) {
    bsMeasurement = &angles->baseStationMeasurementsLh2[baseStation];
  }

  for (int sensor = 0; sensor < PULSE_PROCESSOR_N_SENSORS; sensor++) {
    pulseProcessorSensorMeasurement_t* measurement = &bsMeasurement->sensorMeasurements[sensor];
    if (doApplyCalibration) {
      if (lighthouseBsTypeV2 == angles->measurementType) {
        lighthouseCalibrationApplyV2(calibrationData, measurement->angles, measurement->correctedAngles);
      } else {
        lighthouseCalibrationApplyV1(calibrationData, measurement->angles, measurement->correctedAngles);
      }
    } else {
      lighthouseCalibrationApplyNothing(measurement->angles, measurement->correctedAngles);
    }
  }

  return doApplyCalibration;
}

/**
 * Clear angles information when we know that data became old when it wasn't updated anymore.
 * For example when base stations or sensors are hidden for crazyflie
 *
 * @param appState State that contains the calibration data
 * @param angles The raw and calibrated angles
 * @param baseStation The base station in question
 */
void pulseProcessorClearOutdated(pulseProcessor_t *appState, pulseProcessorResult_t* angles, int baseStation) {
  // Repeated sweep from the same baseStation. So in theory we did a cycle, so we should have had all base stations.
  // If not, cleanup the baseStation that we didn't receive.
  if(appState->receivedBsSweep[baseStation]) {
    for(int bs=0; bs != CONFIG_DECK_LIGHTHOUSE_MAX_N_BS; bs++){
      if(!appState->receivedBsSweep[bs]){
        pulseProcessorClear(angles, bs);
      }
      appState->receivedBsSweep[bs] = false;
    }
  }
  appState->receivedBsSweep[baseStation] = true;
}

/**
 * @brief Update the information about what angles and what base stations are having correct data
 *
 * @param angles The result struct to clear
 * @param baseStation The base station
 */
void processValidAngles(pulseProcessorResult_t* angles, int baseStation)
{
  switch(angles->measurementType) {
    case lighthouseBsTypeV1:
      pulseProcessorV1ProcessValidAngles(angles, baseStation);
      break;
    default:
      // Do nothing
      break;
  }
}

/**
 * @brief Clear the result struct for one base station when the sensor data invalidated
 *
 * @param angles The result struct to clear
 * @param baseStation The base station
 */
void pulseProcessorClear(pulseProcessorResult_t* angles, int baseStation)
{
  for (size_t sensor = 0; sensor < PULSE_PROCESSOR_N_SENSORS; sensor++) {
    angles->baseStationMeasurementsLh1[baseStation].sensorMeasurements[sensor].validCount = 0;
    angles->baseStationMeasurementsLh2[baseStation].sensorMeasurements[sensor].validCount = 0;
  }
  processValidAngles(angles, baseStation);
}

/**
 * @brief Clear result struct when the sensor data is invalidated
 *
 * @param angles
 */
void pulseProcessorAllClear(pulseProcessorResult_t* angles)
{
  for (int baseStation = 0; baseStation < CONFIG_DECK_LIGHTHOUSE_MAX_N_BS; baseStation++) {
    for (size_t sensor = 0; sensor < PULSE_PROCESSOR_N_SENSORS; sensor++) {
      angles->baseStationMeasurementsLh1[baseStation].sensorMeasurements[sensor].validCount = 0;
      angles->baseStationMeasurementsLh2[baseStation].sensorMeasurements[sensor].validCount = 0;
    }
    processValidAngles(angles, baseStation);
  }
}

/**
 * @brief Clear the result struct for one base station when the data is processed and converted to measurements
 *
 * @param angles The result struct to clear
 * @param baseStation The base station
 */
void pulseProcessorProcessed(pulseProcessorResult_t* angles, int baseStation)
{
  processValidAngles(angles, baseStation);

  for (size_t sensor = 0; sensor < PULSE_PROCESSOR_N_SENSORS; sensor++) {
    angles->baseStationMeasurementsLh1[baseStation].sensorMeasurements[sensor].validCount = 0;
    angles->baseStationMeasurementsLh2[baseStation].sensorMeasurements[sensor].validCount = 0;
  }
}

uint8_t pulseProcessorAnglesQuality() {
  return MAX(pulseProcessorV1AnglesQuality(), pulseProcessorV2AnglesQuality());
}
