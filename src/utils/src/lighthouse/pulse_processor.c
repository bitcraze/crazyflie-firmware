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


/**
 * @brief Apply calibration data to the raw angles and write it to the correctedAngles member.
 * If no calibration data is available, the raw angles are simply copied to correctedAngles.
 *
 * Note: Calibration is only applied for V1 base stations.
 *
 * @param state State that contains the calibration data
 * @param angles The raw and calibrated angles
 * @param baseStation The base station in question
 */
void pulseProcessorApplyCalibration(pulseProcessor_t *state, pulseProcessorResult_t* angles, int baseStation){
  const lighthouseCalibration_t* calibrationData = &state->bsCalibration[baseStation];
  // TODO krri Only apply calibration to V1 systems. We have calibration data for V2 systems as well,
  // we just have to figure out how to use it.
  const bool doApplyCalibration = calibrationData->valid && (lighthouseBsTypeV1 == angles->measurementType);

  for (int sensor = 0; sensor < PULSE_PROCESSOR_N_SENSORS; sensor++) {
    pulseProcessorBaseStationMeasuremnt_t* bsMeasurement = &angles->sensorMeasurementsLh1[sensor].baseStatonMeasurements[baseStation];
    if (doApplyCalibration) {
      lighthouseCalibrationApply(calibrationData, bsMeasurement->angles, bsMeasurement->correctedAngles);
    } else {
      lighthouseCalibrationApplyNothing(bsMeasurement->angles, bsMeasurement->correctedAngles);
    }
  }
}

/**
 * @brief Clear the result struct for one base station
 *
 * @param angles The result struct to clear
 * @param baseStation The base station
 */
void pulseProcessorClear(pulseProcessorResult_t* angles, int baseStation)
{
  for (size_t sensor = 0; sensor < PULSE_PROCESSOR_N_SENSORS; sensor++) {
    angles->sensorMeasurementsLh1[sensor].baseStatonMeasurements[baseStation].validCount = 0;
    angles->sensorMeasurementsLh2[sensor].baseStatonMeasurements[baseStation].validCount = 0;
  }
}
