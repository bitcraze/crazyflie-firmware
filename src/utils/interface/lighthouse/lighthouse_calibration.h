#pragma once

#include "ootx_decoder.h"
#include "lighthouse_types.h"

/**
 * @brief Initialize calibration structure from baseStation ootx frame
 *
 * @param calib Calibration  object to initialize, calib->valid will be set to true
 * @param frame ootx frame received from the baseStation
 */
void lighthouseCalibrationInitFromFrame(lighthouseCalibration_t *calib, struct ootxDataFrame_s *frame);

/**
 * @brief Apply baseStation calibration to the two received angles for LH 1
 *
 * @param calib Calibration object to use
 * @param rawAngles Array containing the two raw measured angles
 * @param correctedAngles Array containing the two corrected angles after applying calibration
 */
void lighthouseCalibrationApplyV1(const lighthouseCalibration_t* calib, const float* rawAngles, float* correctedAngles);

/**
 * @brief Apply baseStation calibration to the two received angles for LH 2
 *
 * @param calib Calibration object to use
 * @param rawAngles Array containing the two raw measured angles
 * @param correctedAngles Array containing the two corrected angles after applying calibration
 */
void lighthouseCalibrationApplyV2(const lighthouseCalibration_t* calib, const float* rawAngles, float* correctedAngles);

/**
 * @brief Apply no baseStation calibration to the two received angles, that is copy the raw angles
 *
 * @param rawAngles i/j raw angles measured
 * @param correctedAngles i/j will be same as the raw angles
 */
void lighthouseCalibrationApplyNothing(const float rawAngles[2], float correctedAngles[2]);

/**
 * @brief Predict the measured sweep angle based on a position for a lighthouse 1 rotor. The position is relative to the rotor reference frame.
 * @param x meters
 * @param y meters
 * @param z meters
 * @param t Tilt of the light plane in radians - not used in LH1, will always use 0
 * @param calib Calibration data for the rotor
 * @return float The predicted uncompensated sweep angle of the rotor
 */
float lighthouseCalibrationMeasurementModelLh1(const float x, const float y, const float z, const float t, const lighthouseCalibrationSweep_t* calib);

/**
 * @brief Predict the measured sweep angle based on a position for a lighthouse 2 rotor. The position is relative to the rotor reference frame.
 * @param x meters
 * @param y meters
 * @param z meters
 * @param t Tilt of the light plane in radians
 * @param calib Calibration data for the rotor
 * @return float The predicted uncompensated sweep angle of the rotor
 */
float lighthouseCalibrationMeasurementModelLh2(const float x, const float y, const float z, const float t, const lighthouseCalibrationSweep_t* calib);
