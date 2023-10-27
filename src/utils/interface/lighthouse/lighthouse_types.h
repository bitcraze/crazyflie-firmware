#pragma once

typedef struct {
  float phase;
  float tilt;
  float curve;
  float gibmag;
  float gibphase;
  // Lh2 extra params
  float ogeemag;
  float ogeephase;
} __attribute__((packed)) lighthouseCalibrationSweep_t;

typedef struct {
  lighthouseCalibrationSweep_t sweep[2];
  uint32_t uid;
  bool valid;
} __attribute__((packed)) lighthouseCalibration_t;

/**
 * @brief Generic function pointer type for a calibration measurement model.
 *        Predict the measured sweep angle based on a position for a lighthouse rotor. The position is relative to the rotor reference frame.
 * @param x meters
 * @param y meters
 * @param z meters
 * @param t Tilt of the light plane in radians
 * @param calib Calibration data for the rotor
 * @return float The predicted uncompensated sweep angle of the rotor
 *
 */
typedef float (*lighthouseCalibrationMeasurementModel_t)(const float x, const float y, const float z, const float t, const lighthouseCalibrationSweep_t* calib);
