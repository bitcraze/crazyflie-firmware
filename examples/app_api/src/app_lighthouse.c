/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2020 Bitcraze AB
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
 * app_lighthouse.c - lighthouse example
 */


#include "lighthouse_core.h"
#include "lighthouse_position_est.h"

baseStationGeometry_t sampleGeoData[PULSE_PROCESSOR_N_BASE_STATIONS]  = {
  {.valid = true, .origin = {-1.958483,  0.542299,  3.152727, }, .mat = {{0.79721498, -0.004274, 0.60368103, }, {0.0, 0.99997503, 0.00708, }, {-0.60369599, -0.005645, 0.79719502, }, }},
  {.valid = true, .origin = {1.062398, -2.563488,  3.112367, }, .mat = {{0.018067, -0.999336, 0.031647, }, {0.76125097, 0.034269, 0.64755201, }, {-0.648206, 0.012392, 0.76136398, }, }},
};

lighthouseCalibration_t sampleCalibrationData[PULSE_PROCESSOR_N_BASE_STATIONS] = {
  { // Base station 0
    .valid = true,
    .sweep = {
      {.tilt = -0.047058, .phase = 0.0, .curve = 0.052215, .gibphase = 2.087890, .gibmag = -0.003913, .ogeephase = 0.433105, .ogeemag = -0.049285},
      {.tilt = 0.048065, .phase = -0.005336, .curve = 0.122375, .gibphase = 2.097656, .gibmag = -0.003883, .ogeephase = 0.631835, .ogeemag = -0.034851},
    },
  },
  { // Base station 1
    .valid = true,
    .sweep = {
      {.tilt = -0.051208, .phase = 0.0, .curve = 0.011756, .gibphase = 2.136718, .gibmag = -0.006057, .ogeephase = 2.705078,},
      {.tilt = 0.045623, .phase = -0.004142, .curve = 0.104736, .gibphase = 2.349609, .gibmag = -0.003332, .ogeephase = 0.380859, .ogeemag = -0.240112,},
    },
  },
};

void appInitLighthouse() {
  lighthousePositionSetGeometryData(0, &sampleGeoData[0]);
  lighthousePositionSetGeometryData(1, &sampleGeoData[1]);
  lighthouseCoreSetCalibrationData(0, &sampleCalibrationData[0]);
  lighthouseCoreSetCalibrationData(1, &sampleCalibrationData[1]);
}
