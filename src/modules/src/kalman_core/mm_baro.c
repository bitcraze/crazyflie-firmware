/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--'  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2025 Bitcraze AB
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
 */

#include "mm_baro.h"

void kalmanCoreUpdateWithBaro(kalmanCoreData_t *this, const kalmanCoreParams_t *params, float baroPressurePa, bool quadIsFlying)
{
  float h[KC_STATE_DIM] = {0};
  arm_matrix_instance_f32 H = {1, KC_STATE_DIM, h};

  if (!quadIsFlying || this->baroReferencePressure < 1.0f) {
    this->baroReferencePressure = baroPressurePa;
  }

  float z = this->S[KC_STATE_Z];
  float pRef = this->baroReferencePressure;
  float base = 1.0f - 0.0065f * z / 298.15f;
  float pPred = pRef * powf(base, 5.2561f);

  // Jacobian dp/dz
  h[KC_STATE_Z] = pRef * 5.2561f * (-0.0065f / 298.15f) * powf(base, 4.2561f);

  float innovation = baroPressurePa - pPred;
  kalmanCoreScalarUpdate(this, &H, innovation, params->measNoiseBaro);
}
