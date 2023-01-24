/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2023 Bitcraze AB
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

#pragma once

#include <stdint.h>
#include "imu_types.h"

typedef struct {
  Axis3f sum;
  uint32_t count;
  float conversionFactor;

  Axis3f subSample;
} Axis3fSubSampler_t;

/**
 * @brief Initialize sub sampler
 *
 * @param this  Pointer to sub sampler
 * @param conversionFactor  Conversion factor used for unit conversion.
 */
void axis3fSubSamplerInit(Axis3fSubSampler_t* this, const float conversionFactor);

/**
 * @brief Accumulate a sample
 *
 * @param this  Pointer to sub sampler
 * @param sample  The sample to accumulate
 */
void axis3fSubSamplerAccumulate(Axis3fSubSampler_t* this, const Axis3f* sample);

/**
 * @brief Compute the sub sample, uses simple averaging of samples. The sub sample is multiplied with the conversion
 * factor and the result is stored in the subSample member of the Axis3fSubSampler_t.
 *
 * @param this  Pointer to sub sampler
 * @return Axis3f*  Pointer to the resulting sub sample
 */
Axis3f* axis3fSubSamplerFinalize(Axis3fSubSampler_t* this);
