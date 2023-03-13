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

#include <string.h>
#include "axis3fSubSampler.h"

void axis3fSubSamplerInit(Axis3fSubSampler_t* this, const float conversionFactor) {
  memset(this, 0, sizeof(Axis3fSubSampler_t));
  this->conversionFactor = conversionFactor;
}

void axis3fSubSamplerAccumulate(Axis3fSubSampler_t* this, const Axis3f* sample) {
  this->sum.x += sample->x;
  this->sum.y += sample->y;
  this->sum.z += sample->z;

  this->count++;
}

Axis3f* axis3fSubSamplerFinalize(Axis3fSubSampler_t* this) {
  if (this->count > 0) {
    this->subSample.x = this->sum.x * this->conversionFactor / this->count;
    this->subSample.y = this->sum.y * this->conversionFactor / this->count;
    this->subSample.z = this->sum.z * this->conversionFactor / this->count;

    // Reset
    this->count = 0;
    this->sum = (Axis3f){.axis={0}};
  }

  return &this->subSample;
}
