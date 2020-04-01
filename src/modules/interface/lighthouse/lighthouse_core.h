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
 * lighthouse_core.h - central part of the lighthouse positioning system
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  bool isSyncFrame;
  uint8_t sensor;

  // V1 base station data --------
  uint32_t timestamp;
  uint16_t width;

  // V2 base station data --------
  uint32_t beamData;
  uint32_t offset;
  bool channelFound;
  // Channel is zero indexed (0-15) here, while it is one indexed in the base station config (1 - 16)
  uint8_t channel;
  uint8_t slowbit;
} lighthouseUartFrame_t;

void lighthouseCoreTask(void *param);
