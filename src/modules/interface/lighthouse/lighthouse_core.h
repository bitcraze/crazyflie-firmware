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

#include "pulse_processor.h"

typedef struct {
  bool isSyncFrame;
  pulseProcessorFrame_t data;
} lighthouseUartFrame_t;

typedef struct {
  int sampleCount;
  int hitCount;
} lighthouseBsIdentificationData_t;

void lighthouseCoreInit();
void lighthouseCoreTask(void *param);

/**
 * @brief Set calibration data for one base station of the system
 *
 * @param baseStation   The id of the base station
 * @param calibration   Pointer to calibration data
 */
void lighthouseCoreSetCalibrationData(const uint8_t baseStation, const lighthouseCalibration_t* calibration);

/**
 * @brief Set system type
 *
 * @param type System type
 */
void lighthouseCoreSetSystemType(const lighthouseBaseStationType_t type);

typedef enum {
  lh_led_off = 0,
  lh_led_slow_blink = 1,
  lh_led_fast_blink = 2,
  lh_led_on = 3,
} lighthouseCoreLedState_t;

/**
 * @brief Set LEDs on the lighthouse deck
 *
 * @param red  State of the red LED
 * @param orange State of the orange LED
 * @param green State of the green LED
 */
void lighthouseCoreSetLeds(lighthouseCoreLedState_t red, lighthouseCoreLedState_t orange, lighthouseCoreLedState_t green);

void lighthouseCoreLedTimer();
