/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
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
 */


#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"

#include "log.h"
#include "param.h"

#define DEBUG_MODULE "LEDCYCLE"

void appMain()
{
  DEBUG_PRINT("Starting RGB color cycling app...\n");

  paramVarId_t idRgb = paramGetVarId("led_deck_ctrl", "rgb888");

  uint8_t r = 0, g = 0, b = 0;
  int step = 0;

  TickType_t lastWakeTime = xTaskGetTickCount();
  while(1) {
    // Cycle through 4 phases, each with 256 steps
    int phase = step / 256;
    int value = step % 256;

    switch(phase) {
      case 0:  // G: 0→255, R: 255→0
        r = 255 - value;
        g = value;
        b = 0;
        break;
      case 1:  // B: 0→255, G: 255→0
        r = 0;
        g = 255 - value;
        b = value;
        break;
      case 2:  // R: 0→255, B: 255→0
        r = value;
        g = 0;
        b = 255 - value;
        break;
    }

    // Pack into uint32: 0x00RRGGBB (compatible with RGB888 standard)
    uint32_t rgb_value = ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
    paramSetInt(idRgb, rgb_value);

    step = (step + 1) % (256 * 3);  // Loop through all 3 phases

    vTaskDelayUntil(&lastWakeTime, M2T(3));
  }
}
