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
  DEBUG_PRINT("Starting RGBW color cycling app...\n");

  paramVarId_t idRgbw = paramGetVarId("hprgbw", "rgbw8888");

  // Subscribe to thermal throttle logs
  logVarId_t idDeckTemp = logGetVarId("hprgbw", "deckTemp");
  logVarId_t idThrottlePct = logGetVarId("hprgbw", "throttlePct");

  uint8_t r = 0, g = 0, b = 0, w = 0;
  int step = 0;

  uint32_t lastThermalCheck = xTaskGetTickCount();
  const uint32_t thermalCheckInterval = M2T(100); // Check every 100ms

  while(1) {
    // Cycle through 4 phases, each with 256 steps
    int phase = step / 256;
    int value = step % 256;

    switch(phase) {
      case 0:  // G: 0→255, R: 255→0
        r = 255 - value;
        g = value;
        b = 0;
        w = 0;
        break;
      case 1:  // B: 0→255, G: 255→0
        r = 0;
        g = 255 - value;
        b = value;
        w = 0;
        break;
      case 2:  // W: 0→255, B: 255→0
        r = 0;
        g = 0;
        b = 255 - value;
        w = value;
        break;
      case 3: // R: 0→255, W: 255→0 (white fades into red)
        r = value;
        g = 0;
        b = 0;
        w = 255 - value;
        break;
    }

    // Pack into uint32: 0xWWRRGGBB (compatible with RGB888 standard)
    uint32_t rgbw_value = ((uint32_t)w << 24) | ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
    paramSetInt(idRgbw, rgbw_value);

    // Check for thermal throttling periodically (every 100ms)
    if (xTaskGetTickCount() - lastThermalCheck >= thermalCheckInterval) {
      uint8_t throttlePct = logGetUint(idThrottlePct);
      if (throttlePct) {
        uint8_t deckTemp = logGetUint(idDeckTemp);
        DEBUG_PRINT("WARNING: Thermal throttling active! Temp: %d°C, Throttle: %d%%\n", deckTemp, throttlePct);
      }
      lastThermalCheck = xTaskGetTickCount();
    }

    step = (step + 1) % (256 * 4);  // Loop through all 4 phases

    vTaskDelay(M2T(3));
  }
}
