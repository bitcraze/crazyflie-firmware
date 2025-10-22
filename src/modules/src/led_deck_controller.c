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

#include  <stddef.h>

#include "cfassert.h"
#include "param.h"
#include "led_deck_controller.h"

#define MAX_LED_DECK_HANDLERS 4

// Storage for registered handlers
static const ledDeckHandlerDef_t* handlers[MAX_LED_DECK_HANDLERS];
static uint8_t numHandlers = 0;

// Parameter storage
static uint32_t rgb888 = 0;

void ledDeckRegisterHandler(const ledDeckHandlerDef_t* handler) {
  ASSERT(handler != NULL);
  ASSERT(numHandlers < MAX_LED_DECK_HANDLERS);

  handlers[numHandlers++] = handler;
}

// Parameter callback - called when rgb888 parameter is updated
static void onRgb888Changed(void) {
  // RGB888 standard format: 0x00RRGGBB
  uint8_t rgb[3] = {
    (rgb888 >> 16) & 0xFF,  // Red (bits 16-23)
    (rgb888 >> 8) & 0xFF,   // Green (bits 8-15)
    rgb888 & 0xFF           // Blue (bits 0-7)
  };

  // Call all registered handlers
  for (int i = 0; i < numHandlers; i++) {
    if (handlers[i]->setColor) {
      handlers[i]->setColor(rgb);
    }
  }
}

/**
 * Generic LED deck control parameters
 *
 * Format: RGB888 standard (0x00RRGGBB)
 * Examples: 0xFF0000 = red, 0x00FF00 = green, 0x0000FF = blue
 */
PARAM_GROUP_START(led_deck_ctrl)
PARAM_ADD_WITH_CALLBACK(PARAM_UINT32, rgb888, &rgb888, onRgb888Changed)
PARAM_GROUP_STOP(led_deck_ctrl)
