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
 *
 *
 * led_deck_controller.h - Generic LED Deck Controller
 *
 * Provides a unified interface for controlling LED decks through a generic
 * parameter namespace, abstracting hardware differences between different
 * LED deck types.
 *
 * LED deck drivers register callback functions to receive color updates from this
 * controller. The controller provides a simple parameter interface for setting LED
 * colors.
 *
 * Usage:
 *   - LED decks call ledDeckRegisterHandler() during initialization
 *   - Users set led_deck_ctrl.* parameters to control LEDs
 *   - Last write wins: generic parameters and deck-specific parameters can
 *     be used interchangeably, with the most recent write taking effect
 */

#ifndef __LED_DECK_CONTROLLER_H__
#define __LED_DECK_CONTROLLER_H__

#include <stdint.h>

/**
 * @brief LED deck handler definition
 *
 * LED deck drivers define this structure and register it during initialization.
 */
typedef struct {
  /**
   * @brief Callback to set LED color
   * @param rgb888 Array of 3 bytes: [R, G, B] each 0-255
   */
  void (*setColor)(const uint8_t *rgb888);
} ledDeckHandlerDef_t;

/**
 * @brief Register an LED deck handler
 *
 * Should be called by LED deck drivers during their init() function.
 *
 * @param handler Pointer to the handler definition
 */
void ledDeckRegisterHandler(const ledDeckHandlerDef_t* handler);

#endif // __LED_DECK_CONTROLLER_H__
