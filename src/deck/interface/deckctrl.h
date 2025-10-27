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
 *
 * deckctrl.h - DeckCtrl backend API for deck initialization and control
 *
 * This API provides backend-specific context for DeckCtrl decks discovered
 * via the DeckCtrl discovery backend. Deck drivers can use this context to
 * communicate with their deck controller hardware over I2C.
 */
#pragma once

#include "deck.h"

/**
 * @brief Backend context for DeckCtrl decks
 *
 * This structure is populated by the DeckCtrl discovery backend and attached
 * to the DeckInfo->backendContext pointer. Deck drivers can access this to
 * determine their assigned I2C address and communicate with the deck controller.
 */
struct deckCtrlContext_s {
    uint8_t i2cAddress;  ///< I2C address assigned to this deck during discovery (0x44+)
};
typedef struct deckCtrlContext_s DeckCtrlContext;
