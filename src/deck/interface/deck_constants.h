/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2015 Bitcraze AB
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
 * deck_contants.h - Arduino-compatible constant definition
 */

#ifndef __DECK_CONSTANTS_H__
#define __DECK_CONSTANTS_H__

// For true and false
#include <stdbool.h>

#include "stm32fxxx.h"

#define LOW 0x0
#define HIGH 0x1

#define INPUT           0x0
#define OUTPUT          0x1
#define INPUT_PULLUP    0x2
#define INPUT_PULLDOWN  0x3

/*
 * The Deck port GPIO pins, as named by the deck port / expansion port / expansion breakout documentation on bitcraze.io.
 * Sequenced according to the deckGPIOMapping struct, so that these can be used for lookup in the struct.
 */

// Type used to identify a pin in the deck API.
// id is used as an index in the deckGPIOMapping array
typedef struct {uint8_t id;} deckPin_t;

extern const deckPin_t DECK_GPIO_RX1;
extern const deckPin_t DECK_GPIO_TX1;
extern const deckPin_t DECK_GPIO_SDA;
extern const deckPin_t DECK_GPIO_SCL;
extern const deckPin_t DECK_GPIO_IO1;
extern const deckPin_t DECK_GPIO_IO2;
extern const deckPin_t DECK_GPIO_IO3;
extern const deckPin_t DECK_GPIO_IO4;
extern const deckPin_t DECK_GPIO_TX2;
extern const deckPin_t DECK_GPIO_RX2;
extern const deckPin_t DECK_GPIO_SCK;
extern const deckPin_t DECK_GPIO_MISO;
extern const deckPin_t DECK_GPIO_MOSI;


typedef const struct {
  uint32_t periph;
  GPIO_TypeDef* port;
  uint16_t pin;
  int8_t adcCh; /* -1 means no ADC available for this pin. */
} deckGPIOMapping_t;

extern deckGPIOMapping_t deckGPIOMapping[];

#endif
