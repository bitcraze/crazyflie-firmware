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
 * deck_analog.h - Arduino-compatible analog input API
 */

#ifndef __DECK_ANALOG_H__
#define __DECK_ANALOG_H__

#include <stdint.h>

/*
 * The Deck port GPIO pins, as named by the deck port / expansion port / expansion breakout documentation on bitcraze.io.
 * Sequenced according to the gpioMapping struct in deck_analog.c, so that these can be used for lookup in the struct.
 *
 * TODO: Should be shared across the deck API.
 */
#define DECK_PIN_RX1   1
#define DECK_PIN_TX1   2
#define DECK_PIN_SDA   3
#define DECK_PIN_SCL   4
#define DECK_PIN_IO1   5
#define DECK_PIN_IO2   6
#define DECK_PIN_IO3   7
#define DECK_PIN_IO4   8
#define DECK_PIN_TX2   9
#define DECK_PIN_RX2  10
#define DECK_PIN_SCK  11
#define DECK_PIN_MISO 12
#define DECK_PIN_MOSI 13

/* Voltage reference types for the analogReference() function. */
#define DEFAULT ((uint8_t)00)

void adcInit(void);

uint16_t analogRead(uint32_t pin);

void analogReference(uint8_t type);

#endif
