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

/* Voltage reference types for the analogReference() function. */
#define DEFAULT 0
#define VREF    3.0

void adcInit(void);

uint16_t analogRead(uint32_t pin);

void analogReference(uint8_t type);

void analogReadResolution(uint8_t bits);

/*
 * Read the voltage on a deck pin.
 * @param[in] pin   deck pin to measure.
 * @return          voltage in volts
 */
float analogReadVoltage(uint32_t pin);

#endif
