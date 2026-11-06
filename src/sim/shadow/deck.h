/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2026 Bitcraze AB
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
 * deck.h - Shadow header for the Simmyflie.
 *
 * The mainline deck.h includes the deck pin and bus headers, and through them
 * the STM32 device headers. The sim has no decks. This header keeps the part
 * of its contract that the compiled files use (system.c and
 * estimator_kalman.c), implemented by src/sim/src/deck/deck.c.
 *
 * See docs/development/simmyflie/index.md for the Shadow header rules.
 */
#ifndef __DECK_H__
#define __DECK_H__

#include <stdbool.h>
/* system.c uses memcpy() without including string.h and gets it through
 * the hardware headers of the real deck.h. */
#include <string.h>

#include "estimator.h"

void deckInit(void);
bool deckTest(void);
StateEstimatorType deckGetRequiredEstimator(void);
bool deckGetRequiredLowInterferenceRadioMode(void);
bool deckGetRequiredKalmanEstimatorAttitudeReversionOff(void);

#endif // __DECK_H__
