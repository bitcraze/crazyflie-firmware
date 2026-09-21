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
 * deck.h (hw_shims) - Phase 4.9 shadow for src/deck/interface/deck.h,
 * CONFIG_PLATFORM_SIM only (Simmyflie).
 *
 * See motors.h in this same directory for the full rationale -- this file
 * is the deck.h counterpart, needed because estimator_kalman.c #includes
 * "deck.h" unconditionally, and that header pulls in the STM32 hardware
 * chain via deck_constants.h/deck_digital.h/deck_analog.h.
 *
 * Not a declaration set of its own -- src/deck/interface/sim/deck.h remains
 * the single source of truth (also defined against directly by
 * src/deck/core/sim/deck.c).
 */
#ifndef __DECK_HW_SHIM_H__
#define __DECK_HW_SHIM_H__

#include "sim/deck.h"

#endif // __DECK_HW_SHIM_H__
