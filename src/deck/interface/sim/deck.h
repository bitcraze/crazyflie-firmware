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
 * deck.h (sim) - declares src/deck/core/sim/deck.c's public surface for
 * CONFIG_PLATFORM_SIM (Simmyflie), Phase 4.9.
 *
 * Not the real src/deck/interface/deck.h: that header pulls in the STM32
 * hardware chain via deck_constants.h/deck_digital.h/deck_analog.h (all
 * -> stm32fxxx.h). estimator_kalman.c is the only sim-built translation
 * unit that #includes "deck.h" (a single call,
 * deckGetRequiredKalmanEstimatorAttitudeReversionOff(), to ask whether a
 * deck wants the EKF to skip attitude-reversion at startup -- meaningless
 * with no decks in sim), and src/config/sim/hw_shims/deck.h shadows that
 * #include onto this file instead -- same include-path-shadowing mechanism
 * Phase 4.8 used for motors.h/pm.h/platform.h.
 *
 * Single source of truth for this signature -- both deck.c (the
 * definition) and every consumer include this header instead of hand-typing
 * a matching forward declaration.
 */
#ifndef __DECK_SIM_H__
#define __DECK_SIM_H__

#include <stdbool.h>

bool deckGetRequiredKalmanEstimatorAttitudeReversionOff(void);

#endif // __DECK_SIM_H__
