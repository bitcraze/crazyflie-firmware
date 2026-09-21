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
 * deck.c (sim) - deck.h backend for CONFIG_PLATFORM_SIM (Simmyflie),
 * Phase 4.9.
 *
 * The real deck_info.c tracks requiredKalmanEstimatorAttitudeReversionOff
 * as a flag ORed in from every discovered deck's memory-defined
 * requiredEstimator/estimation-hint fields -- deck discovery itself
 * (deck_discovery.c/deck_drivers.c) isn't built under sim (no deck bus
 * exists -- i2cdevInit() is a no-op stub, see i2cdev_sim.c/Phase 4.0), so
 * there is no deck that could ever request this. Fixed false forever: no
 * deck ever overrides attitude-reversion in sim.
 */

#include "sim/deck.h"

bool deckGetRequiredKalmanEstimatorAttitudeReversionOff(void)
{
  return false;
}
