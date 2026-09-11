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
 * pm_sim.h - declares pm_sim.c's public surface for CONFIG_PLATFORM_SIM
 * (Simmyflie).
 *
 * Not "pm.h": that header pulls in the STM32 hardware chain via deck.h (see
 * pm_sim.c's own header comment). Single source of truth for this
 * signature -- both pm_sim.c (the definition) and every consumer
 * (main_sim.c) include this header instead of hand-typing a matching
 * forward declaration.
 */
#ifndef __PM_SIM_H__
#define __PM_SIM_H__

void pmInit(void);

#endif // __PM_SIM_H__
