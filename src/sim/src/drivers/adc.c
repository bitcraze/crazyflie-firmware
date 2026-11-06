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
 * adc_sim.c - adc.h backend for CONFIG_PLATFORM_SIM (Simmyflie), Phase 4.0.
 *
 * On real hardware, system.c's adcInit() call is oddly satisfied by
 * src/deck/api/deck_analog.c (an Arduino-compatible analog-input deck
 * feature reusing the same symbol name), a fully STM32 ADC2/DMA-register
 * affair -- not built under PLATFORM_SIM (deck/ isn't in the sim source
 * list). This just needs to exist so system.c's call resolves; Phase 4.0
 * has no simulated meaning for it yet (adc.h's other declarations --
 * adcTest()/adcDmaStart()/etc. -- are unused in sim so far and deliberately
 * left undefined; add them if/when a later chunk's call graph needs them).
 */

#include "adc.h"

void adcInit(void)
{
}
