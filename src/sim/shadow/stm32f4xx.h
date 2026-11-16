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
 * stm32f4xx.h - Shadow header for the Simmyflie.
 *
 * Mainline code reads SCB->ICSR to tell whether it runs in an interrupt
 * (estimator.c, console.c). The host has no SCB and no exceptions, so this
 * header gives an SCB whose ICSR always reads 0: no exception is active.
 * Nothing else of the STM32 device header is used by a file the sim build
 * compiles. See docs/development/simmyflie/ for the Shadow header rules.
 */
#ifndef __STM32F4XX_H
#define __STM32F4XX_H

#include <stdint.h>

typedef struct {
  uint32_t ICSR;
} SCB_Type;

#define SCB (&(const SCB_Type){ .ICSR = 0 })
#define SCB_ICSR_VECTACTIVE_Msk 0x1FFUL

#endif // __STM32F4XX_H
