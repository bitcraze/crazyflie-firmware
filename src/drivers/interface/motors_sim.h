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
 * motors_sim.h - declares motors_sim.c's public surface for
 * CONFIG_PLATFORM_SIM (Simmyflie).
 *
 * Not "motors.h": that header pulls in the STM32 hardware chain directly
 * (stm32fxxx.h) plus MotorPerifDef's ST peripheral-register fields (see
 * motors_sim.c's own header comment). motorsInit()'s motor-map argument is
 * narrowed from motors.h's real `const MotorPerifDef**` to `const void**`,
 * same precedent as Phase 4.0's i2cdevInit(void*) -- link-compatible with
 * real callers since C linking matches symbol names only, not per-TU
 * parameter types. Every other signature matches current mainline motors.h
 * exactly (verified against stabilizer.c's setMotorRatios()), which is what
 * should let Phase 4.8's stabilizer.c call this stub unmodified.
 *
 * Single source of truth for these signatures -- both motors_sim.c (the
 * definitions) and every consumer (main_sim.c) include this header instead
 * of hand-typing matching forward declarations.
 */
#ifndef __MOTORS_SIM_H__
#define __MOTORS_SIM_H__

#include <stdbool.h>
#include <stdint.h>

void motorsInit(const void **motorMapSelect);
bool motorsTest(void);
void motorsSetRatio(uint32_t id, uint16_t ratio);
uint16_t motorsGetRatio(uint32_t id);
void motorsStop(void);

#endif // __MOTORS_SIM_H__
