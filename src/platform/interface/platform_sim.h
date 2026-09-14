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
 * platform_sim.h - declares platform_sim.c's public surface for
 * CONFIG_PLATFORM_SIM (Simmyflie).
 *
 * Not "platform.h": that header pulls in motors.h -> stm32fxxx.h, the STM32
 * hardware chain, which has no meaning off-target (see platform_sim.c's own
 * header comment). This is the single source of truth for these two
 * signatures -- both platform_sim.c (the definitions) and every consumer
 * (main_sim.c, platformservice.c; and, via src/config/sim/hw_shims/
 * platform.h, stabilizer.c -- see Phase 4.8) include this header instead of
 * each hand-typing a matching forward declaration, so a signature change is
 * a compile error at the definition site instead of a silent cross-TU
 * mismatch the linker won't catch.
 *
 * Phase 4.8 adds platformConfigGetMotorMapping(): stabilizer.c's
 * motorsInit(platformConfigGetMotorMapping()) call site. Returns
 * `const void**` (not real platform.h's `const MotorPerifDef**`), matching
 * motors_sim.h's own narrowed motorsInit() signature -- the sim
 * motorsInit() ignores its argument entirely (see motors_sim.c), so a
 * fixed NULL is enough.
 */
#ifndef __PLATFORM_SIM_H__
#define __PLATFORM_SIM_H__

int platformInit(void);
const char* platformConfigGetDeviceTypeName(void);
const void** platformConfigGetMotorMapping(void);

#endif // __PLATFORM_SIM_H__
