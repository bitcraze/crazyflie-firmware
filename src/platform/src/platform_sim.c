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
 * platform_sim.c - platformInit() for CONFIG_PLATFORM_SIM (Simmyflie).
 *
 * Deliberately does not use the shared src/platform/src/platform.c
 * dispatcher (platformConfig_t / platformGetListOfConfigurations() /
 * platformInitHardware()): that mechanism pulls in src/platform/interface
 * /platform.h -> src/drivers/interface/motors.h -> src/config/stm32fxxx.h,
 * an STM32 hardware header chain with no meaning off-target. Real
 * sensor/motor plumbing is Phase 5's job (see dev/implementation-plan.md in
 * the simulation_model project); Phase 1 just needs platformInit() to exist.
 */

#include <stddef.h>

#include "config.h"
#include "platform_sim.h"

/* What config.h's MCU_ID_ADDRESS/MCU_FLASH_SIZE_ADDRESS point at in the sim.
 * system.c prints both at boot and exposes them as the cpu.id0-2 and
 * cpu.flash params. The id is the same for every instance. */
const uint32_t simMcuId[3] = {0xFACEBEEF, 0xDEADC0DE, 0x00000BAD};
const uint16_t simMcuFlashSize = 1024;

int platformInit(void)
{
  return 0;
}

/* Phase 4.2 (Platform): the one platform.h query platformservice.c's real
 * versionCommandProcess() needs (CRTP Platform port 13, ch.1
 * getDeviceTypeName) -- a fixed string here rather than routing through
 * the shared platform.c dispatcher's active_config, which this file
 * deliberately bypasses (see file header comment above). */
const char* platformConfigGetDeviceTypeName(void)
{
  return "Simmyflie";
}

/* Phase 4.8: stabilizer.c's motorsInit(platformConfigGetMotorMapping())
 * call site -- motors_sim.c's motorsInit() ignores its argument entirely,
 * so there's no real motor map to return. */
const void** platformConfigGetMotorMapping(void)
{
  return NULL;
}

/* system.c asks these two only to decide on the low interference radio
 * mode, which a deck requests. The sim has neither decks nor a radio. */
bool platformConfigPhysicalLayoutAntennasAreClose(void)
{
  return false;
}

void platformSetLowInterferenceRadioMode(void)
{
}
