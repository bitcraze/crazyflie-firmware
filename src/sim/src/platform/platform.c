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
 * platform.c - Replacement of platform.h for the Simmyflie.
 *
 * The mainline platform.c chooses a hardware configuration from the device
 * type in the OTP memory. The sim has one configuration, so each function
 * returns its value directly.
 */

#include <stddef.h>

#include "config.h"
#include "platform.h"

/* What config.h's MCU_ID_ADDRESS/MCU_FLASH_SIZE_ADDRESS point at in the sim.
 * system.c prints both at boot and exposes them as the cpu.id0-2 and
 * cpu.flash params. The id is the same for every instance. */
const uint32_t simMcuId[3] = {0xFACEBEEF, 0xDEADC0DE, 0x00000BAD};
const uint16_t simMcuFlashSize = 1024;

int platformInit(void)
{
  return 0;
}

/* Returned by the CRTP platform service as the device type name. */
const char* platformConfigGetDeviceTypeName(void)
{
  return "Simmyflie";
}

/* stabilizer.c passes this to motorsInit(), which does not use it. */
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
