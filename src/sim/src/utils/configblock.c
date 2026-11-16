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
 * configblock.c - Stub of configblock.h for the Simmyflie.
 *
 * The mainline configblock keeps the radio settings and the trim calibration
 * in the EEPROM. The sim has no EEPROM and no radio: every function returns
 * the default that the mainline code uses when the config block is not
 * valid.
 */

#include "configblock.h"

int configblockInit(void)
{
  return 0;
}

bool configblockTest(void)
{
  return true;
}

int configblockGetRadioChannel(void)
{
  return 80;
}

int configblockGetRadioSpeed(void)
{
  return 2;
}

uint64_t configblockGetRadioAddress(void)
{
  return 0xE7E7E7E7E7ULL;
}

float configblockGetCalibPitch(void)
{
  return 0.0f;
}

float configblockGetCalibRoll(void)
{
  return 0.0f;
}
