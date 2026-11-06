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
 * storage.c - Stub of storage.h for the Simmyflie.
 *
 * The mainline storage.c keeps key/value pairs, such as persistent
 * parameters, in the I2C EEPROM. The sim has no EEPROM: storage reports
 * itself healthy, stores nothing and never finds a key, so nothing survives a
 * restart.
 */

#include "storage.h"

void storageInit()
{
}

bool storageTest()
{
  return true;
}

bool storageStore(const char* key, const void* buffer, size_t length)
{
  (void)key;
  (void)buffer;
  (void)length;
  return true;
}

size_t storageFetch(const char *key, void* buffer, size_t length)
{
  (void)key;
  (void)buffer;
  (void)length;
  return 0;
}

bool storageDelete(const char* key)
{
  (void)key;
  return false;
}

bool storageForeach(const char* prefix, storageFunc_t func)
{
  (void)prefix;
  (void)func;
  return true;
}

void storagePrintStats()
{
}

bool storageReformat()
{
  return true;
}
