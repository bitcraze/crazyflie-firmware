/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2020 Bitcraze AB
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
 *
 * app_memory_mapping.c - example of how to map memory to the memory subsystem
 * that is accessable from a client
 */

#include <stdint.h>
#include <string.h>

#include "mem.h"

#include "app_memory_mapping.h"

#define APP_MEM_SIZE 10
static uint8_t appMemory[APP_MEM_SIZE];


static uint32_t handleMemGetSize(void) { return APP_MEM_SIZE; }
static bool handleMemRead(const uint32_t virtMemAddr, const uint8_t readLen, uint8_t* buffer);
static bool handleMemWrite(const uint32_t virtMemAddr, const uint8_t writeLen, const uint8_t* buffer);
static const MemoryHandlerDef_t memDef = {
  .type = MEM_TYPE_APP,
  .getSize = handleMemGetSize,
  .read = handleMemRead,
  .write = handleMemWrite,
};

static bool handleMemRead(const uint32_t virtMemAddr, const uint8_t readLen, uint8_t* buffer) {
  bool result = false;

  uint32_t virtEndAddr = virtMemAddr + readLen;
  if (virtEndAddr <= APP_MEM_SIZE) {
    uint8_t* realMemAddr = appMemory;
    memcpy(buffer, realMemAddr + virtMemAddr, readLen);
    result = true;
  }

  return result;
}

static bool handleMemWrite(const uint32_t virtMemAddr, const uint8_t writeLen, const uint8_t* buffer) {
  bool result = false;

  uint32_t virtEndAddr = virtMemAddr + writeLen;
  if (virtEndAddr <= APP_MEM_SIZE) {
    uint8_t* realMemAddr = appMemory;
    memcpy(realMemAddr + virtMemAddr, buffer, writeLen);
    result = true;
  }

  return result;
}

void appRegisterMemory() {
  memoryRegisterHandler(&memDef);
}
