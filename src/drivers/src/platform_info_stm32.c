/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2018 Bitcraze AB
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
 * platform_info_stm32.c - platform information driver for STM32 based platforms
 *
 * Platform information is used to identify the hardware platform the firmware
 * is running on.
 *
 */

#include <string.h>
#include "platform_info_stm32.h"

#define DEFAULT_PLATFORM_STRING "0;CF20"

#ifndef UNIT_TEST_MODE
static char* getAddressOfOtpMemoryBlock(int blockNr) {
  return (char*)(0x1fff7800 + blockNr * 0x20);
}
#else
  // This function is replaced by a mock in unit tests
  char* getAddressOfOtpMemoryBlock(const int blockNr);
#endif




void platformInfoGetPlatformString(char* buf) {
  char* block = 0;

  for (int i = 0; i < PLATFORM_INFO_OTP_NR_OF_BLOCKS; i++) {
    char* candidateBlock = getAddressOfOtpMemoryBlock(i);
    if (candidateBlock[0] != 0) {
      block = candidateBlock;
      break;
    }
  }

  if (!block || ((unsigned char)block[0]) == 0xff) {
    block = DEFAULT_PLATFORM_STRING;
  }

  strncpy(buf, block, PLATFORM_INFO_OTP_BLOCK_LEN);
  buf[PLATFORM_INFO_OTP_BLOCK_LEN] = '\0';
}
