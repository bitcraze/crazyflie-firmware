/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
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
 *
 */

 /**
 * @file platform_stm32f4.c
 * @brief Platform functionality for platforms using the STM32
 *
 * @ingroup system
 *
 */

#include <string.h>
#include "platform.h"

#define PLATFORM_INFO_OTP_NR_OF_BLOCKS 16
#define PLATFORM_INFO_OTP_BLOCK_LEN 32
#if PLATFORM_DEVICE_TYPE_STRING_MAX_LEN < (PLATFORM_INFO_OTP_BLOCK_LEN + 1)
  #error
#endif


#define DEFAULT_PLATFORM_STRING "0;CF20"


#ifndef UNIT_TEST_MODE
static char* getAddressOfOtpMemoryBlock(int blockNr) {
  return (char*)(0x1fff7800 + blockNr * 0x20);
}
#else
  // This function is replaced by a mock in unit tests
  char* getAddressOfOtpMemoryBlock(const int blockNr);
#endif




void platformGetDeviceTypeString(char* deviceTypeString) {
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

  strncpy(deviceTypeString, block, PLATFORM_INFO_OTP_BLOCK_LEN);
  deviceTypeString[PLATFORM_INFO_OTP_BLOCK_LEN] = '\0';
}
