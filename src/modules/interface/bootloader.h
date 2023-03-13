/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) Bitcraze AB
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
 * @file bootloader.h
 * Functions to handle transitioning from the firmware to bootloader (DFU) mode on startup
 *
 */

/* ST includes */
#include "stm32f4xx.h"

/**
 * @brief Check if memory is set after software 
 * reboot indicating we need to branch to the bootloader.
 * Run every time on startup.
 */
void check_enter_bootloader();

/**
 * @brief Initiate the procedure to reboot into the bootloader.
 * 
 * This function does not return, instead setting a flag to 
 * jump to the bootloader on the next start and
 * issuing a software reset.
 * 
 * @param r0 The register to utilize when jumping
 * @param bl_addr The bootloader address to jump to
 */
void enter_bootloader(uint32_t r0, uint32_t bl_addr);