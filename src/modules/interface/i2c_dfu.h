/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C)2025 Bitcraze AB
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
 * Use DFU via I2C to read and write the flash of an MCU on the deck I2C bus. To be
 * used with the STM32 DFU bootloader.
 *
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

/**
 * Write data via I2C to the DFU bootloader.
 *
 * @param i2cAddr I2C 7-bit device address
 * @param memAddr Memory address to write to
 * @param buffer Pointer to data buffer to write
 * @param length Number of bytes to write
 * @return true if write was successful, false otherwise
 */
bool dfu_i2c_write(const uint8_t i2cAddr, const uint32_t memAddr, const uint8_t* buffer, const uint32_t length);

/**
 * Read data via I2C from the DFU bootloader.
 *
 * @param i2cAddr I2C 7-bit device address
 * @param memAddr Memory address to read from
 * @param buffer Pointer to buffer where read data will be stored
 * @param length Number of bytes to read
 * @return true if read was successful, false otherwise
 */
bool dfu_i2c_read(const uint8_t i2cAddr, const uint32_t memAddr, uint8_t* buffer, const uint32_t length);