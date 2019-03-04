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
 * @file lh_bootloader.h
 * Driver for writing/reading to the lighthouse SPI flash mem.
 *
 */
#ifndef LH_BOOTLOADER_H
#define LH_BOOTLOADER_H

#include <stdbool.h>
#include "i2cdev.h"

/**
 * @brief Address of the firmware in the flash
 * 
 */
#define LH_FW_ADDR          0x020000

/**
 * Initialize the i2c lighthouse module
 * @param i2cPort  I2C port (a CPAL_InitTypeDef) the lighthouse is connected to.
 *
 * @return True on success, else false.
 */
bool lhblInit(I2C_Dev *i2cPort);

/**
 * Boot lighthouse to firmware.
 *
 * @return True on success, else false.
 */
bool lhblBootToFW(void);

/**
 * Get bootloader version
 * @param version Pointer to where the single byte version number will be written.
 *
 * @return True on success, else false.
 */
bool lhblGetVersion(uint8_t *version);

/**
 * Read data from lighthouse spi flash.
 * @param address Address to write from
 * @param length  Length of data
 * @param data    Pointer to memory were data will be stored
 *
 * @return True on success, else false.
 */
bool lhblFlashRead(uint32_t address, uint16_t length, uint8_t *data);

/**
 * Write FW data to lighthouse spi flash.
 * @param data    Data to write
 * @param length  Length of data
 *
 * @return True on success, else false.
 */
bool lhblFlashWriteFW(uint8_t *data, uint32_t length);

/**
 * Erase firwmare section in lighthouse spi flash
 *
 * @return True on success, else false.
 */
bool lhblFlashEraseFirmware(void);

/**
 * Wakeup lighthouse spi flash.
 *
 * @return True on success, else false.
 */
bool lhblFlashWakeup(void);


#endif // LH_BOOTLOADER_H
