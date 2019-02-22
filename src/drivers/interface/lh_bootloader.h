/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
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
 * @file eeprom.h
 * Driver for writing/reading the the lighthouse SPI flash mem.
 *
 */
#ifndef EERROM_H
#define EERROM_H

#include <stdbool.h>
#include "i2cdev.h"

/**
 * Initialize the i2c lighthouse module
 * @param i2cPort  I2C port ( a CPAL_InitTypeDef) the lighthouse is connected to.
 *
 * @return True on success, else false.
 */
bool lhblInit(I2C_Dev *i2cPort);

bool lhblBootToFW(void);

bool lhblFlashWriteFW(uint8_t *data, uint32_t length);

bool lhblFlashWritePage(uint32_t address, uint16_t length, uint8_t *data);

bool lhblFlashEraseFirmware(void);

bool lhblFlashWakeup(void);

bool lhblFlashWaitComplete(void);

#endif // EERROM_H
