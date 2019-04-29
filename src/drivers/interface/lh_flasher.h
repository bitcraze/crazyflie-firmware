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
 * @file lh_flasher.h
 * Driver for programming the lighthouse deck SPI flash mem.
 *
 */
#ifndef LH_FLASHER_H
#define LH_FLASHER_H

#include <stdbool.h>

/**
 * Initialize the lighthouse flasher
 * @param i2cPort  I2C port (a CPAL_InitTypeDef) the lighthouse is connected to.
 *
 * @return True on success, else false.
 */
bool lhflashInit();

bool lhflashFlashBootloader();


#endif // LH_FLASHER_H
