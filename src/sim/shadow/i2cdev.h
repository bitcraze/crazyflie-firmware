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
 * i2cdev.h - Shadow header for the Simmyflie.
 *
 * The mainline i2cdev.h includes i2c_drv.h and through it the STM32 device
 * headers. The sim has no I2C bus: the bus handles are NULL and i2cdevInit(),
 * in src/sim/src/drivers/i2cdev.c, does nothing. i2cdevInit() takes
 * `void *` instead of `I2C_Dev *`, which is link-compatible.
 *
 * See docs/development/simmyflie/index.md for the Shadow header rules.
 */
#ifndef __I2CDEV_H__
#define __I2CDEV_H__

#include <stddef.h>

#define I2C1_DEV  NULL
#define I2C3_DEV  NULL

int i2cdevInit(void *dev);

#endif // __I2CDEV_H__
