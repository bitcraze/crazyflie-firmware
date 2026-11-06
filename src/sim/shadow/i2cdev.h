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
 * i2cdev.h (sim shim) - shadows src/drivers/interface/i2cdev.h for
 * CONFIG_PLATFORM_SIM (Simmyflie), which pulls in i2c_drv.h -> stm32fxxx.h.
 * Same include-path shadowing as motors.h/pm.h/platform.h in this directory.
 *
 * The sim has no I2C bus. system.c's i2cdevInit(I2C3_DEV)/
 * i2cdevInit(I2C1_DEV) calls compile against these two macros and reach
 * i2cdev_sim.c's no-op.
 */
#ifndef __I2CDEV_HW_SHIM_H__
#define __I2CDEV_HW_SHIM_H__

#include <stddef.h>

#define I2C1_DEV  NULL
#define I2C3_DEV  NULL

int i2cdevInit(void *dev);

#endif // __I2CDEV_HW_SHIM_H__
