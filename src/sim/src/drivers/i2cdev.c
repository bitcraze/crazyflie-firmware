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
 * i2cdev_sim.c - i2cdevInit() backend for CONFIG_PLATFORM_SIM (Simmyflie).
 *
 * The sim has no I2C bus for sensors or decks, so this is a no-op. The
 * declaration is in src/config/sim/hw_shims/i2cdev.h, which shadows the real
 * i2cdev.h (that one pulls in i2c_drv.h -> stm32fxxx.h). The real
 * i2cdevInit() takes an I2C_Dev*, a struct of hardware registers; the shim
 * takes void* and defines I2C1_DEV/I2C3_DEV as NULL.
 */

#include "i2cdev.h"

int i2cdevInit(void *dev)
{
  (void)dev;
  return 1;
}
