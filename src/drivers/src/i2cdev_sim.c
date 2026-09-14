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
 * i2cdev_sim.c - i2cdevInit() backend for CONFIG_PLATFORM_SIM (Simmyflie),
 * Phase 4.0.
 *
 * Deliberately does not include drivers/interface/i2cdev.h: that header
 * pulls in i2c_drv.h -> stm32fxxx.h (I2C_TypeDef/GPIO_TypeDef/
 * DMA_Stream_TypeDef register types), an STM32 hardware chain with no
 * meaning off-target -- same reasoning as platform_sim.c bypassing
 * platform.h. The real i2cdevInit(I2C_Dev *dev) takes an I2C_Dev*
 * (=I2cDrv*, a hardware-register-laden struct); this sim stand-in widens
 * the parameter to void* rather than pull that struct in just to ignore
 * it. No sim caller needs a real I2C bus, so this is a pure no-op.
 *
 * Open item for Phase 4.10: real system.c calls i2cdevInit(I2C3_DEV) /
 * i2cdevInit(I2C1_DEV), where those macros expand to &sensorsBus/&deckBus
 * (real I2cDrv globals this file doesn't provide). That cutover will need
 * its own answer -- e.g. gating those two calls out under PLATFORM_SIM
 * like uartslkEnableIncoming()/systemRequestNRFVersion() already are,
 * since sim has no I2C bus for decks or sensors to probe. Not a Phase 4.0
 * concern: nothing in this chunk's own call site passes real bus pointers.
 */

int i2cdevInit(void *dev)
{
  (void)dev;
  return 1;
}
