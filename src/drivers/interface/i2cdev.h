/**
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
 * i2cdev.h - Functions to write to I2C devices
 */

#ifndef __I2CDEV_H__
#define __I2CDEV_H__

#include <stdint.h>
#include <stdbool.h>

#include "cpal.h"

// Delay is approx 0.2us per loop @64Mhz
#define I2CDEV_LOOPS_PER_US  5
#define I2CDEV_LOOPS_PER_MS  (1000 * I2CDEV_LOOPS_PER_US)

#define I2CDEV_I2C1_PIN_SDA GPIO_Pin_7
#define I2CDEV_I2C1_PIN_SCL GPIO_Pin_6

#define I2CDEV_NO_MEM_ADDR  0xFF


#ifdef PLATFORM_CF1
typedef I2C_TypeDef      I2C_Dev;
#else
typedef CPAL_InitTypeDef I2C_Dev;
#define I2C1_DEV &I2C1_DevStructure
#define I2C2_DEV &I2C2_DevStructure
#define I2C3_DEV &I2C3_DevStructure
#endif


/**
 * Read bytes from an I2C peripheral
 * @param I2Cx  Pointer to I2C peripheral to read from
 * @param devAddress  The device address to read from
 * @param memAddress  The internal address to read from, I2CDEV_NO_MEM_ADDR if none.
 * @param len  Number of bytes to read.
 * @param data  Pointer to a buffer to read the data to.
 *
 * @return TRUE if read was successful, otherwise FALSE.
 */
bool i2cdevRead(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
               uint16_t len, uint8_t *data);

/**
 * Read bytes from an I2C peripheral with a 16bit internal reg/mem address
 * @param I2Cx  Pointer to I2C peripheral to read from
 * @param devAddress  The device address to read from
 * @param memAddress  The internal address to read from, I2CDEV_NO_MEM_ADDR if none.
 * @param len  Number of bytes to read.
 * @param data  Pointer to a buffer to read the data to.
 *
 * @return TRUE if read was successful, otherwise FALSE.
 */
bool i2cdevRead16(I2C_Dev *dev, uint8_t devAddress, uint16_t memAddress,
               uint16_t len, uint8_t *data);

/**
 * I2C device init function.
 * @param I2Cx  Pointer to I2C peripheral to initialize.
 *
 * @return TRUE if initialization went OK otherwise FALSE.
 */
int i2cdevInit(I2C_Dev *dev);

/**
 * Read a byte from an I2C peripheral
 * @param I2Cx  Pointer to I2C peripheral to read from
 * @param devAddress  The device address to read from
 * @param memAddress  The internal address to read from, I2CDEV_NO_MEM_ADDR if none.
 * @param data  Pointer to a buffer to read the data to.
 *
 * @return TRUE if read was successful, otherwise FALSE.
 */
bool i2cdevReadByte(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                    uint8_t *data);

/**
 * Read a bit from an I2C peripheral
 * @param I2Cx  Pointer to I2C peripheral to read from
 * @param devAddress  The device address to read from
 * @param memAddress  The internal address to read from, I2CDEV_NO_MEM_ADDR if none.
 * @param bitNum  The bit number 0 - 7 to read.
 * @param data  Pointer to a buffer to read the data to.
 *
 * @return TRUE if read was successful, otherwise FALSE.
 */
bool i2cdevReadBit(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                     uint8_t bitNum, uint8_t *data);
/**
 * Read up to 8 bits from an I2C peripheral
 * @param I2Cx  Pointer to I2C peripheral to read from
 * @param devAddress  The device address to read from
 * @param memAddress  The internal address to read from, I2CDEV_NO_MEM_ADDR if none.
 * @param bitStart The bit to start from, 0 - 7.
 * @param length  The number of bits to read, 1 - 8.
 * @param data  Pointer to a buffer to read the data to.
 *
 * @return TRUE if read was successful, otherwise FALSE.
 */
bool i2cdevReadBits(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                    uint8_t bitStart, uint8_t length, uint8_t *data);

/**
 * Write bytes to an I2C peripheral
 * @param I2Cx  Pointer to I2C peripheral to write to
 * @param devAddress  The device address to write to
 * @param memAddress  The internal address to write to, I2CDEV_NO_MEM_ADDR if none.
 * @param len  Number of bytes to read.
 * @param data  Pointer to a buffer to read the data from that will be written.
 *
 * @return TRUE if write was successful, otherwise FALSE.
 */
bool i2cdevWrite(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                 uint16_t len, uint8_t *data);

/**
 * Write bytes to an I2C peripheral with 16bit internal reg/mem address.
 * @param I2Cx  Pointer to I2C peripheral to write to
 * @param devAddress  The device address to write to
 * @param memAddress  The internal address to write to, I2CDEV_NO_MEM_ADDR if none.
 * @param len  Number of bytes to read.
 * @param data  Pointer to a buffer to read the data from that will be written.
 *
 * @return TRUE if write was successful, otherwise FALSE.
 */
bool i2cdevWrite16(I2C_Dev *dev, uint8_t devAddress, uint16_t memAddress,
                   uint16_t len, uint8_t *data);

/**
 * Write a byte to an I2C peripheral
 * @param I2Cx  Pointer to I2C peripheral to write to
 * @param devAddress  The device address to write to
 * @param memAddress  The internal address to write from, I2CDEV_NO_MEM_ADDR if none.
 * @param data  The byte to write.
 *
 * @return TRUE if write was successful, otherwise FALSE.
 */
bool i2cdevWriteByte(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                     uint8_t data);

/**
 * Write a bit to an I2C peripheral
 * @param I2Cx  Pointer to I2C peripheral to write to
 * @param devAddress  The device address to write to
 * @param memAddress  The internal address to write to, I2CDEV_NO_MEM_ADDR if none.
 * @param bitNum  The bit number, 0 - 7, to write.
 * @param data  The bit to write.
 *
 * @return TRUE if read was successful, otherwise FALSE.
 */
bool i2cdevWriteBit(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                    uint8_t bitNum, uint8_t data);

/**
 * Write up to 8 bits to an I2C peripheral
 * @param I2Cx  Pointer to I2C peripheral to write to
 * @param devAddress  The device address to write to
 * @param memAddress  The internal address to write to, I2CDEV_NO_MEM_ADDR if none.
 * @param bitStart The bit to start from, 0 - 7.
 * @param length  The number of bits to write, 1 - 8.
 * @param data  The byte containing the bits to write.
 *
 * @return TRUE if read was successful, otherwise FALSE.
 */
bool i2cdevWriteBits(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                     uint8_t bitStart, uint8_t length, uint8_t data);

/**
 * Unlocks the i2c bus if needed.
 * @param portSCL  Port of the SCL pin
 * @param portSDA  Port of the SDA pin
 * @param pinSCL   SCL Pin
 * @param pinSDA   SDA Pin
 */
void i2cdevUnlockBus(GPIO_TypeDef* portSCL, GPIO_TypeDef* portSDA, uint16_t pinSCL, uint16_t pinSDA);

/**
 * I2C1 DMA interrupt handler
 */
void i2cDmaInterruptHandlerI2c1(void);
/**
 * I2C2 DMA interrupt handler
 */
void i2cDmaInterruptHandlerI2c2(void);

#endif //__I2CDEV_H__
