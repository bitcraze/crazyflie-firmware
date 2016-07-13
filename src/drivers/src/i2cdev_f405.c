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
 * i2cdev.c - Functions to write to I2C devices
 */
#define DEBUG_MODULE "I2CDEV"

#include <stdint.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "queue.h"

#include "i2cdev.h"
#include "i2c_drv.h"
#include "nvicconf.h"
#include "debug.h"

#define OWN_ADDRESS        0x74
#define I2C_CLOCK_SPEED    400000;

#define I2CDEV_CLK_TS (10 * I2CDEV_LOOPS_PER_US)

#define GPIO_WAIT_FOR_HIGH(gpio, pin, timeoutcycles)\
  {\
    int i = timeoutcycles;\
    while(GPIO_ReadInputDataBit(gpio, pin) == Bit_RESET && i--);\
  }

#define GPIO_WAIT_FOR_LOW(gpio, pin, timeoutcycles) \
  {\
    int i = timeoutcycles;\
    while(GPIO_ReadInputDataBit(gpio, pin) == Bit_SET && i--);\
  }


xQueueHandle i2cQueue;

/* Private functions */
static inline void i2cdevRoughLoopDelay(uint32_t us) __attribute__((optimize("O2")));


int i2cdevInit(I2C_Dev *dev)
{
  i2cInit();

  i2cQueue = xQueueCreate(3, sizeof(I2cMessage));

  return true;
}

bool i2cdevReadByte(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                    uint8_t *data)
{
  return i2cdevRead(dev, devAddress, memAddress, 1, data);
}

bool i2cdevReadBit(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                     uint8_t bitNum, uint8_t *data)
{
  uint8_t byte;
  bool status;

  status = i2cdevRead(dev, devAddress, memAddress, 1, &byte);
  *data = byte & (1 << bitNum);

  return status;
}

bool i2cdevReadBits(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                    uint8_t bitStart, uint8_t length, uint8_t *data)
{
  bool status;
  uint8_t byte;

  if ((status = i2cdevReadByte(dev, devAddress, memAddress, &byte)) == true)
  {
      uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
      byte &= mask;
      byte >>= (bitStart - length + 1);
      *data = byte;
  }
  return status;
}

bool i2cdevRead(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
               uint16_t len, uint8_t *data)
{
  I2cMessage message;

  if (memAddress == I2CDEV_NO_MEM_ADDR)
  {
    i2cCreateMessage(&message, devAddress, i2cRead, i2cQueue, len, data);
  }
  else
  {
    i2cCreateMessageIntAddr(&message, devAddress, false, memAddress,
                            i2cRead, i2cQueue, len, data);
  }
  i2cMessageTransfer(&message, portMAX_DELAY);

  if (xQueueReceive(i2cQueue, &message, M2T(100)) == pdTRUE)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool i2cdevRead16(I2C_Dev *dev, uint8_t devAddress, uint16_t memAddress,
               uint16_t len, uint8_t *data)
{
  I2cMessage message;

  i2cCreateMessageIntAddr(&message, devAddress, true, memAddress,
                          i2cRead, i2cQueue, len, data);
  i2cMessageTransfer(&message, portMAX_DELAY);

  if (xQueueReceive(i2cQueue, &message, M2T(100)) == pdTRUE)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool i2cdevWriteByte(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                    uint8_t data)
{
  return i2cdevWrite(dev, devAddress, memAddress, 1, &data);
}

bool i2cdevWriteBit(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                    uint8_t bitNum, uint8_t data)
{
    uint8_t byte;
    i2cdevReadByte(dev, devAddress, memAddress, &byte);
    byte = (data != 0) ? (byte | (1 << bitNum)) : (byte & ~(1 << bitNum));
    return i2cdevWriteByte(dev, devAddress, memAddress, byte);
}

bool i2cdevWriteBits(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                     uint8_t bitStart, uint8_t length, uint8_t data)
{
  bool status;
  uint8_t byte;

  if ((status = i2cdevReadByte(dev, devAddress, memAddress, &byte)) == true)
  {
      uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
      data <<= (bitStart - length + 1); // shift data into correct position
      data &= mask; // zero all non-important bits in data
      byte &= ~(mask); // zero all important bits in existing byte
      byte |= data; // combine data with existing byte
      status = i2cdevWriteByte(dev, devAddress, memAddress, byte);
  }

  return status;
}

bool i2cdevWrite(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                uint16_t len, uint8_t *data)
{
  I2cMessage message;

  if (memAddress == I2CDEV_NO_MEM_ADDR)
  {
    i2cCreateMessage(&message, devAddress, i2cWrite, i2cQueue, len, data);
  }
  else
  {
    i2cCreateMessageIntAddr(&message, devAddress, false, memAddress,
                            i2cWrite, i2cQueue, len, data);
  }
  i2cMessageTransfer(&message, portMAX_DELAY);

  if (xQueueReceive(i2cQueue, &message, M2T(100)) == pdTRUE)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool i2cdevWrite16(I2C_Dev *dev, uint8_t devAddress, uint16_t memAddress,
                   uint16_t len, uint8_t *data)
{
  I2cMessage message;

  i2cCreateMessageIntAddr(&message, devAddress, true, memAddress,
                          i2cWrite, i2cQueue, len, data);
  i2cMessageTransfer(&message, portMAX_DELAY);

  if (xQueueReceive(i2cQueue, &message, M2T(100)) == pdTRUE)
  {
    return true;
  }
  else
  {
    return false;
  }
}

static inline void i2cdevRoughLoopDelay(uint32_t us)
{
  volatile uint32_t delay = 0;
  for(delay = 0; delay < I2CDEV_LOOPS_PER_US * us; ++delay) { };
}

void i2cdevUnlockBus(GPIO_TypeDef* portSCL, GPIO_TypeDef* portSDA, uint16_t pinSCL, uint16_t pinSDA)
{
  GPIO_SetBits(portSDA, pinSDA);
  /* Check SDA line to determine if slave is asserting bus and clock out if so */
  while(GPIO_ReadInputDataBit(portSDA, pinSDA) == Bit_RESET)
  {
    /* Set clock high */
    GPIO_SetBits(portSCL, pinSCL);
    /* Wait for any clock stretching to finish. */
    GPIO_WAIT_FOR_HIGH(portSCL, pinSCL, 10 * I2CDEV_LOOPS_PER_MS);
    i2cdevRoughLoopDelay(I2CDEV_CLK_TS);

    /* Generate a clock cycle */
    GPIO_ResetBits(portSCL, pinSCL);
    i2cdevRoughLoopDelay(I2CDEV_CLK_TS);
    GPIO_SetBits(portSCL, pinSCL);
    i2cdevRoughLoopDelay(I2CDEV_CLK_TS);
  }

  /* Generate a start then stop condition */
  GPIO_SetBits(portSCL, pinSCL);
  i2cdevRoughLoopDelay(I2CDEV_CLK_TS);
  GPIO_ResetBits(portSDA, pinSDA);
  i2cdevRoughLoopDelay(I2CDEV_CLK_TS);
  GPIO_ResetBits(portSDA, pinSDA);
  i2cdevRoughLoopDelay(I2CDEV_CLK_TS);

  /* Set data and clock high and wait for any clock stretching to finish. */
  GPIO_SetBits(portSDA, pinSDA);
  GPIO_SetBits(portSCL, pinSCL);
  GPIO_WAIT_FOR_HIGH(portSCL, pinSCL, 10 * I2CDEV_LOOPS_PER_MS);
  /* Wait for data to be high */
  GPIO_WAIT_FOR_HIGH(portSDA, pinSDA, 10 * I2CDEV_LOOPS_PER_MS);
}
