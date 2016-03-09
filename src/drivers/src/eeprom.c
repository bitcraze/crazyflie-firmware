/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011 Fabio Varesano <fvaresano@yahoo.it>
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
 * @file eeprom.c
 * Driver for the 24AA64F eeprom.
 *
 */
#define DEBUG_MODULE "EEPROM"

#include "FreeRTOS.h"
#include "task.h"

#include "eeprom.h"
#include "debug.h"
#include "eprintf.h"

static uint8_t devAddr;
static I2C_Dev *I2Cx;
static bool isInit;

bool eepromInit(I2C_Dev *i2cPort)
{
  if (isInit)
    return true;

  I2Cx = i2cPort;
  devAddr = EEPROM_I2C_ADDR;

  isInit = true;

  return true;
}

bool eepromTest(void)
{
  bool status;

  status = eepromTestConnection();
  if (status)
  {
    DEBUG_PRINT("I2C connection [OK].\n");
  }
  else
  {
    DEBUG_PRINT("I2C connection [FAIL].\n");
  }

  return status;
}

bool eepromTestConnection(void)
{
  uint8_t tmp;
  bool status;

  if (!isInit)
    return false;

  status = i2cdevRead16(I2Cx, devAddr, 0, 1, &tmp);

  return status;
}

bool eepromReadBuffer(uint8_t* buffer, uint16_t readAddr, uint16_t len)
{
  bool status;

  if ((uint32_t)readAddr + len > EEPROM_SIZE)
  {
     return false;
  }

  status = i2cdevRead16(I2Cx, devAddr, readAddr, len, buffer);

  return status;
}

bool eepromWriteBuffer(uint8_t* buffer, uint16_t writeAddr, uint16_t len)
{
  bool status = true;
  uint16_t index;

  if ((uint32_t)writeAddr + len > EEPROM_SIZE)
  {
     return false;
  }

  for (index = 0; index < len && status; index++)
  {
    status = i2cdevWrite16(I2Cx, devAddr, writeAddr + index, 1, &buffer[index]);
    vTaskDelay(M2T(6));
  }

  return status;
}

bool eepromWritePage(uint8_t* buffer, uint16_t writeAddr)
{
 //TODO: implement
  return false;
}
