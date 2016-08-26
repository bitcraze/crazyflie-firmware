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

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "eeprom.h"
#include "debug.h"
#include "eprintf.h"

#ifdef EEPROM_RUN_WRITE_READ_TEST
static bool eepromTestWriteRead(void);
#endif

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

#ifdef EEPROM_RUN_WRITE_READ_TEST
  status = eepromTestWriteRead();
#endif

  return status;
}

#ifdef EEPROM_RUN_WRITE_READ_TEST
static bool eepromTestWriteRead(void)
{
  bool status = true;
  int i;
  const uint8_t testData[20] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19};
  uint8_t readData[20];

  for (i = 0; i < sizeof(testData) && status; i++)
  {
    // Write one byte with increasing addresses.
    eepromWriteBuffer(&testData[i], i, 1);
    // Read it all back and check
    eepromReadBuffer(readData, 0, i+1);
    status = (memcmp(testData, readData, i+1) == 0);
  }

  // Write it all.
  eepromWriteBuffer(testData, 0, sizeof(testData));
  for (i = 0; i < sizeof(testData) && status; i++)
  {
    // Read one byte with increasing addresses and check
    eepromReadBuffer(&readData[i], i, 1);
    status = (memcmp(testData, readData, i+1) == 0);
  }

  return status;
}
#endif

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
