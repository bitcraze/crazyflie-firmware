/**
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011 Bitcraze AB
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
 * ow.c - One-wire functions
 */
#define DEBUG_MODULE "LHBL"

#include  <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "lh_bootloader.h"
#include "debug.h"
#include "i2cdev.h"

#define LH_I2C_ADDR         0x2F
#define LH_FW_SIZE          0x020000
#define LH_FLASH_PAGE_SIZE  256
#define LH_WRITE_BUF_SIZE   (5 + 4 + LH_FLASH_PAGE_SIZE)

/* Commands */
#define LHBL_BOOT_TO_FW         0x00
#define LHBL_BL_CMD             0x01
#define LHBL_GET_VERSION        0x02
#define FLASH_CMD_READ          0x03
#define FLASH_CMD_READ_STATUS   0x05
#define FLASH_CMD_WRITE_PAGE    0x02
#define FLASH_CMD_WRITE_EN      0x06
#define FLASH_CMD_ERASE_SECTOR  0xD8
#define FLASH_CMD_WAKEUP        0xAB


static uint8_t devAddr;
static bool isInit;
static uint8_t flashWriteBuf[LH_WRITE_BUF_SIZE];
static uint8_t flashReadBuf[LH_WRITE_BUF_SIZE];

static bool lhExchange(uint16_t writeLen, uint8_t* writeData, uint16_t readLen, uint8_t* readData)
{
  bool status = false;

  status = i2cdevWrite(I2C1_DEV, devAddr, writeLen, writeData);
  if (status && readLen && readData)
  {
    status = i2cdevRead(I2C1_DEV, devAddr, readLen, readData);
  }

  return status;
}

static void lhblHeaderPrepare(uint16_t writeLen, uint16_t readLen, uint8_t* buff)
{
  buff[0] = LHBL_BL_CMD;
  buff[1] = (uint8_t)((writeLen >> 0)  & 0x000000FF);
  buff[2] = (uint8_t)((writeLen >> 8)  & 0x000000FF);
  buff[3] = (uint8_t)((readLen >> 0)  & 0x000000FF);
  buff[4] = (uint8_t)((readLen >> 8)  & 0x000000FF);
}

static bool verify(uint8_t *dataA, uint8_t *dataB, uint16_t length)
{
  for (int i = 0; i < length; i++)
  {
    if (dataA[i] != dataB[i])
    {
      return false;
    }
  }

  return true;
}

bool lhblInit()
{
  if (isInit)
    return true;

  devAddr = LH_I2C_ADDR;

  isInit = true;

  return true;
}

bool lhblTest()
{
  return true;
}

static bool lhblFlashReadStatus(uint8_t* status)
{
  lhblHeaderPrepare(1, 1, flashWriteBuf);
  flashWriteBuf[5] = FLASH_CMD_READ_STATUS;

  return lhExchange(5+1, flashWriteBuf, 1, status);
}

static bool lhblFlashWriteEnable(void)
{
  lhblHeaderPrepare(1, 0, flashWriteBuf);
  flashWriteBuf[5] = FLASH_CMD_WRITE_EN;

  return lhExchange(5 + 1, flashWriteBuf, 0, 0);
}

static bool lhblFlashEraseBlock64k(uint32_t address)
{
  lhblHeaderPrepare(4, 0, flashWriteBuf);
  flashWriteBuf[5] = FLASH_CMD_ERASE_SECTOR;
  flashWriteBuf[6] = (uint8_t)((address >> 16) & 0x000000FF);
  flashWriteBuf[7] = (uint8_t)((address >> 8)  & 0x000000FF);
  flashWriteBuf[8] = (uint8_t)((address >> 0)  & 0x000000FF);

  return lhExchange(5 + 4, flashWriteBuf, 0, 0);
}

static bool lhblFlashWaitComplete(void)
{
  bool status;
  uint8_t flashStatus;

  status = lhblFlashReadStatus(&flashStatus);

  while ((flashStatus & 0x01) != 0)
  {
    vTaskDelay(M2T(1));
    status &= lhblFlashReadStatus(&flashStatus);
  }

  return status;
}

bool lhblFlashWritePage(uint32_t address, uint16_t length, const uint8_t *data)
{
  ASSERT(address >= LH_FW_ADDR);
  ASSERT(length <= LH_FLASH_PAGE_SIZE);

  lhblFlashWriteEnable();

  lhblHeaderPrepare(4 + length, 0, flashWriteBuf);
  flashWriteBuf[5] = FLASH_CMD_WRITE_PAGE;
  flashWriteBuf[6] = (uint8_t)((address >> 16) & 0x000000FF);
  flashWriteBuf[7] = (uint8_t)((address >> 8)  & 0x000000FF);
  flashWriteBuf[8] = (uint8_t)((address >> 0)  & 0x000000FF);

  memcpy(&flashWriteBuf[9], data, length);

  return lhExchange(5 + 4 + length, flashWriteBuf, 0, 0);
}

bool lhblBootToFW(void)
{
  uint8_t instruction = LHBL_BOOT_TO_FW;

  return lhExchange(1, &instruction, 0, 0);
}

bool lhblGetVersion(uint8_t *version)
{
  uint8_t instruction[2] = {LHBL_GET_VERSION, 0x42};

  // Add one dummy byte to fix an I2C bug that sends 2 bytes instead of 1
  return lhExchange(2, instruction, 1, version);
}

bool lhblFlashGetProtocolVersion(void)
{
  return false;
}

bool lhblFlashRead(uint32_t address, uint16_t length, uint8_t *data)
{
  lhblHeaderPrepare(4, length, flashWriteBuf);
  flashWriteBuf[5] = FLASH_CMD_READ;
  flashWriteBuf[6] = (uint8_t)((address >> 16) & 0x000000FF);
  flashWriteBuf[7] = (uint8_t)((address >> 8)  & 0x000000FF);
  flashWriteBuf[8] = (uint8_t)((address >> 0)  & 0x000000FF);

  return lhExchange(5 + 4, flashWriteBuf, length, data);
}

bool lhblFlashWakeup(void)
{
  lhblHeaderPrepare(1, 0, flashWriteBuf);
  flashWriteBuf[5] = FLASH_CMD_WAKEUP;

  return lhExchange(5 + 1, flashWriteBuf, 0, 0);
}

bool lhblFlashEraseFirmware(void)
{
  bool status;

  /* Erase first 64K */
  lhblFlashWriteEnable();
  status = lhblFlashEraseBlock64k(LH_FW_ADDR);
  status &= lhblFlashWaitComplete();
  /* Erase last 64K */
  lhblFlashWriteEnable();
  status &= lhblFlashEraseBlock64k(LH_FW_ADDR + 0x10000);
  status &= lhblFlashWaitComplete();

  return status;
}

bool lhblFlashWriteFW(uint8_t *data, uint32_t length)
{
  bool status = true;
  int pageCount = 0;
  int nbrPages = length / LH_FLASH_PAGE_SIZE;
  int lastPageSize = length % LH_FLASH_PAGE_SIZE;

  ASSERT(length <= LH_FW_SIZE);

  for (pageCount = 0; pageCount < nbrPages && status; pageCount++)
  {
    status = lhblFlashWritePage(LH_FW_ADDR + pageCount * LH_FLASH_PAGE_SIZE,
                                LH_FLASH_PAGE_SIZE,
                                &data[pageCount * LH_FLASH_PAGE_SIZE]);
    lhblFlashWaitComplete();

    lhblFlashRead(LH_FW_ADDR + pageCount * LH_FLASH_PAGE_SIZE,
                  LH_FLASH_PAGE_SIZE,
                  flashReadBuf);

    if (verify(&data[pageCount * LH_FLASH_PAGE_SIZE], flashReadBuf, LH_FLASH_PAGE_SIZE) == false)
    {
      return false;
    }
  }
  if (lastPageSize && status)
  {
    status = lhblFlashWritePage(LH_FW_ADDR + pageCount * LH_FLASH_PAGE_SIZE,
                                lastPageSize,
                                &data[pageCount * LH_FLASH_PAGE_SIZE]);
    lhblFlashWaitComplete();

    lhblFlashRead(LH_FW_ADDR + pageCount * LH_FLASH_PAGE_SIZE,
                  lastPageSize,
                  flashReadBuf);

    if (verify(&data[pageCount * LH_FLASH_PAGE_SIZE], flashReadBuf, lastPageSize) == false)
    {
      return false;
    }
  }

  return status;
}
