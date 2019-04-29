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
 * lh_flasher.c - Code to program the Lighthouse deck SPI flash memory
 */
#define DEBUG_MODULE "LHFLASH"

#include  <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "lh_flasher.h"
#include "debug.h"

#include "deck.h"

#include "log.h"

#define STR2(x) #x
#define STR(x) STR2(x)

#define INCBIN(name, file) \
    __asm__(".section .rodata\n" \
            ".global incbin_" STR(name) "_start\n" \
            ".align 4\n" \
            "incbin_" STR(name) "_start:\n" \
            ".incbin \"" file "\"\n" \
            \
            ".global incbin_" STR(name) "_end\n" \
            ".align 1\n" \
            "incbin_" STR(name) "_end:\n" \
            ".byte 0\n" \
            ".align 4\n" \
            STR(name) "Size:\n" \
            ".int incbin_" STR(name) "_end - incbin_" STR(name) "_start\n" \
    ); \
    extern const __attribute__((aligned(4))) void* incbin_ ## name ## _start; \
    extern const void* incbin_ ## name ## _end; \
    extern const int name ## Size; \
    static const __attribute__((used)) unsigned char* name = (unsigned char*) & incbin_ ## name ## _start; \

INCBIN(bootloader, "bootloader.bin");


#define LH_SPI_CS DECK_GPIO_IO1
#define LH_FPGA_RESET DECK_GPIO_RX2

#define LH_I2C_ADDR         0x2F
#define LH_FLASH_PAGE_SIZE  256
#define LH_WRITE_BUF_SIZE   (5 + 4 + LH_FLASH_PAGE_SIZE)

/* Commands */
#define LHBL_BOOT_TO_FW         0x00
#define LHBL_BL_CMD             0x01
#define LHBL_GET_VERSION        0x02
#define FLASH_CMD_WRITE_STATUS  0x01
#define FLASH_CMD_READ          0x03
#define FLASH_CMD_READ_STATUS   0x05
#define FLASH_CMD_WRITE_PAGE    0x02
#define FLASH_CMD_WRITE_EN      0x06
#define FLASH_CMD_ERASE_SECTOR  0xD8
#define FLASH_CMD_WAKEUP        0xAB
#define FLASH_CMD_POWER_DOWN    0xB9


static bool isInit;
static uint8_t flashWriteBuf[LH_WRITE_BUF_SIZE];
static uint8_t flashReadBuf[LH_WRITE_BUF_SIZE];

static bool statusDone = false;
static uint32_t statusCode = 0;

static bool lhExchange(uint16_t writeLen, uint8_t* writeData, uint16_t readLen, uint8_t* readData)
{
  static uint8_t wbuffer[1024];
  static uint8_t rbuffer[1024];

  memcpy(wbuffer, writeData, writeLen);

  spiBeginTransaction(SPI_BAUDRATE_2MHZ);

  digitalWrite(LH_SPI_CS, 0);

  spiExchange(writeLen+readLen, wbuffer, rbuffer);

  digitalWrite(LH_SPI_CS, 1);
  
  spiEndTransaction();

  if (readLen > 0) {
     memcpy(readData, &rbuffer[writeLen], readLen);
  }

  return true;
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

bool lhflashInit()
{
  if (isInit)
    return true;

  spiBegin();

  pinMode(LH_FPGA_RESET, OUTPUT);
  pinMode(LH_SPI_CS, INPUT);

  digitalWrite(LH_FPGA_RESET, 1);


  isInit = true;

  return true;
}

static bool flashReadStatus(uint8_t* status)
{
  flashWriteBuf[0] = FLASH_CMD_READ_STATUS;

  return lhExchange(1, flashWriteBuf, 1, status);
}

static bool flashWriteEnable(void)
{
  bool status = true;
  uint8_t flash_status = 0;

  do {
    flashWriteBuf[0] = FLASH_CMD_WRITE_EN;

    status &= lhExchange(1, flashWriteBuf, 0, 0);

    status &= flashReadStatus(&flash_status);
  } while ((flash_status & 0x02) == 0);
  
  return status;
}

static bool flashEraseBlock64k(uint32_t address)
{
  flashWriteBuf[0] = FLASH_CMD_ERASE_SECTOR;
  flashWriteBuf[1] = (uint8_t)((address >> 16) & 0x000000FF);
  flashWriteBuf[2] = (uint8_t)((address >> 8)  & 0x000000FF);
  flashWriteBuf[3] = (uint8_t)((address >> 0)  & 0x000000FF);

  return lhExchange(4, flashWriteBuf, 0, 0);
}

static bool flashWaitComplete(void)
{
  bool status;
  uint8_t flashStatus;

  status = flashReadStatus(&flashStatus);

  while ((flashStatus & 0x01) != 0)
  {
    vTaskDelay(M2T(1));
    status &= flashReadStatus(&flashStatus);
  }

  return status;
}

static bool flashWritePage(uint32_t address, uint16_t length, uint8_t *data)
{
  ASSERT(length <= LH_FLASH_PAGE_SIZE);

  flashWriteEnable();

  flashWriteBuf[0] = FLASH_CMD_WRITE_PAGE;
  flashWriteBuf[1] = (uint8_t)((address >> 16) & 0x000000FF);
  flashWriteBuf[2] = (uint8_t)((address >> 8)  & 0x000000FF);
  flashWriteBuf[3] = (uint8_t)((address >> 0)  & 0x000000FF);

  memcpy(&flashWriteBuf[4], data, length);

  return lhExchange(4 + length, flashWriteBuf, 0, 0);
}

static void fpgaHoldReset(void)
{
  digitalWrite(LH_FPGA_RESET, 0);
  pinMode(LH_SPI_CS, OUTPUT);
  digitalWrite(LH_SPI_CS, 1);
}

static void fpgaReleaseReset(void)
{
  pinMode(LH_SPI_CS, INPUT);
  digitalWrite(LH_FPGA_RESET, 1);
}

static bool flashRead(uint32_t address, uint16_t length, uint8_t *data)
{
  flashWriteBuf[0] = FLASH_CMD_READ;
  flashWriteBuf[1] = (uint8_t)((address >> 16) & 0x000000FF);
  flashWriteBuf[2] = (uint8_t)((address >> 8)  & 0x000000FF);
  flashWriteBuf[3] = (uint8_t)((address >> 0)  & 0x000000FF);

  return lhExchange(4, flashWriteBuf, length, data);
}

static bool flashWakeup(void)
{
  flashWriteBuf[0] = FLASH_CMD_WAKEUP;

  return lhExchange(1, flashWriteBuf, 0, 0);
}

// static bool flashPowerDown(void)
// {
//   flashWriteBuf[0] = FLASH_CMD_POWER_DOWN;

//   return lhExchange(1, flashWriteBuf, 0, 0);
// }

static bool flashReset()
{
  bool status = true;

  flashWriteBuf[0] = 0x66;

  status &= lhExchange(1, flashWriteBuf, 0, 0);

  flashWriteBuf[0] = 0x99;

  status &= lhExchange(1, flashWriteBuf, 0, 0);

  return status;
}

static bool flashErase(void)
{
  bool status = true;

  /* Disable write protection */
  flashWriteEnable();

  flashWriteBuf[0] = FLASH_CMD_WRITE_STATUS;
  flashWriteBuf[1] = 0;

  lhExchange(2, flashWriteBuf, 0, NULL);
  
  status &= flashWaitComplete();

  /* Erase first 128K */
  flashWriteEnable();
  status = flashEraseBlock64k(0);
  status &= flashWaitComplete();
  
  flashWriteEnable();
  status &= flashEraseBlock64k(0x10000);
  status &= flashWaitComplete();

  // flashWriteEnable();
  // status = flashEraseBlock64k(0x20000);
  // status &= flashWaitComplete();
  
  // flashWriteEnable();
  // status &= flashEraseBlock64k(0x30000);
  // status &= flashWaitComplete();

  return status;
}

static void flashProtectBootloader()
{
  flashWriteEnable();

  flashWriteBuf[0] = FLASH_CMD_WRITE_STATUS;
  flashWriteBuf[1] = 0x28;

  lhExchange(2, flashWriteBuf, 0, NULL);

  flashWaitComplete();
}

static bool flashWriteFW(uint8_t *data, uint32_t length)
{
  bool status = true;
  int pageCount = 0;
  int nbrPages = length / LH_FLASH_PAGE_SIZE;
  int lastPageSize = length % LH_FLASH_PAGE_SIZE;

  for (pageCount = 0; pageCount < nbrPages && status; pageCount++)
  {
    status = flashWritePage(pageCount * LH_FLASH_PAGE_SIZE,
                                LH_FLASH_PAGE_SIZE,
                                &data[pageCount * LH_FLASH_PAGE_SIZE]);
    flashWaitComplete();

    flashRead(pageCount * LH_FLASH_PAGE_SIZE,
                  LH_FLASH_PAGE_SIZE,
                  flashReadBuf);

    if (verify(&data[pageCount * LH_FLASH_PAGE_SIZE], flashReadBuf, LH_FLASH_PAGE_SIZE) == false)
    {
      DEBUG_PRINT("Verify fail page %d!\n", pageCount);
      statusCode = 0x80000000 | pageCount;
      return false;
    }
  }
  if (lastPageSize && status)
  {
    status = flashWritePage(pageCount * LH_FLASH_PAGE_SIZE,
                                lastPageSize,
                                &data[pageCount * LH_FLASH_PAGE_SIZE]);
    flashWaitComplete();

    flashRead(pageCount * LH_FLASH_PAGE_SIZE,
                  lastPageSize,
                  flashReadBuf);

    if (verify(&data[pageCount * LH_FLASH_PAGE_SIZE], flashReadBuf, lastPageSize) == false)
    {
      return false;
    }
  }

  return status;
}

bool lhflashFlashBootloader()
{
  bool pass = true;

  fpgaHoldReset();

  vTaskDelay(M2T(10));

  flashWakeup();
  vTaskDelay(M2T(10));

  flashErase();

  pass &= flashWriteFW((uint8_t *)bootloader, bootloaderSize);

  // Freeze here if the flashing is unsuccessful
  if (pass == false) {
    statusDone = true;
    while(1) {
      vTaskDelay(portMAX_DELAY);
    }
  }

  flashProtectBootloader();

  flashReset();

  fpgaReleaseReset();

  vTaskDelay(M2T(200));

  statusDone = true;

  return pass;
}


LOG_GROUP_START(lhFlasher)
LOG_ADD(LOG_UINT8, done, &statusDone)
LOG_ADD(LOG_UINT32, code, &statusCode)
LOG_GROUP_STOP(lhFlasher)
