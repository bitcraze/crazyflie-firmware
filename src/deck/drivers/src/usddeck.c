/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2016 Bitcraze AB
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * usddeck.c: micro SD deck driver. Implements logging to micro SD card.
 */

#define DEBUG_MODULE "uSD"

#include <stdint.h>
#include <string.h>
#include "stm32fxxx.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "tm_stm32f4_fatfs.h"

#include "deck.h"
#include "usddeck.h"
#include "system.h"
#include "debug.h"
#include "log.h"
#include "param.h"

#define USD_DATAQUEUE_ITEMS       1000  // Items for roughly one second of buffer
#define USD_CLOSE_REOPEN_BYTES    (USD_DATAQUEUE_ITEMS * 60 * sizeof(UsdLogStruct))

xQueueHandle usdDataQueue;

//Fatfs object
FATFS FatFs;
//File object
FIL logFile;

static bool usdMountAndOpen(bool append)
{
  bool fileStatus = false;

  //Mount drive
  if (f_mount(&FatFs, "", 1) == FR_OK)
  {
    DEBUG_PRINT("Drive mounted [OK]\n");
    //Try to open file
    if (append)
    {
      if (f_open(&logFile, "log.bin", FA_OPEN_APPEND | FA_READ | FA_WRITE) == FR_OK)
      {
        fileStatus = true;
      }
    }
    else
    {
      if (f_open(&logFile, "log.bin", FA_CREATE_ALWAYS | FA_READ | FA_WRITE) == FR_OK)
      {
        fileStatus = true;
      }
    }
  }

  return fileStatus;
}

/*********** Tasks ************/
static void usdTask(void *param)
{
  //Free and total space
  uint32_t bytesWritten;
  uint32_t totalBytesWritten;
  uint32_t closeReopenBytes;
  UsdLogStruct  logItem;
  bool fileStatus;

  systemWaitStart();

  fileStatus = usdMountAndOpen(false);

  while (1)
  {
    if (xQueueReceive(usdDataQueue, &logItem, portMAX_DELAY) && fileStatus)
    {
      f_write(&logFile, &logItem, sizeof(UsdLogStruct), (UINT*)&bytesWritten);

      totalBytesWritten += bytesWritten;
      closeReopenBytes += bytesWritten;

      if (closeReopenBytes > USD_CLOSE_REOPEN_BYTES)
      {
        // To be sure to write down the data we close and reopen
        closeReopenBytes = 0;
        f_close(&logFile);
        f_mount(0, "", 1);

        if (!usdMountAndOpen(true))
        {
          // Suspend ourselves
          vTaskSuspend(0);
        }
      }
    }
  }
}

/*********** Deck driver initialization ***************/

static bool isInit = false;

static void usdInit(DeckInfo *info)
{
  xTaskCreate(usdTask, "usdTask", 2*configMINIMAL_STACK_SIZE, NULL, /*priority*/2, NULL);

  usdDataQueue = xQueueCreate(USD_DATAQUEUE_ITEMS, sizeof(UsdLogStruct));

  if (usdDataQueue)
  {
    isInit = true;
  }
}

static bool usdTest()
{
  if (!isInit)
  {
    DEBUG_PRINT("Error while initializing uSD deck\n");
  }

  return isInit;
}

bool usdQueueLogData(UsdLogStruct* logData)
{
  return (xQueueSendToBack(usdDataQueue, logData, 0) == pdTRUE);
}

static const DeckDriver usd_deck = {
  .vid = 0xBC,
  .pid = 0x07,
  .name = "bcUSD",

  .usedGpio = 0,  // FIXME: set the used pins

  .init = usdInit,
  .test = usdTest,
};

DECK_DRIVER(usd_deck);
