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
#include "timers.h"

#include "ff.h"
#include "fatfs_sd.h"

#include "deck.h"
#include "usddeck.h"
#include "deck_spi.h"
#include "system.h"
#include "debug.h"
#include "led.h"

// Hardware defines
#define USD_CS_PIN    DECK_GPIO_IO4

// Log data defines
#define USD_DATAQUEUE_ITEMS       1000  // Items for roughly one second of buffer
#define USD_CLOSE_REOPEN_BYTES    (USD_DATAQUEUE_ITEMS * 10 * sizeof(UsdLogStruct))

// FATFS low lever driver functions.
static void init_spi(void);
static void set_slow_spi_mode(void);
static void set_fast_spi_mode(void);
static BYTE xchg_spi(BYTE dat);
static void rcvr_spi_multi(BYTE *buff, UINT btr);
static void xmit_spi_multi(const BYTE *buff, UINT btx);
static void cs_high(void);
static void cs_low(void);

xQueueHandle usdDataQueue;

static BYTE exchangeBuff[512];
#ifdef USD_RUN_DISKIO_FUNCTION_TESTS
DWORD workBuff[512];  /* 2048 byte working buffer */
#endif

static xTimerHandle timer;
static void usdTimer(xTimerHandle timer);

//Fatfs object
FATFS FatFs;
//File object
FIL logFile;

// Low lever driver functions
static sdSpiContext_t sdSpiContext =
{
  .init_spi = init_spi,
  .set_slow_spi_mode = set_slow_spi_mode,
  .set_fast_spi_mode = set_fast_spi_mode,
  .xchg_spi = xchg_spi,
  .rcvr_spi_multi = rcvr_spi_multi,
  .xmit_spi_multi = xmit_spi_multi,
  .cs_low = cs_low,
  .cs_high = cs_high,

  .Stat = STA_NOINIT,
  .Timer1 = 0,
  .Timer2 = 0
};

static DISKIO_LowLevelDriver_t fatDrv =
{
    SD_disk_initialize,
    SD_disk_status,
    SD_disk_ioctl,
    SD_disk_write,
    SD_disk_read,
    &sdSpiContext,
};


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
  uint32_t totalBytesWritten = 0;
  uint32_t closeReopenBytes = 0;
  UsdLogStruct  logItem;
  bool fileStatus;

  systemWaitStart();

  fileStatus = usdMountAndOpen(false);

  while (1)
  {
    if (xQueueReceive(usdDataQueue, &logItem, portMAX_DELAY) && fileStatus)
    {
      ledSet(LED_GREEN_R, 1);
      f_write(&logFile, &logItem, sizeof(UsdLogStruct), (UINT*)&bytesWritten);
      ledSet(LED_GREEN_R, 0);

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

/*-----------------------------------------------------------------------*/
/* FATFS SPI controls (Platform dependent)                               */
/*-----------------------------------------------------------------------*/

/* Initialize MMC interface */
static void init_spi(void)
{
  spiBegin();   /* Enable SPI function */
  pinMode(USD_CS_PIN, OUTPUT);
  cs_high();      /* Set CS# high */

  // FIXME: DELAY of 10ms?
}

static void set_slow_spi_mode(void)
{
  spiConfigureSlow();
}

static void set_fast_spi_mode(void)
{
  spiConfigureFast();
}

/* Exchange a byte */
static BYTE xchg_spi(BYTE dat)
{
  BYTE receive;

  spiExchange(1, &dat, &receive);
  return (BYTE)receive;
}

/* Receive multiple byte */
static void rcvr_spi_multi(BYTE *buff, UINT btr)
{
  memset(exchangeBuff, 0xFFFFFFFF, btr);
  spiExchange(btr, exchangeBuff, buff);
}

/* Send multiple byte */
static void xmit_spi_multi(const BYTE *buff, UINT btx)
{
  spiExchange(btx, buff, exchangeBuff);
}

static void cs_high(void)
{
  digitalWrite(USD_CS_PIN, 1);
}

static void cs_low(void)
{
  digitalWrite(USD_CS_PIN, 0);
}



/*********** Deck driver initialization ***************/

static bool isInit = false;

static void usdInit(DeckInfo *info)
{
  xTaskCreate(usdTask, "usdTask", 2*configMINIMAL_STACK_SIZE, NULL, /*priority*/0, NULL);

  usdDataQueue = xQueueCreate(USD_DATAQUEUE_ITEMS, sizeof(UsdLogStruct));

  if (usdDataQueue)
  {
    isInit = true;
  }

  FATFS_AddDriver(&fatDrv, 0);

  timer = xTimerCreate( "usdTimer", M2T(SD_DISK_TIMER_PERIOD_MS), pdTRUE, NULL, usdTimer);
  xTimerStart(timer, 0);
}

static bool usdTest()
{
  if (!isInit)
  {
    DEBUG_PRINT("Error while initializing uSD deck\n");
  }
#ifdef USD_RUN_DISKIO_FUNCTION_TESTS
  int result;
  extern int test_diskio (BYTE pdrv, UINT ncyc, DWORD* buff, UINT sz_buff);

  result = test_diskio(0, 1, (DWORD*)&workBuff, sizeof(workBuff));
  if (result)
  {
    DEBUG_PRINT("(result=%d)\nFatFs diskio functions test [FAIL].\n", result);
    isInit = false;
  }
  else
  {
    DEBUG_PRINT("FatFs diskio functions test [OK].\n");
  }
#endif

  return isInit;
}

static void usdTimer(xTimerHandle timer)
{
  SD_disk_timerproc(&sdSpiContext);
}

bool usdQueueLogData(UsdLogStruct* logData)
{
  if (!usdDataQueue || !isInit)
  {
    return 0;
  }
  else
  {
    return (xQueueSendToBack(usdDataQueue, logData, 0) == pdTRUE);
  }
}

static const DeckDriver usd_deck = {
  .vid = 0xBC,
  .pid = 0x07,
  .name = "bcUSD",
  .usedGpio = DECK_USING_MISO|DECK_USING_MOSI|DECK_USING_SCK|DECK_USING_IO_4,
  .usedPeriph = DECK_USING_SPI,
  .init = usdInit,
  .test = usdTest,
};

DECK_DRIVER(usd_deck);
