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
#include <stdlib.h>
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
#include "sensors.h"
#include "debug.h"
#include "led.h"

#include "log.h"
#include "param.h"
#include "crc_bosch.h"

// Hardware defines
#define USD_CS_PIN    DECK_GPIO_IO4

// FATFS low lever driver functions.
static void initSpi(void);
static void setSlowSpiMode(void);
static void setFastSpiMode(void);
static BYTE xchgSpi(BYTE dat);
static void rcvrSpiMulti(BYTE *buff, UINT btr);
static void xmitSpiMulti(const BYTE *buff, UINT btx);
static void csHigh(void);
static void csLow(void);

static void usdLogTask(void* prm);
static void usdWriteTask(void* prm);

static crc crcTable[256];

static usdLogConfig_t usdLogConfig;

static BYTE exchangeBuff[512];
static uint16_t spiSpeed;

#ifdef USD_RUN_DISKIO_FUNCTION_TESTS
DWORD workBuff[512];  /* 2048 byte working buffer */
#endif

//Fatfs object
static FATFS FatFs;
//File object
static FIL logFile;

static xTimerHandle timer;
static void usdTimer(xTimerHandle timer);


// Low lever driver functions
static sdSpiContext_t sdSpiContext =
    {
        .initSpi = initSpi,
        .setSlowSpiMode = setSlowSpiMode,
        .setFastSpiMode = setFastSpiMode,
        .xchgSpi = xchgSpi,
        .rcvrSpiMulti = rcvrSpiMulti,
        .xmitSpiMulti = xmitSpiMulti,
        .csLow = csLow,
        .csHigh = csHigh,

        .stat = STA_NOINIT,
        .timer1 = 0,
        .timer2 = 0
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


/*-----------------------------------------------------------------------*/
/* FATFS SPI controls (Platform dependent)                               */
/*-----------------------------------------------------------------------*/

/* Initialize MMC interface */
static void initSpi(void)
{
  spiBegin();   /* Enable SPI function */
  spiSpeed = SPI_BAUDRATE_2MHZ;

  pinMode(USD_CS_PIN, OUTPUT);
  digitalWrite(USD_CS_PIN, 1);

  // FIXME: DELAY of 10ms?
}

static void setSlowSpiMode(void)
{
  spiSpeed = SPI_BAUDRATE_2MHZ;
}

static void setFastSpiMode(void)
{
  spiSpeed = SPI_BAUDRATE_21MHZ;
}

/* Exchange a byte */
static BYTE xchgSpi(BYTE dat)
{
  BYTE receive;

  spiExchange(1, &dat, &receive);
  return (BYTE)receive;
}

/* Receive multiple byte */
static void rcvrSpiMulti(BYTE *buff, UINT btr)
{
  memset(exchangeBuff, 0xFFFFFFFF, btr);
  spiExchange(btr, exchangeBuff, buff);
}

/* Send multiple byte */
static void xmitSpiMulti(const BYTE *buff, UINT btx)
{
  spiExchange(btx, buff, exchangeBuff);
}

static void csHigh(void)
{
  digitalWrite(USD_CS_PIN, 1);

  // Dummy clock (force DO hi-z for multiple slave SPI)
  // Moved here from fatfs_sd.c to handle bus release
  xchgSpi(0xFF);

  spiEndTransaction();
}

static void csLow(void)
{
  spiBeginTransaction(spiSpeed);
  digitalWrite(USD_CS_PIN, 0);
}



/*********** Deck driver initialization ***************/

static bool isInit = false;

static void usdInit(DeckInfo *info)
{
  if (!isInit) {
    /* create driver structure */
    FATFS_AddDriver(&fatDrv, 0);
    vTaskDelay(M2T(100));
    /* try to mount drives before creating the tasks */
    if (f_mount(&FatFs, "", 1) == FR_OK) {
      DEBUG_PRINT("mount SD-Card [OK].\n");
      /* try to open config file */
      bool success = false;
      while (f_open(&logFile, "config.txt", FA_READ) == FR_OK) {
        /* try to read configuration */
        char readBuffer[32];
        char* endptr;
        TCHAR* line = f_gets(readBuffer, sizeof(readBuffer), &logFile);
        if (!line) break;
        usdLogConfig.frequency = strtol(line, &endptr, 10);
        // strtol(line, &usdLogConfig.frequency, 10);
        line = f_gets(readBuffer, sizeof(readBuffer), &logFile);
        if (!line) break;
        usdLogConfig.bufferSize = strtol(line, &endptr, 10);
        // strtol(line, &usdLogConfig.bufferSize, 10);
        line = f_gets(usdLogConfig.filename, sizeof(usdLogConfig.filename), &logFile);
        if (!line) break;

        int l = strlen(usdLogConfig.filename);
        if (l > sizeof(usdLogConfig.filename) - 2) {
          l = sizeof(usdLogConfig.filename) - 2;
        }
        usdLogConfig.filename[l-1] = '0';
        usdLogConfig.filename[l] = '0';
        usdLogConfig.filename[l+1] = 0;

        usdLogConfig.numSlots = 0;
        usdLogConfig.numBytes = 0;
        while (line) {
          line = f_gets(readBuffer, sizeof(readBuffer), &logFile);
          if (!line) break;
          char* group = line;
          char* name = 0;
          for (int i = 0; i < strlen(line); ++i) {
            if (line[i] == '.') {
              line[i] = 0;
              name = &line[i+1];
              i = strlen(name);
              if (name[i-1] == '\n') {
                name[i-1] = 0; // remove newline at the end
              }
              break;
            }
          }
          int varid = logGetVarId(group, name);
          if (varid == -1) {
            DEBUG_PRINT("Unknown log variable %s.%s.\n", group, name);
            continue;
          }

          ++usdLogConfig.numSlots;
          usdLogConfig.numBytes += logVarSize(logGetType(varid));
        }
        f_close(&logFile);

        DEBUG_PRINT("Config read [OK].\n");
        DEBUG_PRINT("Frequency: %dHz. Buffer size: %d\n",
                    usdLogConfig.frequency, usdLogConfig.bufferSize);
        DEBUG_PRINT("Filename: %s.\n", usdLogConfig.filename);
        DEBUG_PRINT("slots: %d, %d.\n", usdLogConfig.numSlots, usdLogConfig.numBytes);

        /* create usd-log task */
        xTaskCreate(usdLogTask, USDLOG_TASK_NAME,
                    USDLOG_TASK_STACKSIZE, NULL,
                    USDLOG_TASK_PRI, NULL);

        success = true;
        break;
      }
      
      if (!success) {
          DEBUG_PRINT("Config read [FAIL].\n");
      }
    }
    else {
      DEBUG_PRINT("mount SD-Card [FAIL].\n");
    }
  }
  isInit = true;
}

static void usdLogTask(void* prm)
{
  TickType_t lastWakeTime = xTaskGetTickCount();

  DEBUG_PRINT("wait for sensors\n");

  /* wait until sensor calibration is done
   * (memory of bias calculation buffer is free again) */
  while(!sensorsAreCalibrated()) {
    vTaskDelayUntil(&lastWakeTime, F2T(10));
  }

  usdLogConfig.varIds = pvPortMalloc(usdLogConfig.numSlots * sizeof(int*));
  DEBUG_PRINT("Free heap: %d bytes\n", xPortGetFreeHeapSize());

  // store logging variable ids
  {
    uint32_t idx = 0;

    while (f_open(&logFile, "config.txt", FA_READ) == FR_OK) {
      /* try to read configuration */
      char readBuffer[32];
      TCHAR* line = f_gets(readBuffer, sizeof(readBuffer), &logFile);
      if (!line) break;
      line = f_gets(readBuffer, sizeof(readBuffer), &logFile);
      if (!line) break;
      line = f_gets(readBuffer, sizeof(readBuffer), &logFile);
      if (!line) break;

      while (line) {
        line = f_gets(readBuffer, sizeof(readBuffer), &logFile);
        if (!line) break;
        char* group = line;
        char* name = 0;
        for (int i = 0; i < strlen(line); ++i) {
          if (line[i] == '.') {
            line[i] = 0;
            name = &line[i+1];
            i = strlen(name);
            if (name[i-1] == '\n') {
              name[i-1] = 0; // remove newline at the end
            }
            break;
          }
        }
        int varid = logGetVarId(group, name);
        if (varid == -1) {
          continue;
        }

        usdLogConfig.varIds[idx++] = varid;
      }
      break;
    }
    f_close(&logFile);
  }

  /* struct definition for buffering data to write
   * requires up to 100 elements for 1kHz logging */
  struct usdLogStruct {
    uint32_t tick;
    uint8_t data[usdLogConfig.numBytes];
  };

  /* allocate memory for buffer */
  DEBUG_PRINT("malloc buffer ...\n");
  // vTaskDelay(10); // small delay to allow debug message to be send
  struct usdLogStruct* usdLogBufferStart =
      pvPortMalloc(usdLogConfig.bufferSize * sizeof(struct usdLogStruct));
  struct usdLogStruct* usdLogBuffer = usdLogBufferStart;
  DEBUG_PRINT("[OK].\n");
  DEBUG_PRINT("Free heap: %d bytes\n", xPortGetFreeHeapSize());

  /* create queue to hand over pointer to usdLogData */
  QueueHandle_t usdLogQueue =
      xQueueCreate(usdLogConfig.bufferSize, sizeof(usdLogQueuePtr_t));

  /* create usd-write task */
  TaskHandle_t xHandleWriteTask;
  xTaskCreate(usdWriteTask, USDWRITE_TASK_NAME,
              USDWRITE_TASK_STACKSIZE, usdLogQueue,
              USDWRITE_TASK_PRI, &xHandleWriteTask);

  usdLogQueuePtr_t usdLogQueuePtr;
  uint8_t queueMessagesWaiting = 0;

  while(1) {
    vTaskDelayUntil(&lastWakeTime, F2T(usdLogConfig.frequency));
    queueMessagesWaiting = (uint8_t)uxQueueMessagesWaiting(usdLogQueue);
    /* trigger writing once there exists at least one queue item,
     * frequency will result itself */
    if (queueMessagesWaiting) {
      vTaskResume(xHandleWriteTask);
    }
    /* skip if queue is full, one slot will be spared as mutex */
    if (queueMessagesWaiting == (usdLogConfig.bufferSize - 1)) {
      continue;
    }

    /* write data into buffer */
    usdLogBuffer->tick = lastWakeTime;
    int offset = 0;
    for (int i = 0; i < usdLogConfig.numSlots; ++i) {
      int varid = usdLogConfig.varIds[i];
      switch (logGetType(varid)) {
        case LOG_UINT8:
        case LOG_INT8:
        {
          memcpy(&usdLogBuffer->data[offset], logGetAddress(varid), sizeof(uint8_t));
          offset += sizeof(uint8_t);
          break;
        }
        case LOG_UINT16:
        case LOG_INT16:
        {
          memcpy(&usdLogBuffer->data[offset], logGetAddress(varid), sizeof(uint16_t));
          offset += sizeof(uint16_t);
          break;
        }
        case LOG_UINT32:
        case LOG_INT32:
        case LOG_FLOAT:
        {
          memcpy(&usdLogBuffer->data[offset], logGetAddress(varid), sizeof(uint32_t));
          offset += sizeof(uint32_t);
          break;
        }
        default:
          ASSERT(false);
      }
    }

    /* set pointer on latest data and queue */
    usdLogQueuePtr.tick = &usdLogBuffer->tick;
    usdLogQueuePtr.data = usdLogBuffer->data;
    xQueueSend(usdLogQueue, &usdLogQueuePtr, 0);
    /* set pointer to next buffer item */
    if (++usdLogBuffer >= usdLogBufferStart+usdLogConfig.bufferSize) {
      usdLogBuffer = usdLogBufferStart;
    }
  }
}

static void usdWriteTask(void* usdLogQueue)
{
  /* necessary variables for f_write */
  unsigned int bytesWritten;
  uint8_t setsToWrite = 0;

  /* iniatialize crc and create lookup-table */
  crc crcValue;
  crcTableInit(crcTable);

  /* create and start timer for card control timing */
  timer = xTimerCreate("usdTimer", M2T(SD_DISK_TIMER_PERIOD_MS),
                       pdTRUE, NULL, usdTimer);
  xTimerStart(timer, 0);

  vTaskDelay(M2T(50));
  /* look for existing files and use first not existent combination
   * of two chars */
  {
    FILINFO fno;
    uint8_t NUL = 0;
    while(usdLogConfig.filename[NUL] != '\0') {
      NUL++;
    }
    while (f_stat(usdLogConfig.filename, &fno) == FR_OK) {
      /* increase file */
      switch(usdLogConfig.filename[NUL-1]) {
        case '9':
          usdLogConfig.filename[NUL-1] = '0';
          usdLogConfig.filename[NUL-2]++;
          break;
        default:
          usdLogConfig.filename[NUL-1]++;
      }
    }
  }

  /* try to create file */
  if (f_open(&logFile, usdLogConfig.filename, FA_CREATE_ALWAYS | FA_WRITE)
      == FR_OK)
    {
      /* write dataset header */
      {
        uint8_t logWidth = 1 + usdLogConfig.numSlots;
        f_write(&logFile, &logWidth, 1, &bytesWritten);
        crcValue = crcByByte(&logWidth, 1, INITIAL_REMAINDER, 0, crcTable);
      }
      USD_WRITE(&logFile, (uint8_t*)"tick(I),", 8, &bytesWritten,
                crcValue, 0, crcTable)

      for (int i = 0; i < usdLogConfig.numSlots; ++i) {
        char* group;
        char* name;
        int varid = usdLogConfig.varIds[i];
        logGetGroupAndName(varid, &group, &name);
        USD_WRITE(&logFile, (uint8_t*)group, strlen(group), &bytesWritten,
          crcValue, 0, crcTable)
        USD_WRITE(&logFile, (uint8_t*)".", 1, &bytesWritten,
          crcValue, 0, crcTable)
        USD_WRITE(&logFile, (uint8_t*)name, strlen(name), &bytesWritten,
          crcValue, 0, crcTable)
        USD_WRITE(&logFile, (uint8_t*)"(", 1, &bytesWritten,
                    crcValue, 0, crcTable)
        char typeChar;
        switch (logGetType(varid)) {
          case LOG_UINT8:
            typeChar = 'B';
            break;
          case LOG_INT8:
            typeChar = 'b';
            break;
          case LOG_UINT16:
            typeChar = 'H';
            break;
          case LOG_INT16:
            typeChar = 'h';
            break;
          case LOG_UINT32:
            typeChar = 'I';
            break;
          case LOG_INT32:
            typeChar = 'i';
            break;
          case LOG_FLOAT:
            typeChar = 'f';
            break;
          default:
            ASSERT(false);
        }
        USD_WRITE(&logFile, (uint8_t*)&typeChar, 1, &bytesWritten,
                    crcValue, 0, crcTable)
        USD_WRITE(&logFile, (uint8_t*)"),", 2, &bytesWritten,
                    crcValue, 0, crcTable)
      }

      /* negate crc value */
      crcValue = ~(crcValue^FINAL_XOR_VALUE);
      f_write(&logFile, &crcValue, 4, &bytesWritten);
      f_close(&logFile);

      usdLogQueuePtr_t usdLogQueuePtr;

      while (1) {
        /* sleep */
        vTaskSuspend(NULL);
        /* determine how many sets can be written */
        setsToWrite = (uint8_t)uxQueueMessagesWaiting(usdLogQueue);
        /* try to open file in append mode */
        if (f_open(&logFile, usdLogConfig.filename, FA_OPEN_APPEND | FA_WRITE)
            != FR_OK)
          continue;
        f_write(&logFile, &setsToWrite, 1, &bytesWritten);
        crcValue = crcByByte(&setsToWrite, 1, INITIAL_REMAINDER, 0, crcTable);
        do {
          /* receive data pointer from queue */
          xQueueReceive(usdLogQueue, &usdLogQueuePtr, 0);
          /* write binary data and point on next item */
          USD_WRITE(&logFile, (uint8_t*)usdLogQueuePtr.tick, 4,
                    &bytesWritten, crcValue, 0, crcTable)
          USD_WRITE(&logFile, (uint8_t*)usdLogQueuePtr.data,
                    usdLogConfig.numBytes, &bytesWritten, crcValue, 0, crcTable)
        } while(--setsToWrite);
        /* final xor and negate crc value */
        crcValue = ~(crcValue^FINAL_XOR_VALUE);
        f_write(&logFile, &crcValue, 4, &bytesWritten);
        /* close file */
        f_close(&logFile);
      }
  } else {
    f_mount(NULL, "", 0);
  }
  /* something went wrong */
  vTaskDelete(NULL);
}

static bool usdTest()
{
  if (!isInit) {
    DEBUG_PRINT("Error while initializing uSD deck\n");
  }
#ifdef USD_RUN_DISKIO_FUNCTION_TESTS
  int result;
  extern int test_diskio (BYTE pdrv, UINT ncyc, DWORD* buff, UINT sz_buff);

  result = test_diskio(0, 1, (DWORD*)&workBuff, sizeof(workBuff));
  if (result) {
    DEBUG_PRINT("(result=%d)\nFatFs diskio functions test [FAIL].\n", result);
    isInit = false;
  }
  else {
    DEBUG_PRINT("FatFs diskio functions test [OK].\n");
  }
#endif

  return isInit;
}

static void usdTimer(xTimerHandle timer)
{
  SD_disk_timerproc(&sdSpiContext);
}

static const DeckDriver usd_deck = {
    .vid = 0xBC,
    .pid = 0x08,
    .name = "bcUSD",
    .usedGpio = DECK_USING_MISO|DECK_USING_MOSI|DECK_USING_SCK|DECK_USING_IO_4,
    .usedPeriph = DECK_USING_SPI,
    .init = usdInit,
    .test = usdTest,
};

DECK_DRIVER(usd_deck);

PARAM_GROUP_START(deck)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcUSD, &isInit)
PARAM_GROUP_STOP(deck)
