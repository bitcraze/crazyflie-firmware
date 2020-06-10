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
#include <ctype.h>
#include "stm32fxxx.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#include "ff.h"
#include "fatfs_sd.h"

#include "deck.h"
#include "usddeck.h"
#include "system.h"
#include "sensors.h"
#include "debug.h"
#include "led.h"

#include "log.h"
#include "param.h"
#include "crc_bosch.h"
#include "static_mem.h"

// Hardware defines
#ifdef USDDECK_USE_ALT_PINS_AND_SPI
#include "deck_spi3.h"
#define USD_CS_PIN    DECK_GPIO_RX2

#define SPI_BEGIN               spi3Begin
#define USD_SPI_BAUDRATE_2MHZ   SPI3_BAUDRATE_2MHZ
#define USD_SPI_BAUDRATE_21MHZ  SPI3_BAUDRATE_21MHZ
#define SPI_EXCHANGE            spi3Exchange
#define SPI_BEGIN_TRANSACTION   spi3BeginTransaction
#define SPI_END_TRANSACTION     spi3EndTransaction


#else
#include "deck_spi.h"
#define USD_CS_PIN    DECK_GPIO_IO4

#define SPI_BEGIN               spiBegin
#define USD_SPI_BAUDRATE_2MHZ   SPI_BAUDRATE_2MHZ
#define USD_SPI_BAUDRATE_21MHZ  SPI_BAUDRATE_21MHZ
#define SPI_EXCHANGE            spiExchange
#define SPI_BEGIN_TRANSACTION   spiBeginTransaction
#define SPI_END_TRANSACTION     spiEndTransaction
#endif

typedef struct usdLogConfig_s {
  char filename[13];
  uint8_t items;
  uint16_t frequency;
  uint8_t bufferSize;
  uint16_t numSlots;
  uint16_t numBytes;
  int* varIds; // dynamically allocated
  bool enableOnStartup;
  enum usddeckLoggingMode_e mode;
} usdLogConfig_t;

#define USD_WRITE(FILE, MESSAGE, BYTES, BYTES_WRITTEN, CRC_VALUE, CRC_FINALXOR, CRC_TABLE) \
  f_write(FILE, MESSAGE, BYTES, BYTES_WRITTEN); \
  CRC_VALUE = crcByByte(MESSAGE, BYTES, CRC_VALUE, CRC_FINALXOR, CRC_TABLE);

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

NO_DMA_CCM_SAFE_ZERO_INIT static crc crcTable[256];

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
static SemaphoreHandle_t logFileMutex;

static QueueHandle_t usdLogQueue;
static uint8_t* usdLogBufferStart;
static uint8_t* usdLogBuffer;
static TaskHandle_t xHandleWriteTask;

static bool enableLogging;
static uint32_t lastFileSize = 0;

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
  SPI_BEGIN();   /* Enable SPI function */
  spiSpeed = USD_SPI_BAUDRATE_2MHZ;

  pinMode(USD_CS_PIN, OUTPUT);
  digitalWrite(USD_CS_PIN, 1);

  // FIXME: DELAY of 10ms?
}

static void setSlowSpiMode(void)
{
  spiSpeed = USD_SPI_BAUDRATE_2MHZ;
}

static void setFastSpiMode(void)
{
  spiSpeed = USD_SPI_BAUDRATE_21MHZ;
}

/* Exchange a byte */
static BYTE xchgSpi(BYTE dat)
{
  BYTE receive;

  SPI_EXCHANGE(1, &dat, &receive);
  return (BYTE)receive;
}

/* Receive multiple byte */
static void rcvrSpiMulti(BYTE *buff, UINT btr)
{
  memset(exchangeBuff, 0xFFFFFFFF, btr);
  SPI_EXCHANGE(btr, exchangeBuff, buff);
}

/* Send multiple byte */
static void xmitSpiMulti(const BYTE *buff, UINT btx)
{
  SPI_EXCHANGE(btx, buff, exchangeBuff);
}

static void csHigh(void)
{
  digitalWrite(USD_CS_PIN, 1);

  // Dummy clock (force DO hi-z for multiple slave SPI)
  // Moved here from fatfs_sd.c to handle bus release
  xchgSpi(0xFF);

  SPI_END_TRANSACTION();
}

static void csLow(void)
{
  SPI_BEGIN_TRANSACTION(spiSpeed);
  digitalWrite(USD_CS_PIN, 0);
}

/********** FS helper function ***************/

// reads a line and returns the string without any whitespace/comment
//  * comments are indicated by #
//  * a line ending is marked by \n
//  * only up to "len" will be read
TCHAR* f_gets_without_comments (
  TCHAR* buff,  /* Pointer to the string buffer to read */
  int len,    /* Size of string buffer (characters) */
  FIL* fp     /* Pointer to the file object */
)
{
  int n = 0;
  TCHAR c, *p = buff;
  UINT rc;
  bool isComment = false;

  while (n < len - 1) { /* Read characters until buffer gets filled */
    f_read(fp, &c, 1, &rc);
    if (rc != 1) {
      break;
    }
    if (c == '\n') {
      break;   /* Break on EOL */
    }
    if (isspace((int)c)) {
      continue; /* Strip whitespace */
    }
    if (c == '#') {
      isComment = true; /* keep reading until end of line */
    }
    if (!isComment) {
      *p++ = c;
      n++;
    }
  }
  *p = 0;
  return n ? buff : 0;      /* When no data read (eof or error), return with error. */
}


/*********** Deck driver initialization ***************/

static bool isInit = false;
static bool initSuccess = false;

static void usdInit(DeckInfo *info)
{
  if (!isInit) {
    logFileMutex = xSemaphoreCreateMutex();
    /* create driver structure */
    FATFS_AddDriver(&fatDrv, 0);
    vTaskDelay(M2T(100));
    /* try to mount drives before creating the tasks */
    if (f_mount(&FatFs, "", 1) == FR_OK) {
      DEBUG_PRINT("mount SD-Card [OK].\n");
      /* try to open config file */
      while (f_open(&logFile, "config.txt", FA_READ) == FR_OK) {
        /* try to read configuration */
        char readBuffer[32];
        char* endptr;
        TCHAR* line = f_gets_without_comments(readBuffer, sizeof(readBuffer), &logFile);
        if (!line) break;
        usdLogConfig.frequency = strtol(line, &endptr, 10);
        // strtol(line, &usdLogConfig.frequency, 10);
        line = f_gets_without_comments(readBuffer, sizeof(readBuffer), &logFile);
        if (!line) break;
        usdLogConfig.bufferSize = strtol(line, &endptr, 10);
        // strtol(line, &usdLogConfig.bufferSize, 10);
        line = f_gets_without_comments(usdLogConfig.filename, sizeof(usdLogConfig.filename), &logFile);
        if (!line) break;

        int l = strlen(usdLogConfig.filename);
        if (l > sizeof(usdLogConfig.filename) - 3) {
          l = sizeof(usdLogConfig.filename) - 3;
        }
        usdLogConfig.filename[l] = '0';
        usdLogConfig.filename[l+1] = '0';
        usdLogConfig.filename[l+2] = 0;

        line = f_gets_without_comments(readBuffer, sizeof(readBuffer), &logFile);
        if (!line) break;
        usdLogConfig.enableOnStartup = strtol(line, &endptr, 10);

        line = f_gets_without_comments(readBuffer, sizeof(readBuffer), &logFile);
        if (!line) break;
        usdLogConfig.mode = strtol(line, &endptr, 10);

        usdLogConfig.numSlots = 0;
        usdLogConfig.numBytes = 0;
        while (line) {
          line = f_gets_without_comments(readBuffer, sizeof(readBuffer), &logFile);
          if (!line) break;
          char* group = line;
          char* name = 0;
          for (int i = 0; i < strlen(line); ++i) {
            if (line[i] == '.') {
              line[i] = 0;
              name = &line[i+1];
              i = strlen(name);
              break;
            }
          }
          int varid = logGetVarId(group, name);
          if (varid == -1) {
            DEBUG_PRINT("Unknown log variable %s.%s\n", group, name);
            continue;
          }

          ++usdLogConfig.numSlots;
          usdLogConfig.numBytes += logVarSize(logGetType(varid));
        }
        f_close(&logFile);

        DEBUG_PRINT("Config read [OK].\n");
        DEBUG_PRINT("Frequency: %dHz. Buffer size: %d\n",
                    usdLogConfig.frequency, usdLogConfig.bufferSize);
        DEBUG_PRINT("enOnStartup: %d. mode: %d\n", usdLogConfig.enableOnStartup, usdLogConfig.mode);
        DEBUG_PRINT("slots: %d, %d\n", usdLogConfig.numSlots, usdLogConfig.numBytes);

        /* create usd-log task */
        xTaskCreate(usdLogTask, USDLOG_TASK_NAME,
                    USDLOG_TASK_STACKSIZE, NULL,
                    USDLOG_TASK_PRI, NULL);

        initSuccess = true;
        break;
      }

      if (!initSuccess) {
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

  systemWaitStart();
  /* wait until sensor calibration is done
   * (memory of bias calculation buffer is free again) */
  while(!sensorsAreCalibrated()) {
    vTaskDelayUntil(&lastWakeTime, F2T(10));
  }

  usdLogConfig.varIds = pvPortMalloc(usdLogConfig.numSlots * sizeof(int));
  //DEBUG_PRINT("Free heap: %d bytes\n", xPortGetFreeHeapSize());

  // store logging variable ids
  {
    uint32_t idx = 0;

    while (f_open(&logFile, "config.txt", FA_READ) == FR_OK) {
      /* try to read configuration */
      char readBuffer[32];
      TCHAR* line;

      // skip first 5 lines
      for (int i = 0; i < 5; ++i) {
        line = f_gets_without_comments(readBuffer, sizeof(readBuffer), &logFile);
        if (!line) break;
      }

      while (line) {
        line = f_gets_without_comments(readBuffer, sizeof(readBuffer), &logFile);
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

  /* allocate memory for buffer */
  DEBUG_PRINT("malloc buffer %d bytes...\n", usdLogConfig.bufferSize * (4 + usdLogConfig.numBytes));
  // vTaskDelay(10); // small delay to allow debug message to be send
  usdLogBufferStart =
      pvPortMalloc(usdLogConfig.bufferSize * (4 + usdLogConfig.numBytes));
  usdLogBuffer = usdLogBufferStart;
  if (usdLogBufferStart)
  {
    DEBUG_PRINT("[OK].\n");
  }
  else
  {
    DEBUG_PRINT("[FAIL].\n");
  }
  DEBUG_PRINT("Free heap: %d bytes\n", xPortGetFreeHeapSize());

  /* create queue to hand over pointer to usdLogData */
  usdLogQueue = xQueueCreate(usdLogConfig.bufferSize, sizeof(uint8_t*));

  xHandleWriteTask = 0;
  enableLogging = usdLogConfig.enableOnStartup; // enable logging if desired

  /* create usd-write task */
  xTaskCreate(usdWriteTask, USDWRITE_TASK_NAME,
              USDWRITE_TASK_STACKSIZE, usdLogQueue,
              USDWRITE_TASK_PRI, &xHandleWriteTask);

  bool lastEnableLogging = enableLogging;
  while(1) {
    vTaskDelayUntil(&lastWakeTime, F2T(usdLogConfig.frequency));

    // if logging was just disabled, resume the writer task to give up mutex
    if (!enableLogging && lastEnableLogging != enableLogging) {
      vTaskResume(xHandleWriteTask);
    }

    if (enableLogging && usdLogConfig.mode == usddeckLoggingMode_Asyncronous) {
      usddeckTriggerLogging();
    }
    lastEnableLogging = enableLogging;
  }
}

bool usddeckLoggingEnabled(void)
{
  return enableLogging;
}

enum usddeckLoggingMode_e usddeckLoggingMode(void)
{
  return usdLogConfig.mode;
}

int usddeckFrequency(void)
{
  return usdLogConfig.frequency;
}

void usddeckTriggerLogging(void)
{
  uint8_t queueMessagesWaiting = (uint8_t)uxQueueMessagesWaiting(usdLogQueue);

  /* trigger writing once there exists at least one queue item,
   * frequency will result itself */
  if (queueMessagesWaiting && xHandleWriteTask) {
    vTaskResume(xHandleWriteTask);
  }
  /* skip if queue is full, one slot will be spared as mutex */
  if (queueMessagesWaiting == (usdLogConfig.bufferSize - 1)) {
    return;
  }

  /* write data into buffer */
  uint32_t ticks = xTaskGetTickCount();
  memcpy(usdLogBuffer, &ticks, 4);
  int offset = 4;
  for (int i = 0; i < usdLogConfig.numSlots; ++i) {
    int varid = usdLogConfig.varIds[i];
    switch (logGetType(varid)) {
      case LOG_UINT8:
      case LOG_INT8:
      {
        memcpy(usdLogBuffer + offset, logGetAddress(varid), sizeof(uint8_t));
        offset += sizeof(uint8_t);
        break;
      }
      case LOG_UINT16:
      case LOG_INT16:
      {
        memcpy(usdLogBuffer + offset, logGetAddress(varid), sizeof(uint16_t));
        offset += sizeof(uint16_t);
        break;
      }
      case LOG_UINT32:
      case LOG_INT32:
      case LOG_FLOAT:
      {
        memcpy(usdLogBuffer + offset, logGetAddress(varid), sizeof(uint32_t));
        offset += sizeof(uint32_t);
        break;
      }
      default:
        ASSERT(false);
    }
  }
  /* set pointer on latest data and queue */
  xQueueSend(usdLogQueue, &usdLogBuffer, 0);
  /* set pointer to next buffer item */
  usdLogBuffer = usdLogBuffer + 4 + usdLogConfig.numBytes;
  if (usdLogBuffer >= usdLogBufferStart + usdLogConfig.bufferSize * (4 + usdLogConfig.numBytes)) {
    usdLogBuffer = usdLogBufferStart;
  }
}

// returns size of current file if logging is stopped (0 otherwise)
uint32_t usddeckFileSize(void)
{
  return lastFileSize;
}

// Read "length" number of bytes at "offset" into "buffer" of current file
// Only works if logging is stopped
bool usddeckRead(uint32_t offset, uint8_t* buffer, uint16_t length)
{
  bool result = false;
  if (initSuccess && xSemaphoreTake(logFileMutex, 0) == pdTRUE) {
    if (f_open(&logFile, usdLogConfig.filename, FA_READ) == FR_OK) {
      if (f_lseek(&logFile, offset) == FR_OK) {
        UINT bytesRead;
        FRESULT r = f_read(&logFile, buffer, length, &bytesRead);
        f_close(&logFile);
        if (r == FR_OK && bytesRead == length) {
          result = true;
        }
      } else {
        f_close(&logFile);
      }
    }
    xSemaphoreGive(logFileMutex);
  }
  return result;
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

  while (true)
  {
    vTaskSuspend(NULL);
    if (enableLogging) {
      xSemaphoreTake(logFileMutex, portMAX_DELAY);
      lastFileSize = 0;
      usdLogBuffer = usdLogBufferStart;
      xQueueReset(usdLogQueue);
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
          == FR_OK) {

        DEBUG_PRINT("Filename: %s\n", usdLogConfig.filename);

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

        uint8_t* usdLogQueuePtr;

        while (enableLogging) {
          /* sleep */
          vTaskSuspend(NULL);
          /* determine how many sets can be written */
          setsToWrite = (uint8_t)uxQueueMessagesWaiting(usdLogQueue);
          if (setsToWrite > 0) {
            /* try to open file in append mode in every iteration to avoid
               loss of data during/after a crash */
            if (f_open(&logFile, usdLogConfig.filename, FA_OPEN_APPEND | FA_WRITE)
                != FR_OK) {
              continue;
            }
            f_write(&logFile, &setsToWrite, 1, &bytesWritten);
            crcValue = crcByByte(&setsToWrite, 1, INITIAL_REMAINDER, 0, crcTable);
            do {
              /* receive data pointer from queue */
              xQueueReceive(usdLogQueue, &usdLogQueuePtr, 0);
              /* write binary data and point on next item */
              USD_WRITE(&logFile, usdLogQueuePtr,
                        4 + usdLogConfig.numBytes, &bytesWritten, crcValue, 0, crcTable)
            } while(--setsToWrite);
            /* final xor and negate crc value */
            crcValue = ~(crcValue^FINAL_XOR_VALUE);
            f_write(&logFile, &crcValue, 4, &bytesWritten);
            /* close file */
            f_close(&logFile);
          }
        }

        // Update file size for fast query
        FILINFO info;
        if (f_stat(usdLogConfig.filename, &info) == FR_OK) {
          lastFileSize = info.fsize;
        }

        xSemaphoreGive(logFileMutex);
      } else {
        f_mount(NULL, "", 0);
        DEBUG_PRINT("Failed to open file: %s\n", usdLogConfig.filename);
        break;
      }
    }
  }
  /* something went wrong */
  xHandleWriteTask = 0;
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

PARAM_GROUP_START(usd)
PARAM_ADD(PARAM_UINT8, logging, &enableLogging) /* use to start/stop logging*/
PARAM_GROUP_STOP(usd)
