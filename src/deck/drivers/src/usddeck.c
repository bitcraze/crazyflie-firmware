/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2016-2021 Bitcraze AB
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
#include "task.h"
#include "timers.h"
#include "semphr.h"

#include "ff.h"
#include "diskio.h"
#include "fatfs_sd.h"

#include "deck.h"
#include "usddeck.h"
#include "system.h"
#include "sensors.h"
#include "debug.h"
#include "led.h"
#include "pm.h"

#include "statsCnt.h"
#include "log.h"
#include "param.h"
#include "crc32.h"
#include "static_mem.h"
#include "mem.h"
#include "eventtrigger.h"

#include "autoconf.h"

// Hardware defines
#ifdef CONFIG_DECK_USD_USE_ALT_PINS_AND_SPI
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

#define MAX_USD_LOG_VARIABLES_PER_EVENT   (20)
#define MAX_USD_LOG_EVENTS                (20)
#define FIXED_FREQUENCY_EVENT_ID          (0xFFFF)
#define FIXED_FREQUENCY_EVENT_NAME        "fixedFrequency"


/* set to true when graceful shutdown is triggered */
static volatile bool in_shutdown = false;

typedef struct usdLogEventConfig_s {
  uint16_t eventId;
  uint8_t numVars;
  uint16_t numBytes;
  logVarId_t varIds[MAX_USD_LOG_VARIABLES_PER_EVENT];
} usdLogEventConfig_t;

typedef struct usdLogConfig_s {
  char filename[13];
  uint16_t frequency;
  uint16_t bufferSize;
  bool enableOnStartup;
  enum usddeckLoggingMode_e mode;

  uint32_t numEventConfigs;
  usdLogEventConfig_t eventConfigs[MAX_USD_LOG_EVENTS];
  uint8_t fixedFrequencyEventIdx;
} usdLogConfig_t;

typedef struct usdLogStats_s {
  uint32_t eventsRequested;
  uint32_t eventsWritten;
} usdLogStats_t;

// Ring buffer
typedef struct ringBuffer_s {
  uint8_t* buffer;        // pointer to buffer
  uint16_t capacity;      // total capacity of buffer
  uint16_t size;          // used size of buffer
  uint8_t* readPtr;       // pointer for read/pop
  uint8_t* writePtr;      // pointer for write/push
  uint16_t popSize;       // size for ongoing pop operation
} ringBuffer_t;

void ringBuffer_init(ringBuffer_t* b, uint8_t *buffer, uint16_t capacity)
{
  b->buffer = buffer;
  b->capacity = capacity;
  b->size = 0;
  b->readPtr = buffer;
  b->writePtr = buffer;
  b->popSize = 0;
}

void ringBuffer_reset(ringBuffer_t *b)
{
  b->size = 0;
  b->readPtr = b->buffer;
  b->writePtr = b->buffer;
  b->popSize = 0;
}

uint16_t ringBuffer_availableSpace(const ringBuffer_t* b)
{
  return b->capacity - b->size;
}

bool ringBuffer_push(ringBuffer_t* b, const void* data, uint16_t size)
{
  if (ringBuffer_availableSpace(b) < size) {
    return false;
  }
  const uint8_t* dataTyped = (const uint8_t*)data;
  for (uint16_t i = 0; i < size; ++i) {
    *(b->writePtr) = dataTyped[i];
    ++b->writePtr;
    if (b->writePtr ==  b->buffer + b->capacity) {
      b->writePtr = b->buffer;
    }
  }
  b->size += size;
  return true;
}

bool ringBuffer_pop_start(ringBuffer_t* b, const uint8_t** buf, uint16_t* size)
{
  if (b->size == 0) {
    return false;
  }

  *buf = b->readPtr;
  if (b->writePtr > b->readPtr) {
    // writer did not wrap around yet
    *size = b->writePtr - b->readPtr;
    b->readPtr = b->writePtr;
  } else {
    // wrap around -> read until end of buffer, only
    *size = b->buffer + b->capacity - b->readPtr;
    b->readPtr = b->buffer;
  }
  b->popSize = *size;
  return true;
}

void ringBuffer_pop_done(ringBuffer_t *b)
{
  b->size -= b->popSize;
  b->popSize = 0;
}

// FATFS low lever driver functions.
static void initSpi(void);
static void setSlowSpiMode(void);
static void setFastSpiMode(void);
static BYTE xchgSpi(BYTE dat);
static void rcvrSpiMulti(BYTE *buff, UINT btr);
static void xmitSpiMulti(const BYTE *buff, UINT btx);
static void csHigh(BYTE doDummyClock);
static void csLow(void);
static void delayMs(UINT ms);

static void usdLogTask(void* prm);
static void usdWriteTask(void* prm);

static STATS_CNT_RATE_DEFINE(spiWriteRate, 1000);
static STATS_CNT_RATE_DEFINE(spiReadRate, 1000);
static STATS_CNT_RATE_DEFINE(fatWriteRate, 1000);

static usdLogConfig_t usdLogConfig;
static usdLogStats_t usdLogStats;

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

static SemaphoreHandle_t logBufferMutex;
static ringBuffer_t logBuffer;
static TaskHandle_t xHandleWriteTask;

static bool enableLogging;
static uint32_t lastFileSize = 0;
static crc32Context_t crcContext;

static xTimerHandle timer;
static void usdTimer(xTimerHandle timer);

static SemaphoreHandle_t shutdownMutex;

// Handling from the memory module
static uint32_t handleMemGetSize(void) { return usddeckFileSize(); }
static bool handleMemRead(const uint32_t memAddr, const uint8_t readLen, uint8_t* buffer);
static const MemoryHandlerDef_t memDef = {
  .type = MEM_TYPE_USD,
  .getSize = handleMemGetSize,
  .read = handleMemRead,
  .write = 0, // Write not supported
};


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
        .delayMs = delayMs,

        .stat = STA_NOINIT,
        .timer1 = 0,
        .timer2 = 0
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

  STATS_CNT_RATE_EVENT(&spiReadRate);
  STATS_CNT_RATE_EVENT(&spiWriteRate);

  SPI_EXCHANGE(1, &dat, &receive);
  return (BYTE)receive;
}

/* Receive multiple byte */
static void rcvrSpiMulti(BYTE *buff, UINT btr)
{
  STATS_CNT_RATE_MULTI_EVENT(&spiReadRate, btr);

  memset(exchangeBuff, 0xFFFFFFFF, btr);
  SPI_EXCHANGE(btr, exchangeBuff, buff);
}

/* Send multiple byte */
static void xmitSpiMulti(const BYTE *buff, UINT btx)
{
  STATS_CNT_RATE_MULTI_EVENT(&spiWriteRate, btx);

  SPI_EXCHANGE(btx, buff, exchangeBuff);
}

static void csHigh(BYTE doDummyClock)
{
  digitalWrite(USD_CS_PIN, 1);

  // Dummy clock (force DO hi-z for multiple slave SPI)
  // Moved here from fatfs_sd.c to handle bus release
  if (doDummyClock) {
    xchgSpi(0xFF);
  }

  SPI_END_TRANSACTION();
}

static void csLow(void)
{
  SPI_BEGIN_TRANSACTION(spiSpeed);
  digitalWrite(USD_CS_PIN, 0);
}

static void delayMs(UINT ms)
{
  vTaskDelay(M2T(ms));
}

/* FatFS Disk Interface */
DSTATUS disk_initialize(BYTE pdrv)
{
    return SD_disk_initialize(&sdSpiContext);
}

DSTATUS disk_status(BYTE pdrv)
{
  return SD_disk_status(&sdSpiContext);
}

DRESULT disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count)
{
  return SD_disk_read(buff, sector, count, &sdSpiContext);
}

DRESULT disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count)
{
  return SD_disk_write(buff, sector, count, &sdSpiContext);
}

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
  return SD_disk_ioctl(cmd, buff, &sdSpiContext);
}

/*-----------------------------------------------------------------------*/
/* Get time for fatfs for files                                          */
/*-----------------------------------------------------------------------*/
__attribute__((weak)) DWORD get_fattime(void)
{
  /* Returns current time packed into a DWORD variable */
  return ((DWORD)(2016 - 1980) << 25) /* Year 2016 */
         | ((DWORD)1 << 21)           /* Month 1 */
         | ((DWORD)1 << 16)           /* Mday 1 */
         | ((DWORD)0 << 11)           /* Hour 0 */
         | ((DWORD)0 << 5)            /* Min 0 */
         | ((DWORD)0 >> 1);           /* Sec 0 */
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
  bool isPureComment = false;

  while (n < len - 1) { /* Read characters until buffer gets filled */
    f_read(fp, &c, 1, &rc);
    if (rc != 1) {
      *p = 0;
      return 0; /* When no data read (eof or error), return with error. */
    }
    if (c == '\n') {
      if (isPureComment){
        isComment = false;
        isPureComment = false;
        continue;
      }      
      break;   /* Break on EOL */
    }
    if (isspace((int)c)) {
      continue; /* Strip whitespace */
    }
    if (c == '#') {
      isComment = true; /* keep reading until end of line */
      if (n==0){
        isPureComment = true;
      }
    }
    if (!isComment) {
      *p++ = c;
      n++;
    }
  }
  *p = 0;
  return buff;
}


/*********** Deck driver initialization ***************/

static bool isInit = false;
static bool initSuccess = false;

static void usdInit(DeckInfo *info)
{
  if (!isInit) {
    memoryRegisterHandler(&memDef);

    logFileMutex = xSemaphoreCreateMutex();
    logBufferMutex = xSemaphoreCreateMutex();
    shutdownMutex = xSemaphoreCreateMutex();

    /* try to mount drives before creating the tasks */
    if (f_mount(&FatFs, "", 1) == FR_OK) {
      DEBUG_PRINT("mount SD-Card [OK].\n");

      /* create usd-log task */
      xTaskCreate(usdLogTask, USDLOG_TASK_NAME,
                  USDLOG_TASK_STACKSIZE, NULL,
                  USDLOG_TASK_PRI, NULL);
    } else {
      DEBUG_PRINT("mount SD-Card [FAIL].\n");
    }
  }
  isInit = true;
}

static void usddeckWriteEventData(const usdLogEventConfig_t* cfg, const uint8_t* payload, uint8_t payloadSize)
{
  uint64_t ticks = usecTimestamp();

  if (!enableLogging) {
    return;
  }

  ++usdLogStats.eventsRequested;

  xSemaphoreTake(logBufferMutex, portMAX_DELAY);

  // trigger writing once there is some data
  if (logBuffer.size > 0 && xHandleWriteTask) {
    vTaskResume(xHandleWriteTask);
  }

  int dataSize = sizeof(cfg->eventId) + sizeof(ticks) + payloadSize + cfg->numBytes;

  // only write if we have enough space
  if (ringBuffer_availableSpace(&logBuffer) >= dataSize) {
    /* write data into buffer */
    uint16_t event_id = cfg->eventId;
    ringBuffer_push(&logBuffer, &event_id, sizeof(event_id));
    ringBuffer_push(&logBuffer, &ticks, sizeof(ticks));
    if (payloadSize) {
      ringBuffer_push(&logBuffer, payload, payloadSize);
    }

    for (int i = 0; i < cfg->numVars; ++i) {
      logVarId_t varid = cfg->varIds[i];
      switch (logGetType(varid)) {
      case LOG_UINT8:
      case LOG_INT8:
        ringBuffer_push(&logBuffer, logGetAddress(varid), sizeof(uint8_t));
        break;
      case LOG_UINT16:
      case LOG_INT16:
        ringBuffer_push(&logBuffer, logGetAddress(varid), sizeof(uint16_t));
        break;
      case LOG_UINT32:
      case LOG_INT32:
      case LOG_FLOAT:
        ringBuffer_push(&logBuffer, logGetAddress(varid), sizeof(uint32_t));
        break;
      default:
        ASSERT(false);
        break;
      }
    }
    ++usdLogStats.eventsWritten;
  }
  xSemaphoreGive(logBufferMutex);
}

static void usddeckEventtriggerCallback(const eventtrigger *event)
{
  uint16_t eventId = eventtriggerGetId(event);
  for (uint8_t i = 0; i < usdLogConfig.numEventConfigs; ++i) {
    if (usdLogConfig.eventConfigs[i].eventId == eventId) {
      usddeckWriteEventData(&usdLogConfig.eventConfigs[i], event->payload, event->payloadSize);
      break;
    }
  }
}

static void usdGracefulShutdownCallback()
{
  uint32_t timeout = 15; /* ms */
  in_shutdown = true;
  vTaskResume(xHandleWriteTask);
  xSemaphoreTake(shutdownMutex, M2T(timeout));
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

  // loop to break out in case of errors
  while (true) {
    /* open config file */
    // loop to break out in case of errors
    while (f_open(&logFile, "config.txt", FA_READ) == FR_OK) {
      /* try to read configuration */
      char readBuffer[32];
      char* endptr;

      // version
      TCHAR* line = f_gets_without_comments(readBuffer, sizeof(readBuffer), &logFile);
      if (!line) break;
      int version = strtol(line, &endptr, 10);
      if (version != 1) break;
      // buffer size
      line = f_gets_without_comments(readBuffer, sizeof(readBuffer), &logFile);
      if (!line) break;
      usdLogConfig.bufferSize = strtol(line, &endptr, 10);
      // file name
      line = f_gets_without_comments(usdLogConfig.filename, sizeof(usdLogConfig.filename), &logFile);
      if (!line) break;

      int l = strlen(usdLogConfig.filename);
      if (l > sizeof(usdLogConfig.filename) - 3) {
        l = sizeof(usdLogConfig.filename) - 3;
      }
      usdLogConfig.filename[l] = '0';
      usdLogConfig.filename[l+1] = '0';
      usdLogConfig.filename[l+2] = 0;

      // enable on startup
      line = f_gets_without_comments(readBuffer, sizeof(readBuffer), &logFile);
      if (!line) break;
      usdLogConfig.enableOnStartup = strtol(line, &endptr, 10);

      // loop over event triggers "on:<name>"
      usdLogConfig.numEventConfigs = 0;
      usdLogConfig.fixedFrequencyEventIdx = MAX_USD_LOG_EVENTS;
      usdLogConfig.frequency = 10; // use non-zero default value for task loop below
      usdLogEventConfig_t *cfg = &usdLogConfig.eventConfigs[0];
      const char* eventName = 0;
      line = f_gets_without_comments(readBuffer, sizeof(readBuffer), &logFile);
      while (line) {
        if (strncmp(line, "on:", 3) == 0) {
          // special mode for non-event-based logging
          if (strcmp(&line[3], FIXED_FREQUENCY_EVENT_NAME) == 0) {
            // frequency
            line = f_gets_without_comments(readBuffer, sizeof(readBuffer), &logFile);
            if (!line) break;
            usdLogConfig.frequency = strtol(line, &endptr, 10);
            // mode
            line = f_gets_without_comments(readBuffer, sizeof(readBuffer), &logFile);
            if (!line) break;
            usdLogConfig.mode = strtol(line, &endptr, 10);
            cfg->eventId = FIXED_FREQUENCY_EVENT_ID;
            eventName = FIXED_FREQUENCY_EVENT_NAME;
            usdLogConfig.fixedFrequencyEventIdx = usdLogConfig.numEventConfigs;
          } else {
            // handle event triggers
            const eventtrigger *et = eventtriggerGetByName(&line[3]);
            if (et) {
              cfg->eventId = eventtriggerGetId(et);
              eventName = et->name;
            } else {
              DEBUG_PRINT("Unknown event %s\n", &line[3]);
              line = f_gets_without_comments(readBuffer, sizeof(readBuffer), &logFile);
              continue;
            }
          }

          // Add log variables
          cfg->numVars = 0;
          cfg->numBytes = 0;
          while (true) {
            line = f_gets_without_comments(readBuffer, sizeof(readBuffer), &logFile);
            if (!line || strncmp(line, "on:", 3) == 0) {
              break;
            }
            // skip lines that do not have at least two characters (1 for group, 1 for '.', 1 for name)
            if (strlen(line) <= 3) {
              continue;
            }
            char *group = line;
            char *name = 0;
            for (int i = 0; i < strlen(line); ++i) {
              if (line[i] == '.') {
                line[i] = 0;
                name = &line[i + 1];
                i = strlen(name);
                break;
              }
            }
            logVarId_t varid = logGetVarId(group, name);
            if (!logVarIdIsValid(varid)) {
              DEBUG_PRINT("Unknown log variable %s.%s\n", group, name);
              continue;
            }
            if (cfg->numVars < MAX_USD_LOG_VARIABLES_PER_EVENT) {
              cfg->varIds[cfg->numVars] = varid;
              ++cfg->numVars;
              cfg->numBytes += logVarSize(logGetType(varid));
            } else {
              DEBUG_PRINT("Skip log variable %s: %s.%s (out of storage)\n", eventName, group, name);
              continue;
            }
          }
          if (usdLogConfig.numEventConfigs < MAX_USD_LOG_EVENTS - 1) {
            ++usdLogConfig.numEventConfigs;
            cfg = &usdLogConfig.eventConfigs[usdLogConfig.numEventConfigs];
          } else {
            DEBUG_PRINT("Skip config after event %s (out of storage)\n", eventName);
            break;
          }
        } else {
          line = f_gets_without_comments(readBuffer, sizeof(readBuffer), &logFile);
        }
      }
      f_close(&logFile);

      eventtriggerRegisterCallback(eventtriggerHandler_USD, &usddeckEventtriggerCallback);

      DEBUG_PRINT("Config read [OK].\n");
      // DEBUG_PRINT("Frequency: %d Hz. Buffer size: %d\n",
      //             usdLogConfig.frequency, usdLogConfig.bufferSize);
      // DEBUG_PRINT("enOnStartup: %d. mode: %d\n", usdLogConfig.enableOnStartup, usdLogConfig.mode);
      // DEBUG_PRINT("slots: %d, %d\n", usdLogConfig.numSlots, usdLogConfig.numBytes);
      initSuccess = true;
      break;
    }

    if (!initSuccess) {
      DEBUG_PRINT("Config read [FAIL].\n");
      break;
    }

    /* allocate memory for buffer */
    DEBUG_PRINT("malloc buffer %d bytes ", usdLogConfig.bufferSize);
    // vTaskDelay(10); // small delay to allow debug message to be send
    uint8_t* logBufferData = pvPortMalloc(usdLogConfig.bufferSize);
    if (logBufferData) {
      DEBUG_PRINT("[OK].\n");
    } else {
      DEBUG_PRINT("[FAIL].\n");
      break;
    }
    ringBuffer_init(&logBuffer, logBufferData, usdLogConfig.bufferSize);

    /* create queue to hand over pointer to usdLogData */
    // usdLogQueue = xQueueCreate(usdLogConfig.queueSize, sizeof(uint8_t*));

    xHandleWriteTask = 0;
    enableLogging = usdLogConfig.enableOnStartup; // enable logging if desired

    pmRegisterGracefulShutdownCallback(usdGracefulShutdownCallback);

    /* create usd-write task */
    xTaskCreate(usdWriteTask, USDWRITE_TASK_NAME,
                USDWRITE_TASK_STACKSIZE, 0,
                USDWRITE_TASK_PRI, &xHandleWriteTask);

    bool lastEnableLogging = enableLogging;
    while(1) {
      vTaskDelayUntil(&lastWakeTime, F2T(usdLogConfig.frequency));

      // if logging was just disabled, resume the writer task to give up mutex
      if (!enableLogging && lastEnableLogging != enableLogging) {
        vTaskResume(xHandleWriteTask);
      }

      if (enableLogging && usdLogConfig.mode == usddeckLoggingMode_Asynchronous) {
        usddeckTriggerLogging();
      }
      lastEnableLogging = enableLogging;
    }
  }

  /* something went wrong */
  vTaskDelete(NULL);
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
  if (usdLogConfig.fixedFrequencyEventIdx < MAX_USD_LOG_EVENTS) {
    usddeckWriteEventData(&usdLogConfig.eventConfigs[usdLogConfig.fixedFrequencyEventIdx], 0, 0);
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

static bool handleMemRead(const uint32_t memAddr, const uint8_t readLen, uint8_t* buffer) {
  bool result = false;

  if (memAddr + readLen <= usddeckFileSize()) {
    if (usddeckRead(memAddr, buffer, readLen)) {
      result = true;
    }
  }

  return result;
}

static void usdWriteData(const void *data, size_t size)
{
  UINT bytesWritten;
  FRESULT status = f_write(&logFile, data, size, &bytesWritten);
  ASSERT(status == FR_OK);
  crc32Update(&crcContext, data, size);
  STATS_CNT_RATE_MULTI_EVENT(&fatWriteRate, bytesWritten);
}

static void usdWriteTask(void* prm)
{
  /* create and start timer for card control timing */
  timer = xTimerCreate("usdTimer", M2T(SD_DISK_TIMER_PERIOD_MS),
                       pdTRUE, NULL, usdTimer);
  xTimerStart(timer, 0);

  vTaskDelay(M2T(50));

  while (!in_shutdown) {
    vTaskSuspend(NULL);
    if (enableLogging) {
      // reset stats
      usdLogStats.eventsRequested = 0;
      usdLogStats.eventsWritten = 0;

      // reset the buffer
      xSemaphoreTake(logBufferMutex, portMAX_DELAY);
      ringBuffer_reset(&logBuffer);
      xSemaphoreGive(logBufferMutex);

      xSemaphoreTake(logFileMutex, portMAX_DELAY);
      lastFileSize = 0;

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

        DEBUG_PRINT("Logging to: %s\n", usdLogConfig.filename);

        // iniatialize crc
        crc32ContextInit(&crcContext);

        // write header
        uint8_t magic = 0xBC;
        usdWriteData(&magic, sizeof(magic));

        uint16_t version = 2;
        usdWriteData(&version, sizeof(version));

        uint16_t numEventTypes = usdLogConfig.numEventConfigs;
        usdWriteData(&numEventTypes, sizeof(numEventTypes));

        for (int i = 0; i < numEventTypes; ++i) {
          usdLogEventConfig_t* cfg = &usdLogConfig.eventConfigs[i];
          const eventtrigger *et = eventtriggerGetById(cfg->eventId);
          uint16_t numVariables = cfg->numVars;

          usdWriteData(&cfg->eventId, sizeof(cfg->eventId));

          if (cfg->eventId == FIXED_FREQUENCY_EVENT_ID) {
            usdWriteData(FIXED_FREQUENCY_EVENT_NAME, strlen(FIXED_FREQUENCY_EVENT_NAME) + 1);
          } else {
            usdWriteData(et->name, strlen(et->name) + 1);
            numVariables += et->numPayloadVariables;
          }
          usdWriteData(&numVariables, sizeof(numVariables));
          if (et) {
            for (int j = 0; j < et->numPayloadVariables; ++j) {
              usdWriteData(et->payloadDesc[j].name, strlen(et->payloadDesc[j].name));
              usdWriteData("(", 1);
              char typeChar;
              switch (et->payloadDesc[j].type)
              {
              case eventtriggerType_uint8:
                typeChar = 'B';
                break;
              case eventtriggerType_int8:
                typeChar = 'b';
                break;
              case eventtriggerType_uint16:
                typeChar = 'H';
                break;
              case eventtriggerType_int16:
                typeChar = 'h';
                break;
              case eventtriggerType_uint32:
                typeChar = 'I';
                break;
              case eventtriggerType_int32:
                typeChar = 'i';
                break;
              case eventtriggerType_float:
                typeChar = 'f';
                break;
              case eventtrigerType_fp16:
                typeChar = 'e';
                break;
              default:
                ASSERT(false);
              }
              usdWriteData(&typeChar, 1);
              usdWriteData(")", 2);
            }
          }
          for (int j = 0; j < cfg->numVars; ++j) {
            char *group;
            char *name;
            logVarId_t varid = cfg->varIds[j];
            logGetGroupAndName(varid, &group, &name);
            usdWriteData(group, strlen(group));
            usdWriteData(".", 1);
            usdWriteData(name, strlen(name));
            usdWriteData("(", 1);
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
            usdWriteData(&typeChar, 1);
            usdWriteData(")", 2);
          }
        }

        while (enableLogging) {
          /* sleep */
          vTaskSuspend(NULL);

          // check if we have anything to write
          xSemaphoreTake(logBufferMutex, portMAX_DELAY);
          const uint8_t* buf;
          uint16_t size;
          bool hasData = ringBuffer_pop_start(&logBuffer, &buf, &size);
          xSemaphoreGive(logBufferMutex);

          // execute the actual write operation
          if (hasData) {
            usdWriteData(buf, size);

            xSemaphoreTake(logBufferMutex, portMAX_DELAY);
            ringBuffer_pop_done(&logBuffer);
            xSemaphoreGive(logBufferMutex);
          }
        }
        // write everything that's still in the buffer
        xSemaphoreTake(logBufferMutex, portMAX_DELAY);
        while (true) {
          const uint8_t *buf;
          uint16_t size;
          bool hasData = ringBuffer_pop_start(&logBuffer, &buf, &size);
          if (hasData) {
            usdWriteData(buf, size);
            ringBuffer_pop_done(&logBuffer);
          } else {
            break;
          }
        }
        xSemaphoreGive(logBufferMutex);

        // write CRC
        uint32_t crcValue = crc32Out(&crcContext);
        usdWriteData(&crcValue, sizeof(crcValue));

        // close file
        f_close(&logFile);

        // Update file size for fast query
        FILINFO info;
        if (f_stat(usdLogConfig.filename, &info) == FR_OK) {
          lastFileSize = info.fsize;
        }

        DEBUG_PRINT("Wrote %ld B to: %s (%ld of %ld events)\n",
          lastFileSize,
          usdLogConfig.filename,
          usdLogStats.eventsWritten,
          usdLogStats.eventsRequested);

        xSemaphoreGive(logFileMutex);
      } else {
        f_mount(NULL, "", 0);
        DEBUG_PRINT("Failed to open file: %s\n", usdLogConfig.filename);
        break;
      }
    }
  }

  if (in_shutdown) {
    xSemaphoreGive(shutdownMutex);
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
    .usedGpio = DECK_USING_IO_4,
    .usedPeriph = DECK_USING_SPI,
    .init = usdInit,
    .test = usdTest,
};

DECK_DRIVER(usd_deck);

PARAM_GROUP_START(deck)

/**
 * @brief Nonzero if [SD-card deck](%https://store.bitcraze.io/collections/decks/products/sd-card-deck) is attached
*/
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcUSD, &isInit)

PARAM_GROUP_STOP(deck)

/**
 * The micro SD card deck is used for on-board logging of data to a micro SD card.
 */
PARAM_GROUP_START(usd)
/**
 * @brief Non zero if logging is possible, 0 indicates there might be a problem with the logging configuration.
 */
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, canLog, &initSuccess)

/**
 * @brief Controls if logging to the SD-card is active. Set to 1 to start logging, set to 0 to stop logging (default).
 */
PARAM_ADD_CORE(PARAM_UINT8, logging, &enableLogging) /* use to start/stop logging*/
PARAM_GROUP_STOP(usd)

/**
 * Micro-SD related log variables for debug purposes mainly.
 */
LOG_GROUP_START(usd)
/**
 * @brief SPI write rate (includes overhead) [bytes/s]
 */
STATS_CNT_RATE_LOG_ADD(spiWrBps, &spiWriteRate)
/**
 * @brief SPI read rate (includes overhead) [bytes/s]
 */
STATS_CNT_RATE_LOG_ADD(spiReBps, &spiReadRate)
/**
 * @brief Data write rate to the SD card [bytes/s]
 */
STATS_CNT_RATE_LOG_ADD(fatWrBps, &fatWriteRate)
LOG_GROUP_STOP(usd)
