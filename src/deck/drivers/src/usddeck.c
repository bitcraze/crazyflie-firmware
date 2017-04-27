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

#include "log.h"
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
  pinMode(USD_CS_PIN, OUTPUT);
  csHigh();

  // FIXME: DELAY of 10ms?
}

static void setSlowSpiMode(void)
{
  spiConfigureSlow();
}

static void setFastSpiMode(void)
{
  spiConfigureFast();
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
}

static void csLow(void)
{
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
			/* try to open config file */
			if (f_open(&logFile, "config", FA_READ) == FR_OK) {/* try to read configuration */
				char readBuffer[5];
				unsigned int bytesRead;
				if (f_read(&logFile, readBuffer, 4, &bytesRead) == FR_OK) {
					/* assign bytes to config struct */
					usdLogConfig.items = readBuffer[0];
					usdLogConfig.frequency = readBuffer[1]<<8 | readBuffer[2];
					usdLogConfig.bufferSize = readBuffer[3];
					if ( (f_read(&logFile, usdLogConfig.filename, 10, &bytesRead)!= FR_OK) && ~bytesRead ) {
						bytesRead = 12;
						while(bytesRead--)
							usdLogConfig.filename[bytesRead] = '0';
						usdLogConfig.filename[12] = '\0';
					}
					else {
						usdLogConfig.filename[bytesRead + 0] = '0';
						usdLogConfig.filename[bytesRead + 1] = '0';
						usdLogConfig.filename[bytesRead + 2] = '\0';
					}

					f_close(&logFile);

					usdLogConfig.floatSlots = ((usdLogConfig.items & USDLOG_ACC) ? 3 : 0)
#ifdef LOG_SEC_IMU
							+ ((usdLogConfig.items & USDLOG_ACC) ? 3 : 0)
#endif
							+ ((usdLogConfig.items & USDLOG_GYRO) ? 3 : 0)
#ifdef LOG_SEC_IMU
							+ ((usdLogConfig.items & USDLOG_GYRO) ? 3 : 0)
#endif
							+ ((usdLogConfig.items & USDLOG_BARO) ? 3 : 0)
							+ ((usdLogConfig.items & USDLOG_MAG) ? 3 : 0)
							+ ((usdLogConfig.items & USDLOG_STABILIZER) ? 4 : 0)
							+ ((usdLogConfig.items & USDLOG_CONTROL) ? 3 : 0);
					usdLogConfig.intSlots = ((usdLogConfig.items & USDLOG_RANGE) ? 1 : 0);

					/* create usd-log task */
					xTaskCreate(usdLogTask, USDLOG_TASK_NAME,
							USDLOG_TASK_STACKSIZE, NULL, USDLOG_TASK_PRI, NULL);
				}
			}
		}
	}
	isInit = true;
}

void usdLogTask(void* prm)
{
	int floatIds[usdLogConfig.floatSlots];
	int intIds[usdLogConfig.intSlots];
	{
		uint8_t usedSlots = 0;
		/* acquire log ids */
		if (usdLogConfig.items & USDLOG_ACC) {
			floatIds[0] = logGetVarId("acc", "x");
			floatIds[1] = logGetVarId("acc", "y");
			floatIds[2] = logGetVarId("acc", "z");
			usedSlots += 3;
#ifdef LOG_SEC_IMU
			floatIds[0 + usedSlots] = logGetVarId("accSpare", "x");
			floatIds[1 + usedSlots] = logGetVarId("accSpare", "y");
			floatIds[2 + usedSlots] = logGetVarId("accSpare", "z");
			usedSlots += 3;
#endif
		}
		if (usdLogConfig.items & USDLOG_GYRO) {
			floatIds[0 + usedSlots] = logGetVarId("gyro", "x");
			floatIds[1 + usedSlots] = logGetVarId("gyro", "y");
			floatIds[2 + usedSlots] = logGetVarId("gyro", "z");
			usedSlots += 3;
#ifdef LOG_SEC_IMU
			floatIds[0 + usedSlots] = logGetVarId("gyroSpare", "x");
			floatIds[1 + usedSlots] = logGetVarId("gyroSpare", "y");
			floatIds[2 + usedSlots] = logGetVarId("gyroSpare", "z");
			usedSlots += 3;
#endif
		}
		if (usdLogConfig.items & USDLOG_BARO) {
			floatIds[0 + usedSlots] = logGetVarId("baro", "asl");
			floatIds[1 + usedSlots] = logGetVarId("baro", "temp");
			floatIds[2 + usedSlots] = logGetVarId("baro", "pressure");
			usedSlots += 3;
		}
		if (usdLogConfig.items & USDLOG_MAG) {
			floatIds[0 + usedSlots] = logGetVarId("mag", "x");
			floatIds[1 + usedSlots] = logGetVarId("mag", "y");
			floatIds[2 + usedSlots] = logGetVarId("mag", "z");
			usedSlots += 3;
		}
		if (usdLogConfig.items & USDLOG_STABILIZER) {
			floatIds[0 + usedSlots] = logGetVarId("stabilizer", "roll");
			floatIds[1 + usedSlots] = logGetVarId("stabilizer", "pitch");
			floatIds[2 + usedSlots] = logGetVarId("stabilizer", "yaw");
			floatIds[3 + usedSlots] = logGetVarId("stabilizer", "thrust");
			usedSlots += 4;
		}
		if (usdLogConfig.items & USDLOG_CONTROL) {
			floatIds[0 + usedSlots] = logGetVarId("ctrltarget", "roll");
			floatIds[1 + usedSlots] = logGetVarId("ctrltarget", "pitch");
			floatIds[2 + usedSlots] = logGetVarId("ctrltarget", "yaw");
			usedSlots += 3;
		}
		usedSlots = 0;
		if (usdLogConfig.items & USDLOG_RANGE) {
			intIds[0] = logGetVarId("range", "zrange");
			usedSlots += 1;
		}
	}

    TickType_t lastWakeTime = xTaskGetTickCount();
	int i;
	uint8_t usdLogBufferIdx = usdLogConfig.bufferSize - 1;

	/* struct for buffering data to write
	 * requires up to 100 elements for 1kHz logging */
	struct usdLogStruct {
		uint32_t tick;
		float floats[usdLogConfig.floatSlots];
		int ints[usdLogConfig.intSlots];
	} usdLogData[usdLogConfig.bufferSize];

	/* create queue to hand over pointer to usdLogData */
	QueueHandle_t usdLogQueue = xQueueCreate( usdLogConfig.bufferSize, sizeof(usdLogDataPtr_t) );

	/* create usd-write task */
	TaskHandle_t xHandleWriteTask;
	xTaskCreate(usdWriteTask, USDWRITE_TASK_NAME,
			USDWRITE_TASK_STACKSIZE, usdLogQueue, USDWRITE_TASK_PRI, &xHandleWriteTask);

	/*  */
	usdLogDataPtr_t usdLogDataPtr;
	uint8_t queueMessagesWaiting;

	//systemWaitStart();
    while(1) {
		vTaskDelayUntil(&lastWakeTime, F2T(usdLogConfig.frequency));
		queueMessagesWaiting = uxQueueMessagesWaiting(usdLogQueue);
		/* skip if queue is still full */
		if (queueMessagesWaiting != usdLogConfig.bufferSize){
			/* write data into buffer */
			usdLogData[usdLogBufferIdx].tick = lastWakeTime;
			for (i = usdLogConfig.floatSlots-1; i >= 0; i--) {
				usdLogData[usdLogBufferIdx].floats[i] = logGetFloat(floatIds[i]);
			}
			for (i = usdLogConfig.intSlots-1; i >= 0; i--) {
				usdLogData[usdLogBufferIdx].ints[i] = logGetInt(intIds[i]);
			}
			/* set pointer on latest data and queue */
			usdLogDataPtr.tick = &usdLogData[usdLogBufferIdx].tick;
			usdLogDataPtr.floats = usdLogData[usdLogBufferIdx].floats;
			usdLogDataPtr.ints = usdLogData[usdLogBufferIdx].ints;
			xQueueSend(usdLogQueue, &usdLogDataPtr, 0);
			/* decrease BufferIdx */
			usdLogBufferIdx =
					( (usdLogBufferIdx == 0) ? (usdLogConfig.bufferSize - 1) : (usdLogBufferIdx - 1) );
		}
     	/* start writing at quarter buffer size
     	 * (seems to work good for high frequencies) */
		if (queueMessagesWaiting > (usdLogConfig.bufferSize >> 2) )
			vTaskResume(xHandleWriteTask);
    }
}

static void usdWriteTask(void* usdLogQueue)
{
	/* necessary variables for f_write */
	unsigned int bytesWritten;
	uint8_t setsInQueue = 0;

	/* iniatialize crc and create lookup-table */
	crc crcValue;
	crcTableInit(crcTable);

	/* create and start timer for card control timing */
	timer = xTimerCreate( "usdTimer", M2T(SD_DISK_TIMER_PERIOD_MS), pdTRUE, NULL, usdTimer);
	xTimerStart(timer, 0);

	vTaskDelay(M2T(50));
	/* look for existing files and use first not existent combination of two chars */
	{
		FILINFO fno;
		uint8_t NUL = 0;
		while(usdLogConfig.filename[NUL] != '\0')
			NUL++;
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
	if (f_open(&logFile, usdLogConfig.filename, FA_CREATE_ALWAYS | FA_WRITE) == FR_OK) {
		/* write dataset header */
		{
		uint8_t logWidth = 1 + usdLogConfig.floatSlots + usdLogConfig.intSlots;
		f_write(&logFile, &logWidth, 1, &bytesWritten);
		crcValue = crcByByte(&logWidth, 1, INITIAL_REMAINDER, 0, crcTable);
		}
		USD_WRITE(&logFile, (uint8_t*)"Itick", 5, &bytesWritten, crcValue, 0, crcTable)

     	if (usdLogConfig.items & USDLOG_ACC) {
			USD_WRITE(&logFile, (uint8_t*)"faccxfaccyfaccz", 15, &bytesWritten, crcValue, 0, crcTable)
#ifdef LOG_SEC_IMU
			USD_WRITE(&logFile, (uint8_t*)"fac2xfac2yfac2z", 15, &bytesWritten, crcValue, 0, crcTable)
#endif
    	}

    	if (usdLogConfig.items & USDLOG_GYRO) {
			USD_WRITE(&logFile, (uint8_t*)"fgyrxfgyryfgyrz", 15, &bytesWritten, crcValue, 0, crcTable)
#ifdef LOG_SEC_IMU
			USD_WRITE(&logFile, (uint8_t*)"fgy2xfgy2yfgy2z", 15, &bytesWritten, crcValue, 0, crcTable)
#endif
    	}

    	if (usdLogConfig.items & USDLOG_BARO) {
			USD_WRITE(&logFile, (uint8_t*)"f aslftempfpres", 15, &bytesWritten, crcValue, 0, crcTable)
    	}

    	if (usdLogConfig.items & USDLOG_MAG) {
			USD_WRITE(&logFile, (uint8_t*)"fmagxfmagyfmagz", 15, &bytesWritten, crcValue, 0, crcTable)
    	}

    	if (usdLogConfig.items & USDLOG_STABILIZER) {
			USD_WRITE(&logFile, (uint8_t*)"fsrolfspitfsyawfsthr", 20, &bytesWritten, crcValue, 0, crcTable)
    	}

    	if (usdLogConfig.items & USDLOG_CONTROL) {
			USD_WRITE(&logFile, (uint8_t*)"fcrolfcpitfcyaw", 15, &bytesWritten, crcValue, 0, crcTable)
    	}

    	if (usdLogConfig.items & USDLOG_RANGE) {
			USD_WRITE(&logFile, (uint8_t*)"irang", 5, &bytesWritten, crcValue, 0, crcTable)
    	}

		/* negate crc value */
		crcValue = ~(crcValue^FINAL_XOR_VALUE);
		f_write(&logFile, &crcValue, 4, &bytesWritten);
		f_close(&logFile);

		uint8_t floatBytes = usdLogConfig.floatSlots * 4;
		uint8_t intBytes = usdLogConfig.intSlots * 4;
		usdLogDataPtr_t usdLogDataPtr;

		//systemWaitStart();
		while (1) {
			/* sleep */
			vTaskSuspend(NULL);
			/* try to open file in append mode */
			if (f_open(&logFile, usdLogConfig.filename, FA_OPEN_APPEND | FA_WRITE) != FR_OK)
				continue;
			/* determine how many sets can be written */
			setsInQueue = uxQueueMessagesWaiting(usdLogQueue);
			f_write(&logFile, &setsInQueue, 1, &bytesWritten);
			crcValue = crcByByte(&setsInQueue, 1, INITIAL_REMAINDER, 0, crcTable);
			do {
				/* receive data pointer from queue */
				xQueueReceive(usdLogQueue, &usdLogDataPtr, 0);
				/* write binary data and point on next item */
				USD_WRITE(&logFile, (uint8_t*)usdLogDataPtr.tick, 4, &bytesWritten, crcValue, 0, crcTable)
				if (usdLogConfig.floatSlots)
					USD_WRITE(&logFile, (uint8_t*)usdLogDataPtr.floats, floatBytes, &bytesWritten, crcValue, 0, crcTable)
				if (usdLogConfig.intSlots)
					USD_WRITE(&logFile, (uint8_t*)usdLogDataPtr.ints, intBytes, &bytesWritten, crcValue, 0, crcTable)
			} while(--setsInQueue);
			/* final xor and negate crc value */
			crcValue = ~(crcValue^FINAL_XOR_VALUE);
			f_write(&logFile, &crcValue, 4, &bytesWritten);
			/* close file */
			f_close(&logFile);
		}
	} else f_mount(NULL, "", 0);
	/* something went wrong */
	vTaskDelete(NULL);
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
