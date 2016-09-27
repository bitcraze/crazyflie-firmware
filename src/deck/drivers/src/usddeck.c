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
#include "semphr.h"
#include "task.h"

#include "tm_stm32f4_fatfs.h"

#include "deck.h"
#include "system.h"
#include "debug.h"
#include "log.h"
#include "param.h"

#define DATABUFFER_SIZE 1000
#define DATABUFFER_HALF (DATABUFFER_SIZE/2)

typedef struct
{
  uint32_t timestamp;
  float gx;
  float gy;
  float gz;
  float ax;
  float ay;
  float az;
  float mx;
  float my;
  float mz;
}  __attribute__((packed)) DataStruct;

//Fatfs object
FATFS FatFs;
//File object
FIL logFile;

DataStruct dataBuf[DATABUFFER_SIZE];
uint32_t writePtr = 0;



/*********** Tasks ************/
static void usdTask(void *param)
{
  //Free and total space
  uint32_t total, free;
  bool fileStatus = false;
  int gxid, gyid, gzid, axid, ayid, azid, mxid, myid, mzid;

systemWaitStart();

  //xLastWakeTime = xTaskGetTickCount();


  gxid = logGetVarId("gyro", "x");
  gyid = logGetVarId("gyro", "y");
  gzid = logGetVarId("gyro", "z");
  axid = logGetVarId("acc", "x");
  ayid = logGetVarId("acc", "y");
  azid = logGetVarId("acc", "z");
  mxid = logGetVarId("mag", "x");
  myid = logGetVarId("mag", "y");
  mzid = logGetVarId("mag", "z");

  //Mount drive
  if (f_mount(&FatFs, "", 1) == FR_OK)
  {
    DEBUG_PRINT("Drive mounted [OK]\n");
    //Try to open file
    if (f_open(&logFile, "log.bin", FA_CREATE_ALWAYS | FA_READ | FA_WRITE) == FR_OK)
    {
      DEBUG_PRINT("File opened [OK]\n");
      if (f_puts("timestamp, gx, gy, gz, ax, ay, az, mx, my, mz\n", &logFile) > 0)
      {
        fileStatus = true;
      }
      //Close file, don't forget this!
      f_close(&logFile);
    }
  }



  while (1)
  {
    if (fileStatus)
    {
      if (f_open(&logFile, "log.bin", FA_OPEN_ALWAYS | FA_READ | FA_WRITE) == FR_OK)
      {
      dataBuf[writePtr].timestamp = xTaskGetTickCount();
      dataBuf[writePtr].gx = logGetFloat(gxid);
      dataBuf[writePtr].gy = logGetFloat(gyid);
      dataBuf[writePtr].gz = logGetFloat(gzid);
      dataBuf[writePtr].ax = logGetFloat(axid);
      dataBuf[writePtr].ay = logGetFloat(ayid);
      dataBuf[writePtr].az = logGetFloat(azid);
      dataBuf[writePtr].mx = logGetFloat(mxid);
      dataBuf[writePtr].my = logGetFloat(myid);
      dataBuf[writePtr].mz = logGetFloat(mzid);
      writePtr++;


      logGetFloat(pitchid)
    }
    else
    {
      DEBUG_PRINT("Drive mounted [FAIL]\n");
    }

    //Unmount drive, don't forget this!
    f_mount(0, "", 1);

    vTaskDelay(M2T(100000));
    //vTaskDelayUntil(&xLastWakeTime, M2T(10));
  }
}

/*********** Deck driver initialization ***************/

static bool isInit = false;

static void usdInit(DeckInfo *info)
{
  xTaskCreate(usdTask, "usdTask", 2*configMINIMAL_STACK_SIZE, NULL, /*priority*/2, NULL);

  isInit = true;
}

static bool usdTest()
{
  if (!isInit) {
    DEBUG_PRINT("Error while initializing uSD deck\n");
  }

  return isInit;
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
