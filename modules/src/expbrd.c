/**
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
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
 * expbrd.c - Expansion board handling functions
 */
#define DEBUG_MODULE "EXPBRD"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "expbrd.h"
#include "debug.h"
#include "ow.h"
#include "crc.h"
#include "exptest.h"
#include "neopixelring.h"
#include "imu.h"
#include "config.h"

static bool isInit;
static uint8_t nBoards = 0;

static ExpbrdData expbrdData[EXPBRD_MAX];

static void expbrdPrintData(ExpbrdData *data);
static bool expbrdIsValid(ExpbrdData *data);
static bool expbrdScan(void);
static bool expbrdIsPresent(uint8_t vid, uint8_t pid);

void expbrdInit()
{
  if(isInit)
    return;

  owInit();
  expbrdScan();

#ifndef FORCE_EXP_DETECT
  if (expbrdIsPresent(EXPBRD_VID_BITCRAZE, EXPBRD_PID_LEDRING))
#endif
  {
    neopixelringInit();
  }

  isInit = true;
}

bool expbrdTest()
{
  bool status = true;

  if(!isInit)
    return false;

  // For production test
  if (expbrdIsPresent(EXPBRD_VID_BITCRAZE, EXPBRD_PID_ET))
  {
    status &= imu6ManufacturingTest();
    status &= exptestRun();
    if (status)
    {
      DEBUG_PRINT("Expansion port test [OK].\n");
    }
    else
    {
      DEBUG_PRINT("Expansion port test [FAIL].\n");
    }
  }
  status &= owTest();

  return status;
}

static bool expbrdScan(void)
{
  uint8_t i = 0;
  bool status = false;

  if (owScan(&nBoards))
  {
    DEBUG_PRINT("Found %d memories.\n", nBoards);
  }
  else
  {
    DEBUG_PRINT("Scan [FAILED].\n");
  }

  for (i = 0; i < nBoards; i++)
  {
    if (owRead(i, EXPBRD_OW_ADDR, sizeof(ExpbrdData), (uint8_t *)&expbrdData[i]))
    {
      if (expbrdIsValid(&expbrdData[i]))
      {
        DEBUG_PRINT("Info board %i:\n", i);
        expbrdPrintData(&expbrdData[i]);
        status = true;
      }
    }
    else
    {
      DEBUG_PRINT("Reading board nr:%d [FAILED].\n", i);
    }
  }

  return status;
}

static bool expbrdIsValid(ExpbrdData *data)
{
  bool status = false;
  uint8_t crc;

  if (data->header == EXPBRD_ID)
  {
    crc = crcSlow(data, sizeof(ExpbrdData) - 1);
    if (crc == data->crc)
    {
      status = true;
    }
    else
    {
      DEBUG_PRINT("Wrong CRC:0x%X!=0x%X\n", crc, data->crc);
    }
  }
  else
  {
    DEBUG_PRINT("Wrong header id: 0x%X\n", data->header);
  }

  return status;
}

static void expbrdPrintData(ExpbrdData *data)
{
  consolePrintf(" -Header:%X\n", data->header);
  consolePrintf(" -Pins  :0x%X\n", *(unsigned int*)&data->usedPins);
  consolePrintf(" -Vid   :%X\n", data->vid);
  consolePrintf(" -Pid   :%X\n", data->pid);
  consolePrintf(" -crc   :%X\n", data->crc);
}

static bool expbrdIsPresent(uint8_t vid, uint8_t pid)
{
  int i;

  if (nBoards == 0)
    return false;

  for (i = 0; i < EXPBRD_MAX; i++)
  {
    if (expbrdData[i].vid == vid && expbrdData[i].pid == pid)
    {
      return true;
    }
  }

  return false;
}
