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
 * cfassert.c - Assert implementation
 */

#define DEBUG_MODULE "SYS"

#include <stdint.h>
#include "FreeRTOS.h"
#include "cfassert.h"
#include "led.h"
#include "motors.h"
#include "debug.h"

#define MAGIC_ASSERT_INDICATOR 0x2f8a001f

typedef struct SNAPSHOT_DATA {
  uint32_t magicNumber;
  char* fileName;
  int line;
} SNAPSHOT_DATA;

// The .nzds section is not cleared at startup, data here will survive a
// reset (by the watch dog for instance)
SNAPSHOT_DATA snapshot __attribute__((section(".nzds"))) = {
  .magicNumber = 0,
  .fileName = "",
  .line = 0
};


void assertFail(char *exp, char *file, int line)
{
  portDISABLE_INTERRUPTS();
  storeAssertSnapshotData(file, line);
  DEBUG_PRINT("Assert failed %s:%d\n", file, line);

  motorsSetRatio(MOTOR_M1, 0);
  motorsSetRatio(MOTOR_M2, 0);
  motorsSetRatio(MOTOR_M3, 0);
  motorsSetRatio(MOTOR_M4, 0);

  ledClearAll();
  ledSet(ERR_LED1, 1);
  ledSet(ERR_LED2, 1);

  while (1);
}

void storeAssertSnapshotData(char *file, int line)
{
  snapshot.magicNumber = MAGIC_ASSERT_INDICATOR;
  snapshot.fileName = file;
  snapshot.line = line;
}

void printAssertSnapshotData()
{
  if (MAGIC_ASSERT_INDICATOR == snapshot.magicNumber) {
    DEBUG_PRINT("Assert failed at %s:%d\n", snapshot.fileName, snapshot.line);
  } else {
    DEBUG_PRINT("No assert information found\n");
  }
}



