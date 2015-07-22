/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2015 Bitcraze AB
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
 * proximity.c - Implementation of hardware abstraction layer for proximity sensors
 */

#include <string.h>
#include "FreeRTOS.h"
#include "task.h"

#include "config.h"
#include "deck.h"
#include "proximity.h"
#include "mb.h"
#include "system.h"
#include "param.h"
#include "log.h"

#include "stm32fxxx.h"

static bool isInit = false;

/* The exported values. */
uint32_t proximityDistance  =  0; /* The distance measured in millimeters. */
uint32_t proximityAccuracy  =  0; /* The accuracy as reported by the sensor. */
uint16_t proximityFrequency = 10; /* The frequency at which to read sensors. Default is 10Hz. */

/* A sliding window used to calculate average distance. */
#define PROXIMITY_SWIN_SIZE 5                /* Number of samples in the sliding window. */
uint32_t proximitySWin[PROXIMITY_SWIN_SIZE]; /* Must be initialized before use. */
uint32_t proximityDistanceAvg = 0;           /* Average distance initialized to zero. */

LOG_GROUP_START(proximity)
LOG_ADD(LOG_UINT32, distance, &proximityDistance)
LOG_ADD(LOG_UINT32, distanceAvg, &proximityDistanceAvg)
LOG_ADD(LOG_UINT32, accuracy, &proximityAccuracy)
LOG_GROUP_STOP(proximity)

PARAM_GROUP_START(proximity)
PARAM_ADD(PARAM_UINT32, frequency, &proximityFrequency)
PARAM_GROUP_STOP(proximity)

#if defined(PROXIMITY_ENABLED)
/**
 * This function adds a distance measurement to the sliding window, discarding the oldest sample.
 * After having added the new sample, a new average value of the samples is calculated and returned.
 *
 * @param distance The new sample to add to the sliding window.
 *
 * @return The new average value of the samples in the sliding window (after adding the new sample).
 */
static uint32_t proximitySWinAdd(uint32_t distance)
{
  /* Discard oldest sample, move remaining samples one slot to the left. */
  memmove(&proximitySWin[0], &proximitySWin[1], (PROXIMITY_SWIN_SIZE - 1) * sizeof(uint32_t));

  /* Add the new sample in the last (right-most) slot. */
  proximitySWin[PROXIMITY_SWIN_SIZE - 1] = distance;

  /**
   * Calculate the new average distance. Sum all the samples into a uint64_t,
   * so that we only do a single division at the end.
   */
  uint64_t proximityNewAvg = 0;
  uint8_t n;
  for (n = 0; n < PROXIMITY_SWIN_SIZE; n++) {
    proximityNewAvg += proximitySWin[n];
  }
  proximityNewAvg = proximityNewAvg / PROXIMITY_SWIN_SIZE;

  return (uint32_t)proximityNewAvg;
}

static void proximityTask(void* param)
{
  uint32_t lastWakeTime;

  vTaskSetApplicationTaskTag(0, (void*)TASK_PROXIMITY_ID_NBR);

  //Wait for the system to be fully started to start stabilization loop
  systemWaitStart();

  lastWakeTime = xTaskGetTickCount();

  while(1)
  {
    vTaskDelayUntil(&lastWakeTime, F2T(proximityFrequency));

#if defined(MB_ENABLED)
    /* Read the MaxBotix sensor. */
    proximityDistance = mbReadDistance(MB1040AN, &proximityAccuracy);

    /* If the accuracy is considered better than 0 (0 means completely inaccurate), add the new sample. */
    proximityDistanceAvg = proximitySWinAdd(proximityDistance);
#endif
  }
}
#endif

void proximityInit(void)
{
  if(isInit)
    return;

  /* Initialise the sliding window to zero. */
  memset(&proximitySWin, 0, sizeof(uint32_t)*PROXIMITY_SWIN_SIZE);

#if defined(PROXIMITY_ENABLED)
  /* Only start the task if the proximity subsystem is enabled in conf.h */
  xTaskCreate(proximityTask, (const signed char * const)PROXIMITY_TASK_NAME,
              PROXIMITY_TASK_STACKSIZE, NULL, PROXIMITY_TASK_PRI, NULL);
#endif

  isInit = true;
}
