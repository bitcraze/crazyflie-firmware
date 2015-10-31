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
 *
 * queuemonitor.c - Monitoring functionality for queues
 */
#define DEBUG_MODULE "QM"

#include "queuemonitor.h"

#ifdef DEBUG_QUEUE_MONITOR

#include <stdbool.h>
#include "timers.h"
#include "debug.h"
#include "cfassert.h"

#define MAX_NR_OF_QUEUES 20
#define TIMER_PERIOD M2T(10000)

typedef struct
{
  char* fileName;
  char* queueName;
  int sendCount;
  int maxWaiting;
  int fullCount;
} Data;

static Data data[MAX_NR_OF_QUEUES];

static xTimerHandle timer;
static unsigned char nrOfQueues = 1; // Unregistered queues will end up at 0
static bool initialized = false;

static void timerHandler(xTimerHandle timer);
static void debugPrintData();
static Data* getQueueData(xQueueHandle* xQueue);
static int getMaxWaiting(xQueueHandle* xQueue, int prevPeak);

unsigned char ucQueueGetQueueNumber( xQueueHandle xQueue );


void queueMonitorInit() {
  ASSERT(!initialized);
  timer = xTimerCreate( (const signed char *)"queueMonitorTimer", TIMER_PERIOD,
    pdTRUE, NULL, timerHandler );
  xTimerStart(timer, 100);

  data[0].fileName = "Na";
  data[0].queueName = "Na";

  initialized = true;
}

void qm_traceQUEUE_SEND(void* xQueue) {
  if(initialized) {
    Data* queueData = getQueueData(xQueue);

    queueData->sendCount++;
    queueData->maxWaiting = getMaxWaiting(xQueue, queueData->maxWaiting);
  }
}

void qm_traceQUEUE_SEND_FAILED(void* xQueue) {
  if(initialized) {
    Data* queueData = getQueueData(xQueue);

    queueData->fullCount++;
  }
}

void qmRegisterQueue(xQueueHandle* xQueue, char* fileName, char* queueName) {
  ASSERT(initialized);
  ASSERT(nrOfQueues < MAX_NR_OF_QUEUES);
  Data* queueData = &data[nrOfQueues];

  queueData->fileName = fileName;
  queueData->queueName = queueName;
  vQueueSetQueueNumber(xQueue, nrOfQueues);

  nrOfQueues++;
}

static Data* getQueueData(xQueueHandle* xQueue) {
  unsigned char number = ucQueueGetQueueNumber(xQueue);
  ASSERT(number < MAX_NR_OF_QUEUES);
  return &data[number];
}

static int getMaxWaiting(xQueueHandle* xQueue, int prevPeak) {
  unsigned portBASE_TYPE waiting = uxQueueMessagesWaitingFromISR(xQueue);

  if (waiting > prevPeak) {
    return waiting;
  }
  return prevPeak;
}

static void debugPrintData() {
  DEBUG_PRINT("Report\n");

  int i;
  for (i = 0; i < nrOfQueues; i++) {
    Data* curr = &data[i];

    // The nr of items waiting in a queue is measured BEFORE adding next item.
    // Must add 1 to get peak value.
    int peak = curr->maxWaiting + 1;

    DEBUG_PRINT("%s:%s, sent: %i, peak: %i, full: %i\n",
      curr->fileName, curr->queueName, curr->sendCount, peak,
      curr->fullCount);
  }
}

static void timerHandler(xTimerHandle timer) {
  debugPrintData();
}

#endif // DEBUG_QUEUE_MONITOR