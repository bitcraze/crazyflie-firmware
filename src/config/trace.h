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
 * trace.h - ITM trace implementation/definition
 */

#ifndef __TRACE_H__
#define __TRACE_H__

#define configUSE_TRACE_FACILITY	1

// ITM useful macros
#ifndef ITM_NO_OVERFLOW
#define ITM_SEND(CH, DATA) ((uint32_t*)0xE0000000)[CH] = DATA
#else
#define ITM_SEND(CH, DATA) while(((uint32_t*)0xE0000000)[CH] == 0);\
                           ((uint32_t*)0xE0000000)[CH] = DATA
#endif

// Send 4 first characters of task name to ITM port 1
#define traceTASK_SWITCHED_IN() ITM_SEND(1, *((uint32_t*)pxCurrentTCB->pcTaskName))

// Systick value on port 2
#define traceTASK_INCREMENT_TICK(xTickCount) ITM_SEND(2, xTickCount)

// Queue trace on port 3
#define ITM_QUEUE_SEND 0x0100
#define ITM_QUEUE_FAILED 0x0200
#define ITM_BLOCKING_ON_QUEUE_RECEIVE 0x0300
#define ITM_BLOCKING_ON_QUEUE_SEND 0x0400

#define traceQUEUE_SEND(xQueue) ITM_SEND(3, ITM_QUEUE_SEND | ((xQUEUE *) xQueue)->uxQueueNumber)
#define traceQUEUE_SEND_FAILED(xQueue) ITM_SEND(3, ITM_QUEUE_FAILED | ((xQUEUE *) xQueue)->uxQueueNumber)
#define traceBLOCKING_ON_QUEUE_RECEIVE(xQueue) ITM_SEND(3, ITM_BLOCKING_ON_QUEUE_RECEIVE | ((xQUEUE *) xQueue)->uxQueueNumber)
#define traceBLOCKING_ON_QUEUE_SEND(xQueue) ITM_SEND(3, ITM_BLOCKING_ON_QUEUE_SEND | ((xQUEUE *) xQueue)->uxQueueNumber)

#endif
