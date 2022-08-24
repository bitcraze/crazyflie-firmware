/*
 * MIT License
 *
 * Copyright (c) 2022 Christos Zosimidis
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 *
 * DTR_handlers.h
 *
 *  Created on: 14.02.2021
 *      Author: Christos Zosimidis
 *  
 * Modified for the P2P protocol by: Bitcraze AB
 *
 */
#ifndef SRC_RADIO_DTR_HANDLERS_H_
#define SRC_RADIO_DTR_HANDLERS_H_

#include "FreeRTOS.h"
#include "task.h"
#include <stdlib.h>

#include "DTR_types.h"
#include "token_ring.h"
#include "timers.h"

#define MAX_WAIT_TIME_FOR_RTS      10 // ms
#define MAX_WAIT_TIME_FOR_CTS      11 // ms
#define MAX_WAIT_TIME_FOR_DATA_ACK 12 // ms

#define DTR_PROTOCOL_PERIOD 5 // ms (random value)


// DTR PROTOCOL TASK
#define DTR_PROTOCOL_TASK_STACK_SIZE (2 * configMINIMAL_STACK_SIZE)
#define DTR_PROTOCOL_TASK_PRIORITY 1 //Higher number higher priority

// DTR protocol Task
void dtrStartProtocolTask(void);

void dtrStopProtocolTask(void);

// ================ DTR sender timer ==================
void dtrInitSenderTimer(void);

void dtrShutdownSenderTimer(void);

void dtrStartSenderTimer(unsigned int time_out);

#endif /* SRC_RADIO_DTR_HANDLERS_H_ */
