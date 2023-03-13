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
 * DTR_handlers.c
 *
 *  Created on: 14.02.2021
 *      Author: Christos Zosimidis
 *
 *  Modified for the P2P protocol by: Bitcraze AB
 * 
 */


#include "DTR_handlers.h"

#define DEBUG_MODULE "DTR_HANDL"
#include "debug.h"

static xTimerHandle sender_timer;
static TaskHandle_t DTRtaskHandler = NULL;

static bool sender_timer_running = false;

static char type_to_spam[15];

void dtrStartProtocolTask(void){
	xTaskCreate(dtrTaskHandler, "DTR_P2P", DTR_PROTOCOL_TASK_STACK_SIZE, NULL,DTR_PROTOCOL_TASK_PRIORITY, &DTRtaskHandler);
}

void dtrStopProtocolTask(void){
	vTaskDelete(DTRtaskHandler);
}

void dtrInitSenderTimer(void) {
	sender_timer = xTimerCreate("DTRSenderTimer", M2T(20), pdTRUE, NULL, dtrTimeOutCallBack);
}

void dtrShutdownSenderTimer(void) {
	if (xTimerIsTimerActive(sender_timer)==pdTRUE) {
		xTimerStop(sender_timer, 0);
		DTR_DEBUG_PRINT("Stopped spamming messages\n");
		sender_timer_running = false;
	}else{
		DTR_DEBUG_PRINT("Radio timer not running\n");
	}
}


void dtrStartSenderTimer(unsigned int time_out) {

	if(time_out == MAX_WAIT_TIME_FOR_RTS ){
		strcpy(type_to_spam, "RTS");
	}else if (time_out == MAX_WAIT_TIME_FOR_CTS ){
		strcpy(type_to_spam, "CTS");
	}else if (time_out == MAX_WAIT_TIME_FOR_DATA_ACK ){
		strcpy(type_to_spam, "DATA");
	}

	if (sender_timer_running){
		DTR_DEBUG_PRINT("Radio timer already running\n");
	}else{
		#ifdef DEBUG_DTR_PROTOCOL
		DTR_DEBUG_PRINT("Started spamming %s\n", type_to_spam);
		#endif

		xTimerStart(sender_timer, 20);
		// xTimerChangePeriod(sender_timer, M2T(time_out), 0);

		sender_timer_running = true;
	}
}
