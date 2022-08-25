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
 * queueing.h
 *
 *  Created on: 14.02.2021
 *      Author: Christos Zosimidis
 *
 *  Modified for the P2P protocol by: Bitcraze AB
 * 
 */

#ifndef _QUEUEING_H_
#define _QUEUEING_H_

#include "FreeRTOS.h"
#include "queue.h"
#include "DTR_types.h"
#include "queuemonitor.h"
#include "static_mem.h"

#include "token_ring.h"

#define TX_DATA_QUEUE_SIZE 10
#define RX_SRV_QUEUE_SIZE 20
#define RX_DATA_QUEUE_SIZE 10

#define TX_RECEIVED_WAIT_TIME 5// ms
#define RX_RECEIVED_WAIT_TIME 5// ms

typedef enum queue_names_e {
    TX_DATA_Q,
	RX_SRV_Q,
	RX_DATA_Q,
} DTRQueue_Names;

void dtrQueueingInit();

bool dtrIsPacketInQueueAvailable(DTRQueue_Names qName);

uint8_t dtrGetNumberOfPacketsInQueue(DTRQueue_Names qName);

bool dtrGetPacketFromQueue(dtrPacket *packet, DTRQueue_Names qName, uint32_t timeout);

// Blocks to wait for a packet to be received for a given time
// new_packet_received --> True if a new packet has been received and False if the timeout has been reached
bool dtrReceivePacketWaitUntil(dtrPacket *packet, DTRQueue_Names qName, uint32_t timeout_ms, bool *new_packet_received);

bool dtrInsertPacketToQueue(dtrPacket *packet, DTRQueue_Names qName);

bool dtrReleasePacketFromQueue(DTRQueue_Names qName);

void dtrEmptyQueue(DTRQueue_Names qName);

void dtrEmptyQueues(void);

#endif /* _QUEUEING_H_ */
