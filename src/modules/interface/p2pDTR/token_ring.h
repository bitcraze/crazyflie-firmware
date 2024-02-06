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
 * token_ring.h
 *
 *  Created on: 28.01.2021
 *      Author: Christos Zosimidis
 *
 *  Modified for the P2P protocol by: Bitcraze AB
 */

#ifndef SRC_RADIO_RADIO_H_
#define SRC_RADIO_RADIO_H_

#include "FreeRTOS.h"
#include "timers.h"

#include "stdint.h"
#include "stdbool.h"
#include "string.h"
#include "radiolink.h"
#include "log.h"
#include "configblock.h"


#include "DTR_types.h"
#include "DTR_handlers.h"
#include "DTR_p2p_interface.h"
#include "queueing.h"

// #define UNUSED(...) ((void)sizeof((_Bool[]){__VA_ARGS__})); //Used to silence compiler warnings about unused parameters
#define UNUSED(...) ; //Used to silence compiler warnings about unused parameters

// Enable debug prints for the DTR protocol (WARNING: May cause problems )
// #define DEBUG_DTR_PROTOCOL

#ifdef DEBUG_DTR_PROTOCOL
    #define DTR_DEBUG_PRINT(fmt, ... ) DEBUG_PRINT(DEBUG_FMT(fmt), ##__VA_ARGS__)
#else
    //do nothing
    #define DTR_DEBUG_PRINT( fmt,   ... ) (void) (fmt) 
#endif


#define PROTOCOL_TIMEOUT_MS 4 * 1000.0f // ms 
#define DTR_P2P_PORT 15 // between 0 and 15(4 bits)

#define START_PACKET 0xBCCF

uint8_t dtrGetDeviceAddress();

void dtrTimeOutCallBack(xTimerHandle timer);

void dtrTaskHandler(void *param);

const dtrRadioInfo* dtrGetRadioInfo();

void dtrResetRadioMetaInfo();

void dtrPrintPacket(dtrPacket* packet);

uint8_t dtrGetSelfId(void);
// =========================== DTR API ===========================

// Starts the task of the Dynamic Token Ring Protocol (DTR) and initializes the protocol
// @param topology The topology of the network (see DTR_types.h)
void dtrEnableProtocol(dtrTopology topology);

// Stops the task of the Dynamic Token Ring Protocol (DTR) and deinitializes the protocol
void DisableDTRProtocol(void);

// Sends a packet to the DTR protocol
// @param packet The packet to be sent
// @return true if the packet was sent successfully to the DTR (not the final receiver), false otherwise 
bool dtrSendPacket(dtrPacket* packet);

// Receives a packet from the DTR Protocol
// Blocks for the specified timeout if no packet is received
// @param packet: the packet to be received
// @param timeout: the timeout in ticks
// @return true if the packet was received, false otherwise
bool dtrGetPacket(dtrPacket* packet, uint32_t timeout);

#endif /* SRC_RADIO_RADIO_H_ */
