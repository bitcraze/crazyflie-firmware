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
 * DTR_types.h
 *
 *  Created on: 14.02.2021
 *      Author: Christos Zosimidis
 *
 *  Modified for the P2P protocol by: Bitcraze AB
 * 
 */

#ifndef DTR_TYPES_H
#define DTR_TYPES_H

#include "FreeRTOS.h"
#include "stdint.h"
#include "stdbool.h"
#include "radiolink.h"

#define MAX_NETWORK_SIZE 20
#define DTR_PACKET_HEADER_SIZE 5

// max usable size for a packet is -1 byte for the port
#define P2P_MAX_USABLE_DATA_SIZE P2P_MAX_DATA_SIZE -1

#define MAXIMUM_DTR_PACKET_DATA_SIZE P2P_MAX_USABLE_DATA_SIZE - DTR_PACKET_HEADER_SIZE

typedef enum tx_states_e {
	TX_TOKEN,
	TX_CTS,
	TX_RTS,
	TX_DATA_FRAME,
	TX_DATA_ACK,
} dtrTxStates;

typedef enum rx_states_e {
	RX_IDLE,
	RX_WAIT_CTS,
	RX_WAIT_RTS,
	RX_WAIT_DATA_ACK,
} dtrRxStates;

enum dtrMessageTypes {
	DATA_FRAME = 0,
	TOKEN_FRAME = 1,
	CTS_FRAME = 2,
	RTS_FRAME = 3,
	DATA_ACK_FRAME = 4,
};

// |--------------------|
// |   PACKET FORMAT    |
// |--------------------|
// | 	packetSize   	|	(1 B)
// |--------------------|
// |   messageType 		|	(1 B)
// |--------------------|
// | 	sourceId     	|	(1 B)
// |--------------------|
// | 	dataSize 		|	(1 B)
// |--------------------|
// | 	  data 			|	(dataSize B)
// |--------------------|
typedef struct {
	uint8_t packetSize;
	uint8_t messageType;
	uint8_t sourceId;
	uint8_t targetId;
	uint8_t dataSize;
	uint8_t data[MAXIMUM_DTR_PACKET_DATA_SIZE];
} dtrPacket;

typedef struct {
	uint8_t deviceId;

	uint32_t failedCRC;
	uint32_t failedRxQueueFull;
	uint32_t failedTxQueueFull;

	uint32_t timeOutRTS;
	uint32_t timeOutTOKEN;
	uint32_t timeOutDATA;

	uint32_t sendPackets;
	uint32_t receivedPackets;

} dtrRadioInfo;

typedef struct {
	uint8_t size; // number of nodes in the network
	uint8_t devices_ids[MAX_NETWORK_SIZE]; // array that contains the device id of each node in the network
} dtrTopology;

#endif //DTR_TYPES_H
