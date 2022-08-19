/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
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
 * peer_to_peer.c - App layer application of simple demonstration peer to peer
 *  communication. Two crazyflies need this program in order to send and receive.
 */


#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#include "radiolink.h"
#include "configblock.h"

#define DEBUG_MODULE "P2P"
#include "debug.h"

#include "token_ring.h"
#include "p2p_interface.h"

#define INTERESTING_DATA 104

static uint8_t my_id;

void loadTXPacketsForTesting(void){
	DTRpacket  testSignal;
	testSignal.message_type = DATA_FRAME;
	testSignal.source_id = my_id;
	testSignal.target_id = 1;
	
	uint8_t data_size = 25;
	for (int i = 0; i < data_size; i++){
		testSignal.data[i] = i;
	}
	testSignal.dataSize = data_size;
	testSignal.allToAllFlag = 1;
	testSignal.packetSize = DTR_PACKET_HEADER_SIZE + testSignal.dataSize;
	bool res;
	for (int i = 0; i < TX_DATA_QUEUE_SIZE - 1; i++){
		testSignal.data[0] = 100+i;
		res = insertDTRPacketToQueue(&testSignal,TX_DATA_Q);
		if (res){
			DTR_DEBUG_PRINT("TX Packet sent to TX_DATA Q\n");
		}
		else{
			DEBUG_PRINT("Packet not sent to TX_DATA Q\n");
		}
	}
}

void p2pcallbackHandler(P2PPacket *p){
	// DEBUG_PRINT("P2P callback at port: %d\n", p->port);
	// If the packet is a DTR service packet, then the handler will handle it.
    DTRp2pIncomingHandler(p);

	// Then write your own code below ...
}

void appMain(){
	my_id = get_self_id();
	EnableDTRProtocol();
	vTaskDelay(2000);

	// Register the callback function so that the CF can receive packets as well.
	p2pRegisterCB(p2pcallbackHandler);

	if (my_id == 0){
		DTR_DEBUG_PRINT("Starting communication...\n");
		startRadioCommunication();
		loadTXPacketsForTesting();
	}

	DTRpacket received_packet;
	uint32_t start = T2M(xTaskGetTickCount());
	while(1){
		getDTRPacketFromQueue(&received_packet, RX_DATA_Q, portMAX_DELAY);
		uint32_t dt = T2M(xTaskGetTickCount()) - start;
		DEBUG_PRINT("Received data from %d : %d  --> Time elapsed: %lu msec\n",received_packet.source_id, received_packet.data[0],dt);
		start = T2M(xTaskGetTickCount());

		if (my_id == 1 && received_packet.data[0] == INTERESTING_DATA){
			received_packet.source_id = my_id;
			received_packet.data[0] = 123;
			DEBUG_PRINT("Sending response...\n");
			insertDTRPacketToQueue(&received_packet,TX_DATA_Q);
		}
	}
}
