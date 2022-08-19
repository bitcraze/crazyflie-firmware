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
 * p2p_interface.c
 *  
 */



#include "p2p_interface.h"

#define DEBUG_MODULE "DTR_P2P"
#include "debug.h"

static P2PPacket p2p_TXpacket;
static DTRpacket prev_received = {0};

void sendDTRpacket(const DTRpacket* packet) {
    p2p_TXpacket.port = DTR_P2P_PORT;

    memcpy(&p2p_TXpacket.data[0], packet, packet->packetSize);
    p2p_TXpacket.size = packet->packetSize;
    
    radiolinkSendP2PPacketBroadcast(&p2p_TXpacket);
}

void DTRp2pIncomingHandler(P2PPacket *p){
    if (p->port != DTR_P2P_PORT){
        return;
    }

    DTRpacket incoming_DTR;	

    uint8_t DTRpacket_size = p->data[0];

	memcpy(&incoming_DTR, &(p->data[0]), DTRpacket_size);
    feedDTRPacketToProtocol(&incoming_DTR);
}


void feedDTRPacketToProtocol(DTRpacket *incoming_DTR) {

     bool same_packet_received =  incoming_DTR->message_type == prev_received.message_type && 
                        incoming_DTR->target_id == prev_received.target_id &&
                        incoming_DTR->source_id == prev_received.source_id;

    // if there are packets in the queue and the new packet is the same as the previous one, ignore it
    DTR_DEBUG_PRINT("Packets in RX_SRV queue: %d\n", getNumberOfDTRPacketsInQueue(RX_SRV_Q) );
    if ( getNumberOfDTRPacketsInQueue(RX_SRV_Q) !=0 && same_packet_received ) {
        DTR_DEBUG_PRINT("Duplicate packet received\n");
        DTR_DEBUG_PRINT("Message type: %d\n", incoming_DTR->message_type);
        DTR_DEBUG_PRINT("Target id: %d\n", incoming_DTR->target_id);

        return;
    }

    prev_received.message_type = incoming_DTR->message_type;
    prev_received.target_id = incoming_DTR->target_id;
    prev_received.source_id = incoming_DTR->source_id;

    insertDTRPacketToQueue(incoming_DTR, RX_SRV_Q);
}
