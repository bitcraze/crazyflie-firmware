/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2022 Bitcraze AB
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
 * DTR_p2p_interface.c
 *  
 */



#include "DTR_p2p_interface.h"

#define DEBUG_MODULE "DTR_P2P"
#include "debug.h"

static P2PPacket p2p_TXpacket;
static dtrPacket prev_received = {0};

void dtrSendP2Ppacket(const dtrPacket* packet) {
    p2p_TXpacket.port = DTR_P2P_PORT;

    memcpy(&p2p_TXpacket.data[0], packet, packet->packetSize);
    p2p_TXpacket.size = packet->packetSize;
    
    radiolinkSendP2PPacketBroadcast(&p2p_TXpacket);
}

static void dtrFeedPacketToProtocol(dtrPacket *incoming_DTR) {

     bool same_packet_received =  incoming_DTR->messageType == prev_received.messageType && 
                        incoming_DTR->targetId == prev_received.targetId &&
                        incoming_DTR->sourceId == prev_received.sourceId;

    // if there are packets in the queue and the new packet is the same as the previous one, ignore it
    DTR_DEBUG_PRINT("Packets in RX_SRV queue: %d\n", dtrGetNumberOfPacketsInQueue(RX_SRV_Q) );
    if ( dtrGetNumberOfPacketsInQueue(RX_SRV_Q) !=0 && same_packet_received ) {
        DTR_DEBUG_PRINT("Duplicate packet received\n");
        DTR_DEBUG_PRINT("Message type: %d\n", incoming_DTR->messageType);
        DTR_DEBUG_PRINT("Target id: %d\n", incoming_DTR->targetId);

        return;
    }

    prev_received.messageType = incoming_DTR->messageType;
    prev_received.targetId = incoming_DTR->targetId;
    prev_received.sourceId = incoming_DTR->sourceId;

    dtrInsertPacketToQueue(incoming_DTR, RX_SRV_Q);
}

bool dtrP2PIncomingHandler(P2PPacket *p){
    if (p->port != DTR_P2P_PORT){
        return false;
    }

    dtrPacket incoming_DTR;	

    uint8_t DTRpacket_size = p->data[0];

	memcpy(&incoming_DTR, &(p->data[0]), DTRpacket_size);
    dtrFeedPacketToProtocol(&incoming_DTR);

    return true;
}
