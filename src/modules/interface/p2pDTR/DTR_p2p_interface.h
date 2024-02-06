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
 * DTR_p2p_interface.h
 * 
 */

#pragma once

#include "FreeRTOS.h"
#include "DTR_types.h"
#include "radiolink.h"
#include "token_ring.h"

#define INCOMING_DTR_QUEUE_SIZE 10

// Broadcasts a DTR packet through the P2P network
void dtrSendP2Ppacket(const dtrPacket* packet) ;

// Puts the DTR packet in the queue for the token ring to pick up.
bool dtrP2PIncomingHandler(P2PPacket *p);

