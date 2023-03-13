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
 */
#pragma once

#include "cpx.h"

#define CPX_UART_TRANSPORT_MTU 100

/**
 * @brief Initialize the UART transport
 * 
 * This will also initialize the UART2 peripheral.
 * 
 */
void cpxUARTTransportInit(void);

/**
 * @brief De-initialize the UART transport
 * 
 * This is used to gracefully shut down the UART transport so
 * it will stop using the UART. Note, this function will block
 * until the RX/TX tasks has shut down.
 * 
 */
void cpxUARTTransportDeinit();

/**
 * @brief Send a CPX packet via the UART transport
 * 
 * This will send a CPX packet, packing it according to the
 * specification for the link.
 * 
 * @param packet CPX packet to send
 */
void cpxUARTTransportSend(const CPXRoutablePacket_t* packet);

/**
 * @brief Receive a CPX packet via the UART transport
 * 
 * This will receive a CPX packet, unpacking it according to the
 * specification for the link.
 * 
 * @param packet CPX packet where received packet will be stored
 */
void cpxUARTTransportReceive(CPXRoutablePacket_t* packet);