/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * AI-deck GAP8
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
#include <stdint.h>
#include <stdbool.h>

#include "cpx.h"

#define CPX_MTU (100)
#define CPX_HEADER_SIZE (2)

/**
 * @brief Initialize the internal router
 *
 */
void cpxInternalRouterInit(void);



/**
 * @brief Receive a CPX packet from the ESP32
 *
 * This function will block until a packet is availale from CPX. The
 * function will return all packets routed to the STM32.
 *
 * @param function function to receive packet on
 * @param packet received packet will be stored here
 */
//void cpxReceivePacketBlocking(CPXFunction_t function, CPXPacket_t * packet);

/**
 * @brief Enable receiving on queue
 *
 * This will allocate data for a queue to be used when receiving
 * packets for a specific function.
 *
 * @param function packet to be sent
 */
//void cpxEnableFunction( CPXFunction_t function);

/**
 * @brief Send a CPX packet to the ESP32
 *
 * This will send a packet to the ESP32 to be routed using CPX. This
 * will block until the packet can be queued up for sending.
 *
 * @param packet packet to be sent
 */
void cpxSendPacketBlocking(const CPXPacket_t * packet);

/**
 * @brief Send a CPX packet to the ESP32
 *
 * This will send a packet to the ESP32 to be routed using CPX.
 *
 * @param packet packet to be sent
 * @param timeout timeout before giving up if packet cannot be queued
 * @return true if package could be queued for sending
 * @return false if package could not be queued for sending within timeout
 */
//bool cpxSendPacket(const CPXPacket_t * packet, uint32_t timeout);

/**
 * @brief Initialize CPX routing data.
 *
 * Initialize values and set lastPacket to true.
 *
 * @param source The starting point of the packet
 * @param destination The destination to send the packet to
 * @param function The function of the content
 * @param route Pointer to the route data to initialize
 */
//void cpxInitRoute(const CPXTarget_t source, const CPXTarget_t destination, const CPXFunction_t function, CPXRouting_t* route);

/**
 * @brief Print debug data though CPX
 *
 * This will print debug data though CPX to the Crazyflie client console.
 * The function doesn't add a newline, so this will have to be supplied.
 *
 * @param target The target where the printout should go
 * @param fmt Standard C format string
 * @param variable Data for format string
 */
//void cpxPrintToConsole(CPXConsoleTarget_t target, const char * fmt, ...);

void cpxInternalRouterRouteIn(const CPXRoutablePacket_t* packet);

void cpxInternalRouterRouteOut(CPXRoutablePacket_t* packet);

int cpxInternalRouterReceiveCRTP(CPXPacket_t * packet);

void cpxInternalRouterReceiveOthers(CPXPacket_t * packet);