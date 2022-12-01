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
#include <stdint.h>
#include <stdbool.h>

#include "cpx.h"

/**
 * @brief Initialize the internal router
 *
 * Initialize the internal router, used for routing CPX packets
 * on function inside the STM32.
 *
 */
void cpxInternalRouterInit(void);

/**
 * @brief Send a CPX packet, blocking
 *
 * This will send a packet to the ESP32 to be routed using CPX. This
 * will block until the packet can be queued up for sending.
 *
 * @param packet packet to be sent
 */
void cpxSendPacketBlocking(const CPXPacket_t * packet);

/**
 * @brief Send a CPX packet, blocking with timeout
 *
 * @param packet packet to be sent
 * @param timeout Timeout in ticks
 * @return True if sent, false if timeout
 */
bool cpxSendPacketBlockingTimeout(const CPXPacket_t * packet, const uint32_t timeout);

/**
 * @brief Receive a CPX packet sent with the CRTP function
 *
 * @param packet CPX packet where received packet is stored
 * @return int 0 if no packet was received otherwise non zero.
 */
int cpxInternalRouterReceiveCRTP(CPXPacket_t * packet);

/**
 * @brief Receive a CPX packet sent with another function than CRTP
 *
 * @param packet CPX packet where received packet is stored
 * @return int 0 if no packet was received otherwise non zero.
 */
void cpxInternalRouterReceiveOthers(CPXPacket_t * packet);

/**
 * @brief Send a CPX packet from the external router into the internal
 * router
 *
 * @param packet CPX packet to send
 */
void cpxInternalRouterRouteIn(const CPXRoutablePacket_t* packet);

/**
 * @brief Retrieve a CPX packet from the internal router to be
 * routed externally.
 *
 * @param packet CPX packet where retrieved packet is stored
 */
void cpxInternalRouterRouteOut(CPXRoutablePacket_t* packet);
