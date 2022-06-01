/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
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
 * vcp_esc_passthrough.h - Module to handle 4way passthough interface
 */
#pragma once

#include <stdint.h>

/**
 * Init passthrough module
 */
void passthroughInit();

/**
 * Unlock passthough task and launch the passthough process
 */
void passthroughEnableFromISR();

/**
 * Queue serial data incoming from VCP interrupt routine
 */
void passthroughVcpRxSendFromISR(uint8_t Ch);

/**
 * Queue serial data incoming from VCP, blocking.
 */
void passthroughVcpRxSendBlock(uint8_t Ch);

/**
 * Read serial data that has been queued from VCP, none-blocking
 */
int passthroughVcpRxReceive(uint8_t* receiveChPtr);

/**
 * Read serial data that has been queued from VCP, blocking
 */
int passthroughVcpRxReceiveBlock(uint8_t* receiveChPtr);

/**
 * Queue outgoing data that should be sent, none-blocking
 */
void passthroughVcpTxSend(uint8_t Ch);

/**
 * Consume outgoing serial VCP data from the send queue.
 */
int  passthroughVcpTxReceiveFromISR(uint8_t* receiveChPtr);
