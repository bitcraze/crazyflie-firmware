/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2021 Bitcraze AB
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * aideck.h: AI-deck interface
 */

#pragma once
#include <stdint.h>
#include <stdbool.h>

#include "routing_info.h"

#define ESP_BITSTREAM_SIZE 610576
#define AIDECK_UART_TRANSPORT_MTU 100

typedef struct {
    uint8_t start; // Should be 0xFF
    uint8_t length; // Length of data
    uint8_t data[AIDECK_UART_TRANSPORT_MTU];
} __attribute__((packed)) uart_transport_packet_t;

typedef struct {
    uint8_t start; // Should be 0xFF
    uint8_t length; // Length of data - 2
    routable_packet_header_t route;
    uint8_t data[AIDECK_UART_TRANSPORT_MTU - 2];
} __attribute__((packed)) aideckRoutablePacket_t;

// These are for your own stuff, where your own protocol can be implemented
void aideckSendBlocking(aideckRoutablePacket_t *packet);
void aideckReceiveBlocking(aideckRoutablePacket_t *packet);

bool aideckSend(aideckRoutablePacket_t *packet, unsigned int timeoutInMS);
bool aideckReceive(aideckRoutablePacket_t *packet, unsigned int timeoutInMS);
