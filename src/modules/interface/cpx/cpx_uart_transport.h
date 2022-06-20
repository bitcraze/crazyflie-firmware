
#pragma once

#include "cpx.h"

#define CPX_UART_TRANSPORT_MTU 100

void cpxUARTTransportSend(const CPXRoutablePacket_t* packet);

void cpxUARTTransportReceive(CPXRoutablePacket_t* packet);

// De init for ESP32 bootloader, then we should not send or receive anymore
// and remove the ISR