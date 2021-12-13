#pragma once

#include <stdint.h>

// Targets where data can be routed
#define STM32  ((uint8_t) 0x00)
#define ESP32  ((uint8_t) 0x10)
#define GAP8   ((uint8_t) 0x20)

// Functionality where data can be
// sent for each target.

// System includes PM etc.
#define SYSTEM      ((uint8_t) 0x00)
// Console messages
#define CONSOLE     ((uint8_t) 0x01)
// CrazyRealTimeProtocol passthough
#define CRTP        ((uint8_t) 0x02)
// WiFi control data (i.e connect and stuff)
#define WIFI_CTRL   ((uint8_t) 0x05)
// WiFi data. Once connection is established this acts as passthough
#define WIFI_DATA   ((uint8_t) 0x06)
// Application running on the target§
#define APP         ((uint8_t) 0x07)
// Test running on the target (SINK/SOURCE/ECHO)
#define TEST        ((uint8_t) 0x0E)
// Bootloader running on target
#define BOOTLOADER  ((uint8_t) 0x0F)

#define MAKE_ROUTE(TARGET, FUNCTION)  (TARGET | FUNCTION)
#define ROUTE_TARGET(ROUTE)           (ROUTE & 0xF0)
#define ROUTE_FUNCTION(ROUTE)         (ROUTE & 0x0F)


// Header used for routing, place after length (which is not part
// of routing)
typedef struct {
    uint8_t dst;
    uint8_t src;
} __attribute__((packed)) routable_packet_header_t;
