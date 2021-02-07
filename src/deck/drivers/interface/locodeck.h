/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2016 Bitcraze AB
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
 * locodeck.h: Dwm1000 deck driver.
 */

#ifndef __LOCODECK_H__
#define __LOCODECK_H__

#include <stddef.h>
#include <stdint.h>

#include "FreeRTOS.h"

#include "libdw1000.h"
#include "stabilizer_types.h"

// Timestamp counter frequency
#define LOCODECK_TS_FREQ (499.2e6 * 128)

#define LOCODECK_ANTENNA_OFFSET 154.6   // In meters
#define LOCODECK_ANTENNA_DELAY  ((LOCODECK_ANTENNA_OFFSET * LOCODECK_TS_FREQ) / SPEED_OF_LIGHT) // In radio ticks

typedef enum uwbEvent_e {
  eventTimeout,
  eventPacketReceived,
  eventPacketSent,
  eventReceiveTimeout,
  eventReceiveFailed,
} uwbEvent_t;

typedef uint64_t locoAddress_t;

#define LPS_NUMBER_OF_ALGORITHMS 3

#define LPS_AUTO_MODE_SWITCH_PERIOD M2T(1000)

typedef enum {
  lpsMode_auto = 0,
  lpsMode_TWR = 1,
  lpsMode_TDoA2 = 2,
  lpsMode_TDoA3 = 3,
} lpsMode_t;

typedef struct {
  // The status of anchors. A bit field (bit 0 - anchor 0, bit 1 - anchor 1 and so on)
  // where a set bit indicates that an anchor reentry has been detected
  volatile uint16_t rangingState;

  // Requested and current ranging mode
  lpsMode_t userRequestedMode;
  lpsMode_t currentRangingMode;

  // State of the ranging mode auto detection
  bool modeAutoSearchDoInitialize;
  bool modeAutoSearchActive;
  uint32_t nextSwitchTick;
} lpsAlgoOptions_t;

bool locoDeckGetAnchorPosition(const uint8_t anchorId, point_t* position);
uint8_t locoDeckGetAnchorIdList(uint8_t unorderedAnchorList[], const int maxListSize);
uint8_t locoDeckGetActiveAnchorIdList(uint8_t unorderedAnchorList[], const int maxListSize);

// Callbacks for uwb algorithms
typedef struct uwbAlgorithm_s {
  void (*init)(dwDevice_t *dev);
  uint32_t (*onEvent)(dwDevice_t *dev, uwbEvent_t event);
  bool (*isRangingOk)();
  bool (*getAnchorPosition)(const uint8_t anchorId, point_t* position);
  uint8_t (*getAnchorIdList)(uint8_t unorderedAnchorList[], const int maxListSize);
  uint8_t (*getActiveAnchorIdList)(uint8_t unorderedAnchorList[], const int maxListSize);
} uwbAlgorithm_t;

#include <FreeRTOS.h>

#define MAX_TIMEOUT portMAX_DELAY

// Send a short configuration packet to the LPS system
// Returns true if packet will be send, false instead
bool lpsSendLppShort(uint8_t destId, void* data, size_t length);

typedef struct {
  uint8_t dest;
  uint8_t length;
  uint8_t data[30];
} lpsLppShortPacket_t;

// Poll if there is a LPS short configuration packet to send
// Return true if the packet data has been filled in shortPacket
// Return false if no packet to send
// Function to be used by the LPS algorithm
bool lpsGetLppShort(lpsLppShortPacket_t* shortPacket);

uint16_t locoDeckGetRangingState();
void locoDeckSetRangingState(const uint16_t newState);

// LPP Packet types and format
#define LPP_HEADER_SHORT_PACKET 0xF0

#define LPP_SHORT_ANCHORPOS 0x01

struct lppShortAnchorPos_s {
  float x;
  float y;
  float z;
} __attribute__((packed));

#endif // __LOCODECK_H__
