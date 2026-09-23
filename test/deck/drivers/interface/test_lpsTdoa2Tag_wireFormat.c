/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2026 Bitcraze AB
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
 * test_lpsTdoa2Tag_wireFormat.c - Wire-format contract tests for issue #1714:
 * the TDoA2 anchor (lps-node-firmware) always transmits a fixed 8-slot
 * rangePacket_t, but rangePacket2_t on the tag side (this repo) is sized by
 * the locally configurable CONFIG_DECK_LOCO_NR_OF_ANCHORS. When that value
 * is set below 8, the tag misinterprets the anchor's raw bytes: timestamps
 * and distances decode to garbage, and ordinary ranging data can be
 * misidentified as an LPP anchor-position packet.
 *
 * @NO_MODULE is used on the lpsTdoa2Tag.h include below so this file only
 * needs the type/macro definitions, not lpsTdoa2Tag.c and its dw1000/
 * FreeRTOS/estimator dependencies.
 */

// @IGNORE_IF_NOT CONFIG_DECK_LOCO

#include <string.h>
#include <stdint.h>
#include <stddef.h>

#include "unity.h"

// File under test: the real, currently-configured rangePacket2_t and the
// LPS_TDOA2_LPP_* offsets derived from it.
#include "lpsTdoa2Tag.h" // @NO_MODULE

// ---- Anchor side: lps-node-firmware/src/uwb_tdoa_anchor2.c ----
// Not part of this repo, so mirrored here. NSLOTS is hardcoded to 8 in the
// anchor firmware and does not depend on any tag-side configuration.
#define ANCHOR_NSLOTS 8
#define TS_TX_SIZE 4

typedef struct {
  uint8_t type;
  uint8_t pid[ANCHOR_NSLOTS];
  uint8_t timestamps[ANCHOR_NSLOTS][TS_TX_SIZE];
  uint16_t distances[ANCHOR_NSLOTS];
} __attribute__((packed)) anchorRangePacket_t;

// A second, explicitly 4-anchor version of the tag's struct, independent of
// CONFIG_DECK_LOCO_NR_OF_ANCHORS, used to demonstrate the bug deterministically
// regardless of how this build happens to be configured.
#define BUGGY_TAG_NR_OF_ANCHORS 4

typedef struct {
  uint8_t type;
  uint8_t sequenceNrs[BUGGY_TAG_NR_OF_ANCHORS];
  uint32_t timestamps[BUGGY_TAG_NR_OF_ANCHORS];
  uint16_t distances[BUGGY_TAG_NR_OF_ANCHORS];
} __attribute__((packed)) buggyTagRangePacket_t;

static void populateAnchorPacket(anchorRangePacket_t* packet) {
  // Mirrors populateTxData()/dwSetData() in uwb_tdoa_anchor2.c: fill every
  // field with a distinguishable, known value.
  packet->type = 0x22;
  for (int i = 0; i < ANCHOR_NSLOTS; i++) {
    packet->pid[i] = 0x10 + i;
    uint32_t ts = 0x11110000u + i;
    memcpy(packet->timestamps[i], &ts, TS_TX_SIZE);
    packet->distances[i] = 0x2000 + i;
  }
}

// Regardless of CONFIG_DECK_LOCO_NR_OF_ANCHORS, the tag's rangePacket2_t must
// describe the exact same bytes the anchor actually transmits. This test
// passes today only because this repo's default build/.config happens to set
// CONFIG_DECK_LOCO_NR_OF_ANCHORS to 8 - reconfiguring it to any value in its
// documented 4-8 range (see src/deck/drivers/src/Kconfig) reproduces issue
// #1714 and fails this test. After the fix in 04-fix/fix.md (Alternative 1)
// it should pass for every value in that range.
void testRangePacket2MatchesAnchorWireFormatAtConfiguredAnchorCount() {
  // Fixture
  anchorRangePacket_t anchorPacket;
  populateAnchorPacket(&anchorPacket);

  // Test: reinterpret the anchor's raw bytes as the tag's struct, the way
  // rxcallback() in lpsTdoa2Tag.c does.
  const rangePacket2_t* tagPacket = (const rangePacket2_t*)&anchorPacket;

  // Assert
  TEST_ASSERT_EQUAL_UINT32((uint32_t)sizeof(anchorRangePacket_t), (uint32_t)sizeof(rangePacket2_t));
  TEST_ASSERT_EQUAL_UINT32(0x11110000u, tagPacket->timestamps[0]);
  TEST_ASSERT_EQUAL_UINT32(0x11110001u, tagPacket->timestamps[1]);

  // LPS_TDOA2_LPP_HEADER must point exactly past the anchor's actual data,
  // never partway into it - otherwise ordinary bytes can be misread as an
  // LPP short packet (see testWireFormatMismatchMisparsesAnchorData below).
  TEST_ASSERT_EQUAL_UINT32((uint32_t)sizeof(anchorRangePacket_t), (uint32_t)LPS_TDOA2_LPP_HEADER);
}

// Demonstrates why a tag-side anchor count smaller than the anchor's fixed
// 8 slots corrupts the decoded data (independent of CONFIG_DECK_LOCO_NR_OF_ANCHORS,
// so it reproduces the bug deterministically no matter how this build is
// configured). See 02-reproduction/path-a-host-simulation/struct_layout_sim.c
// for the original host-side version of this simulation.
void testWireFormatMismatchMisparsesAnchorData() {
  // Fixture
  anchorRangePacket_t anchorPacket;
  populateAnchorPacket(&anchorPacket);

  // Test: reinterpret the same bytes as a tag struct sized for only 4 anchors.
  const buggyTagRangePacket_t* tagPacket = (const buggyTagRangePacket_t*)&anchorPacket;

  // Assert: struct sizes and field offsets no longer agree...
  TEST_ASSERT_NOT_EQUAL_UINT32((uint32_t)sizeof(anchorRangePacket_t), (uint32_t)sizeof(buggyTagRangePacket_t));

  // ...so the anchor's slot-0 timestamp (0x11110000, see populateAnchorPacket())
  // is decoded from the wrong bytes...
  TEST_ASSERT_EQUAL_UINT32(0x17161514u, tagPacket->timestamps[0]);

  // ...and LPS_TDOA2_LPP_HEADER (== sizeof(the tag's struct)) now points into
  // the anchor's ordinary timestamp data instead of past the end of the real
  // packet.
  const uint8_t* rawBytes = (const uint8_t*)&anchorPacket;
  size_t buggyLppHeaderOffset = sizeof(buggyTagRangePacket_t);
  uint8_t lppPacketHeaderByte = rawBytes[buggyLppHeaderOffset];
  TEST_ASSERT_NOT_EQUAL_UINT32(LPP_HEADER_SHORT_PACKET, lppPacketHeaderByte);

  // A raw hardware timestamp/distance byte that happens to equal 0xF0 at that
  // offset - a 1-in-256 event per packet, over the radio, in the field -
  // makes the tag misidentify the rest of the ordinary packet as an LPP short
  // packet. Craft that byte pattern deterministically to demonstrate it.
  anchorRangePacket_t craftedPacket;
  memcpy(&craftedPacket, &anchorPacket, sizeof(craftedPacket));
  uint8_t* craftedBytes = (uint8_t*)&craftedPacket;
  craftedBytes[buggyLppHeaderOffset] = LPP_HEADER_SHORT_PACKET; // 0xF0
  craftedBytes[buggyLppHeaderOffset + 1] = LPP_SHORT_ANCHORPOS;
  struct lppShortAnchorPos_s craftedPos = { 1.25f, -2.50f, 3.75f };
  memcpy(&craftedBytes[buggyLppHeaderOffset + 2], &craftedPos, sizeof(craftedPos));

  TEST_ASSERT_EQUAL_HEX8(LPP_HEADER_SHORT_PACKET, craftedBytes[buggyLppHeaderOffset]);
  TEST_ASSERT_EQUAL_HEX8(LPP_SHORT_ANCHORPOS, craftedBytes[buggyLppHeaderOffset + 1]);

  struct lppShortAnchorPos_s decodedPos;
  memcpy(&decodedPos, &craftedBytes[buggyLppHeaderOffset + 2], sizeof(decodedPos));
  TEST_ASSERT_EQUAL_FLOAT(1.25f, decodedPos.x);
  TEST_ASSERT_EQUAL_FLOAT(-2.50f, decodedPos.y);
  TEST_ASSERT_EQUAL_FLOAT(3.75f, decodedPos.z);
}
