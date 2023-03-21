// File under test cpx.h
#include "cpx.h"

#include "unity.h"

void testCPXRoutingPackedFormat() {
  // Fixture
  uint16_t expected = 0b0100111110011110; // 2bit version, 6bit function, 1bit reserved, 1bit lastPacket, 3bits source, 3bits destination
 
  // Test
  CPXRoutingPacked_t *actual = (CPXRoutingPacked_t*)&expected;
  
 
  // Assert
  TEST_ASSERT_EQUAL_UINT8(0b110, actual->destination);
  TEST_ASSERT_EQUAL_UINT8(0b011, actual->source);
  TEST_ASSERT_EQUAL_UINT8(0b0, actual->lastPacket);
  TEST_ASSERT_EQUAL_UINT8(0b1, actual->reserved);
  // TEST_ASSERT_EQUAL_UINT8(0b01001111, actual->function);
  TEST_ASSERT_EQUAL_UINT8(0b001111, actual->function);
  TEST_ASSERT_EQUAL_UINT8(0b01, actual->version);
}