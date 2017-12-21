#include "unity.h"
#include <math.h>
#include "lighthouse_pulse_processor.h"

void testFullFrameReturnsTrue() {
  // Fixture
  LhPulseType fullFrame[12] = {
      {479637,  9693}, {514749,  7965},  {815882,  368},
      {1180949, 7065}, {1216055, 8831},  {1546318, 458},
      {1882115, 6185}, {1917209, 11456}, {2152833, 384},
      {2583431, 8844}, {2618527, 5334},  {2980187, 384},
  };
  LhObj lh;

  // Test
  TEST_ASSERT_FALSE(lhppAnalysePulse(&lh, &fullFrame[0]))
  TEST_ASSERT_FALSE(lhppAnalysePulse(&lh, &fullFrame[1]))
  TEST_ASSERT_FALSE(lhppAnalysePulse(&lh, &fullFrame[2]))
  TEST_ASSERT_FALSE(lhppAnalysePulse(&lh, &fullFrame[3]))
  TEST_ASSERT_FALSE(lhppAnalysePulse(&lh, &fullFrame[4]))
  TEST_ASSERT_FALSE(lhppAnalysePulse(&lh, &fullFrame[5]))
  TEST_ASSERT_FALSE(lhppAnalysePulse(&lh, &fullFrame[6]))
  TEST_ASSERT_FALSE(lhppAnalysePulse(&lh, &fullFrame[7]))
  TEST_ASSERT_FALSE(lhppAnalysePulse(&lh, &fullFrame[8]))
  TEST_ASSERT_FALSE(lhppAnalysePulse(&lh, &fullFrame[9]))
  TEST_ASSERT_FALSE(lhppAnalysePulse(&lh, &fullFrame[10]))

  // Assert
  TEST_ASSERT_TRUE(lhppAnalysePulse(&lh, &fullFrame[11]))
}

void testMissingOneX0SyncReturnsFalse() {
  // Fixture
  LhPulseType fullFrame[11] = {
      /*missing*/      {514749,  7965},  {815882,  368},
      {1180949, 7065}, {1216055, 8831},  {1546318, 458},
      {1882115, 6185}, {1917209, 11456}, {2152833, 384},
      {2583431, 8844}, {2618527, 5334},  {2980187, 384},
  };
  LhObj lh;

  // Test & Assert
  TEST_ASSERT_FALSE(lhppAnalysePulse(&lh, &fullFrame[0]))
  TEST_ASSERT_FALSE(lhppAnalysePulse(&lh, &fullFrame[1]))
  TEST_ASSERT_FALSE(lhppAnalysePulse(&lh, &fullFrame[2]))
  TEST_ASSERT_FALSE(lhppAnalysePulse(&lh, &fullFrame[3]))
  TEST_ASSERT_FALSE(lhppAnalysePulse(&lh, &fullFrame[4]))
  TEST_ASSERT_FALSE(lhppAnalysePulse(&lh, &fullFrame[5]))
  TEST_ASSERT_FALSE(lhppAnalysePulse(&lh, &fullFrame[6]))
  TEST_ASSERT_FALSE(lhppAnalysePulse(&lh, &fullFrame[7]))
  TEST_ASSERT_FALSE(lhppAnalysePulse(&lh, &fullFrame[8]))
  TEST_ASSERT_FALSE(lhppAnalysePulse(&lh, &fullFrame[9]))
  TEST_ASSERT_FALSE(lhppAnalysePulse(&lh, &fullFrame[10]))

 }

void testMissingOneY0SyncReturnsFalse() {
  // Fixture
  LhPulseType fullFrame[11] = {
      {479637,  9693}, {514749,  7965},  {815882,  368},
       /*missing*/     {1216055, 8831},  {1546318, 458},
      {1882115, 6185}, {1917209, 11456}, {2152833, 384},
      {2583431, 8844}, {2618527, 5334},  {2980187, 384},
  };
  LhObj lh;

  // Test & Assert
  TEST_ASSERT_FALSE(lhppAnalysePulse(&lh, &fullFrame[0]))
  TEST_ASSERT_FALSE(lhppAnalysePulse(&lh, &fullFrame[1]))
  TEST_ASSERT_FALSE(lhppAnalysePulse(&lh, &fullFrame[2]))
  TEST_ASSERT_FALSE(lhppAnalysePulse(&lh, &fullFrame[3]))
  TEST_ASSERT_FALSE(lhppAnalysePulse(&lh, &fullFrame[4]))
  TEST_ASSERT_FALSE(lhppAnalysePulse(&lh, &fullFrame[5]))
  TEST_ASSERT_FALSE(lhppAnalysePulse(&lh, &fullFrame[6]))
  TEST_ASSERT_FALSE(lhppAnalysePulse(&lh, &fullFrame[7]))
  TEST_ASSERT_FALSE(lhppAnalysePulse(&lh, &fullFrame[8]))
  TEST_ASSERT_FALSE(lhppAnalysePulse(&lh, &fullFrame[9]))
  TEST_ASSERT_FALSE(lhppAnalysePulse(&lh, &fullFrame[10]))

 }

void testMissingOneX1SyncReturnsFalse() {
  // Fixture
  LhPulseType fullFrame[11] = {
      {479637,  9693}, {514749,  7965},  {815882,  368},
      {1180949, 7065}, {1216055, 8831},  {1546318, 458},
       /*missing*/     {1917209, 11456}, {2152833, 384},
      {2583431, 8844}, {2618527, 5334},  {2980187, 384},
  };
  LhObj lh;

  // Test & Assert
  TEST_ASSERT_FALSE(lhppAnalysePulse(&lh, &fullFrame[0]))
  TEST_ASSERT_FALSE(lhppAnalysePulse(&lh, &fullFrame[1]))
  TEST_ASSERT_FALSE(lhppAnalysePulse(&lh, &fullFrame[2]))
  TEST_ASSERT_FALSE(lhppAnalysePulse(&lh, &fullFrame[3]))
  TEST_ASSERT_FALSE(lhppAnalysePulse(&lh, &fullFrame[4]))
  TEST_ASSERT_FALSE(lhppAnalysePulse(&lh, &fullFrame[5]))
  TEST_ASSERT_FALSE(lhppAnalysePulse(&lh, &fullFrame[6]))
  TEST_ASSERT_FALSE(lhppAnalysePulse(&lh, &fullFrame[7]))
  TEST_ASSERT_FALSE(lhppAnalysePulse(&lh, &fullFrame[8]))
  TEST_ASSERT_FALSE(lhppAnalysePulse(&lh, &fullFrame[9]))
  TEST_ASSERT_FALSE(lhppAnalysePulse(&lh, &fullFrame[10]))

 }

void testMissingOneY1SyncReturnsFalse() {
  // Fixture
  LhPulseType fullFrame[11] = {
      {479637,  9693}, {514749,  7965},  {815882,  368},
      {1180949, 7065}, {1216055, 8831},  {1546318, 458},
      {1882115, 6185}, {1917209, 11456}, {2152833, 384},
      /*missing*/      {2618527, 5334},  {2980187, 384},
  };
  LhObj lh;

  // Test & Assert
  TEST_ASSERT_FALSE(lhppAnalysePulse(&lh, &fullFrame[0]))
  TEST_ASSERT_FALSE(lhppAnalysePulse(&lh, &fullFrame[1]))
  TEST_ASSERT_FALSE(lhppAnalysePulse(&lh, &fullFrame[2]))
  TEST_ASSERT_FALSE(lhppAnalysePulse(&lh, &fullFrame[3]))
  TEST_ASSERT_FALSE(lhppAnalysePulse(&lh, &fullFrame[4]))
  TEST_ASSERT_FALSE(lhppAnalysePulse(&lh, &fullFrame[5]))
  TEST_ASSERT_FALSE(lhppAnalysePulse(&lh, &fullFrame[6]))
  TEST_ASSERT_FALSE(lhppAnalysePulse(&lh, &fullFrame[7]))
  TEST_ASSERT_FALSE(lhppAnalysePulse(&lh, &fullFrame[8]))
  TEST_ASSERT_FALSE(lhppAnalysePulse(&lh, &fullFrame[9]))
  TEST_ASSERT_FALSE(lhppAnalysePulse(&lh, &fullFrame[10]))

 }
