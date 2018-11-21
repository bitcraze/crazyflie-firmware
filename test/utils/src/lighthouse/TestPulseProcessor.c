#include "pulseProcessor.h"

#include <stdlib.h>
#include <string.h>
#include "unity.h"

#define FRAME_LENGTH 400000    // 8.333ms
#define SWEEP_MAX_WIDTH 1024    // 20us
#define SWEEP_CENTER 192000    // 4ms
#define SYNC_BASE_WIDTH 2750
#define SYNC_DIVIDER 500
#define SYNC_SEPARATION 19200
#define MAX_FRAME_LENGTH_NOISE 40

#define SYNC_X SYNC_BASE_WIDTH+(SYNC_DIVIDER * 0)
#define SYNC_Y SYNC_BASE_WIDTH+(SYNC_DIVIDER * 1)
#define SYNC_X_SKIP SYNC_BASE_WIDTH+(SYNC_DIVIDER * 4)
#define SYNC_Y_SKIP SYNC_BASE_WIDTH+(SYNC_DIVIDER * 5)
#define SWEEP 512
#define LONG_SWEEP 1500


void setUp(void) {
}

void testThatFindSyncCanDetectSync0FromTwoBasestations()
{
  // Fixture
  uint32_t actualSyncTime = 0;
  uint32_t expectedSyncTime = FRAME_LENGTH;
  pulseProcessorPulse_t pulseHistory[PULSE_PROCESSOR_HISTORY_LENGTH] =  {
    {.width = SYNC_X,      .timestamp = expectedSyncTime + (0*FRAME_LENGTH)+0},
    {.width = SYNC_X_SKIP, .timestamp = expectedSyncTime + (0*FRAME_LENGTH)+SYNC_SEPARATION},
    {.width = SWEEP,       .timestamp = expectedSyncTime + (0*FRAME_LENGTH)+SWEEP_CENTER},
    {.width = SYNC_Y,      .timestamp = expectedSyncTime + (1*FRAME_LENGTH)+0},
    {.width = SYNC_Y_SKIP, .timestamp = expectedSyncTime + (1*FRAME_LENGTH)+SYNC_SEPARATION},
    {.width = SWEEP,       .timestamp = expectedSyncTime + (1*FRAME_LENGTH)+SWEEP_CENTER},
    {.width = SYNC_X_SKIP, .timestamp = expectedSyncTime + (2*FRAME_LENGTH)+0},
    {.width = SYNC_X,      .timestamp = expectedSyncTime + (2*FRAME_LENGTH)+SYNC_SEPARATION},
  };
  limitTimestamps(pulseHistory);

  // Test
  bool found = findSyncTime(pulseHistory, &actualSyncTime);

  // Assert
  TEST_ASSERT_TRUE(found);
  assertSyncTimeIsMultipleOfFrameLength(expectedSyncTime, actualSyncTime);
}

void testThatFindSyncCanDetectSync0FromTwoBasestationsMissaligned()
{
  // Fixture
  uint32_t actualSyncTime = 0;
  uint32_t expectedSyncTime = 2*FRAME_LENGTH;
  pulseProcessorPulse_t pulseHistory[PULSE_PROCESSOR_HISTORY_LENGTH] =  {
    {.width = SYNC_X_SKIP, .timestamp = expectedSyncTime + (-1*FRAME_LENGTH)+SYNC_SEPARATION},
    {.width = SWEEP,       .timestamp = expectedSyncTime + (-1*FRAME_LENGTH)+SWEEP_CENTER},
    {.width = SYNC_Y,      .timestamp = expectedSyncTime + ( 0*FRAME_LENGTH)+0},
    {.width = SYNC_Y_SKIP, .timestamp = expectedSyncTime + ( 0*FRAME_LENGTH)+SYNC_SEPARATION},
    {.width = SWEEP,       .timestamp = expectedSyncTime + ( 0*FRAME_LENGTH)+SWEEP_CENTER},
    {.width = SYNC_X_SKIP, .timestamp = expectedSyncTime + ( 1*FRAME_LENGTH)+0},
    {.width = SYNC_X,      .timestamp = expectedSyncTime + ( 1*FRAME_LENGTH)+SYNC_SEPARATION},
    {.width = SWEEP,       .timestamp = expectedSyncTime + ( 1*FRAME_LENGTH)+SWEEP_CENTER},
  };
  limitTimestamps(pulseHistory);

  // Test
  bool found = findSyncTime(pulseHistory, &actualSyncTime);

  // Assert
  TEST_ASSERT_TRUE(found);
  assertSyncTimeIsMultipleOfFrameLength(expectedSyncTime, actualSyncTime);
}

void testThatFindSyncCanDetectSync0FromOneBasestation()
{
  // Fixture
  uint32_t actualSyncTime = 0;
  uint32_t expectedSyncTime = FRAME_LENGTH;
  pulseProcessorPulse_t pulseHistory[PULSE_PROCESSOR_HISTORY_LENGTH] =  {
    {.width = SYNC_X,      .timestamp = expectedSyncTime + (0*FRAME_LENGTH)+0},
    {.width = SWEEP,       .timestamp = expectedSyncTime + (0*FRAME_LENGTH)+SWEEP_CENTER},
    {.width = SYNC_Y,      .timestamp = expectedSyncTime + (1*FRAME_LENGTH)+0},
    {.width = SWEEP,       .timestamp = expectedSyncTime + (1*FRAME_LENGTH)+SWEEP_CENTER},
    {.width = SYNC_X,      .timestamp = expectedSyncTime + (2*FRAME_LENGTH)+0},
    {.width = SWEEP,       .timestamp = expectedSyncTime + (2*FRAME_LENGTH)+SWEEP_CENTER},
    {.width = SYNC_Y,      .timestamp = expectedSyncTime + (3*FRAME_LENGTH)+0},
    {.width = SWEEP,       .timestamp = expectedSyncTime + (3*FRAME_LENGTH)+SWEEP_CENTER},
  };
  limitTimestamps(pulseHistory);

  // Test
  bool found = findSyncTime(pulseHistory, &actualSyncTime);

  // Assert
  TEST_ASSERT_TRUE(found);
  assertSyncTimeIsMultipleOfFrameLength(expectedSyncTime, actualSyncTime);
}

void testThatFindSyncCanDetectSync0FromTwoBasestationsWithShortSpuriousSpike()
{
  // Fixture
  uint32_t actualSyncTime = 0;
  uint32_t expectedSyncTime = FRAME_LENGTH;
  pulseProcessorPulse_t pulseHistory[PULSE_PROCESSOR_HISTORY_LENGTH] =  {
    {.width = SYNC_X,      .timestamp = expectedSyncTime + (0*FRAME_LENGTH)+0},
    {.width = SYNC_X_SKIP, .timestamp = expectedSyncTime + (0*FRAME_LENGTH)+SYNC_SEPARATION},
    {.width = SWEEP,       .timestamp = expectedSyncTime + (0*FRAME_LENGTH)+SWEEP_CENTER},
    {.width = SWEEP,       .timestamp = expectedSyncTime + (0*FRAME_LENGTH)+SWEEP_CENTER+100},
    {.width = SYNC_Y,      .timestamp = expectedSyncTime + (1*FRAME_LENGTH)+0},
    {.width = SYNC_Y_SKIP, .timestamp = expectedSyncTime + (1*FRAME_LENGTH)+SYNC_SEPARATION},
    {.width = SWEEP,       .timestamp = expectedSyncTime + (1*FRAME_LENGTH)+SWEEP_CENTER},
    {.width = SYNC_X_SKIP, .timestamp = expectedSyncTime + (2*FRAME_LENGTH)+0},
    {.width = SYNC_X,      .timestamp = expectedSyncTime + (2*FRAME_LENGTH)+SYNC_SEPARATION},
  };
  limitTimestamps(pulseHistory);

  // Test
  bool found = findSyncTime(pulseHistory, &actualSyncTime);

  // Assert
  TEST_ASSERT_TRUE(found);
  assertSyncTimeIsMultipleOfFrameLength(expectedSyncTime, actualSyncTime);
}

void testThatFindSyncFailsWhenReceivingFromTwoBasestationsWithShortSpuriousSpikeBetweenSyncs()
{
  // Fixture
  uint32_t actualSyncTime = 0;
  uint32_t expectedSyncTime = FRAME_LENGTH;
  pulseProcessorPulse_t pulseHistory[PULSE_PROCESSOR_HISTORY_LENGTH] =  {
    {.width = SYNC_X,      .timestamp = expectedSyncTime + (0*FRAME_LENGTH)+0},
    {.width = SWEEP,       .timestamp = expectedSyncTime + (0*FRAME_LENGTH)+(SYNC_SEPARATION/2)},
    {.width = SYNC_X_SKIP, .timestamp = expectedSyncTime + (0*FRAME_LENGTH)+SYNC_SEPARATION},
    {.width = SWEEP,       .timestamp = expectedSyncTime + (0*FRAME_LENGTH)+SWEEP_CENTER},
    {.width = SYNC_Y,      .timestamp = expectedSyncTime + (1*FRAME_LENGTH)+0},
    {.width = SYNC_Y_SKIP, .timestamp = expectedSyncTime + (1*FRAME_LENGTH)+SYNC_SEPARATION},
    {.width = SWEEP,       .timestamp = expectedSyncTime + (1*FRAME_LENGTH)+SWEEP_CENTER},
    {.width = SYNC_X_SKIP, .timestamp = expectedSyncTime + (2*FRAME_LENGTH)+0},
    {.width = SYNC_X,      .timestamp = expectedSyncTime + (2*FRAME_LENGTH)+SYNC_SEPARATION},
  };
  limitTimestamps(pulseHistory);

  // Test
  bool found = findSyncTime(pulseHistory, &actualSyncTime);

  // Assert
  TEST_ASSERT_FALSE(found);
}

void testThatFindSyncCanDetectSync0FromTwoBasestationsWithNoisyPulseLength()
{
  // Fixture
  uint32_t actualSyncTime = 0;
  uint32_t expectedSyncTime = FRAME_LENGTH;
  pulseProcessorPulse_t pulseHistory[PULSE_PROCESSOR_HISTORY_LENGTH] =  {
    {.width = SYNC_X + 10,      .timestamp = expectedSyncTime + (0*FRAME_LENGTH)+0},
    {.width = SYNC_X_SKIP - 10, .timestamp = expectedSyncTime + (0*FRAME_LENGTH)+SYNC_SEPARATION},
    {.width = SWEEP - 10,       .timestamp = expectedSyncTime + (0*FRAME_LENGTH)+SWEEP_CENTER},
    {.width = SYNC_Y +  7,      .timestamp = expectedSyncTime + (1*FRAME_LENGTH)+0},
    {.width = SYNC_Y_SKIP - 10, .timestamp = expectedSyncTime + (1*FRAME_LENGTH)+SYNC_SEPARATION},
    {.width = SWEEP + 10,       .timestamp = expectedSyncTime + (1*FRAME_LENGTH)+SWEEP_CENTER},
    {.width = SYNC_X_SKIP -  8, .timestamp = expectedSyncTime + (2*FRAME_LENGTH)+0},
    {.width = SYNC_X +  4,      .timestamp = expectedSyncTime + (2*FRAME_LENGTH)+SYNC_SEPARATION},
  };
  limitTimestamps(pulseHistory);

  // Test
  bool found = findSyncTime(pulseHistory, &actualSyncTime);

  // Assert
  TEST_ASSERT_TRUE(found);
  assertSyncTimeIsMultipleOfFrameLength(expectedSyncTime, actualSyncTime);
}

void testThatFindSyncFailsWhenReceivingFromTwoBasestationsWithLongSweep()
{
  // Fixture
  uint32_t actualSyncTime = 0;
  uint32_t expectedSyncTime = FRAME_LENGTH;
  pulseProcessorPulse_t pulseHistory[PULSE_PROCESSOR_HISTORY_LENGTH] =  {
    {.width = SYNC_X,      .timestamp = expectedSyncTime + (0*FRAME_LENGTH)+0},
    {.width = SYNC_X_SKIP, .timestamp = expectedSyncTime + (0*FRAME_LENGTH)+SYNC_SEPARATION},
    {.width = LONG_SWEEP,  .timestamp = expectedSyncTime + (0*FRAME_LENGTH)+SWEEP_CENTER},
    {.width = SYNC_Y,      .timestamp = expectedSyncTime + (1*FRAME_LENGTH)+0},
    {.width = SYNC_Y_SKIP, .timestamp = expectedSyncTime + (1*FRAME_LENGTH)+SYNC_SEPARATION},
    {.width = LONG_SWEEP,  .timestamp = expectedSyncTime + (1*FRAME_LENGTH)+SWEEP_CENTER},
    {.width = SYNC_X_SKIP, .timestamp = expectedSyncTime + (2*FRAME_LENGTH)+0},
    {.width = SYNC_X,      .timestamp = expectedSyncTime + (2*FRAME_LENGTH)+SYNC_SEPARATION},
  };
  limitTimestamps(pulseHistory);

  // Test
  bool found = findSyncTime(pulseHistory, &actualSyncTime);

  // Assert
  TEST_ASSERT_FALSE(found);
}

void testThatFindSyncCanDetectSync0FromTwoBasestationsWithTimingNoise()
{
  // Fixture
  uint32_t actualSyncTime = 0;
  uint32_t expectedSyncTime = FRAME_LENGTH;
  pulseProcessorPulse_t pulseHistory[PULSE_PROCESSOR_HISTORY_LENGTH] =  {
    {.width = SYNC_X,      .timestamp = expectedSyncTime + 10 + (0*FRAME_LENGTH)+0},
    {.width = SYNC_X_SKIP, .timestamp = expectedSyncTime -  3 + (0*FRAME_LENGTH)+SYNC_SEPARATION},
    {.width = SWEEP,       .timestamp = expectedSyncTime +  4 + (0*FRAME_LENGTH)+SWEEP_CENTER},
    {.width = SYNC_Y,      .timestamp = expectedSyncTime +  7 + (1*FRAME_LENGTH)+0},
    {.width = SYNC_Y_SKIP, .timestamp = expectedSyncTime -  9 + (1*FRAME_LENGTH)+SYNC_SEPARATION},
    {.width = SWEEP,       .timestamp = expectedSyncTime + 10 + (1*FRAME_LENGTH)+SWEEP_CENTER},
    {.width = SYNC_X_SKIP, .timestamp = expectedSyncTime - 10 + (2*FRAME_LENGTH)+0},
    {.width = SYNC_X,      .timestamp = expectedSyncTime +  3 + (2*FRAME_LENGTH)+SYNC_SEPARATION},
  };
  limitTimestamps(pulseHistory);

  // Test
  bool found = findSyncTime(pulseHistory, &actualSyncTime);

  // Assert
  TEST_ASSERT_TRUE(found);
  assertSyncTimeIsMultipleOfFrameLength(expectedSyncTime, actualSyncTime);
}

void testThatFindSyncCanDetectSync0FromTwoBasestationsWithTimestampWrapping()
{
  // Fixture
  uint32_t actualSyncTime = 0;
  uint32_t expectedSyncTime = (1<<TIMESTAMP_BITWIDTH) - ((1*FRAME_LENGTH)+SYNC_SEPARATION);
  pulseProcessorPulse_t pulseHistory[PULSE_PROCESSOR_HISTORY_LENGTH] =  {
    {.width = SYNC_X,      .timestamp = expectedSyncTime + (0*FRAME_LENGTH)+0},
    {.width = SYNC_X_SKIP, .timestamp = expectedSyncTime + (0*FRAME_LENGTH)+SYNC_SEPARATION},
    {.width = SWEEP,       .timestamp = expectedSyncTime + (0*FRAME_LENGTH)+SWEEP_CENTER},

    {.width = SYNC_Y,      .timestamp = expectedSyncTime + (1*FRAME_LENGTH)+0},
    {.width = SYNC_Y_SKIP, .timestamp = expectedSyncTime + (1*FRAME_LENGTH)+SYNC_SEPARATION},
    {.width = SWEEP,       .timestamp = expectedSyncTime + (1*FRAME_LENGTH)+SWEEP_CENTER},

    {.width = SYNC_X_SKIP, .timestamp = expectedSyncTime + (2*FRAME_LENGTH)+0},
    {.width = SYNC_X,      .timestamp = expectedSyncTime + (2*FRAME_LENGTH)+SYNC_SEPARATION},
  };
  limitTimestamps(pulseHistory);

  // Test
  bool found = findSyncTime(pulseHistory, &actualSyncTime);

  // Assert
  TEST_ASSERT_TRUE(found);
  assertSyncTimeIsMultipleOfFrameLength(expectedSyncTime, actualSyncTime);
}


// Test helpers

void assertSyncTimeIsMultipleOfFrameLength(uint32_t expectedSyncTime, uint32_t actualSyncTime)
{
  uint32_t diff = actualSyncTime - expectedSyncTime;
  
  TEST_ASSERT_TRUE_MESSAGE(diff % FRAME_LENGTH < MAX_FRAME_LENGTH_NOISE, "Sync time out of bound");
}

void limitTimestamps(pulseProcessorPulse_t history[])
{
  for (int i=0; i<PULSE_PROCESSOR_HISTORY_LENGTH; i++) {
    history[i].timestamp &= ((1<<TIMESTAMP_BITWIDTH)-1);
  }
}