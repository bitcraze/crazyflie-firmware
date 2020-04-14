// File under test pulse_processor_v2.c
#include "pulse_processor_v2.h"

#include <stdlib.h>
#include <string.h>
#include "unity.h"

// Functions under test
void clearWorkspace(pulseProcessorV2PulseWorkspace_t* pulseWorkspace);
bool processBlock(const pulseProcessorV2PulseWorkspace_t* pulseWorkspace, pulseProcessorV2SweepBlock_t* block);
bool isBlockPairGood(const pulseProcessorV2SweepBlock_t* latest, pulseProcessorV2SweepBlock_t* storage);


// Helpers
static void allSensorsSet();
static void setChannel(uint8_t channel, uint8_t slowbit);
static void setOffset();
static void setOffsetBase(uint32_t ts_base);
static void setUpOkBlocks(pulseProcessorV2SweepBlock_t* newBlock, pulseProcessorV2SweepBlock_t* storageBlock);

static pulseProcessorV2PulseWorkspace_t ws;
static pulseProcessorV2SweepBlock_t block;

const uint32_t NO_OFFSET = 0;
const uint32_t OFFSET_BASE = 100000;
const uint32_t TIMESTAMP_BASE = 2000000;
const uint32_t TIMESTAMP_STEP = 1000;

void setUp(void) {
    clearWorkspace(&ws);
    allSensorsSet();
    setChannel(0, 0);
    setOffset();
}

void testThatWorkspaceIsCleared() {
    // Fixture

    // Test
    clearWorkspace(&ws);

    // Assert
    TEST_ASSERT_FALSE(ws.sensors[2].isSet);
}

void testThatProcessBlockRejectsMissingSensors() {
    // Fixture
    ws.sensors[2].isSet = false;

    // Test
    bool actual = processBlock(&ws, &block);

    // Assert
    TEST_ASSERT_FALSE(actual);
}

void testThatProcessBlockFindsChannel() {
    // Fixture
    uint8_t expected = 3;
    setChannel(expected, 0);

    // Test
    bool ok = processBlock(&ws, &block);

    // Assert
    uint8_t actual = block.channel;
    TEST_ASSERT_EQUAL_UINT8(expected, actual);
    TEST_ASSERT_TRUE(ok);
}

void testThatProcessBlockFindsSlowbit() {
    // Fixture
    uint8_t expected = 1;
    setChannel(0, expected);

    // Test
    bool ok = processBlock(&ws, &block);

    // Assert
    uint8_t actual = block.slowbit;
    TEST_ASSERT_EQUAL_UINT8(expected, actual);
    TEST_ASSERT_TRUE(ok);
}

void testThatProcessBlockRejectsMissingChannel() {
    // Fixture
    uint8_t channel = 3;

    setChannel(channel, 0);
    ws.sensors[0].channelFound = false;

    // Test
    bool actual = processBlock(&ws, &block);

    // Assert
    TEST_ASSERT_FALSE(actual);
}

void testThatProcessBlockRejectsMultipleChannels() {
    // Fixture
    uint8_t channel = 3;

    setChannel(channel, 0);
    ws.sensors[0].channel = channel + 1;

    // Test
    bool actual = processBlock(&ws, &block);

    // Assert
    TEST_ASSERT_FALSE(actual);
}

void testThatProcessBlockRejectsMultipleSlowbits() {
    // Fixture
    uint8_t channel = 3;

    setChannel(channel, 0);
    ws.sensors[0].slowbit = 1;

    // Test
    bool actual = processBlock(&ws, &block);

    // Assert
    TEST_ASSERT_FALSE(actual);
}

void testThatProcessBlockSetsOffsets() {
    // Fixture
    setOffsetBase(TIMESTAMP_BASE);

    // Test
    bool ok = processBlock(&ws, &block);

    // Assert
    TEST_ASSERT_EQUAL_UINT32(OFFSET_BASE + TIMESTAMP_STEP, block.offset[0]);
    TEST_ASSERT_EQUAL_UINT32(OFFSET_BASE, block.offset[1]);
    TEST_ASSERT_EQUAL_UINT32(OFFSET_BASE - TIMESTAMP_STEP, block.offset[2]);
    TEST_ASSERT_EQUAL_UINT32(OFFSET_BASE + TIMESTAMP_STEP * 2, block.offset[3]);

    TEST_ASSERT_TRUE(ok);
}

void testThatProcessBlockSetsOffsetsWhenTimeStampWraps() {
    // Fixture
    setOffsetBase(0x00ffff00);

    // Test
    bool ok = processBlock(&ws, &block);

    // Assert
    TEST_ASSERT_EQUAL_UINT32(OFFSET_BASE + TIMESTAMP_STEP, block.offset[0]);
    TEST_ASSERT_EQUAL_UINT32(OFFSET_BASE, block.offset[1]);
    TEST_ASSERT_EQUAL_UINT32(OFFSET_BASE - TIMESTAMP_STEP, block.offset[2]);
    TEST_ASSERT_EQUAL_UINT32(OFFSET_BASE + TIMESTAMP_STEP * 2, block.offset[3]);

    TEST_ASSERT_TRUE(ok);
}

void testThatProcessBlockRejectsMisissingOffset() {
    // Fixture
    ws.sensors[1].offset = NO_OFFSET;

    // Test
    bool actual = processBlock(&ws, &block);

    // Assert
    TEST_ASSERT_FALSE(actual);
}

void testThatProcessBlockRejectsMultipleOffsets() {
    // Fixture
    ws.sensors[0].offset = OFFSET_BASE;

    // Test
    bool actual = processBlock(&ws, &block);

    // Assert
    TEST_ASSERT_FALSE(actual);
}

void testThatProcessBlockSetsTimestampToSensor0Timestamp() {
    // Fixture
    uint32_t expected = TIMESTAMP_BASE + TIMESTAMP_STEP;

    // Test
    bool ok = processBlock(&ws, &block);

    // Assert
    uint32_t actual = block.timestamp;
    TEST_ASSERT_EQUAL_UINT32(expected, actual);
    TEST_ASSERT_TRUE(ok);
}

void testThatProcessBlockPairAcceptBlocks() {
    // Fixture
    pulseProcessorV2SweepBlock_t newBlock;
    pulseProcessorV2SweepBlock_t storageBlock;

    setUpOkBlocks(&newBlock, &storageBlock);

    // Test
    bool actual = isBlockPairGood(&newBlock, &storageBlock);

    // Assert
    TEST_ASSERT_TRUE(actual);
}

void testThatProcessBlockPairRejectsBlocksWithDifferentChannels() {
    // Fixture
    pulseProcessorV2SweepBlock_t newBlock;
    pulseProcessorV2SweepBlock_t storageBlock;

    setUpOkBlocks(&newBlock, &storageBlock);

    newBlock.channel = 0;

    // Test
    bool actual = isBlockPairGood(&newBlock, &storageBlock);

    // Assert
    TEST_ASSERT_FALSE(actual);
}

void testThatProcessBlockPairRejectsBlocksWhenTooLongApart() {
    // Fixture
    pulseProcessorV2SweepBlock_t newBlock;
    pulseProcessorV2SweepBlock_t storageBlock;

    setUpOkBlocks(&newBlock, &storageBlock);

    newBlock.timestamp = 2000000;

    // Test
    bool actual = isBlockPairGood(&newBlock, &storageBlock);

    // Assert
    TEST_ASSERT_FALSE(actual);
}

void testThatProcessBlockPairRejectsBlocksWhenSweepsInWrongOrder() {
    // Fixture
    pulseProcessorV2SweepBlock_t newBlock;
    pulseProcessorV2SweepBlock_t storageBlock;

    setUpOkBlocks(&newBlock, &storageBlock);

    newBlock.offset[0] = OFFSET_BASE;
    storageBlock.offset[0] = OFFSET_BASE + 100000;

    // Test
    bool actual = isBlockPairGood(&newBlock, &storageBlock);

    // Assert
    TEST_ASSERT_FALSE(actual);
}

// Helpers ------------------------------------------------

static void allSensorsSet() {
    for (int i = 0; i < PULSE_PROCESSOR_N_SENSORS; i++) {
        ws.sensors[i].isSet = true;
    }
}

static void setChannel(uint8_t channel, uint8_t slowbit) {
    for (int i = 0; i < (PULSE_PROCESSOR_N_SENSORS - 1); i++) {
        ws.sensors[i].channel = channel;
        ws.sensors[i].slowbit = slowbit;
        ws.sensors[i].channelFound = true;
    }

    ws.sensors[PULSE_PROCESSOR_N_SENSORS - 1].channel = 0;
    ws.sensors[PULSE_PROCESSOR_N_SENSORS - 1].slowbit = 0;
    ws.sensors[PULSE_PROCESSOR_N_SENSORS - 1].channelFound = false;
}

static void setOffset() {
    setOffsetBase(TIMESTAMP_BASE);
}

static void setOffsetBase(uint32_t ts_base) {
    ws.sensors[0].timestamp = ts_base + TIMESTAMP_STEP * 1;
    ws.sensors[0].offset = NO_OFFSET;

    ws.sensors[1].timestamp = ts_base;
    ws.sensors[1].offset = OFFSET_BASE;

    ws.sensors[2].timestamp = ts_base - TIMESTAMP_STEP * 1;
    ws.sensors[2].offset = NO_OFFSET;

    ws.sensors[3].timestamp = ts_base + TIMESTAMP_STEP * 2;
    ws.sensors[3].offset = NO_OFFSET;
}

static void setUpOkBlocks(pulseProcessorV2SweepBlock_t* newBlock, pulseProcessorV2SweepBlock_t* storageBlock) {
    storageBlock->channel = 1;
    storageBlock->timestamp = 1000000;

    newBlock->channel = 1;
    newBlock->timestamp = 1150000;
}
