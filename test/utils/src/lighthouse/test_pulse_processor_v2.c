// File under test pulse_processor_v2.c
#include "pulse_processor_v2.h"

#include <stdlib.h>
#include <string.h>
#include "unity.h"

#include "mock_ootx_decoder.h"
#include "mock_lighthouse_calibration.h"


// Functions under test
void clearWorkspace(pulseProcessorV2PulseWorkspace_t* pulseWorkspace);
bool storePulse(const pulseProcessorFrame_t* frameData, pulseProcessorV2PulseWorkspace_t* pulseWorkspace);
void augmentFramesInWorkspace(pulseProcessorV2PulseWorkspace_t* pulseWorkspace);
bool processWorkspaceBlock(const pulseProcessorFrame_t slots[], pulseProcessorV2SweepBlock_t* block);
bool isBlockPairGood(const pulseProcessorV2SweepBlock_t* latest, const pulseProcessorV2SweepBlock_t* storage);
void handleCalibrationData(pulseProcessor_t *state, const pulseProcessorFrame_t* frameData);

// Helpers
static void addDefaultFrames();
static void setChannel(uint8_t channel);
static void setOffsetBase(uint32_t ts_base);
static void setUpOkBlocks(pulseProcessorV2SweepBlock_t* newBlock, pulseProcessorV2SweepBlock_t* storageBlock);
static void addFrameToWs(uint8_t sensor, uint32_t timestamp, uint32_t offset, bool channelFound, uint8_t channel);
static void setUpOotxDecoderProcessBitCallCounter();
static void setUpSlowbitFrame();
static void clearSlowbitState();

static pulseProcessor_t state;
static pulseProcessorV2PulseWorkspace_t ws;
static pulseProcessorV2SweepBlock_t block;

static pulseProcessorFrame_t slowbitFrame;
static int nrOfCallsToOotxDecoderProcessBit;

const uint32_t NO_OFFSET = 0;
const uint32_t OFFSET_BASE = 100000;
const uint32_t TIMESTAMP_BASE = 2000000;
const uint32_t TIMESTAMP_STEP = 1000;

const uint32_t A_TS = 4711;
const uint32_t AN_OFFSET = 17;

const uint32_t SB_CHANNEL = 1;
const uint32_t SB_BIT = 1;

// The order the sensors are hit by the beam
const uint8_t SWEEP_SENS_0 = 3;
const uint8_t SWEEP_SENS_1 = 1;
const uint8_t SWEEP_SENS_2 = 0;
const uint8_t SWEEP_SENS_3 = 2;

void setUp(void) {
    clearWorkspace(&ws);
    addDefaultFrames();

    block.channel = 0;

    setUpSlowbitFrame();
    clearSlowbitState();
    nrOfCallsToOotxDecoderProcessBit = -1;
}

void testThatWorkspaceIsCleared() {
    // Fixture

    // Test
    clearWorkspace(&ws);

    // Assert
    TEST_ASSERT_EQUAL_INT(0, ws.slotsUsed);
}

void testThatAPulseIsStoredInTheWorkspace() {
    // Fixture
    clearWorkspace(&ws);

    pulseProcessorFrame_t frame;

    uint8_t expected = 7;
    frame.channel = expected;

    // Test
    bool actual = storePulse(&frame, &ws);

    // Assert
    TEST_ASSERT_TRUE(actual);
    TEST_ASSERT_EQUAL_INT(1, ws.slotsUsed);
    TEST_ASSERT_EQUAL_UINT8(expected, ws.slots[0].channel);
}

void testThatAPulseIsRejectedWhenWorkspaceIsFull() {
    // Fixture
    clearWorkspace(&ws);

    pulseProcessorFrame_t frame;

    for (int i = 0; i < PULSE_PROCESSOR_N_WORKSPACE; i++) {
        bool actual = storePulse(&frame, &ws);
        TEST_ASSERT_TRUE(actual);
    }

    // Test
    bool actual = storePulse(&frame, &ws);

    // Assert
    TEST_ASSERT_FALSE(actual);
}

void testThatCleanFramesAreAugmented() {
    // Fixture
    uint8_t expectedChan1 = 2;
    uint8_t expectedChan2 = 5;

    clearWorkspace(&ws);

    addFrameToWs(0, A_TS, NO_OFFSET, false, 0);
    addFrameToWs(1, A_TS, AN_OFFSET, true, expectedChan1);
    addFrameToWs(2, A_TS, NO_OFFSET, true, expectedChan1);
    addFrameToWs(3, A_TS, NO_OFFSET, true, expectedChan1);

    addFrameToWs(0, A_TS, AN_OFFSET, false, 0);
    addFrameToWs(1, A_TS, AN_OFFSET, true, expectedChan2);
    addFrameToWs(2, A_TS, AN_OFFSET, true, expectedChan2);
    addFrameToWs(3, A_TS, AN_OFFSET, true, expectedChan2);

    // Test
    augmentFramesInWorkspace(&ws);

    // Assert
    TEST_ASSERT_EQUAL_UINT8(expectedChan1, ws.slots[0].channel);
    TEST_ASSERT_TRUE(ws.slots[0].channelFound);

    TEST_ASSERT_EQUAL_UINT8(expectedChan2, ws.slots[4].channel);
    TEST_ASSERT_TRUE(ws.slots[4].channelFound);
}

void testThatUnCleanFramesAreAugmented() {
    // Fixture
    uint8_t expectedChan1 = 2;
    uint8_t expectedChan2 = 5;

    clearWorkspace(&ws);

    addFrameToWs(0, A_TS, AN_OFFSET, true, 1);
    addFrameToWs(1, A_TS, AN_OFFSET, false, 0);
    addFrameToWs(2, A_TS, AN_OFFSET, true, expectedChan1);
    addFrameToWs(3, A_TS, AN_OFFSET, false, 0);

    addFrameToWs(0, A_TS, AN_OFFSET, false, 0);
    addFrameToWs(1, A_TS, AN_OFFSET, true, expectedChan2);
    addFrameToWs(2, A_TS, AN_OFFSET, true, 1);
    addFrameToWs(3, A_TS, AN_OFFSET, false, 0);

    // Test
    augmentFramesInWorkspace(&ws);

    // Assert
    TEST_ASSERT_EQUAL_UINT8(expectedChan1, ws.slots[1].channel);
    TEST_ASSERT_FALSE(ws.slots[3].channelFound);
    TEST_ASSERT_EQUAL_UINT8(expectedChan2, ws.slots[4].channel);
    TEST_ASSERT_FALSE(ws.slots[7].channelFound);
}

void testThatProcessBlockRejectsMissingSensors() {
    // Fixture
    ws.slots[2].sensor = SWEEP_SENS_0;

    // Test
    bool actual = processWorkspaceBlock(&ws.slots[0], &block);

    // Assert
    TEST_ASSERT_FALSE(actual);
}

void testThatProcessBlockFindsChannel() {
    // Fixture
    uint8_t expected = 3;
    setChannel(expected);

    // Test
    bool ok = processWorkspaceBlock(&ws.slots[0], &block);

    // Assert
    uint8_t actual = block.channel;
    TEST_ASSERT_EQUAL_UINT8(expected, actual);
    TEST_ASSERT_TRUE(ok);
}

void testThatProcessBlockFindsChannelWhenSomeChannelsAreMissing() {
    // Fixture
    uint8_t expected = 3;
    setChannel(expected);
    ws.slots[2].channelFound = false;
    ws.slots[3].channelFound = false;

    // Test
    bool ok = processWorkspaceBlock(&ws.slots[0], &block);

    // Assert
    uint8_t actual = block.channel;
    TEST_ASSERT_EQUAL_UINT8(expected, actual);
    TEST_ASSERT_TRUE(ok);
}

void testThatProcessBlockRejectsAllChannelsMissing() {
    // Fixture
    ws.slots[0].channelFound = false;
    ws.slots[1].channelFound = false;
    ws.slots[2].channelFound = false;
    ws.slots[3].channelFound = false;

    // Test
    bool actual = processWorkspaceBlock(&ws.slots[0], &block);

    // Assert
    TEST_ASSERT_FALSE(actual);
}

void testThatProcessBlockRejectsMultipleChannels() {
    // Fixture
    uint8_t channel = 3;

    setChannel(channel);
    ws.slots[0].channel = channel + 1;

    // Test
    bool actual = processWorkspaceBlock(&ws.slots[0], &block);

    // Assert
    TEST_ASSERT_FALSE(actual);
}

void testThatProcessBlockRejectsMisissingOffset() {
    // Fixture
    ws.slots[1].offset = NO_OFFSET;

    // Test
    bool actual = processWorkspaceBlock(&ws.slots[0], &block);

    // Assert
    TEST_ASSERT_FALSE(actual);
}

void testThatProcessBlockRejectsMultipleOffsets() {
    // Fixture
    ws.slots[0].offset = OFFSET_BASE;

    // Test
    bool actual = processWorkspaceBlock(&ws.slots[0], &block);

    // Assert
    TEST_ASSERT_FALSE(actual);
}

void testThatProcessBlockSetsOffsets() {
    // Fixture
    setOffsetBase(TIMESTAMP_BASE);

    // Test
    bool ok = processWorkspaceBlock(&ws.slots[0], &block);

    // Assert
    TEST_ASSERT_EQUAL_UINT32(OFFSET_BASE + TIMESTAMP_STEP, block.offset[SWEEP_SENS_0]);
    TEST_ASSERT_EQUAL_UINT32(OFFSET_BASE, block.offset[SWEEP_SENS_1]);
    TEST_ASSERT_EQUAL_UINT32(OFFSET_BASE - TIMESTAMP_STEP, block.offset[SWEEP_SENS_2]);
    TEST_ASSERT_EQUAL_UINT32(OFFSET_BASE + TIMESTAMP_STEP * 2, block.offset[SWEEP_SENS_3]);

    TEST_ASSERT_TRUE(ok);
}

void testThatProcessBlockSetsOffsetsWhenTimeStampWraps() {
    // Fixture
    setOffsetBase(0x00ffff00);

    // Test
    bool ok = processWorkspaceBlock(&ws.slots[0], &block);

    // Assert
    TEST_ASSERT_EQUAL_UINT32(OFFSET_BASE + TIMESTAMP_STEP, block.offset[SWEEP_SENS_0]);
    TEST_ASSERT_EQUAL_UINT32(OFFSET_BASE, block.offset[SWEEP_SENS_1]);
    TEST_ASSERT_EQUAL_UINT32(OFFSET_BASE - TIMESTAMP_STEP, block.offset[SWEEP_SENS_2]);
    TEST_ASSERT_EQUAL_UINT32(OFFSET_BASE + TIMESTAMP_STEP * 2, block.offset[SWEEP_SENS_3]);

    TEST_ASSERT_TRUE(ok);
}


void testThatProcessBlockSetsTimestampToStartOfRotation() {
    // Fixture
    setOffsetBase(TIMESTAMP_BASE);
    uint32_t expected = TIMESTAMP_BASE - OFFSET_BASE;

    // Test
    bool ok = processWorkspaceBlock(&ws.slots[0], &block);

    // Assert
    uint32_t actual = block.timestamp0;
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

    newBlock.timestamp0 = newBlock.timestamp0 + 20;

    // Test
    bool actual = isBlockPairGood(&newBlock, &storageBlock);

    // Assert
    TEST_ASSERT_FALSE(actual);
}

void testThatSlowBitIsProcessed() {
    // Fixture
    ootxDecoderProcessBit_ExpectAndReturn(&state.ootxDecoder[SB_CHANNEL], SB_BIT, false);

    // Test
    handleCalibrationData(&state, &slowbitFrame);

    // Assert
    // Validated in mock
}

void testThatSlowBitIsNotProcessedIfChannelIsMissing() {
    // Fixture
    slowbitFrame.channelFound = false;

    setUpOotxDecoderProcessBitCallCounter();

    // Test
    handleCalibrationData(&state, &slowbitFrame);

    // Assert
    TEST_ASSERT_EQUAL(0, nrOfCallsToOotxDecoderProcessBit);
}

void testThatSlowBitIsNotProcessedIfChannelIsOutOfBounds() {
    // Fixture
    slowbitFrame.channel = PULSE_PROCESSOR_N_BASE_STATIONS + 1;

    setUpOotxDecoderProcessBitCallCounter();

    // Test
    handleCalibrationData(&state, &slowbitFrame);

    // Assert
    TEST_ASSERT_EQUAL(0, nrOfCallsToOotxDecoderProcessBit);
}

void testThatSlowBitIsNotProcessedIfOffsetIsMissing() {
    // Fixture
    slowbitFrame.offset = NO_OFFSET;

    setUpOotxDecoderProcessBitCallCounter();

    // Test
    handleCalibrationData(&state, &slowbitFrame);

    // Assert
    TEST_ASSERT_EQUAL(0, nrOfCallsToOotxDecoderProcessBit);
}

void testThatSlowBitIsNotProcessedIfThereAlreadyIsCalibrationData() {
    // Fixture
    state.bsCalibration[SB_CHANNEL].valid = true;

    setUpOotxDecoderProcessBitCallCounter();

    // Test
    handleCalibrationData(&state, &slowbitFrame);

    // Assert
    TEST_ASSERT_EQUAL(0, nrOfCallsToOotxDecoderProcessBit);
}

void testThatMoreThanOneSlowBitIsProcessed() {
    // Fixture
    setUpOotxDecoderProcessBitCallCounter();

    uint32_t timestamp0 = 1000000;
    slowbitFrame.offset = 10000;

    // First Bit
    slowbitFrame.timestamp = timestamp0 + slowbitFrame.offset;
    handleCalibrationData(&state, &slowbitFrame);

    // Second sweep
    slowbitFrame.timestamp = slowbitFrame.timestamp + 959000 / 2;

    // Test
    handleCalibrationData(&state, &slowbitFrame);

    // Assert
    TEST_ASSERT_EQUAL(2, nrOfCallsToOotxDecoderProcessBit);
}

void testThatOnlyOneSlowBitPerRevolutionIsProcessed1() {
    // Fixture
    setUpOotxDecoderProcessBitCallCounter();

    uint32_t timestamp0 = 1000000;
    // First Sweep
    slowbitFrame.offset = 10000;
    slowbitFrame.timestamp = timestamp0 + slowbitFrame.offset + 3;
    handleCalibrationData(&state, &slowbitFrame);

    // Second sweep
    slowbitFrame.offset = 200000;
    slowbitFrame.timestamp = timestamp0 + slowbitFrame.offset - 2;

    // Test
    handleCalibrationData(&state, &slowbitFrame);

    // Assert
    TEST_ASSERT_EQUAL(1, nrOfCallsToOotxDecoderProcessBit);
}

void testThatOnlyOneSlowBitsPerRevolutionIsProcessed2() {
    // Fixture
    setUpOotxDecoderProcessBitCallCounter();

    uint32_t timestamp0 = 1000000;
    // First Sweep
    slowbitFrame.offset = 10000;
    slowbitFrame.timestamp = timestamp0 + slowbitFrame.offset - 3;
    handleCalibrationData(&state, &slowbitFrame);

    // Second sweep
    slowbitFrame.offset = 200000;
    slowbitFrame.timestamp = timestamp0 + slowbitFrame.offset + 2;

    // Test
    handleCalibrationData(&state, &slowbitFrame);

    // Assert
    TEST_ASSERT_EQUAL(1, nrOfCallsToOotxDecoderProcessBit);
}

void testThatFullSlowbitMessageIsPassedOnAsCalibrationData() {
    // Fixture
    ootxDecoderProcessBit_IgnoreAndReturn(true);
    lighthouseCalibrationInitFromFrame_Expect(&state.bsCalibration[SB_CHANNEL], &state.ootxDecoder[SB_CHANNEL].frame);

    // Test
    handleCalibrationData(&state, &slowbitFrame);

    // Assert
    // Verified in mocks
}


// Helpers ------------------------------------------------

static void addFrameToWs(uint8_t sensor, uint32_t timestamp, uint32_t offset, bool channelFound, uint8_t channel) {
    pulseProcessorFrame_t frame;

    frame.sensor = sensor;
    frame.timestamp = timestamp;
    frame.offset = offset;
    frame.channelFound = channelFound;
    frame.channel = channel;

    storePulse(&frame, &ws);
}

static void addDefaultFrames() {
    addFrameToWs(SWEEP_SENS_0, TIMESTAMP_STEP * 0, NO_OFFSET, true, 3);
    addFrameToWs(SWEEP_SENS_1, TIMESTAMP_STEP * 1, OFFSET_BASE, true, 3);
    addFrameToWs(SWEEP_SENS_2, TIMESTAMP_STEP * 2, NO_OFFSET, true, 3);
    addFrameToWs(SWEEP_SENS_3, TIMESTAMP_STEP * 3, NO_OFFSET, true, 3);
}

static void setChannel(uint8_t channel) {
    for (int i = 0; i < PULSE_PROCESSOR_N_SENSORS; i++) {
        ws.slots[i].channel = channel;
        ws.slots[i].channelFound = true;
    }
}

static void setOffsetBase(uint32_t ts_base) {
    ws.slots[0].timestamp = ts_base + TIMESTAMP_STEP * 1;
    ws.slots[0].offset = NO_OFFSET;

    ws.slots[1].timestamp = ts_base;
    ws.slots[1].offset = OFFSET_BASE;

    ws.slots[2].timestamp = ts_base - TIMESTAMP_STEP * 1;
    ws.slots[2].offset = NO_OFFSET;

    ws.slots[3].timestamp = ts_base + TIMESTAMP_STEP * 2;
    ws.slots[3].offset = NO_OFFSET;
}

static void setUpOkBlocks(pulseProcessorV2SweepBlock_t* newBlock, pulseProcessorV2SweepBlock_t* storageBlock) {
    storageBlock->channel = 1;
    storageBlock->timestamp0 = 1000000;

    newBlock->channel = 1;
    newBlock->timestamp0 = 1000003;
}


static bool ootxDecoderProcessBitCallback(ootxDecoderState_t* state, int data, int cmock_num_calls) {
    nrOfCallsToOotxDecoderProcessBit++;
    return false;
}

static void setUpOotxDecoderProcessBitCallCounter() {
    nrOfCallsToOotxDecoderProcessBit = 0;
    ootxDecoderProcessBit_StubWithCallback(ootxDecoderProcessBitCallback);
}

static void setUpSlowbitFrame(){
    slowbitFrame.timestamp = 1000000;
    slowbitFrame.offset = 100000;
    slowbitFrame.channel = SB_CHANNEL;
    slowbitFrame.slowbit = SB_BIT;
    slowbitFrame.channelFound = true;
}

static void clearSlowbitState() {
    for (int i = 0; i < PULSE_PROCESSOR_N_BASE_STATIONS; i++) {
        state.v2.ootxTimestamps[i] = 0;
        state.bsCalibration[i].valid = false;
    }
}
