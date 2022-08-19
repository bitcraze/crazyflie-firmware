// @IGNORE_IF_NOT CONFIG_DECK_LIGHTHOUSE

// File under test pulse_processor_v2.c
#include "pulse_processor_v2.h"

#include <stdlib.h>
#include <string.h>
#include "unity.h"

#include "mock_ootx_decoder.h"
#include "mock_lighthouse_calibration.h"
#include "mock_usec_time.h"
#include "mock_pulse_processor.h"


// Functions under test
void clearWorkspace(pulseProcessorV2PulseWorkspace_t* pulseWorkspace);
int processFrame(const pulseProcessorFrame_t* frameData, pulseProcessorV2PulseWorkspace_t* pulseWorkspace, pulseProcessorV2BlockWorkspace_t* blockWorkspace);
bool storePulse(const pulseProcessorFrame_t* frameData, pulseProcessorV2PulseWorkspace_t* pulseWorkspace);
void augmentFramesInWorkspace(pulseProcessorV2PulseWorkspace_t* pulseWorkspace);
bool processWorkspaceBlock(const pulseProcessorFrame_t slots[], pulseProcessorV2SweepBlock_t* block);
bool isBlockPairGood(const pulseProcessorV2SweepBlock_t* latest, const pulseProcessorV2SweepBlock_t* storage);
bool handleCalibrationData(pulseProcessor_t *state, const pulseProcessorFrame_t* frameData);

// Helpers
static void addDefaultFrames();
static void setChannel(uint8_t channel);
static void setOffsetBase(uint32_t ts_base);
static void setUpOkBlocks(pulseProcessorV2SweepBlock_t* newBlock, pulseProcessorV2SweepBlock_t* storageBlock);
static void addFrameToWs(uint8_t sensor, uint32_t timestamp, uint32_t offset, bool channelFound, uint8_t channel);
static void setUpOotxDecoderProcessBitCallCounter();
static void setUpSlowbitFrame();
static void clearSlowbitState();
static void validateProcessFrame(int expectedNrOfBlocks, uint8_t sensor, uint32_t timestamp, uint32_t offset, uint8_t channel, bool channelFound);

static pulseProcessor_t state;
static pulseProcessorV2PulseWorkspace_t ws;
static pulseProcessorV2SweepBlock_t block;
static pulseProcessorV2BlockWorkspace_t blockWorkspace;

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

void testThatMultipleFramesAreAugmented() {
    // Fixture
    uint8_t expectedChan1 = 2;
    uint8_t expectedChan2 = 5;

    clearWorkspace(&ws);

    addFrameToWs(0, A_TS, NO_OFFSET, false, 0);
    addFrameToWs(1, A_TS, NO_OFFSET, false, 0);
    addFrameToWs(2, A_TS, NO_OFFSET, false, 0);
    addFrameToWs(3, A_TS, AN_OFFSET, true, expectedChan1);

    addFrameToWs(0, A_TS, NO_OFFSET, false, 0);
    addFrameToWs(1, A_TS, NO_OFFSET, false, 0);
    addFrameToWs(2, A_TS, NO_OFFSET, false, 0);
    addFrameToWs(3, A_TS, AN_OFFSET, true, expectedChan2);

    // Test
    augmentFramesInWorkspace(&ws);

    // Assert
    TEST_ASSERT_EQUAL_UINT8(expectedChan1, ws.slots[0].channel);
    TEST_ASSERT_TRUE(ws.slots[0].channelFound);
    TEST_ASSERT_EQUAL_UINT8(expectedChan1, ws.slots[1].channel);
    TEST_ASSERT_TRUE(ws.slots[1].channelFound);
    TEST_ASSERT_EQUAL_UINT8(expectedChan1, ws.slots[2].channel);
    TEST_ASSERT_TRUE(ws.slots[2].channelFound);

    TEST_ASSERT_EQUAL_UINT8(expectedChan2, ws.slots[4].channel);
    TEST_ASSERT_TRUE(ws.slots[4].channelFound);
    TEST_ASSERT_EQUAL_UINT8(expectedChan2, ws.slots[5].channel);
    TEST_ASSERT_TRUE(ws.slots[5].channelFound);
    TEST_ASSERT_EQUAL_UINT8(expectedChan2, ws.slots[6].channel);
    TEST_ASSERT_TRUE(ws.slots[6].channelFound);
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
    TEST_ASSERT_TRUE(ws.slots[3].channelFound);
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


// Sensor: 1  TS:c53a8f  Width: 10f  Chan: None  offset:     0  BeamWord:10b2c
// Sensor: 3  TS:c53a79  Width: 125  Chan: None  offset:     0  BeamWord:1a42c
// Sensor: 2  TS:c53cbb  Width: 11e  Chan: 2(1)  offset: 44731  BeamWord:1409a
// Sensor: 0  TS:c53cd4  Width: 11e  Chan: 2(1)  offset:     0  BeamWord:0269a

void testThatProcessBlockSetsOffsetsWithOffsetOnThirdFrame() {
    // Fixture
    clearWorkspace(&ws);
    addFrameToWs(SWEEP_SENS_0, OFFSET_BASE + TIMESTAMP_STEP * 0, NO_OFFSET, true, 3);
    addFrameToWs(SWEEP_SENS_1, OFFSET_BASE + TIMESTAMP_STEP * 1, NO_OFFSET, true, 3);
    addFrameToWs(SWEEP_SENS_2, OFFSET_BASE + TIMESTAMP_STEP * 2, OFFSET_BASE, true, 3);
    addFrameToWs(SWEEP_SENS_3, OFFSET_BASE + TIMESTAMP_STEP * 3, NO_OFFSET, true, 3);

    // Test
    bool ok = processWorkspaceBlock(&ws.slots[0], &block);

    // Assert
    TEST_ASSERT_EQUAL_UINT32(OFFSET_BASE - TIMESTAMP_STEP * 2, block.offset[SWEEP_SENS_0]);
    TEST_ASSERT_EQUAL_UINT32(OFFSET_BASE - TIMESTAMP_STEP * 1, block.offset[SWEEP_SENS_1]);
    TEST_ASSERT_EQUAL_UINT32(OFFSET_BASE + TIMESTAMP_STEP * 0, block.offset[SWEEP_SENS_2]);
    TEST_ASSERT_EQUAL_UINT32(OFFSET_BASE + TIMESTAMP_STEP * 1, block.offset[SWEEP_SENS_3]);

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
    slowbitFrame.channel = CONFIG_DECK_LIGHTHOUSE_MAX_N_BS + 1;

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

void testThatFullSlowbitMessageIsReported() {
    // Fixture
    ootxDecoderProcessBit_IgnoreAndReturn(true);

    // Test
    bool actual = handleCalibrationData(&state, &slowbitFrame);

    // Assert
    TEST_ASSERT_TRUE(actual);
}

void testThatNonFullSlowbitMessageIsNotReported() {
    // Fixture
    ootxDecoderProcessBit_IgnoreAndReturn(false);

    // Test
    bool actual = handleCalibrationData(&state, &slowbitFrame);

    // Assert
    TEST_ASSERT_FALSE(actual);
}

// Recording with one base station in a random orientation
void testRecordedSequence1Bs_1() {
    // Fixture
    clearWorkspace(&ws);

    // Test and assert
    validateProcessFrame(0, 0, 2156620, 0,      0, false);
    validateProcessFrame(0, 2, 2156972, 165916, 0, true);
    validateProcessFrame(0, 1, 2157193, 0,      0, true);
    validateProcessFrame(0, 3, 2157561, 0,      0, true);

    validateProcessFrame(1, 2, 2290186, 0,      0, false);
    validateProcessFrame(0, 0, 2290608, 299556, 0, true);
    validateProcessFrame(0, 3, 2290750, 0,      0, true);
    validateProcessFrame(0, 1, 2291154, 0,      0, true);

    validateProcessFrame(1, 0, 2635114, 0,      0, false);
    validateProcessFrame(0, 2, 2635466, 165920, 0, true);
    validateProcessFrame(0, 1, 2635687, 0,      0, true);
    validateProcessFrame(0, 3, 2636051, 0,      0, true);

    validateProcessFrame(1, 2, 2768686, 0,      0, false);
    validateProcessFrame(0, 0, 2769102, 299556, 0, true);
    validateProcessFrame(0, 3, 2769244, 0,      0, true);
    validateProcessFrame(0, 1, 2769648, 0,      0, true);
}

void testIssue901Data() {
    // Fixture
    clearWorkspace(&ws);

    // Test and assert
    validateProcessFrame(0, 1, 0xc53a8f, 0,      0, false);
    validateProcessFrame(0, 3, 0xc53a79, 0,      0, false);
    validateProcessFrame(0, 2, 0xc53cbb, 44731,  0, true);
    validateProcessFrame(0, 0, 0xc53cd4, 0,      0, true);

    // Fake data from now on
    validateProcessFrame(1, 1, 0xc53a8f + 130000, 0,      0, false);
    validateProcessFrame(0, 3, 0xc53a79 + 130000, 0,      0, false);
    validateProcessFrame(0, 2, 0xc53cbb + 130000, 44731,  0, true);
    validateProcessFrame(0, 0, 0xc53cd4 + 130000, 0,      0, true);
}



// Helpers ------------------------------------------------

static void validateProcessFrame(int expectedNrOfBlocks, uint8_t sensor, uint32_t timestamp, uint32_t offset, uint8_t channel, bool channelFound) {
    pulseProcessorFrame_t frameData;
    frameData.sensor = sensor;
    frameData.timestamp = timestamp;
    frameData.offset = offset;
    frameData.channel = channel;
    frameData.channelFound = channelFound;

    int actual = processFrame(&frameData, &ws, &blockWorkspace);

    char errorMsg[200];
    sprintf(errorMsg, "Failed for timestamp=%ul", timestamp);
    TEST_ASSERT_EQUAL_INT32_MESSAGE(expectedNrOfBlocks, actual, errorMsg);
}

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
    slowbitFrame.slowBit = SB_BIT;
    slowbitFrame.channelFound = true;
}

static void clearSlowbitState() {
    for (int i = 0; i < CONFIG_DECK_LIGHTHOUSE_MAX_N_BS; i++) {
        state.v2.ootxTimestamps[i] = 0;
        state.bsCalibration[i].valid = false;
    }
}
