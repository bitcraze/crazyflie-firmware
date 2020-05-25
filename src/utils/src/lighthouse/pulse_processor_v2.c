/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2020 Bitcraze AB
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
 *
 * pulse_processor_v2.c - pulse decoding for lighthouse V2 base stations
 *
 */

#include "pulse_processor_v2.h"

#include <string.h>
#include <math.h>
#include "math3d.h"
#include "test_support.h"
#include "debug.h"

static const uint32_t MAX_TICKS_SENSOR_TO_SENSOR = 10000;
static const uint32_t MAX_TICKS_BETWEEN_SWEEP_STARTS_TWO_BLOCKS = 10;
static const uint32_t MIN_TICKS_BETWEEN_SLOW_BITS = (887000 / 2) * 8 / 10; // 80 of one revolution

static const uint8_t NO_CHANNEL = 0xff;
static const int NO_SENSOR = -1;
static const uint32_t NO_OFFSET = 0;

#define V2_N_CHANNELS 16

// The cycle times come from the base stations and are expressed in a 48 MHz clock, we use 24 MHz clock hence the "/ 2".
static const uint32_t CYCLE_PERIODS[V2_N_CHANNELS] = {
    959000 / 2, 957000 / 2, 953000 / 2, 949000 / 2,
    947000 / 2, 943000 / 2, 941000 / 2, 939000 / 2,
    937000 / 2, 929000 / 2, 919000 / 2, 911000 / 2,
    907000 / 2, 901000 / 2, 893000 / 2, 887000 / 2
};

TESTABLE_STATIC bool processWorkspaceBlock(const pulseProcessorFrame_t slots[], pulseProcessorV2SweepBlock_t* block) {
    // Check we have data for all sensors
    uint8_t sensorMask = 0;
    for (int i = 0; i < PULSE_PROCESSOR_N_SENSORS; i++) {
        const pulseProcessorFrame_t* frame = &slots[i];
        sensorMask |= (1 << frame->sensor);
    }

    if (sensorMask != 0xf) {
        // All sensors not present - discard
        return false;
    }

    // Channel - should all be the same or not set
    block->channel = NO_CHANNEL;
    for (int i = 0; i < PULSE_PROCESSOR_N_SENSORS; i++) {
        const pulseProcessorFrame_t* frame = &slots[i];

        if (frame->channelFound) {
            if (block->channel == NO_CHANNEL) {
                block->channel = frame->channel;
            }

            if (block->channel != frame->channel) {
                // Multiple channels in the block - discard
                return false;
            }
        }
    }

    if (block->channel == NO_CHANNEL) {
        // Channel is missing - discard
        return false;
    }

    // Offset - should be offset on one and only one sensor
    int indexWithOffset = NO_SENSOR;
    for (int i = 0; i < PULSE_PROCESSOR_N_SENSORS; i++) {
        const pulseProcessorFrame_t* frame = &slots[i];

        if (frame->offset != NO_OFFSET) {
            if (indexWithOffset == NO_SENSOR) {
                indexWithOffset = i;
            } else {
                // Duplicate offsets - discard
                return false;
            }
        }
    }

    if (indexWithOffset == NO_SENSOR) {
        // No offset found - discard
        return false;
    }

    // Calculate offsets for all sensors
    const pulseProcessorFrame_t* baseSensor = &slots[indexWithOffset];
    for (int i = 0; i < PULSE_PROCESSOR_N_SENSORS; i++) {
        const pulseProcessorFrame_t* frame = &slots[i];
        uint8_t sensor = frame->sensor;

        if (i == indexWithOffset) {
            block->offset[sensor] = frame->offset;
        } else {
            uint32_t timestamp_delta = TS_DIFF(baseSensor->timestamp, frame->timestamp);
            block->offset[sensor] = TS_DIFF(baseSensor->offset, timestamp_delta);
        }
    }

    block->timestamp0 = TS_DIFF(baseSensor->timestamp, baseSensor->offset);

    return true;
}

/**
 * @brief Set channel for frames preceding a frame with known channel.
 * The FPGA decoding process does not fill in this data since it needs data
 * from a second sensor to make sens of the first one
 *
 * @param pulseWorkspace
 */
TESTABLE_STATIC void augmentFramesInWorkspace(pulseProcessorV2PulseWorkspace_t* pulseWorkspace) {
    const int slotsUsed = pulseWorkspace->slotsUsed;

    for (int i = 0; i < slotsUsed - 1; i++) {
        pulseProcessorFrame_t* previousFrame = &pulseWorkspace->slots[i];
        const pulseProcessorFrame_t* frame = &pulseWorkspace->slots[i + 1];
        if (! previousFrame->channelFound) {
            if (frame->channelFound) {
                previousFrame->channel = frame->channel;
                previousFrame->channelFound = frame->channelFound;
                i++;
            }
        }
    }
}

static int processWorkspace(pulseProcessorV2PulseWorkspace_t* pulseWorkspace, pulseProcessorV2BlockWorkspace_t* blockWorkspace) {
    augmentFramesInWorkspace(pulseWorkspace);

    // Handle missing channels
    // Sometimes the FPGA failes to decode the bitstream for a sensor, but we
    // can assume the timestamp is correct anyway. To know which channel it
    // has, we add the limitation that we only accept blocks where all sensors are present.
    // Further more we only accept workspaces with full and consecutive blocks.

    const int slotsUsed = pulseWorkspace->slotsUsed;
    // We must have at least one block
    if (slotsUsed < PULSE_PROCESSOR_N_SENSORS) {
        return 0;
    }

    // The number of slots used must be a multiple of the block size
    if ((slotsUsed % PULSE_PROCESSOR_N_SENSORS) != 0) {
        return 0;
    }

    // Process one block at a time in the workspace
    int blocksInWorkspace = slotsUsed / PULSE_PROCESSOR_N_SENSORS;
    for (int blockIndex = 0; blockIndex < blocksInWorkspace; blockIndex++) {
        int blockBaseIndex = blockIndex * PULSE_PROCESSOR_N_SENSORS;
        if (! processWorkspaceBlock(&pulseWorkspace->slots[blockBaseIndex], &blockWorkspace->blocks[blockIndex])) {
            // If one block is bad, reject the full workspace
            return 0;
        }
    }

    return blocksInWorkspace;
}

TESTABLE_STATIC bool storePulse(const pulseProcessorFrame_t* frameData, pulseProcessorV2PulseWorkspace_t* pulseWorkspace) {
    bool result = false;
    if (pulseWorkspace->slotsUsed < PULSE_PROCESSOR_N_WORKSPACE) {
        memcpy(&pulseWorkspace->slots[pulseWorkspace->slotsUsed], frameData, sizeof(pulseProcessorFrame_t));
        pulseWorkspace->slotsUsed += 1;
        result = true;
    }

    return result;
}

TESTABLE_STATIC void clearWorkspace(pulseProcessorV2PulseWorkspace_t* pulseWorkspace) {
    pulseWorkspace->slotsUsed = 0;
}

static bool processFrame(const pulseProcessorFrame_t* frameData, pulseProcessorV2PulseWorkspace_t* pulseWorkspace, pulseProcessorV2BlockWorkspace_t* blockWorkspace) {
    int nrOfBlocks = 0;

    // Sensor timestamps may arrive in the wrong order, we need an abs() when checking the diff
    const bool isFirstFrameInNewWorkspace = TS_ABS_DIFF_LARGER_THAN(frameData->timestamp, pulseWorkspace->latestTimestamp, MAX_TICKS_SENSOR_TO_SENSOR);
    if (isFirstFrameInNewWorkspace) {
        nrOfBlocks = processWorkspace(pulseWorkspace, blockWorkspace);
        clearWorkspace(pulseWorkspace);
    }

    pulseWorkspace->latestTimestamp = frameData->timestamp;

    if (! storePulse(frameData, pulseWorkspace)) {
        clearWorkspace(pulseWorkspace);
    }

    return nrOfBlocks;
}

void pulseProcessorV2ConvertToV1Angles(const float v2Angle1, const float v2Angle2, float* v1Angles) {
    const float tan_p_2 = 0.5773502691896258f;   // tan(60 / 2)

    v1Angles[0] = (v2Angle1 + v2Angle2) / 2.0f;
    float beta = v2Angle2 - v2Angle1;
    v1Angles[1] = atan(sinf(beta / 2.0f) / tan_p_2);
}

static void calculateAngles(const pulseProcessorV2SweepBlock_t* latestBlock, const pulseProcessorV2SweepBlock_t* previousBlock, pulseProcessorResult_t* angles) {
    const uint8_t channel = latestBlock->channel;

    for (int i = 0; i < PULSE_PROCESSOR_N_SENSORS; i++) {
        uint32_t firstOffset = previousBlock->offset[i];
        uint32_t secondOffset = latestBlock->offset[i];
        uint32_t period = CYCLE_PERIODS[channel];

        float firstBeam = (firstOffset * 2 * M_PI_F / period) - M_PI_F + M_PI_F / 3.0f;
        float secondBeam = (secondOffset * 2 * M_PI_F / period) - M_PI_F - M_PI_F / 3.0f;

        pulseProcessorBaseStationMeasuremnt_t* measurement = &angles->sensorMeasurementsLh2[i].baseStatonMeasurements[channel];
        measurement->angles[0] = firstBeam;
        measurement->angles[1] = secondBeam;
        measurement->validCount = 2;
    }
}

TESTABLE_STATIC bool isBlockPairGood(const pulseProcessorV2SweepBlock_t* latest, const pulseProcessorV2SweepBlock_t* storage) {
    if (latest->channel != storage->channel) {
        return false;
    }

    if (TS_ABS_DIFF_LARGER_THAN(latest->timestamp0, storage->timestamp0, MAX_TICKS_BETWEEN_SWEEP_STARTS_TWO_BLOCKS)) {
        return false;
    }

    return true;
}

static void printBSInfo(struct ootxDataFrame_s *frame) {
  DEBUG_PRINT("Got calibration from %08X\n", (unsigned int)frame->id);
//   DEBUG_PRINT("  tilt0: %f\n", (double)frame->tilt0);
//   DEBUG_PRINT("  phase0: %f\n", (double)frame->phase0);
//   DEBUG_PRINT("  curve0: %f\n", (double)frame->curve0);
//   DEBUG_PRINT("  gibphase0: %f\n", (double)frame->gibphase0);
//   DEBUG_PRINT("  gibmag0: %f\n", (double)frame->gibmag0);
//   DEBUG_PRINT("  ogeephase0: %f\n", (double)frame->ogeephase0);
//   DEBUG_PRINT("  ogeemag0: %f\n", (double)frame->ogeemag0);

//   DEBUG_PRINT("  tilt1: %f\n", (double)frame->tilt1);
//   DEBUG_PRINT("  phase1: %f\n", (double)frame->phase1);
//   DEBUG_PRINT("  curve1: %f\n", (double)frame->curve1);
//   DEBUG_PRINT("  gibphase1: %f\n", (double)frame->gibphase1);
//   DEBUG_PRINT("  gibmag1: %f\n", (double)frame->gibmag1);
//   DEBUG_PRINT("  ogeephase1: %f\n", (double)frame->ogeephase1);
//   DEBUG_PRINT("  ogeemag1: %f\n", (double)frame->ogeemag1);
}

TESTABLE_STATIC void handleCalibrationData(pulseProcessor_t *state, const pulseProcessorFrame_t* frameData) {
    if (frameData->channelFound && frameData->channel < PULSE_PROCESSOR_N_BASE_STATIONS) {
        const uint8_t channel = frameData->channel;
        if (frameData->offset != NO_OFFSET) {
            if (! state->bsCalibration[channel].valid) {
                const uint32_t prevTimestamp0 = state->v2.ootxTimestamps[channel];
                const uint32_t timestamp0 = TS_DIFF(frameData->timestamp, frameData->offset);

                if (TS_ABS_DIFF_LARGER_THAN(timestamp0, prevTimestamp0, MIN_TICKS_BETWEEN_SLOW_BITS)) {
                    bool fullMessage = ootxDecoderProcessBit(&state->ootxDecoder[channel], frameData->slowbit);
                    if (fullMessage) {
                        printBSInfo(&state->ootxDecoder[channel].frame);
                        lighthouseCalibrationInitFromFrame(&state->bsCalibration[channel], &state->ootxDecoder[channel].frame);
                    }
                }

                state->v2.ootxTimestamps[channel] = timestamp0;
            }
        }
    }
}

bool handleAngles(pulseProcessor_t *state, const pulseProcessorFrame_t* frameData, pulseProcessorResult_t* angles, int *baseStation, int *axis) {
    bool anglesMeasured = false;
    int nrOfBlocks = processFrame(frameData, &state->v2.pulseWorkspace, &state->v2.blockWorkspace);
    for (int i = 0; i < nrOfBlocks; i++) {
        const pulseProcessorV2SweepBlock_t* block = &state->v2.blockWorkspace.blocks[i];
        const uint8_t channel = block->channel;
        if (channel < PULSE_PROCESSOR_N_BASE_STATIONS) {
            pulseProcessorV2SweepBlock_t* previousBlock = &state->v2.blocks[channel];
            if (isBlockPairGood(block, previousBlock)) {
                calculateAngles(block, previousBlock, angles);

                *baseStation = channel;
                *axis = sweepDirection_y;
                angles->measurementType = lighthouseBsTypeV2;

                anglesMeasured = true;
            } else {
                memcpy(previousBlock, block, sizeof(pulseProcessorV2SweepBlock_t));
            }
        }
    }

    return anglesMeasured;
}

bool pulseProcessorV2ProcessPulse(pulseProcessor_t *state, const pulseProcessorFrame_t* frameData, pulseProcessorResult_t* angles, int *baseStation, int *axis) {
    handleCalibrationData(state, frameData);
    return handleAngles(state, frameData, angles, baseStation, axis);
}
