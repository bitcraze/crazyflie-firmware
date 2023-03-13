/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2022 Bitcraze AB
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
 * Unit tests for buf2buf
 */

// Module under test
#include "buf2buf.h"

#include "unity.h"
#include <string.h>

static Buf2bufContext_t context;

#define IN_BUF_SIZE 100
static uint8_t inBuf[IN_BUF_SIZE];

#define OUT_BUF_SIZE 100
static uint8_t outBuf[OUT_BUF_SIZE];

// Helpers

static void validateStdUseCase(uint32_t inBufSize, uint32_t nrOfInBufs, uint32_t outBufSize);

void setUp(void) {
  for (int i = 0; i < IN_BUF_SIZE; i++) {
    inBuf[i] = i + 1;
  }

  memset(outBuf, 0, sizeof(outBuf));
  memset(&context, 0, sizeof(context));
}

void tearDown(void) {}

void testThatASmallInBufferIsNotFillingLargeOutBuffer() {
  // Fixture
  buf2bufInit(&context, outBuf, 10);

  // Test
  buf2bufAddInBuf(&context, inBuf, 4);
  bool actual = buf2bufConsumeInBuf(&context);

  // Assert
  TEST_ASSERT_FALSE(actual);
}

void testThatEqualSizeInAndOutBufferFillsOutBuffer() {
  // Fixture
  buf2bufInit(&context, outBuf, 10);

  // Test
  buf2bufAddInBuf(&context, inBuf, 10);
  bool actual = buf2bufConsumeInBuf(&context);

  // Assert
  TEST_ASSERT_TRUE(actual);
}

void testThatALargeInBufferFillsSmallerOutBuffer() {
  // Fixture
  buf2bufInit(&context, outBuf, 10);

  // Test
  buf2bufAddInBuf(&context, inBuf, 15);
  bool actual = buf2bufConsumeInBuf(&context);

  // Assert
  TEST_ASSERT_TRUE(actual);
}

void testThatMultipleSmallInBufferFillsOutBuffer() {
  // Fixture
  buf2bufInit(&context, outBuf, 10);

  // Test
  buf2bufAddInBuf(&context, inBuf, 7);
  bool actual1 = buf2bufConsumeInBuf(&context);

  buf2bufAddInBuf(&context, inBuf, 7);
  bool actual2 = buf2bufConsumeInBuf(&context);

  // Assert
  TEST_ASSERT_FALSE(actual1);
  TEST_ASSERT_TRUE(actual2);
}

void testThatOutBufferWithDataReportsSizeWhenReleased() {
  // Fixture
  buf2bufInit(&context, outBuf, 10);

  buf2bufAddInBuf(&context, inBuf, 4);
  buf2bufConsumeInBuf(&context);

  buf2bufAddInBuf(&context, inBuf, 4);
  buf2bufConsumeInBuf(&context);

  // Test
  uint32_t actual = buf2bufReleaseOutBuf(&context);

  // Assert
  TEST_ASSERT_EQUAL_UINT32(8, actual);
}

void testStdUseCaseWithVariousConfigurations() {
  // Same buffer size
  validateStdUseCase(5, 5, 5);

  // Smaller in buffer than out buffer
  validateStdUseCase(5, 10, 7);

  // Our buffer multiple size of in buffer
  validateStdUseCase(4, 5, 8);

  // In buffer multiple size of out buffer
  validateStdUseCase(8, 5, 4);

  // Larger in buffer than out buffer
  validateStdUseCase(7, 10, 5);

  // Last in buffer overflows into new out buffer
  validateStdUseCase(5, 2, 7);

  // Last in buffer fits in out buffer
  validateStdUseCase(5, 3, 8);
}

// Helpers -------------------------------

static void validateStdUseCase(uint32_t inBufSize, uint32_t nrOfInBufs, uint32_t outBufSize) {
  TEST_ASSERT_LESS_OR_EQUAL(IN_BUF_SIZE, inBufSize);
  TEST_ASSERT_LESS_OR_EQUAL(OUT_BUF_SIZE, outBufSize);

  // Buffers for concatenating all in and out buffers, will be compared at the end
  uint8_t concatInBufs[inBufSize * nrOfInBufs];
  uint8_t concatOutBufs[inBufSize * nrOfInBufs];

  uint32_t inIndex = 0;
  uint32_t outIndex = 0;

  buf2bufInit(&context, outBuf, outBufSize);

  // Simulate a bunch of in buffers
  for (uint32_t i = 0; i < nrOfInBufs; i++) {
    memcpy(&concatInBufs[inIndex], inBuf, inBufSize);
    inIndex += inBufSize;

    buf2bufAddInBuf(&context, inBuf, inBufSize);
    while(buf2bufConsumeInBuf(&context)) {
      // Store out buffers
      memcpy(&concatOutBufs[outIndex], outBuf, outBufSize);
      outIndex += outBufSize;
    }
    buf2bufReleaseInBuf(&context);
  }

  // Store the last potential out buffer
  uint32_t size = buf2bufReleaseOutBuf(&context);
  memcpy(&concatOutBufs[outIndex], outBuf, size);
  outIndex += size;

  TEST_ASSERT_EQUAL_UINT32(inIndex, outIndex);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(concatInBufs, concatOutBufs, inIndex);
}
