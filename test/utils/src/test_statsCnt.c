/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
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
 * testStatsCnt.h - unit tests for statsCnt
 */

// File under test
#include "statsCnt.h"

#include "unity.h"


// Helpers
void assertRateCounterIsInitialized(statsCntRateCounter_t* sut);
void assertLoggerIsInitialized(statsCntRateLogger_t* sut);

void testThatRateCounterIsInitialized() {
  // Fixture
  statsCntRateCounter_t sut = {
    .count = 1,
    .latestCount = 2,
    .latestAveragingMs = 3,
    .latestRate = 4.0f,
    .intervalMs = 5,
  };

  // Test
  statsCntRateCounterInit(&sut, 4711);

  // Assert
  assertRateCounterIsInitialized(&sut);
}

void testThatRateIsComputedWhenTimeSinceLastComputationIsLongerThanTheInterval() {
  // Fixture
  uint32_t interval = 500;

  statsCntRateCounter_t sut;
  statsCntRateCounterInit(&sut, interval);

  uint32_t firstCount = 100;
  sut.count = firstCount;

  uint32_t firstUpdate_ms = 1000;
  statsCntRateCounterUpdate(&sut, firstUpdate_ms);

  uint32_t dCount = 100;
  sut.count = firstCount + dCount;

  // Make sure dt is longer than the interval
  uint32_t dt_ms = interval * 2;
  uint32_t now = firstUpdate_ms + dt_ms;

  float expected = dCount * 1000.0f / dt_ms;

  // Test
  float actual = statsCntRateCounterUpdate(&sut, now);

  // Assert
  TEST_ASSERT_EQUAL_FLOAT(expected, actual);
  TEST_ASSERT_EQUAL_FLOAT(expected, sut.latestRate);
}

void testThatRateNotIsComputedWhenTimeSinceLastComputationIsLessThanTheInterval() {
  // Fixture
  uint32_t interval = 500;

  statsCntRateCounter_t sut;
  statsCntRateCounterInit(&sut, interval);

  uint32_t firstCount = 100;
  sut.count = firstCount;

  uint32_t firstUpdate_ms = 1000;
  float expected = statsCntRateCounterUpdate(&sut, firstUpdate_ms);

  uint32_t dCount = 100;
  sut.count = firstCount + dCount;

  // Make sure dt is less than the interval
  uint32_t dt_ms = interval - 1;
  uint32_t now = firstUpdate_ms + dt_ms;

  // Test
  float actual = statsCntRateCounterUpdate(&sut, now);

  // Assert
  TEST_ASSERT_EQUAL_FLOAT(expected, actual);
  TEST_ASSERT_EQUAL_FLOAT(expected, sut.latestRate);
}

void testThatRateLoggerIsInitialized() {
  // Fixture
  statsCntRateLogger_t sut = {
    .logByFunction = {.data = 0, .aquireFloat = 0},
    .rateCounter = {.latestAveragingMs = 3},
  };

  // Test
  STATS_CNT_RATE_INIT(&sut, 4711);

  // Assert
  assertLoggerIsInitialized(&sut);
}

void testThatRateLoggerIsInitializedInline() {
  // Fixture
  // Test
  STATS_CNT_RATE_DEFINE(sut, 4711);

  // Assert
  assertLoggerIsInitialized(&sut);
}

void testThatStatsCntRateLoggerCanBeCastToLogByFunction() {
  // Fixture
  statsCntRateLogger_t sut = {
    .logByFunction = {.data = (void*)47, .aquireFloat = (logAcquireFloat)11},
  };

  // Test
  logByFunction_t* actual = (logByFunction_t*)&sut;

  // Assert
  TEST_ASSERT_EQUAL_PTR(47, actual->data);
  TEST_ASSERT_EQUAL_PTR(11, actual->aquireFloat);
}

void testThatCounterIsIncreased() {
  // Fixture
  statsCntRateLogger_t sut;
  STATS_CNT_RATE_INIT(&sut, 4711);

  // Test
  STATS_CNT_RATE_EVENT(&sut);

  // Assert
  TEST_ASSERT_EQUAL_UINT32(1, sut.rateCounter.count);
}


// Helpers

void assertRateCounterIsInitialized(statsCntRateCounter_t* sut) {
  TEST_ASSERT_EQUAL_UINT32(4711, sut->intervalMs);
  TEST_ASSERT_EQUAL_UINT32(0, sut->count);
  TEST_ASSERT_EQUAL_UINT32(0, sut->latestCount);
  TEST_ASSERT_EQUAL_UINT32(0, sut->latestAveragingMs);
  TEST_ASSERT_EQUAL_FLOAT(0.0, sut->latestRate);
}

void assertLoggerIsInitialized(statsCntRateLogger_t* sut) {
  TEST_ASSERT_EQUAL_PTR(sut, sut->logByFunction.data);
  TEST_ASSERT_EQUAL_PTR(statsCntRateLogHandler, sut->logByFunction.aquireFloat);

  assertRateCounterIsInitialized(&sut->rateCounter);
}
