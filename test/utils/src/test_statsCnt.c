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

static const uint32_t resetTime = 1234;
static statsCntRate_t data;

void setUp(void) {
  statsCntReset(&data, resetTime);
}

void tearDown(void) {}

void testThatDataIsReset() {
  // Fixture
  data.count = 17;
  data.result = 47.11f;
  data.latestAverage_ms = 9876;

  // Test
  statsCntReset(&data, resetTime);

  // Assert
  TEST_ASSERT_EQUAL_UINT32(0, data.count);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, data.result);
  TEST_ASSERT_EQUAL_UINT32(resetTime, data.latestAverage_ms);
}

void testThatCounterIsIncreased() {
  // Fixture

  // Test
  statsCntInc(&data);

  // Assert
  TEST_ASSERT_EQUAL_UINT32(1, data.count);
}

void testThatRateIsComputed() {
  // Fixture
  int count = 10;
  for (int i = 0; i < count; i++) {
      statsCntInc(&data);
  }

  uint32_t dt_ms = 500;
  uint32_t now = resetTime + dt_ms;

  float expected = (float)count * 1000.0f / (float)dt_ms;

  // Test
  float actual = statsCntRate(&data, now);

  // Assert
  TEST_ASSERT_EQUAL_FLOAT(expected, actual);
  TEST_ASSERT_EQUAL_FLOAT(expected, data.result);
}

void testThatRateIsComputedWhenNoTimeHasPassed() {
  // Fixture
  int count = 10;
  for (int i = 0; i < count; i++) {
      statsCntInc(&data);
  }

  data.result = 47.11f;

  uint32_t now = resetTime;
  float expected = 0.0f;

  // Test
  float actual = statsCntRate(&data, now);

  // Assert
  TEST_ASSERT_EQUAL_FLOAT(expected, actual);
  TEST_ASSERT_EQUAL_FLOAT(expected, data.result);
}

void testThatCountersAreResetWhenRateIsComputed() {
  // Fixture
  statsCntInc(&data);

  uint32_t dt_ms = 500;
  uint32_t now = resetTime + dt_ms;

  // Test
  statsCntRate(&data, now);

  // Assert
  TEST_ASSERT_EQUAL_UINT32(0, data.count);
  TEST_ASSERT_EQUAL_UINT32(now, data.latestAverage_ms);
}

