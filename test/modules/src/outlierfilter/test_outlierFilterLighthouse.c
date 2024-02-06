// File under test outlierFilterLighthouse.c
#include "outlierFilterLighthouse.h"

#include "unity.h"

// Helpers
uint32_t fixtureCloseLhFilter(OutlierFilterLhState_t* this);
uint32_t fixtureOpenLhFilter(OutlierFilterLhState_t* this);


void setUp(void) {
  // Empty
}

void tearDown(void) {
  // Empty
}


#define LH_DISTANCE 4
#define LH_BAD_ANGLE 1
#define LH_GOOD_ANGLE 0.0001
#define LH_TIME_STEP (1000 / 120)

void testThatLhFilterLetsGoodSampleThroughWhenOpen() {
  // Fixture
  OutlierFilterLhState_t this;
  uint32_t time = fixtureOpenLhFilter(&this);
  bool expected = true;

  // Test
  bool actual = outlierFilterLighthouseValidateSweep(&this, LH_DISTANCE, LH_GOOD_ANGLE, time);

  // Assert
  TEST_ASSERT_EQUAL(expected, actual);
}

void testThatLhFilterLetsBadSampleThroughWhenOpen() {
  // Fixture
  OutlierFilterLhState_t this;
  uint32_t time = fixtureOpenLhFilter(&this);
  bool expected = true;

  // Test
  bool actual = outlierFilterLighthouseValidateSweep(&this, LH_DISTANCE, LH_BAD_ANGLE, time);

  // Assert
  TEST_ASSERT_EQUAL(expected, actual);
}

void testThatLhFilterLetsGoodSampleThroughWhenClosed() {
  // Fixture
  OutlierFilterLhState_t this;
  uint32_t time = fixtureCloseLhFilter(&this);
  bool expected = true;

  // Test
  bool actual = outlierFilterLighthouseValidateSweep(&this, LH_DISTANCE, LH_GOOD_ANGLE, time);

  // Assert
  TEST_ASSERT_EQUAL(expected, actual);
}

void testThatLhFilterBlocksBadSampleWhenClosed() {
  // Fixture
  OutlierFilterLhState_t this;
  uint32_t time = fixtureCloseLhFilter(&this);
  bool expected = false;

  // Test
  bool actual = outlierFilterLighthouseValidateSweep(&this, LH_DISTANCE, LH_BAD_ANGLE, time);

  // Assert
  TEST_ASSERT_EQUAL(expected, actual);
}

void testThatLhFilterOpensForManyBadSamples() {
  // Fixture
  OutlierFilterLhState_t this;
  uint32_t time = fixtureCloseLhFilter(&this);

  // Test, Assert
  for (int i = 0; i < 10; i++) {
    bool actual = outlierFilterLighthouseValidateSweep(&this, LH_DISTANCE, LH_BAD_ANGLE, time);
    time += LH_TIME_STEP;

    // Should block the first 4 samples and let the rest through
    bool expected = (i >= 4);
    TEST_ASSERT_EQUAL(expected, actual);
  }
}

void testThatLhFilterOpensAfterInactivity() {
  // Fixture
  OutlierFilterLhState_t this;
  uint32_t time = fixtureCloseLhFilter(&this);
  uint32_t newTime = time + LH_TIME_STEP * 200;
  bool expected = true;

  // Test
  bool actual = outlierFilterLighthouseValidateSweep(&this, LH_DISTANCE, LH_BAD_ANGLE, newTime);

  // Assert
  TEST_ASSERT_EQUAL(expected, actual);
}

void testThatLhFilterClosesAfterManyGoodSamples() {
  // Fixture
  OutlierFilterLhState_t this;
  uint32_t time = fixtureOpenLhFilter(&this);

  // Test
  for (int i = 0; i < 7; i++) {
    outlierFilterLighthouseValidateSweep(&this, LH_DISTANCE, LH_GOOD_ANGLE, time);
    time += LH_TIME_STEP;
  }

  // Assert
  bool expected = false;
  bool actual = outlierFilterLighthouseValidateSweep(&this, LH_DISTANCE, LH_BAD_ANGLE, time);
  TEST_ASSERT_EQUAL(expected, actual);
}

void testThatLhFilterOpensWithMixedSamples() {
  // Fixture
  OutlierFilterLhState_t this;
  uint32_t time = fixtureCloseLhFilter(&this);

  // Test
  for (int i = 0; i < 7; i++) {
    outlierFilterLighthouseValidateSweep(&this, LH_DISTANCE, LH_GOOD_ANGLE, time);
    outlierFilterLighthouseValidateSweep(&this, LH_DISTANCE, LH_BAD_ANGLE, time);
    time += LH_TIME_STEP;
  }

  // Assert
  bool expected = true;
  bool actual = outlierFilterLighthouseValidateSweep(&this, LH_DISTANCE, LH_BAD_ANGLE, time);
  TEST_ASSERT_EQUAL(expected, actual);
}


// Helpers /////////////////////////////////////////////////////////////////////////////////
uint32_t fixtureCloseLhFilter(OutlierFilterLhState_t* this) {
  uint32_t time = 1000;

  outlierFilterLighthouseReset(this, time);
  time += LH_TIME_STEP;

  for (int i = 0; i < 20; i++) {
    outlierFilterLighthouseValidateSweep(this, LH_DISTANCE, LH_GOOD_ANGLE, time);
    time += LH_TIME_STEP;
  }

  return time;
}

uint32_t fixtureOpenLhFilter(OutlierFilterLhState_t* this) {
  uint32_t time = 1000;

  outlierFilterLighthouseReset(this, time);
  time += LH_TIME_STEP;

  return time;
}
