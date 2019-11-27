// File under test outlierFilter.h
#include "outlierFilter.h"

#include "unity.h"

#include "mock_cfassert.h"

static tdoaMeasurement_t tdoa;

// Helpers
uint32_t fixtureCloseLhFilter(OutlierFilterLhState_t* this);
uint32_t fixtureOpenLhFilter(OutlierFilterLhState_t* this);


void setUp(void) {
  // TDoA filter
  tdoa.anchorPosition[0].x = 1.0;
  tdoa.anchorPosition[0].y = 1.0;
  tdoa.anchorPosition[0].z = 1.0;

  tdoa.anchorPosition[1].x = 5.0;
  tdoa.anchorPosition[1].y = 1.0;
  tdoa.anchorPosition[1].z = 1.0;

  tdoa.distanceDiff = 0.0;
}

void tearDown(void) {
  // Empty
}


void testThatSamplesAreAcceptedWhenTdoaIsCloserThanDistanceBetweenAnchors() {
  // Fixture
  tdoa.distanceDiff = 1.0;
  bool expected = true;

  // Test
  bool actual = outlierFilterValidateTdoaSimple(&tdoa);

  // Assert
  TEST_ASSERT_EQUAL(expected, actual);
}


void testThatSamplesAreRejectedWhenTdoaIsGreaterThanDistanceBetweenAnchors() {
  // Fixture
  tdoa.distanceDiff = 10.0;
  bool expected = false;

  // Test
  bool actual = outlierFilterValidateTdoaSimple(&tdoa);

  // Assert
  TEST_ASSERT_EQUAL(expected, actual);
}


void testThatSamplesAreRejectedWhenTdoaIsGreaterButNegativeThanDistanceBetweenAnchors() {
  // Fixture
  tdoa.distanceDiff = -10.0;
  bool expected = false;

  // Test
  bool actual = outlierFilterValidateTdoaSimple(&tdoa);

  // Assert
  TEST_ASSERT_EQUAL(expected, actual);
}


// Lighthouse filter tests ----------------------------------------------------------

#define LH_DISTANCE 4
#define LH_BAD_ANGLE 1
#define LH_GOOD_ANGLE 0.0001
#define LH_TIME_STEP (1000 / 120)

// TOOD krri remove
static void print(OutlierFilterLhState_t* this, uint32_t time) {
  printf("win:%i openingTime:%i time:%i ", this->openingWindow, this->openingTime, time);
  bool isFilterClosed = (time < this->openingTime);
  if (isFilterClosed) {
    printf("Is closed\n");
  } else {
    printf("Is open\n");
  }
}


void testThatLhFilterLetsGoodSampleThroughWhenOpen() {
  // Fixture
  OutlierFilterLhState_t this;
  uint32_t time = fixtureOpenLhFilter(&this);
  bool expected = true;

  // Test
  bool actual = outlierFilterValidateLighthouseSweep(&this, LH_DISTANCE, LH_GOOD_ANGLE, time);

  // Assert
  TEST_ASSERT_EQUAL(expected, actual);
}

void testThatLhFilterLetsBadSampleThroughWhenOpen() {
  // Fixture
  OutlierFilterLhState_t this;
  uint32_t time = fixtureOpenLhFilter(&this);
  bool expected = true;

  // Test
  bool actual = outlierFilterValidateLighthouseSweep(&this, LH_DISTANCE, LH_BAD_ANGLE, time);

  // Assert
  TEST_ASSERT_EQUAL(expected, actual);
}

void testThatLhFilterLetsGoodSampleThroughWhenClosed() {
  // Fixture
  OutlierFilterLhState_t this;
  uint32_t time = fixtureCloseLhFilter(&this);
  bool expected = true;

  // Test
  bool actual = outlierFilterValidateLighthouseSweep(&this, LH_DISTANCE, LH_GOOD_ANGLE, time);

  // Assert
  TEST_ASSERT_EQUAL(expected, actual);
}

void testThatLhFilterBlocksBadSampleWhenClosed() {
  // Fixture
  OutlierFilterLhState_t this;
  uint32_t time = fixtureCloseLhFilter(&this);
  bool expected = false;

  // Test
  bool actual = outlierFilterValidateLighthouseSweep(&this, LH_DISTANCE, LH_BAD_ANGLE, time);

  // Assert
  TEST_ASSERT_EQUAL(expected, actual);
}

void testThatLhFilterOpensForManyBadSamples() {
  // Fixture
  OutlierFilterLhState_t this;
  uint32_t time = fixtureCloseLhFilter(&this);

  // Test, Assert
  for (int i = 0; i < 10; i++) {
    bool actual = outlierFilterValidateLighthouseSweep(&this, LH_DISTANCE, LH_BAD_ANGLE, time);
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
  bool actual = outlierFilterValidateLighthouseSweep(&this, LH_DISTANCE, LH_BAD_ANGLE, newTime);

  // Assert
  TEST_ASSERT_EQUAL(expected, actual);
}

void testThatLhFilterClosesAfterManyGoodSamples() {
  // Fixture
  OutlierFilterLhState_t this;
  uint32_t time = fixtureOpenLhFilter(&this);

  // Test
  for (int i = 0; i < 7; i++) {
    outlierFilterValidateLighthouseSweep(&this, LH_DISTANCE, LH_GOOD_ANGLE, time);
    time += LH_TIME_STEP;
  }

  // Assert
  bool expected = false;
  bool actual = outlierFilterValidateLighthouseSweep(&this, LH_DISTANCE, LH_BAD_ANGLE, time);
  TEST_ASSERT_EQUAL(expected, actual);
}

void testThatLhFilterOpensWithMixedSamples() {
  // Fixture
  OutlierFilterLhState_t this;
  uint32_t time = fixtureCloseLhFilter(&this);

  // Test
  for (int i = 0; i < 7; i++) {
    outlierFilterValidateLighthouseSweep(&this, LH_DISTANCE, LH_GOOD_ANGLE, time);
    outlierFilterValidateLighthouseSweep(&this, LH_DISTANCE, LH_BAD_ANGLE, time);
    time += LH_TIME_STEP;
  }

  // Assert
  bool expected = true;
  bool actual = outlierFilterValidateLighthouseSweep(&this, LH_DISTANCE, LH_BAD_ANGLE, time);
  TEST_ASSERT_EQUAL(expected, actual);
}


// Helpers /////////////////////////////////////////////////////////////////////////////////
uint32_t fixtureCloseLhFilter(OutlierFilterLhState_t* this) {
  uint32_t time = 1000;

  outlierFilterReset(this, time);
  time += LH_TIME_STEP;

  for (int i = 0; i < 20; i++) {
    outlierFilterValidateLighthouseSweep(this, LH_DISTANCE, LH_GOOD_ANGLE, time);
    time += LH_TIME_STEP;
  }

  return time;
}

uint32_t fixtureOpenLhFilter(OutlierFilterLhState_t* this) {
  uint32_t time = 1000;

  outlierFilterReset(this, time);
  time += LH_TIME_STEP;

  return time;
}
