// File under test outlierFilter.h
#include "outlierFilter.h"

#include "unity.h"

#include "mock_cfassert.h"

static tdoaMeasurement_t tdoa;

void setUp(void) {
  outlierFilterReset();

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
  TEST_ASSERT_EQUAL(actual, expected);
}


void testThatSamplesAreRejectedWhenTdoaIsGreaterThanDistanceBetweenAnchors() {
  // Fixture
  tdoa.distanceDiff = 10.0;
  bool expected = false;

  // Test
  bool actual = outlierFilterValidateTdoaSimple(&tdoa);

  // Assert
  TEST_ASSERT_EQUAL(actual, expected);
}


void testThatSamplesAreRejectedWhenTdoaIsGreaterButNegativeThanDistanceBetweenAnchors() {
  // Fixture
  tdoa.distanceDiff = -10.0;
  bool expected = false;

  // Test
  bool actual = outlierFilterValidateTdoaSimple(&tdoa);

  // Assert
  TEST_ASSERT_EQUAL(actual, expected);
}
