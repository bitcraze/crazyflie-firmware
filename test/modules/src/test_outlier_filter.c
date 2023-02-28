// File under test outlierFilterTdoa.c
#include "outlierFilterTdoa.h"

#include "unity.h"

static tdoaMeasurement_t tdoa;


void setUp(void) {
  // TDoA filter
  tdoa.anchorPositions[0].x = 1.0;
  tdoa.anchorPositions[0].y = 1.0;
  tdoa.anchorPositions[0].z = 1.0;

  tdoa.anchorPositions[1].x = 5.0;
  tdoa.anchorPositions[1].y = 1.0;
  tdoa.anchorPositions[1].z = 1.0;

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
  bool actual = outlierFilterTdoaValidateSimple(&tdoa);

  // Assert
  TEST_ASSERT_EQUAL(expected, actual);
}


void testThatSamplesAreRejectedWhenTdoaIsGreaterThanDistanceBetweenAnchors() {
  // Fixture
  tdoa.distanceDiff = 10.0;
  bool expected = false;

  // Test
  bool actual = outlierFilterTdoaValidateSimple(&tdoa);

  // Assert
  TEST_ASSERT_EQUAL(expected, actual);
}


void testThatSamplesAreRejectedWhenTdoaIsGreaterButNegativeThanDistanceBetweenAnchors() {
  // Fixture
  tdoa.distanceDiff = -10.0;
  bool expected = false;

  // Test
  bool actual = outlierFilterTdoaValidateSimple(&tdoa);

  // Assert
  TEST_ASSERT_EQUAL(expected, actual);
}
