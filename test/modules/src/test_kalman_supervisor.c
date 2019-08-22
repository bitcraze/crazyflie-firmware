// File under test kalman_supervisor.c
#include "kalman_supervisor.h"

#include "unity.h"

kalmanCoreData_t coreData;

void setUp(void) {
  memset(&coreData, 0, sizeof(coreData));
}

void tearDown(void) {
  // Empty
}


void testThatStationaryInOriginIsAccepted() {
  // Fixture
  bool expected = true;

  // Test
  bool actual = kalmanSupervisorIsStateWithinBounds(&coreData);

  // Assert
  TEST_ASSERT_EQUAL(actual, expected);
}

void testThatPositionOutOfBoundsPositiveIsNotAccepted() {
  // Fixture
  bool expected = false;
  coreData.S[KC_STATE_X] = 200;

  // Test
  bool actual = kalmanSupervisorIsStateWithinBounds(&coreData);

  // Assert
  TEST_ASSERT_EQUAL(actual, expected);
}

void testThatPositionOutOfBoundsNegativeIsNotAccepted() {
  // Fixture
  bool expected = false;
  coreData.S[KC_STATE_Z] = -200;

  // Test
  bool actual = kalmanSupervisorIsStateWithinBounds(&coreData);

  // Assert
  TEST_ASSERT_EQUAL(actual, expected);
}

void testThatVelocityOutOfBoundsPositiveIsNotAccepted() {
  // Fixture
  bool expected = false;
  coreData.S[KC_STATE_PX] = 20;

  // Test
  bool actual = kalmanSupervisorIsStateWithinBounds(&coreData);

  // Assert
  TEST_ASSERT_EQUAL(actual, expected);
}

void testThatVelocityOutOfBoundsNegativeIsNotAccepted() {
  // Fixture
  bool expected = false;
  coreData.S[KC_STATE_PZ] = -20;

  // Test
  bool actual = kalmanSupervisorIsStateWithinBounds(&coreData);

  // Assert
  TEST_ASSERT_EQUAL(actual, expected);
}
