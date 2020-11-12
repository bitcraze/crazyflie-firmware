#include "lighthouse_geometry.h"

#include <stdlib.h>
#include <string.h>
#include "unity.h"

#include "mock_cfassert.h"


// Build the arm dsp math lib and use the "real thing" instead of mocking calls to it
// @BUILD_LIB ARM_DSP_MATH

void setUp(void) {
}

void testThatBaseStationPositionIsExtracted() {
  // Fixture
  vec3d actual;
  baseStationGeometry_t baseStationGeometry = {.origin = {1.0,  1.0,  3.0, }};

  vec3d expected = {1.0,  1.0,  3.0};

  // Test
  lighthouseGeometryGetBaseStationPosition(&baseStationGeometry, actual);

  // Assert
  TEST_ASSERT_EQUAL_FLOAT_ARRAY(expected, actual, vec3d_size);
}

void testThatRayIsCalculatedFromNonRotatedBaseStation() {
  // Fixture
  baseStationGeometry_t bsGeo = {.mat = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};

  float angle1 = 0.0f;
  float angle2 = 0.0f;
  vec3d actual;

  vec3d expected = {1.0,  0.0,  0.0};

  // Test
  lighthouseGeometryGetRay(&bsGeo, angle1, angle2, actual);

  // Assert
  TEST_ASSERT_EQUAL_FLOAT_ARRAY(expected, actual, vec3d_size);
}

void testThatRayIsCalculatedFromRotatedBaseStation() {
  // Fixture

  // Rotation about Y-axis
  float rotY = 0.1;
  baseStationGeometry_t bsGeo = {.mat = {{cos(rotY), 0, sin(rotY)}, {0, 1, 0}, {-sin(rotY), 0, cos(rotY)}}};

  float angle1 = 0.0f;
  float angle2 = 0.0f;
  vec3d actual;

  vec3d expected = {cos(rotY),  0.0, -sin(rotY)};

  // Test
  lighthouseGeometryGetRay(&bsGeo, angle1, angle2, actual);

  // Assert
  TEST_ASSERT_EQUAL_FLOAT_ARRAY(expected, actual, vec3d_size);
}

void testThatRayIsCalculatedFromNonRotatedBaseStationWithHorizontalSweepAngle() {
  // Fixture
  baseStationGeometry_t bsGeo = {.mat = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};

  float angle1 = 0.1f;
  float angle2 = 0.0f;
  vec3d actual;

  vec3d expected = {cos(angle1),  sin(angle1),  0.0};

  // Test
  lighthouseGeometryGetRay(&bsGeo, angle1, angle2, actual);

  // Assert
  TEST_ASSERT_EQUAL_FLOAT_ARRAY(expected, actual, vec3d_size);
}

void testThatRayIsCalculatedFromNonRotatedBaseStationWithVerticalSweepAngle() {
  // Fixture
  baseStationGeometry_t bsGeo = {.mat = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};

  float angle1 = 0.0f;
  float angle2 = 0.1f;
  vec3d actual;

  vec3d expected = {cos(angle2),  0.0,  sin(angle2)};

  // Test
  lighthouseGeometryGetRay(&bsGeo, angle1, angle2, actual);

  // Assert
  TEST_ASSERT_EQUAL_FLOAT_ARRAY(expected, actual, vec3d_size);
}

void testThatIntersectionPointIsFoundForLinePerpendicularToPlane() {
  // Fixture
  vec3d linePoint = {1, 1, 2};
  vec3d lineVec = {0, 0, -1};
  vec3d planePoint = {0, 0, 1};
  vec3d planeNormal = {0, 0, 1};

  vec3d actualIntersecionPoint;

  vec3d expected = {1, 1, 1};

  // Test
  bool actualFound = lighthouseGeometryIntersectionPlaneVector(linePoint, lineVec, planePoint, planeNormal, actualIntersecionPoint);

  // Assert
  TEST_ASSERT_TRUE(actualFound);
  TEST_ASSERT_EQUAL_FLOAT_ARRAY(expected, actualIntersecionPoint, vec3d_size);
}

void testThatNoIntersectionPointIsFoundForLineParallelToPlane() {
  // Fixture
  vec3d linePoint = {1, 1, 2};
  vec3d lineVec = {1, 1, 0};
  vec3d planePoint = {0, 0, 1};
  vec3d planeNormal = {0, 0, 1};

  vec3d actualIntersecionPoint;

  // Test
  bool actualFound = lighthouseGeometryIntersectionPlaneVector(linePoint, lineVec, planePoint, planeNormal, actualIntersecionPoint);

  // Assert
  TEST_ASSERT_FALSE(actualFound);
}

void testThatSensorPositionIsTranslated() {
  // Fixture
  vec3d cfPos = {1, 2, 3};
  vec3d sensorPos = {-0.015, -0.0075, 0.0};

  float r[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  arm_matrix_instance_f32 R = {3, 3, (float*)r};

  // Sensor 1 is in the right back corner of the deck
  vec3d expected = {1 - 0.015, 2 - 0.0075, 3};

  vec3d actual;

  // Test
  lighthouseGeometryGetSensorPosition(cfPos, &R, sensorPos, actual);

  // Assert
  TEST_ASSERT_EQUAL_FLOAT_ARRAY(expected, actual, vec3d_size);
}

void testThatSensorPositionIsRotated() {
  // Fixture
  vec3d cfPos = {1, 2, 3};
  vec3d sensorPos = {-0.015, -0.0075, 0.0};

  // Rotate 90 degrees about the Y-axis
  float r[3][3] = {{0, 0, 1}, {0, 1, 0}, {-1, 0, 0}};
  arm_matrix_instance_f32 R = {3, 3, (float*)r};

  // Sensor 1 is in the right back corner of the deck
  vec3d expected = {1, 2 - 0.0075, 3 + 0.015};

  vec3d actual;

  // Test
  lighthouseGeometryGetSensorPosition(cfPos, &R, sensorPos, actual);

  // Assert
  TEST_ASSERT_EQUAL_FLOAT_ARRAY(expected, actual, vec3d_size);
}

void testThatYawDeltaIsCalculatedForZeroAngle() {
  // Fixture
  vec3d ipv = {0, 1, 1};
  vec3d spv = {0, 2, 2};
  vec3d n = {1, 0, 0};

  float actualAngle;
  float expected = 0.0f;

  // Test
  float actualResult = lighthouseGeometryYawDelta(ipv, spv, n, &actualAngle);

  // Assert
  TEST_ASSERT_TRUE(actualResult);
  TEST_ASSERT_EQUAL_FLOAT(expected, actualAngle);
}

void testThatYawDeltaIsCalculatedForPiAngle() {
  // Fixture
  vec3d ipv = {0, 1, 1};
  vec3d spv = {0, -2, -2};
  vec3d n = {1, 0, 0};

  float actualAngle;
  float expected = PI;

  // Test
  float actualResult = lighthouseGeometryYawDelta(ipv, spv, n, &actualAngle);

  // Assert
  TEST_ASSERT_TRUE(actualResult);
  TEST_ASSERT_EQUAL_FLOAT(expected, actualAngle);
}

void testThatYawDeltaIsCalculatedForPerpendicularVectors() {
  // Fixture
  vec3d ipv = {0, 1, 1};
  vec3d spv = {0, 2, -2};
  vec3d n = {1, 0, 0};

  float actualAngle;
  float expected = -PI / 2;

  // Test
  float actualResult = lighthouseGeometryYawDelta(ipv, spv, n, &actualAngle);

  // Assert
  TEST_ASSERT_TRUE(actualResult);
  TEST_ASSERT_EQUAL_FLOAT(expected, actualAngle);
}

void testThatYawDeltaIsCalculatedForPerpendicularVectorsWithNegativeAngle() {
  // Fixture
  vec3d ipv = {0, 1, -1};
  vec3d spv = {0, 2, 2};
  vec3d n = {1, 0, 0};

  float actualAngle;
  float expected = PI / 2;

  // Test
  float actualResult = lighthouseGeometryYawDelta(ipv, spv, n, &actualAngle);

  // Assert
  TEST_ASSERT_TRUE(actualResult);
  TEST_ASSERT_EQUAL_FLOAT(expected, actualAngle);
}

void testThatYawDeltaIsNotCalculatedForSmallIntersectionPointVector() {
  // Fixture
  vec3d ipv = {0, 0.00001, -0.00001};
  vec3d spv = {0, 2, 2};
  vec3d n = {1, 0, 0};

  float actualAngle;

  // Test
  float actualResult = lighthouseGeometryYawDelta(ipv, spv, n, &actualAngle);

  // Assert
  TEST_ASSERT_FALSE(actualResult);
}
