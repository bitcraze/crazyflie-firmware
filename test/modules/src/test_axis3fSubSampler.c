// File under test axis3fSubSampler.c
#include "axis3fSubSampler.h"

#include "unity.h"

Axis3fSubSampler_t subSampler;

const Axis3f sample1 = {.x=1.0, .y=2.0, .z=3.0};
const Axis3f sample2 = {.x=4.0, .y=5.0, .z=6.0};
const Axis3f sample3 = {.x=7.0, .y=8.0, .z=9.0};

void setUp(void) {
}

void tearDown(void) {
  // Empty
}


void testThatOneSampleIsUnchanged() {
  // Fixture
  axis3fSubSamplerInit(&subSampler, 1.0);

  axis3fSubSamplerAccumulate(&subSampler, &sample1);

  // Test
  Axis3f* actual = axis3fSubSamplerFinalize(&subSampler);

  // Assert
  TEST_ASSERT_EQUAL_FLOAT(sample1.x, actual->x);
  TEST_ASSERT_EQUAL_FLOAT(sample1.y, actual->y);
  TEST_ASSERT_EQUAL_FLOAT(sample1.z, actual->z);
}


void testThatSamplesAreAveraged() {
  // Fixture
  axis3fSubSamplerInit(&subSampler, 1.0);

  axis3fSubSamplerAccumulate(&subSampler, &sample1);
  axis3fSubSamplerAccumulate(&subSampler, &sample2);
  axis3fSubSamplerAccumulate(&subSampler, &sample3);

  // Test
  Axis3f* actual = axis3fSubSamplerFinalize(&subSampler);

  // Assert
  TEST_ASSERT_EQUAL_FLOAT(4.0, actual->x);
  TEST_ASSERT_EQUAL_FLOAT(5.0, actual->y);
  TEST_ASSERT_EQUAL_FLOAT(6.0, actual->z);
}


void testThatResultIsUnchangedWhenNoNewSamplesAreAccumulated() {
  // Fixture
  axis3fSubSamplerInit(&subSampler, 1.0);
  axis3fSubSamplerAccumulate(&subSampler, &sample1);
  axis3fSubSamplerFinalize(&subSampler);
  // The sub sampler has now been finalized

  // Test
  Axis3f* actual = axis3fSubSamplerFinalize(&subSampler);

  // Assert
  TEST_ASSERT_EQUAL_FLOAT(sample1.x, actual->x);
  TEST_ASSERT_EQUAL_FLOAT(sample1.y, actual->y);
  TEST_ASSERT_EQUAL_FLOAT(sample1.z, actual->z);
}

void testThatResultIsUpdatedWhenNewSamplesAreAccumulated() {
  // Fixture
  axis3fSubSamplerInit(&subSampler, 1.0);
  axis3fSubSamplerAccumulate(&subSampler, &sample1);
  axis3fSubSamplerFinalize(&subSampler);
  // The sub sampler has now been finalized

  // Test
  axis3fSubSamplerAccumulate(&subSampler, &sample2);
  Axis3f* actual = axis3fSubSamplerFinalize(&subSampler);

  // Assert
  TEST_ASSERT_EQUAL_FLOAT(sample2.x, actual->x);
  TEST_ASSERT_EQUAL_FLOAT(sample2.y, actual->y);
  TEST_ASSERT_EQUAL_FLOAT(sample2.z, actual->z);
}

void testThatResultIsMultipliedWithConversionFactor() {
  // Fixture
  axis3fSubSamplerInit(&subSampler, 3.0);

  axis3fSubSamplerAccumulate(&subSampler, &sample1);

  // Test
  Axis3f* actual = axis3fSubSamplerFinalize(&subSampler);

  // Assert
  TEST_ASSERT_EQUAL_FLOAT(sample1.x * 3.0f, actual->x);
  TEST_ASSERT_EQUAL_FLOAT(sample1.y * 3.0f, actual->y);
  TEST_ASSERT_EQUAL_FLOAT(sample1.z * 3.0f, actual->z);
}
