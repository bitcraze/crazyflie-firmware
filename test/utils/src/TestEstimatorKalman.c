#include "estimatorKalmanEngine.h"
#include "estimatorKalmanStorage.h"

#include "FIFO.h"
#include "math.h"
#include "arm_math.h"

#include "unity.h"

/*
 Control the tick count passed to the filter
 */
static uint32_t msCount = 0;

uint32_t getTickCount() {
  return msCount;
}

/*
 Mock assertions
 */

void assertFail(char *exp, char *file, int line) {
  printf("ASSERTION FAILURE: %s. File: %s. Line: %d\n", exp, file, line);
  TEST_FAIL();
}

/*
 Mock the arm functions for math calculations
 */

float32_t arm_cos_f32(float32_t x) {
  return cosf(x);
}

float32_t arm_sin_f32(float32_t x) {
  return sinf(x);
}

arm_status arm_mat_trans_f32(const arm_matrix_instance_f32 * pSrc, arm_matrix_instance_f32 * pDst) {
  pDst->numRows = pSrc->numCols;
  pDst->numCols = pSrc->numRows;

  for (int i = 0; i < pDst->numRows; i++) {
    for(int j = 0; j < pDst->numCols; j++) {
      pDst->pData[i*pDst->numCols + j] = pSrc->pData[j*pSrc->numCols + i];
    }
  }

  return ARM_MATH_SUCCESS;
}

arm_status arm_mat_mult_f32(const arm_matrix_instance_f32 * pSrcA, const arm_matrix_instance_f32 * pSrcB, arm_matrix_instance_f32 * pDst) {
  pDst->numRows = pSrcA->numRows;
  pDst->numCols = pSrcB->numCols;

  for (int i = 0; i < pSrcA->numRows; i++) {
    for (int j = 0; j < pSrcB->numCols; j++) {
      pDst->pData[i*pDst->numCols + j] = 0;
      for (int k = 0; k < pSrcA->numCols; k++) {
        pDst->pData[i*pDst->numCols + j] += pSrcA->pData[i*pSrcA->numCols + k] * pSrcB->pData[k*pSrcB->numCols + j];
      }
    }
  }

  return ARM_MATH_SUCCESS;
}

/*
 Helper functions
 */

void helper_initStorageWithZeroMeassurements(estimatorKalmanStorage_t* storage) {
  const vec3Measurement_t meassurement = {
    .value = { .x = 0, .y = 0, .z = 0 },
    .stdDev = { .x = 0, .y = 0, .z = 0 }
  };

  estimatorKalmanEngine.init(storage, &meassurement, &meassurement, &meassurement);
}

/*
 Tests
 */

void setUp(void) {
  msCount = 0;
  estimatorKalmanEngine.initializeEngine(getTickCount, 1000);
}

void tearDown(void) {
}

void testInit() {
  estimatorKalmanStorage_t storage;

  TEST_ASSERT_FALSE(storage.isInit);
  helper_initStorageWithZeroMeassurements(&storage);
  TEST_ASSERT_TRUE(storage.isInit);
}

void testEnqueuePosition() {
  estimatorKalmanStorage_t storage;
  helper_initStorageWithZeroMeassurements(&storage);

  TEST_ASSERT_TRUE(fifo_is_empty(&storage.positionDataQueue));
  positionMeasurement_t positionMeassurement = { .x = 1, .y = 2, .z = 3, .stdDev = 0.16 };
  estimatorKalmanEngine.enqueuePosition(&storage, &positionMeassurement);
  TEST_ASSERT_FALSE(fifo_is_empty(&storage.positionDataQueue));

  positionMeasurement_t position;
  fifo_get(&storage.positionDataQueue, &position);
  TEST_ASSERT_EQUAL_FLOAT(positionMeassurement.x, position.x);
  TEST_ASSERT_EQUAL_FLOAT(positionMeassurement.y, position.y);
  TEST_ASSERT_EQUAL_FLOAT(positionMeassurement.z, position.z);
  TEST_ASSERT_EQUAL_FLOAT(positionMeassurement.stdDev, position.stdDev);
  TEST_ASSERT_TRUE(fifo_is_empty(&storage.positionDataQueue));
}

void testEnqueuePositionAndUpdate() {
  estimatorKalmanStorage_t storage;
  helper_initStorageWithZeroMeassurements(&storage);

  for (int i = 1; i < 10; i++) {
    msCount += 100;
    positionMeasurement_t positionMeassurement = { .x = i, .y = i, .z = i, .stdDev = 0 };
    estimatorKalmanEngine.enqueuePosition(&storage, &positionMeassurement);
    estimatorKalmanEngine.update(&storage, false);

    point_t position;
    estimatorKalmanEngine.getPosition(&storage, &position);
    TEST_ASSERT_EQUAL_FLOAT(positionMeassurement.x, position.x);
    TEST_ASSERT_EQUAL_FLOAT(positionMeassurement.y, position.y);
    TEST_ASSERT_EQUAL_FLOAT(positionMeassurement.z, position.z);
    TEST_ASSERT_EQUAL_UINT32(msCount, storage.lastUpdate);
  }
}

void testEnqueueSpecificPositionWithStdDevAndUpdate() {
  estimatorKalmanStorage_t storage;
  helper_initStorageWithZeroMeassurements(&storage);

  msCount += 100;
  positionMeasurement_t positionMeassurement = { .x = 3, .y = 4, .z = 5, .stdDev = 0.0001 };
  estimatorKalmanEngine.enqueuePosition(&storage, &positionMeassurement);
  estimatorKalmanEngine.update(&storage, false);

  point_t position;
  estimatorKalmanEngine.getPosition(&storage, &position);
  TEST_ASSERT_EQUAL_FLOAT(2.97059, position.x);
  TEST_ASSERT_EQUAL_FLOAT(3.96078, position.y);
  TEST_ASSERT_EQUAL_FLOAT(4.95098, position.z);
  TEST_ASSERT_EQUAL_UINT32(msCount, storage.lastUpdate);
}
