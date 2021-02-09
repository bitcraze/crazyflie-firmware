#include "unity.h"
#include "mock_kalman_core.h"

// Helpers to simplify testing of measurment model implementations in the kalman core

// Use to set up expectations for one call to kalmanCoreScalarUpdate(). This helper does not support multiple calls.

// Storage for expected values
const kalmanCoreData_t* kcsus_expectedThis;
float kcsus_expectedHm[KC_STATE_DIM];
float kcsus_expectedError;
float kcsus_expectedStdMeasNoise;
float kcsus_scalarUpadateWasCalled;

void mock_kalmanCoreScalarUpdate_callback(kalmanCoreData_t* actualThis, arm_matrix_instance_f32* actualHm, float actualError, float actualStdMeasNoise, int cmock_num_calls);

// Inti the mock with default (unlikely) values
void initKalmanCoreScalarUpdateExpectationsSingleCall() {
  kcsus_expectedThis = (void*)0;
  memset(kcsus_expectedHm, 0, sizeof(kcsus_expectedHm));
  kcsus_expectedError = 1234567890.0f;
  kcsus_expectedStdMeasNoise = 987654321.0f;
  kcsus_scalarUpadateWasCalled = false;
}

// Call this fkn to set expected values for the mock
// Note: expectedHm is a pure c float vector, not arm_matrix_instance_f32 to make life somple.
void setKalmanCoreScalarUpdateExpectationsSingleCall(const kalmanCoreData_t* expectedThis, const float* expectedHm, const float expectedError, const float expectedStdMeasNoise) {
  kcsus_expectedThis = expectedThis;
  memcpy(kcsus_expectedHm, expectedHm, sizeof(kcsus_expectedHm));
  kcsus_expectedError = expectedError;
  kcsus_expectedStdMeasNoise = expectedStdMeasNoise;

  kalmanCoreScalarUpdate_StubWithCallback(mock_kalmanCoreScalarUpdate_callback);
}

// Callback doing the validation work
void mock_kalmanCoreScalarUpdate_callback(kalmanCoreData_t* actualThis, arm_matrix_instance_f32* actualHm, float actualError, float actualStdMeasNoise, int cmock_num_calls) {
  kcsus_scalarUpadateWasCalled = true;
  TEST_ASSERT_EQUAL_INT_MESSAGE(0, cmock_num_calls, "Only expect one call");
  TEST_ASSERT_EQUAL_PTR_MESSAGE(kcsus_expectedThis, actualThis, "Unexpected this pointer");

  // Verify size and contents of Hm vector
  TEST_ASSERT_EQUAL_UINT16_MESSAGE(1, actualHm->numRows, "Uexpected nr of rows in Hm Vector");
  TEST_ASSERT_EQUAL_UINT16_MESSAGE(KC_STATE_DIM, actualHm->numCols, "Unexpected nr of collumns in Hm vector");
  for (int i = 0; i < KC_STATE_DIM; i++) {
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(1e-20, kcsus_expectedHm[i], actualHm->pData[i], "Unexpected value of element in Hm vector");
  }

  TEST_ASSERT_EQUAL_FLOAT_MESSAGE(kcsus_expectedError, actualError, "Unexpected value of error");
  TEST_ASSERT_EQUAL_FLOAT_MESSAGE(kcsus_expectedStdMeasNoise, actualStdMeasNoise, "Unexpected value of stdMeasNoise");
}

void assertScalarUpdateWasCalled() {
  TEST_ASSERT_TRUE_MESSAGE(kcsus_scalarUpadateWasCalled, "kalmanCoreScalarUpdate() was never called");
}

void assertScalarUpdateWasNotCalled() {
  TEST_ASSERT_FALSE_MESSAGE(kcsus_scalarUpadateWasCalled, "kalmanCoreScalarUpdate() got an unexpected call");
}
