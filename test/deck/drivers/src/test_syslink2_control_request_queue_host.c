#include <stdint.h>
#include <string.h>

#include "unity.h"
#include "syslink2_control_request_queue.h"

static syslink2_control_request_t request_with_id(uint8_t id) {
  syslink2_control_request_t request;
  memset(&request, 0, sizeof(request));
  request.user_request_id = id;
  request.path_len = 1u;
  request.path[0] = 'p';
  request.value.data[3] = id;
  return request;
}

void setUp(void) {
  syslink2_control_request_queue_init();
}

void tearDown(void) {}

void testEmptyQueueGivesNothing(void) {
  syslink2_control_request_t request;
  TEST_ASSERT_FALSE(syslink2_control_request_queue_take(&request));
}

void testRequestsAreTakenInOrderAsCopies(void) {
  syslink2_control_request_t first = request_with_id(1);
  syslink2_control_request_t second = request_with_id(2);
  TEST_ASSERT_TRUE(syslink2_control_request_queue_put(&first));
  TEST_ASSERT_TRUE(syslink2_control_request_queue_put(&second));
  first.user_request_id = 99u;

  syslink2_control_request_t taken;
  TEST_ASSERT_TRUE(syslink2_control_request_queue_take(&taken));
  TEST_ASSERT_EQUAL_UINT8(1, taken.user_request_id);
  TEST_ASSERT_EQUAL_UINT8(1, taken.value.data[3]);
  TEST_ASSERT_TRUE(syslink2_control_request_queue_take(&taken));
  TEST_ASSERT_EQUAL_UINT8(2, taken.user_request_id);
  TEST_ASSERT_FALSE(syslink2_control_request_queue_take(&taken));
}

void testQueueHoldsFourRequests(void) {
  for (uint8_t i = 0; i < SYSLINK2_CONTROL_REQUEST_QUEUE_LENGTH; i++) {
    syslink2_control_request_t request = request_with_id(i);
    TEST_ASSERT_TRUE(syslink2_control_request_queue_put(&request));
  }
  syslink2_control_request_t extra = request_with_id(10);
  TEST_ASSERT_FALSE(syslink2_control_request_queue_put(&extra));

  syslink2_control_request_t taken;
  TEST_ASSERT_TRUE(syslink2_control_request_queue_take(&taken));
  TEST_ASSERT_EQUAL_UINT8(0, taken.user_request_id);
  TEST_ASSERT_TRUE(syslink2_control_request_queue_put(&extra));
  for (uint8_t i = 1; i < SYSLINK2_CONTROL_REQUEST_QUEUE_LENGTH; i++) {
    TEST_ASSERT_TRUE(syslink2_control_request_queue_take(&taken));
    TEST_ASSERT_EQUAL_UINT8(i, taken.user_request_id);
  }
  TEST_ASSERT_TRUE(syslink2_control_request_queue_take(&taken));
  TEST_ASSERT_EQUAL_UINT8(10, taken.user_request_id);
}
