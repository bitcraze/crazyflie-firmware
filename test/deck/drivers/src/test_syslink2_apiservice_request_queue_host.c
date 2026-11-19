#include <stdint.h>
#include <string.h>

#include "unity.h"
#include "syslink2_apiservice_request_queue.h"

static void on_complete_raw(syslink2_apiservice_outcome_t outcome,
                            uint8_t user_request_id, const uint8_t *data,
                            uint8_t len) {
  (void)outcome;
  (void)user_request_id;
  (void)data;
  (void)len;
}

static syslink2_apiservice_queue_entry_t request_with_id(uint8_t id) {
  syslink2_apiservice_queue_entry_t entry;
  memset(&entry, 0, sizeof(entry));
  entry.request.user_request_id = id;
  entry.request.path_len = 1u;
  entry.request.path[0] = 'p';
  entry.request.value.data[3] = id;
  entry.on_complete_raw = on_complete_raw;
  return entry;
}

void setUp(void) {
  syslink2_apiservice_request_queue_init();
}

void tearDown(void) {}

void testEmptyQueueGivesNothing(void) {
  syslink2_apiservice_queue_entry_t request;
  TEST_ASSERT_FALSE(syslink2_apiservice_request_queue_take(&request));
}

void testRequestsAreTakenInOrderAsCopies(void) {
  syslink2_apiservice_queue_entry_t first = request_with_id(1);
  syslink2_apiservice_queue_entry_t second = request_with_id(2);
  TEST_ASSERT_TRUE(syslink2_apiservice_request_queue_put(&first));
  TEST_ASSERT_TRUE(syslink2_apiservice_request_queue_put(&second));
  first.request.user_request_id = 99u;
  first.on_complete_raw = NULL;

  syslink2_apiservice_queue_entry_t taken;
  TEST_ASSERT_TRUE(syslink2_apiservice_request_queue_take(&taken));
  TEST_ASSERT_EQUAL_UINT8(1, taken.request.user_request_id);
  TEST_ASSERT_EQUAL_UINT8(1, taken.request.value.data[3]);
  TEST_ASSERT_TRUE(taken.on_complete_raw == on_complete_raw);
  TEST_ASSERT_TRUE(taken.on_complete == NULL);
  TEST_ASSERT_TRUE(syslink2_apiservice_request_queue_take(&taken));
  TEST_ASSERT_EQUAL_UINT8(2, taken.request.user_request_id);
  TEST_ASSERT_FALSE(syslink2_apiservice_request_queue_take(&taken));
}

void testQueueHoldsFourRequests(void) {
  for (uint8_t i = 0; i < SYSLINK2_APISERVICE_REQUEST_QUEUE_LENGTH; i++) {
    syslink2_apiservice_queue_entry_t request = request_with_id(i);
    TEST_ASSERT_TRUE(syslink2_apiservice_request_queue_put(&request));
  }
  syslink2_apiservice_queue_entry_t extra = request_with_id(10);
  TEST_ASSERT_FALSE(syslink2_apiservice_request_queue_put(&extra));

  syslink2_apiservice_queue_entry_t taken;
  TEST_ASSERT_TRUE(syslink2_apiservice_request_queue_take(&taken));
  TEST_ASSERT_EQUAL_UINT8(0, taken.request.user_request_id);
  TEST_ASSERT_TRUE(syslink2_apiservice_request_queue_put(&extra));
  for (uint8_t i = 1; i < SYSLINK2_APISERVICE_REQUEST_QUEUE_LENGTH; i++) {
    TEST_ASSERT_TRUE(syslink2_apiservice_request_queue_take(&taken));
    TEST_ASSERT_EQUAL_UINT8(i, taken.request.user_request_id);
  }
  TEST_ASSERT_TRUE(syslink2_apiservice_request_queue_take(&taken));
  TEST_ASSERT_EQUAL_UINT8(10, taken.request.user_request_id);
}
