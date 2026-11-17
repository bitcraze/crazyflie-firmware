#include "syslink2_control_service.h"

#include <stddef.h>

#include "syslink2_control_channel.h"
#include "syslink2_control_codec.h"
#include "syslink2_control_request_handling.h"
#include "syslink2_control_request_queue.h"

#if !defined(UNIT_TEST) && !defined(UNIT_TEST_MODE)
#include "FreeRTOS.h"
#include "task.h"
#include "static_mem.h"
#include "system.h"

#define SYSLINK2_CONTROL_TASK_NAME "bcCamCtrl"
#define SYSLINK2_CONTROL_TASK_STACKSIZE (configMINIMAL_STACK_SIZE * 2)
#define SYSLINK2_CONTROL_TASK_PRI (tskIDLE_PRIORITY + 2)
#define SYSLINK2_CONTROL_TASK_PERIOD_MS 10u

STATIC_MEM_TASK_ALLOC(syslink2ControlTask, SYSLINK2_CONTROL_TASK_STACKSIZE);
#endif

/* Written only in the Control task (ready) or once in init (initialized),
 * read from user tasks. */
static volatile bool initialized;
static volatile bool ready;

#if !defined(UNIT_TEST) && !defined(UNIT_TEST_MODE)
static void syslink2ControlTask(void *arg) {
  (void)arg;
  systemWaitStart();
  while (true) {
    syslink2_control_service_step((uint32_t)xTaskGetTickCount());
    vTaskDelay(M2T(SYSLINK2_CONTROL_TASK_PERIOD_MS));
  }
}
#endif

void syslink2_control_service_init(bccam_uart_runtime_t *runtime) {
  if (initialized) {
    return;
  }
  ready = false;
  syslink2_control_request_queue_init();
  syslink2_control_request_handling_init();
  syslink2_control_channel_init(runtime);
  initialized = true;
#if !defined(UNIT_TEST) && !defined(UNIT_TEST_MODE)
  STATIC_MEM_TASK_CREATE(syslink2ControlTask, syslink2ControlTask,
                         SYSLINK2_CONTROL_TASK_NAME, NULL,
                         SYSLINK2_CONTROL_TASK_PRI);
#endif
}

static syslink2_control_submit_result_t queue_valid_request(
  const syslink2_control_request_t *request) {
  if (!syslink2_control_codec_request_valid(request)) {
    return SYSLINK2_CONTROL_SUBMIT_INVALID;
  }
  return syslink2_control_request_queue_put(request) ?
    SYSLINK2_CONTROL_SUBMIT_ACCEPTED : SYSLINK2_CONTROL_SUBMIT_QUEUE_FULL;
}

syslink2_control_submit_result_t syslink2_control_service_submit(
  const syslink2_control_request_t *request) {
  if (!initialized) {
    return SYSLINK2_CONTROL_SUBMIT_NOT_INITIALIZED;
  }
  return queue_valid_request(request);
}

syslink2_control_submit_result_t syslink2_control_service_submit_raw(
  const uint8_t *data, uint8_t len, const syslink2_control_user_t *user) {
  if (!initialized) {
    return SYSLINK2_CONTROL_SUBMIT_NOT_INITIALIZED;
  }
  syslink2_control_request_t request;
  if (!syslink2_control_codec_decode_request(data, len, user, &request)) {
    return SYSLINK2_CONTROL_SUBMIT_INVALID;
  }
  return queue_valid_request(&request);
}

bool syslink2_control_service_is_ready(void) {
  return ready;
}

void syslink2_control_service_step(uint32_t now_ticks) {
  if (!initialized) {
    return;
  }
  if (!syslink2_control_channel_runtime_ready()) {
    ready = false;
    syslink2_control_request_handling_step(false, NULL, now_ticks);
    return;
  }

  if (!ready) {
    syslink2_control_channel_open();
    ready = true;
  }

  syslink2_control_frame_t frame;
  if (syslink2_control_channel_receive(&frame)) {
    syslink2_control_response_t response;
    if (syslink2_control_codec_decode(&frame, &response) ==
        SYSLINK2_CONTROL_FRAME_CLASS_RESPONSE) {
      syslink2_control_request_handling_step(true, &response, now_ticks);
      return;
    }
    /* Unsolicited and malformed frames are dropped. */
  }
  syslink2_control_request_handling_step(true, NULL, now_ticks);
}

#if defined(UNIT_TEST) || defined(UNIT_TEST_MODE)
void syslink2_control_service_test_reset(void) {
  initialized = false;
  ready = false;
}
#endif
