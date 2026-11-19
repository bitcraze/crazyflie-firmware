#include "syslink2_apiservice.h"

#include <stddef.h>

#include "syslink2_apiservice_channel.h"
#include "syslink2_apiservice_codec.h"
#include "syslink2_apiservice_request_handling.h"
#include "syslink2_apiservice_request_queue.h"

#if !defined(UNIT_TEST) && !defined(UNIT_TEST_MODE)
#include "FreeRTOS.h"
#include "task.h"
#include "static_mem.h"
#include "system.h"

#define SYSLINK2_APISERVICE_TASK_NAME "bcCamApis"
#define SYSLINK2_APISERVICE_TASK_STACKSIZE (configMINIMAL_STACK_SIZE * 2)
#define SYSLINK2_APISERVICE_TASK_PRI (tskIDLE_PRIORITY + 2)
#define SYSLINK2_APISERVICE_TASK_PERIOD_MS 10u

STATIC_MEM_TASK_ALLOC(syslink2ApiserviceTask, SYSLINK2_APISERVICE_TASK_STACKSIZE);
#endif

/* Written only in the API Service task (ready) or once in init (initialized),
 * read from user tasks. */
static volatile bool initialized;
static volatile bool ready;
/* Set at init, possibly while the API Service task runs, read in the API Service
 * task. */
static volatile syslink2_apiservice_unsolicited_fn unsolicited_handler;

#if !defined(UNIT_TEST) && !defined(UNIT_TEST_MODE)
static void syslink2ApiserviceTask(void *arg) {
  (void)arg;
  systemWaitStart();
  while (true) {
    syslink2_apiservice_step((uint32_t)xTaskGetTickCount());
    vTaskDelay(M2T(SYSLINK2_APISERVICE_TASK_PERIOD_MS));
  }
}
#endif

void syslink2_apiservice_init(bccam_uart_runtime_t *runtime) {
  if (initialized) {
    return;
  }
  ready = false;
  syslink2_apiservice_request_queue_init();
  syslink2_apiservice_request_handling_init();
  syslink2_apiservice_channel_init(runtime);
  initialized = true;
#if !defined(UNIT_TEST) && !defined(UNIT_TEST_MODE)
  STATIC_MEM_TASK_CREATE(syslink2ApiserviceTask, syslink2ApiserviceTask,
                         SYSLINK2_APISERVICE_TASK_NAME, NULL,
                         SYSLINK2_APISERVICE_TASK_PRI);
#endif
}

static syslink2_apiservice_submit_result_t queue_valid_entry(
  const syslink2_apiservice_queue_entry_t *entry) {
  if ((entry->on_complete == NULL && entry->on_complete_raw == NULL) ||
      !syslink2_apiservice_codec_request_valid(&entry->request)) {
    return SYSLINK2_APISERVICE_SUBMIT_INVALID;
  }
  return syslink2_apiservice_request_queue_put(entry) ?
    SYSLINK2_APISERVICE_SUBMIT_ACCEPTED : SYSLINK2_APISERVICE_SUBMIT_QUEUE_FULL;
}

syslink2_apiservice_submit_result_t syslink2_apiservice_submit(
  const syslink2_apiservice_request_t *request,
  syslink2_apiservice_on_complete_t on_complete) {
  if (!initialized) {
    return SYSLINK2_APISERVICE_SUBMIT_NOT_INITIALIZED;
  }
  if (request == NULL) {
    return SYSLINK2_APISERVICE_SUBMIT_INVALID;
  }
  const syslink2_apiservice_queue_entry_t entry = {
    .request = *request,
    .on_complete = on_complete,
    .on_complete_raw = NULL,
  };
  return queue_valid_entry(&entry);
}

syslink2_apiservice_submit_result_t syslink2_apiservice_submit_raw(
  const uint8_t *data, uint8_t len,
  syslink2_apiservice_on_complete_raw_t on_complete_raw) {
  if (!initialized) {
    return SYSLINK2_APISERVICE_SUBMIT_NOT_INITIALIZED;
  }
  syslink2_apiservice_queue_entry_t entry = {
    .on_complete = NULL,
    .on_complete_raw = on_complete_raw,
  };
  if (!syslink2_apiservice_codec_decode_request(data, len, &entry.request)) {
    return SYSLINK2_APISERVICE_SUBMIT_INVALID;
  }
  return queue_valid_entry(&entry);
}

void syslink2_apiservice_set_unsolicited_handler(
  syslink2_apiservice_unsolicited_fn handler) {
  unsolicited_handler = handler;
}

bool syslink2_apiservice_is_ready(void) {
  return ready;
}

void syslink2_apiservice_step(uint32_t now_ticks) {
  if (!initialized) {
    return;
  }
  if (!syslink2_apiservice_channel_runtime_ready()) {
    ready = false;
    syslink2_apiservice_request_handling_step(false, NULL, now_ticks);
    return;
  }

  if (!ready) {
    syslink2_apiservice_channel_open();
    ready = true;
  }

  syslink2_apiservice_frame_t frame;
  if (syslink2_apiservice_channel_receive(&frame)) {
    syslink2_apiservice_response_t response;
    const syslink2_apiservice_frame_class_t frame_class =
      syslink2_apiservice_codec_decode(&frame, &response);
    if (frame_class == SYSLINK2_APISERVICE_FRAME_CLASS_RESPONSE) {
      syslink2_apiservice_request_handling_step(true, &response, now_ticks);
      return;
    }
    /* Unsolicited frames go to their handler. Malformed frames, and
     * unsolicited ones without a handler, are dropped. The receive slot is
     * already free again. */
    if (frame_class == SYSLINK2_APISERVICE_FRAME_CLASS_UNSOLICITED &&
        unsolicited_handler != NULL) {
      unsolicited_handler(frame.data, frame.len);
    }
  }
  syslink2_apiservice_request_handling_step(true, NULL, now_ticks);
}

#if defined(UNIT_TEST) || defined(UNIT_TEST_MODE)
void syslink2_apiservice_test_reset(void) {
  initialized = false;
  ready = false;
  unsolicited_handler = NULL;
}
#endif
