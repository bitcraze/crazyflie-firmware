#include "syslink2_control_request_queue.h"

#include <stddef.h>
#include <string.h>

#if !defined(UNIT_TEST) && !defined(UNIT_TEST_MODE)

#include "FreeRTOS.h"
#include "queue.h"
#include "static_mem.h"

static xQueueHandle queue;
STATIC_MEM_QUEUE_ALLOC(queue, SYSLINK2_CONTROL_REQUEST_QUEUE_LENGTH,
                       sizeof(syslink2_control_queue_entry_t));

void syslink2_control_request_queue_init(void) {
  queue = STATIC_MEM_QUEUE_CREATE(queue);
}

bool syslink2_control_request_queue_put(
  const syslink2_control_queue_entry_t *entry) {
  return queue != NULL && entry != NULL &&
         xQueueSend(queue, entry, 0) == pdTRUE;
}

bool syslink2_control_request_queue_take(syslink2_control_queue_entry_t *entry) {
  return queue != NULL && entry != NULL &&
         xQueueReceive(queue, entry, 0) == pdTRUE;
}

#else

/* Host tests: a ring buffer with the same behaviour as the FreeRTOS queue,
 * without its task safety. */
static syslink2_control_queue_entry_t entries[SYSLINK2_CONTROL_REQUEST_QUEUE_LENGTH];
static uint8_t head;
static uint8_t count;

void syslink2_control_request_queue_init(void) {
  memset(entries, 0, sizeof(entries));
  head = 0u;
  count = 0u;
}

bool syslink2_control_request_queue_put(
  const syslink2_control_queue_entry_t *entry) {
  if (entry == NULL || count >= SYSLINK2_CONTROL_REQUEST_QUEUE_LENGTH) {
    return false;
  }
  entries[(head + count) % SYSLINK2_CONTROL_REQUEST_QUEUE_LENGTH] = *entry;
  count++;
  return true;
}

bool syslink2_control_request_queue_take(syslink2_control_queue_entry_t *entry) {
  if (entry == NULL || count == 0u) {
    return false;
  }
  *entry = entries[head];
  head = (uint8_t)((head + 1u) % SYSLINK2_CONTROL_REQUEST_QUEUE_LENGTH);
  count--;
  return true;
}

#endif
