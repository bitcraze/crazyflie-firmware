/**
 * Hands submitted requests from user tasks to the Control task.
 *
 * See docs/functional-areas/camera-deck-control.md,
 * syslink2_control_request_queue.
 */
#pragma once

#include <stdbool.h>

#include "syslink2_control_types.h"

#define SYSLINK2_CONTROL_REQUEST_QUEUE_LENGTH 4u

/** Create the queue. */
void syslink2_control_request_queue_init(void);

/** Copy the entry into the queue without blocking. False when full. */
bool syslink2_control_request_queue_put(
  const syslink2_control_queue_entry_t *entry);

/** Copy the oldest entry out without blocking. False when empty. */
bool syslink2_control_request_queue_take(syslink2_control_queue_entry_t *entry);
