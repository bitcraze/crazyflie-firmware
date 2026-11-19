/**
 * Handles one outstanding API Service request: take, send, match the answer and
 * call on_complete or on_complete_raw.
 *
 * See docs/functional-areas/camera-deck-apiservice.md,
 * syslink2_apiservice_request_handling.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "syslink2_apiservice_types.h"

/** Timeout of one request, from when it is taken from the queue. */
#define SYSLINK2_APISERVICE_TIMEOUT_MS 500u

/** Set init values. The deck request ID starts at 1. */
void syslink2_apiservice_request_handling_init(void);

/**
 * Do one step. response is the response received in this step, or NULL.
 * now_ticks is the tick count.
 */
void syslink2_apiservice_request_handling_step(
  bool ready, const syslink2_apiservice_response_t *response, uint32_t now_ticks);
