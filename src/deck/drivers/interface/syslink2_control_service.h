/**
 * Control Service: GET and SET on paths of the camera deck.
 *
 * The user API is syslink2_control_service_submit,
 * syslink2_control_service_submit_raw and syslink2_control_service_is_ready,
 * plus the user's own completion callback. See
 * docs/functional-areas/camera-deck-control.md.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "bccam_uart_runtime.h"
#include "syslink2_control_types.h"

/**
 * Initialize the Control Service and its units, and create the Control task.
 * Called once, from the deck init.
 */
void syslink2_control_service_init(bccam_uart_runtime_t *runtime);

/**
 * Submit a request without blocking. The request is copied. Every request
 * that gives SYSLINK2_CONTROL_SUBMIT_ACCEPTED gets exactly one on_complete.
 * A NULL on_complete gives SYSLINK2_CONTROL_SUBMIT_INVALID.
 */
syslink2_control_submit_result_t syslink2_control_service_submit(
  const syslink2_control_request_t *request,
  syslink2_control_on_complete_t on_complete);

/**
 * Submit a raw request without blocking, laid out as a GET or SET frame whose
 * request_id byte holds the user request ID. The bytes are not kept. Every
 * request that gives SYSLINK2_CONTROL_SUBMIT_ACCEPTED gets exactly one
 * on_complete_raw, with the raw response for OK, VALUE and ERROR. A NULL
 * on_complete_raw gives SYSLINK2_CONTROL_SUBMIT_INVALID.
 */
syslink2_control_submit_result_t syslink2_control_service_submit_raw(
  const uint8_t *data, uint8_t len,
  syslink2_control_on_complete_raw_t on_complete_raw);

/**
 * Receives each unsolicited frame from the deck, such as a NOTIFY, in the
 * Control task. data is borrowed for the call. The frame is not checked
 * beyond its kind byte (0xc0..0xff).
 */
typedef void (*syslink2_control_unsolicited_fn)(const uint8_t *data,
                                                uint8_t len);

/**
 * Set the function that receives unsolicited frames, or NULL to drop them,
 * which is the default. Call it at init. It may be called after the Control
 * task has started: an unsolicited frame received before then is dropped.
 */
void syslink2_control_service_set_unsolicited_handler(
  syslink2_control_unsolicited_fn handler);

/** The ready flag. Requests are sent to the deck only while ready. */
bool syslink2_control_service_is_ready(void);

/** One step. Called only from the Control task, and from host tests. */
void syslink2_control_service_step(uint32_t now_ticks);
