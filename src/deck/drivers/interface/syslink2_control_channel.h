/**
 * Transmit and receive of raw Control frames, and their credit. The only unit
 * that calls the runtime.
 *
 * See docs/functional-areas/camera-deck-control.md, syslink2_control_channel.
 */
#pragma once

#include <stdbool.h>

#include "bccam_uart_runtime.h"
#include "syslink2_control_types.h"

/** Store the runtime instance used for all runtime calls. */
void syslink2_control_channel_init(bccam_uart_runtime_t *runtime);

/** Pass on bccam_uart_runtime_control_ready(). */
bool syslink2_control_channel_runtime_ready(void);

/** Grant the first receive credit. */
void syslink2_control_channel_open(void);

/** True when there is transmit credit. */
bool syslink2_control_channel_can_send(void);

/** Send a raw frame and sort the runtime result into a send result. */
syslink2_control_send_result_t syslink2_control_channel_send(
  const syslink2_control_frame_t *frame);

/**
 * Copy a received frame out of the receive slot and grant receive credit at
 * once. False when there was none; frame must then not be used.
 */
bool syslink2_control_channel_receive(syslink2_control_frame_t *frame);
