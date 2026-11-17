/**
 * Encoding, decoding and validation of Control frames. Pure functions.
 *
 * See docs/functional-areas/camera-deck-control.md, syslink2_control_codec.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "syslink2_control_types.h"

/** True when the request passes the request checks. */
bool syslink2_control_codec_request_valid(
  const syslink2_control_request_t *request);

/**
 * Decode a raw request, laid out as a GET or SET frame, into request and set
 * request->user to user. Returns false when it cannot be decoded; request
 * must then not be used. Does not run the request checks.
 */
bool syslink2_control_codec_decode_request(const uint8_t *data, uint8_t len,
                                           const syslink2_control_user_t *user,
                                           syslink2_control_request_t *request);

/** Build a GET frame. Precondition: path is from a valid request. */
syslink2_control_frame_t syslink2_control_codec_encode_get(uint8_t request_id,
                                                           const char *path,
                                                           uint8_t path_len);

/** Build a SET frame. Precondition: path and value are from a valid request. */
syslink2_control_frame_t syslink2_control_codec_encode_set(
  uint8_t request_id, const char *path, uint8_t path_len,
  const syslink2_control_value_t *value);

/**
 * Classify a received frame. Fills in response only when the class is
 * SYSLINK2_CONTROL_FRAME_CLASS_RESPONSE.
 */
syslink2_control_frame_class_t syslink2_control_codec_decode(
  const syslink2_control_frame_t *frame,
  syslink2_control_response_t *response);
