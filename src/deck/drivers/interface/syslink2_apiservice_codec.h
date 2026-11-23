/**
 * Encoding, decoding and validation of API Service frames. Pure functions.
 *
 * See docs/functional-areas/camera-deck-apiservice.md, syslink2_apiservice_codec.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "syslink2_apiservice_types.h"

/** True when the request passes the request checks. */
bool syslink2_apiservice_codec_request_valid(
  const syslink2_apiservice_request_t *request);

/**
 * Decode a raw request, laid out as a GET, SET or CMD frame, into request.
 * Returns false when it cannot be decoded; request must then not be used. Does
 * not run the request checks.
 */
bool syslink2_apiservice_codec_decode_request(const uint8_t *data, uint8_t len,
                                           syslink2_apiservice_request_t *request);

/** Build a GET frame. Precondition: path is from a valid request. */
syslink2_apiservice_frame_t syslink2_apiservice_codec_encode_get(uint8_t request_id,
                                                           const char *path,
                                                           uint8_t path_len);

/** Build a SET frame. Precondition: path and value are from a valid request. */
syslink2_apiservice_frame_t syslink2_apiservice_codec_encode_set(
  uint8_t request_id, const char *path, uint8_t path_len,
  const syslink2_apiservice_value_t *value);

/**
 * Build a CMD frame. argument is NULL for a CMD without an argument, and the
 * frame then ends after the path. Precondition: path and argument are from a
 * valid request.
 */
syslink2_apiservice_frame_t syslink2_apiservice_codec_encode_cmd(
  uint8_t request_id, const char *path, uint8_t path_len,
  const syslink2_apiservice_value_t *argument);

/**
 * Classify a received frame. Fills in response only when the class is
 * SYSLINK2_APISERVICE_FRAME_CLASS_RESPONSE.
 */
syslink2_apiservice_frame_class_t syslink2_apiservice_codec_decode(
  const syslink2_apiservice_frame_t *frame,
  syslink2_apiservice_response_t *response);

/**
 * Build a raw response, an OK, VALUE or ERROR frame, from a decoded response,
 * with request_id in place of response->request_id. Precondition: response
 * was filled in by syslink2_apiservice_codec_decode, and value_valid is true if
 * it is a VALUE.
 */
syslink2_apiservice_frame_t syslink2_apiservice_codec_encode_response(
  uint8_t request_id, const syslink2_apiservice_response_t *response);
