#include "syslink2_control_codec.h"

#include <stddef.h>
#include <string.h>

#define HEADER_LEN 2u
#define VALUE_HEADER_LEN 2u
#define UNSOLICITED_KIND_MIN 0xc0u

/** The length a value of type must have, or 0 when the type is out of scope. */
static uint8_t value_len_for_type(uint8_t type) {
  switch (type) {
    case SYSLINK2_CONTROL_TYPE_BOOL:
    case SYSLINK2_CONTROL_TYPE_U8:
    case SYSLINK2_CONTROL_TYPE_I8:
      return 1u;
    case SYSLINK2_CONTROL_TYPE_U16:
    case SYSLINK2_CONTROL_TYPE_I16:
      return 2u;
    case SYSLINK2_CONTROL_TYPE_U32:
    case SYSLINK2_CONTROL_TYPE_I32:
      return 4u;
    default:
      return 0u;
  }
}

/** The value checks, shared by SET requests and received VALUE frames. */
static bool value_checks(uint8_t type, uint8_t len, const uint8_t *data) {
  const uint8_t expected = value_len_for_type(type);
  if (expected == 0u || len != expected) {
    return false;
  }
  if (type == SYSLINK2_CONTROL_TYPE_BOOL && data[0] > 1u) {
    return false;
  }
  return true;
}

bool syslink2_control_codec_request_valid(
  const syslink2_control_request_t *request) {
  if (request == NULL) {
    return false;
  }
  if (request->operation != SYSLINK2_CONTROL_KIND_GET &&
      request->operation != SYSLINK2_CONTROL_KIND_SET) {
    return false;
  }
  if (request->path_len == 0u ||
      request->path_len > SYSLINK2_CONTROL_PATH_MAX_LEN) {
    return false;
  }
  for (uint8_t i = 0u; i < request->path_len; i++) {
    const uint8_t c = (uint8_t)request->path[i];
    if (c < 0x20u || c > 0x7eu) {
      return false;
    }
  }
  if (request->operation == SYSLINK2_CONTROL_KIND_SET &&
      !value_checks(request->value.type, request->value.len,
                    request->value.data)) {
    return false;
  }
  return true;
}

bool syslink2_control_codec_decode_request(const uint8_t *data, uint8_t len,
                                           syslink2_control_request_t *request) {
  if (data == NULL || request == NULL || len < 3u) {
    return false;
  }
  const uint8_t kind = data[0];
  if (kind != SYSLINK2_CONTROL_KIND_GET && kind != SYSLINK2_CONTROL_KIND_SET) {
    return false;
  }
  const uint8_t path_len = data[2];
  if (path_len > SYSLINK2_CONTROL_PATH_MAX_LEN ||
      (uint16_t)3u + path_len > len) {
    return false;
  }
  uint16_t offset = (uint16_t)(3u + path_len);
  syslink2_control_value_t value;
  memset(&value, 0, sizeof(value));
  if (kind == SYSLINK2_CONTROL_KIND_GET) {
    if (offset != len) {
      return false;
    }
  } else {
    if (offset + VALUE_HEADER_LEN > len) {
      return false;
    }
    value.type = data[offset];
    value.len = data[offset + 1u];
    offset = (uint16_t)(offset + VALUE_HEADER_LEN);
    if (value.len > SYSLINK2_CONTROL_VALUE_MAX_LEN ||
        offset + value.len != len) {
      return false;
    }
    memcpy(value.data, &data[offset], value.len);
  }

  memset(request, 0, sizeof(*request));
  request->operation = kind;
  request->user_request_id = data[1];
  request->path_len = path_len;
  memcpy(request->path, &data[3], path_len);
  request->value = value;
  return true;
}

static uint8_t encode_header(syslink2_control_frame_t *frame, uint8_t kind,
                             uint8_t request_id, const char *path,
                             uint8_t path_len) {
  frame->data[0] = kind;
  frame->data[1] = request_id;
  frame->data[2] = path_len;
  memcpy(&frame->data[3], path, path_len);
  return (uint8_t)(3u + path_len);
}

syslink2_control_frame_t syslink2_control_codec_encode_get(uint8_t request_id,
                                                           const char *path,
                                                           uint8_t path_len) {
  syslink2_control_frame_t frame;
  memset(&frame, 0, sizeof(frame));
  frame.len = encode_header(&frame, SYSLINK2_CONTROL_KIND_GET, request_id,
                            path, path_len);
  return frame;
}

syslink2_control_frame_t syslink2_control_codec_encode_set(
  uint8_t request_id, const char *path, uint8_t path_len,
  const syslink2_control_value_t *value) {
  syslink2_control_frame_t frame;
  memset(&frame, 0, sizeof(frame));
  uint8_t offset = encode_header(&frame, SYSLINK2_CONTROL_KIND_SET, request_id,
                                 path, path_len);
  frame.data[offset++] = value->type;
  frame.data[offset++] = value->len;
  memcpy(&frame.data[offset], value->data, value->len);
  frame.len = (uint8_t)(offset + value->len);
  return frame;
}

syslink2_control_frame_t syslink2_control_codec_encode_response(
  uint8_t request_id, const syslink2_control_response_t *response) {
  syslink2_control_frame_t frame;
  memset(&frame, 0, sizeof(frame));
  frame.data[0] = response->kind;
  frame.data[1] = request_id;
  uint8_t offset = HEADER_LEN;
  if (response->kind == SYSLINK2_CONTROL_KIND_VALUE) {
    frame.data[offset++] = response->value.type;
    frame.data[offset++] = response->value.len;
    memcpy(&frame.data[offset], response->value.data, response->value.len);
    offset = (uint8_t)(offset + response->value.len);
  } else if (response->kind == SYSLINK2_CONTROL_KIND_ERROR) {
    frame.data[offset++] = response->error_code;
    frame.data[offset++] = response->detail_len;
    memcpy(&frame.data[offset], response->detail, response->detail_len);
    offset = (uint8_t)(offset + response->detail_len);
  }
  frame.len = offset;
  return frame;
}

static bool decode_value(const syslink2_control_frame_t *frame,
                         syslink2_control_response_t *response) {
  if (frame->len < HEADER_LEN + VALUE_HEADER_LEN) {
    return false;
  }
  const uint8_t type = frame->data[2];
  const uint8_t len = frame->data[3];
  if ((uint16_t)HEADER_LEN + VALUE_HEADER_LEN + len != frame->len) {
    return false;
  }
  const uint8_t *data = &frame->data[HEADER_LEN + VALUE_HEADER_LEN];
  response->value_valid = value_checks(type, len, data);
  if (response->value_valid) {
    response->value.type = type;
    response->value.len = len;
    memcpy(response->value.data, data, len);
  }
  return true;
}

static bool decode_error(const syslink2_control_frame_t *frame,
                         syslink2_control_response_t *response) {
  if (frame->len < HEADER_LEN + 2u) {
    return false;
  }
  const uint8_t detail_len = frame->data[3];
  if ((uint16_t)HEADER_LEN + 2u + detail_len != frame->len ||
      detail_len > SYSLINK2_CONTROL_DETAIL_MAX_LEN) {
    return false;
  }
  response->error_code = frame->data[2];
  response->detail_len = detail_len;
  memcpy(response->detail, &frame->data[HEADER_LEN + 2u], detail_len);
  return true;
}

syslink2_control_frame_class_t syslink2_control_codec_decode(
  const syslink2_control_frame_t *frame,
  syslink2_control_response_t *response) {
  if (frame == NULL || response == NULL ||
      frame->len > SYSLINK2_CONTROL_FRAME_MAX_LEN || frame->len < 1u) {
    return SYSLINK2_CONTROL_FRAME_CLASS_MALFORMED;
  }
  const uint8_t kind = frame->data[0];
  if (kind >= UNSOLICITED_KIND_MIN) {
    return SYSLINK2_CONTROL_FRAME_CLASS_UNSOLICITED;
  }
  if (frame->len < HEADER_LEN) {
    return SYSLINK2_CONTROL_FRAME_CLASS_MALFORMED;
  }

  syslink2_control_response_t decoded;
  memset(&decoded, 0, sizeof(decoded));
  decoded.kind = kind;
  decoded.request_id = frame->data[1];
  bool ok = false;
  switch (kind) {
    case SYSLINK2_CONTROL_KIND_OK:
      ok = frame->len == HEADER_LEN;
      break;
    case SYSLINK2_CONTROL_KIND_VALUE:
      ok = decode_value(frame, &decoded);
      break;
    case SYSLINK2_CONTROL_KIND_ERROR:
      ok = decode_error(frame, &decoded);
      break;
    default:
      ok = false;
      break;
  }
  if (!ok) {
    return SYSLINK2_CONTROL_FRAME_CLASS_MALFORMED;
  }
  *response = decoded;
  return SYSLINK2_CONTROL_FRAME_CLASS_RESPONSE;
}
