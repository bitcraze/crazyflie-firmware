#include <stdint.h>
#include <string.h>

#include "unity.h"
#include "syslink2_apiservice_codec.h"

static syslink2_apiservice_request_t make_request(uint8_t operation,
                                               const char *path) {
  syslink2_apiservice_request_t request;
  memset(&request, 0, sizeof(request));
  request.operation = operation;
  request.user_request_id = 9u;
  request.path_len = (uint8_t)strlen(path);
  memcpy(request.path, path, request.path_len);
  request.value.type = SYSLINK2_APISERVICE_TYPE_U8;
  request.value.len = 1u;
  request.value.data[0] = 42u;
  return request;
}

static syslink2_apiservice_frame_t frame_of(const uint8_t *bytes, uint8_t len) {
  syslink2_apiservice_frame_t frame;
  memset(&frame, 0, sizeof(frame));
  memcpy(frame.data, bytes, len);
  frame.len = len;
  return frame;
}

void setUp(void) {}
void tearDown(void) {}

/* Request checks */

void testValidGetAndSetPass(void) {
  syslink2_apiservice_request_t get = make_request(SYSLINK2_APISERVICE_KIND_GET, "a.b");
  syslink2_apiservice_request_t set = make_request(SYSLINK2_APISERVICE_KIND_SET, "a.b");
  TEST_ASSERT_TRUE(syslink2_apiservice_codec_request_valid(&get));
  TEST_ASSERT_TRUE(syslink2_apiservice_codec_request_valid(&set));
}

void testUnknownOperationIsInvalid(void) {
  syslink2_apiservice_request_t request = make_request(0x03u, "a.b");
  TEST_ASSERT_FALSE(syslink2_apiservice_codec_request_valid(&request));
}

void testPathLengthLimits(void) {
  syslink2_apiservice_request_t request = make_request(SYSLINK2_APISERVICE_KIND_GET, "a");
  request.path_len = 0u;
  TEST_ASSERT_FALSE(syslink2_apiservice_codec_request_valid(&request));
  memset(request.path, 'x', sizeof(request.path));
  request.path_len = SYSLINK2_APISERVICE_PATH_MAX_LEN;
  TEST_ASSERT_TRUE(syslink2_apiservice_codec_request_valid(&request));
  request.path_len = SYSLINK2_APISERVICE_PATH_MAX_LEN + 1u;
  TEST_ASSERT_FALSE(syslink2_apiservice_codec_request_valid(&request));
}

void testNonPrintablePathIsInvalid(void) {
  syslink2_apiservice_request_t request = make_request(SYSLINK2_APISERVICE_KIND_GET, "a.b");
  request.path[1] = 0x1f;
  TEST_ASSERT_FALSE(syslink2_apiservice_codec_request_valid(&request));
  request.path[1] = 0x7f;
  TEST_ASSERT_FALSE(syslink2_apiservice_codec_request_valid(&request));
}

void testSetValueChecks(void) {
  syslink2_apiservice_request_t request = make_request(SYSLINK2_APISERVICE_KIND_SET, "a.b");
  request.value.type = 0x18u; /* out of scope */
  TEST_ASSERT_FALSE(syslink2_apiservice_codec_request_valid(&request));
  request.value.type = SYSLINK2_APISERVICE_TYPE_U16;
  request.value.len = 1u;
  TEST_ASSERT_FALSE(syslink2_apiservice_codec_request_valid(&request));
  request.value.len = 2u;
  TEST_ASSERT_TRUE(syslink2_apiservice_codec_request_valid(&request));
  request.value.type = SYSLINK2_APISERVICE_TYPE_BOOL;
  request.value.len = 1u;
  request.value.data[0] = 2u;
  TEST_ASSERT_FALSE(syslink2_apiservice_codec_request_valid(&request));
  request.value.data[0] = 1u;
  TEST_ASSERT_TRUE(syslink2_apiservice_codec_request_valid(&request));
}

void testSetValueChecks64Bit(void) {
  syslink2_apiservice_request_t request = make_request(SYSLINK2_APISERVICE_KIND_SET, "a.b");
  request.value.type = SYSLINK2_APISERVICE_TYPE_U64;
  request.value.len = 4u;
  TEST_ASSERT_FALSE(syslink2_apiservice_codec_request_valid(&request));
  request.value.len = 8u;
  TEST_ASSERT_TRUE(syslink2_apiservice_codec_request_valid(&request));
  request.value.type = SYSLINK2_APISERVICE_TYPE_I64;
  TEST_ASSERT_TRUE(syslink2_apiservice_codec_request_valid(&request));
  request.value.len = 9u;
  TEST_ASSERT_FALSE(syslink2_apiservice_codec_request_valid(&request));
}

void testSetValueChecksString(void) {
  syslink2_apiservice_request_t request = make_request(SYSLINK2_APISERVICE_KIND_SET, "a.b");
  request.value.type = SYSLINK2_APISERVICE_TYPE_STRING;
  request.value.len = 0u;
  TEST_ASSERT_TRUE(syslink2_apiservice_codec_request_valid(&request));
  memcpy(request.value.data, "hello", 5);
  request.value.len = 5u;
  TEST_ASSERT_TRUE(syslink2_apiservice_codec_request_valid(&request));
  request.value.data[2] = 0u;
  TEST_ASSERT_FALSE(syslink2_apiservice_codec_request_valid(&request));
  request.value.type = SYSLINK2_APISERVICE_TYPE_BYTES;
  TEST_ASSERT_TRUE(syslink2_apiservice_codec_request_valid(&request));
}

void testSetValueOverMaxIsInvalid(void) {
  syslink2_apiservice_request_t request = make_request(SYSLINK2_APISERVICE_KIND_SET, "a");
  request.value.type = SYSLINK2_APISERVICE_TYPE_BYTES;
  request.value.len = SYSLINK2_APISERVICE_VALUE_MAX_LEN + 1u;
  TEST_ASSERT_FALSE(syslink2_apiservice_codec_request_valid(&request));
}

void testSetMustFitInOneFrame(void) {
  syslink2_apiservice_request_t request = make_request(SYSLINK2_APISERVICE_KIND_SET, "a");
  memset(request.path, 'x', sizeof(request.path));
  memset(request.value.data, 'v', sizeof(request.value.data));
  request.value.type = SYSLINK2_APISERVICE_TYPE_BYTES;
  /* 5 + path_len + value.len = 64 */
  request.path_len = 1u;
  request.value.len = 58u;
  TEST_ASSERT_TRUE(syslink2_apiservice_codec_request_valid(&request));
  request.value.len = 59u;
  TEST_ASSERT_FALSE(syslink2_apiservice_codec_request_valid(&request));
  request.path_len = 50u;
  request.value.len = 9u;
  TEST_ASSERT_TRUE(syslink2_apiservice_codec_request_valid(&request));
  request.value.len = 10u;
  TEST_ASSERT_FALSE(syslink2_apiservice_codec_request_valid(&request));
  request.value.type = SYSLINK2_APISERVICE_TYPE_U64;
  request.value.len = 8u;
  request.path_len = SYSLINK2_APISERVICE_PATH_MAX_LEN;
  TEST_ASSERT_FALSE(syslink2_apiservice_codec_request_valid(&request));
  request.path_len = 51u;
  TEST_ASSERT_TRUE(syslink2_apiservice_codec_request_valid(&request));
}

void testGetWithLongStringValueFieldIsValid(void) {
  syslink2_apiservice_request_t request = make_request(SYSLINK2_APISERVICE_KIND_GET, "a");
  memset(request.path, 'x', sizeof(request.path));
  request.path_len = SYSLINK2_APISERVICE_PATH_MAX_LEN;
  request.value.type = SYSLINK2_APISERVICE_TYPE_STRING;
  request.value.len = SYSLINK2_APISERVICE_VALUE_MAX_LEN;
  TEST_ASSERT_TRUE(syslink2_apiservice_codec_request_valid(&request));
}

void testGetValueIsNotRead(void) {
  syslink2_apiservice_request_t request = make_request(SYSLINK2_APISERVICE_KIND_GET, "a.b");
  request.value.type = 0xffu;
  request.value.len = 200u;
  TEST_ASSERT_TRUE(syslink2_apiservice_codec_request_valid(&request));
}

void testNullRequestIsInvalid(void) {
  TEST_ASSERT_FALSE(syslink2_apiservice_codec_request_valid(NULL));
}

/* Raw requests */

void testDecodeRawGet(void) {
  const uint8_t raw[] = { 0x01, 7, 3, 'a', '.', 'b' };
  syslink2_apiservice_request_t request;
  TEST_ASSERT_TRUE(syslink2_apiservice_codec_decode_request(raw, sizeof(raw), &request));
  TEST_ASSERT_EQUAL_UINT8(SYSLINK2_APISERVICE_KIND_GET, request.operation);
  TEST_ASSERT_EQUAL_UINT8(7, request.user_request_id);
  TEST_ASSERT_EQUAL_UINT8(3, request.path_len);
  TEST_ASSERT_EQUAL_MEMORY("a.b", request.path, 3);
}

void testDecodeRawSet(void) {
  const uint8_t raw[] = { 0x02, 8, 1, 'q', 0x12, 2, 0x34, 0x12 };
  syslink2_apiservice_request_t request;
  TEST_ASSERT_TRUE(syslink2_apiservice_codec_decode_request(raw, sizeof(raw), &request));
  TEST_ASSERT_EQUAL_UINT8(SYSLINK2_APISERVICE_KIND_SET, request.operation);
  TEST_ASSERT_EQUAL_UINT8(0x12, request.value.type);
  TEST_ASSERT_EQUAL_UINT8(2, request.value.len);
  TEST_ASSERT_EQUAL_UINT8(0x34, request.value.data[0]);
  TEST_ASSERT_EQUAL_UINT8(0x12, request.value.data[1]);
}

void testDecodeRawRejectsBadLengths(void) {
  syslink2_apiservice_request_t request;
  const uint8_t short_raw[] = { 0x01, 7 };
  TEST_ASSERT_FALSE(syslink2_apiservice_codec_decode_request(short_raw, sizeof(short_raw), &request));
  TEST_ASSERT_FALSE(syslink2_apiservice_codec_decode_request(NULL, 3, &request));
  const uint8_t bad_kind[] = { 0x81, 7, 1, 'a' };
  TEST_ASSERT_FALSE(syslink2_apiservice_codec_decode_request(bad_kind, sizeof(bad_kind), &request));
  const uint8_t path_too_long_for_len[] = { 0x01, 7, 4, 'a', 'b' };
  TEST_ASSERT_FALSE(syslink2_apiservice_codec_decode_request(path_too_long_for_len, sizeof(path_too_long_for_len), &request));
  const uint8_t get_trailing[] = { 0x01, 7, 1, 'a', 0 };
  TEST_ASSERT_FALSE(syslink2_apiservice_codec_decode_request(get_trailing, sizeof(get_trailing), &request));
  const uint8_t set_no_value_header[] = { 0x02, 7, 1, 'a', 0x10 };
  TEST_ASSERT_FALSE(syslink2_apiservice_codec_decode_request(set_no_value_header, sizeof(set_no_value_header), &request));
  uint8_t set_value_too_long[6 + SYSLINK2_APISERVICE_VALUE_MAX_LEN + 1];
  memset(set_value_too_long, 'v', sizeof(set_value_too_long));
  set_value_too_long[0] = 0x02;
  set_value_too_long[1] = 7;
  set_value_too_long[2] = 1;
  set_value_too_long[3] = 'a';
  set_value_too_long[4] = 0x21;
  set_value_too_long[5] = SYSLINK2_APISERVICE_VALUE_MAX_LEN + 1u;
  TEST_ASSERT_FALSE(syslink2_apiservice_codec_decode_request(set_value_too_long, sizeof(set_value_too_long), &request));
  const uint8_t set_value_trailing[] = { 0x02, 7, 1, 'a', 0x10, 1, 5, 0 };
  TEST_ASSERT_FALSE(syslink2_apiservice_codec_decode_request(set_value_trailing, sizeof(set_value_trailing), &request));
}

void testDecodeRawSetString(void) {
  const uint8_t raw[] = { 0x02, 8, 1, 'q', 0x20, 3, 'a', 'b', 'c' };
  syslink2_apiservice_request_t request;
  TEST_ASSERT_TRUE(syslink2_apiservice_codec_decode_request(raw, sizeof(raw), &request));
  TEST_ASSERT_EQUAL_UINT8(SYSLINK2_APISERVICE_TYPE_STRING, request.value.type);
  TEST_ASSERT_EQUAL_UINT8(3, request.value.len);
  TEST_ASSERT_EQUAL_MEMORY("abc", request.value.data, 3);
  TEST_ASSERT_TRUE(syslink2_apiservice_codec_request_valid(&request));
}

void testDecodeRawPathOverMaxIsRejected(void) {
  uint8_t raw[3 + SYSLINK2_APISERVICE_PATH_MAX_LEN + 1];
  memset(raw, 'x', sizeof(raw));
  raw[0] = 0x01;
  raw[1] = 0;
  raw[2] = SYSLINK2_APISERVICE_PATH_MAX_LEN + 1u;
  syslink2_apiservice_request_t request;
  TEST_ASSERT_FALSE(syslink2_apiservice_codec_decode_request(raw, sizeof(raw), &request));
}

void testDecodeRawLeavesContentChecksToRequestChecks(void) {
  const uint8_t raw[] = { 0x01, 7, 0 };
  syslink2_apiservice_request_t request;
  TEST_ASSERT_TRUE(syslink2_apiservice_codec_decode_request(raw, sizeof(raw), &request));
  TEST_ASSERT_FALSE(syslink2_apiservice_codec_request_valid(&request));
}

/* Encoding */

void testEncodeGet(void) {
  const syslink2_apiservice_frame_t frame = syslink2_apiservice_codec_encode_get(5, "a.b", 3);
  const uint8_t expected[] = { 0x01, 5, 3, 'a', '.', 'b' };
  TEST_ASSERT_EQUAL_UINT8(sizeof(expected), frame.len);
  TEST_ASSERT_EQUAL_MEMORY(expected, frame.data, sizeof(expected));
}

void testEncodeSetU32(void) {
  const syslink2_apiservice_value_t value = { .type = 0x14, .len = 4, .data = { 1, 2, 3, 4 } };
  const syslink2_apiservice_frame_t frame = syslink2_apiservice_codec_encode_set(6, "q", 1, &value);
  const uint8_t expected[] = { 0x02, 6, 1, 'q', 0x14, 4, 1, 2, 3, 4 };
  TEST_ASSERT_EQUAL_UINT8(sizeof(expected), frame.len);
  TEST_ASSERT_EQUAL_MEMORY(expected, frame.data, sizeof(expected));
}

void testEncodeSetWithLongestPathFitsFrame(void) {
  char path[SYSLINK2_APISERVICE_PATH_MAX_LEN];
  memset(path, 'x', sizeof(path));
  const syslink2_apiservice_value_t value = { .type = 0x14, .len = 4 };
  const syslink2_apiservice_frame_t frame =
    syslink2_apiservice_codec_encode_set(6, path, sizeof(path), &value);
  TEST_ASSERT_EQUAL_UINT8(SYSLINK2_APISERVICE_FRAME_MAX_LEN, frame.len);
}

void testEncodeSetU64(void) {
  const syslink2_apiservice_value_t value = { .type = 0x16, .len = 8, .data = { 1, 2, 3, 4, 5, 6, 7, 8 } };
  const syslink2_apiservice_frame_t frame = syslink2_apiservice_codec_encode_set(6, "q", 1, &value);
  const uint8_t expected[] = { 0x02, 6, 1, 'q', 0x16, 8, 1, 2, 3, 4, 5, 6, 7, 8 };
  TEST_ASSERT_EQUAL_UINT8(sizeof(expected), frame.len);
  TEST_ASSERT_EQUAL_MEMORY(expected, frame.data, sizeof(expected));
}

void testEncodeSetString(void) {
  const syslink2_apiservice_value_t value = { .type = 0x20, .len = 2, .data = { 'h', 'i' } };
  const syslink2_apiservice_frame_t frame = syslink2_apiservice_codec_encode_set(6, "q", 1, &value);
  const uint8_t expected[] = { 0x02, 6, 1, 'q', 0x20, 2, 'h', 'i' };
  TEST_ASSERT_EQUAL_UINT8(sizeof(expected), frame.len);
  TEST_ASSERT_EQUAL_MEMORY(expected, frame.data, sizeof(expected));
}

void testEncodeSetLongestBytesFitsFrame(void) {
  syslink2_apiservice_value_t value = { .type = 0x21, .len = 58 };
  memset(value.data, 'v', value.len);
  const syslink2_apiservice_frame_t frame = syslink2_apiservice_codec_encode_set(6, "q", 1, &value);
  TEST_ASSERT_EQUAL_UINT8(SYSLINK2_APISERVICE_FRAME_MAX_LEN, frame.len);
  TEST_ASSERT_EQUAL_UINT8('v', frame.data[SYSLINK2_APISERVICE_FRAME_MAX_LEN - 1u]);
}

/* Received frames */

void testDecodeOk(void) {
  const uint8_t bytes[] = { 0x80, 3 };
  syslink2_apiservice_frame_t frame = frame_of(bytes, sizeof(bytes));
  syslink2_apiservice_response_t response;
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_FRAME_CLASS_RESPONSE, syslink2_apiservice_codec_decode(&frame, &response));
  TEST_ASSERT_EQUAL_UINT8(0x80, response.kind);
  TEST_ASSERT_EQUAL_UINT8(3, response.request_id);
}

void testDecodeOkWithBodyIsMalformed(void) {
  const uint8_t bytes[] = { 0x80, 3, 0 };
  syslink2_apiservice_frame_t frame = frame_of(bytes, sizeof(bytes));
  syslink2_apiservice_response_t response;
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_FRAME_CLASS_MALFORMED, syslink2_apiservice_codec_decode(&frame, &response));
}

void testDecodeValue(void) {
  const uint8_t bytes[] = { 0x81, 4, 0x13, 2, 0xfe, 0xff };
  syslink2_apiservice_frame_t frame = frame_of(bytes, sizeof(bytes));
  syslink2_apiservice_response_t response;
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_FRAME_CLASS_RESPONSE, syslink2_apiservice_codec_decode(&frame, &response));
  TEST_ASSERT_TRUE(response.value_valid);
  TEST_ASSERT_EQUAL_UINT8(0x13, response.value.type);
  TEST_ASSERT_EQUAL_UINT8(2, response.value.len);
  TEST_ASSERT_EQUAL_UINT8(0xfe, response.value.data[0]);
}

void testDecodeValueOutOfScopeIsResponseWithInvalidValue(void) {
  const uint8_t bytes[] = { 0x81, 4, 0x18, 3, 1, 2, 3 }; /* unknown type */
  syslink2_apiservice_frame_t frame = frame_of(bytes, sizeof(bytes));
  syslink2_apiservice_response_t response;
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_FRAME_CLASS_RESPONSE, syslink2_apiservice_codec_decode(&frame, &response));
  TEST_ASSERT_FALSE(response.value_valid);
}

void testDecodeValueI64(void) {
  const uint8_t bytes[] = { 0x81, 4, 0x17, 8, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
  syslink2_apiservice_frame_t frame = frame_of(bytes, sizeof(bytes));
  syslink2_apiservice_response_t response;
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_FRAME_CLASS_RESPONSE, syslink2_apiservice_codec_decode(&frame, &response));
  TEST_ASSERT_TRUE(response.value_valid);
  TEST_ASSERT_EQUAL_UINT8(SYSLINK2_APISERVICE_TYPE_I64, response.value.type);
  TEST_ASSERT_EQUAL_UINT8(8, response.value.len);
  TEST_ASSERT_EQUAL_MEMORY(&bytes[4], response.value.data, 8);
}

void testDecodeValue64BitWrongLengthIsInvalidValue(void) {
  const uint8_t bytes[] = { 0x81, 4, 0x16, 4, 1, 2, 3, 4 };
  syslink2_apiservice_frame_t frame = frame_of(bytes, sizeof(bytes));
  syslink2_apiservice_response_t response;
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_FRAME_CLASS_RESPONSE, syslink2_apiservice_codec_decode(&frame, &response));
  TEST_ASSERT_FALSE(response.value_valid);
}

void testDecodeValueString(void) {
  const uint8_t bytes[] = { 0x81, 4, 0x20, 5, 'h', 'e', 'l', 'l', 'o' };
  syslink2_apiservice_frame_t frame = frame_of(bytes, sizeof(bytes));
  syslink2_apiservice_response_t response;
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_FRAME_CLASS_RESPONSE, syslink2_apiservice_codec_decode(&frame, &response));
  TEST_ASSERT_TRUE(response.value_valid);
  TEST_ASSERT_EQUAL_UINT8(SYSLINK2_APISERVICE_TYPE_STRING, response.value.type);
  TEST_ASSERT_EQUAL_UINT8(5, response.value.len);
  TEST_ASSERT_EQUAL_MEMORY("hello", response.value.data, 5);
}

void testDecodeValueEmptyString(void) {
  const uint8_t bytes[] = { 0x81, 4, 0x20, 0 };
  syslink2_apiservice_frame_t frame = frame_of(bytes, sizeof(bytes));
  syslink2_apiservice_response_t response;
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_FRAME_CLASS_RESPONSE, syslink2_apiservice_codec_decode(&frame, &response));
  TEST_ASSERT_TRUE(response.value_valid);
  TEST_ASSERT_EQUAL_UINT8(0, response.value.len);
}

void testDecodeStringWithNulIsInvalidValue(void) {
  const uint8_t bytes[] = { 0x81, 4, 0x20, 3, 'a', 0, 'b' };
  syslink2_apiservice_frame_t frame = frame_of(bytes, sizeof(bytes));
  syslink2_apiservice_response_t response;
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_FRAME_CLASS_RESPONSE, syslink2_apiservice_codec_decode(&frame, &response));
  TEST_ASSERT_FALSE(response.value_valid);
}

void testDecodeBytesWithNulIsValid(void) {
  const uint8_t bytes[] = { 0x81, 4, 0x21, 3, 'a', 0, 'b' };
  syslink2_apiservice_frame_t frame = frame_of(bytes, sizeof(bytes));
  syslink2_apiservice_response_t response;
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_FRAME_CLASS_RESPONSE, syslink2_apiservice_codec_decode(&frame, &response));
  TEST_ASSERT_TRUE(response.value_valid);
  TEST_ASSERT_EQUAL_UINT8(SYSLINK2_APISERVICE_TYPE_BYTES, response.value.type);
  TEST_ASSERT_EQUAL_MEMORY(&bytes[4], response.value.data, 3);
}

void testDecodeLongestBytesValue(void) {
  uint8_t bytes[SYSLINK2_APISERVICE_FRAME_MAX_LEN];
  memset(bytes, 0xab, sizeof(bytes));
  bytes[0] = 0x81;
  bytes[1] = 4;
  bytes[2] = 0x21;
  bytes[3] = SYSLINK2_APISERVICE_VALUE_MAX_LEN;
  syslink2_apiservice_frame_t frame = frame_of(bytes, sizeof(bytes));
  syslink2_apiservice_response_t response;
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_FRAME_CLASS_RESPONSE, syslink2_apiservice_codec_decode(&frame, &response));
  TEST_ASSERT_TRUE(response.value_valid);
  TEST_ASSERT_EQUAL_UINT8(SYSLINK2_APISERVICE_VALUE_MAX_LEN, response.value.len);
  TEST_ASSERT_EQUAL_MEMORY(&bytes[4], response.value.data, SYSLINK2_APISERVICE_VALUE_MAX_LEN);
}

void testDecodeBadBoolIsInvalidValue(void) {
  const uint8_t bytes[] = { 0x81, 4, 0x01, 1, 2 };
  syslink2_apiservice_frame_t frame = frame_of(bytes, sizeof(bytes));
  syslink2_apiservice_response_t response;
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_FRAME_CLASS_RESPONSE, syslink2_apiservice_codec_decode(&frame, &response));
  TEST_ASSERT_FALSE(response.value_valid);
}

void testDecodeValueLengthMismatchIsMalformed(void) {
  const uint8_t too_short[] = { 0x81, 4, 0x10, 2, 1 };
  const uint8_t trailing[] = { 0x81, 4, 0x10, 1, 1, 0 };
  const uint8_t no_header[] = { 0x81, 4, 0x10 };
  syslink2_apiservice_response_t response;
  syslink2_apiservice_frame_t frame = frame_of(too_short, sizeof(too_short));
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_FRAME_CLASS_MALFORMED, syslink2_apiservice_codec_decode(&frame, &response));
  frame = frame_of(trailing, sizeof(trailing));
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_FRAME_CLASS_MALFORMED, syslink2_apiservice_codec_decode(&frame, &response));
  frame = frame_of(no_header, sizeof(no_header));
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_FRAME_CLASS_MALFORMED, syslink2_apiservice_codec_decode(&frame, &response));
}

void testDecodeError(void) {
  const uint8_t bytes[] = { 0x82, 5, 3, 2, 'n', 'o' };
  syslink2_apiservice_frame_t frame = frame_of(bytes, sizeof(bytes));
  syslink2_apiservice_response_t response;
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_FRAME_CLASS_RESPONSE, syslink2_apiservice_codec_decode(&frame, &response));
  TEST_ASSERT_EQUAL_UINT8(3, response.error_code);
  TEST_ASSERT_EQUAL_UINT8(2, response.detail_len);
  TEST_ASSERT_EQUAL_MEMORY("no", response.detail, 2);
}

void testDecodeErrorLengthMismatchIsMalformed(void) {
  const uint8_t bytes[] = { 0x82, 5, 3, 3, 'n', 'o' };
  syslink2_apiservice_frame_t frame = frame_of(bytes, sizeof(bytes));
  syslink2_apiservice_response_t response;
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_FRAME_CLASS_MALFORMED, syslink2_apiservice_codec_decode(&frame, &response));
}

void testUnsolicitedNeedsOnlyKind(void) {
  const uint8_t bytes[] = { 0xc0 };
  syslink2_apiservice_frame_t frame = frame_of(bytes, sizeof(bytes));
  syslink2_apiservice_response_t response;
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_FRAME_CLASS_UNSOLICITED, syslink2_apiservice_codec_decode(&frame, &response));
  const uint8_t notify[] = { 0xff, 1, 2, 3 };
  frame = frame_of(notify, sizeof(notify));
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_FRAME_CLASS_UNSOLICITED, syslink2_apiservice_codec_decode(&frame, &response));
}

void testOtherKindsAndShortFramesAreMalformed(void) {
  syslink2_apiservice_response_t response;
  const uint8_t request_kind[] = { 0x01, 1 };
  const uint8_t reserved[] = { 0x83, 1 };
  const uint8_t one_byte[] = { 0x80 };
  syslink2_apiservice_frame_t frame = frame_of(request_kind, sizeof(request_kind));
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_FRAME_CLASS_MALFORMED, syslink2_apiservice_codec_decode(&frame, &response));
  frame = frame_of(reserved, sizeof(reserved));
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_FRAME_CLASS_MALFORMED, syslink2_apiservice_codec_decode(&frame, &response));
  frame = frame_of(one_byte, sizeof(one_byte));
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_FRAME_CLASS_MALFORMED, syslink2_apiservice_codec_decode(&frame, &response));
  frame.len = 0;
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_FRAME_CLASS_MALFORMED, syslink2_apiservice_codec_decode(&frame, &response));
}

/* Raw responses */

static void assert_round_trip(const uint8_t *bytes, uint8_t len) {
  const syslink2_apiservice_frame_t received = frame_of(bytes, len);
  syslink2_apiservice_response_t response;
  TEST_ASSERT_EQUAL(SYSLINK2_APISERVICE_FRAME_CLASS_RESPONSE,
                    syslink2_apiservice_codec_decode(&received, &response));
  const syslink2_apiservice_frame_t raw =
    syslink2_apiservice_codec_encode_response(9u, &response);
  TEST_ASSERT_EQUAL_UINT8(len, raw.len);
  TEST_ASSERT_EQUAL_UINT8(bytes[0], raw.data[0]);
  TEST_ASSERT_EQUAL_UINT8(9u, raw.data[1]);
  if (len > 2u) {
    TEST_ASSERT_EQUAL_MEMORY(&bytes[2], &raw.data[2], len - 2u);
  }
}

void testEncodeResponseOk(void) {
  const uint8_t bytes[] = { 0x80, 5 };
  assert_round_trip(bytes, sizeof(bytes));
}

void testEncodeResponseValue(void) {
  const uint8_t bytes[] = { 0x81, 5, 0x14, 4, 0x78, 0x56, 0x34, 0x12 };
  assert_round_trip(bytes, sizeof(bytes));
}

void testEncodeResponseLongestStringValue(void) {
  uint8_t bytes[SYSLINK2_APISERVICE_FRAME_MAX_LEN];
  memset(bytes, 's', sizeof(bytes));
  bytes[0] = 0x81;
  bytes[1] = 5;
  bytes[2] = 0x20;
  bytes[3] = SYSLINK2_APISERVICE_VALUE_MAX_LEN;
  assert_round_trip(bytes, sizeof(bytes));
}

void testEncodeResponseErrorWithDetail(void) {
  const uint8_t bytes[] = { 0x82, 5, 0x03, 4, 'n', 'o', 'p', 'e' };
  assert_round_trip(bytes, sizeof(bytes));
}

void testEncodeResponseErrorWithLongestDetail(void) {
  uint8_t bytes[SYSLINK2_APISERVICE_FRAME_MAX_LEN];
  memset(bytes, 'd', sizeof(bytes));
  bytes[0] = 0x82;
  bytes[1] = 5;
  bytes[2] = 0x09;
  bytes[3] = SYSLINK2_APISERVICE_DETAIL_MAX_LEN;
  assert_round_trip(bytes, sizeof(bytes));
}
