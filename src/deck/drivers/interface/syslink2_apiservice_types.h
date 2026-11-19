/**
 * Types and constants shared by the units of the API Service.
 *
 * See docs/functional-areas/camera-deck-apiservice.md. This header includes no
 * other project header.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

/** Link MTU of the UART camera profile: the largest frame. */
#define SYSLINK2_APISERVICE_FRAME_MAX_LEN 64u
/** Largest value in scope, u32 and i32. */
#define SYSLINK2_APISERVICE_VALUE_MAX_LEN 4u
/** Longest path that fits a SET frame with the largest value. */
#define SYSLINK2_APISERVICE_PATH_MAX_LEN 55u
/** Longest detail that fits an ERROR frame. */
#define SYSLINK2_APISERVICE_DETAIL_MAX_LEN 60u

/* Message kinds, the wire `kind` byte. */
#define SYSLINK2_APISERVICE_KIND_GET 0x01u
#define SYSLINK2_APISERVICE_KIND_SET 0x02u
#define SYSLINK2_APISERVICE_KIND_OK 0x80u
#define SYSLINK2_APISERVICE_KIND_VALUE 0x81u
#define SYSLINK2_APISERVICE_KIND_ERROR 0x82u

/* Value type codes in scope. */
#define SYSLINK2_APISERVICE_TYPE_BOOL 0x01u
#define SYSLINK2_APISERVICE_TYPE_U8 0x10u
#define SYSLINK2_APISERVICE_TYPE_I8 0x11u
#define SYSLINK2_APISERVICE_TYPE_U16 0x12u
#define SYSLINK2_APISERVICE_TYPE_I16 0x13u
#define SYSLINK2_APISERVICE_TYPE_U32 0x14u
#define SYSLINK2_APISERVICE_TYPE_I32 0x15u

/** One API Service frame, the payload of one UART packet on the API Service. */
typedef struct {
  uint8_t data[SYSLINK2_APISERVICE_FRAME_MAX_LEN];
  uint8_t len;
} syslink2_apiservice_frame_t;

/** A typed value. Multi-byte integers are little-endian in data. */
typedef struct {
  uint8_t type;
  uint8_t len;
  uint8_t data[SYSLINK2_APISERVICE_VALUE_MAX_LEN];
} syslink2_apiservice_value_t;

typedef enum {
  SYSLINK2_APISERVICE_OUTCOME_OK = 0,
  SYSLINK2_APISERVICE_OUTCOME_VALUE,
  SYSLINK2_APISERVICE_OUTCOME_ERROR,
  SYSLINK2_APISERVICE_OUTCOME_TIMEOUT,
  SYSLINK2_APISERVICE_OUTCOME_NOT_READY,
  SYSLINK2_APISERVICE_OUTCOME_ABORTED,
  SYSLINK2_APISERVICE_OUTCOME_PROTOCOL_ERROR,
  SYSLINK2_APISERVICE_OUTCOME_SEND_FAILED,
} syslink2_apiservice_outcome_t;

typedef enum {
  SYSLINK2_APISERVICE_SUBMIT_ACCEPTED = 0,
  SYSLINK2_APISERVICE_SUBMIT_QUEUE_FULL,
  SYSLINK2_APISERVICE_SUBMIT_INVALID,
  SYSLINK2_APISERVICE_SUBMIT_NOT_INITIALIZED,
} syslink2_apiservice_submit_result_t;

typedef enum {
  SYSLINK2_APISERVICE_FRAME_CLASS_RESPONSE = 0,
  SYSLINK2_APISERVICE_FRAME_CLASS_UNSOLICITED,
  SYSLINK2_APISERVICE_FRAME_CLASS_MALFORMED,
} syslink2_apiservice_frame_class_t;

typedef enum {
  SYSLINK2_APISERVICE_SEND_SENT = 0,
  SYSLINK2_APISERVICE_SEND_BUSY,
  SYSLINK2_APISERVICE_SEND_CLOSED,
  SYSLINK2_APISERVICE_SEND_REJECTED,
} syslink2_apiservice_send_result_t;

/** The result of a completed request, passed to on_complete. */
typedef struct {
  syslink2_apiservice_outcome_t outcome;
  /** Valid only for SYSLINK2_APISERVICE_OUTCOME_VALUE. */
  syslink2_apiservice_value_t value;
  /** Valid only for SYSLINK2_APISERVICE_OUTCOME_ERROR. */
  uint8_t error_code;
} syslink2_apiservice_result_t;

/** A GET or SET request from a user. */
typedef struct {
  /** SYSLINK2_APISERVICE_KIND_GET or SYSLINK2_APISERVICE_KIND_SET. */
  uint8_t operation;
  /** Owned by the user. Not read by the API Service. */
  uint8_t user_request_id;
  /** ASCII, path_len bytes, not NUL-terminated. */
  char path[SYSLINK2_APISERVICE_PATH_MAX_LEN];
  uint8_t path_len;
  /** Read only for SET. */
  syslink2_apiservice_value_t value;
} syslink2_apiservice_request_t;

/**
 * Completion of a request from syslink2_apiservice_submit. Called in the
 * API Service task. Must not block.
 */
typedef void (*syslink2_apiservice_on_complete_t)(
  const syslink2_apiservice_request_t *request,
  const syslink2_apiservice_result_t *result);

/**
 * Completion of a raw request from syslink2_apiservice_submit_raw.
 * Called in the API Service task. Must not block. data and len hold the raw
 * response, an OK, VALUE or ERROR frame whose request_id byte is
 * user_request_id, only for SYSLINK2_APISERVICE_OUTCOME_OK, _VALUE and _ERROR.
 * Otherwise data is NULL and len is 0. data is valid only during the call.
 */
typedef void (*syslink2_apiservice_on_complete_raw_t)(
  syslink2_apiservice_outcome_t outcome, uint8_t user_request_id,
  const uint8_t *data, uint8_t len);

/**
 * A submitted request and the callback to complete it with. Exactly one of
 * on_complete and on_complete_raw is set; on_complete_raw for a raw request.
 * Internal to the API Service.
 */
typedef struct {
  syslink2_apiservice_request_t request;
  syslink2_apiservice_on_complete_t on_complete;
  syslink2_apiservice_on_complete_raw_t on_complete_raw;
} syslink2_apiservice_queue_entry_t;

/** A decoded OK, VALUE or ERROR frame. */
typedef struct {
  /** SYSLINK2_APISERVICE_KIND_OK, _VALUE or _ERROR. */
  uint8_t kind;
  /** The deck request ID. */
  uint8_t request_id;
  /** Valid only when kind is VALUE and value_valid is true. */
  syslink2_apiservice_value_t value;
  /** Valid only when kind is VALUE. */
  bool value_valid;
  /** Valid only when kind is ERROR. */
  uint8_t error_code;
  /** UTF-8, detail_len bytes, not NUL-terminated. Valid only for ERROR. */
  char detail[SYSLINK2_APISERVICE_DETAIL_MAX_LEN];
  uint8_t detail_len;
} syslink2_apiservice_response_t;
