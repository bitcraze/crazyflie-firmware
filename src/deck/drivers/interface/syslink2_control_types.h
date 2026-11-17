/**
 * Types and constants shared by the units of the Control Service.
 *
 * See docs/functional-areas/camera-deck-control.md. This header includes no
 * other project header.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

/** Link MTU of the UART camera profile: the largest frame. */
#define SYSLINK2_CONTROL_FRAME_MAX_LEN 64u
/** Largest value in scope, u32 and i32. */
#define SYSLINK2_CONTROL_VALUE_MAX_LEN 4u
/** Longest path that fits a SET frame with the largest value. */
#define SYSLINK2_CONTROL_PATH_MAX_LEN 55u
/** Longest detail that fits an ERROR frame. */
#define SYSLINK2_CONTROL_DETAIL_MAX_LEN 60u

/* Message kinds, the wire `kind` byte. */
#define SYSLINK2_CONTROL_KIND_GET 0x01u
#define SYSLINK2_CONTROL_KIND_SET 0x02u
#define SYSLINK2_CONTROL_KIND_OK 0x80u
#define SYSLINK2_CONTROL_KIND_VALUE 0x81u
#define SYSLINK2_CONTROL_KIND_ERROR 0x82u

/* Value type codes in scope. */
#define SYSLINK2_CONTROL_TYPE_BOOL 0x01u
#define SYSLINK2_CONTROL_TYPE_U8 0x10u
#define SYSLINK2_CONTROL_TYPE_I8 0x11u
#define SYSLINK2_CONTROL_TYPE_U16 0x12u
#define SYSLINK2_CONTROL_TYPE_I16 0x13u
#define SYSLINK2_CONTROL_TYPE_U32 0x14u
#define SYSLINK2_CONTROL_TYPE_I32 0x15u

/** One Control frame, the payload of one UART packet on the Control service. */
typedef struct {
  uint8_t data[SYSLINK2_CONTROL_FRAME_MAX_LEN];
  uint8_t len;
} syslink2_control_frame_t;

/** A typed value. Multi-byte integers are little-endian in data. */
typedef struct {
  uint8_t type;
  uint8_t len;
  uint8_t data[SYSLINK2_CONTROL_VALUE_MAX_LEN];
} syslink2_control_value_t;

typedef enum {
  SYSLINK2_CONTROL_OUTCOME_OK = 0,
  SYSLINK2_CONTROL_OUTCOME_VALUE,
  SYSLINK2_CONTROL_OUTCOME_ERROR,
  SYSLINK2_CONTROL_OUTCOME_TIMEOUT,
  SYSLINK2_CONTROL_OUTCOME_NOT_READY,
  SYSLINK2_CONTROL_OUTCOME_ABORTED,
  SYSLINK2_CONTROL_OUTCOME_PROTOCOL_ERROR,
  SYSLINK2_CONTROL_OUTCOME_SEND_FAILED,
} syslink2_control_outcome_t;

typedef enum {
  SYSLINK2_CONTROL_SUBMIT_ACCEPTED = 0,
  SYSLINK2_CONTROL_SUBMIT_QUEUE_FULL,
  SYSLINK2_CONTROL_SUBMIT_INVALID,
  SYSLINK2_CONTROL_SUBMIT_NOT_INITIALIZED,
} syslink2_control_submit_result_t;

typedef enum {
  SYSLINK2_CONTROL_FRAME_CLASS_RESPONSE = 0,
  SYSLINK2_CONTROL_FRAME_CLASS_UNSOLICITED,
  SYSLINK2_CONTROL_FRAME_CLASS_MALFORMED,
} syslink2_control_frame_class_t;

typedef enum {
  SYSLINK2_CONTROL_SEND_SENT = 0,
  SYSLINK2_CONTROL_SEND_BUSY,
  SYSLINK2_CONTROL_SEND_CLOSED,
  SYSLINK2_CONTROL_SEND_REJECTED,
} syslink2_control_send_result_t;

/** The result of a completed request, passed to on_complete. */
typedef struct {
  syslink2_control_outcome_t outcome;
  /** Valid only for SYSLINK2_CONTROL_OUTCOME_VALUE. */
  syslink2_control_value_t value;
  /** Valid only for SYSLINK2_CONTROL_OUTCOME_ERROR. */
  uint8_t error_code;
} syslink2_control_result_t;

struct syslink2_control_request_s;

/** One user of the Control Service. Static, outlives every request. */
typedef struct {
  /** Called in the Control task when a request completes. Must not block. */
  void (*on_complete)(const struct syslink2_control_request_s *request,
                      const syslink2_control_result_t *result);
} syslink2_control_user_t;

/** A GET or SET request from a user. */
typedef struct syslink2_control_request_s {
  /** SYSLINK2_CONTROL_KIND_GET or SYSLINK2_CONTROL_KIND_SET. */
  uint8_t operation;
  /** Owned by the user. Not read by the Control Service. */
  uint8_t user_request_id;
  /** ASCII, path_len bytes, not NUL-terminated. */
  char path[SYSLINK2_CONTROL_PATH_MAX_LEN];
  uint8_t path_len;
  /** Read only for SET. */
  syslink2_control_value_t value;
  const syslink2_control_user_t *user;
} syslink2_control_request_t;

/** A decoded OK, VALUE or ERROR frame. */
typedef struct {
  /** SYSLINK2_CONTROL_KIND_OK, _VALUE or _ERROR. */
  uint8_t kind;
  /** The deck request ID. */
  uint8_t request_id;
  /** Valid only when kind is VALUE and value_valid is true. */
  syslink2_control_value_t value;
  /** Valid only when kind is VALUE. */
  bool value_valid;
  /** Valid only when kind is ERROR. */
  uint8_t error_code;
  /** UTF-8, detail_len bytes, not NUL-terminated. Valid only for ERROR. */
  char detail[SYSLINK2_CONTROL_DETAIL_MAX_LEN];
  uint8_t detail_len;
} syslink2_control_response_t;
