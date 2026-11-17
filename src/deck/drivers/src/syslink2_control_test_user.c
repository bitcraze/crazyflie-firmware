/**
 * Temporary test user of the Control Service, for testing on hardware.
 *
 * Enabled with CONFIG_DECK_BCCAM_CONTROL_TEST. Set the params in the
 * bcCamCtlT group, then write trigger to submit. Results are printed to the
 * console and exposed in the bcCamCtlT log group.
 */
#include <stdint.h>
#include <string.h>

#include "log.h"
#include "param.h"
#include "syslink2_control_service.h"

#define DEBUG_MODULE "CTLTEST"
#include "debug.h"

static const char *const paths[] = {
  "stream.max_mjpeg_quality",
  "stream.mjpeg_quality",
  "meta.schema_modules",
  "no.such.path",
};
#define PATH_COUNT (sizeof(paths) / sizeof(paths[0]))

/* Params, written by the client. */
static uint8_t pathIndex;
static uint8_t operation = 0x01u;
static uint8_t valueType = 0x10u;
static int32_t value;
static uint8_t useRaw;
static uint8_t count = 1u;
static uint8_t trigger;

/* Results, written in the Control task or the param task. */
static uint8_t nextUserId;
static uint8_t lastSubmit;
static uint8_t lastId;
static uint8_t lastOutcome;
static int32_t lastValue;
static uint8_t lastType;
static uint8_t lastErrorCode;
static uint8_t lastRawLen;
static uint16_t completions;
static uint16_t accepted;
static uint8_t ready;

static int32_t decodeValue(const syslink2_control_value_t *v) {
  switch (v->type) {
    case 0x01u:
    case 0x10u:
      return v->data[0];
    case 0x11u:
      return (int8_t)v->data[0];
    case 0x12u:
      return (uint16_t)(v->data[0] | (v->data[1] << 8));
    case 0x13u:
      return (int16_t)(v->data[0] | (v->data[1] << 8));
    default:
      return (int32_t)((uint32_t)v->data[0] | ((uint32_t)v->data[1] << 8) |
                       ((uint32_t)v->data[2] << 16) |
                       ((uint32_t)v->data[3] << 24));
  }
}

static uint8_t lenForType(uint8_t type) {
  switch (type) {
    case 0x12u:
    case 0x13u:
      return 2u;
    case 0x14u:
    case 0x15u:
      return 4u;
    default:
      return 1u;
  }
}

static void onComplete(const syslink2_control_request_t *request,
                       const syslink2_control_result_t *result) {
  lastId = request->user_request_id;
  lastOutcome = (uint8_t)result->outcome;
  lastType = result->value.type;
  lastValue = result->outcome == SYSLINK2_CONTROL_OUTCOME_VALUE ?
    decodeValue(&result->value) : 0;
  lastErrorCode = result->error_code;
  lastRawLen = 0u;
  completions++;
  DEBUG_PRINT("id=%u op=%u outcome=%u value=%ld err=%u\n",
              request->user_request_id, request->operation, lastOutcome,
              (long)lastValue, lastErrorCode);
}

static void onCompleteRaw(syslink2_control_outcome_t outcome,
                          uint8_t userRequestId, const uint8_t *data,
                          uint8_t len) {
  lastId = userRequestId;
  lastOutcome = (uint8_t)outcome;
  lastRawLen = len;
  lastType = 0u;
  lastValue = 0;
  lastErrorCode = 0u;
  if (outcome == SYSLINK2_CONTROL_OUTCOME_VALUE && len >= 4u) {
    syslink2_control_value_t v = { .type = data[2], .len = data[3] };
    memcpy(v.data, &data[4], v.len <= sizeof(v.data) ? v.len : sizeof(v.data));
    lastType = v.type;
    lastValue = decodeValue(&v);
  } else if (outcome == SYSLINK2_CONTROL_OUTCOME_ERROR && len >= 3u) {
    lastErrorCode = data[2];
  }
  completions++;
  DEBUG_PRINT("raw id=%u outcome=%u len=%u value=%ld err=%u\n", userRequestId,
              lastOutcome, len, (long)lastValue, lastErrorCode);
}

static syslink2_control_submit_result_t submitOne(uint8_t userId) {
  const char *path = paths[pathIndex < PATH_COUNT ? pathIndex : 0u];
  const uint8_t pathLen = (uint8_t)strlen(path);
  syslink2_control_value_t v = { .type = valueType, .len = lenForType(valueType) };
  for (uint8_t i = 0u; i < v.len; i++) {
    v.data[i] = (uint8_t)((uint32_t)value >> (8u * i));
  }

  if (useRaw) {
    uint8_t raw[SYSLINK2_CONTROL_FRAME_MAX_LEN];
    uint8_t len = 0u;
    raw[len++] = operation;
    raw[len++] = userId;
    raw[len++] = pathLen;
    memcpy(&raw[len], path, pathLen);
    len = (uint8_t)(len + pathLen);
    if (operation == 0x02u) {
      raw[len++] = v.type;
      raw[len++] = v.len;
      memcpy(&raw[len], v.data, v.len);
      len = (uint8_t)(len + v.len);
    }
    return syslink2_control_service_submit_raw(raw, len, onCompleteRaw);
  }

  syslink2_control_request_t request = {
    .operation = operation,
    .user_request_id = userId,
    .path_len = pathLen,
    .value = v,
  };
  memcpy(request.path, path, pathLen);
  return syslink2_control_service_submit(&request, onComplete);
}

static void triggerChanged(void) {
  if (trigger == 0u) {
    return;
  }
  trigger = 0u;
  ready = syslink2_control_service_is_ready() ? 1u : 0u;
  for (uint8_t i = 0u; i < count; i++) {
    const uint8_t userId = nextUserId++;
    const syslink2_control_submit_result_t r = submitOne(userId);
    lastSubmit = (uint8_t)r;
    if (r == SYSLINK2_CONTROL_SUBMIT_ACCEPTED) {
      accepted++;
    }
    DEBUG_PRINT("submit id=%u result=%u ready=%u\n", userId, lastSubmit, ready);
  }
}

static uint8_t readyLog(uint32_t timestamp, void *data) {
  (void)timestamp;
  (void)data;
  return syslink2_control_service_is_ready() ? 1u : 0u;
}

static logByFunction_t readyLogDef = { .acquireUInt8 = readyLog, .data = NULL };

PARAM_GROUP_START(bcCamCtlT)
PARAM_ADD(PARAM_UINT8, path, &pathIndex)
PARAM_ADD(PARAM_UINT8, op, &operation)
PARAM_ADD(PARAM_UINT8, type, &valueType)
PARAM_ADD(PARAM_INT32, value, &value)
PARAM_ADD(PARAM_UINT8, raw, &useRaw)
PARAM_ADD(PARAM_UINT8, count, &count)
PARAM_ADD_WITH_CALLBACK(PARAM_UINT8, trigger, &trigger, triggerChanged)
PARAM_GROUP_STOP(bcCamCtlT)

LOG_GROUP_START(bcCamCtlT)
LOG_ADD_BY_FUNCTION(LOG_UINT8, ready, &readyLogDef)
LOG_ADD(LOG_UINT8, lastSubmit, &lastSubmit)
LOG_ADD(LOG_UINT8, lastId, &lastId)
LOG_ADD(LOG_UINT8, lastOutcome, &lastOutcome)
LOG_ADD(LOG_INT32, lastValue, &lastValue)
LOG_ADD(LOG_UINT8, lastType, &lastType)
LOG_ADD(LOG_UINT8, lastErr, &lastErrorCode)
LOG_ADD(LOG_UINT8, lastRawLen, &lastRawLen)
LOG_ADD(LOG_UINT16, completions, &completions)
LOG_ADD(LOG_UINT16, accepted, &accepted)
LOG_GROUP_STOP(bcCamCtlT)
