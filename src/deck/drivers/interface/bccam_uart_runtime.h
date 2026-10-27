#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "bccam_uart_frame.h"
#include "bccam_uart_link.h"
#include "bccam_uart_rx_collector.h"

/* true means the complete packet finished local transmission. false means
 * ambiguous completion and forces fail-stop Link Fault; the packet is never
 * retried. */
typedef bool (*bccam_uart_runtime_send_t)(void *context,
                                          const uint8_t *data,
                                          uint32_t length);

#define BCCAM_UART_CONTROL_MAX_SCHEMA_MODULES 4u
#define BCCAM_UART_CONTROL_SCHEMA_NAMESPACE_MAX_LEN 24u
#define BCCAM_UART_CONTROL_SCHEMA_CONTRACT_ID_MAX_LEN 48u

typedef struct {
  char namespace[BCCAM_UART_CONTROL_SCHEMA_NAMESPACE_MAX_LEN];
  char contract_id[BCCAM_UART_CONTROL_SCHEMA_CONTRACT_ID_MAX_LEN];
  uint8_t major;
  uint8_t minor;
} bccam_uart_control_schema_module_t;

typedef enum {
  BCCAM_UART_CONTROL_PROBE_WAITING_FOR_LINK = 0,
  BCCAM_UART_CONTROL_PROBE_WAITING_FOR_CONTROL_SERVICE,
  BCCAM_UART_CONTROL_PROBE_WAITING_FOR_LOCAL_RX_CREDIT,
  BCCAM_UART_CONTROL_PROBE_WAITING_FOR_TARGET_TX_CREDIT,
  BCCAM_UART_CONTROL_PROBE_WAITING_FOR_RESPONSE,
  BCCAM_UART_CONTROL_PROBE_DONE,
} bccam_uart_control_probe_phase_t;

typedef enum {
  BCCAM_UART_FIRMWARE_STARTUP_WAITING = 0,
  BCCAM_UART_FIRMWARE_STARTUP_READY,
  BCCAM_UART_FIRMWARE_STARTUP_INCOMPATIBLE,
  BCCAM_UART_FIRMWARE_STARTUP_ABNORMAL,
} bccam_uart_firmware_startup_result_t;

typedef struct bccam_uart_runtime_t {
  bccam_uart_link_endpoint_t link;
  bccam_uart_frame_parser_t parser;
  uint16_t rx_bytes;
  uint16_t tx_bytes;
  uint16_t tx_flushes;
  int last_error;
  bool restart_requested;
  uint8_t next_descriptor_ordinal;
  bool control_service_bound;
  uint8_t control_service_id;
  bool control_rx_credit_opened;
  bool control_request_sent;
  bool control_probe_done;
  bool control_rx_release_pending;
  uint16_t control_rx_malformed;
  uint8_t control_schema_module_count;
  bccam_uart_control_schema_module_t
    control_schema_modules[BCCAM_UART_CONTROL_MAX_SCHEMA_MODULES];
} bccam_uart_runtime_t;

void bccam_uart_runtime_init(bccam_uart_runtime_t *runtime);

int bccam_uart_runtime_on_firmware_boot(bccam_uart_runtime_t *runtime);

void bccam_uart_runtime_on_bootloader_enter(bccam_uart_runtime_t *runtime);

int bccam_uart_runtime_receive_byte(bccam_uart_runtime_t *runtime, uint8_t byte);

int bccam_uart_runtime_on_raw_frame(bccam_uart_runtime_t *runtime,
                                    const uint8_t *frame,
                                    uint16_t frame_len);

int bccam_uart_runtime_on_rx_fault(bccam_uart_runtime_t *runtime,
                                   bccam_uart_rx_fault_t fault);

int bccam_uart_runtime_flush_tx(bccam_uart_runtime_t *runtime,
                                bccam_uart_runtime_send_t send,
                                void *send_context);

int bccam_uart_runtime_pump_tx(bccam_uart_runtime_t *runtime,
                               bccam_uart_runtime_send_t send,
                               void *send_context);

int bccam_uart_runtime_step_control_probe(bccam_uart_runtime_t *runtime);

bool bccam_uart_runtime_control_probe_done(const bccam_uart_runtime_t *runtime);

bool bccam_uart_runtime_control_service_bound(const bccam_uart_runtime_t *runtime);

/**
 * Check whether this session discovered a compatible Console service.
 *
 * @param runtime Runtime containing the current Link session.
 * @return true when a compatible bitcraze.console service is bound, otherwise
 *         false. A NULL runtime returns false.
 */
bool bccam_uart_runtime_console_service_bound(const bccam_uart_runtime_t *runtime);

/**
 * Make one receive slot available to the bound Console service.
 *
 * Opening an already advertised slot is idempotent. Console and Control own
 * independent receive staging slots, so either service may make progress
 * while the other is stalled.
 *
 * @param runtime Runtime that owns the Link endpoint and receive slot.
 * @return BCCAM_UART_OK when the slot is open or already open;
 *         BCCAM_UART_ERR_BAD_ARGUMENT for NULL;
 *         BCCAM_UART_ERR_UNKNOWN_SERVICE when Console is not bound; otherwise
 *         the Link error produced while queuing the credit update.
 */
int bccam_uart_runtime_open_console_rx(bccam_uart_runtime_t *runtime);

/**
 * Take a pending Console frame from the runtime receive staging slot.
 *
 * Taking a frame removes it from the staging slot but does not return its Link
 * credit. After the downstream consumer has accepted the complete frame, the
 * caller must call bccam_uart_runtime_release_console_rx().
 *
 * @param runtime Runtime that owns the pending receive frame.
 * @param payload Buffer that receives the frame payload when one is present.
 *                Must not be NULL and must remain valid for this call.
 * @param payload_capacity Number of bytes available in payload.
 * @param payload_len Output set to the received payload length, or zero when
 *                    no Console frame is pending. Must not be NULL.
 * @param unit_present Output set true only when a frame was copied. Must not
 *                     be NULL.
 * @return BCCAM_UART_OK when a frame was taken or none was pending;
 *         BCCAM_UART_ERR_BAD_ARGUMENT for invalid pointers;
 *         BCCAM_UART_ERR_UNKNOWN_SERVICE when Console is not bound;
 *         BCCAM_UART_ERR_BUFFER_TOO_SMALL when payload cannot hold the frame;
 *         otherwise the error returned by the Link receive operation.
 */
int bccam_uart_runtime_take_console_rx(bccam_uart_runtime_t *runtime,
                                       uint8_t *payload,
                                       size_t payload_capacity,
                                       uint16_t *payload_len,
                                       bool *unit_present);

/**
 * Return the consumed Console receive slot to the peer.
 *
 * Call this exactly once for each frame successfully taken and fully accepted
 * by the downstream consumer. This advertises one new Console receive credit.
 *
 * @param runtime Runtime that owns the consumed Console receive slot.
 * @return BCCAM_UART_OK when the credit update was queued;
 *         BCCAM_UART_ERR_BAD_ARGUMENT for NULL;
 *         BCCAM_UART_ERR_UNKNOWN_SERVICE when Console is not bound; otherwise
 *         the Link error produced while releasing the slot.
 */
int bccam_uart_runtime_release_console_rx(bccam_uart_runtime_t *runtime);

uint8_t bccam_uart_runtime_control_service_id(const bccam_uart_runtime_t *runtime);

bccam_uart_control_probe_phase_t bccam_uart_runtime_control_probe_phase(
  const bccam_uart_runtime_t *runtime);

bccam_uart_firmware_startup_result_t bccam_uart_runtime_firmware_startup_result(
  const bccam_uart_runtime_t *runtime);

uint8_t bccam_uart_runtime_control_tx_credit(const bccam_uart_runtime_t *runtime);

uint8_t bccam_uart_runtime_control_rx_slots(const bccam_uart_runtime_t *runtime);

uint8_t bccam_uart_runtime_control_schema_module_count(
  const bccam_uart_runtime_t *runtime);

const bccam_uart_control_schema_module_t *
bccam_uart_runtime_control_schema_module(const bccam_uart_runtime_t *runtime,
                                         uint8_t index);

uint16_t bccam_uart_runtime_get_control_malformed_count(
  const bccam_uart_runtime_t *runtime);

bccam_uart_link_state_t bccam_uart_runtime_get_state(const bccam_uart_runtime_t *runtime);

const bccam_uart_link_counters_t *bccam_uart_runtime_get_counters(
  const bccam_uart_runtime_t *runtime);
