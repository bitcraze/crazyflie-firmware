#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "bccam_deck_controller.h"

typedef struct {
  bccam_deck_controller_t *deck_controller;
  bool flash_erased;
  uint8_t write_buffer[1024];
  uint16_t write_buffer_used;
  uint32_t write_buffer_base;
  uint32_t bytes_written;
  uint32_t image_base;
  uint32_t image_size;
  bool image_complete;
} bccam_bootloader_uart_client_t;

void bccam_bootloader_uart_client_init(
  bccam_bootloader_uart_client_t *client,
  bccam_deck_controller_t *deck_controller);

bool bccam_bootloader_uart_client_enter(
  bccam_bootloader_uart_client_t *client);

void bccam_bootloader_uart_client_reset_flash_session(
  bccam_bootloader_uart_client_t *client);

bool bccam_bootloader_uart_client_write_flash(
  bccam_bootloader_uart_client_t *client,
  uint32_t mem_addr,
  uint8_t write_len,
  const uint8_t *buffer,
  uint32_t new_fw_size);

bool bccam_bootloader_uart_client_flash_completed(
  const bccam_bootloader_uart_client_t *client);

bool bccam_bootloader_uart_client_read_flash(
  bccam_bootloader_uart_client_t *client,
  uint32_t mem_addr,
  uint8_t read_len,
  uint8_t *buffer);

#if defined(UNIT_TEST) || defined(UNIT_TEST_MODE)
	typedef enum {
	  BCCAM_BOOTLOADER_UART_CLIENT_TEST_DECK_BEGIN_BOOT = 0,
	  BCCAM_BOOTLOADER_UART_CLIENT_TEST_DECK_RELEASE_BOOT,
	  BCCAM_BOOTLOADER_UART_CLIENT_TEST_UART_INIT,
	  BCCAM_BOOTLOADER_UART_CLIENT_TEST_UART_SET_BAUDRATE,
	  BCCAM_BOOTLOADER_UART_CLIENT_TEST_UART_SEND,
	  BCCAM_BOOTLOADER_UART_CLIENT_TEST_UART_SEND_DMA,
	  BCCAM_BOOTLOADER_UART_CLIENT_TEST_UART_RECEIVE,
  BCCAM_BOOTLOADER_UART_CLIENT_TEST_UART_DRAIN,
} bccam_bootloader_uart_client_test_event_t;

typedef struct {
  bccam_bootloader_uart_client_test_event_t event;
  bccam_deck_boot_mode_t boot_mode;
  uint32_t value;
  uint32_t length;
  uint8_t byte;
  uint8_t bytes[1024];
} bccam_bootloader_uart_client_test_trace_entry_t;

void bccam_bootloader_uart_client_test_trace_reset(void);
uint8_t bccam_bootloader_uart_client_test_trace_count(void);
const bccam_bootloader_uart_client_test_trace_entry_t *
bccam_bootloader_uart_client_test_trace_entry(uint8_t index);
void bccam_bootloader_uart_client_test_queue_rx(const uint8_t *bytes,
                                                uint32_t length);
#endif
