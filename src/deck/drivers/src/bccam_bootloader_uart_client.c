#include "bccam_bootloader_uart_client.h"

#include <stddef.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#if !defined(UNIT_TEST) && !defined(UNIT_TEST_MODE)
#include "uart1.h"

#define DEBUG_MODULE "BCCAM"
#include "debug.h"
#else
#define DEBUG_PRINT(...)
#endif

#define BCCAM_ISP_BAUDRATE        500000
#define ISP_HANDSHAKE_BYTE        0x55
#define ISP_HANDSHAKE_COUNT       32
#define ISP_CMD_TIMEOUT           M2T(5000)
#define ISP_CMD_GET_BOOTINFO      0x10
#define ISP_CMD_FLASH_ERASE       0x30
#define ISP_CMD_FLASH_WRITE       0x31
#define ISP_CMD_FLASH_READ        0x32
#define ISP_CMD_FLASH_SET_PARA    0x3B
#define ISP_ACK_OK_L              0x4F
#define ISP_ACK_OK_H              0x4B
#define ISP_ACK_SCAN_MAX          2048
#define BCCAM_WRITE_BUF_SIZE      1024

static bool tick_has_reached(TickType_t now, TickType_t deadline) {
  return (int32_t)(now - deadline) >= 0;
}

#if defined(UNIT_TEST) || defined(UNIT_TEST_MODE)
#define TEST_TRACE_MAX 64
#define TEST_RX_MAX 4096

static bccam_bootloader_uart_client_test_trace_entry_t test_trace[TEST_TRACE_MAX];
static uint8_t test_trace_count;
static uint8_t test_rx[TEST_RX_MAX];
static uint32_t test_rx_head;
static uint32_t test_rx_tail;
static TickType_t test_tick;

static void test_trace_append(bccam_bootloader_uart_client_test_event_t event) {
  if (test_trace_count >= TEST_TRACE_MAX) {
    return;
  }

  memset(&test_trace[test_trace_count], 0, sizeof(test_trace[test_trace_count]));
  test_trace[test_trace_count].event = event;
  test_trace_count++;
}

static bccam_bootloader_uart_client_test_trace_entry_t *
test_trace_append_entry(bccam_bootloader_uart_client_test_event_t event) {
  if (test_trace_count >= TEST_TRACE_MAX) {
    return NULL;
  }

  memset(&test_trace[test_trace_count], 0, sizeof(test_trace[test_trace_count]));
  test_trace[test_trace_count].event = event;
  return &test_trace[test_trace_count++];
}

void bccam_bootloader_uart_client_test_trace_reset(void) {
  memset(test_trace, 0, sizeof(test_trace));
  test_trace_count = 0;
  test_rx_head = 0;
  test_rx_tail = 0;
  test_tick = 0;
  bccam_deck_controller_test_trace_reset();
}

uint8_t bccam_bootloader_uart_client_test_trace_count(void) {
  return test_trace_count;
}

const bccam_bootloader_uart_client_test_trace_entry_t *
bccam_bootloader_uart_client_test_trace_entry(uint8_t index) {
  if (index >= test_trace_count) {
    return NULL;
  }

  return &test_trace[index];
}

void bccam_bootloader_uart_client_test_queue_rx(const uint8_t *bytes,
                                                uint32_t length) {
  if (bytes == NULL) {
    return;
  }

  for (uint32_t i = 0; i < length && test_rx_tail < TEST_RX_MAX; i++) {
    test_rx[test_rx_tail++] = bytes[i];
  }
}

static TickType_t bootloader_now_ticks(void) {
  return test_tick++;
}

static void bootloader_uart_init(uint32_t baudrate) {
  test_trace_append(BCCAM_BOOTLOADER_UART_CLIENT_TEST_UART_INIT);
  bccam_bootloader_uart_client_test_trace_entry_t *entry =
    test_trace_append_entry(BCCAM_BOOTLOADER_UART_CLIENT_TEST_UART_SET_BAUDRATE);
  if (entry != NULL) {
    entry->value = baudrate;
  }
}

static void bootloader_uart_send(uint32_t size, const uint8_t *data) {
  bccam_bootloader_uart_client_test_trace_entry_t *entry =
    test_trace_append_entry(BCCAM_BOOTLOADER_UART_CLIENT_TEST_UART_SEND);
  if (entry != NULL) {
    entry->length = size;
    if (data != NULL) {
      const uint32_t copy_len =
        (size <= sizeof(entry->bytes)) ? size : sizeof(entry->bytes);
      memcpy(entry->bytes, data, copy_len);
    }
  }
}

static void bootloader_uart_send_dma(uint32_t size, const uint8_t *data) {
  bccam_bootloader_uart_client_test_trace_entry_t *entry =
    test_trace_append_entry(BCCAM_BOOTLOADER_UART_CLIENT_TEST_UART_SEND_DMA);
  if (entry != NULL) {
    entry->length = size;
    if (data != NULL) {
      const uint32_t copy_len =
        (size <= sizeof(entry->bytes)) ? size : sizeof(entry->bytes);
      memcpy(entry->bytes, data, copy_len);
    }
  }
}

static bool bootloader_uart_recv(uint8_t *byte, uint32_t timeout_ticks) {
  (void)timeout_ticks;
  if (byte == NULL || test_rx_head >= test_rx_tail) {
    return false;
  }

  *byte = test_rx[test_rx_head++];
  bccam_bootloader_uart_client_test_trace_entry_t *entry =
    test_trace_append_entry(BCCAM_BOOTLOADER_UART_CLIENT_TEST_UART_RECEIVE);
  if (entry != NULL) {
    entry->byte = *byte;
  }
  return true;
}
#else
static TickType_t bootloader_now_ticks(void) {
  return xTaskGetTickCount();
}

static void bootloader_uart_init(uint32_t baudrate) {
  if (uart1Test()) {
    uart1SetBaudrate(baudrate);
  } else {
    uart1Init(baudrate);
  }
}

static void bootloader_uart_send(uint32_t size, const uint8_t *data) {
  uart1SendData(size, (uint8_t *)data);
}

static void bootloader_uart_send_dma(uint32_t size, const uint8_t *data) {
  uart1SendDmaIfAvailable(size, (uint8_t *)data);
}

static bool bootloader_uart_recv(uint8_t *byte, uint32_t timeout_ticks) {
  return uart1GetDataWithTimeout(byte, timeout_ticks);
}
#endif

static bool bootloader_deck_begin_boot(bccam_deck_controller_t *deck_controller,
                                       bccam_deck_boot_mode_t mode,
                                       uint32_t hold_ticks,
                                       uint32_t setup_ticks) {
#if defined(UNIT_TEST) || defined(UNIT_TEST_MODE)
  bccam_bootloader_uart_client_test_trace_entry_t *entry =
    test_trace_append_entry(BCCAM_BOOTLOADER_UART_CLIENT_TEST_DECK_BEGIN_BOOT);
  if (entry != NULL) {
    entry->boot_mode = mode;
  }
#endif
  return bccam_deck_controller_begin_boot(deck_controller,
                                          mode,
                                          hold_ticks,
                                          setup_ticks);
}

static bool bootloader_deck_release_boot(bccam_deck_controller_t *deck_controller,
                                         uint32_t wait_ticks) {
#if defined(UNIT_TEST) || defined(UNIT_TEST_MODE)
  test_trace_append(BCCAM_BOOTLOADER_UART_CLIENT_TEST_DECK_RELEASE_BOOT);
#endif
  return bccam_deck_controller_release_boot(deck_controller, wait_ticks);
}

static bool isp_recv_bytes(uint8_t *buf, uint32_t len, uint32_t timeout_ticks) {
  for (uint32_t i = 0; i < len; i++) {
    if (!bootloader_uart_recv(&buf[i], timeout_ticks)) {
      return false;
    }
  }
  return true;
}

static void isp_drain_rx(uint32_t timeout_ms) {
#if defined(UNIT_TEST) || defined(UNIT_TEST_MODE)
  (void)timeout_ms;
  test_trace_append(BCCAM_BOOTLOADER_UART_CLIENT_TEST_UART_DRAIN);
#else
  uint8_t dummy;
  const TickType_t end = xTaskGetTickCount() + M2T(timeout_ms);
  while (!tick_has_reached(xTaskGetTickCount(), end)) {
    if (!uart1GetDataWithTimeout(&dummy, M2T(5))) {
      break;
    }
  }
#endif
}

static void isp_send_cmd(uint8_t cmd_id,
                         const uint8_t *payload,
                         uint16_t payload_len) {
  uint8_t header[4];
  header[0] = cmd_id;
  header[2] = (uint8_t)(payload_len & 0xFF);
  header[3] = (uint8_t)((payload_len >> 8) & 0xFF);

  uint8_t checksum = header[2] + header[3];
  for (uint16_t i = 0; i < payload_len; i++) {
    checksum += payload[i];
  }
  header[1] = checksum;

  bootloader_uart_send(sizeof(header), header);
  if (payload_len > 0 && payload != NULL) {
    bootloader_uart_send(payload_len, payload);
  }
}

static bool isp_wait_ack(void) {
  uint8_t prev = 0;
  bool have_prev = false;
  uint32_t scanned = 0;

  for (uint32_t i = 0; i < ISP_ACK_SCAN_MAX; i++) {
    uint8_t b;
    if (!bootloader_uart_recv(&b, ISP_CMD_TIMEOUT)) {
      DEBUG_PRINT("ISP: No ACK after %lu bytes\n", (unsigned long)scanned);
      return false;
    }
    scanned++;

    if (have_prev && prev == ISP_ACK_OK_L && b == ISP_ACK_OK_H) {
      return true;
    }
    if (have_prev && prev == 0x46 /* F */ && b == 0x4C /* L */) {
      uint8_t err_bytes[2] = { 0 };
      isp_recv_bytes(err_bytes, sizeof(err_bytes), M2T(100));
      const uint16_t err_code =
        (uint16_t)err_bytes[0] | ((uint16_t)err_bytes[1] << 8);
      DEBUG_PRINT("ISP: NACK 0x%04X (after %lu skipped)\n",
                  err_code, (unsigned long)(scanned - 2));
      return false;
    }
    prev = b;
    have_prev = true;
  }

  DEBUG_PRINT("ISP: ACK scan exhausted\n");
  return false;
}

static bool isp_flash_erase(uint32_t start_addr, uint32_t end_addr) {
  uint8_t payload[8];
  memcpy(&payload[0], &start_addr, 4);
  memcpy(&payload[4], &end_addr, 4);

  isp_send_cmd(ISP_CMD_FLASH_ERASE, payload, sizeof(payload));
  return isp_wait_ack();
}

static bool isp_flash_write(uint32_t addr, const uint8_t *data, uint16_t len) {
  const uint16_t total_len = 4 + len;
  uint8_t header[8];
  header[0] = ISP_CMD_FLASH_WRITE;
  header[2] = (uint8_t)(total_len & 0xFF);
  header[3] = (uint8_t)((total_len >> 8) & 0xFF);
  memcpy(&header[4], &addr, 4);

  uint8_t checksum = header[2] + header[3];
  for (uint8_t i = 4; i < sizeof(header); i++) {
    checksum += header[i];
  }
  for (uint16_t i = 0; i < len; i++) {
    checksum += data[i];
  }
  header[1] = checksum;

  bootloader_uart_send(sizeof(header), header);
  bootloader_uart_send_dma(len, data);

  return isp_wait_ack();
}

static bool isp_flash_read(uint32_t addr, uint8_t *data, uint16_t len) {
  uint8_t payload[8];
  const uint32_t read_len = len;
  memcpy(&payload[0], &addr, 4);
  memcpy(&payload[4], &read_len, 4);

  isp_send_cmd(ISP_CMD_FLASH_READ, payload, sizeof(payload));
  if (!isp_wait_ack()) {
    return false;
  }

  uint8_t len_bytes[2];
  if (!isp_recv_bytes(len_bytes, sizeof(len_bytes), ISP_CMD_TIMEOUT)) {
    return false;
  }

  const uint16_t resp_len =
    (uint16_t)len_bytes[0] | ((uint16_t)len_bytes[1] << 8);
  if (resp_len != len) {
    DEBUG_PRINT("ISP: Read length mismatch (%u vs %u)\n", resp_len, len);
    isp_drain_rx(100);
    return false;
  }

  return isp_recv_bytes(data, len, ISP_CMD_TIMEOUT);
}

void bccam_bootloader_uart_client_init(
  bccam_bootloader_uart_client_t *client,
  bccam_deck_controller_t *deck_controller) {
  if (client == NULL) {
    return;
  }

  client->deck_controller = deck_controller;
  bccam_bootloader_uart_client_reset_flash_session(client);
}

bool bccam_bootloader_uart_client_enter(
  bccam_bootloader_uart_client_t *client) {
  if (client == NULL || client->deck_controller == NULL) {
    return false;
  }

  DEBUG_PRINT("ISP: Bootloader entry starting\n");

  bccam_bootloader_uart_client_reset_flash_session(client);

  uint8_t handshake[ISP_HANDSHAKE_COUNT];
  memset(handshake, ISP_HANDSHAKE_BYTE, sizeof(handshake));

  isp_drain_rx(50);

  if (!bootloader_deck_begin_boot(client->deck_controller,
                                  BCCAM_DECK_BOOT_BOOTLOADER,
                                  M2T(50),
                                  M2T(10))) {
    return false;
  }

  bootloader_uart_init(BCCAM_ISP_BAUDRATE);

  if (!bootloader_deck_release_boot(client->deck_controller, M2T(50))) {
    return false;
  }

  bool got_ok = false;
  const TickType_t deadline = bootloader_now_ticks() + M2T(5000);

  while (!tick_has_reached(bootloader_now_ticks(), deadline)) {
    bootloader_uart_send(sizeof(handshake), handshake);

    uint8_t c;
    if (bootloader_uart_recv(&c, M2T(1)) && c == ISP_ACK_OK_L) {
      uint8_t c2;
      if (bootloader_uart_recv(&c2, M2T(100)) && c2 == ISP_ACK_OK_H) {
        got_ok = true;
        break;
      }
    }
  }

  if (!got_ok) {
    DEBUG_PRINT("ISP: Handshake timeout (no OK from ROM)\n");
    return false;
  }

  isp_drain_rx(50);
  DEBUG_PRINT("ISP: ROM handshake OK\n");

  isp_send_cmd(ISP_CMD_GET_BOOTINFO, NULL, 0);
  if (!isp_wait_ack()) {
    DEBUG_PRINT("ISP: GET_BOOTINFO failed\n");
    return false;
  }
  isp_drain_rx(100);
  DEBUG_PRINT("ISP: Got boot info\n");

  static const uint8_t flash_pin_cfg[4] = { 0x24, 0x41, 0x01, 0x01 };
  isp_send_cmd(ISP_CMD_FLASH_SET_PARA, flash_pin_cfg, sizeof(flash_pin_cfg));
  if (!isp_wait_ack()) {
    DEBUG_PRINT("ISP: FLASH_SET_PARA NACK at 500k\n");
    return false;
  }

  DEBUG_PRINT("ISP: ROM bootloader ready\n");
  return true;
}

void bccam_bootloader_uart_client_reset_flash_session(
  bccam_bootloader_uart_client_t *client) {
  if (client == NULL) {
    return;
  }

  memset(client->write_buffer, 0, sizeof(client->write_buffer));
  client->write_buffer_used = 0;
  client->write_buffer_base = 0;
  client->bytes_written = 0;
  client->image_base = 0;
  client->image_size = 0;
  client->flash_erased = false;
  client->image_complete = false;
}

bool bccam_bootloader_uart_client_write_flash(
  bccam_bootloader_uart_client_t *client,
  uint32_t mem_addr,
  uint8_t write_len,
  const uint8_t *buffer,
  uint32_t new_fw_size) {
  if (client == NULL || buffer == NULL) {
    return false;
  }

  client->image_complete = false;

  if (!client->flash_erased && new_fw_size > 0) {
    if (mem_addr > UINT32_MAX - (new_fw_size - 1u)) {
      return false;
    }

    const uint32_t erase_end = mem_addr + new_fw_size - 1u;
    if (!isp_flash_erase(mem_addr, erase_end)) {
      return false;
    }
    client->flash_erased = true;
    client->write_buffer_base = mem_addr;
    client->write_buffer_used = 0;
    client->bytes_written = 0;
    client->image_base = mem_addr;
    client->image_size = new_fw_size;
  }

  if (client->flash_erased) {
    if (mem_addr < client->image_base) {
      return false;
    }

    const uint32_t image_offset = mem_addr - client->image_base;
    if (image_offset > client->image_size ||
        write_len > client->image_size - image_offset) {
      return false;
    }
  }

  if (client->write_buffer_used > 0 &&
      mem_addr != client->write_buffer_base + client->write_buffer_used) {
    if (!isp_flash_write(client->write_buffer_base,
                         client->write_buffer,
                         client->write_buffer_used)) {
      return false;
    }
    client->write_buffer_base = mem_addr;
    client->write_buffer_used = 0;
  }

  if (client->write_buffer_used == 0) {
    client->write_buffer_base = mem_addr;
  }

  const uint16_t previous_write_buffer_used = client->write_buffer_used;
  const uint32_t previous_bytes_written = client->bytes_written;

  if (client->write_buffer_used > sizeof(client->write_buffer) ||
      write_len > sizeof(client->write_buffer) - client->write_buffer_used) {
    return false;
  }

  memcpy(&client->write_buffer[client->write_buffer_used], buffer, write_len);
  client->write_buffer_used += write_len;
  client->bytes_written += write_len;

  const bool is_last =
    (new_fw_size > 0 && client->bytes_written >= new_fw_size);
  if (client->write_buffer_used >= BCCAM_WRITE_BUF_SIZE - 256 || is_last) {
    if (!isp_flash_write(client->write_buffer_base,
                         client->write_buffer,
                         client->write_buffer_used)) {
      client->write_buffer_used = previous_write_buffer_used;
      client->bytes_written = previous_bytes_written;
      return false;
    }

    client->write_buffer_base += client->write_buffer_used;
    client->write_buffer_used = 0;

    if (is_last) {
      client->flash_erased = false;
      client->bytes_written = 0;
      client->image_base = 0;
      client->image_size = 0;
      client->image_complete = true;
    }
  }

  return true;
}

bool bccam_bootloader_uart_client_flash_completed(
  const bccam_bootloader_uart_client_t *client) {
  return client != NULL && client->image_complete;
}

bool bccam_bootloader_uart_client_read_flash(
  bccam_bootloader_uart_client_t *client,
  uint32_t mem_addr,
  uint8_t read_len,
  uint8_t *buffer) {
  (void)client;
  if (buffer == NULL) {
    return false;
  }

  return isp_flash_read(mem_addr, buffer, read_len);
}
