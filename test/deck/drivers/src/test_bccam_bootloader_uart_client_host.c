#include <stdint.h>
#include <string.h>

#include "unity.h"
#include "bccam_bootloader_uart_client.h"
// @MODULE "bccam_deck_controller.c"

#define TEST_ISP_BAUDRATE 500000
#define TEST_HANDSHAKE_BYTE 0x55
#define TEST_HANDSHAKE_COUNT 32
#define TEST_FLASH_BASE 0x23000000

static bccam_deck_controller_t deck_controller;
static bccam_bootloader_uart_client_t client;

void setUp(void) {
  bccam_deck_controller_init(&deck_controller, NULL);
  bccam_bootloader_uart_client_init(&client, &deck_controller);
  bccam_bootloader_uart_client_test_trace_reset();
}

void tearDown(void) {
}

static void queue_ok(void) {
  const uint8_t ok[] = { 'O', 'K' };
  bccam_bootloader_uart_client_test_queue_rx(ok, sizeof(ok));
}

static const bccam_bootloader_uart_client_test_trace_entry_t *
trace_entry(uint8_t index) {
  const bccam_bootloader_uart_client_test_trace_entry_t *entry =
    bccam_bootloader_uart_client_test_trace_entry(index);

  TEST_ASSERT_NOT_NULL(entry);
  return entry;
}

static uint8_t find_event_after(bccam_bootloader_uart_client_test_event_t event,
                                uint8_t after_index) {
  const uint8_t count = bccam_bootloader_uart_client_test_trace_count();

  for (uint8_t i = after_index + 1; i < count; i++) {
    const bccam_bootloader_uart_client_test_trace_entry_t *entry =
      bccam_bootloader_uart_client_test_trace_entry(i);
    if (entry != NULL && entry->event == event) {
      return i;
    }
  }

  TEST_FAIL_MESSAGE("event not found");
  return 0;
}

static uint32_t count_isp_command(uint8_t command) {
  uint32_t count = 0;
  const uint8_t trace_count = bccam_bootloader_uart_client_test_trace_count();

  for (uint8_t i = 0; i < trace_count; i++) {
    const bccam_bootloader_uart_client_test_trace_entry_t *entry =
      trace_entry(i);
    if (entry->event == BCCAM_BOOTLOADER_UART_CLIENT_TEST_UART_SEND &&
        entry->length > 0 &&
        entry->bytes[0] == command) {
      count++;
    }
  }

  return count;
}

static const bccam_bootloader_uart_client_test_trace_entry_t *
nth_isp_command(uint8_t command, uint32_t command_index) {
  uint32_t count = 0;
  const uint8_t trace_count = bccam_bootloader_uart_client_test_trace_count();

  for (uint8_t i = 0; i < trace_count; i++) {
    const bccam_bootloader_uart_client_test_trace_entry_t *entry =
      trace_entry(i);
    if (entry->event == BCCAM_BOOTLOADER_UART_CLIENT_TEST_UART_SEND &&
        entry->length > 0 &&
        entry->bytes[0] == command) {
      if (count == command_index) {
        return entry;
      }
      count++;
    }
  }

  TEST_FAIL_MESSAGE("ISP command not found");
  return NULL;
}

static uint32_t isp_write_address(
  const bccam_bootloader_uart_client_test_trace_entry_t *entry) {
  return (uint32_t)entry->bytes[4] |
         ((uint32_t)entry->bytes[5] << 8) |
         ((uint32_t)entry->bytes[6] << 16) |
         ((uint32_t)entry->bytes[7] << 24);
}

static uint16_t isp_write_length(
  const bccam_bootloader_uart_client_test_trace_entry_t *entry) {
  const uint16_t total_len =
    (uint16_t)entry->bytes[2] | ((uint16_t)entry->bytes[3] << 8);
  return total_len - 4;
}

static void assert_sent_handshake(const bccam_bootloader_uart_client_test_trace_entry_t *entry) {
  TEST_ASSERT_EQUAL_INT(BCCAM_BOOTLOADER_UART_CLIENT_TEST_UART_SEND,
                        entry->event);
  TEST_ASSERT_EQUAL_UINT32(TEST_HANDSHAKE_COUNT, entry->length);
  for (uint8_t i = 0; i < TEST_HANDSHAKE_COUNT; i++) {
    TEST_ASSERT_EQUAL_UINT8(TEST_HANDSHAKE_BYTE, entry->bytes[i]);
  }
}

void testFlashWriteRejectsEraseRangeOverflowBeforeSendingErase(void) {
  const uint8_t data[1] = { 0xA5 };

  TEST_ASSERT_FALSE(bccam_bootloader_uart_client_write_flash(&client,
                                                             0xFFFFFFF0u,
                                                             sizeof(data),
                                                             data,
                                                             0x20u));

  TEST_ASSERT_EQUAL_UINT32(0, count_isp_command(0x30));
}

void testFirstFlashWriteErasesBeforeBufferedWrite(void) {
  const uint8_t data[4] = { 1, 2, 3, 4 };

  queue_ok();
  queue_ok();

  TEST_ASSERT_TRUE(bccam_bootloader_uart_client_write_flash(&client,
                                                            TEST_FLASH_BASE,
                                                            sizeof(data),
                                                            data,
                                                            sizeof(data)));

  TEST_ASSERT_EQUAL_UINT32(1, count_isp_command(0x30));
  TEST_ASSERT_EQUAL_UINT32(1, count_isp_command(0x31));

  const bccam_bootloader_uart_client_test_trace_entry_t *write =
    nth_isp_command(0x31, 0);
  TEST_ASSERT_EQUAL_UINT32(TEST_FLASH_BASE, isp_write_address(write));
  TEST_ASSERT_EQUAL_UINT16(sizeof(data), isp_write_length(write));
}

void testFlashCompletedIsLatchedOnlyAfterSuccessfulFinalWrite(void) {
  const uint8_t data[4] = { 1, 2, 3, 4 };

  TEST_ASSERT_FALSE(bccam_bootloader_uart_client_flash_completed(&client));

  queue_ok();
  queue_ok();
  TEST_ASSERT_TRUE(bccam_bootloader_uart_client_write_flash(&client,
                                                            TEST_FLASH_BASE,
                                                            sizeof(data),
                                                            data,
                                                            sizeof(data)));

  TEST_ASSERT_TRUE(bccam_bootloader_uart_client_flash_completed(&client));

  bccam_bootloader_uart_client_reset_flash_session(&client);

  TEST_ASSERT_FALSE(bccam_bootloader_uart_client_flash_completed(&client));
}

void testFlashWriteRejectsChunkPastDeclaredImageWithoutSendingWrite(void) {
  const uint8_t first_chunk[2] = { 1, 2 };
  const uint8_t overflowing_chunk[3] = { 3, 4, 5 };

  queue_ok();

  TEST_ASSERT_TRUE(bccam_bootloader_uart_client_write_flash(&client,
                                                            TEST_FLASH_BASE,
                                                            sizeof(first_chunk),
                                                            first_chunk,
                                                            4));
  TEST_ASSERT_EQUAL_UINT32(1, count_isp_command(0x30));
  TEST_ASSERT_EQUAL_UINT32(0, count_isp_command(0x31));

  TEST_ASSERT_FALSE(bccam_bootloader_uart_client_write_flash(&client,
                                                             TEST_FLASH_BASE + 2,
                                                             sizeof(overflowing_chunk),
                                                             overflowing_chunk,
                                                             4));
  TEST_ASSERT_EQUAL_UINT32(0, count_isp_command(0x31));
}

void testResetFlashSessionClearsPartialBufferedWrite(void) {
  const uint8_t first_chunk[4] = { 1, 2, 3, 4 };
  const uint8_t second_image[4] = { 5, 6, 7, 8 };

  queue_ok();

  TEST_ASSERT_TRUE(bccam_bootloader_uart_client_write_flash(&client,
                                                            TEST_FLASH_BASE,
                                                            sizeof(first_chunk),
                                                            first_chunk,
                                                            8));
  TEST_ASSERT_EQUAL_UINT32(1, count_isp_command(0x30));
  TEST_ASSERT_EQUAL_UINT32(0, count_isp_command(0x31));

  bccam_bootloader_uart_client_reset_flash_session(&client);

  queue_ok();
  queue_ok();
  TEST_ASSERT_TRUE(bccam_bootloader_uart_client_write_flash(&client,
                                                            TEST_FLASH_BASE + 0x1000,
                                                            sizeof(second_image),
                                                            second_image,
                                                            sizeof(second_image)));

  TEST_ASSERT_EQUAL_UINT32(2, count_isp_command(0x30));
  TEST_ASSERT_EQUAL_UINT32(1, count_isp_command(0x31));

  const bccam_bootloader_uart_client_test_trace_entry_t *write =
    nth_isp_command(0x31, 0);
  TEST_ASSERT_EQUAL_UINT32(TEST_FLASH_BASE + 0x1000, isp_write_address(write));
  TEST_ASSERT_EQUAL_UINT16(sizeof(second_image), isp_write_length(write));
}

void testFailedFinalFlashWriteDoesNotDuplicateChunkOnRetry(void) {
  const uint8_t image[4] = { 1, 2, 3, 4 };

  queue_ok();

  TEST_ASSERT_FALSE(bccam_bootloader_uart_client_write_flash(&client,
                                                             TEST_FLASH_BASE,
                                                             sizeof(image),
                                                             image,
                                                             sizeof(image)));
  TEST_ASSERT_EQUAL_UINT32(1, count_isp_command(0x31));
  TEST_ASSERT_EQUAL_UINT16(sizeof(image),
                           isp_write_length(nth_isp_command(0x31, 0)));

  queue_ok();
  TEST_ASSERT_TRUE(bccam_bootloader_uart_client_write_flash(&client,
                                                            TEST_FLASH_BASE,
                                                            sizeof(image),
                                                            image,
                                                            sizeof(image)));

  TEST_ASSERT_EQUAL_UINT32(2, count_isp_command(0x31));
  const bccam_bootloader_uart_client_test_trace_entry_t *retry =
    nth_isp_command(0x31, 1);
  TEST_ASSERT_EQUAL_UINT32(TEST_FLASH_BASE, isp_write_address(retry));
  TEST_ASSERT_EQUAL_UINT16(sizeof(image), isp_write_length(retry));
}

void testFailedThresholdFlashWriteDoesNotDuplicateChunkOnRetry(void) {
  uint8_t chunk[255];
  const uint8_t retry_chunk[4] = { 9, 10, 11, 12 };
  uint8_t next_chunk[255];

  memset(chunk, 0x5A, sizeof(chunk));
  memset(next_chunk, 0xA5, sizeof(next_chunk));
  queue_ok();

  TEST_ASSERT_TRUE(bccam_bootloader_uart_client_write_flash(&client,
                                                            TEST_FLASH_BASE,
                                                            sizeof(chunk),
                                                            chunk,
                                                            1024));
  TEST_ASSERT_TRUE(bccam_bootloader_uart_client_write_flash(&client,
                                                            TEST_FLASH_BASE + 0xFF,
                                                            sizeof(chunk),
                                                            chunk,
                                                            1024));
  TEST_ASSERT_TRUE(bccam_bootloader_uart_client_write_flash(&client,
                                                            TEST_FLASH_BASE + 0x1FE,
                                                            sizeof(chunk),
                                                            chunk,
                                                            1024));
  TEST_ASSERT_EQUAL_UINT32(0, count_isp_command(0x31));

  TEST_ASSERT_FALSE(bccam_bootloader_uart_client_write_flash(&client,
                                                             TEST_FLASH_BASE + 0x2FD,
                                                             sizeof(retry_chunk),
                                                             retry_chunk,
                                                             1024));
  TEST_ASSERT_EQUAL_UINT32(1, count_isp_command(0x31));
  TEST_ASSERT_EQUAL_UINT16(769,
                           isp_write_length(nth_isp_command(0x31, 0)));

  queue_ok();
  TEST_ASSERT_TRUE(bccam_bootloader_uart_client_write_flash(&client,
                                                            TEST_FLASH_BASE + 0x2FD,
                                                            sizeof(retry_chunk),
                                                            retry_chunk,
                                                            1024));

  TEST_ASSERT_EQUAL_UINT32(2, count_isp_command(0x31));
  const bccam_bootloader_uart_client_test_trace_entry_t *retry =
    nth_isp_command(0x31, 1);
  TEST_ASSERT_EQUAL_UINT32(TEST_FLASH_BASE, isp_write_address(retry));
  TEST_ASSERT_EQUAL_UINT16(769, isp_write_length(retry));

  queue_ok();
  TEST_ASSERT_TRUE(bccam_bootloader_uart_client_write_flash(&client,
                                                            TEST_FLASH_BASE + 0x301,
                                                            sizeof(next_chunk),
                                                            next_chunk,
                                                            1024));

  TEST_ASSERT_EQUAL_UINT32(3, count_isp_command(0x31));
  const bccam_bootloader_uart_client_test_trace_entry_t *next =
    nth_isp_command(0x31, 2);
  TEST_ASSERT_EQUAL_UINT32(TEST_FLASH_BASE + 0x301, isp_write_address(next));
  TEST_ASSERT_EQUAL_UINT16(sizeof(next_chunk), isp_write_length(next));
}

void testEnterBootloaderSetsIspBaudrateBeforeReleasingReset(void) {
  queue_ok();
  queue_ok();
  queue_ok();

  TEST_ASSERT_TRUE(bccam_bootloader_uart_client_enter(&client));

  uint8_t index = find_event_after(BCCAM_BOOTLOADER_UART_CLIENT_TEST_DECK_BEGIN_BOOT,
                                   (uint8_t)-1);
  const bccam_bootloader_uart_client_test_trace_entry_t *entry =
    trace_entry(index);
  TEST_ASSERT_EQUAL_INT(BCCAM_DECK_BOOT_BOOTLOADER, entry->boot_mode);

  index = find_event_after(BCCAM_BOOTLOADER_UART_CLIENT_TEST_UART_SET_BAUDRATE,
                           index);
  entry = trace_entry(index);
  TEST_ASSERT_EQUAL_UINT32(TEST_ISP_BAUDRATE, entry->value);

  index = find_event_after(BCCAM_BOOTLOADER_UART_CLIENT_TEST_DECK_RELEASE_BOOT,
                           index);

  index = find_event_after(BCCAM_BOOTLOADER_UART_CLIENT_TEST_UART_SEND, index);
  assert_sent_handshake(trace_entry(index));

  index = find_event_after(BCCAM_BOOTLOADER_UART_CLIENT_TEST_UART_RECEIVE,
                           index);
  TEST_ASSERT_EQUAL_UINT8('O', trace_entry(index)->byte);

  index = find_event_after(BCCAM_BOOTLOADER_UART_CLIENT_TEST_UART_RECEIVE,
                           index);
  TEST_ASSERT_EQUAL_UINT8('K', trace_entry(index)->byte);
}

void testEnterBootloaderInitializesUartBeforeSettingIspBaudrate(void) {
  queue_ok();
  queue_ok();
  queue_ok();

  TEST_ASSERT_TRUE(bccam_bootloader_uart_client_enter(&client));

  uint8_t index = find_event_after(BCCAM_BOOTLOADER_UART_CLIENT_TEST_DECK_BEGIN_BOOT,
                                   (uint8_t)-1);
  index = find_event_after(BCCAM_BOOTLOADER_UART_CLIENT_TEST_UART_INIT,
                           index);
  index = find_event_after(BCCAM_BOOTLOADER_UART_CLIENT_TEST_UART_SET_BAUDRATE,
                           index);
  const bccam_bootloader_uart_client_test_trace_entry_t *entry =
    trace_entry(index);
  TEST_ASSERT_EQUAL_UINT32(TEST_ISP_BAUDRATE, entry->value);
}

void testEnterBootloaderSendsHandshakeUntilOk(void) {
  const uint8_t delayed_ok[] = { 0x00, 'O', 'K' };
  bccam_bootloader_uart_client_test_queue_rx(delayed_ok, sizeof(delayed_ok));
  queue_ok();
  queue_ok();

  TEST_ASSERT_TRUE(bccam_bootloader_uart_client_enter(&client));

  uint32_t handshake_sends = 0;
  const uint8_t count = bccam_bootloader_uart_client_test_trace_count();
  for (uint8_t i = 0; i < count; i++) {
    const bccam_bootloader_uart_client_test_trace_entry_t *entry =
      trace_entry(i);
    if (entry->event == BCCAM_BOOTLOADER_UART_CLIENT_TEST_UART_SEND &&
        entry->length == TEST_HANDSHAKE_COUNT &&
        entry->bytes[0] == TEST_HANDSHAKE_BYTE) {
      handshake_sends++;
    }
  }

  TEST_ASSERT_EQUAL_UINT32(2, handshake_sends);
}

void testFlashWriteEncodesIspWriteCommandAndWaitsForAck(void) {
  const uint8_t data[] = { 1, 2, 3, 4 };

  queue_ok();
  queue_ok();

  TEST_ASSERT_TRUE(bccam_bootloader_uart_client_write_flash(&client,
                                                            TEST_FLASH_BASE,
                                                            sizeof(data),
                                                            data,
                                                            sizeof(data)));

  const uint8_t count = bccam_bootloader_uart_client_test_trace_count();
  TEST_ASSERT_GREATER_THAN_UINT8(0, count);

  uint8_t write_index = find_event_after(BCCAM_BOOTLOADER_UART_CLIENT_TEST_UART_SEND,
                                         (uint8_t)-1);
  while (trace_entry(write_index)->bytes[0] != 0x31) {
    write_index = find_event_after(BCCAM_BOOTLOADER_UART_CLIENT_TEST_UART_SEND,
                                   write_index);
  }

  const bccam_bootloader_uart_client_test_trace_entry_t *write_entry =
    trace_entry(write_index);
  TEST_ASSERT_EQUAL_INT(BCCAM_BOOTLOADER_UART_CLIENT_TEST_UART_SEND,
                        write_entry->event);
  TEST_ASSERT_EQUAL_UINT32(8, write_entry->length);
  TEST_ASSERT_EQUAL_UINT8(0x31, write_entry->bytes[0]);
  TEST_ASSERT_EQUAL_UINT8(0x35, write_entry->bytes[1]);
  TEST_ASSERT_EQUAL_UINT8(8, write_entry->bytes[2]);
  TEST_ASSERT_EQUAL_UINT8(0, write_entry->bytes[3]);
  TEST_ASSERT_EQUAL_UINT8(0x00, write_entry->bytes[4]);
  TEST_ASSERT_EQUAL_UINT8(0x00, write_entry->bytes[5]);
  TEST_ASSERT_EQUAL_UINT8(0x00, write_entry->bytes[6]);
  TEST_ASSERT_EQUAL_UINT8(0x23, write_entry->bytes[7]);

  const bccam_bootloader_uart_client_test_trace_entry_t *data_entry =
    trace_entry(write_index + 1);
  TEST_ASSERT_EQUAL_INT(BCCAM_BOOTLOADER_UART_CLIENT_TEST_UART_SEND_DMA,
                        data_entry->event);
  TEST_ASSERT_EQUAL_UINT32(sizeof(data), data_entry->length);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(data, data_entry->bytes, sizeof(data));

  uint8_t first_ack = find_event_after(BCCAM_BOOTLOADER_UART_CLIENT_TEST_UART_RECEIVE,
                                       write_index + 1);
  TEST_ASSERT_EQUAL_UINT8('O', trace_entry(first_ack)->byte);
  uint8_t second_ack = find_event_after(BCCAM_BOOTLOADER_UART_CLIENT_TEST_UART_RECEIVE,
                                        first_ack);
  TEST_ASSERT_EQUAL_UINT8('K', trace_entry(second_ack)->byte);
}

void testFlashReadRejectsLengthMismatch(void) {
  uint8_t read_buffer[4];
  const uint8_t response[] = { 'O', 'K', 3, 0, 0xAA, 0xBB, 0xCC };

  memset(read_buffer, 0, sizeof(read_buffer));
  bccam_bootloader_uart_client_test_queue_rx(response, sizeof(response));

  TEST_ASSERT_FALSE(bccam_bootloader_uart_client_read_flash(&client,
                                                            TEST_FLASH_BASE,
                                                            sizeof(read_buffer),
                                                            read_buffer));

  const uint8_t count = bccam_bootloader_uart_client_test_trace_count();
  TEST_ASSERT_GREATER_THAN_UINT8(0, count);
  TEST_ASSERT_EQUAL_INT(BCCAM_BOOTLOADER_UART_CLIENT_TEST_UART_DRAIN,
                        trace_entry(count - 1)->event);
}
