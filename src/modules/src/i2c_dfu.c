/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C)2025 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * Use DFU via I2C to read and write the flash of an MCU on the deck I2C bus. To be
 * used with the STM32 DFU bootloader. The protocol documentation can be found here:
 * https://www.st.com/resource/en/application_note/an4221-i2c-protocol-used-in-the-stm32-bootloader-stmicroelectronics.pdf
 *
 */

#include <stdint.h>
#include <stdbool.h>

#include "i2cdev.h"

#define DEBUG_MODULE "I2C DFU"
#include "debug.h"

#include "i2c_dfu.h"

#define I2C_DFU_TIMEOUT_MS  1000
#define DFU_ACK             0x79
#define DFU_NACK            0x1F
#define DFU_BUSY            0x76
#define MAX_DFU_DATA_SIZE   256
#define FLASH_PAGE_SIZE     2048

static bool dfu_i2c_erase_pages(const uint8_t i2cAddr, const uint32_t start_page, const uint32_t num_pages) {
  if (num_pages > 255) {
    DEBUG_PRINT("Too many pages to erase: %lu\n", num_pages);
    return false;
  }

  // Send page erase command
  uint8_t cmd[2] = {0x44, 0xBB};
  if (!i2cdevWrite(I2C1_DEV, i2cAddr, 2, cmd)) {
    DEBUG_PRINT("Error sending erase command\n");
    return false;
  }

  // Wait for ACK
  uint8_t status;
  if (!i2cdevRead(I2C1_DEV, i2cAddr, 1, &status)) {
    DEBUG_PRINT("Error reading ACK after erase command\n");
    return false;
  }
  if (status != DFU_ACK) {
    DEBUG_PRINT("NACK after erase command: 0x%02X\n", status);
    return false;
  }

  // Send number of pages - 1 and checksum
  uint16_t num_pages_minus_1 = (num_pages - 1) & 0xFFFF;
  uint8_t erase_info[3];
  erase_info[0] = (num_pages_minus_1 >> 8) & 0xFF;  // MSB
  erase_info[1] = num_pages_minus_1 & 0xFF;         // LSB
  erase_info[2] = erase_info[0] ^ erase_info[1];    // Checksum

  if (!i2cdevWrite(I2C1_DEV, i2cAddr, 3, erase_info)) {
    DEBUG_PRINT("Error sending erase info\n");
    return false;
  }

  // Wait for ACK
  if (!i2cdevRead(I2C1_DEV, i2cAddr, 1, &status)) {
    DEBUG_PRINT("Error reading ACK after erase info\n");
    return false;
  }
  if (status != DFU_ACK) {
    DEBUG_PRINT("NACK after erase info: 0x%02X\n", status);
    return false;
  }

  // Send (N+1) page numbers (2 bytes each, MSB first) and checksum
  // For num_pages, we send: num_pages * 2 bytes + 1 checksum byte
  uint8_t page_buf[num_pages * 2 + 1];
  uint8_t checksum = 0;

  for (uint32_t i = 0; i < num_pages; i++) {
    uint16_t page_num = start_page + i;

    page_buf[i * 2] = (page_num >> 8) & 0xFF;      // MSB
    page_buf[i * 2 + 1] = page_num & 0xFF;         // LSB
    checksum ^= page_buf[i * 2];
    checksum ^= page_buf[i * 2 + 1];
  }
  page_buf[num_pages * 2] = checksum;

  if (!i2cdevWrite(I2C1_DEV, i2cAddr, num_pages * 2 + 1, page_buf)) {
    DEBUG_PRINT("Error sending page numbers\n");
    return false;
  }

  // Wait for ACK. May take a while so handle busy responses
  do {
    vTaskDelay(M2T(10));

    if (!i2cdevRead(I2C1_DEV, i2cAddr, 1, &status)) {
      DEBUG_PRINT("Error reading ACK after page erase\n");
      return false;
    }

  } while (status == DFU_BUSY);

  if (status != DFU_ACK) {
    DEBUG_PRINT("NACK after page erase: 0x%02X\n", status);
    return false;
  }

  return true;
}

bool dfu_i2c_write(const uint8_t i2cAddr, const uint32_t memAddr, const uint8_t* buffer, const uint32_t length) {
  if (length == 0 || length > MAX_DFU_DATA_SIZE) {
    DEBUG_PRINT("Invalid length: %lu (must be 1-256)\n", length);
    return false;
  }

  // Check if we need to erase pages (if write crosses a page boundary)
  uint32_t start_page = memAddr / FLASH_PAGE_SIZE;
  uint32_t end_page = (memAddr + length - 1) / FLASH_PAGE_SIZE;

  if (start_page != end_page || (memAddr % FLASH_PAGE_SIZE) == 0) {
    if (!dfu_i2c_erase_pages(i2cAddr, end_page, 1)) {
      DEBUG_PRINT("Failed to erase pages\n");
      return false;
    }
  }

  // Send write command
  uint8_t cmd[2] = {0x31, 0xCE};
  if (!i2cdevWrite(I2C1_DEV, i2cAddr, 2, cmd)) {
    DEBUG_PRINT("Error sending write command\n");
    return false;
  }

  // Wait for ACK
  uint8_t status;
  if (!i2cdevRead(I2C1_DEV, i2cAddr, 1, &status)) {
    DEBUG_PRINT("Error reading ACK after write command\n");
    return false;
  }
  if (status != DFU_ACK) {
    DEBUG_PRINT("NACK after write command: 0x%02X\n", status);
    return false;
  }

  uint32_t dfu_address = 0x08000000 + memAddr;

  // Send address (MSB first) + checksum
  uint8_t addr_checksum = ((dfu_address >> 24) & 0xFF) ^
                          ((dfu_address >> 16) & 0xFF) ^
                          ((dfu_address >> 8) & 0xFF) ^
                          (dfu_address & 0xFF);
  uint8_t addr_buf[5];
  addr_buf[0] = (dfu_address >> 24) & 0xFF;  // MSB (byte 3)
  addr_buf[1] = (dfu_address >> 16) & 0xFF;  // byte 4
  addr_buf[2] = (dfu_address >> 8) & 0xFF;   // byte 5
  addr_buf[3] = dfu_address & 0xFF;          // LSB (byte 6)
  addr_buf[4] = addr_checksum;               // byte 7

  if (!i2cdevWrite(I2C1_DEV, i2cAddr, 5, addr_buf)) {
    DEBUG_PRINT("Error sending address\n");
    return false;
  }

  // Wait for ACK
  if (!i2cdevRead(I2C1_DEV, i2cAddr, 1, &status)) {
    DEBUG_PRINT("Error reading ACK after address\n");
    return false;
  }
  if (status != DFU_ACK) {
    DEBUG_PRINT("NACK after address: 0x%02X\n", status);
    return false;
  }

  // Send N (number of bytes - 1), data bytes, and checksum
  uint8_t num_bytes = (length - 1) & 0xFF;

  // Calculate checksum: XOR of N and all data bytes
  uint8_t data_checksum = num_bytes;
  for (uint32_t i = 0; i < length; i++) {
    data_checksum ^= buffer[i];
  }

  // Prepare buffer: N + data + checksum
  uint8_t write_buf[MAX_DFU_DATA_SIZE + 2];
  write_buf[0] = num_bytes;
  for (uint32_t i = 0; i < length; i++) {
    write_buf[i + 1] = buffer[i];
  }
  write_buf[length + 1] = data_checksum;

  if (!i2cdevWrite(I2C1_DEV, i2cAddr, length + 2, write_buf)) {
    DEBUG_PRINT("Error sending data\n");
    return false;
  }

  // Wait for ACK, may take a while so handle busy responses
  do {
    vTaskDelay(M2T(10));

    if (!i2cdevRead(I2C1_DEV, i2cAddr, 1, &status)) {
      DEBUG_PRINT("Error reading ACK after data\n");
      return false;
    }

  } while (status == DFU_BUSY);

  if (status != DFU_ACK) {
    DEBUG_PRINT("NACK after data: 0x%02X\n", status);
    return false;
  }

  return true;
}

bool dfu_i2c_read(const uint8_t i2cAddr, const uint32_t memAddr, uint8_t* buffer, const uint32_t length) {
  if (length == 0 || length > MAX_DFU_DATA_SIZE) {
    DEBUG_PRINT("Invalid length: %lu (must be 1-256)\n", length);
    return false;
  }

  // Send read command
  uint8_t cmd[2] = {0x11, 0xEE};
  if (!i2cdevWrite(I2C1_DEV, i2cAddr, 2, cmd)) {
    DEBUG_PRINT("Error sending read command\n");
    return false;
  }

  // Wait for ACK
  uint8_t status;
  if (!i2cdevRead(I2C1_DEV, i2cAddr, 1, &status)) {
    DEBUG_PRINT("Error reading ACK after read command\n");
    return false;
  }
  if (status != DFU_ACK) {
    DEBUG_PRINT("NACK after read command: 0x%02X\n", status);
    return false;
  }

  uint32_t dfu_address = 0x08000000 + memAddr;

  // Send address (MSB first) + checksum
  uint8_t addr_checksum = ((dfu_address >> 24) & 0xFF) ^
                          ((dfu_address >> 16) & 0xFF) ^
                          ((dfu_address >> 8) & 0xFF) ^
                          (dfu_address & 0xFF);
  uint8_t addr_buf[5];
  addr_buf[0] = (dfu_address >> 24) & 0xFF;  // MSB
  addr_buf[1] = (dfu_address >> 16) & 0xFF;
  addr_buf[2] = (dfu_address >> 8) & 0xFF;
  addr_buf[3] = (dfu_address & 0xFF);        // LSB
  addr_buf[4] = addr_checksum;

  if (!i2cdevWrite(I2C1_DEV, i2cAddr, 5, addr_buf)) {
    DEBUG_PRINT("Error sending address\n");
    return false;
  }

  // Wait for ACK
  if (!i2cdevRead(I2C1_DEV, i2cAddr, 1, &status)) {
    DEBUG_PRINT("Error reading ACK after address\n");
    return false;
  }
  if (status != DFU_ACK) {
    DEBUG_PRINT("NACK after address: 0x%02X\n", status);
    return false;
  }

  // Send number of bytes to read - 1 and its complement
  uint8_t num_bytes = (length - 1) & 0xFF;
  uint8_t length_buf[2];
  length_buf[0] = num_bytes;
  length_buf[1] = ~num_bytes;  // Complement

  if (!i2cdevWrite(I2C1_DEV, i2cAddr, 2, length_buf)) {
    DEBUG_PRINT("Error sending length\n");
    return false;
  }

  // Wait for ACK
  if (!i2cdevRead(I2C1_DEV, i2cAddr, 1, &status)) {
    DEBUG_PRINT("Error reading ACK after length\n");
    return false;
  }
  if (status != DFU_ACK) {
    DEBUG_PRINT("NACK after length: 0x%02X\n", status);
    return false;
  }

  // Read the data
  if (!i2cdevRead(I2C1_DEV, i2cAddr, length, buffer)) {
    DEBUG_PRINT("Error reading data\n");
    return false;
  }

  return true;
}