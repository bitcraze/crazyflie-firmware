/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2021 Bitcraze AB
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
 * @file esp_slip.h
 * Protocol for assembling, sending, receiving and decoding SLIP packets to/from the ESP32 ROM bootloader
 *
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#define ESP_SLIP_MTU 4000
#define ESP_SLIP_TX_BUFFER_SIZE 128
#define ESP_SLIP_OVERHEAD_LEN 8

// to account for additional overhead in the SLIP packet while flashing data
#define ESP_SLIP_ADDITIONAL_DATA_OVERHEAD_LEN 16
#define ESP_SLIP_START_CODE_LEN 1
#define ESP_SLIP_STOP_CODE_LEN 1
#define ESP_SLIP_DATA_START (ESP_SLIP_START_CODE_LEN + ESP_SLIP_OVERHEAD_LEN + ESP_SLIP_ADDITIONAL_DATA_OVERHEAD_LEN)

/* Commands */
#define DIR_CMD 0x00
#define FLASH_BEGIN 0x02
#define FLASH_DATA 0x03
#define FLASH_END 0x04
#define SYNC 0x08
#define SPI_FLASH_MD5 0x13
#define ERASE_FLASH 0xd0
#define READ_FLASH 0xd2
#define CHANGE_BAUDRATE 0x0f
#define SPI_ATTACH 0x0d

#define SLIP_START_STOP_BYTE 0xc0

typedef void (*espSlipSendBuffer_t)(uint32_t size, uint8_t *data);
typedef bool (*espSlipGetDataWithTimeout_t)(uint8_t *c, const uint32_t timeoutTicks);

typedef enum
{
  SLIP_DECODING,
  SLIP_SUCCESS,
  SLIP_ERROR
} slipDecoderStatus_t;

typedef struct
{
  uint8_t direction;
  uint8_t command;
  uint16_t dataSize;
  uint32_t checksum;
} __attribute__((packed)) espSlipSendPacket_t;

typedef struct
{
  uint8_t direction;
  uint8_t command;
  uint16_t dataSize;
  uint32_t value; // only for READ_REG command
  uint8_t data[256];
  uint8_t status;
  uint8_t error;
} __attribute__((packed)) espSlipReceivePacket_t;

/**
* @brief Called to send a SLIP packet to the ESP, and receive the response.
*
* @param *sendBuffer Pointer to the to be sent buffer
* @param *receiverPacket Pointer to receiver packet struct, which will be filled with the response
* @param *senderPacket Pointer to sender packet struct, which will be used to fill the send buffer header
* @param sendBufferFunction Function pointer to the function that sends the buffer (split into pages) to the ESP
* @param getDataWithTimeoutFunction Function pointer to the function that receives data (byte by byte) from the ESP
* @param timeoutTicks Number of ticks to wait for a response from the ESP
*
* @return true if ESP responds with a status byte indicating success.
**/
bool espSlipExchange(uint8_t *sendBuffer, espSlipReceivePacket_t *receiverPacket, espSlipSendPacket_t *senderPacket, espSlipSendBuffer_t sendBufferFunction,
                     espSlipGetDataWithTimeout_t getDataWithTimeout, uint32_t timeoutTicks);
