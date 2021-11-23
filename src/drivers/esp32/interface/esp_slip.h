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

#define ESP_MTU 4000

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


typedef struct
{
  uint8_t direction;
  uint8_t command;
  uint16_t dataSize;
  uint32_t checksum;
} __attribute__((packed)) esp_slip_send_packet;

typedef struct
{
  uint8_t direction;
  uint8_t command;
  uint16_t dataSize;
  uint32_t value; // only for READ_REG command
  uint8_t data[256];
  uint8_t status;
  uint8_t error;
} __attribute__((packed)) esp_slip_receive_packet;

