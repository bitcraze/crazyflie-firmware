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
 *  @file esp_rom_bootloader.h
 * Driver for communicating with the ESP32 ROM bootloader
 *
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_slip.h"

#define ESP_BOOTLOADER_ADDRESS 0x1000
#define ESP_PARTITION_ADDRESS 0x8000
#define ESP_FW_ADDRESS 0x10000

/**
* @brief Called to sync with the bootloader. Allows the ESP to automatically configure the baud rate.
*
* @param *sendBuffer Pointer to a buffer used to construct the sync packet. Can be left empty.
*
* @return true if sync was successful, false otherwise.
**/
bool espRomBootloaderSync(uint8_t *sendBuffer);

/**
* @brief Called to initialize the flashing with the ESP.
*
* @param *sendBuffer Pointer to a buffer used to construct the flash begin packet. Can be left empty.
* @param numberOfFlashBuffers The number of data packets that will be sent
* @param firmwareSize The total to be flashed size
* @param flashOffset The offset in flash where flashing will start
*
* @return true if flash begin command was accepted by ESP, false otherwise.
**/
bool espRomBootloaderFlashBegin(uint8_t *sendBuffer, uint32_t numberOfFlashBuffers, uint32_t firmwareSize, uint32_t flashOffset);

/**
* @brief Called to flash a packet onto the ESP.
*
* @param *sendBuffer Pointer to a buffer used to construct the slip packet. Must contain actual to be flashed data.
* @param flashDataSize Size of the current data packet
* @param sequenceNumber The sequence number of the current data packet, indicates this is the nth packet
*
* @return true if data was succesfully flashed to ESP, false otherwise.
**/
bool espRomBootloaderFlashData(uint8_t *sendBuffer, uint32_t flashDataSize, uint32_t sequenceNumber);

/**
* @brief Called to attach the SPI memory to the ESP. Must be called before issuing any flash command.
*
* @param sendBuffer Buffer used to construct the SPI attach packet. Can be left empty.
*
* @return true if SPI was succesfully attached to ESP, false otherwise.
**/
bool espRomBootloaderSpiAttach(uint8_t *sendBuffer);
