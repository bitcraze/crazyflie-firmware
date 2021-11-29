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
* @brief Called to reboot the ESP into bootloader mode.
**/
void espRomBootloaderInit();