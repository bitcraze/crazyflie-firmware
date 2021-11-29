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
 * @file esp_deck_flasher.h
 * Handles flashing of binaries on the ESP32
 *  
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

/**
* @brief Dummy function that can be adapted to verify firmware checksum before booting ESP32. An MD5 checksum can be calculated by the ESP32 ROM bootloader and verified with a hard-coded checksum here.
*
**/
bool espDeckFlasherCheckVersionAndBoot();

/**
* @brief Repeatedly called upon arrival of a data packet from the radio when flashing the ESP from the cfclient with zip.
*
* @param memAddr The address in memory where the data should be written.
* @param writeLen The length of the data to write.
* @param *buffer Pointer to the data to write.
*
* @return true if the data was written successfully, false otherwise.
**/
bool espDeckFlasherWrite(const uint32_t memAddr, const uint8_t writeLen, const uint8_t *buffer);

uint8_t espDeckFlasherPropertiesQuery();
