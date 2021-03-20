/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2020 Bitcraze AB
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
 *
 * lighthouse_deck_flasher.h - handles flashing of FPGA binaries on the
 * Lighthouse deck
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

bool lighthouseDeckFlasherCheckVersionAndBoot();

bool lighthouseDeckFlasherRead(const uint32_t memAddr, const uint8_t readLen, uint8_t* buffer);

bool lighthouseDeckFlasherWrite(const uint32_t memAddr, const uint8_t writeLen, const uint8_t* buffer);

uint8_t lighthouseDeckFlasherPropertiesQuery();
