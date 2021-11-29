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
 * @file esp_rom_bootloader.c
 * Driver for communicating with the ESP32 ROM bootloader
 *  
 */

#define DEBUG_MODULE "ESPROMBL"

#include <string.h>

#include "FreeRTOS.h"
#include "debug.h"
#include "deck.h"
#include "esp_rom_bootloader.h"
#include "task.h"
#include "uart2.h"

void espRomBootloaderInit()
{
  pinMode(DECK_GPIO_IO1, OUTPUT);
  digitalWrite(DECK_GPIO_IO1, LOW);
  pinMode(DECK_GPIO_IO4, OUTPUT);
  digitalWrite(DECK_GPIO_IO4, LOW);
  vTaskDelay(10);
  digitalWrite(DECK_GPIO_IO4, HIGH);
  pinMode(DECK_GPIO_IO4, INPUT_PULLUP);
  vTaskDelay(100);
  digitalWrite(DECK_GPIO_IO1, HIGH);
  pinMode(DECK_GPIO_IO1, INPUT_PULLUP);
}

