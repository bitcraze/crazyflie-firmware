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

static espSlipSendPacket_t senderPacket;
static espSlipReceivePacket_t receiverPacket;

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

bool espRomBootloaderSync(uint8_t *sendBuffer)
{
  senderPacket.command = SYNC;
  senderPacket.dataSize = 0x24;
  sendBuffer[9 + 0] = 0x07;
  sendBuffer[9 + 1] = 0x07;
  sendBuffer[9 + 2] = 0x12;
  sendBuffer[9 + 3] = 0x20;
  for (int i = 0; i < 32; i++)
  {
    sendBuffer[9 + 4 + i] = 0x55;
  }

  bool sync = false;
  for (int i = 0; i < 10 && !sync; i++) // max 10 sync attempts
  {
    sync = espSlipExchange(sendBuffer, &receiverPacket, &senderPacket, uart2SendDataDmaBlocking, uart2GetDataWithTimeout, 100);
  }

  //
  // ESP32 responds multiple times upon succesful SYNC. Wait until all responses are received, so they can be cleared before next transmission.
  //
  vTaskDelay(M2T(100));

  return sync;
}
