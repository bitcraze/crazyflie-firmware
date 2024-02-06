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

#define DEBUG_MODULE "ESP_ROM_BL"

#include <string.h>

#include "FreeRTOS.h"
#include "debug.h"
#include "deck.h"
#include "esp_rom_bootloader.h"
#include "task.h"
#include "uart2.h"

#if UART2_DMA_BUFFER_SIZE < ESP_SLIP_TX_BUFFER_SIZE
#warning "ESP SLIP transmission buffer size must be smaller than or equal to UART2 DMA buffer size"
#endif

#define SYNC_ATTEMPTS 10

static espSlipSendPacket_t senderPacket;
static espSlipReceivePacket_t receiverPacket;

bool espRomBootloaderSync(uint8_t *sendBuffer)
{
  senderPacket.command = SYNC;
  senderPacket.dataSize = 0x24;
  sendBuffer[1 + ESP_SLIP_OVERHEAD_LEN + 0] = 0x07; // + 1 to account for SLIP start byte 0xC0
  sendBuffer[1 + ESP_SLIP_OVERHEAD_LEN + 1] = 0x07;
  sendBuffer[1 + ESP_SLIP_OVERHEAD_LEN + 2] = 0x12;
  sendBuffer[1 + ESP_SLIP_OVERHEAD_LEN + 3] = 0x20;
  for (int i = 0; i < 32; i++)
  {
    sendBuffer[9 + 4 + i] = 0x55;
  }

  bool sync = false;
  for (int i = 0; i < SYNC_ATTEMPTS && !sync; i++)
  {
    sync = espSlipExchange(sendBuffer, &receiverPacket, &senderPacket, uart2SendDataDmaBlocking, uart2GetCharWithTimeout, 100);
  }

  //
  // ESP32 responds multiple times upon successful SYNC. Wait until all responses are received, so they can be cleared before next transmission.
  //
  vTaskDelay(M2T(100));

  return sync;
}

bool espRomBootloaderSpiAttach(uint8_t *sendBuffer)
{
  senderPacket.command = SPI_ATTACH;
  senderPacket.dataSize = 0x4;
  sendBuffer[1 + ESP_SLIP_OVERHEAD_LEN + 0] = 0x00;
  sendBuffer[1 + ESP_SLIP_OVERHEAD_LEN + 1] = 0x00;
  sendBuffer[1 + ESP_SLIP_OVERHEAD_LEN + 2] = 0x00;
  sendBuffer[1 + ESP_SLIP_OVERHEAD_LEN + 3] = 0x00;
  sendBuffer[1 + ESP_SLIP_OVERHEAD_LEN + 4] = 0x00;
  sendBuffer[1 + ESP_SLIP_OVERHEAD_LEN + 5] = 0x00;
  sendBuffer[1 + ESP_SLIP_OVERHEAD_LEN + 6] = 0x00;
  sendBuffer[1 + ESP_SLIP_OVERHEAD_LEN + 7] = 0x00;

  return espSlipExchange(sendBuffer, &receiverPacket, &senderPacket, uart2SendDataDmaBlocking, uart2GetCharWithTimeout, 100);
}

bool espRomBootloaderFlashBegin(uint8_t *sendBuffer, uint32_t numberOfFlashBuffers, uint32_t firmwareSize, uint32_t flashOffset)
{
  senderPacket.command = FLASH_BEGIN;
  senderPacket.dataSize = 0x10;
  sendBuffer[1 + ESP_SLIP_OVERHEAD_LEN + 0] = (uint8_t)((firmwareSize >> 0) & 0x000000FF);
  sendBuffer[1 + ESP_SLIP_OVERHEAD_LEN + 1] = (uint8_t)((firmwareSize >> 8) & 0x000000FF);
  sendBuffer[1 + ESP_SLIP_OVERHEAD_LEN + 2] = (uint8_t)((firmwareSize >> 16) & 0x000000FF);
  sendBuffer[1 + ESP_SLIP_OVERHEAD_LEN + 3] = (uint8_t)((firmwareSize >> 24) & 0x000000FF);

  sendBuffer[1 + ESP_SLIP_OVERHEAD_LEN + 4] = (uint8_t)((numberOfFlashBuffers >> 0) & 0x000000FF);
  sendBuffer[1 + ESP_SLIP_OVERHEAD_LEN + 5] = (uint8_t)((numberOfFlashBuffers >> 8) & 0x000000FF);
  sendBuffer[1 + ESP_SLIP_OVERHEAD_LEN + 6] = (uint8_t)((numberOfFlashBuffers >> 16) & 0x000000FF);
  sendBuffer[1 + ESP_SLIP_OVERHEAD_LEN + 7] = (uint8_t)((numberOfFlashBuffers >> 24) & 0x000000FF);

  sendBuffer[1 + ESP_SLIP_OVERHEAD_LEN + 8] = (uint8_t)((ESP_SLIP_MTU >> 0) & 0x000000FF);
  sendBuffer[1 + ESP_SLIP_OVERHEAD_LEN + 9] = (uint8_t)((ESP_SLIP_MTU >> 8) & 0x000000FF);
  sendBuffer[1 + ESP_SLIP_OVERHEAD_LEN + 10] = (uint8_t)((ESP_SLIP_MTU >> 16) & 0x000000FF);
  sendBuffer[1 + ESP_SLIP_OVERHEAD_LEN + 11] = (uint8_t)((ESP_SLIP_MTU >> 24) & 0x000000FF);

  sendBuffer[1 + ESP_SLIP_OVERHEAD_LEN + 12] = (uint8_t)((flashOffset >> 0) & 0x000000FF);
  sendBuffer[1 + ESP_SLIP_OVERHEAD_LEN + 13] = (uint8_t)((flashOffset >> 8) & 0x000000FF);
  sendBuffer[1 + ESP_SLIP_OVERHEAD_LEN + 14] = (uint8_t)((flashOffset >> 16) & 0x000000FF);
  sendBuffer[1 + ESP_SLIP_OVERHEAD_LEN + 15] = (uint8_t)((flashOffset >> 24) & 0x000000FF);

  return espSlipExchange(sendBuffer, &receiverPacket, &senderPacket, uart2SendDataDmaBlocking, uart2GetCharWithTimeout, 10000);
}

bool espRomBootloaderFlashData(uint8_t *sendBuffer, uint32_t flashDataSize, uint32_t sequenceNumber)
{
  senderPacket.command = FLASH_DATA;
  senderPacket.dataSize = ESP_SLIP_MTU + ESP_SLIP_ADDITIONAL_DATA_OVERHEAD_LEN;

  sendBuffer[1 + ESP_SLIP_OVERHEAD_LEN + 0] = (uint8_t)((ESP_SLIP_MTU >> 0) & 0x000000FF);
  sendBuffer[1 + ESP_SLIP_OVERHEAD_LEN + 1] = (uint8_t)((ESP_SLIP_MTU >> 8) & 0x000000FF);
  sendBuffer[1 + ESP_SLIP_OVERHEAD_LEN + 2] = (uint8_t)((ESP_SLIP_MTU >> 16) & 0x000000FF);
  sendBuffer[1 + ESP_SLIP_OVERHEAD_LEN + 3] = (uint8_t)((ESP_SLIP_MTU >> 24) & 0x000000FF);
  sendBuffer[1 + ESP_SLIP_OVERHEAD_LEN + 4] = (uint8_t)((sequenceNumber >> 0) & 0x000000FF);
  sendBuffer[1 + ESP_SLIP_OVERHEAD_LEN + 5] = (uint8_t)((sequenceNumber >> 8) & 0x000000FF);
  sendBuffer[1 + ESP_SLIP_OVERHEAD_LEN + 6] = (uint8_t)((sequenceNumber >> 16) & 0x000000FF);
  sendBuffer[1 + ESP_SLIP_OVERHEAD_LEN + 7] = (uint8_t)((sequenceNumber >> 24) & 0x000000FF);
  sendBuffer[1 + ESP_SLIP_OVERHEAD_LEN + 8] = 0x00;
  sendBuffer[1 + ESP_SLIP_OVERHEAD_LEN + 9] = 0x00;
  sendBuffer[1 + ESP_SLIP_OVERHEAD_LEN + 10] = 0x00;
  sendBuffer[1 + ESP_SLIP_OVERHEAD_LEN + 11] = 0x00;
  sendBuffer[1 + ESP_SLIP_OVERHEAD_LEN + 12] = 0x00;
  sendBuffer[1 + ESP_SLIP_OVERHEAD_LEN + 13] = 0x00;
  sendBuffer[1 + ESP_SLIP_OVERHEAD_LEN + 14] = 0x00;
  sendBuffer[1 + ESP_SLIP_OVERHEAD_LEN + 15] = 0x00;

  if (flashDataSize < ESP_SLIP_MTU)
  {
    // pad the data with 0xFF
    memset(&sendBuffer[ESP_SLIP_DATA_START + flashDataSize], 0xFF, ESP_SLIP_MTU - flashDataSize);
  }

  return espSlipExchange(sendBuffer, &receiverPacket, &senderPacket, uart2SendDataDmaBlocking, uart2GetCharWithTimeout, 100);
}
