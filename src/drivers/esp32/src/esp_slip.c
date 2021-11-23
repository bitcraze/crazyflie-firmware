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
 * @file esp_slip.c
 * Protocol for assembling, sending, receiving and decoding SLIP packets to/from the ESP32 ROM bootloader
 *  
 */

#include <string.h>

#include "esp_slip.h"
#include "uart2.h"

#define ESP_OVERHEAD_LEN 8

static uint32_t sendSize;

static uint8_t generateChecksum(uint8_t *sendBuffer, esp_slip_send_packet *senderPacket)
{
  uint8_t checksum = 0xEF; // seed
  for (int i = 0; i < senderPacket->dataSize - 16; i++)
  {
    checksum ^= sendBuffer[9 + 16 + i];
  }
  return checksum;
}

static void sendSlipPacket(uint32_t size, uint8_t *data, coms_sendbuffer_t sendBufferFn)
{
  uint32_t i;
  static uint8_t dmaSendBuffer[UART2_DMA_BUFFER_SIZE];
  uint32_t dmaSendSize = 0;

  for (i = 0; i < size; i++)
  {
    if ((data[i] == 0xC0 && i != 0 && i != size - 1) || (data[i] == 0xDB && i != 0 && i != size - 1))
    {
      for (int j = 0; j < 2; j++)
      {
        j == 0 ? (dmaSendBuffer[dmaSendSize] = 0xDB) : data[i] == 0xC0 ? (dmaSendBuffer[dmaSendSize] = 0xDC)
                                                                       : (dmaSendBuffer[dmaSendSize] = 0xDD);
        dmaSendSize += 1;
      }
    }
    else
    {
      dmaSendBuffer[dmaSendSize] = data[i];
      dmaSendSize += 1;
    }
    if (dmaSendSize >= (UART2_DMA_BUFFER_SIZE - 2))
    {
      sendBufferFn(dmaSendSize, &dmaSendBuffer[0]);
      dmaSendSize = 0;
    }
  }
  if (dmaSendSize)
  {
    sendBufferFn(dmaSendSize, &dmaSendBuffer[0]);
  }
}
static void assembleBuffer(uint8_t *sendBuffer, esp_slip_send_packet *senderPacket)
{
  sendSize = senderPacket->dataSize + ESP_OVERHEAD_LEN + 2;

  sendBuffer[0] = 0xC0;
  sendBuffer[1] = DIR_CMD;
  sendBuffer[2] = senderPacket->command;
  sendBuffer[3] = (uint8_t)((senderPacket->dataSize >> 0) & 0x000000FF);
  sendBuffer[4] = (uint8_t)((senderPacket->dataSize >> 8) & 0x000000FF);

  if (senderPacket->command == FLASH_DATA) // or MEM_DATA
  {
    uint32_t checksum = (uint32_t)generateChecksum(sendBuffer, senderPacket);
    sendBuffer[5] = (uint8_t)((checksum >> 0) & 0x000000FF);
    sendBuffer[6] = (uint8_t)((checksum >> 8) & 0x000000FF);
    sendBuffer[7] = (uint8_t)((checksum >> 16) & 0x000000FF);
    sendBuffer[8] = (uint8_t)((checksum >> 24) & 0x000000FF);
  }
  else
  {
    sendBuffer[5] = 0x00;
    sendBuffer[6] = 0x00;
    sendBuffer[7] = 0x00;
    sendBuffer[8] = 0x00;
  }

  sendBuffer[9 + senderPacket->dataSize] = 0xC0;
}
