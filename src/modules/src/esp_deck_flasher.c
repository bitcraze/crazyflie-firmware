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
 * @file esp_deck_flasher.c
 * Handles flashing of binaries on the ESP32
 *
 */

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#define DEBUG_MODULE "ESP_FLASHER"
#include "debug.h"

#include "FreeRTOS.h"
#include "aideck.h"
#include "deck.h"
#include "esp_deck_flasher.h"
#include "esp_rom_bootloader.h"
#include "uart2.h"


uint32_t espDeckFlasherNewBinarySize = 0;

static uint32_t sequenceNumber;
static uint32_t numberOfFlashBuffers;
static uint8_t sendBuffer[ESP_SLIP_DATA_START + ESP_SLIP_MTU + ESP_SLIP_STOP_CODE_LEN];
static uint8_t overshoot;
static uint32_t sendBufferIndex;


static bool initialize() {
  if (!espRomBootloaderSync(&sendBuffer[0])){
    DEBUG_PRINT("Write failed - cannot sync with bootloader\n");
    return false;
  }

  if (!espRomBootloaderSpiAttach(&sendBuffer[0])){
    DEBUG_PRINT("Write failed - cannot attach SPI flash\n");
    return false;
  }

  numberOfFlashBuffers = 1 + (espDeckFlasherNewBinarySize - 1) / ESP_SLIP_MTU;
  // It should be possible to send the actual binary size to the ESP but we get flashing errors sometimes for
  // the last (smaller) buffer. Solve it by sending full buffers.
  const uint32_t quantizedSize = numberOfFlashBuffers * ESP_SLIP_MTU;
  if (!espRomBootloaderFlashBegin(&sendBuffer[0], numberOfFlashBuffers, quantizedSize, ESP_FW_ADDRESS)) {
    DEBUG_PRINT("Failed to start flashing\n");
    return false;
  }

  sequenceNumber = 0;
  sendBufferIndex = 0;

  return true;
}

static void appendToSendBuffer(const uint8_t writeLen, const uint8_t *buffer) {
  const uint32_t tail = ESP_SLIP_DATA_START + sendBufferIndex;

  const uint32_t newIndex = sendBufferIndex + writeLen;
  if (newIndex <= ESP_SLIP_MTU) {
    memcpy(&sendBuffer[tail], buffer, writeLen);
    sendBufferIndex = newIndex;
  } else {
    overshoot = newIndex - ESP_SLIP_MTU;
    memcpy(&sendBuffer[tail], buffer, writeLen - overshoot);
    sendBufferIndex = ESP_SLIP_MTU;
  }
}

static void appendOvershootToSendBuffer(const uint8_t writeLen, const uint8_t *buffer) {
  // put overshoot into send buffer for next send & update sendBufferIndex
  if (overshoot) {
    memcpy(&sendBuffer[ESP_SLIP_DATA_START], &buffer[writeLen - overshoot], overshoot);
    sendBufferIndex = overshoot;
    overshoot = 0;
  }
}


bool espDeckFlasherWrite(const uint32_t memAddr, const uint8_t writeLen, const uint8_t *buffer, const DeckMemDef_t* memDef) {
  if (memAddr == 0) {
    if (!initialize()) {
      return false;
    }
  }

  appendToSendBuffer(writeLen, buffer);

  const bool isSendBufferFull = (sendBufferIndex == ESP_SLIP_MTU);
  if (isSendBufferFull) {
    if (!espRomBootloaderFlashData(sendBuffer, sendBufferIndex, sequenceNumber)) {
      DEBUG_PRINT("Flash write failed\n");
      return false;
    }
    sendBufferIndex = 0;
    sequenceNumber++;

    appendOvershootToSendBuffer(writeLen, buffer);
  }

  // If this is the last radio packet and we have a half full flash buffer, send it (with padding) to the ESP
  const bool isLastPacket = ((memAddr + writeLen) == espDeckFlasherNewBinarySize);
  if (isLastPacket && sendBufferIndex) {
    if (!espRomBootloaderFlashData(sendBuffer, sendBufferIndex, sequenceNumber)) {
      DEBUG_PRINT("Flash write failed\n");
      return false;
    }
  }

  return true;
}
