/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2025 Bitcraze AB
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
 * hprgbw.c - Deck driver for the High Power RGBW deck
 */

#include "stm32fxxx.h"
#include "FreeRTOS.h"
#include "task.h"

#include "system.h"
#include "deck.h"
#include "log.h"
#include "param.h"
#include "eventtrigger.h"
#include "i2cdev.h"
#include "deckctrl_gpio.h"
#include "math.h"

#define DEBUG_MODULE "HP_RGBW"
#include "debug.h"
#include "config.h"
#include "led_deck_controller.h"


static bool isInit = false;

#define HPRGBW_DECK_I2C_ADDRESS 0x10

// Protocol commands
#define CMD_GET_VERSION 0x00
#define CMD_SET_COLOR   0x01

// Expected protocol version
#define HP_LED_PROTOCOL_VERSION_REQUIRED 1


static uint32_t currentRgbw8888 = 0;
static uint32_t rgbw8888 = 0;


static void task(void* param);
static paramVarId_t rgbwParamId;


// Generic LED controller callback
static void hprgbwSetColor(const uint8_t *rgb888) {
  // White extraction: W = min(R, G, B), then subtract from RGB
  uint8_t w = rgb888[0];
  if (rgb888[1] < w) w = rgb888[1];
  if (rgb888[2] < w) w = rgb888[2];

  // Subtract white from RGB to avoid double-brightness
  uint8_t r = rgb888[0] - w;
  uint8_t g = rgb888[1] - w;
  uint8_t b = rgb888[2] - w;

  // Build RGBW value: 0xWWRRGGBB
  uint32_t newRgbw = ((uint32_t)w << 24) |
                     ((uint32_t)r << 16) |
                     ((uint32_t)g << 8) |
                     b;

  paramSetInt(rgbwParamId, newRgbw);
}

static const LedDeckHandlerDef_t hprgbwLedHandler = {
  .setColor = hprgbwSetColor,
};

static bool checkProtocolVersion(void) {
  // Fixed packet size: CMD + 4 dummy bytes
  uint8_t cmd[5] = {CMD_GET_VERSION, 0, 0, 0, 0};
  uint8_t response[2];

  // Send version request (5 bytes to match fixed packet size)
  if (i2cdevWrite(I2C1_DEV, HPRGBW_DECK_I2C_ADDRESS, 5, cmd) == false) {
    DEBUG_PRINT("Failed to request version\n");
    return false;
  }

  vTaskDelay(M2T(10)); // Give the LED deck time to prepare response

  // Read version response
  if (i2cdevRead(I2C1_DEV, HPRGBW_DECK_I2C_ADDRESS, 2, response) == false) {
    DEBUG_PRINT("Failed to read version\n");
    return false;
  }

  if (response[0] != CMD_GET_VERSION) {
    DEBUG_PRINT("Invalid version response\n");
    return false;
  }

  uint8_t version = response[1];
  DEBUG_PRINT("HP LED Protocol version: %d (required: %d)\n", version, HP_LED_PROTOCOL_VERSION_REQUIRED);

  if (version != HP_LED_PROTOCOL_VERSION_REQUIRED) {
    DEBUG_PRINT("Protocol version mismatch!\n");
    return false;
  }

  return true;
}

static void hprgbwDeckInit(DeckInfo *info) {
  if (isInit) {
    return;
  }

  deckctrl_gpio_set_direction(info, DECKCTRL_GPIO_PIN_0, true);
  deckctrl_gpio_write(info, DECKCTRL_GPIO_PIN_0, true);

  xTaskCreate(task, HPRGBW_TASK_NAME,
              HPRGBW_TASK_STACKSIZE, NULL, HPRGBW_TASK_PRI, NULL);

  // Register with generic LED controller
  rgbwParamId = paramGetVarId("hprgbw", "rgbw8888");
  ledDeckRegisterHandler(&hprgbwLedHandler);

  isInit = true;
}

static bool hprgbwDeckTest() {
  if (!isInit) {
    return false;
  }

  if (!checkProtocolVersion()) {
    DEBUG_PRINT("HP RGBW deck protocol version check failed\n");
    return false;
  }

  return true;
}

static void task(void *param) {
  systemWaitStart();

  // TODO: Check LED-Deck firmware version

  while (1)
  {
    if (currentRgbw8888 != rgbw8888) {
      currentRgbw8888 = rgbw8888;
      // Format: 0xWWRRGGBB -> Hardware expects [R, G, B, W]
      uint8_t rgbw_data[5] = {
        CMD_SET_COLOR,
        (currentRgbw8888 >> 16) & 0xFF, // r
        (currentRgbw8888 >> 8) & 0xFF,  // g
        (currentRgbw8888) & 0xFF,       // b
        (currentRgbw8888 >> 24) & 0xFF  // w
      };
      i2cdevWrite(I2C1_DEV, HPRGBW_DECK_I2C_ADDRESS, sizeof(rgbw_data), rgbw_data);
    }

    vTaskDelay(M2T(10));
  }
}

static const DeckDriver hprgbw_deck = {
  .vid = 0xBC,
  .pid = 0x13,
  .name = "bcHPRGBW",

  .init = hprgbwDeckInit,
  .test = hprgbwDeckTest,
};

DECK_DRIVER(hprgbw_deck);

PARAM_GROUP_START(hprgbw)
PARAM_ADD(PARAM_UINT32, rgbw8888, &rgbw8888)
PARAM_GROUP_STOP(hprgbw)
