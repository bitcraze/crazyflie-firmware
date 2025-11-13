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
 * color_led_deck.c - Deck driver for the Color LED deck
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
#include "color_led_deck.h"

#define DEBUG_MODULE "COLORLED"
#include "debug.h"
#include "config.h"
#include "led_deck_controller.h"


static bool isInit = false;
static uint8_t brightnessCorr = true;

static uint32_t currentWrgb8888 = 0;
static uint32_t wrgb8888 = 0;

// Thermal status from deck
static uint8_t deckTemperature = 0;
static uint8_t throttlePercentage = 0;

static void task(void* param);
static paramVarId_t wrgbParamId;

// Generic LED controller callback
static void colorLedDeckSetColor(const uint8_t *rgb888) {
  // White extraction: W = min(R, G, B), then subtract from RGB
  uint8_t w = rgb888[0];
  if (rgb888[1] < w) w = rgb888[1];
  if (rgb888[2] < w) w = rgb888[2];

  // Subtract white from RGB to avoid double-brightness
  uint8_t r = rgb888[0] - w;
  uint8_t g = rgb888[1] - w;
  uint8_t b = rgb888[2] - w;

  // Build WRGB value: 0xWWRRGGBB
  uint32_t newWrgb = ((uint32_t)w << 24) |
                     ((uint32_t)r << 16) |
                     ((uint32_t)g << 8) |
                     b;

  paramSetInt(wrgbParamId, newWrgb);
}

static const ledDeckHandlerDef_t colorLedDeckLedHandler = {
  .setColor = colorLedDeckSetColor,
};


// Gamma correction LUT (gamma = 2.0 with minimum threshold)
// Input 0 -> 0 (off), Input 1-255 -> 3-255 (gamma corrected)
// Minimum output of 3 ensures LEDs start at lowest visible level
static const uint8_t gamma8[256] = {
      0,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   4,   4,   4,   4,
      4,   4,   4,   4,   5,   5,   5,   5,   5,   5,   6,   6,   6,   6,   6,   7,
      7,   7,   7,   8,   8,   8,   9,   9,   9,  10,  10,  10,  11,  11,  11,  12,
     12,  12,  13,  13,  13,  14,  14,  15,  15,  16,  16,  16,  17,  17,  18,  18,
     19,  19,  20,  20,  21,  21,  22,  23,  23,  24,  24,  25,  25,  26,  27,  27,
     28,  28,  29,  30,  30,  31,  32,  32,  33,  34,  34,  35,  36,  37,  37,  38,
     39,  39,  40,  41,  42,  43,  43,  44,  45,  46,  47,  47,  48,  49,  50,  51,
     52,  52,  53,  54,  55,  56,  57,  58,  59,  60,  61,  62,  63,  64,  65,  66,
     66,  67,  68,  70,  71,  72,  73,  74,  75,  76,  77,  78,  79,  80,  81,  82,
     83,  84,  86,  87,  88,  89,  90,  91,  93,  94,  95,  96,  97,  99, 100, 101,
    102, 103, 105, 106, 107, 109, 110, 111, 112, 114, 115, 116, 118, 119, 120, 122,
    123, 124, 126, 127, 129, 130, 131, 133, 134, 136, 137, 139, 140, 141, 143, 144,
    146, 147, 149, 150, 152, 153, 155, 156, 158, 160, 161, 163, 164, 166, 167, 169,
    171, 172, 174, 176, 177, 179, 180, 182, 184, 185, 187, 189, 191, 192, 194, 196,
    197, 199, 201, 203, 204, 206, 208, 210, 212, 213, 215, 217, 219, 221, 223, 224,
    226, 228, 230, 232, 234, 236, 238, 239, 241, 243, 245, 247, 249, 251, 253, 255
};

static inline uint8_t applyGammaCorrection(const uint8_t value) {
    return gamma8[value];
}

static wrgb_t normalizeLuminance(const wrgb_t *input_rgb) {
    // Normalize brightness across all channels based on LED datasheet luminance values
    // This ensures equal perceived brightness when channels are set to the same value
    //
    // This scales ALL channels down to match the weakest LED
    // This significantly reduces maximum achievable brightness but provides perceptual uniformity
    //
    // Use brightnessCorr parameter to bypass this normalization for maximum brightness

    uint8_t target_lumens = fminf(fminf(fminf(LED_LUMINANCE.r_lumens, LED_LUMINANCE.g_lumens),
                                            LED_LUMINANCE.b_lumens),
                                        LED_LUMINANCE.w_lumens);

    wrgb_t result = {
        .w = input_rgb->w * target_lumens / LED_LUMINANCE.w_lumens,
        .r = input_rgb->r * target_lumens / LED_LUMINANCE.r_lumens,
        .g = input_rgb->g * target_lumens / LED_LUMINANCE.g_lumens,
        .b = input_rgb->b * target_lumens / LED_LUMINANCE.b_lumens
    };

    return result;
}

static wrgb_t applyBrightnessCorrection(const wrgb_t *input_wrgb){
    // Apply intensity scaling based on LED datasheet
    wrgb_t led_wrgb = normalizeLuminance(input_wrgb);

    // Apply gamma correction for perceptual linearity
    // This makes brightness changes feel uniform across the entire range
    led_wrgb.w = applyGammaCorrection(led_wrgb.w);
    led_wrgb.r = applyGammaCorrection(led_wrgb.r);
    led_wrgb.g = applyGammaCorrection(led_wrgb.g);
    led_wrgb.b = applyGammaCorrection(led_wrgb.b);

    return led_wrgb;
}

static bool checkProtocolVersion(void) {
  // Fixed packet size: CMD + 4 dummy bytes
  uint8_t cmd[TXBUFFERSIZE] = {CMD_GET_VERSION, 0, 0, 0, 0};
  uint8_t response[RXBUFFERSIZE];

  // Send version request (5 bytes to match fixed packet size)
  if (i2cdevWrite(I2C1_DEV, COLORLED_BOT_DECK_I2C_ADDRESS, TXBUFFERSIZE, cmd) == false) {
    DEBUG_PRINT("Failed to request version\n");
    return false;
  }

  vTaskDelay(M2T(10)); // Give the LED deck time to prepare response

  // Read version response
  if (i2cdevRead(I2C1_DEV, COLORLED_BOT_DECK_I2C_ADDRESS, RXBUFFERSIZE, response) == false) {
    DEBUG_PRINT("Failed to read version\n");
    return false;
  }

  if (response[0] != CMD_GET_VERSION) {
    DEBUG_PRINT("Invalid version response\n");
    return false;
  }

  uint8_t version = response[1];
  DEBUG_PRINT("Color LED Deck Protocol version: %d (required: %d)\n", version, COLORLED_PROTOCOL_VERSION_REQUIRED);

  if (version != COLORLED_PROTOCOL_VERSION_REQUIRED) {
    DEBUG_PRINT("Protocol version mismatch!\n");
    return false;
  }

  return true;
}

static void colorLedDeckInit(DeckInfo *info) {
  if (isInit) {
    return;
  }

  deckctrl_gpio_set_direction(info, DECKCTRL_GPIO_PIN_0, true);
  deckctrl_gpio_write(info, DECKCTRL_GPIO_PIN_0, true);

  xTaskCreate(task, COLORLED_TASK_NAME,
              COLORLED_TASK_STACKSIZE, NULL, COLORLED_TASK_PRIO, NULL);

  // Register with generic LED controller
  wrgbParamId = paramGetVarId("colorled", "wrgb8888");
  ledDeckRegisterHandler(&colorLedDeckLedHandler);

  isInit = true;
}

static bool colorLedDeckTest() {
  if (!isInit) {
    return false;
  }

  if (!checkProtocolVersion()) {
    DEBUG_PRINT("Color LED deck protocol version check failed\n");
    return false;
  }

  return true;
}

static void task(void *param) {
  systemWaitStart();

  TickType_t lastWakeTime = xTaskGetTickCount();
  const TickType_t loopInterval = M2T(10); // 10ms loop interval

  TickType_t lastStatusPoll = xTaskGetTickCount();
  const TickType_t statusPollInterval = M2T(100); // Poll every 100ms

  uint8_t response[RXBUFFERSIZE];

  while (1)
  {
    // Read any available response from the deck
    if (i2cdevRead(I2C1_DEV, COLORLED_BOT_DECK_I2C_ADDRESS, RXBUFFERSIZE, response)) {
      // Process response based on command type
      switch (response[0]) {
        case CMD_GET_THERMAL_STATUS:
          deckTemperature = response[1];
          throttlePercentage = response[2];
          break;

        case CMD_GET_VERSION:
          // Version responses are handled during init
          break;

        case CMD_SET_COLOR:
          // No response expected for color commands
          break;

        default:
          // Unknown or empty response
          break;
      }
    }

    // Send thermal status request periodically
    if (xTaskGetTickCount() - lastStatusPoll >= statusPollInterval) {
      uint8_t cmd[TXBUFFERSIZE] = {CMD_GET_THERMAL_STATUS, 0, 0, 0, 0};
      i2cdevWrite(I2C1_DEV, COLORLED_BOT_DECK_I2C_ADDRESS, TXBUFFERSIZE, cmd);
      lastStatusPoll = xTaskGetTickCount();
    }

    // Send color updates when changed
    if (currentWrgb8888 != wrgb8888) {
      currentWrgb8888 = wrgb8888;

      // Unpack to struct (format: 0xWWRRGGBB)
      wrgb_t input = {
          .w = (currentWrgb8888 >> 24) & 0xFF,
          .r = (currentWrgb8888 >> 16) & 0xFF,
          .g = (currentWrgb8888 >> 8) & 0xFF,
          .b = currentWrgb8888 & 0xFF
      };

      static wrgb_t output;
      if (brightnessCorr) {
        // Apply correction
        output = applyBrightnessCorrection(&input);
      } else {
        output = input;
      }

      // Format: 0xWWRRGGBB -> Hardware expects [W, R, G, B]
      uint8_t wrgb_data[5] = {
        CMD_SET_COLOR,
        output.w,
        output.r,
        output.g,
        output.b
      };
      i2cdevWrite(I2C1_DEV, COLORLED_BOT_DECK_I2C_ADDRESS, TXBUFFERSIZE, wrgb_data);
    }

    // Maintain precise 10ms loop timing
    vTaskDelayUntil(&lastWakeTime, loopInterval);
  }
}

static const DeckDriver color_led_deck = {
  .vid = 0xBC,
  .pid = 0x13,
  .name = "bcColorLED",

  .init = colorLedDeckInit,
  .test = colorLedDeckTest,
};

DECK_DRIVER(color_led_deck);

PARAM_GROUP_START(colorled)
PARAM_ADD(PARAM_UINT32, wrgb8888, &wrgb8888)
PARAM_ADD(PARAM_UINT8, brightnessCorr, &brightnessCorr)
PARAM_GROUP_STOP(colorled)

LOG_GROUP_START(colorled)
LOG_ADD(LOG_UINT8, deckTemp, &deckTemperature)
LOG_ADD(LOG_UINT8, throttlePct, &throttlePercentage)
LOG_GROUP_STOP(colorled)

PARAM_GROUP_START(deck)

/**
 * @brief Nonzero if [Color LED deck](%https://store.bitcraze.io/collections/decks/products/color-led-deck) is attached
 */
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcColorLED, &isInit)

PARAM_GROUP_STOP(deck)
