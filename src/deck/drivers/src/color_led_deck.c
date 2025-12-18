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
#include "i2c_dfu.h"

#define DEBUG_MODULE "COLORLED"
#include "debug.h"
#include "config.h"
#include "led_deck_controller.h"


// Context structure to hold per-instance state
typedef struct {
  bool isInit;
  uint8_t brightnessCorr;
  uint32_t currentWrgb8888;
  uint32_t wrgb8888;
  uint8_t deckTemperature;
  uint8_t throttlePercentage;
  uint8_t ledPosition;
  uint16_t ledCurrent[4];  // LED current in milliamps for 4 channels (R, G, B, W)
  paramVarId_t wrgbParamId;
  uint8_t i2cAddress;
  DeckInfo *deckInfo;
  bool isInBootloader;
  bool isInFirmware;
  uint8_t testResults;  // Bitmap of self-test results
} colorLedContext_t;

// Instance indices for accessing contexts array
#define BOTTOM_IDX 0
#define TOP_IDX 1

// Two instances: bottom and top
static colorLedContext_t contexts[2] = {
  { .isInit = false, .brightnessCorr = true, .i2cAddress = COLORLED_BOT_DECK_I2C_ADDRESS, .isInFirmware = true, .isInBootloader = false, .testResults = 0 }, // bottom
  { .isInit = false, .brightnessCorr = true, .i2cAddress = COLORLED_TOP_DECK_I2C_ADDRESS, .isInFirmware = true, .isInBootloader = false, .testResults = 0 }  // top
};

// Enable deck power by pulling high
#define GPIO_PWR_EN DECKCTRL_GPIO_PIN_0
// Enable DFU by pulling high (note that this must be input for SWD to work)
#define GPIO_DFU_EN DECKCTRL_GPIO_PIN_7
// Set Color LED MCU I2C address LSB by pulling high/low
#define GPIO_I2C_ADDR_LSB DECKCTRL_GPIO_PIN_11

// Self-test result bits (1 = failure, 0 = success)
#define TEST_PROTOCOL_VERSION (1 << 0)  // Bit 0: Protocol version check failed
#define TEST_LED_POSITION     (1 << 1)  // Bit 1: LED position read failed
#define TEST_I2C_ADDR_PIN     (1 << 2)  // Bit 2: I2C address pin test failed

static void task(void* param);
static bool pollThermalStatus(colorLedContext_t *ctx);
static bool pollLedCurrent(colorLedContext_t *ctx);
static bool verifyLedPosition(colorLedContext_t *ctx, uint8_t expectedPosition);
static bool testI2cAddrPin(colorLedContext_t *ctx);

// Generic LED controller callback - forwards to all initialized instances
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

  // Forward to all initialized instances
  for (int i = 0; i < 2; i++) {
    if (contexts[i].isInit) {
      paramSetInt(contexts[i].wrgbParamId, newWrgb);
    }
  }
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

static wrgb_t applyPerceptualScaling(const wrgb_t *input_rgb) {
    // Apply perceptual balance factors from user survey
    // These scale brightness values to achieve perceptually balanced colors
    //
    // This scales ALL channels down to match the weakest LED
    // This significantly reduces maximum achievable brightness but provides perceptual uniformity
    //
    // Use brightnessCorr parameter to bypass this normalization for maximum brightness

    wrgb_t result = {
        .w = (uint8_t)(input_rgb->w * LED_PERCEPTUAL_SCALE.w),
        .r = (uint8_t)(input_rgb->r * LED_PERCEPTUAL_SCALE.r),
        .g = (uint8_t)(input_rgb->g * LED_PERCEPTUAL_SCALE.g),
        .b = (uint8_t)(input_rgb->b * LED_PERCEPTUAL_SCALE.b)
    };

    return result;
}

static wrgb_t applyBrightnessCorrection(const wrgb_t *input_wrgb){
    // Apply perceptual scaling from user survey
    wrgb_t led_wrgb = applyPerceptualScaling(input_wrgb);

    // Apply gamma correction for perceptual linearity
    // This makes brightness changes feel uniform across the entire range
    led_wrgb.w = applyGammaCorrection(led_wrgb.w);
    led_wrgb.r = applyGammaCorrection(led_wrgb.r);
    led_wrgb.g = applyGammaCorrection(led_wrgb.g);
    led_wrgb.b = applyGammaCorrection(led_wrgb.b);

    return led_wrgb;
}

static bool checkProtocolVersion(uint8_t i2cAddress) {
  // Fixed packet size: CMD + 4 dummy bytes
  uint8_t cmd[TXBUFFERSIZE] = {CMD_GET_VERSION, 0, 0, 0, 0};
  uint8_t response[RXBUFFERSIZE];

  // Send version request (5 bytes to match fixed packet size)
  if (i2cdevWrite(I2C1_DEV, i2cAddress, TXBUFFERSIZE, cmd) == false) {
    DEBUG_PRINT("Failed to request version\n");
    return false;
  }

  vTaskDelay(M2T(1)); // Give the LED deck time to prepare response

  // Read version response
  if (i2cdevRead(I2C1_DEV, i2cAddress, RXBUFFERSIZE, response) == false) {
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

// Common init function used by both bottom and top variants
static void colorLedDeckInit(DeckInfo *info, colorLedContext_t *ctx, const char *taskName, const char *paramGroupName) {
  if (ctx->isInit) {
    return;
  }

  ctx->deckInfo = info;

  if (!deckctrl_gpio_set_direction(info, GPIO_PWR_EN, OUTPUT)) {
    DEBUG_PRINT("Failed to configure GPIO 0 as output for deck power enable\n");
    return;
  }

  if (!deckctrl_gpio_write(info, GPIO_PWR_EN, HIGH)) {
    DEBUG_PRINT("Failed to set GPIO 0 HIGH for deck power enable\n");
    return;
  }

  vTaskDelay(M2T(50)); // Wait for deck to boot

  if (xTaskCreate(task, taskName,
                  COLORLED_TASK_STACKSIZE, ctx, COLORLED_TASK_PRIO, NULL) != pdPASS) {
    DEBUG_PRINT("Failed to create task %s\n", taskName);
    return;
  }

  // Register with generic LED controller
  ctx->wrgbParamId = paramGetVarId(paramGroupName, "wrgb8888");

  // Register LED handler only once (it forwards to all instances)
  static bool handlerRegistered = false;
  if (!handlerRegistered) {
    ledDeckRegisterHandler(&colorLedDeckLedHandler);
    handlerRegistered = true;
  }

  ctx->isInit = true;
}

// Common test function used by both bottom and top variants
static bool colorLedDeckTest(colorLedContext_t *ctx, uint8_t expectedPosition) {
  // Clear test results
  ctx->testResults = 0;

  if (!ctx->isInit) {
    return false;
  }

  // Test 1: Protocol version check
  if (!checkProtocolVersion(ctx->i2cAddress)) {
    ctx->testResults |= TEST_PROTOCOL_VERSION;
    DEBUG_PRINT("Color LED deck protocol version check failed\n");
    return false;
  }

  // Test 2: Verify LED position matches expected (fixed by hardware)
  if (!verifyLedPosition(ctx, expectedPosition)) {
    ctx->testResults |= TEST_LED_POSITION;
    DEBUG_PRINT("LED position test failed (wrong side or communication error)\n");
    return false;
  }

  // Test 3: I2C_ADDRESS pin test
  if (!testI2cAddrPin(ctx)) {
    ctx->testResults |= TEST_I2C_ADDR_PIN;
    DEBUG_PRINT("Failed I2C address pin test\n");
    return false;
  }

  return true;
}

// Bottom deck wrapper functions
static void colorLedBottomDeckInit(DeckInfo *info) {
  colorLedDeckInit(info, &contexts[BOTTOM_IDX], "COLOR_LED_BOTTOM", "colorLedBot");
}

static bool colorLedBottomDeckTest() {
  return colorLedDeckTest(&contexts[BOTTOM_IDX], COLORLED_LED_POS_BOTTOM);
}

// Top deck wrapper functions
static void colorLedTopDeckInit(DeckInfo *info) {
  // GPIO 11 controls the I2C address to differentiate top deck from bottom deck
  if (!deckctrl_gpio_set_direction(info, GPIO_I2C_ADDR_LSB, OUTPUT)) {
    DEBUG_PRINT("Failed to configure GPIO 11 as output for I2C address selection\n");
    return;
  }

  if (!deckctrl_gpio_write(info, GPIO_I2C_ADDR_LSB, HIGH)) {
    DEBUG_PRINT("Failed to set GPIO 11 HIGH for I2C address selection\n");
    return;
  }
  colorLedDeckInit(info, &contexts[TOP_IDX], "COLOR_LED_TOP", "colorLedTop");
}

static bool colorLedTopDeckTest() {
  return colorLedDeckTest(&contexts[TOP_IDX], COLORLED_LED_POS_TOP);
}

static bool pollThermalStatus(colorLedContext_t *ctx) {
  uint8_t cmd[TXBUFFERSIZE] = {CMD_GET_THERMAL_STATUS, 0, 0, 0, 0};
  uint8_t response[RXBUFFERSIZE];

  if (i2cdevWrite(I2C1_DEV, ctx->i2cAddress, TXBUFFERSIZE, cmd)) {
    vTaskDelay(M2T(1));
    if (i2cdevRead(I2C1_DEV, ctx->i2cAddress, RXBUFFERSIZE, response)) {
      if (response[0] == CMD_GET_THERMAL_STATUS) {
        ctx->deckTemperature = response[1];
        ctx->throttlePercentage = response[2];
        return true;
      }
    }
  }
  return false;
}

static bool pollLedCurrent(colorLedContext_t *ctx) {
  uint8_t cmd[TXBUFFERSIZE] = {CMD_GET_LED_CURRENT, 0, 0, 0, 0};
  uint8_t response[RXBUFFERSIZE];

  if (i2cdevWrite(I2C1_DEV, ctx->i2cAddress, TXBUFFERSIZE, cmd)) {
    vTaskDelay(M2T(1));
    if (i2cdevRead(I2C1_DEV, ctx->i2cAddress, RXBUFFERSIZE, response)) {
      if (response[0] == CMD_GET_LED_CURRENT) {
        // Each current value is 2 bytes: high byte, low byte (milliamps)
        ctx->ledCurrent[0] = (response[1] << 8) | response[2];  // Red
        ctx->ledCurrent[1] = (response[3] << 8) | response[4];  // Green
        ctx->ledCurrent[2] = (response[5] << 8) | response[6];  // Blue
        ctx->ledCurrent[3] = (response[7] << 8) | response[8];  // White
        return true;
      }
    }
  }
  return false;
}

static bool verifyLedPosition(colorLedContext_t *ctx, uint8_t expectedPosition) {
  uint8_t cmd[TXBUFFERSIZE] = {CMD_GET_LED_POSITION, 0, 0, 0, 0};
  uint8_t response[RXBUFFERSIZE];

  if (!i2cdevWrite(I2C1_DEV, ctx->i2cAddress, TXBUFFERSIZE, cmd)) {
    DEBUG_PRINT("Failed to write CMD_GET_LED_POSITION\n");
    return false;
  }

  vTaskDelay(M2T(1));

  if (!i2cdevRead(I2C1_DEV, ctx->i2cAddress, RXBUFFERSIZE, response)) {
    DEBUG_PRINT("Failed to read LED position response\n");
    return false;
  }

  if (response[0] != CMD_GET_LED_POSITION) {
    DEBUG_PRINT("Invalid response command: 0x%02x\n", response[0]);
    return false;
  }

  uint8_t actualPosition = response[1];
  ctx->ledPosition = actualPosition;

  if (actualPosition != expectedPosition) {
    const char* expectedStr = (expectedPosition == COLORLED_LED_POS_BOTTOM) ? "BOTTOM" : "TOP";
    const char* actualStr;
    if (actualPosition == COLORLED_LED_POS_BOTTOM) {
      actualStr = "BOTTOM";
    } else if (actualPosition == COLORLED_LED_POS_TOP) {
      actualStr = "TOP";
    } else if (actualPosition == COLORLED_LED_POS_NONE) {
      actualStr = "NONE";
    } else {
      actualStr = "UNKNOWN";
    }
    DEBUG_PRINT("LED position mismatch: expected 0x%02x (%s side), got 0x%02x (%s side)\n",
                expectedPosition, expectedStr, actualPosition, actualStr);
    return false;
  }

  return true;
}

static bool testI2cAddrPin(colorLedContext_t *ctx) {
  uint8_t cmd[TXBUFFERSIZE] = {CMD_GET_I2C_ADDR_PIN, 0, 0, 0, 0};
  uint8_t response[RXBUFFERSIZE];

  // Configure pin as output
  if (!deckctrl_gpio_set_direction(ctx->deckInfo, GPIO_I2C_ADDR_LSB, OUTPUT)) {
    DEBUG_PRINT("Failed to configure GPIO 11 as output for I2C address selection\n");
    return false;
  }

  // Test with pin LOW
  if (!deckctrl_gpio_write(ctx->deckInfo, GPIO_I2C_ADDR_LSB, LOW)) {
    DEBUG_PRINT("Failed to set GPIO 11 LOW for I2C address selection\n");
    return false;
  }

  vTaskDelay(M2T(2)); // Allow pin to settle

  if (!i2cdevWrite(I2C1_DEV, ctx->i2cAddress, TXBUFFERSIZE, cmd)) {
    DEBUG_PRINT("Failed to write I2C command for LOW state test\n");
    return false;
  }
  vTaskDelay(M2T(1));

  if (!i2cdevRead(I2C1_DEV, ctx->i2cAddress, RXBUFFERSIZE, response)) {
    DEBUG_PRINT("Failed to read I2C response for LOW state test\n");
    return false;
  }

  if (response[0] != CMD_GET_I2C_ADDR_PIN) {
    DEBUG_PRINT("Invalid response command: 0x%02x\n", response[0]);
    return false;
  }

  uint8_t pinState = response[1];
  if (pinState != 0) {
    DEBUG_PRINT("I2C Address Pin test failed: expected LOW state\n");
    return false;
  }

  // Test with pin HIGH
  if (!deckctrl_gpio_write(ctx->deckInfo, GPIO_I2C_ADDR_LSB, HIGH)) {
    DEBUG_PRINT("Failed to set GPIO 11 HIGH for I2C address selection\n");
    return false;
  }

  vTaskDelay(M2T(2)); // Allow pin to settle

  if (!i2cdevWrite(I2C1_DEV, ctx->i2cAddress, TXBUFFERSIZE, cmd)) {
    DEBUG_PRINT("Failed to write I2C command for HIGH state test\n");
    return false;
  }
  vTaskDelay(M2T(1));

  if (!i2cdevRead(I2C1_DEV, ctx->i2cAddress, RXBUFFERSIZE, response)) {
    DEBUG_PRINT("Failed to read I2C response for HIGH state test\n");
    return false;
  }

  if (response[0] != CMD_GET_I2C_ADDR_PIN) {
    DEBUG_PRINT("Invalid response command: 0x%02x\n", response[0]);
    return false;
  }

  pinState = response[1];
  if (pinState != 1) {
    DEBUG_PRINT("I2C Address Pin test failed: expected HIGH state\n");
    return false;
  }

  return true;
}

static void task(void *param) {
  colorLedContext_t *ctx = (colorLedContext_t *)param;
  systemWaitStart();

  TickType_t lastWakeTime = xTaskGetTickCount();
  const TickType_t loopInterval = M2T(10); // 10ms loop interval

  TickType_t lastThermalPoll = xTaskGetTickCount();
  const TickType_t thermalPollInterval = M2T(100); // Poll thermal status every 100ms

  TickType_t lastCurrentPoll = xTaskGetTickCount();
  const TickType_t currentPollInterval = M2T(1000); // Poll LED current every 1000ms

  while (1)
  {
    if (ctx->isInFirmware) {
      // Poll thermal status periodically
      if (xTaskGetTickCount() - lastThermalPoll >= thermalPollInterval) {
        if (!pollThermalStatus(ctx)) {
          DEBUG_PRINT("Failed to poll thermal status\n");
        }
        lastThermalPoll = xTaskGetTickCount();
      }

      // Poll LED current periodically
      if (xTaskGetTickCount() - lastCurrentPoll >= currentPollInterval) {
        if (!pollLedCurrent(ctx)) {
          DEBUG_PRINT("Failed to poll LED current\n");
        }
        lastCurrentPoll = xTaskGetTickCount();
      }

      // Send color updates when changed
      if (ctx->currentWrgb8888 != ctx->wrgb8888) {
        ctx->currentWrgb8888 = ctx->wrgb8888;

        // Unpack to struct (format: 0xWWRRGGBB)
        wrgb_t input = {
            .w = (ctx->currentWrgb8888 >> 24) & 0xFF,
            .r = (ctx->currentWrgb8888 >> 16) & 0xFF,
            .g = (ctx->currentWrgb8888 >> 8) & 0xFF,
            .b = ctx->currentWrgb8888 & 0xFF
        };

        wrgb_t output;
        if (ctx->brightnessCorr) {
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
        if (!i2cdevWrite(I2C1_DEV, ctx->i2cAddress, TXBUFFERSIZE, wrgb_data)) {
          DEBUG_PRINT("Failed to write color command to deck at I2C address 0x%02X\n", ctx->i2cAddress);
        }
      }
    }

    // Maintain precise 10ms loop timing
    vTaskDelayUntil(&lastWakeTime, loopInterval);
  }
}

// Bottom deck bootloader functions
static bool colorFlasherBottomWriteFlash(const uint32_t memAddr, const uint8_t writeLen, const uint8_t* buffer, const DeckMemDef_t* memDef) {
  return dfu_i2c_write(DFU_STM32C0_I2C_ADDRESS, memAddr, buffer, writeLen);
}

static bool colorFlasherBottomReadFlash(const uint32_t memAddr, const uint8_t readLen, uint8_t* buffer) {
  return dfu_i2c_read(DFU_STM32C0_I2C_ADDRESS, memAddr, buffer, readLen);
}

static uint8_t colorFlasherBottomPropertiesQuery() {
  uint8_t result = 0;

  result |= DECK_MEMORY_MASK_STARTED | DECK_MEMORY_MASK_SUPPORTS_HOT_RESTART;

  if (contexts[BOTTOM_IDX].isInBootloader) {
    result |= DECK_MEMORY_MASK_BOOT_LOADER_ACTIVE;
  }

  return result;
}

static void resetColorBottomDeckToBootloader() {
  contexts[BOTTOM_IDX].isInFirmware = false;

  if (!deckctrl_gpio_write(contexts[BOTTOM_IDX].deckInfo, GPIO_PWR_EN, LOW)) {
    DEBUG_PRINT("Bottom deck: Failed to set GPIO_PWR_EN LOW\n");
    return;
  }
  if (!deckctrl_gpio_set_direction(contexts[BOTTOM_IDX].deckInfo, GPIO_DFU_EN, OUTPUT)) {
    DEBUG_PRINT("Bottom deck: Failed to configure GPIO_DFU_EN as output\n");
    return;
  }
  if (!deckctrl_gpio_write(contexts[BOTTOM_IDX].deckInfo, GPIO_DFU_EN, HIGH)) {
    DEBUG_PRINT("Bottom deck: Failed to set GPIO_DFU_EN HIGH\n");
    return;
  }
  vTaskDelay(M2T(10));
  if (!deckctrl_gpio_write(contexts[BOTTOM_IDX].deckInfo, GPIO_PWR_EN, HIGH)) {
    DEBUG_PRINT("Bottom deck: Failed to set GPIO_PWR_EN HIGH\n");
    return;
  }
  vTaskDelay(M2T(10));
  if (!deckctrl_gpio_set_direction(contexts[BOTTOM_IDX].deckInfo, GPIO_DFU_EN, INPUT)) {
    DEBUG_PRINT("Bottom deck: Failed to configure GPIO_DFU_EN as input\n");
    return;
  }

  contexts[BOTTOM_IDX].isInBootloader = true;
}

static void resetColorBottomDeckToFw() {
  contexts[BOTTOM_IDX].isInBootloader = false;

  if (!deckctrl_gpio_write(contexts[BOTTOM_IDX].deckInfo, GPIO_PWR_EN, LOW)) {
    DEBUG_PRINT("Bottom deck: Failed to set GPIO_PWR_EN LOW\n");
    return;
  }
  vTaskDelay(M2T(10));
  if (!deckctrl_gpio_write(contexts[BOTTOM_IDX].deckInfo, GPIO_PWR_EN, HIGH)) {
    DEBUG_PRINT("Bottom deck: Failed to set GPIO_PWR_EN HIGH\n");
    return;
  }

  contexts[BOTTOM_IDX].isInFirmware = true;
}

// Top deck bootloader functions
static bool colorFlasherTopWriteFlash(const uint32_t memAddr, const uint8_t writeLen, const uint8_t* buffer, const DeckMemDef_t* memDef) {
  return dfu_i2c_write(DFU_STM32C0_I2C_ADDRESS, memAddr, buffer, writeLen);
}

static bool colorFlasherTopReadFlash(const uint32_t memAddr, const uint8_t readLen, uint8_t* buffer) {
  return dfu_i2c_read(DFU_STM32C0_I2C_ADDRESS, memAddr, buffer, readLen);
}

static uint8_t colorFlasherTopPropertiesQuery() {
  uint8_t result = 0;

  result |= DECK_MEMORY_MASK_STARTED | DECK_MEMORY_MASK_SUPPORTS_HOT_RESTART;

  if (contexts[TOP_IDX].isInBootloader) {
    result |= DECK_MEMORY_MASK_BOOT_LOADER_ACTIVE;
  }

  return result;
}

static void resetColorTopDeckToBootloader() {
  contexts[TOP_IDX].isInFirmware = false;

  if (!deckctrl_gpio_write(contexts[TOP_IDX].deckInfo, GPIO_I2C_ADDR_LSB, LOW)) {
    DEBUG_PRINT("Top deck: Failed to set GPIO_I2C_ADDR_LSB LOW\n");
    return;
  }
  if (!deckctrl_gpio_write(contexts[TOP_IDX].deckInfo, GPIO_PWR_EN, LOW)) {
    DEBUG_PRINT("Top deck: Failed to set GPIO_PWR_EN LOW\n");
    return;
  }
  if (!deckctrl_gpio_set_direction(contexts[TOP_IDX].deckInfo, GPIO_DFU_EN, OUTPUT)) {
    DEBUG_PRINT("Top deck: Failed to configure GPIO_DFU_EN as output\n");
    return;
  }
  if (!deckctrl_gpio_write(contexts[TOP_IDX].deckInfo, GPIO_DFU_EN, HIGH)) {
    DEBUG_PRINT("Top deck: Failed to set GPIO_DFU_EN HIGH\n");
    return;
  }
  vTaskDelay(M2T(10));
  if (!deckctrl_gpio_write(contexts[TOP_IDX].deckInfo, GPIO_PWR_EN, HIGH)) {
    DEBUG_PRINT("Top deck: Failed to set GPIO_PWR_EN HIGH\n");
    return;
  }
  vTaskDelay(M2T(10));
  if (!deckctrl_gpio_set_direction(contexts[TOP_IDX].deckInfo, GPIO_DFU_EN, INPUT)) {
    DEBUG_PRINT("Top deck: Failed to configure GPIO_DFU_EN as input\n");
    return;
  }

  contexts[TOP_IDX].isInBootloader = true;
}

static void resetColorTopDeckToFw() {
  contexts[TOP_IDX].isInBootloader = false;

  if (!deckctrl_gpio_write(contexts[TOP_IDX].deckInfo, GPIO_PWR_EN, LOW)) {
    DEBUG_PRINT("Top deck: Failed to set GPIO_PWR_EN LOW\n");
    return;
  }
  vTaskDelay(M2T(10));
  // Restore GPIO_I2C_ADDR_LSB to HIGH for top deck firmware I2C address
  if (!deckctrl_gpio_write(contexts[TOP_IDX].deckInfo, GPIO_I2C_ADDR_LSB, HIGH)) {
    DEBUG_PRINT("Top deck: Failed to set GPIO_I2C_ADDR_LSB HIGH\n");
    return;
  }
  if (!deckctrl_gpio_write(contexts[TOP_IDX].deckInfo, GPIO_PWR_EN, HIGH)) {
    DEBUG_PRINT("Top deck: Failed to set GPIO_PWR_EN HIGH\n");
    return;
  }

  contexts[TOP_IDX].isInFirmware = true;
}

// Memory definitions for the MCU controlling the LED
static const DeckMemDef_t colorBottomMemoryDef = {
    .write = colorFlasherBottomWriteFlash,
    .read = colorFlasherBottomReadFlash,
    .properties = colorFlasherBottomPropertiesQuery,
    .supportsUpgrade = true,
    .id = "col",

    .commandResetToBootloader = resetColorBottomDeckToBootloader,
    .commandResetToFw = resetColorBottomDeckToFw,
};

static const DeckMemDef_t colorTopMemoryDef = {
    .write = colorFlasherTopWriteFlash,
    .read = colorFlasherTopReadFlash,
    .properties = colorFlasherTopPropertiesQuery,
    .supportsUpgrade = true,
    .id = "col",

    .commandResetToBootloader = resetColorTopDeckToBootloader,
    .commandResetToFw = resetColorTopDeckToFw,
};

// Bottom deck driver (original PID)
static const DeckDriver color_led_bottom_deck = {
  .vid = 0xBC,
  .pid = 0x13,
  .name = "bcColorLedBot",

  .memoryDef = &colorBottomMemoryDef,

  .init = colorLedBottomDeckInit,
  .test = colorLedBottomDeckTest,
};

// Top deck driver (new PID)
static const DeckDriver color_led_top_deck = {
  .vid = 0xBC,
  .pid = 0x14,
  .name = "bcColorLedTop",

  .memoryDef = &colorTopMemoryDef,

  .init = colorLedTopDeckInit,
  .test = colorLedTopDeckTest,
};

DECK_DRIVER(color_led_bottom_deck);
DECK_DRIVER(color_led_top_deck);

// Bottom deck parameters
PARAM_GROUP_START(colorLedBot)

/**
 * @brief Color value in WRGB format (0xWWRRGGBB) for bottom deck. Example: 0x000000FF = 255 = max blue
 */
PARAM_ADD(PARAM_UINT32, wrgb8888, &contexts[BOTTOM_IDX].wrgb8888)

/**
 * @brief Enable brightness correction (gamma and luminance normalization) for bottom deck. 0=off, 1=on
 */
PARAM_ADD(PARAM_UINT8, brightCorr, &contexts[BOTTOM_IDX].brightnessCorr)

PARAM_GROUP_STOP(colorLedBot)

// Top deck parameters
PARAM_GROUP_START(colorLedTop)

/**
 * @brief Color value in WRGB format (0xWWRRGGBB) for top deck. Example: 0x000000FF = 255 = max blue
 */
PARAM_ADD(PARAM_UINT32, wrgb8888, &contexts[TOP_IDX].wrgb8888)

/**
 * @brief Enable brightness correction (gamma and luminance normalization) for top deck. 0=off, 1=on
 */
PARAM_ADD(PARAM_UINT8, brightCorr, &contexts[TOP_IDX].brightnessCorr)

PARAM_GROUP_STOP(colorLedTop)

// Bottom deck logs
LOG_GROUP_START(colorLedBot)

/**
 * @brief Deck temperature in degrees Celsius for bottom deck
 */
LOG_ADD(LOG_UINT8, deckTemp, &contexts[BOTTOM_IDX].deckTemperature)

/**
 * @brief Thermal throttle percentage (0-100) for bottom deck
 */
LOG_ADD(LOG_UINT8, throttlePct, &contexts[BOTTOM_IDX].throttlePercentage)

/**
 * @brief Red LED current in milliamps for bottom deck
 */
LOG_ADD(LOG_UINT16, ledCurR, &contexts[BOTTOM_IDX].ledCurrent[0])

/**
 * @brief Green LED current in milliamps for bottom deck
 */
LOG_ADD(LOG_UINT16, ledCurG, &contexts[BOTTOM_IDX].ledCurrent[1])

/**
 * @brief Blue LED current in milliamps for bottom deck
 */
LOG_ADD(LOG_UINT16, ledCurB, &contexts[BOTTOM_IDX].ledCurrent[2])

/**
 * @brief White LED current in milliamps for bottom deck
 */
LOG_ADD(LOG_UINT16, ledCurW, &contexts[BOTTOM_IDX].ledCurrent[3])

LOG_GROUP_STOP(colorLedBot)

// Top deck logs
LOG_GROUP_START(colorLedTop)

/**
 * @brief Deck temperature in degrees Celsius for top deck
 */
LOG_ADD(LOG_UINT8, deckTemp, &contexts[TOP_IDX].deckTemperature)

/**
 * @brief Thermal throttle percentage (0-100) for top deck
 */
LOG_ADD(LOG_UINT8, throttlePct, &contexts[TOP_IDX].throttlePercentage)

/**
 * @brief Red LED current in milliamps for top deck
 */
LOG_ADD(LOG_UINT16, ledCurR, &contexts[TOP_IDX].ledCurrent[0])

/**
 * @brief Green LED current in milliamps for top deck
 */
LOG_ADD(LOG_UINT16, ledCurG, &contexts[TOP_IDX].ledCurrent[1])

/**
 * @brief Blue LED current in milliamps for top deck
 */
LOG_ADD(LOG_UINT16, ledCurB, &contexts[TOP_IDX].ledCurrent[2])

/**
 * @brief White LED current in milliamps for top deck
 */
LOG_ADD(LOG_UINT16, ledCurW, &contexts[TOP_IDX].ledCurrent[3])

LOG_GROUP_STOP(colorLedTop)

PARAM_GROUP_START(deck)

/**
 * @brief Nonzero if [Color LED deck bottom-mounted](%https://store.bitcraze.io/products/color-led-deck-copy?variant=62615266722141) is attached
 */
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcColorLedBot, &contexts[BOTTOM_IDX].isInit)

/**
 * @brief Nonzero if [Color LED deck top-mounted](%https://store.bitcraze.io/products/color-led-deck?variant=58838312616285) is attached
 */
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcColorLedTop, &contexts[TOP_IDX].isInit)

PARAM_GROUP_STOP(deck)

PARAM_GROUP_START(deckTest)

/**
 * @brief Self-test result bitmap for bottom Color LED deck
 *
 * 0 = all tests passed. Any non-zero value indicates failure:
 * Bit 0: Protocol version check failed
 * Bit 1: LED position read failed
 * Bit 2: I2C address pin test failed
 */
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcColorLedBot, &contexts[BOTTOM_IDX].testResults)

/**
 * @brief Self-test result bitmap for top Color LED deck
 *
 * 0 = all tests passed. Any non-zero value indicates failure:
 * Bit 0: Protocol version check failed
 * Bit 1: LED position read failed
 * Bit 2: I2C address pin test failed
 */
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcColorLedTop, &contexts[TOP_IDX].testResults)

PARAM_GROUP_STOP(deckTest)
