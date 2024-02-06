/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2021 Bitcraze AB
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
 * aidecktest.c - Testing of AI deck in production
 */
#define DEBUG_MODULE "AIDECK-TEST"

#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "stm32fxxx.h"
#include "config.h"
#include "console.h"
#include "uart1.h"
#include "debug.h"
#include "deck.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "log.h"
#include "param.h"

#include "uart1.h"
#include "uart2.h"

//#define DEBUG_PRINT_COM DEBUG_PRINT
#define DEBUG_PRINT_COM(...)

#define START_UP_BYTE 0xBC

static bool isInit;

static void aitdecktestInit(DeckInfo *info)
{
  if (isInit)
    return;

  DEBUG_PRINT("Initialize AI-deck test\n");

  // FOr the GAP8
  uart1Init(115200);
  // For the NINA
  uart2Init(115200);

  isInit = true;
}


static bool testOnNina(const uint8_t command, const uint8_t expected)
{
  uint8_t byte;

  uart2Putchar(command);

  if (uart2GetCharWithDefaultTimeout(&byte) == true)
  {
    DEBUG_PRINT_COM("[NINA] Received: 0x%02X\r\n", byte);
    if (byte == expected)
    {
      return true;
    }
  }
  DEBUG_PRINT_COM("[NINA] Received timeout!\r\n");
  return false;
}

static bool testOnNinaMask(const uint8_t command, uint8_t *byte)
{

  uart2Putchar(command);

  if (uart2GetCharWithDefaultTimeout(byte) == true)
  {
    DEBUG_PRINT_COM("Received mask: 0x%02X\r\n", *byte);

    return true;
  }

  return false;
}

static bool testOnGAP8(const uint8_t command, const uint8_t expected)
{
  uint8_t byte;
  uint8_t timeout_counter = 0;

  uart1Putchar(command);

  while(timeout_counter < 5){
    if (uart1GetDataWithDefaultTimeout(&byte) == true)
    {
      DEBUG_PRINT_COM("[GAP8] Received: 0x%02X\r\n", byte);

      if (byte == expected)
      {
        return true;
      }
    }
    timeout_counter++;
  }

  return false;
}

/* At startup NINA and GAP8 should send 0xBC */

/* NINA command set
 * 0x01 - Button has been high (1=true, 0=false)
 * 0x02 - Button has been low (1=true, 0=false)
 * 0x03 - Reset button low status (1=pass,0=fail) (will probably just return pass, it's just for setting)
 * 0x04 - Read GPIO mask (read out bitmask bits 0-5 as XX54 3210)
 * 0x05 - Reset GAP8 though NINA (1=pass,0=fail) (will probably just return pass, it's just for setting)
 */

/* GAP8 command set
 * 0x01 - Test Hyper flash (1=pass, 0=fail)
 * 0x02 - Test camera (1=pass, 0=fail)
 * 0x03 - Test I2C by reading EEPROM
 * 0x40 - 5 bits for GPIO output [XA54 3210] where A=1 and 0-5 is the GPIOs in the list below (1=pass, 0=fail) (will probably just return pass, it's just for setting)
 */

/* GPIO list for 0x40
 * 0 - GAP8_SPI_MISO
 * 1 - GAP8_GPIO_NINA_IO
 * 2 - GAP8_SPI_MOSI
 * 3 - NINA_GPIO_GAP8_IO
 * 4 - GAP8_SPI_CS0
 * 5 - GAP8_SPI_CLK
 */

/* GPIO list for 0xC0
 * 0 - I2C SDA
 * 1 - I2C SCL
 */

/* Bit   - Function (1=fail, 0=pass)
 *
 * 0     - Nina button high
 * 1     - Nina button low (by press)
 * 2     - Nina button low (by IO)
 * 3-8   - GPIO NINA <-> GAP8
 * 9-14  - GPIO NINA <-> GAP8 inverted
 * 15    - Hyper flash
 * 16    - Camera
 * 17    - I2C
 * 18    - GAP8 reset though NINA
 * 19    - GAP8 reset tough RST
 * 20    - NINA reset tough RST
 */

uint32_t testmask = 0x01FFFFFUL;    // Not true, set all bits in mask to 1
uint8_t testdone;                   // Set to 1 when testing is completed

#define NINA_INIT_CHAR 0xBC
#define GAP8_INIT_CHAR 0xBC

#define NINA_BOOT_HIGH                0x01
#define NINA_BOOT_HIGH_EXPECTED       0x01
#define NINA_BOOT_HIGH_POS            0x00

#define NINA_BOOT_LOW                 0x02
#define NINA_BOOT_LOW_EXPECTED        0x01
#define NINA_BOOT_LOW_POS             0x01UL

#define NINA_BOOT_LOW_RESET           0x03
#define NINA_BOOT_LOW_RESET_EXPECTED  0x01
#define NINA_BOOT_LOW_IO_POS          0x02UL

#define GAP8_GPIO_COMMAND             0x40
#define GAP8_GPIO_MASK                0x15
#define GAP8_GPIO_MASK_EXPECTED       0x01UL

#define NINA_GAP8_GPIO_COMMAND        0x04
#define NINA_GAP8_GPIO_POS            0x03
#define NINA_GAP8_GPIO_INV_POS        0x09UL

#define GAP8_HYPER_COMMAND            0x01
#define GAP8_HYPER_EXPECTED           0x01
#define GAP8_HYPER_POS                0x0FUL

#define GAP8_CAMERA_COMMAND           0x02
#define GAP8_CAMERA_EXPECTED          0x01
#define GAP8_CAMERA_POS               0x10UL

#define GAP8_I2C_COMMAND              0x03
#define GAP8_I2C_EXPECTED             0x01
#define GAP8_I2C_POS                  0x11UL

#define NINA_GAP8_RST_COMMAND 0x05
#define NINA_GAP8_RST_EXPECTED 0x01
#define NINA_GAP8_RST_POS 0x12

#define CF2_GAP8_RST_POS 0x13
#define CF2_NINA_RST_POS 0x14

static bool aitdecktestTest()
{
  bool testHasBeenTriggered = false;
  uint8_t byte;
  uint8_t gpio_mask;

  DEBUG_PRINT("Running AI-deck test, waiting for button press\r\n");

  //Reset GAP8 and NINA to start with
  pinMode(DECK_GPIO_IO4, OUTPUT);
  digitalWrite(DECK_GPIO_IO4, LOW);
  vTaskDelay(10);
  digitalWrite(DECK_GPIO_IO4, HIGH);
  pinMode(DECK_GPIO_IO4, INPUT_PULLUP);

  // Wait for the NINA to start
  vTaskDelay(M2T(1000));
  // Empty the buffer from NINA
  while (uart2GetCharWithDefaultTimeout(&byte) == true)
    ;

  while (uart1GetDataWithDefaultTimeout(&byte) == true)
    ;

  while (!testHasBeenTriggered)
  {
    // Send test for button low to NINA
    if (testOnNina(NINA_BOOT_LOW, NINA_BOOT_LOW_EXPECTED) == true)
    {
      testmask &= ~(1UL << NINA_BOOT_LOW_POS);
      testHasBeenTriggered = true;
    }

    vTaskDelay(M2T(100));
  }

  // Send test for button high NINA
  if (testOnNina(NINA_BOOT_HIGH, NINA_BOOT_HIGH_EXPECTED) == true)
  {
    testmask &= ~(1UL << NINA_BOOT_HIGH_POS);
    DEBUG_PRINT("NINA button test [OK]\r\n");
  }
  else
  {
    DEBUG_PRINT("NINA button test [FAILED]\r\n");
  }



  if (testOnNina(NINA_BOOT_LOW_RESET, NINA_BOOT_LOW_RESET_EXPECTED) == true)
  {
    // Pull boot and read out button low on NINA

    pinMode(DECK_GPIO_IO1, OUTPUT);
    digitalWrite(DECK_GPIO_IO1, LOW);
    vTaskDelay(150);
    digitalWrite(DECK_GPIO_IO1, HIGH);

    // Send reset button status to NINA (not done on NINA yet)
    if (testOnNina(NINA_BOOT_LOW, NINA_BOOT_LOW_EXPECTED) == true)
    {
      testmask &= ~(1UL << NINA_BOOT_LOW_IO_POS);
      DEBUG_PRINT("NINA boot pin test [OK]\r\n");
    }
    else
    {
      DEBUG_PRINT("NINA boot pin test [FAILED]\r\n");
    }
  }



  // Send test for GPIO mask to GAP8 (should be optimized to it's inveterd to neighbours)

  //     In GAP8, the command (GAP8_GPIO_MASK 0x04) should be removed from the mask so that it is like
  //     GAP8_GPIO_MASK again

  if (testOnGAP8(GAP8_GPIO_COMMAND|GAP8_GPIO_MASK, GAP8_GPIO_MASK_EXPECTED) == true)
  {

    vTaskDelay(M2T(100));
    // Send test for GPIO mask to NINA
    if (testOnNinaMask(NINA_GAP8_GPIO_COMMAND, &gpio_mask) == true)
    {

      uint32_t gpio_mask_result = (uint32_t)0X3F << NINA_GAP8_GPIO_POS;
      gpio_mask_result ^= 0x01FFFFF;
      gpio_mask_result |= (uint32_t)(gpio_mask ^ GAP8_GPIO_MASK) << NINA_GAP8_GPIO_POS;
      testmask &= gpio_mask_result;
      DEBUG_PRINT("GAP8->NINA gpio test [OK]\r\n");
    }
    else
    {
      DEBUG_PRINT("GAP8->NINA gpio test [FAILED]\r\n");
    }
  }
  else
  {
    DEBUG_PRINT("Set GAP8 gpio mask [FAILED]\r\n");
  }
  vTaskDelay(M2T(100));


  uint8_t not_mask = (~GAP8_GPIO_MASK) & 0X3F;

  // Send test for ~ GPIO mask to GAP8
  if (testOnGAP8(GAP8_GPIO_COMMAND | not_mask, GAP8_GPIO_MASK_EXPECTED) == true)
  {
    vTaskDelay(M2T(100));
    // Send test for ~ GPIO mask to NINA
    if (testOnNinaMask(NINA_GAP8_GPIO_COMMAND, &gpio_mask) == true)
    {
      uint32_t gpio_mask_result = (uint32_t)0X3F << NINA_GAP8_GPIO_INV_POS;
      gpio_mask_result ^= 0x01FFFFF;
      gpio_mask_result |= (uint32_t)(gpio_mask ^ ((~GAP8_GPIO_MASK) & 0X3F)) << NINA_GAP8_GPIO_INV_POS;
      testmask &= gpio_mask_result;
      DEBUG_PRINT("GAP8->NINA gpio not-test [OK]\r\n");
    }
    else
    {
      DEBUG_PRINT("GAP8->NINA gpio not-test [FAILED]\r\n");
    }
  }
  else
  {
    DEBUG_PRINT("Set GAP8 gpio not-mask [FAILED]\r\n");
  }




  // Send test for Hyper flash to GAP8
  if (testOnGAP8(GAP8_HYPER_COMMAND, GAP8_HYPER_EXPECTED) == true)
  {
    testmask &= ~(1UL << GAP8_HYPER_POS);
    DEBUG_PRINT("GAP8 Hyper test [OK]\r\n");
  }
  else
  {
    DEBUG_PRINT("GAP8 Hyper test [FAILED]\r\n");
  }

  // Send test for Camera to GAP8
  if (testOnGAP8(GAP8_CAMERA_COMMAND, GAP8_CAMERA_EXPECTED) == true)
  {
    testmask &= ~(1UL << GAP8_CAMERA_POS);
    DEBUG_PRINT("GAP8 Camera test [OK]\r\n");
  }
  else
  {
    DEBUG_PRINT("GAP8 Camera test [FAILED]\r\n");
  }

  // Test I2C by GAP8 by reading the EEPROM for the address and magic number
  //       MAGIC               0x43427830
  //       EEPROM_I2C_ADDR     0x50
  // NOTE: should be not be run at startup!

  if (testOnGAP8(GAP8_I2C_COMMAND, GAP8_I2C_EXPECTED) == true)
  {
    testmask &= ~(1UL << GAP8_I2C_POS);
    DEBUG_PRINT("GAP8 I2C test [OK]\r\n");
  }
  else
  {
    DEBUG_PRINT("GAP8 I2C test [FAILED]\r\n");
  }

  // Test RST of GAP8 though NINA
  // (listen on GAP8 uart for hello)
  if (testOnNina(NINA_GAP8_RST_COMMAND, NINA_GAP8_RST_EXPECTED) == true)
  {
    while (uart1GetDataWithDefaultTimeout(&byte) == true)
    {
      if (byte == GAP8_INIT_CHAR)
      {
        testmask &= ~(1UL << NINA_GAP8_RST_POS);
        DEBUG_PRINT("NINA->GAP8 reset [OK]\r\n");
        break;
      }
    }
    if (byte != GAP8_INIT_CHAR)
    {
      DEBUG_PRINT("NINA->GAP8 reset [FAILED]\r\n");
    }
  }
  else
  {
    DEBUG_PRINT("NINA->GAP8 reset, NINA not responding. [FAILED]\r\n");
  }

  //Test RST of both GAP8 and NINA by pulling reset
  pinMode(DECK_GPIO_IO4, OUTPUT);
  digitalWrite(DECK_GPIO_IO4, LOW);
  vTaskDelay(10);
  digitalWrite(DECK_GPIO_IO4, HIGH);
  pinMode(DECK_GPIO_IO4, INPUT);

  // (listen on GAP8 and NINA uart for 0xbc)
  while (uart2GetCharWithDefaultTimeout(&byte) == true)
  {
    if (byte == NINA_INIT_CHAR)
    {
      testmask &= ~(1UL << CF2_NINA_RST_POS);
      DEBUG_PRINT("NINA reset [OK]\r\n");
      break;
    }
  }
  if (byte != NINA_INIT_CHAR)
  {
    DEBUG_PRINT("NINA reset [FAILED]\r\n");
  }

  while (uart1GetDataWithDefaultTimeout(&byte) == true)
  {
    if (byte == GAP8_INIT_CHAR)
    {
      DEBUG_PRINT("GAP8 reset [OK]\r\n");
      testmask &= ~(1UL << CF2_GAP8_RST_POS);
      break;
    }
  }
  if (byte != GAP8_INIT_CHAR)
  {
    DEBUG_PRINT("GAP8 reset [FAILED]\r\n");
  }

  // Set all tests done
  DEBUG_PRINT("AI deck test mask: 0x%08X\r\n", testmask);


  testdone = 1;

  return (testmask == 0);
}

static const DeckDriver aitest_deck = {
    .name = "bcAIDeckTest",

    .usedPeriph = DECK_USING_UART1,
    .usedGpio = DECK_USING_IO_4,

    .init = aitdecktestInit,
    .test = aitdecktestTest,
};

DECK_DRIVER(aitest_deck);

LOG_GROUP_START(aidecktest)
LOG_ADD(LOG_UINT32, testresult, &testmask)
LOG_ADD(LOG_UINT8, done, &testdone)
LOG_GROUP_STOP(aidecktest)
