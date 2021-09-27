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
 * aideck.c - Deck driver for the AIdeck
 */
#define DEBUG_MODULE "AIDECK"

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
#include "system.h"
#include "uart1.h"
#include "uart2.h"
#include "static_mem.h"

#define NINALINK_MAGIC "\xbc\xa1"
#define NINALINK_MTU 128

static bool isInit = false;
static uint8_t byte;

typedef enum {
  NOP,
  PRINT,
  GAP8_FIRMWARE,
} ninalink_packet_type_t;

typedef enum
{
  waitForFirstStart,
  waitForSecondStart,
  waitForType,
  waitForLength,
  waitForData,
  waitForChksum1,
  waitForChksum2
} NinalinkRxState;

typedef struct {
  uint8_t type;
  uint8_t length;

  uint8_t data[NINALINK_MTU];
} __attribute__((packed)) NinalinkPacket;

static volatile NinalinkPacket nlp = { 0, };
static volatile NinalinkRxState ninalinkRxState = waitForFirstStart;
static volatile uint8_t dataIndex = 0;
static volatile uint8_t cksum[2] = { 0, };

static void dispatchNinalinkPacket()
{
  switch (nlp.type) {
    case NOP:
        DEBUG_PRINT("[NINA] NOP\n");
        break;

    case PRINT:
        DEBUG_PRINT("[NINA] %s", nlp.data);
        break;

    default:
        DEBUG_PRINT("Unknown packet received\n");
  }
}

void ninalinkReceive(uint8_t c)
{
    switch (ninalinkRxState) {
    case waitForFirstStart:
        ninalinkRxState = (c == NINALINK_MAGIC[0]) ? waitForSecondStart : waitForFirstStart;
        break;
    case waitForSecondStart:
        ninalinkRxState = (c == NINALINK_MAGIC[1]) ? waitForType : waitForFirstStart;
    break;
    case waitForType:
        cksum[0] = c;
        cksum[1] = c;
        nlp.type = c;
        ninalinkRxState = waitForLength;
        break;
    case waitForLength:
        if (c <= NINALINK_MTU) {
        nlp.length = c;
        cksum[0] += c;
        cksum[1] += cksum[0];
        dataIndex = 0;
        ninalinkRxState = (c > 0) ? waitForData : waitForChksum1;
        } else {
            ninalinkRxState = waitForFirstStart;
        }
        break;
    case waitForData:
        nlp.data[dataIndex] = c;
        cksum[0] += c;
        cksum[1] += cksum[0];
        dataIndex++;
        if (dataIndex == nlp.length) {
            ninalinkRxState = waitForChksum1;
        }
        break;
    case waitForChksum1:
        if (cksum[0] == c) {
            ninalinkRxState = waitForChksum2;
        } else {
            ninalinkRxState = waitForFirstStart; //Checksum error
        }
        break;
    case waitForChksum2:
        if (cksum[1] == c) {
            dispatchNinalinkPacket();
        } else {
            ASSERT(0);
        }
        ninalinkRxState = waitForFirstStart;
        break;
    default:
        ASSERT(0);
        break;
    }
  }

static void NinaTask(void *param)
{
    systemWaitStart();
    vTaskDelay(M2T(1000));
    DEBUG_PRINT("Starting reading out NINA debugging messages:\n");
    vTaskDelay(M2T(2000));

    // Pull the reset button to get a clean read out of the data
    pinMode(DECK_GPIO_IO4, OUTPUT);
    digitalWrite(DECK_GPIO_IO4, LOW);
    vTaskDelay(10);
    digitalWrite(DECK_GPIO_IO4, HIGH);
    pinMode(DECK_GPIO_IO4, INPUT_PULLUP);

    // Read out the byte the NINA sends and immediately send it to the console.
    uint8_t byte;
    while (1) {
        if (uart2GetDataWithDefaultTimeout(&byte) == true) {
            ninalinkReceive(byte);
        }
        vTaskDelay(M2T(10));
    }
}

static void Gap8Task(void *param)
{
    systemWaitStart();
    vTaskDelay(M2T(1000));

    // Pull the reset button to get a clean read out of the data
    pinMode(DECK_GPIO_IO4, OUTPUT);
    digitalWrite(DECK_GPIO_IO4, LOW);
    vTaskDelay(10);
    digitalWrite(DECK_GPIO_IO4, HIGH);
    pinMode(DECK_GPIO_IO4, INPUT_PULLUP);

    // Read out the byte the Gap8 sends and immediately send it to the console.
    while (1)
    {
        uart1GetDataWithDefaultTimeout(&byte);
    }
}

static void aideckInit(DeckInfo *info)
{

    if (isInit)
        return;

    // Intialize the UART for the GAP8
    uart1Init(115200);
    // Initialize task for the GAP8
    xTaskCreate(Gap8Task, AI_DECK_GAP_TASK_NAME, AI_DECK_TASK_STACKSIZE, NULL,
                AI_DECK_TASK_PRI, NULL);


    uart2Init(115200);
    xTaskCreate(NinaTask, AI_DECK_NINA_TASK_NAME, AI_DECK_TASK_STACKSIZE, NULL,
                AI_DECK_TASK_PRI, NULL);

    isInit = true;
}

static bool aideckTest()
{

    return true;
}

static const DeckDriver aideck_deck = {
    .vid = 0xBC,
    .pid = 0x12,
    .name = "bcAI",

    .usedGpio = DECK_USING_IO_4,
    .usedPeriph = DECK_USING_UART1,

    .init = aideckInit,
    .test = aideckTest,
};

LOG_GROUP_START(aideck)
LOG_ADD(LOG_UINT8, receivebyte, &byte)
LOG_GROUP_STOP(aideck)

/** @addtogroup deck
*/
PARAM_GROUP_START(deck)

/**
 * @brief Nonzero if [AI deck](%https://store.bitcraze.io/collections/decks/products/ai-deck-1-1) is attached
 */
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcAIDeck, &isInit)

PARAM_GROUP_STOP(deck)

DECK_DRIVER(aideck_deck);
