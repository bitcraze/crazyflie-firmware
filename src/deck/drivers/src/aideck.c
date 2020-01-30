/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
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
#include "system.h"
#include "uart1.h"
#include "uart2.h"

//#define DEBUG_PRINT_COM DEBUG_PRINT
#define DEBUG_PRINT_COM(...)

static bool isInit;
/*static void GAP8Task(void *param)
{
    systemWaitStart();
    DEBUG_PRINT("STarting AI TAKS\n");

    uint8_t byte;
    while (1)
    {
        if (uart1GetDataWithTimout(&byte) == true)
        {
            DEBUG_PRINT_COM("[NINA] Received: 0x%02X\r\n", byte);
        }
    }
}*/

static void NinaTask(void *param)
{
    systemWaitStart();
    DEBUG_PRINT("STarting AI TAKS\n");

    uint8_t byte;
    while (1)
    {
        vTaskDelay(10);
        DEBUG_PRINT("hello\n");

        if (uart2GetDataWithTimout(&byte) == true)
        {
            DEBUG_PRINT("[NINA] Received: 0x%02X\r\n", byte);
        }
    }
}
static void aideckInit(DeckInfo *info)
{
    if (isInit)
        return;

    DEBUG_PRINT("Initialize AI-deck test\n");

    // FOr the GAP8
    uart1Init(115200);
    // For the NINA
    uart2Init(115200);

        //Reset GAP8 and NINA to start with
    pinMode(DECK_GPIO_IO4, OUTPUT);
    digitalWrite(DECK_GPIO_IO4, LOW);
    vTaskDelay(10);
    digitalWrite(DECK_GPIO_IO4, HIGH);
    pinMode(DECK_GPIO_IO4, INPUT_PULLUP);

   /* xTaskCreate(GAP8Task, AI_DECK_GAP_TASK_NAME, AI_DECK_TASK_STACKSIZE, NULL,
                AI_DECK_TASK_PRI, NULL);*/
    xTaskCreate(NinaTask, AI_DECK_NINA_TASK_NAME, AI_DECK_TASK_STACKSIZE, NULL,
                AI_DECK_TASK_PRI, NULL);

    isInit = true;
}



static bool aideckTest()
{



    return true;
}

static const DeckDriver aideck_deck = {
    .name = "bcAIDeck",

    .usedPeriph = 0,
    .usedGpio = 0, // FIXME: Edit the used GPIOs

    .init = aideckInit,
    .test = aideckTest,
};

DECK_DRIVER(aideck_deck);
