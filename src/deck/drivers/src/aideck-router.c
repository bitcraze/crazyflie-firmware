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
#define DEBUG_MODULE "AIDECK-ROUTER"

#include <stdint.h>

#include "FreeRTOS.h"
#include "config.h"
#include "debug.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "queue.h"
#include "stm32fxxx.h"
#include "system.h"

#include "aideck.h"
#include "aideck-router.h"

#define WIFI_SET_SSID_CMD         0x10
#define WIFI_SET_KEY_CMD          0x11

#define WIFI_CONNECT_CMD          0x20
#define WIFI_CONNECT_AS_AP        0x01
#define WIFI_CONNECT_AS_STA       0x00
#define WIFI_CONNECT_AS_LENGTH    2

#define WIFI_AP_CONNECTED_CMD     0x31
#define WIFI_CLIENT_CONNECTED_CMD 0x32

static CPXPacket_t cpxRx;

static void cxpRxTest(void *param)
{
  systemWaitStart();
  while (1) {
    cpxReceivePacketBlocking(&cpxRx);

    //DEBUG_PRINT("CPX RX: Message from [0x%02X] to function [0x%02X] (size=%u)\n", cpxRx.route.source, cpxRx.route.function, cpxRx.dataLength);

    switch (cpxRx.route.function) {
      case CPX_F_WIFI_CTRL:
        if (cpxRx.data[0] == WIFI_AP_CONNECTED_CMD) {
            DEBUG_PRINT("WiFi connected to ip: %u.%u.%u.%u\n",
                        cpxRx.data[1],
                        cpxRx.data[2],
                        cpxRx.data[3],
                        cpxRx.data[4]);
        }
        if (cpxRx.data[0] == WIFI_CLIENT_CONNECTED_CMD) {
          DEBUG_PRINT("WiFi client connected\n");
        }
        break;
      case CPX_F_CONSOLE:
        if (cpxRx.route.source == CPX_T_ESP32) {
          consolePrintf("ESP32: %s", cpxRx.data);
        } else if (cpxRx.route.source == CPX_T_GAP8) {
          consolePrintf("GAP8: %s", cpxRx.data);
        } else {
          consolePrintf("UNKNOWN: %s", cpxRx.data);
        }
        break;
      case CPX_F_BOOTLOADER:
        cpxBootloaderMessage(&cpxRx);
        break;
      default:
        DEBUG_PRINT("Not handling function [0x%02X] from [0x%02X]\n", cpxRx.route.function, cpxRx.route.source);
    }
  }
}

#ifndef CONFIG_DECK_AI_WIFI_NO_SETUP
static CPXPacket_t cpxTx;
static void setupWiFi() {
#ifdef CONFIG_DECK_AI_WIFI_SETUP_STA
  DEBUG_PRINT("AI-deck will connect to WiFi\n");
#endif

#ifdef CONFIG_DECK_AI_WIFI_SETUP_AP
  DEBUG_PRINT("AI-deck will become access point\n");
#endif

  cpxInitRoute(CPX_T_STM32, CPX_T_ESP32, CPX_F_WIFI_CTRL, &cpxTx.route);

  cpxTx.data[0] = WIFI_SET_SSID_CMD; // Set SSID
  memcpy(&cpxTx.data[1], CONFIG_DECK_AI_SSID, sizeof(CONFIG_DECK_AI_SSID));
  cpxTx.dataLength = sizeof(CONFIG_DECK_AI_SSID);
  cpxSendPacketBlocking(&cpxTx);

  cpxTx.data[0] = WIFI_SET_KEY_CMD; // Set SSID
  memcpy(&cpxTx.data[1], CONFIG_DECK_AI_PASSWORD, sizeof(CONFIG_DECK_AI_PASSWORD));
  cpxTx.dataLength = sizeof(CONFIG_DECK_AI_PASSWORD);
  cpxSendPacketBlocking(&cpxTx);

  cpxTx.data[0] = WIFI_CONNECT_CMD; // Connect wifi
#ifdef CONFIG_DECK_AI_WIFI_SETUP_STA
  cpxTx.data[1] = WIFI_CONNECT_AS_STA;
#endif
#ifdef CONFIG_DECK_AI_WIFI_SETUP_AP
  cpxTx.data[1] = WIFI_CONNECT_AS_AP;
#endif
  cpxTx.dataLength = WIFI_CONNECT_AS_LENGTH;
  cpxSendPacketBlocking(&cpxTx);
}
#endif

void aideckRouterInit(void) {
  xTaskCreate(cxpRxTest, "CPX Router RX", configMINIMAL_STACK_SIZE, NULL,
              AI_DECK_TASK_PRI, NULL);

#ifdef CONFIG_DECK_AI_WIFI_NO_SETUP
  DEBUG_PRINT("Not setting up WiFi\n");
#else
  setupWiFi(); 
#endif

}