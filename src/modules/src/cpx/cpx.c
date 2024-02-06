/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2022 Bitcraze AB
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
 */

/* Handles incoming CPX messages */

#define DEBUG_MODULE "CPX"

#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "event_groups.h"
#include "debug.h"
#include "system.h"
#include "radiolink.h"

#include "cpxlink.h"
#include "cpx_internal_router.h"
#ifdef CONFIG_DECK_AI
#include "aideck.h"
#endif
#include "cpx.h"

static CPXPacket_t cpxRx;

static volatile cpxAppMessageHandlerCallback_t appMessageHandlerCallback;

#define WIFI_SET_SSID_CMD         0x10
#define WIFI_SET_KEY_CMD          0x11

#define WIFI_CONNECT_CMD          0x20
#define WIFI_CONNECT_AS_AP        0x01
#define WIFI_CONNECT_AS_STA       0x00
#define WIFI_CONNECT_AS_LENGTH    2

#define WIFI_AP_CONNECTED_CMD     0x31
#define WIFI_CLIENT_CONNECTED_CMD 0x32

#define CPX_ENABLE_CRTP_BRIDGE    0x21
#define CPX_SET_CLIENT_CONNECTED  0x20

void cpxInitRoute(const CPXTarget_t source, const CPXTarget_t destination, const CPXFunction_t function, CPXRouting_t* route) {
    route->source = source;
    route->destination = destination;
    route->function = function;
    route->version = CPX_VERSION;
}

bool cpxCheckVersion(const uint8_t version) {
  static bool hasLoggedVersionMismatch = false;

  // Version mismatch is generally handled by ignoring messages and logging the problem once.
  // Asserting has turned out to be a bad idea as it prevents flashing new firmware in some cases.

  const bool isVersionOk = (version == CPX_VERSION);

  if (!isVersionOk) {
    if (!hasLoggedVersionMismatch) {
      DEBUG_PRINT("WARNING! CPX version mismatch. Got %i, require %i\n", version, CPX_VERSION);
      hasLoggedVersionMismatch = true;
    }
  }

  return isVersionOk;
}

void cpxRegisterAppMessageHandler(cpxAppMessageHandlerCallback_t callback) {
  appMessageHandlerCallback = callback;
}

static void cpx(void* _param) {
  systemWaitStart();
  while (1) {
    cpxInternalRouterReceiveOthers(&cpxRx);

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
          if (cpxRx.data[1] == 0x00) {
            cpxLinkSetConnected(false);
            DEBUG_PRINT("CPX disconnected\n");
          } else {
            cpxLinkSetConnected(true);
            DEBUG_PRINT("CPX connected\n");
          }
        }
        break;
      case CPX_F_CONSOLE:
        if (cpxRx.route.source == CPX_T_ESP32) {
          DEBUG_PRINT("ESP32: %s", cpxRx.data);
        } else if (cpxRx.route.source == CPX_T_GAP8) {
          DEBUG_PRINT("GAP8: %s", cpxRx.data);
        } else {
          DEBUG_PRINT("UNKNOWN: %s", cpxRx.data);
        }
        break;
      case CPX_F_BOOTLOADER:
#ifdef CONFIG_DECK_AI
        cpxBootloaderMessage(&cpxRx);
#endif
        break;
      case CPX_F_SYSTEM:
        if (cpxRx.data[0] == CPX_ENABLE_CRTP_BRIDGE) {
          if (cpxRx.data[1] == 0x00) {
            crtpSetLink(radiolinkGetLink());
            DEBUG_PRINT("Disable CPX <> CRTP bridge\n");
          } else {
            crtpSetLink(cpxlinkGetLink());
            DEBUG_PRINT("Enable CPX <> CRTP bridge\n");
          }
        }

        if (cpxRx.data[0] == CPX_SET_CLIENT_CONNECTED) {
          if (cpxRx.data[1] == 0x00) {
            cpxLinkSetConnected(false);
          } else {
            cpxLinkSetConnected(true);
          }
        }
        break;
      case CPX_F_APP:
        if (appMessageHandlerCallback) {
          appMessageHandlerCallback(&cpxRx);
        }
        break;
      default:
        DEBUG_PRINT("Not handling function [0x%02X] from [0x%02X]\n", cpxRx.route.function, cpxRx.route.source);
    }
  }
}

void cpxInit() {
  xTaskCreate(cpx, CPX_TASK_NAME, AI_DECK_TASK_STACKSIZE, NULL, AI_DECK_TASK_PRI, NULL);
}
