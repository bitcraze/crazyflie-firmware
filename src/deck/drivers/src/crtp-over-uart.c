/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
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
#define DEBUG_MODULE "CRTP-OVER-UART"

#include <stdint.h>

#include "deck.h"
#include "param.h"

#include "cpx_internal_router.h"
#include "cpx_external_router.h"
#include "cpx_uart_transport.h"
#include "cpx.h"

static bool isInit = false;

static void crtpOverUartInit(DeckInfo *info)
{
  if (isInit)
    return;

  cpxUARTTransportInit();
  cpxInternalRouterInit();
  cpxExternalRouterInit();
  cpxInit();

  isInit = true;
}

static bool crtpOverUartTest()
{
  return true;
}

static const DeckDriver crtpOverUART = {
    .vid = 0xBC,
    .pid = 0xFF,
    .name = "crtpOverUART",

    .usedPeriph = DECK_USING_UART2,

    .init = crtpOverUartInit,
    .test = crtpOverUartTest,
};

/** @addtogroup deck
*/
PARAM_GROUP_START(deck)

/**
 * @brief Nonzero if CRTP over UART has been forced
 */
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, crtpOverUART, &isInit)

PARAM_GROUP_STOP(deck)

DECK_DRIVER(crtpOverUART);