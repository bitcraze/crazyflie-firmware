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

/* Fake deck driver for using CPX (and CRTP) over UART2 on the expansion connector.
 * Note that this has to be forced on in the deck subsystem, since there's no 1-wire
 * memory available.
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

static void cpxOverUart2Init(DeckInfo *info)
{
  if (isInit)
    return;

  cpxUARTTransportInit();
  cpxInternalRouterInit();
  cpxExternalRouterInit();
  cpxInit();

  isInit = true;
}

static bool cpxOverUart2Test()
{
  return true;
}

static const DeckDriver crtpOver2UART = {
    .name = "cpxOverUART2",

    .usedPeriph = DECK_USING_UART2,

    .init = cpxOverUart2Init,
    .test = cpxOverUart2Test,
};

/** @addtogroup deck
*/
PARAM_GROUP_START(deck)

/**
 * @brief Nonzero if CRTP over UART has been forced
 */
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, cpxOverUART2, &isInit)

PARAM_GROUP_STOP(deck)

DECK_DRIVER(crtpOver2UART);