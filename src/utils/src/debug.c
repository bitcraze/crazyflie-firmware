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
 * debug.c - Debugging utility functions
 */
#include "debug.h"

#ifdef CONFIG_DEBUG_PRINT_ON_SWO
#include "stm32fxxx.h"

// Unlock key for the CoreSight lock access registers
#define CORESIGHT_UNLOCK 0xC5ACCE55

// Set up SWO on PB3 as NRZ (UART) output of ITM stimulus port 0. PB3 comes out
// of reset as AF0 (JTDO/TRACESWO) and nothing else in the firmware uses it.
static void swoInit(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  // Asynchronous trace mode (TRACE_MODE = 00) with the trace pin enabled
  DBGMCU->CR = (DBGMCU->CR & ~DBGMCU_CR_TRACE_MODE) | DBGMCU_CR_TRACE_IOEN;

  TPI->SPPR = 2;  // NRZ
  // TRACECLKIN is HCLK; round to the nearest divisor
  TPI->ACPR = (SystemCoreClock + CONFIG_DEBUG_PRINT_ON_SWO_BAUDRATE / 2) / CONFIG_DEBUG_PRINT_ON_SWO_BAUDRATE - 1;
  TPI->FFCR = TPI_FFCR_TrigIn_Msk;  // Formatter off: plain ITM packets

  ITM->LAR = CORESIGHT_UNLOCK;
  ITM->TCR = (1 << ITM_TCR_TraceBusID_Pos) | ITM_TCR_ITMENA_Msk;
  ITM->TPR = 0;
  ITM->TER = 1;
}

int swoPutchar(int ch)
{
  ITM_SendChar(ch);
  return (unsigned char)ch;
}
#endif


void debugInit(void)
{
#ifdef CONFIG_DEBUG_PRINT_ON_SWO
  swoInit();
#endif
#ifdef DEBUG_PRINT_ON_SEGGER_RTT
  SEGGER_RTT_Init();
  SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_TRIM);
#endif
}
