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
 * debug.h - Debugging utility functions
 */
#include "config.h"
#include "console.h"

#ifdef DEBUG_MODULE
#define DEBUG_FMT(fmt) DEBUG_MODULE ": " fmt
#endif

#ifndef DEBUG_FMT
#define DEBUG_FMT(fmt) fmt
#endif

#if defined(DEBUG_PRINT_ON_UART)
  #ifndef ENABLE_UART
    #error "Need to define ENABLE_UART to use DEBUG_PRINT_ON_UART"
  #endif
  #define DEBUG_PRINT(fmt, ...) uartPrintf(DEBUG_FMT(fmt), ##__VA_ARGS__)
  #define DEBUG_PRINT_OS(fmt, ...) uartPrintf(DEBUG_FMT(fmt), ##__VA_ARGS__)
#elif defined(DEBUG_PRINT_ON_SWO)
  #define DEBUG_PRINT(fmt, ...) eprintf(ITM_SendChar, fmt, ## __VA_ARGS__)
  #define DEBUG_PRINT_OS(fmt, ...) eprintf(ITM_SendChar, fmt, ## __VA_ARGS__)
#else
  #define DEBUG_PRINT(fmt, ...) consolePrintf(DEBUG_FMT(fmt), ##__VA_ARGS__)
#define DEBUG_PRINT_OS(fmt, ...) consolePrintf(DEBUG_FMT(fmt), ##__VA_ARGS__)
  //#define DEBUG_PRINT(fmt, ...)
#endif

#ifndef PRINT_OS_DEBUG_INFO
  #undef DEBUG_PRINT_OS
  #define DEBUG_PRINT_OS(fmt, ...)
#endif


#ifdef TEST_PRINTS
  #define TEST_AND_PRINT(e, msgOK, msgFail)\
    if(e) { DEBUG_PRINT(msgOK); } else { DEBUG_PRINT(msgFail); }
  #define FAIL_PRINT(msg) DEBUG_PRINT(msg)
#else
  #define TEST_AND_PRINT(e, msgOK, msgFail)
  #define FAIL_PRINT(msg)
#endif

