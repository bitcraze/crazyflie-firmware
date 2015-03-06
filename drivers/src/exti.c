/**
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
 * exti.c - Unified implementation of the exti interrupts
 */
#include <stdbool.h>

#include "stm32fxxx.h"

#include "nvicconf.h"
#include "nrf24l01.h"

#ifdef PLATFORM_CF1
  #define RADIO_GPIO_IRQ_LINE   EXTI_Line9
  #define RADIO_IRQ_CHANNEL     EXTI9_5_IRQn
#else
  #define RADIO_GPIO_IRQ_LINE   EXTI_Line10
  #define RADIO_IRQ_CHANNEL     EXTI15_10_IRQn
#endif

static bool isInit;

/* Interruption initialisation */
void extiInit()
{
  if (isInit)
    return;

  NVIC_InitTypeDef NVIC_InitStructure;

  NVIC_InitStructure.NVIC_IRQChannel = RADIO_IRQ_CHANNEL;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_RADIO_PRI;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  isInit = true;
}

bool extiTest(void)
{
  return isInit;
}

#ifdef PLATFORM_CF1
void __attribute__((used)) EXTI9_5_IRQHandler(void)
{
  if (EXTI_GetITStatus(RADIO_GPIO_IRQ_LINE) == SET)
  {
    EXTI_ClearITPendingBit(RADIO_GPIO_IRQ_LINE);
    nrfIsr();
  }
}

#else
void __attribute__((used)) EXTI15_10_IRQHandler(void)
{
  if (EXTI_GetITStatus(RADIO_GPIO_IRQ_LINE) == SET)
  {
    EXTI_ClearITPendingBit(RADIO_GPIO_IRQ_LINE);
  }
}
#endif
