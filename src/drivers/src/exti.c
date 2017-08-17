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

#include "exti.h"
#include "nvicconf.h"
#include "nrf24l01.h"

static bool isInit;

/* Interruption initialisation */
void extiInit()
{
  static NVIC_InitTypeDef NVIC_InitStructure;

  if (isInit)
    return;

  // This is required for the EXTI interrupt configuration since EXTI
  // lines are set via the SYSCFG peripheral; eg.
  // SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource13);
  RCC_AHB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE); 

  // Here we enable all EXTI interrupt handlers to save conflicting
  // reinitialization code for the 9_5 and 15_10 handlers. Note that
  // the individual EXTI interrupts still need to be enabled.

  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = EXTI0_PRI;
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = EXTI1_PRI;
  NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = EXTI2_PRI;
  NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = EXTI3_PRI;
  NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = EXTI4_PRI;
  NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = EXTI9_5_PRI;
  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = EXTI15_10_PRI;
  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
  NVIC_Init(&NVIC_InitStructure);

  isInit = true;
}

bool extiTest(void)
{
  return isInit;
}

void __attribute__((used)) EXTI0_IRQHandler(void)
{
  EXTI_ClearITPendingBit(EXTI_Line0);
  EXTI0_Callback();
}

void __attribute__((used)) EXTI1_IRQHandler(void)
{
  EXTI_ClearITPendingBit(EXTI_Line1);
  EXTI1_Callback();
}

void __attribute__((used)) EXTI2_IRQHandler(void)
{
  EXTI_ClearITPendingBit(EXTI_Line2);
  EXTI2_Callback();
}

void __attribute__((used)) EXTI3_IRQHandler(void)
{
  EXTI_ClearITPendingBit(EXTI_Line3);
  EXTI3_Callback();
}

void __attribute__((used)) EXTI4_IRQHandler(void)
{
  EXTI_ClearITPendingBit(EXTI_Line4);
  EXTI4_Callback();
}

void __attribute__((used)) EXTI9_5_IRQHandler(void)
{
  if (EXTI_GetITStatus(EXTI_Line5) == SET) {
    EXTI_ClearITPendingBit(EXTI_Line5);
    EXTI5_Callback();
  }

  if (EXTI_GetITStatus(EXTI_Line6) == SET) {
    EXTI_ClearITPendingBit(EXTI_Line6);
    EXTI6_Callback();
  }

  if (EXTI_GetITStatus(EXTI_Line7) == SET) {
    EXTI_ClearITPendingBit(EXTI_Line7);
    EXTI7_Callback();
  }

  if (EXTI_GetITStatus(EXTI_Line8) == SET) {
    EXTI_ClearITPendingBit(EXTI_Line8);
    EXTI8_Callback();
  }

  if (EXTI_GetITStatus(EXTI_Line9) == SET) {
    EXTI_ClearITPendingBit(EXTI_Line9);
    EXTI9_Callback();
  }
}

void __attribute__((used)) EXTI15_10_IRQHandler(void)
{
  if (EXTI_GetITStatus(EXTI_Line10) == SET) {
    EXTI_ClearITPendingBit(EXTI_Line10);
    EXTI10_Callback();
  }

  if (EXTI_GetITStatus(EXTI_Line11) == SET) {
    EXTI_ClearITPendingBit(EXTI_Line11);
    EXTI11_Callback();
  }

  if (EXTI_GetITStatus(EXTI_Line12) == SET) {
    EXTI_ClearITPendingBit(EXTI_Line12);
    EXTI12_Callback();
  }

  if (EXTI_GetITStatus(EXTI_Line13) == SET) {
    EXTI_ClearITPendingBit(EXTI_Line13);
    EXTI13_Callback();
  }

  if (EXTI_GetITStatus(EXTI_Line14) == SET) {
    EXTI_ClearITPendingBit(EXTI_Line14);
    EXTI14_Callback();
  }

  if (EXTI_GetITStatus(EXTI_Line15) == SET) {
    EXTI_ClearITPendingBit(EXTI_Line15);
    EXTI15_Callback();
  }
}

void __attribute__((weak)) EXTI0_Callback(void) { }
void __attribute__((weak)) EXTI1_Callback(void) { }
void __attribute__((weak)) EXTI2_Callback(void) { }
void __attribute__((weak)) EXTI3_Callback(void) { }
void __attribute__((weak)) EXTI4_Callback(void) { }
void __attribute__((weak)) EXTI5_Callback(void) { }
void __attribute__((weak)) EXTI6_Callback(void) { }
void __attribute__((weak)) EXTI7_Callback(void) { }
void __attribute__((weak)) EXTI8_Callback(void) { }
void __attribute__((weak)) EXTI9_Callback(void) { }
void __attribute__((weak)) EXTI10_Callback(void) { }
void __attribute__((weak)) EXTI11_Callback(void) { }
void __attribute__((weak)) EXTI12_Callback(void) { }
void __attribute__((weak)) EXTI13_Callback(void) { }
void __attribute__((weak)) EXTI14_Callback(void) { }
void __attribute__((weak)) EXTI15_Callback(void) { }
