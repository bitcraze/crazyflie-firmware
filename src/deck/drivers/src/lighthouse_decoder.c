/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2017 Bitcraze AB
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
 *
 * cppm.c - Combined PPM / PPM-Sum driver
 */

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "deck.h"

#include "stm32fxxx.h"
#include "nvicconf.h"

#define DEBUG_MODULE  "LHPULSE"
#include "debug.h"
#include "log.h"


#define LHPULSE_TIMER                   TIM5
#define LHPULSE_TIMER_RCC               RCC_APB1Periph_TIM5
#define LHPULSE_GPIO_RCC                RCC_AHB1Periph_GPIOA
#define LHPULSE_GPIO_PORT               GPIOA

#define LHPULSE_RIGHT_GPIO_PIN                GPIO_Pin_2      // TIM5_CH3
#define LHPULSE_RIGHT_GPIO_SOURCE             GPIO_PinSource2

#define LHPULSE_LEFT_GPIO_PIN                GPIO_Pin_3      // TIM5_CH4
#define LHPULSE_LEFT_GPIO_SOURCE             GPIO_PinSource3

#define LHPULSE_GPIO_AF                 GPIO_AF_TIM5

#define LHPULSE_TIM_PRESCALER           (0) // TIM14 clock running at sysclk/2. Gives us 84MHz

void lhInit(DeckInfo* info)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_ICInitTypeDef  TIM_ICInitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_AHB1PeriphClockCmd(LHPULSE_GPIO_RCC, ENABLE);
  RCC_APB1PeriphClockCmd(LHPULSE_TIMER_RCC, ENABLE);

  // Configure the GPIO for the timer input
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = LHPULSE_RIGHT_GPIO_PIN;
  GPIO_Init(LHPULSE_GPIO_PORT, &GPIO_InitStructure);

  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = LHPULSE_LEFT_GPIO_PIN;
  GPIO_Init(LHPULSE_GPIO_PORT, &GPIO_InitStructure);

  GPIO_PinAFConfig(LHPULSE_GPIO_PORT, LHPULSE_RIGHT_GPIO_SOURCE, LHPULSE_GPIO_AF);
  GPIO_PinAFConfig(LHPULSE_GPIO_PORT, LHPULSE_LEFT_GPIO_SOURCE, LHPULSE_GPIO_AF);

  // Time base configuration. Count at 84MHz.
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Prescaler = LHPULSE_TIM_PRESCALER;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(LHPULSE_TIMER, &TIM_TimeBaseStructure);



  // Setup input capture to measure pulse width
  // RIGHT
  // This activates XOR to route channel 3 to channel 1
  TIM_SelectHallSensor(LHPULSE_TIMER, ENABLE);

  TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInit(LHPULSE_TIMER, &TIM_ICInitStructure);

  TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_IndirectTI;
  TIM_ICInit(LHPULSE_TIMER, &TIM_ICInitStructure);

  // LEFT
  TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_IndirectTI;
  TIM_ICInit(LHPULSE_TIMER, &TIM_ICInitStructure);

  TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInit(LHPULSE_TIMER, &TIM_ICInitStructure);

  // Enable interrupts
  NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_MID_PRI;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  TIM_ITConfig(LHPULSE_TIMER, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4, ENABLE);
  TIM_Cmd(LHPULSE_TIMER, ENABLE);
}

static uint32_t pulseWidthRight;
static uint32_t pulseWidthLeft;

void __attribute__((used)) TIM5_IRQHandler()
{
  static uint32_t pulseStartRight = 0;
  static uint32_t pulseStartLeft = 0;

  // pulseWidth++;

  if (TIM_GetITStatus(LHPULSE_TIMER, TIM_IT_CC1) != RESET)
  {
    pulseStartRight = TIM_GetCapture1(LHPULSE_TIMER);

    TIM_ClearITPendingBit(LHPULSE_TIMER, TIM_IT_CC1);
  }

  if (TIM_GetITStatus(LHPULSE_TIMER, TIM_IT_CC2) != RESET)
  {

    pulseWidthRight = TIM_GetCapture2(LHPULSE_TIMER) - pulseStartRight;

    TIM_ClearITPendingBit(LHPULSE_TIMER, TIM_IT_CC2);
  }


  if (TIM_GetITStatus(LHPULSE_TIMER, TIM_IT_CC3) != RESET)
  {
    pulseStartLeft = TIM_GetCapture3(LHPULSE_TIMER);

    TIM_ClearITPendingBit(LHPULSE_TIMER, TIM_IT_CC3);
  }

  if (TIM_GetITStatus(LHPULSE_TIMER, TIM_IT_CC4) != RESET)
  {

    pulseWidthLeft = TIM_GetCapture4(LHPULSE_TIMER) - pulseStartLeft;

    TIM_ClearITPendingBit(LHPULSE_TIMER, TIM_IT_CC4);
  }

}

static const DeckDriver lighthouse_deck = {
  .vid = 0,
  .pid = 0,
  .name = "bcLH",

  .usedGpio = 0,  // FIXME: set the used pins

  .init = lhInit,
};

DECK_DRIVER(lighthouse_deck);

LOG_GROUP_START(lhpulse)
LOG_ADD(LOG_UINT32, widthRight, &pulseWidthRight)
LOG_ADD(LOG_UINT32, widthLeft, &pulseWidthLeft)
LOG_GROUP_STOP(lhpulse)
