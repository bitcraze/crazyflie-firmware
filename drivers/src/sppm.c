/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2013 Bitcraze AB
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
 * sppm.c - PPM sum driver
 */

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "stm32fxxx.h"
#include "sppm.h"
#include "nvicconf.h"
#include "commander.h"

#define DEBUG_MODULE  "SPPM"
#include "debug.h"
#include "log.h"


#define SPPM_TIMER                   TIM14
#define SPPM_TIMER_RCC               RCC_APB1Periph_TIM14
#define SPPM_TIMER_CH_Init           TIM_OC1Init
#define SPPM_TIMER_CH_PreloadConfig  TIM_OC1PreloadConfig
#define SPPM_TIMER_CH_SetCompare     TIM_SetCompare1
#define SPPM_GPIO_RCC                RCC_AHB1Periph_GPIOA
#define SPPM_GPIO_PORT               GPIOA
#define SPPM_GPIO_PIN                GPIO_Pin_7
#define SPPM_GPIO_SOURCE             GPIO_PinSource7
#define SPPM_GPIO_AF                 GPIO_AF_TIM14

#define SPPM_TIM_PRESCALER           (84 - 1) // TIM14 clock running at sysclk/2. Will give 1us tick.

#define SPPM_MIN_PPM_USEC            1100
#define SPPM_MAX_PPM_USEC            1900

static xQueueHandle captureQueue;
static uint16_t prevCapureVal;
static bool captureFlag;
static bool isAvailible;

void sppmInit(void)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_ICInitTypeDef  TIM_ICInitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_AHB1PeriphClockCmd(SPPM_GPIO_RCC, ENABLE);
  RCC_APB1PeriphClockCmd(SPPM_TIMER_RCC, ENABLE);

  // Configure the GPIO for the timer input
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = SPPM_GPIO_PIN;
  GPIO_Init(SPPM_GPIO_PORT, &GPIO_InitStructure);

  GPIO_PinAFConfig(SPPM_GPIO_PORT, SPPM_GPIO_SOURCE, SPPM_GPIO_AF);

  // Time base configuration. 1us tick.
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Prescaler = SPPM_TIM_PRESCALER;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(SPPM_TIMER, &TIM_TimeBaseStructure);

  // Setup input capture using default config.
  TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInit(SPPM_TIMER, &TIM_ICInitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = TIM8_TRG_COM_TIM14_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_SPPM_PRI;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  captureQueue = xQueueCreate(64, sizeof(uint16_t));

  TIM_ITConfig(SPPM_TIMER, TIM_IT_Update | TIM_IT_CC1, ENABLE);
  TIM_Cmd(SPPM_TIMER, ENABLE);
}

bool sppmIsAvailible(void)
{
  return isAvailible;
}

int sppmGetTimestamp(uint16_t *timestamp)
{
  ASSERT(timestamp);

  return xQueueReceive(captureQueue, timestamp, portMAX_DELAY);
}

void sppmClearQueue(void)
{
  xQueueReset(captureQueue);
}

float sppmConvert2Float(uint16_t timestamp, float min, float max)
{
  if (timestamp < SPPM_MIN_PPM_USEC)
  {
    timestamp = SPPM_MIN_PPM_USEC;
  }
  if (timestamp > SPPM_MAX_PPM_USEC)
  {
    timestamp = SPPM_MAX_PPM_USEC;
  }

  float scale = (float)(timestamp - SPPM_MIN_PPM_USEC) / (float)(SPPM_MAX_PPM_USEC - SPPM_MIN_PPM_USEC);

  return min + ((max - min) * scale);
}

uint16_t sppmConvert2uint16(uint16_t timestamp)
{
  if (timestamp < SPPM_MIN_PPM_USEC)
  {
    timestamp = SPPM_MIN_PPM_USEC;
  }
  if (timestamp > SPPM_MAX_PPM_USEC)
  {
    timestamp = SPPM_MAX_PPM_USEC;
  }

  uint16_t base = (timestamp - SPPM_MIN_PPM_USEC);

  return base * (65535 / (SPPM_MAX_PPM_USEC - SPPM_MIN_PPM_USEC));
}

void __attribute__((used)) TIM8_TRG_COM_TIM14_IRQHandler()
{
  uint16_t capureVal;
  uint16_t capureValDiff;
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  if (TIM_GetITStatus(SPPM_TIMER, TIM_IT_CC1) != RESET)
  {
    if (TIM_GetFlagStatus(SPPM_TIMER, TIM_FLAG_CC1OF) != RESET)
    {
      //TODO: Handle overflow error
    }

    capureVal = TIM_GetCapture1(SPPM_TIMER);
    capureValDiff = capureVal - prevCapureVal;
    prevCapureVal = capureVal;

    xQueueSendFromISR(captureQueue, &capureValDiff, &xHigherPriorityTaskWoken);

    captureFlag = true;
    TIM_ClearITPendingBit(SPPM_TIMER, TIM_IT_CC1);
  }

  if (TIM_GetITStatus(SPPM_TIMER, TIM_IT_Update) != RESET)
  {
    // Update input status
    isAvailible = (captureFlag == true);
    captureFlag = false;
    TIM_ClearITPendingBit(SPPM_TIMER, TIM_IT_Update);
  }
}
