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
 * piezo.c - Piezo/Buzzer driver
 *
 * This code mainly interfacing the PWM peripheral lib of ST.
 */

#include <stdbool.h>

/* ST includes */
#include "stm32fxxx.h"

#include "piezo.h"

//FreeRTOS includes
#include "FreeRTOS.h"
#include "task.h"

// HW defines
#define PIEZO_TIM_PERIF       RCC_APB1Periph_TIM5
#define PIEZO_TIM             TIM5
#define PIEZO_TIM_DBG         DBGMCU_TIM5_STOP
#define PIEZO_TIM_SETCOMPARE  TIM_SetCompare2
#define PIEZO_TIM_GETCAPTURE  TIM_GetCapture2

#define PIEZO_GPIO_POS_PERIF         RCC_AHB1Periph_GPIOA
#define PIEZO_GPIO_POS_PORT          GPIOA
#define PIEZO_GPIO_POS_PIN           GPIO_Pin_2 // TIM5_CH3
#define PIEZO_GPIO_AF_POS_PIN        GPIO_PinSource2
#define PIEZO_GPIO_AF_POS            GPIO_AF_TIM5

#define PIEZO_GPIO_NEG_PERIF         RCC_AHB1Periph_GPIOA
#define PIEZO_GPIO_NEG_PORT          GPIOA
#define PIEZO_GPIO_NEG_PIN           GPIO_Pin_3 // TIM5_CH4
#define PIEZO_GPIO_AF_NEG_PIN        GPIO_PinSource3
#define PIEZO_GPIO_AF_NEG            GPIO_AF_TIM5

#define PIEZO_PWM_BITS      (8)
#define PIEZO_PWM_PERIOD    ((1<<PIEZO_PWM_BITS) - 1)
#define PIEZO_PWM_PRESCALE  (0)

/* This should be calculated.. */
#define PIEZO_BASE_FREQ (329500)

static bool isInit = false;

/* Public functions */

void piezoInit()
{
  if (isInit)
    return;

  //Init structures
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  //Clock the gpio and the timers
  RCC_AHB1PeriphClockCmd(PIEZO_GPIO_POS_PERIF | PIEZO_GPIO_NEG_PERIF, ENABLE);
  RCC_APB1PeriphClockCmd(PIEZO_TIM_PERIF, ENABLE);

  // Configure the GPIO for the timer output
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Pin = PIEZO_GPIO_POS_PIN;
  GPIO_Init(PIEZO_GPIO_POS_PORT, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = PIEZO_GPIO_NEG_PIN;
  GPIO_Init(PIEZO_GPIO_NEG_PORT, &GPIO_InitStructure);

  //Map timers to alternate functions
  GPIO_PinAFConfig(PIEZO_GPIO_POS_PORT, PIEZO_GPIO_AF_POS_PIN, PIEZO_GPIO_AF_POS);
  GPIO_PinAFConfig(PIEZO_GPIO_NEG_PORT, PIEZO_GPIO_AF_NEG_PIN, PIEZO_GPIO_AF_NEG);

  //Timer configuration
  TIM_TimeBaseStructure.TIM_Period = PIEZO_PWM_PERIOD;
  TIM_TimeBaseStructure.TIM_Prescaler = PIEZO_PWM_PRESCALE;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(PIEZO_TIM, &TIM_TimeBaseStructure);

  // PWM channels configuration
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

  // Configure OC3
  TIM_OC3Init(PIEZO_TIM, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(PIEZO_TIM, TIM_OCPreload_Enable);

  // Configure OC4 inverted
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC4Init(PIEZO_TIM, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(PIEZO_TIM, TIM_OCPreload_Enable);

  //Enable the timer PWM outputs
  TIM_CtrlPWMOutputs(PIEZO_TIM, ENABLE);
  TIM_SetCompare3(PIEZO_TIM, 0x00);
  TIM_SetCompare4(PIEZO_TIM, 0x00);

  //Enable the timer
  TIM_Cmd(PIEZO_TIM, ENABLE);

  isInit = true;
}

bool piezoTest(void)
{
  return isInit;
}

void piezoSetRatio(uint8_t ratio)
{
  TIM_SetCompare3(PIEZO_TIM, ratio);
  TIM_SetCompare4(PIEZO_TIM, ratio);
}

void piezoSetFreq(uint16_t freq)
{
  TIM_PrescalerConfig(PIEZO_TIM, (PIEZO_BASE_FREQ/freq), TIM_PSCReloadMode_Update);
}
