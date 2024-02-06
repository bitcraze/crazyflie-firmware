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
 * rpm.c - Deck that measure the motor RPM using QRD1114 IR reflector-sensor.
 */
#define DEBUG_MODULE "RPM"

#include <stdint.h>
#include <string.h>

#include "stm32fxxx.h"
#include "config.h"
#include "deck.h"
#include "debug.h"
#include "log.h"

//Hardware configuration
#define ET_GPIO_PERIF   (RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC)

#define ET_GPIO_PORT_TX2  GPIOA
#define ET_GPIO_PIN_TX2   GPIO_Pin_2
#define ET_GPIO_PORT_RX2  GPIOA
#define ET_GPIO_PIN_RX2   GPIO_Pin_3
#define ET_GPIO_PORT_IO2  GPIOB
#define ET_GPIO_PIN_IO2   GPIO_Pin_5
#define ET_GPIO_PORT_IO3  GPIOB
#define ET_GPIO_PIN_IO3   GPIO_Pin_4

#define ER_NBR_PINS         4

typedef struct _etGpio
{
  GPIO_TypeDef     *port;
  uint16_t          pin;
  char              name[6];
} EtGpio;

EtGpio erGpio[ER_NBR_PINS] =
{
    {ET_GPIO_PORT_TX2,  ET_GPIO_PIN_TX2, "TX2"},
    {ET_GPIO_PORT_RX2,  ET_GPIO_PIN_RX2, "RX2"},
    {ET_GPIO_PORT_IO2,  ET_GPIO_PIN_IO2, "IO2"},
    {ET_GPIO_PORT_IO3,  ET_GPIO_PIN_IO3, "IO3"},
};

static bool isInit;
static uint16_t lastcc1Val;
static uint16_t lastcc2Val;
static uint16_t lastcc3Val;
static uint16_t lastcc4Val;
static uint16_t m1Time[2];
static uint16_t m2Time[2];
static uint16_t m3Time[2];
static uint16_t m4Time[2];
static int m1cnt;
static int m2cnt;
static int m3cnt;
static int m4cnt;
static uint16_t m1rpm;
static uint16_t m2rpm;
static uint16_t m3rpm;
static uint16_t m4rpm;


static void rpmInit(DeckInfo *info)
{
  int i;
  isInit = true;

  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_ICInitTypeDef  TIM_ICInitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  // Enable Clocks
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM5, ENABLE);

  // Configure optical switch input pins
  for (i = 0; i < ER_NBR_PINS; i++)
  {
    GPIO_InitStructure.GPIO_Pin = erGpio[i].pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
    GPIO_Init(erGpio[i].port, &GPIO_InitStructure);
  }

   // Map timer to alternate functions
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM5);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM5);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
  TIM_TimeBaseStructure.TIM_Prescaler = (84 - 1); // 84 / 84M = 1uS
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM5, &TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
  TIM_ICInit(TIM5, &TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInit(TIM3, &TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInit(TIM3, &TIM_ICInitStructure);

  /* Enable the TIM5 and TIM3 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 10;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_Init(&NVIC_InitStructure);

  /* TIM enable counter */
  TIM_Cmd(TIM5, ENABLE);
  TIM_Cmd(TIM3, ENABLE);
  TIM_ITConfig(TIM5, TIM_IT_CC3, ENABLE);
  TIM_ITConfig(TIM5, TIM_IT_CC4, ENABLE);
  TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
  TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);
  TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE);
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
}

static uint16_t calcRPM(uint32_t t1, uint32_t t2)
{
  uint16_t rpm;
  // (timer resolution * sec/min) / (time for one revolution)
  rpm = (1000000 * 60) / (t1 + t2);

  return rpm;
}

void __attribute__((used)) TIM5_IRQHandler(void)
{
  static int updateM1Cnt = 2;
  static int updateM4Cnt = 2;
  uint16_t ccVal;

  //Motor1
  if(TIM_GetITStatus(TIM5, TIM_IT_CC3) == SET)
  {
    /* Clear TIM5 Capture compare interrupt pending bit */
    TIM_ClearITPendingBit(TIM5, TIM_IT_CC3);
    ccVal = TIM_GetCapture3(TIM5);

    if (TIM_GetFlagStatus(TIM5, TIM_FLAG_CC3OF))
    {
      // Overflow
      lastcc3Val = ccVal;
    }

    if (ccVal > lastcc3Val)
      m1Time[m1cnt & 0x01] = ccVal - lastcc3Val;
    else
      m1Time[m1cnt & 0x01] = (uint16_t)((0xFFFF + (uint32_t)ccVal) - lastcc3Val);

    lastcc3Val = ccVal;
    m1cnt++;
    m1rpm = calcRPM(m1Time[0], m1Time[1]);
    updateM1Cnt = 2;
  }

  //Motor4
  if(TIM_GetITStatus(TIM5, TIM_IT_CC4) == SET)
  {
    /* Clear TIM5 Capture compare interrupt pending bit */
    TIM_ClearITPendingBit(TIM5, TIM_IT_CC4);
    ccVal = TIM_GetCapture4(TIM5);

    if (TIM_GetFlagStatus(TIM5, TIM_FLAG_CC4OF))
    {
      // Overflow
      lastcc4Val = ccVal;
    }

    if (ccVal > lastcc4Val)
      m4Time[m4cnt & 0x01] = ccVal - lastcc4Val;
    else
      m4Time[m4cnt & 0x01] = (uint16_t)((0xFFFF + (uint32_t)ccVal) - lastcc4Val);

    lastcc4Val = ccVal;
    m4cnt++;
    m4rpm = calcRPM(m4Time[0], m4Time[1]);
    updateM4Cnt = 2;
  }

  if(TIM_GetITStatus(TIM5, TIM_IT_Update) == SET)
  {
    TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
    if (--updateM1Cnt < 0)
    {
      m1rpm = 0;
    }
    if (--updateM4Cnt < 0)
    {
      m4rpm = 0;
    }
  }
}

void __attribute__((used)) TIM3_IRQHandler(void)
{
  static int updateM2Cnt = 2;
  static int updateM3Cnt = 2;
  uint16_t ccVal;

  //Motor2
  if(TIM_GetITStatus(TIM3, TIM_IT_CC1) == SET)
  {
    /* Clear TIM3 Capture compare interrupt pending bit */
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
    ccVal = TIM_GetCapture1(TIM3);

    if (TIM_GetFlagStatus(TIM3, TIM_FLAG_CC1OF))
    {
      // Overflow
      lastcc1Val = ccVal;
    }

    if (ccVal > lastcc1Val)
      m2Time[m2cnt & 0x01] = ccVal - lastcc1Val;
    else
      m2Time[m2cnt & 0x01] = (uint16_t)((0xFFFF + (uint32_t)ccVal) - lastcc1Val);

    lastcc1Val = ccVal;
    m2cnt++;
    m2rpm = calcRPM(m2Time[0], m2Time[1]);
    updateM2Cnt = 2;

//    rpmPutchar((m2Time >> 8) & 0x00FF);
//    rpmPutchar(m2Time & 0x00FF);
  }

  //Motor3
  if(TIM_GetITStatus(TIM3, TIM_IT_CC2) == SET)
  {
    /* Clear TIM3 Capture compare interrupt pending bit */
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
    ccVal = TIM_GetCapture2(TIM3);

    if (TIM_GetFlagStatus(TIM3, TIM_FLAG_CC2OF))
    {
      // Overflow
      lastcc2Val = ccVal;
    }

    if (ccVal > lastcc2Val)
      m3Time[m3cnt & 0x01] = ccVal - lastcc2Val;
    else
      m3Time[m3cnt & 0x01] = (uint16_t)((0xFFFF + (uint32_t)ccVal) - lastcc2Val);

    lastcc2Val = ccVal;
    m3cnt++;
    m3rpm = calcRPM(m3Time[0], m3Time[1]);
    updateM3Cnt = 2;

//    rpmPutchar((m3rpm >> 8) & 0x00FF);
//    rpmPutchar(m3rpm & 0x00FF);
  }

  if(TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
    if (--updateM2Cnt < 0)
    {
      m2rpm = 0;
    }
    if (--updateM3Cnt < 0)
    {
      m3rpm = 0;
    }
  }
}

static const DeckDriver rpm_deck = {
  .vid = 0x00,
  .pid = 0x00,
  .name = "bcRpm",

  .usedGpio = DECK_USING_IO_2 | DECK_USING_IO_3 | DECK_USING_PA2 | DECK_USING_PA3,

  .init = rpmInit,
};

DECK_DRIVER(rpm_deck);


LOG_GROUP_START(rpm)
LOG_ADD(LOG_UINT16, m1, &m1rpm)
LOG_ADD(LOG_UINT16, m2, &m2rpm)
LOG_ADD(LOG_UINT16, m3, &m3rpm)
LOG_ADD(LOG_UINT16, m4, &m4rpm)
LOG_GROUP_STOP(rpm)

