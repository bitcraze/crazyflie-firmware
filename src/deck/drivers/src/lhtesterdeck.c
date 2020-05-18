/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2018 BitCraze AB
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
 * lhtesterdeck.c - Deck driver for the Lighthouse tester deck.
 *
 * Used in production tests of Lighthouse decks.
 *
 * The deck has an IR LED that is used to test the receivers on Lighthouse decks.
 * The LED is modulated at > 1 MHz and pulsed on/off.
 *
 */

#include <stdint.h>
#include <stdlib.h>
#include "stm32fxxx.h"
#include "FreeRTOS.h"
#include "timers.h"


#include "param.h"
#include "deck.h"

static bool isInit;

#define TIM_PERIF       RCC_APB1Periph_TIM5
#define TIM             TIM5
#define TIM_DBG         DBGMCU_TIM5_STOP
#define TIM_SETCOMPARE  TIM_SetCompare2
#define TIM_GETCAPTURE  TIM_GetCapture2

#define GPIO_POS_PERIF         RCC_AHB1Periph_GPIOA
#define GPIO_POS_PORT          GPIOA
#define GPIO_POS_PIN           GPIO_Pin_2 // TIM5_CH3
#define GPIO_AF_POS_PIN        GPIO_PinSource2
#define GPIO_AF_POS            GPIO_AF_TIM5

#define PERIOD 0x40

static void ledOnOff(bool on)
{
  uint8_t ratio = 0;
  if (on) {
    ratio = PERIOD / 2;
  }

  TIM_SetCompare3(TIM, ratio);
  TIM_SetCompare4(TIM, ratio);
}

static void timerFcn(xTimerHandle xTimer)
{
  static int onCounter = 0;

  bool isOn = (onCounter == 0);
  ledOnOff(isOn);

  onCounter++;
  if (onCounter > 10) {
    onCounter = 0;
  }
}

static void startSwTimer() {
  xTimerHandle timer = xTimerCreate("lhTesterTimer", M2T(1), pdTRUE, 0, timerFcn);
  xTimerStart(timer, 0);
}

static void setUpHwTimer(){
  //Init structures
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  //Clock the gpio and the timers
  RCC_AHB1PeriphClockCmd(GPIO_POS_PERIF, ENABLE);
  RCC_APB1PeriphClockCmd(TIM_PERIF, ENABLE);

  // Configure the GPIO for the timer output
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Pin = GPIO_POS_PIN;
  GPIO_Init(GPIO_POS_PORT, &GPIO_InitStructure);

  //Map timers to alternate functions
  GPIO_PinAFConfig(GPIO_POS_PORT, GPIO_AF_POS_PIN, GPIO_AF_POS);

  //Timer configuration
  TIM_TimeBaseStructure.TIM_Period = PERIOD - 1;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM, &TIM_TimeBaseStructure);

  // PWM channels configuration
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

  // Configure OC3
  TIM_OC3Init(TIM, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM, TIM_OCPreload_Enable);

  //Enable the timer PWM outputs
  TIM_CtrlPWMOutputs(TIM, ENABLE);
  TIM_SetCompare3(TIM, 0x00);
  TIM_SetCompare4(TIM, 0x00);

  //Enable the timer
  TIM_Cmd(TIM, ENABLE);
}


static void lhTesterDeckInit(DeckInfo *info)
{
  if (isInit) {
    return;
  }

  // HW timer for modulating the LED with > 1MHz
  setUpHwTimer();

  // SW timer for turning the modulated light on and off with a 20 ms cycle
  startSwTimer();

  isInit = true;
}

static const DeckDriver lhTester_deck = {
  .vid = 0x00,
  .pid = 0x00,
  .name = "bcLhTester",

  .usedPeriph = DECK_USING_TIMER5,
  .usedGpio = DECK_USING_TX2 | DECK_USING_RX2,

  .init = lhTesterDeckInit,
};

DECK_DRIVER(lhTester_deck);

PARAM_GROUP_START(deck)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcLhTester, &isInit)
PARAM_GROUP_STOP(deck)
