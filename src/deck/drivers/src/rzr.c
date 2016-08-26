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
 * rzr.c - Crazyflie RZR board.
 */
#define DEBUG_MODULE "RZR"

#include <stdint.h>
#include <string.h>

#include "stm32fxxx.h"
#include "config.h"
#include "motors.h"
#include "debug.h"
#include "deck.h"
#include "extrx.h"
#include "pm.h"

#define RZR_GPIO_RCC_M1_OVERRIDE    RCC_AHB1Periph_GPIOA
#define RZR_GPIO_PORT_M1_OVERRIDE   GPIOA
#define RZR_GPIO_PIN_M1_OVERRIDE    GPIO_Pin_0
#define RZR_GPIO_RCC_M2_OVERRIDE    RCC_AHB1Periph_GPIOB
#define RZR_GPIO_PORT_M2_OVERRIDE   GPIOB
#define RZR_GPIO_PIN_M2_OVERRIDE    GPIO_Pin_12
#define RZR_GPIO_RCC_M3_OVERRIDE    RCC_AHB1Periph_GPIOC
#define RZR_GPIO_PORT_M3_OVERRIDE   GPIOC
#define RZR_GPIO_PIN_M3_OVERRIDE    GPIO_Pin_8
#define RZR_GPIO_RCC_M4_OVERRIDE    RCC_AHB1Periph_GPIOC
#define RZR_GPIO_PORT_M4_OVERRIDE   GPIOC
#define RZR_GPIO_PIN_M4_OVERRIDE    GPIO_Pin_15

//Hardware configuration
static bool isInit;

static void rzrInit(DeckInfo *info)
{
  GPIO_InitTypeDef GPIO_InitStructure = {0};

  if(isInit)
    return;

  DEBUG_PRINT("Switching to brushless.\n");


  // Configure GPIO for power to BL motor connectors
  RCC_AHB1PeriphClockCmd(RZR_GPIO_RCC_M1_OVERRIDE, ENABLE);
  RCC_AHB1PeriphClockCmd(RZR_GPIO_RCC_M2_OVERRIDE, ENABLE);
  RCC_AHB1PeriphClockCmd(RZR_GPIO_RCC_M3_OVERRIDE, ENABLE);
  RCC_AHB1PeriphClockCmd(RZR_GPIO_RCC_M4_OVERRIDE, ENABLE);

  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;

  GPIO_InitStructure.GPIO_Pin = RZR_GPIO_PIN_M1_OVERRIDE;
  GPIO_Init(RZR_GPIO_PORT_M1_OVERRIDE, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = RZR_GPIO_PIN_M2_OVERRIDE;
  GPIO_Init(RZR_GPIO_PORT_M2_OVERRIDE, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = RZR_GPIO_PIN_M3_OVERRIDE;
  GPIO_Init(RZR_GPIO_PORT_M3_OVERRIDE, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = RZR_GPIO_PIN_M4_OVERRIDE;
  GPIO_Init(RZR_GPIO_PORT_M4_OVERRIDE, &GPIO_InitStructure);

  // Enable for power to BL motor connectors
  GPIO_WriteBit(RZR_GPIO_PORT_M1_OVERRIDE, RZR_GPIO_PIN_M1_OVERRIDE, 1);
  GPIO_WriteBit(RZR_GPIO_PORT_M2_OVERRIDE, RZR_GPIO_PIN_M2_OVERRIDE, 1);
  GPIO_WriteBit(RZR_GPIO_PORT_M3_OVERRIDE, RZR_GPIO_PIN_M3_OVERRIDE, 1);
  GPIO_WriteBit(RZR_GPIO_PORT_M4_OVERRIDE, RZR_GPIO_PIN_M4_OVERRIDE, 1);

  // Remap motor PWM output
  motorsInit(motorMapRZRBrushless);

  isInit = true;
}

static bool rzrTest()
{
  bool status = true;

  if(!isInit)
    return false;

  status = motorsTest();

  return status;
}

static const DeckDriver rzr_deck = {
//  .vid = 0xBC,
//  .pid = 0x08,
  .name = "bcRZR",

  .usedPeriph = 0,
  .usedGpio = 0,
  .init = rzrInit,
  .test = rzrTest,
};

DECK_DRIVER(rzr_deck);
