/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2015 Bitcraze AB
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
 * digital.c - Deck-API digital IO implementation
 */

#include "deck.h"

#include "stm32fxxx.h"

void pinMode(uint32_t pin, uint32_t mode)
{
  if (pin > 13) {
    return;
  }

  RCC_AHB1PeriphClockCmd(deckGPIOMapping[pin-1].periph, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure = {0};

  GPIO_InitStructure.GPIO_Pin = deckGPIOMapping[pin-1].pin;
  GPIO_InitStructure.GPIO_Mode = (mode == OUTPUT) ? GPIO_Mode_OUT:GPIO_Mode_IN;
  if (mode == OUTPUT) GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  if (mode == INPUT_PULLUP) GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  if (mode == INPUT_PULLDOWN) GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_Init(deckGPIOMapping[pin-1].port, &GPIO_InitStructure);
}

void digitalWrite(uint32_t pin, uint32_t val)
{
  if (pin > 13) {
    return;
  }

  if (val) val = Bit_SET;

  GPIO_WriteBit(deckGPIOMapping[pin-1].port, deckGPIOMapping[pin-1].pin, val);
}

int digitalRead(uint32_t pin)
{
  if (pin > 13) {
    return LOW;
  }

  int val = GPIO_ReadInputDataBit(deckGPIOMapping[pin-1].port, deckGPIOMapping[pin-1].pin);
  return (val==Bit_SET)?HIGH:LOW;
}
