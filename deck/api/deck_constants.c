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
 * deck_constants.c - Constants for the Deck API
 */

#include "deck.h"

/* Mapping between Deck Pin number, real GPIO and ADC channel */
deckGPIOMapping_t deckGPIOMapping[13] = {
  {.periph= RCC_AHB1Periph_GPIOC, .port= GPIOC, .pin=GPIO_Pin_11, .adcCh=-1},            /* RX1 */
  {.periph= RCC_AHB1Periph_GPIOC, .port= GPIOC, .pin=GPIO_Pin_10, .adcCh=-1},            /* TX1 */
  {.periph= RCC_AHB1Periph_GPIOB, .port= GPIOB, .pin=GPIO_Pin_7,  .adcCh=-1},            /* SDA */
  {.periph= RCC_AHB1Periph_GPIOB, .port= GPIOB, .pin=GPIO_Pin_6,  .adcCh=-1},            /* SCL */
  {.periph= RCC_AHB1Periph_GPIOB, .port= GPIOB, .pin=GPIO_Pin_8,  .adcCh=-1},            /* IO1 */
  {.periph= RCC_AHB1Periph_GPIOB, .port= GPIOB, .pin=GPIO_Pin_5,  .adcCh=-1},            /* IO2 */
  {.periph= RCC_AHB1Periph_GPIOB, .port= GPIOB, .pin=GPIO_Pin_4,  .adcCh=-1},            /* IO3 */
  {.periph= RCC_AHB1Periph_GPIOC, .port= GPIOC, .pin=GPIO_Pin_12, .adcCh=-1},            /* IO4 */
  {.periph= RCC_AHB1Periph_GPIOA, .port= GPIOA, .pin=GPIO_Pin_2,  .adcCh=ADC_Channel_2}, /* TX2 */
  {.periph= RCC_AHB1Periph_GPIOA, .port= GPIOA, .pin=GPIO_Pin_3,  .adcCh=ADC_Channel_3}, /* RX2 */
  {.periph= RCC_AHB1Periph_GPIOA, .port= GPIOA, .pin=GPIO_Pin_5,  .adcCh=ADC_Channel_5}, /* SCK */
  {.periph= RCC_AHB1Periph_GPIOA, .port= GPIOA, .pin=GPIO_Pin_6,  .adcCh=ADC_Channel_6}, /* MISO */
  {.periph= RCC_AHB1Periph_GPIOA, .port= GPIOA, .pin=GPIO_Pin_7,  .adcCh=ADC_Channel_7}, /* MOSI */
};
