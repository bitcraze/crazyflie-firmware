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
 * deck_analog.c - Arduino-compatible analog input implementation
 */

#include "deck.h"

#include "stm32fxxx.h"

// Mapping between Pin number and real GPIO
// TODO: Copy of identical struct in deck_digital.c - should be shared
const struct {
  uint32_t periph;
  GPIO_TypeDef* port;
  uint16_t pin;
} gpioMapping2[13] = {
  {.periph= RCC_AHB1Periph_GPIOC, .port= GPIOC, .pin=GPIO_Pin_11},
  {.periph= RCC_AHB1Periph_GPIOC, .port= GPIOC, .pin=GPIO_Pin_10},
  {.periph= RCC_AHB1Periph_GPIOB, .port= GPIOB, .pin=GPIO_Pin_7},
  {.periph= RCC_AHB1Periph_GPIOB, .port= GPIOB, .pin=GPIO_Pin_6},
  {.periph= RCC_AHB1Periph_GPIOB, .port= GPIOB, .pin=GPIO_Pin_8},
  {.periph= RCC_AHB1Periph_GPIOB, .port= GPIOB, .pin=GPIO_Pin_5},
  {.periph= RCC_AHB1Periph_GPIOB, .port= GPIOB, .pin=GPIO_Pin_4},
  {.periph= RCC_AHB1Periph_GPIOC, .port= GPIOC, .pin=GPIO_Pin_12},
  {.periph= RCC_AHB1Periph_GPIOA, .port= GPIOA, .pin=GPIO_Pin_2},
  {.periph= RCC_AHB1Periph_GPIOA, .port= GPIOA, .pin=GPIO_Pin_3},
  {.periph= RCC_AHB1Periph_GPIOA, .port= GPIOA, .pin=GPIO_Pin_5},
  {.periph= RCC_AHB1Periph_GPIOA, .port= GPIOA, .pin=GPIO_Pin_6},
  {.periph= RCC_AHB1Periph_GPIOA, .port= GPIOA, .pin=GPIO_Pin_7},
};

/* TODO: Merged as extra column into the gpioMapping struct? */
const struct {
    int8_t adcCh; /* -1 means no ADC available for this pin. */
} adcMapping[13] = {
    {.adcCh=-1},            /* RX1 */
    {.adcCh=-1},            /* TX1 */
    {.adcCh=-1},            /* SDA */
    {.adcCh=-1},            /* SCL */
    {.adcCh=-1},            /* IO1 */
    {.adcCh=-1},            /* IO2 */
    {.adcCh=-1},            /* IO3 */
    {.adcCh=-1},            /* IO4 */
    {.adcCh=ADC_Channel_2}, /* TX2 */
    {.adcCh=ADC_Channel_3}, /* RX2 */
    {.adcCh=ADC_Channel_5}, /* SCK */
    {.adcCh=ADC_Channel_6}, /* MISO */
    {.adcCh=ADC_Channel_7}, /* MOSI */
};

void adcInit(void)
{
  /*
   * Note: This function initializes only ADC1, and only for single channel, single conversion mode. No DMA, no interrupts, no bells or whistles.
   */

  /* Note that this de-initializes registers for all ADCs (ADCx) */
  // ADC_DeInit();

  /* Define ADC init structures */
  ADC_InitTypeDef       ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;

  /* IMPORTANT: populates structures with reset values */
  ADC_StructInit(&ADC_InitStructure);
  ADC_CommonStructInit(&ADC_CommonInitStructure);

  /* enable ADC clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

  /* init ADCs in independent mode, div clock by two */
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);

  /* init ADC1: 10bit, single-conversion for Arduino compatibility */
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_10b;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = 0;
  ADC_InitStructure.ADC_ExternalTrigConv = 0;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC1, &ADC_InitStructure);

  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);
}

static uint16_t analogReadChannel(uint8_t channel)
{
  ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_480Cycles);

  /* Start the conversion */
  ADC_SoftwareStartConv(ADC1);

  /* Wait until conversion completion */
  while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);

  /* Get the conversion value */
  return ADC_GetConversionValue(ADC1);
}

uint16_t analogRead(uint32_t pin)
{
  assert_param((pin >= 1) && (pin <= 13));
  assert_param(adcMapping[pin-1].adcCh > -1);

  /* Now set the GPIO pin to analog mode. */

  /* Enable clock for the peripheral of the pin.*/
  RCC_AHB1PeriphClockCmd(gpioMapping2[pin-1].periph, ENABLE);

  /* Populate structure with RESET values. */
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_StructInit(&GPIO_InitStructure);

  /* Initialise the GPIO pin to analog mode. According to the datasheet, only the analog mode needs to be set. */
  GPIO_InitStructure.GPIO_Pin = gpioMapping2[pin-1].pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;

  /* TODO: Any settling time before we can do ADC after init on the GPIO pin? */
  GPIO_Init(gpioMapping2[pin-1].port, &GPIO_InitStructure);

  /* Read the appropriate ADC channel. */
  return analogReadChannel((uint8_t)adcMapping[pin-1].adcCh);
}

void analogReference(uint8_t type)
{
  /*
   * TODO: We should probably support the Arduino EXTERNAL type here.
   * TODO: Figure out which voltage reference to compensate with.
   */
  assert_param(type == 0 /* DEFAULT */);
}
