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

void adcInit(void)
{
  /*
   * Note: This function initializes only ADC2, and only for single channel, single conversion mode. No DMA, no interrupts, no bells or whistles.
   */

  /* Note that this de-initializes registers for all ADCs (ADCx) */
  ADC_DeInit();

  /* Define ADC init structures */
  ADC_InitTypeDef       ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;

  /* Populates structures with reset values */
  ADC_StructInit(&ADC_InitStructure);
  ADC_CommonStructInit(&ADC_CommonInitStructure);

  /* enable ADC clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);

  /* init ADCs in independent mode, div clock by two */
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2; /* HCLK = 168MHz, PCLK2 = 84MHz, ADCCLK = 42MHz (when using ADC_Prescaler_Div2) */
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);

  /* Init ADC2: 10bit, single-conversion for Arduino compatibility */
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_10b;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = 0;
  ADC_InitStructure.ADC_ExternalTrigConv = 0;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC2, &ADC_InitStructure);

  /* Enable ADC2 */
  ADC_Cmd(ADC2, ENABLE);
}

static uint16_t analogReadChannel(uint8_t channel)
{
  /* According to datasheet, minimum sampling time for 10-bit conversion is almost 15 cycles. */
  ADC_RegularChannelConfig(ADC2, channel, 1, ADC_SampleTime_15Cycles);

  /* Start the conversion */
  ADC_SoftwareStartConv(ADC2);

  /* Wait until conversion completion */
  while(ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC) == RESET);

  /* Get the conversion value */
  return ADC_GetConversionValue(ADC2);
}

uint16_t analogRead(uint32_t pin)
{
  assert_param((pin >= 1) && (pin <= 13));
  assert_param(deckGPIOMapping[pin-1].adcCh > -1);

  /* Now set the GPIO pin to analog mode. */

  /* Enable clock for the peripheral of the pin.*/
  RCC_AHB1PeriphClockCmd(deckGPIOMapping[pin-1].periph, ENABLE);

  /* Populate structure with RESET values. */
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_StructInit(&GPIO_InitStructure);

  /* Initialise the GPIO pin to analog mode. */
  GPIO_InitStructure.GPIO_Pin   = deckGPIOMapping[pin-1].pin;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

  /* TODO: Any settling time before we can do ADC after init on the GPIO pin? */
  GPIO_Init(deckGPIOMapping[pin-1].port, &GPIO_InitStructure);

  /* Read the appropriate ADC channel. */
  return analogReadChannel((uint8_t)deckGPIOMapping[pin-1].adcCh);
}

void analogReference(uint8_t type)
{
  /*
   * TODO: We should probably support the Arduino EXTERNAL type here.
   * TODO: Figure out which voltage reference to compensate with.
   */
  assert_param(type == 0 /* DEFAULT */);
}
