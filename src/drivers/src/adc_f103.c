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
 * adc.c - Analog Digital Conversion
 *
 * TODO: Describe functionality.
 *
 * Sample time: According to the formula in the stm32 product manual
 *              page 69, with a Ts of 28.5 samples, 12-bit, and ADC@12
 *              the highest impedance to use is 25.2kOhm.
 */
#include "stm32fxxx.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "adc.h"
#include "pm.h"
#include "nvicconf.h"
#include "imu.h"
#include "static_mem.h"

// PORT A
#define GPIO_VBAT        GPIO_Pin_3

// CHANNELS
#define NBR_OF_ADC_CHANNELS   1
#define CH_VBAT               ADC_Channel_3

#define CH_VREF               ADC_Channel_17
#define CH_TEMP               ADC_Channel_16

static bool isInit;
volatile AdcGroup adcValues[ADC_MEAN_SIZE * 2];

static xQueueHandle adcQueue;
STATIC_MEM_QUEUE_ALLOC(adcQueue, 1, AdcGroup);
STATIC_MEM_TASK_ALLOC(adcTask, ADC_TASK_STACKSIZE);

static void adcDmaInit(void)
{
  DMA_InitTypeDef DMA_InitStructure;

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  // DMA channel1 configuration
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&adcValues;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = NBR_OF_ADC_CHANNELS * (ADC_MEAN_SIZE * 2);
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  // Enable DMA channel1
  DMA_Cmd(DMA1_Channel1, ENABLE);
}

/**
 * Decimates the adc samples after oversampling
 */
static void adcDecimate(AdcGroup* oversampled, AdcGroup* decimated)
{
  uint32_t i, j;
  uint32_t sum;
  uint32_t sumVref;
  AdcGroup* adcIterator;
  AdcPair *adcOversampledPair;
  AdcPair *adcDecimatedPair;

  // Compute sums and decimate each channel
  adcDecimatedPair = (AdcPair*)decimated;
  for (i = 0; i < NBR_OF_ADC_CHANNELS; i++)
  {
    adcIterator = oversampled;
    sum = 0;
    sumVref = 0;
    for (j = 0; j < ADC_MEAN_SIZE; j++)
    {
      adcOversampledPair = &((AdcPair*)adcIterator)[i];
      sum += adcOversampledPair->val;
      sumVref += adcOversampledPair->vref;
      adcIterator++;
    }
    // Decimate
    adcDecimatedPair->val = sum / ADC_DECIMATE_DIVEDEND;
    adcDecimatedPair->vref = sumVref / ADC_DECIMATE_DIVEDEND;
    adcDecimatedPair++;
  }
}

void adcInit(void)
{

  if(isInit)
    return;

  ADC_InitTypeDef ADC_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  // Enable TIM2, GPIOA and ADC1 clock
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2 |
                         RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

  //Timer configuration
  TIM_TimeBaseStructure.TIM_Period = ADC_TRIG_PERIOD;
  TIM_TimeBaseStructure.TIM_Prescaler = ADC_TRIG_PRESCALE;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  // TIM2 channel2 configuration in PWM mode
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 1;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OC2Init(TIM2, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
  // Halt timer 2 during debug halt.
  DBGMCU_Config(DBGMCU_TIM2_STOP, ENABLE);

  adcDmaInit();

  // ADC1 configuration
  ADC_DeInit(ADC1);
  ADC_InitStructure.ADC_Mode = ADC_Mode_RegSimult;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_CC2;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = NBR_OF_ADC_CHANNELS;
  ADC_Init(ADC1, &ADC_InitStructure);

  // ADC1 channel sequence
  ADC_RegularChannelConfig(ADC1, CH_VREF, 1, ADC_SampleTime_28Cycles5);

  // ADC2 configuration
  ADC_DeInit(ADC2);
  ADC_InitStructure.ADC_Mode = ADC_Mode_RegSimult;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = NBR_OF_ADC_CHANNELS;
  ADC_Init(ADC2, &ADC_InitStructure);

  // ADC2 channel sequence
  ADC_RegularChannelConfig(ADC2, CH_VBAT, 1, ADC_SampleTime_28Cycles5);

  // Enable ADC1
  ADC_Cmd(ADC1, ENABLE);
  // Calibrate ADC1
  ADC_ResetCalibration(ADC1);
  while(ADC_GetResetCalibrationStatus(ADC1));
  ADC_StartCalibration(ADC1);
  while(ADC_GetCalibrationStatus(ADC1));

  // Enable ADC1 external trigger
  ADC_ExternalTrigConvCmd(ADC1, ENABLE);
  ADC_TempSensorVrefintCmd(ENABLE);

  // Enable ADC2
  ADC_Cmd(ADC2, ENABLE);
  // Calibrate ADC2
  ADC_ResetCalibration(ADC2);
  while(ADC_GetResetCalibrationStatus(ADC2));
  ADC_StartCalibration(ADC2);
  while(ADC_GetCalibrationStatus(ADC2));

  // Enable ADC2 external trigger
  ADC_ExternalTrigConvCmd(ADC2, ENABLE);

  // Enable the DMA1 channel1 Interrupt
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_ADC_PRI;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  adcQueue = STATIC_MEM_QUEUE_CREATE(adcQueue);

  STATIC_MEM_TASK_CREATE(adcTask, adcTask, ADC_TASK_NAME, NULL, ADC_TASK_NAME);

  isInit = true;
}

bool adcTest(void)
{
  return isInit;
}

float adcConvertToVoltageFloat(uint16_t v, uint16_t vref)
{
  return (v / (vref / ADC_INTERNAL_VREF));
}

void adcDmaStart(void)
{
  // Enable the Transfer Complete and Half Transfer Interrupt
  DMA_ITConfig(DMA1_Channel1, DMA_IT_TC | DMA_IT_HT, ENABLE);
  // Enable ADC1 DMA
  ADC_DMACmd(ADC1, ENABLE);
  // TIM2 counter enable
  TIM_Cmd(TIM2, ENABLE);
}

void adcDmaStop(void)
{
//  TIM_Cmd(TIM2, DISABLE);
}

void adcInterruptHandler(void)
{
  portBASE_TYPE xHigherPriorityTaskWoken;
  AdcGroup* adcBuffer;

  if(DMA_GetITStatus(DMA1_IT_HT1))
  {
    DMA_ClearITPendingBit(DMA1_IT_HT1);
    adcBuffer = (AdcGroup*)&adcValues[0];
    xQueueSendFromISR(adcQueue, &adcBuffer, &xHigherPriorityTaskWoken);
  }
  if(DMA_GetITStatus(DMA1_IT_TC1))
  {
    DMA_ClearITPendingBit(DMA1_IT_TC1);
    adcBuffer = (AdcGroup*)&adcValues[ADC_MEAN_SIZE];
    xQueueSendFromISR(adcQueue, &adcBuffer, &xHigherPriorityTaskWoken);
  }
}

void adcTask(void *param)
{
  AdcGroup* adcRawValues;
  AdcGroup adcValues;

  vTaskSetApplicationTaskTag(0, (void*)TASK_ADC_ID_NBR);
  vTaskDelay(1000);

  adcDmaStart();

  while(1)
  {
    xQueueReceive(adcQueue, &adcRawValues, portMAX_DELAY);
    adcDecimate(adcRawValues, &adcValues);  // 10% CPU
    pmBatteryUpdate(&adcValues);

#ifdef ADC_OUTPUT_RAW_DATA
    uartSendDataDma(sizeof(AdcGroup)*ADC_MEAN_SIZE, (uint8_t*)adcRawValues);
#endif
  }
}

void __attribute__((used)) DMA1_Channel1_IRQHandler(void)
{
  adcInterruptHandler();
}
