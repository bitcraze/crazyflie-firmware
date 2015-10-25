/**
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2014 Bitcraze AB
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
 * ws2812.c: Neopixel LED driver
 * Mostly from Elia's electonic blog: http://eliaselectronics.com/driving-a-ws2812-rgb-led-with-an-stm32/
 */

#include <string.h>

// ST lib includes
#include "stm32fxxx.h"

#include "FreeRTOS.h"
#include "semphr.h"

//#define TIM1_CCR1_Address 0x40012C34	// physical memory address of Timer 3 CCR1 register

static TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
static TIM_OCInitTypeDef  TIM_OCInitStructure;
static GPIO_InitTypeDef GPIO_InitStructure;
static DMA_InitTypeDef DMA_InitStructure;
static NVIC_InitTypeDef NVIC_InitStructure;

static xSemaphoreHandle allLedDone = NULL;

// The minimum is to have 2 leds (1 per half buffer) in the buffer, this
// consume 42Bytes and will trigger the DMA interrupt at ~2KHz.
// Putting 2 there will divide by 2 the interrupt frequency but will also 
// double the memory consumption (no free lunch ;-)
#define LED_PER_HALF 1

#define TIMING_ONE  75
#define TIMING_ZERO 29

static union {
    uint16_t buffer[2*LED_PER_HALF*24];
    struct {
      uint16_t begin[LED_PER_HALF*24];
      uint16_t end[LED_PER_HALF*24];
    } __attribute__((packed));
} led_dma;

void ws2812Init(void)
{
	uint16_t PrescalerValue;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	/* GPIOB Configuration: TIM3 Channel 1 as alternate function push-pull */
  // Configure the GPIO PB4 for the timer output
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  //Map timer to alternate functions
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM3);
	
	/* Compute the prescaler value */
	PrescalerValue = 0;
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = (105 - 1); // 800kHz
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	/* PWM1 Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);

//  TIM_Cmd(TIM3, ENABLE);                      // Go!!!
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_CtrlPWMOutputs(TIM3, ENABLE);           // enable Timer 3

	/* configure DMA */
	/* DMA clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

	/* DMA1 Channel2 Config */
	DMA_DeInit(DMA1_Stream5);

  // USART TX DMA Channel Config
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&TIM3->CCR2;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)led_dma.buffer;    // this is the buffer memory
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_BufferSize = 0;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
  DMA_InitStructure.DMA_Channel = DMA_Channel_5;
	DMA_Init(DMA1_Stream5, &DMA_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 11;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  DMA_ITConfig(DMA1_Stream5, DMA_IT_TC, ENABLE);
  DMA_ITConfig(DMA1_Stream5, DMA_IT_HT, ENABLE);

	/* TIM3 CC2 DMA Request enable */
	TIM_DMACmd(TIM3, TIM_DMA_CC2, ENABLE);
	
	vSemaphoreCreateBinary(allLedDone);
	
}

static void fillLed(uint16_t *buffer, uint8_t *color)
{
    int i;
    
    for(i=0; i<8; i++) // GREEN data
	{
	    buffer[i] = ((color[1]<<i) & 0x0080) ? TIMING_ONE:TIMING_ZERO;
	}
	for(i=0; i<8; i++) // RED
	{
	    buffer[8+i] = ((color[0]<<i) & 0x0080) ? TIMING_ONE:TIMING_ZERO;
	}
	for(i=0; i<8; i++) // BLUE
	{
	    buffer[16+i] = ((color[2]<<i) & 0x0080) ? TIMING_ONE:TIMING_ZERO;
	}
}

static int current_led = 0;
static int total_led = 0;
static uint8_t (*color_led)[3] = NULL;

void ws2812Send(uint8_t (*color)[3], int len)
{
    int i;
	if(len<1) return;
	
	//Wait for previous transfer to be finished
	xSemaphoreTake(allLedDone, portMAX_DELAY);

	// Set interrupt context ...
	current_led = 0;
	total_led = len;
	color_led = color;
	
    for(i=0; (i<LED_PER_HALF) && (current_led<total_led+2); i++, current_led++) {
        if (current_led<total_led)
            fillLed(led_dma.begin+(24*i), color_led[current_led]);
        else
            bzero(led_dma.begin+(24*i), sizeof(led_dma.begin));
    }
    
    for(i=0; (i<LED_PER_HALF) && (current_led<total_led+2); i++, current_led++) {
        if (current_led<total_led)
            fillLed(led_dma.end+(24*i), color_led[current_led]);
        else
            bzero(led_dma.end+(24*i), sizeof(led_dma.end));
    }
		
	DMA1_Stream5->NDTR = sizeof(led_dma.buffer) / sizeof(led_dma.buffer[0]); // load number of bytes to be transferred
	DMA_Cmd(DMA1_Stream5, ENABLE); 			// enable DMA channel 2
	TIM_Cmd(TIM3, ENABLE);                      // Go!!!
}

void ws2812DmaIsr(void)
{
    portBASE_TYPE xHigherPriorityTaskWoken;
    uint16_t * buffer;
    int i;
    
    if (total_led == 0)
    {
      TIM_Cmd(TIM3, DISABLE);
    	DMA_Cmd(DMA1_Stream5, DISABLE);
    }
    
    if (DMA_GetITStatus(DMA1_Stream5, DMA_IT_HTIF5))
    {
      DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_HTIF5);
      buffer = led_dma.begin;
    }
    
    if (DMA_GetITStatus(DMA1_Stream5, DMA_IT_TCIF5))
    {
      DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF5);
      buffer = led_dma.end;
    }

    for(i=0; (i<LED_PER_HALF) && (current_led<total_led+2); i++, current_led++) {
      if (current_led<total_led)
          fillLed(buffer+(24*i), color_led[current_led]);
      else
          bzero(buffer+(24*i), sizeof(led_dma.end));
    }
    
    if (current_led >= total_led+2) {
      xSemaphoreGiveFromISR(allLedDone, &xHigherPriorityTaskWoken);
	
	    TIM_Cmd(TIM3, DISABLE); 					// disable Timer 3
	    DMA_Cmd(DMA1_Stream5, DISABLE); 			// disable DMA stream4
	    
	    total_led = 0;
    }
}

