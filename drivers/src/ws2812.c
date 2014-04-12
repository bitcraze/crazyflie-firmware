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
#include "stm32f10x_conf.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "led.h"
#include "ledseq.h"

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

static union {
    uint8_t buffer[2*LED_PER_HALF*24];
    struct {
        uint8_t begin[LED_PER_HALF*24];
        uint8_t end[LED_PER_HALF*24];
    } __attribute__((packed));
} led_dma;

void ws2812Init(void)
{
	uint16_t PrescalerValue;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO | RCC_APB2Periph_TIM1, ENABLE);
	/* GPIOA Configuration: TIM3 Channel 1 as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM1, ENABLE);
	
	/* Compute the prescaler value */
	PrescalerValue = (uint16_t) (72000000 / 24000000) - 1;
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 29; // 800kHz 
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	/* PWM1 Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
//	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
//	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
//	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
	TIM_CtrlPWMOutputs(TIM1, ENABLE);           // enable Timer 1
	
	/* configure DMA */
	/* DMA clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	
	/* DMA1 Channel2 Config */
	DMA_DeInit(DMA1_Channel2);

	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&TIM1->CCR1; //TIM1_CCR1_Address;	// physical address of Timer 3 CCR1
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)led_dma.buffer;		// this is the buffer memory 
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;						// data shifted from memory to peripheral
	DMA_InitStructure.DMA_BufferSize = sizeof(led_dma.buffer);
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;					// automatically increase buffer index
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	
	DMA_Init(DMA1_Channel2, &DMA_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 9;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	
    DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE);
    DMA_ITConfig(DMA1_Channel2, DMA_IT_HT, ENABLE);
    
	/* TIM1 CC1 DMA Request enable */
	TIM_DMACmd(TIM1, TIM_DMA_CC1, ENABLE);
	
	vSemaphoreCreateBinary(allLedDone);
	
}

static void fillLed(uint8_t *buffer, uint8_t *color)
{
    int i;
    
    for(i=0; i<8; i++) // GREEN data
	{
	    buffer[i] = ((color[1]<<i) & 0x80)?17:9;
	}
	for(i=0; i<8; i++) // RED
	{
	    buffer[8+i] = ((color[0]<<i) & 0x80)?17:9;
	}
	for(i=0; i<8; i++) // BLUE
	{
	    buffer[16+i] = ((color[2]<<i) & 0x80)?17:9;
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
            bzero(led_dma.begin+(24*i), 24);
    }
    
    for(i=0; (i<LED_PER_HALF) && (current_led<total_led+2); i++, current_led++) {
        if (current_led<total_led)
            fillLed(led_dma.end+(24*i), color_led[current_led]);
        else
            bzero(led_dma.end+(24*i), 24);
    }
		
	DMA1_Channel2->CNDTR = sizeof(led_dma.buffer); // load number of bytes to be transferred
	DMA_Cmd(DMA1_Channel2, ENABLE); 			// enable DMA channel 2
	TIM_Cmd(TIM1, ENABLE);                      // Go!!!
}

void ws2812DmaIsr(void)
{
    portBASE_TYPE xHigherPriorityTaskWoken;
    uint8_t * buffer;
    int i;
    
    if (total_led == 0)
    {
        TIM_Cmd(TIM1, DISABLE);
    	DMA_Cmd(DMA1_Channel2, DISABLE);
    }
    
    if (DMA_GetITStatus(DMA1_IT_HT2))
    {
        DMA_ClearITPendingBit(DMA1_IT_HT2);
        buffer = led_dma.begin;
    }
    
    if (DMA_GetITStatus(DMA1_IT_TC2))
    {
        DMA_ClearITPendingBit(DMA1_IT_TC2);
        buffer = led_dma.end;
    }

    for(i=0; (i<LED_PER_HALF) && (current_led<total_led+2); i++, current_led++) {
        if (current_led<total_led)
            fillLed(buffer+(24*i), color_led[current_led]);
        else
            bzero(buffer+(24*i), 24);
    }
    
    if (current_led >= total_led+2) {
        xSemaphoreGiveFromISR(allLedDone, &xHigherPriorityTaskWoken);
	
	    TIM_Cmd(TIM1, DISABLE); 					// disable Timer 1
	    DMA_Cmd(DMA1_Channel2, DISABLE); 			// disable DMA channel 2
	    
	    total_led = 0;
    }
}

