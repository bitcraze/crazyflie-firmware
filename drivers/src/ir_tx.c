
#include <stdint.h>
#include <stm32fxxx.h>
#include "ir_tx.h"
#include "debug.h"

#define IR_TX_CARRIER_FREQ          56000
#define IR_TX_CARRIER_PWM_PERIOD    (SystemCoreClock / IR_TX_CARRIER_FREQ)
#define IR_TX_DELAY_PRESCALER       (84 - 1)

#define IR_RX_RCC                   RCC_AHB1Periph_GPIOA
#define IR_RX_PORT                  GPIOA
#define IR_RX_PIN                   GPIO_Pin_7

#define IR_RX_TIMER                 TIM14

#define IR_TX_CARRIER_TIMER                   TIM10
#define IR_TX_CARRIER_TIMER_RCC               RCC_APB2Periph_TIM10
#define IR_TX_CARRIER_TIMER_CH_Init           TIM_OC1Init
#define IR_TX_CARRIER_TIMER_CH_PreloadConfig  TIM_OC1PreloadConfig
#define IR_TX_CARRIER_TIMER_CH_SetCompare     TIM_SetCompare1
#define IR_TX_CARRIER_RCC                     RCC_AHB1Periph_GPIOB
#define IR_TX_CARRIER_PORT                    GPIOB
#define IR_TX_CARRIER_PIN                     GPIO_Pin_8

#define IR_TX_DELAY_TIMER                     TIM6
#define IR_TX_DELAY_TIMER_IRQ                 TIM6_DAC_IRQn
#define IR_TX_DELAY_TIMER_RCC                 RCC_APB1Periph_TIM6

volatile uint16_t g_tx_bufferIndex;
volatile uint16_t g_tx_repeatCount;
volatile IrCode* g_tx_code;

void _ir_tx_on();
void _ir_tx_off();

void ir_tx_setup() {
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_AHB1PeriphClockCmd(IR_TX_CARRIER_RCC, ENABLE);
  RCC_APB2PeriphClockCmd(IR_TX_CARRIER_TIMER_RCC, ENABLE);

//  RCC_APB2PeriphClockCmd(IR_TX_RCC, ENABLE);
//  gpioInitStructure.GPIO_Pin = IR_TX_PIN;
//  gpioInitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//  gpioInitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_Init(IR_TX_PORT, &gpioInitStructure);

  // Configure the GPIO for the timer output
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Pin = IR_TX_CARRIER_PIN;
  GPIO_Init(IR_TX_CARRIER_PORT, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM10);

  // Time base configuration
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Period = IR_TX_CARRIER_PWM_PERIOD;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(IR_TX_CARRIER_TIMER, &TIM_TimeBaseStructure);

  // PWM channel configuration
  TIM_OCStructInit(&TIM_OCInitStructure);
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
  // Configure Output Compare for PWM
  IR_TX_CARRIER_TIMER_CH_Init(IR_TX_CARRIER_TIMER, &TIM_OCInitStructure);
  IR_TX_CARRIER_TIMER_CH_PreloadConfig(IR_TX_CARRIER_TIMER, TIM_OCPreload_Enable);

//  TIM_SelectOnePulseMode(IR_TX_CARRIER_TIMER, TIM_OPMode_Repetitive);
  TIM_ARRPreloadConfig(IR_TX_CARRIER_TIMER, ENABLE);
  IR_TX_CARRIER_TIMER_CH_SetCompare(IR_TX_CARRIER_TIMER, IR_TX_CARRIER_PWM_PERIOD / 2);
  TIM_Cmd(IR_TX_CARRIER_TIMER, ENABLE);

  // Delay timer
  RCC_APB2PeriphClockCmd(IR_TX_DELAY_TIMER_RCC, ENABLE);

  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Period = 0xffff;
  TIM_TimeBaseStructure.TIM_Prescaler = IR_TX_DELAY_PRESCALER;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(IR_TX_DELAY_TIMER, &TIM_TimeBaseStructure);

  NVIC_InitStructure.NVIC_IRQChannel = IR_TX_DELAY_TIMER_IRQ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  TIM_ITConfig(IR_TX_DELAY_TIMER, TIM_IT_Update, ENABLE);

  _ir_tx_off();

}

void ir_tx_send(IrCode* code)
{
  g_tx_bufferIndex = 0;
  g_tx_repeatCount = code->repeatCount;
  g_tx_code = code;

  TIM_SetCounter(IR_TX_DELAY_TIMER, 1);
  TIM_SetAutoreload(IR_TX_DELAY_TIMER, g_tx_code->code[g_tx_bufferIndex++]);
  _ir_tx_on();

  TIM_Cmd(IR_TX_DELAY_TIMER, ENABLE);
}

void on_tim1_irq()
{
  if (TIM_GetITStatus(IR_TX_DELAY_TIMER, TIM_IT_Update) != RESET)
  {
    if((g_tx_bufferIndex % 2) == 0)
    {
      _ir_tx_on();
    }
    else
    {
      _ir_tx_off();
    }

    if(g_tx_bufferIndex < g_tx_code->codeLength - 1)
    {
      TIM_SetCounter(IR_TX_DELAY_TIMER, 1);
      TIM_SetAutoreload(IR_TX_DELAY_TIMER, g_tx_code->code[g_tx_bufferIndex++]);
    }
    else
    {
      g_tx_repeatCount--;
      _ir_tx_off();
      if(g_tx_repeatCount == 0)
      {
        TIM_Cmd(IR_TX_DELAY_TIMER, DISABLE);
      } else
      {
        g_tx_bufferIndex = 0;
        TIM_SetCounter(IR_TX_DELAY_TIMER, 1);
        TIM_SetAutoreload(IR_TX_DELAY_TIMER, g_tx_code->gap);
      }
    }

    TIM_ClearITPendingBit(IR_TX_DELAY_TIMER, TIM_IT_Update);
  }
}

void _ir_tx_on()
{
  TIM_CCxCmd(IR_TX_CARRIER_TIMER, TIM_Channel_1, ENABLE);
}

void _ir_tx_off()
{
  TIM_CCxCmd(IR_TX_CARRIER_TIMER, TIM_Channel_1, DISABLE);
}
