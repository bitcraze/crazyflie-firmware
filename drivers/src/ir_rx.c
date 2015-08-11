
#include <stm32f10x.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_tim.h>
#include <stm32f10x_exti.h>
#include <misc.h>
#include <stdint.h>
#include <stddef.h>
#include "ir_rx.h"
#include "debug.h"
#include "platform_config.h"

void on_exti1_irq();
void on_tim2_irq();
void _ir_rx_process_buffer();

#define IR_RX_COUNT 10

IrRecv irRecvs[IR_RX_COUNT];
volatile uint16_t irRecvReadIndex;
volatile uint16_t irRecvWriteIndex;
volatile uint16_t irRecvAvailable;

#define TIM_PRESCALER  (72 * 2)
#define TIM_PERIOD     0xfffe
#define US_PER_TIM     1

void ir_rx_setup() {
  GPIO_InitTypeDef gpioConfig;
  NVIC_InitTypeDef nvicInit;
  EXTI_InitTypeDef extiInit;
  TIM_TimeBaseInitTypeDef timeBaseInit;

  debug_write_line("?BEGIN ir_rx_setup");

  irRecvAvailable = 0;
  irRecvReadIndex = 0;
  irRecvWriteIndex = 0;
  irRecvs[irRecvWriteIndex].bufferLength = -1;

  RCC_APB2PeriphClockCmd(IR_RX_RCC, ENABLE);
  gpioConfig.GPIO_Pin = IR_RX_PIN;
  gpioConfig.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  gpioConfig.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(IR_RX_PORT, &gpioConfig);

  // enable EXTI
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);

  EXTI_StructInit(&extiInit);
  extiInit.EXTI_Line = EXTI_Line0;
  extiInit.EXTI_Mode = EXTI_Mode_Interrupt;
  extiInit.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  extiInit.EXTI_LineCmd = ENABLE;
  EXTI_Init(&extiInit);

  nvicInit.NVIC_IRQChannel = EXTI0_IRQn;
  nvicInit.NVIC_IRQChannelPreemptionPriority = 0;
  nvicInit.NVIC_IRQChannelSubPriority = 0;
  nvicInit.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvicInit);

  // setup timer
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  debug_write("?ir_rx timer period: ");
  debug_write_u32(TIM_PERIOD, 10);
  debug_write_line("");
  debug_write("?ir_rx timer us: ");
  debug_write_u32(US_PER_TIM, 10);
  debug_write_line("");

  TIM_TimeBaseStructInit(&timeBaseInit);
  timeBaseInit.TIM_Period = TIM_PERIOD;
  timeBaseInit.TIM_Prescaler = TIM_PRESCALER;
  timeBaseInit.TIM_ClockDivision = 0;
  timeBaseInit.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(IR_RX_TIMER, &timeBaseInit);

  nvicInit.NVIC_IRQChannel = TIM2_IRQn;
  nvicInit.NVIC_IRQChannelPreemptionPriority = 1;
  nvicInit.NVIC_IRQChannelSubPriority = 0;
  nvicInit.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvicInit);

  TIM_ITConfig(IR_RX_TIMER, TIM_IT_Update, ENABLE);
  TIM_SetCounter(IR_RX_TIMER, 0);
  TIM_Cmd(IR_RX_TIMER, ENABLE);

  debug_write_line("?END ir_rx_setup");
}

IrRecv* ir_rx_recv() {
  if(irRecvAvailable == 0) {
    return NULL;
  }
  IrRecv* result = &irRecvs[irRecvReadIndex];
  irRecvAvailable--;
  irRecvReadIndex = (irRecvReadIndex + 1) % IR_RX_COUNT;
  return result;
}

void _ir_rx_process_buffer() {
  irRecvWriteIndex = (irRecvWriteIndex + 1) % IR_RX_COUNT;
  irRecvAvailable++;
  irRecvs[irRecvWriteIndex].bufferLength = -1;
}

void on_exti0_irq() {
  if(EXTI_GetITStatus(EXTI_Line0) != RESET) {
    IrRecv* rx = &irRecvs[irRecvWriteIndex];

    // skip the first signal change, this is the start of the signal and should not be recorded
    if(rx->bufferLength < 0) {
      rx->bufferLength++;
    } else {
      rx->buffer[rx->bufferLength] = TIM_GetCounter(TIM2) * 2;
      if(rx->buffer[rx->bufferLength] > 10000 || rx->bufferLength == (IR_RX_CAPTURE_BUFFER_MAX_LEN - 1)) {
        rx->bufferLength++;
        _ir_rx_process_buffer();
        irRecvs[irRecvWriteIndex].bufferLength = 0;
      } else {
        rx->bufferLength++;
      }
      debug_led_set(1);
    }

    TIM_SetCounter(TIM2, 0);
    EXTI_ClearITPendingBit(EXTI_Line0);
  }
}

void on_tim2_irq() {
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
    debug_led_set(0);
    if(irRecvs[irRecvWriteIndex].bufferLength > 1) {
      _ir_rx_process_buffer();
    }
    irRecvs[irRecvWriteIndex].bufferLength = -1;
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
  }
}

