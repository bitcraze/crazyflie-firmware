
#include <stm32fxxx.h>
#include <stdint.h>
#include <stddef.h>

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "ir_rx.h"
#include "usec_time.h"
#include "debug.h"
#include "nvicconf.h"


#define IR_RX_RCC                   RCC_AHB1Periph_GPIOA
#define IR_RX_RCC_GPIO_CMD          RCC_AHB1PeriphClockCmd
#define IR_RX_PORT                  GPIOA
#define IR_RX_PIN                   GPIO_Pin_7
#define IR_RX_EXTI_IRQn             EXTI9_5_IRQn
#define IR_RX_EXTI_LINE             EXTI_Line7
#define IR_RX_EXTI_IRQ_HANDLER      EXTI9_5_IRQHandler

#define IR_RX_COUNT 5

IrRecv irRecvs[IR_RX_COUNT];
volatile uint16_t irRecvReadIndex;
volatile uint16_t irRecvWriteIndex;
volatile uint16_t irRecvAvailable;
volatile uint64_t prevTimestamp;

static xQueueHandle newTimediffQueue;

static void irRxProcessBuffer();

void irRxTask(void *arg)
{
  uint16_t timediff;
  IrRecv* rx;

  while (true)
  {
    rx = &irRecvs[irRecvWriteIndex];

    if (xQueueReceive(newTimediffQueue, &timediff, M2T(10)) == pdTRUE)
    {
      // skip the first signal change, this is the start of the signal and should not be recorded
      if (rx->bufferLength < 0)
      {
        rx->bufferLength++;
      }
      else
      {
        // Record pulse length
        rx->buffer[rx->bufferLength] = timediff;
        rx->bufferLength++;

        if (rx->bufferLength == (IR_RX_CAPTURE_BUFFER_MAX_LEN - 1))
        {
          irRxProcessBuffer();
//          irRecvs[irRecvWriteIndex].bufferLength = 0;
        }
      }
    }
    else
    {
      if (rx->bufferLength > 0)
      {
        irRxProcessBuffer();
      }
    }
  }
}

void irRxInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef nvicInit;
  EXTI_InitTypeDef extiInit;

  irRecvAvailable = 0;
  irRecvReadIndex = 0;
  irRecvWriteIndex = 0;
  irRecvs[irRecvWriteIndex].bufferLength = -1;

  IR_RX_RCC_GPIO_CMD(IR_RX_RCC, ENABLE);
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = IR_RX_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_Init(IR_RX_PORT, &GPIO_InitStructure);

  EXTI_StructInit(&extiInit);
  extiInit.EXTI_Line = IR_RX_EXTI_LINE;
  extiInit.EXTI_Mode = EXTI_Mode_Interrupt;
  extiInit.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  extiInit.EXTI_LineCmd = ENABLE;
  EXTI_Init(&extiInit);

  nvicInit.NVIC_IRQChannel = IR_RX_EXTI_IRQn;
  nvicInit.NVIC_IRQChannelPreemptionPriority = NVIC_IR_RX_EXTI_PRI;
  nvicInit.NVIC_IRQChannelSubPriority = 0;
  nvicInit.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvicInit);

  NVIC_EnableIRQ(IR_RX_EXTI_IRQn);

  newTimediffQueue = xQueueCreate(10, sizeof(uint16_t));

  xTaskCreate(irRxTask, (const signed char * const)"IR-RX", 50, NULL, 0, NULL);
  prevTimestamp = usecGetTimestamp();
}

IrRecv* irRxReceive()
{
  if (irRecvAvailable == 0)
  {
    return NULL;
  }
  IrRecv* result = &irRecvs[irRecvReadIndex];
  irRecvAvailable--;
  irRecvReadIndex = (irRecvReadIndex + 1) % IR_RX_COUNT;
  return result;
}

static void irRxProcessBuffer()
{
  irRecvWriteIndex = (irRecvWriteIndex + 1) % IR_RX_COUNT;
  irRecvAvailable++;
  irRecvs[irRecvWriteIndex].bufferLength = -1;
}
/**
 * Edge triggered interrupt that takes a us timestamp
 * for every edge and puts it on a receive queue.
 */
void __attribute__((used)) IR_RX_EXTI_IRQ_HANDLER(void)
{
  uint64_t timestamp = usecGetTimestamp();

  if (EXTI_GetITStatus(IR_RX_EXTI_LINE) != RESET)
  {
    uint16_t timediff;
    portBASE_TYPE xHigherPriorityTaskWoken;

    if (timestamp - prevTimestamp < 0xFFFF)
    {
      timediff = (uint16_t)(timestamp - prevTimestamp);
    }
    else
    {
      timediff = 0xFFFF;
    }

    xQueueSendFromISR(newTimediffQueue, &timediff, &xHigherPriorityTaskWoken);

    prevTimestamp = timestamp;
    EXTI_ClearITPendingBit(IR_RX_EXTI_LINE);
  }
}
