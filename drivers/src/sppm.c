/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2013 Bitcraze AB
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
 *
 * sppm.c - PPM sum driver
 */

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "stm32fxxx.h"
#include "sppm.h"
#include "nvicconf.h"
#include "commander.h"

#define DEBUG_MODULE  "SPPM"
#include "debug.h"
#include "log.h"


#define SPPM_TIMER                   TIM14
#define SPPM_TIMER_RCC               RCC_APB1Periph_TIM14
#define SPPM_TIMER_CH_Init           TIM_OC1Init
#define SPPM_TIMER_CH_PreloadConfig  TIM_OC1PreloadConfig
#define SPPM_TIMER_CH_SetCompare     TIM_SetCompare1
#define SPPM_GPIO_RCC                RCC_AHB1Periph_GPIOA
#define SPPM_GPIO_PORT               GPIOA
#define SPPM_GPIO_PIN                GPIO_Pin_7
#define SPPM_GPIO_SOURCE             GPIO_PinSource7
#define SPPM_GPIO_AF                 GPIO_AF_TIM14

#define SPPM_NR_CHANNELS             4

static xQueueHandle captureQueue;
static struct CommanderCrtpValues commanderPacket;
static uint16_t prevCapureVal;
static bool captureFlag;
static bool isAvailible;

static uint16_t ch[SPPM_NR_CHANNELS];

static void sppmTask(void *param);

void sppmInit(void)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_ICInitTypeDef  TIM_ICInitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_AHB1PeriphClockCmd(SPPM_GPIO_RCC, ENABLE);
  RCC_APB1PeriphClockCmd(SPPM_TIMER_RCC, ENABLE);

  // Configure the GPIO for the timer input
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = SPPM_GPIO_PIN;
  GPIO_Init(SPPM_GPIO_PORT, &GPIO_InitStructure);

  GPIO_PinAFConfig(SPPM_GPIO_PORT, SPPM_GPIO_SOURCE, SPPM_GPIO_AF);

  // Time base configuration. 1us tick.
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(SPPM_TIMER, &TIM_TimeBaseStructure);

  // Setup input capture using default config.
  TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInit(SPPM_TIMER, &TIM_ICInitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = TIM8_TRG_COM_TIM14_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 14;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  captureQueue = xQueueCreate(50, sizeof(uint16_t));

  xTaskCreate(sppmTask, (const signed char * const)SPPM_TASK_NAME,
              SPPM_TASK_STACKSIZE, NULL, SPPM_TASK_PRI, NULL);

  TIM_Cmd(SPPM_TIMER, ENABLE);
  TIM_ITConfig(SPPM_TIMER, TIM_IT_Update | TIM_IT_CC1, ENABLE);
}

bool sppmIsAvailible(void)
{
  return isAvailible;
}

int sppmGetTimestamp(uint16_t *timestamp)
{
  ASSERT(timestamp);

  return xQueueReceive(captureQueue, timestamp, portMAX_DELAY);
}

float sppmConvert2Float(uint16_t timestamp, float min, float max)
{
  float scale = (float)(timestamp - 1000) / 1000.0f;

  return min + ((max - min) * scale);
}

uint16_t sppmConvert2uint16(uint16_t timestamp)
{
  uint16_t base = (timestamp - 1000);

  return base * 65;
}


static void sppmTask(void *param)
{
  uint16_t ppm;
  uint8_t currChannel = 0;

  while (true)
  {
    if (sppmGetTimestamp(&ppm) == pdTRUE)
    {
      if (sppmIsAvailible() &&  ppm < 2100)
      {
        switch (currChannel)
        {
          case 0:
            ch[0] = ppm;
            commanderPacket.thrust = sppmConvert2uint16(ppm);
            break;
          case 1:
            ch[1] = ppm;
            commanderPacket.roll = sppmConvert2Float(ppm, -40.0f, 40.0f);
            break;
          case 2:
            ch[2] = ppm;
             commanderPacket.pitch = sppmConvert2Float(ppm, -40.0f, 40.0f);
            break;
          case 3:
            ch[3] = ppm;
            commanderPacket.yaw = sppmConvert2Float(ppm, -400.0f, 400.0f);
            commanderSet(&commanderPacket);
            break;
          default:
            break;
        }
        currChannel++;
      }
      else
      {
        currChannel = 0;
      }
    }
  }
}


void __attribute__((used)) TIM8_TRG_COM_TIM14_IRQHandler()
{
  uint16_t capureVal;
  uint16_t capureValDiff;
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  if (TIM_GetITStatus(SPPM_TIMER, TIM_IT_CC1) != RESET)
  {
    if (TIM_GetFlagStatus(SPPM_TIMER, TIM_FLAG_CC1OF) != RESET)
    {
      //TODO: Handle overflow error
    }

    capureVal = TIM_GetCapture1(SPPM_TIMER);
    capureValDiff = capureVal - prevCapureVal;
    prevCapureVal = capureVal;

    xQueueSendFromISR(captureQueue, &capureValDiff, &xHigherPriorityTaskWoken);

    captureFlag = true;
    TIM_ClearITPendingBit(SPPM_TIMER, TIM_IT_CC1);
  }

  if (TIM_GetITStatus(SPPM_TIMER, TIM_IT_Update) != RESET)
  {
    // Update input status
    isAvailible = (captureFlag == true);
    captureFlag = false;
    TIM_ClearITPendingBit(SPPM_TIMER, TIM_IT_Update);
  }
}

#define ENABLE_SPPM_LOG
/* Loggable variables */
#ifdef ENABLE_SPPM_LOG
LOG_GROUP_START(sppm)
LOG_ADD(LOG_UINT16, ch0, &ch[0])
LOG_ADD(LOG_UINT16, ch1, &ch[1])
LOG_ADD(LOG_UINT16, ch2, &ch[2])
LOG_ADD(LOG_UINT16, ch3, &ch[3])
LOG_ADD(LOG_UINT16, thrust, &commanderPacket.thrust)
LOG_ADD(LOG_FLOAT, roll, &commanderPacket.roll)
LOG_ADD(LOG_FLOAT, pitch, &commanderPacket.pitch)
LOG_ADD(LOG_FLOAT, yaw, &commanderPacket.yaw)
LOG_GROUP_STOP(sppm)
#endif

