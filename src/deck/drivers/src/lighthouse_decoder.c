/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2017 Bitcraze AB
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
 * cppm.c - Combined PPM / PPM-Sum driver
 */
#define DEBUG_MODULE  "LHPULSE"

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "stm32fxxx.h"

#include "system.h"
#include "deck.h"
#include "nvicconf.h"
#include "lighthouse_pulse_processor.h"
#include "lighthouseGeometry.h"
#include "debug.h"
#include "log.h"

#include "estimator_kalman.h"


#define LHPULSE_TIMER                   TIM5
#define LHPULSE_TIMER_RCC               RCC_APB1Periph_TIM5
#define LHPULSE_GPIO_RCC                RCC_AHB1Periph_GPIOA
#define LHPULSE_GPIO_PORT               GPIOA

#define LHPULSE_RIGHT_GPIO_PIN                GPIO_Pin_2      // TIM5_CH3
#define LHPULSE_RIGHT_GPIO_SOURCE             GPIO_PinSource2

#define LHPULSE_LEFT_GPIO_PIN                GPIO_Pin_3      // TIM5_CH4
#define LHPULSE_LEFT_GPIO_SOURCE             GPIO_PinSource3

#define LHPULSE_GPIO_AF                 GPIO_AF_TIM5

#define LHPULSE_TIM_PRESCALER           (0) // TIM14 clock running at sysclk/2. Gives us 84MHz

static xQueueHandle pulsesQueueRight;
static xQueueHandle pulsesQueueLeft;

LhObj       lhObjLeft;
LhPulseType lhPulseLeft;

static void lhTask(void *param);
int lhGetPulseRight(LhPulseType *pulse);
int lhGetPulseLeft(LhPulseType *pulse);


void lhInit(DeckInfo* info)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_ICInitTypeDef  TIM_ICInitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_AHB1PeriphClockCmd(LHPULSE_GPIO_RCC, ENABLE);
  RCC_APB1PeriphClockCmd(LHPULSE_TIMER_RCC, ENABLE);

  // Configure the GPIO for the timer input
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = LHPULSE_RIGHT_GPIO_PIN;
  GPIO_Init(LHPULSE_GPIO_PORT, &GPIO_InitStructure);

  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = LHPULSE_LEFT_GPIO_PIN;
  GPIO_Init(LHPULSE_GPIO_PORT, &GPIO_InitStructure);

  GPIO_PinAFConfig(LHPULSE_GPIO_PORT, LHPULSE_RIGHT_GPIO_SOURCE, LHPULSE_GPIO_AF);
  GPIO_PinAFConfig(LHPULSE_GPIO_PORT, LHPULSE_LEFT_GPIO_SOURCE, LHPULSE_GPIO_AF);

  // Time base configuration. Count at 84MHz.
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Prescaler = LHPULSE_TIM_PRESCALER;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(LHPULSE_TIMER, &TIM_TimeBaseStructure);

  pulsesQueueRight = xQueueCreate(10, sizeof(LhPulseType));
  pulsesQueueLeft = xQueueCreate(10, sizeof(LhPulseType));

  // Setup input capture to measure pulse width
  // RIGHT
  // This activates XOR to route channel 3 to channel 1
  TIM_SelectHallSensor(LHPULSE_TIMER, ENABLE);

  TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInit(LHPULSE_TIMER, &TIM_ICInitStructure);

  TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_IndirectTI;
  TIM_ICInit(LHPULSE_TIMER, &TIM_ICInitStructure);

  // LEFT
  TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_IndirectTI;
  TIM_ICInit(LHPULSE_TIMER, &TIM_ICInitStructure);

  TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInit(LHPULSE_TIMER, &TIM_ICInitStructure);

  // Enable interrupts
  NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_MID_PRI;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  TIM_ITConfig(LHPULSE_TIMER, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4, ENABLE);
  TIM_Cmd(LHPULSE_TIMER, ENABLE);

  xTaskCreate(lhTask, "LHTASK", 2*configMINIMAL_STACK_SIZE, NULL, /*priority*/3, NULL);
}

#define LH_TEST

#ifdef LH_TEST
LhPulseType pulses[18] = {
    {175243,  382}, {479637,  9693}, {514749,  7965},  {815882,  368}, {1180949, 7065}, {1216055, 8831},
    {1546318, 458}, {1882115, 6185}, {1917209, 11456}, {2152833, 384}, {2583431, 8844}, {2618527, 5334},
    {2980187, 384}, {3284600, 9691}, {3319677, 6177},  {3620805, 367}, {3985899, 5345}, {4021009, 10582}
};

// My base baseStations
static baseStationGeometry_t baseStations[2] = {
  {.origin = {1.576887, 2.448647, -1.601387}, .mat = {{-0.634552, -0.280564, 0.720158}, {-0.016947, 0.936612, 0.349959}, {-0.772695, 0.209863, -0.599083}}},
  {.origin = {-1.828970, 2.628525, 1.256474}, .mat = {{0.482050, 0.446738, -0.753693}, {-0.005340, 0.861721, 0.507355}, {0.876127, -0.240546, 0.417778}}},
};

static vec3d position = {};
static float delta = 0;

static positionMeasurement_t ext_pos;

static void lhTask(void *param)
{
  systemWaitStart();

  for (int i=0; i < 18; i++)
  {
    lhppAnalysePulse(&lhObjLeft, &pulses[i]);
  }

  while(1) {
    lhGetPulseRight(&lhPulseLeft);
    if (lhppAnalysePulse(&lhObjLeft, &lhPulseLeft) == true)
    {
      float angles[4] = {lhObjLeft.angles.x0, lhObjLeft.angles.y0, lhObjLeft.angles.x1, lhObjLeft.angles.y1};
      lhgeometryGetPosition(baseStations, angles, position, &delta);

      ext_pos.x = position[0];
      ext_pos.y = -position[2];
      ext_pos.z = position[1];
      ext_pos.stdDev = 0.01;
      estimatorKalmanEnqueuePosition(&ext_pos);
    }
  }
}
#else
static void lhTask(void *param)
{
  systemWaitStart();

  while(1) {
    lhGetPulseRight(&lhPulseLeft);
    lhppAnalysePulse(&lhObjLeft, &lhPulseLeft);
  }
}
#endif


int lhGetPulseRight(LhPulseType *pulse)
{
   ASSERT(pulse);

   return xQueueReceive(pulsesQueueRight, pulse, portMAX_DELAY);
}

int lhGetPulseLeft(LhPulseType *pulse)
{
   ASSERT(pulse);

   return xQueueReceive(pulsesQueueLeft, pulse, portMAX_DELAY);
}

static LhPulseType pulseRight;
static LhPulseType pulseLeft;

static LhPulseType buffR[1000];
static LhPulseType buffL[1000];
static int pR=0, pL=0;

void __attribute__((used)) TIM5_IRQHandler()
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  if (TIM_GetITStatus(LHPULSE_TIMER, TIM_IT_CC1) != RESET)
  {
    if (pR < 1000-1) pR++;
    pulseRight.tsRise = TIM_GetCapture1(LHPULSE_TIMER);
    buffR[pR].tsRise = pulseRight.tsRise;
    TIM_ClearITPendingBit(LHPULSE_TIMER, TIM_IT_CC1);
  }

  if (TIM_GetITStatus(LHPULSE_TIMER, TIM_IT_CC2) != RESET)
  {
    pulseRight.width = TIM_GetCapture2(LHPULSE_TIMER) -  pulseRight.tsRise;
    buffR[pR].width = pulseRight.width;
    xQueueSendFromISR(pulsesQueueRight, &pulseRight, &xHigherPriorityTaskWoken);

    TIM_ClearITPendingBit(LHPULSE_TIMER, TIM_IT_CC2);
  }


  if (TIM_GetITStatus(LHPULSE_TIMER, TIM_IT_CC3) != RESET)
  {
    if (pL < 1000-1) pL++;
    pulseLeft.tsRise = TIM_GetCapture3(LHPULSE_TIMER);
    buffL[pL].tsRise = pulseLeft.tsRise;

    TIM_ClearITPendingBit(LHPULSE_TIMER, TIM_IT_CC3);
  }

  if (TIM_GetITStatus(LHPULSE_TIMER, TIM_IT_CC4) != RESET)
  {
    pulseLeft.width = TIM_GetCapture4(LHPULSE_TIMER) - pulseLeft.tsRise;
    buffL[pL].width = pulseLeft.width;
    xQueueSendFromISR(pulsesQueueLeft, &pulseLeft, &xHigherPriorityTaskWoken);

    TIM_ClearITPendingBit(LHPULSE_TIMER, TIM_IT_CC4);
  }

}

static const DeckDriver lighthouse_deck = {
  .vid = 0,
  .pid = 0,
  .name = "bcLH",

  .usedGpio = 0,  // FIXME: set the used pins
  .requiredEstimator = kalmanEstimator,

  .init = lhInit,
};

DECK_DRIVER(lighthouse_deck);

LOG_GROUP_START(lhpulse)
LOG_ADD(LOG_UINT32, widthRight, &pulseRight.width)
LOG_ADD(LOG_UINT32, widthLeft, &pulseLeft.width)
LOG_GROUP_STOP(lhpulse)

LOG_GROUP_START(lighthouse)
LOG_ADD(LOG_FLOAT, anglex0, &lhObjLeft.angles.x0)
LOG_ADD(LOG_FLOAT, angley0, &lhObjLeft.angles.y0)
LOG_ADD(LOG_FLOAT, anglex1, &lhObjLeft.angles.x1)
LOG_ADD(LOG_FLOAT, angley1, &lhObjLeft.angles.y1)
LOG_ADD(LOG_FLOAT, positionX, &position[0])
LOG_ADD(LOG_FLOAT, positionY, &position[1])
LOG_ADD(LOG_FLOAT, positionZ, &position[2])
LOG_ADD(LOG_FLOAT, delta, &delta)
LOG_GROUP_STOP(lighthouse)
