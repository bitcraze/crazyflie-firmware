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
 * motors.c - Motor driver
 *
 * This code mainly interfacing the PWM peripheral lib of ST.
 */

#include <stdbool.h>

#include "motors.h"

// ST lib includes
#include "stm32f10x_conf.h"

//FreeRTOS includes
#include "FreeRTOS.h"
#include "task.h"

// HW defines
#define MOTORS_GPIO_TIM_PERIF     RCC_APB1Periph_TIM3
#define MOTORS_GPIO_TIM_M1_2      TIM3
#define MOTORS_GPIO_TIM_M1_2_DBG  DBGMCU_TIM3_STOP
#define MOTORS_REMAP              GPIO_PartialRemap_TIM3

#define MOTORS_GPIO_TIM_M3_4_PERIF  RCC_APB1Periph_TIM4
#define MOTORS_GPIO_TIM_M3_4        TIM4
#define MOTORS_GPIO_TIM_M3_4_DBG    DBGMCU_TIM4_STOP

#define MOTORS_GPIO_PERIF         RCC_APB2Periph_GPIOB
#define MOTORS_GPIO_PORT          GPIOB
#define MOTORS_GPIO_M1            GPIO_Pin_1 // T3_CH4
#define MOTORS_GPIO_M2            GPIO_Pin_0 // T3_CH3
#define MOTORS_GPIO_M3            GPIO_Pin_9 // T4_CH4
#define MOTORS_GPIO_M4            GPIO_Pin_8 // T4_CH3

/* Utils Conversion macro */
#ifdef BRUSHLESS_MOTORCONTROLLER
  #define C_BITS_TO_16(X) (0xFFFF * (X - MOTORS_PWM_CNT_FOR_1MS) / MOTORS_PWM_CNT_FOR_1MS)
  #define C_16_TO_BITS(X) (MOTORS_PWM_CNT_FOR_1MS + ((X * MOTORS_PWM_CNT_FOR_1MS) / 0xFFFF))
#else
  #define C_BITS_TO_16(X) ((X)<<(16-MOTORS_PWM_BITS))
  #define C_16_TO_BITS(X) ((X)>>(16-MOTORS_PWM_BITS)&((1<<MOTORS_PWM_BITS)-1))
#endif

const int MOTORS[] = { MOTOR_M1, MOTOR_M2, MOTOR_M3, MOTOR_M4 };
static bool isInit = false;

/* Public functions */

//Initialization. Will set all motors ratio to 0%
void motorsInit()
{
  if (isInit)
    return;

  //Init structures
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  //Enable gpio and the timer
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | MOTORS_GPIO_PERIF, ENABLE);
  RCC_APB1PeriphClockCmd(MOTORS_GPIO_TIM_PERIF | MOTORS_GPIO_TIM_M3_4_PERIF, ENABLE);
  // Configure the GPIO for the timer output
  GPIO_InitStructure.GPIO_Pin = (MOTORS_GPIO_M1 |
                                 MOTORS_GPIO_M2 |
                                 MOTORS_GPIO_M3 |
                                 MOTORS_GPIO_M4);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(MOTORS_GPIO_PORT, &GPIO_InitStructure);

  //Remap M2-4
  GPIO_PinRemapConfig(MOTORS_REMAP , ENABLE);

  //Timer configuration
  TIM_TimeBaseStructure.TIM_Period = MOTORS_PWM_PERIOD;
  TIM_TimeBaseStructure.TIM_Prescaler = MOTORS_PWM_PRESCALE;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(MOTORS_GPIO_TIM_M1_2, &TIM_TimeBaseStructure);

  TIM_TimeBaseStructure.TIM_Period = MOTORS_PWM_PERIOD;
  TIM_TimeBaseStructure.TIM_Prescaler = MOTORS_PWM_PRESCALE;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(MOTORS_GPIO_TIM_M3_4, &TIM_TimeBaseStructure);

  //PWM channels configuration (All identical!)
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = MOTORS_POLARITY;

  TIM_OC3Init(MOTORS_GPIO_TIM_M3_4, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(MOTORS_GPIO_TIM_M3_4, TIM_OCPreload_Enable);

  TIM_OC4Init(MOTORS_GPIO_TIM_M3_4, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(MOTORS_GPIO_TIM_M3_4, TIM_OCPreload_Enable);

  TIM_OC3Init(MOTORS_GPIO_TIM_M1_2, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(MOTORS_GPIO_TIM_M1_2, TIM_OCPreload_Enable);

  TIM_OC4Init(MOTORS_GPIO_TIM_M1_2, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(MOTORS_GPIO_TIM_M1_2, TIM_OCPreload_Enable);

  //Enable the timer
  TIM_Cmd(MOTORS_GPIO_TIM_M1_2, ENABLE);
  TIM_Cmd(MOTORS_GPIO_TIM_M3_4, ENABLE);
  //Enable the timer PWM outputs
  TIM_CtrlPWMOutputs(MOTORS_GPIO_TIM_M1_2, ENABLE);
  TIM_CtrlPWMOutputs(MOTORS_GPIO_TIM_M3_4, ENABLE);
  // Halt timer during debug halt.
  DBGMCU_Config(MOTORS_GPIO_TIM_M1_2_DBG, ENABLE);
  DBGMCU_Config(MOTORS_GPIO_TIM_M3_4_DBG, ENABLE);
  
  isInit = true;
}

bool motorsTest(void)
{
#ifndef BRUSHLESS_MOTORCONTROLLER
  int i;

  for (i = 0; i < sizeof(MOTORS) / sizeof(*MOTORS); i++)
  {
    motorsSetRatio(MOTORS[i], MOTORS_TEST_RATIO);
    vTaskDelay(M2T(MOTORS_TEST_ON_TIME_MS));
    motorsSetRatio(MOTORS[i], 0);
    vTaskDelay(M2T(MOTORS_TEST_DELAY_TIME_MS));
  }
#endif

  return isInit;
}


void motorsSetRatio(int id, uint16_t ratio)
{
  switch(id)
  {
    case MOTOR_M1:
      TIM_SetCompare4(MOTORS_GPIO_TIM_M1_2, C_16_TO_BITS(ratio));
      break;
    case MOTOR_M2:
      TIM_SetCompare3(MOTORS_GPIO_TIM_M1_2, C_16_TO_BITS(ratio));
      break;
    case MOTOR_M3:
      TIM_SetCompare4(MOTORS_GPIO_TIM_M3_4, C_16_TO_BITS(ratio));
      break;
    case MOTOR_M4:
      TIM_SetCompare3(MOTORS_GPIO_TIM_M3_4, C_16_TO_BITS(ratio));
      break;
  }
}

int motorsGetRatio(int id)
{
  switch(id)
  {
    case MOTOR_M1:
      return C_BITS_TO_16(TIM_GetCapture4(MOTORS_GPIO_TIM_M1_2));
    case MOTOR_M2:
      return C_BITS_TO_16(TIM_GetCapture3(MOTORS_GPIO_TIM_M1_2));
    case MOTOR_M3:
      return C_BITS_TO_16(TIM_GetCapture4(MOTORS_GPIO_TIM_M3_4));
    case MOTOR_M4:
      return C_BITS_TO_16(TIM_GetCapture3(MOTORS_GPIO_TIM_M3_4));
  }

  return -1;
}

#ifdef MOTOR_RAMPUP_TEST
// FreeRTOS Task to test the Motors driver with a rampup of each motor alone.
void motorsTestTask(void* params)
{
  int step=0;
  float rampup = 0.01;

  motorsSetRatio(MOTOR_M4, 1*(1<<16) * 0.0);
  motorsSetRatio(MOTOR_M3, 1*(1<<16) * 0.0);
  motorsSetRatio(MOTOR_M2, 1*(1<<16) * 0.0);
  motorsSetRatio(MOTOR_M1, 1*(1<<16) * 0.0);
  vTaskDelay(M2T(1000));

  while(1)
  {
    vTaskDelay(M2T(100));

    motorsSetRatio(MOTOR_M4, 1*(1<<16) * rampup);
    motorsSetRatio(MOTOR_M3, 1*(1<<16) * rampup);
    motorsSetRatio(MOTOR_M2, 1*(1<<16) * rampup);
    motorsSetRatio(MOTOR_M1, 1*(1<<16) * rampup);

    rampup += 0.001;
    if (rampup >= 0.1)
    {
      if(++step>3) step=0;
      rampup = 0.01;
    }
  }
}
#else
// FreeRTOS Task to test the Motors driver
void motorsTestTask(void* params)
{
  static const int sequence[] = {0.1*(1<<16), 0.15*(1<<16), 0.2*(1<<16), 0.25*(1<<16)};
  int step=0;

  //Wait 3 seconds before starting the motors
  vTaskDelay(M2T(3000));

  while(1)
  {
    motorsSetRatio(MOTOR_M4, sequence[step%4]);
    motorsSetRatio(MOTOR_M3, sequence[(step+1)%4]);
    motorsSetRatio(MOTOR_M2, sequence[(step+2)%4]);
    motorsSetRatio(MOTOR_M1, sequence[(step+3)%4]);

    if(++step>3) step=0;

    vTaskDelay(M2T(1000));
  }
}
#endif

