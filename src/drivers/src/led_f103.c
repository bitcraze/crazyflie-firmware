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
 * led.c - LED handing functions
 */
#include "led.h"

#include "motors.h"

#include <stdbool.h>
#include "stm32fxxx.h"

/*FreeRtos includes*/
#include "FreeRTOS.h"

static bool isInit=false;

static GPIO_TypeDef* led_port[] = {
  [LED_GREEN] = LED_GPIO_PORT, 
  [LED_RED] = LED_GPIO_PORT,
};
static unsigned int led_pin[] = {
  [LED_GREEN] = LED_GPIO_GREEN, 
  [LED_RED]   = LED_GPIO_RED,
};
static int led_polarity[] = {
  [LED_GREEN] = LED_POL_GREEN, 
  [LED_RED] = LED_POL_RED,
};

//Initialize the green led pin as output
void ledInit()
{
  if(isInit)
    return;

  GPIO_InitTypeDef GPIO_InitStructure;

  // Enable GPIO
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | LED_GPIO_PERIF, ENABLE);

  // Remap PB4
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_NoJTRST , ENABLE);

  //Initialize the LED pins as an output
  GPIO_InitStructure.GPIO_Pin = LED_GPIO_GREEN | LED_GPIO_RED;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;

  GPIO_Init(GPIOB, &GPIO_InitStructure);

  //Turn off the LED:s
  ledSet(LED_GREEN, 0);
  ledSet(LED_RED, 0);

  isInit = true;
}

bool ledTest(void)
{
  return isInit;
}

void ledClearAll(void)
{
  int i;

  for (i = 0; i < LED_NUM; i++)
  {
    //Turn off the LED:s
    ledSet(i, 0);
  }
}

void ledSetAll(void)
{
  int i;

  for (i = 0; i < LED_NUM; i++)
  {
    //Turn on the LED:s
    ledSet(i, 1);
  }
}
void ledSet(led_t led, bool value)
{
  if (led>LED_NUM)
    return;

  if (led_polarity[led]==LED_POL_NEG)
    value = !value;
  
  if(value)
    GPIO_SetBits(led_port[led], led_pin[led]);
  else
    GPIO_ResetBits(led_port[led], led_pin[led]); 
}


