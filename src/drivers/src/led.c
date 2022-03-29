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
#include <stdbool.h>

#include "stm32fxxx.h"

/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "task.h"

#include "led.h"
#include "syslink.h"
#include "param.h"

#define LED_ENABLE_BITMASK_BIT    7

static GPIO_TypeDef* led_port[] =
{
  [LED_BLUE_L] = LED_GPIO_PORT_BLUE,
  [LED_GREEN_L] = LED_GPIO_PORT,
  [LED_RED_L] = LED_GPIO_PORT,
  [LED_GREEN_R] = LED_GPIO_PORT,
  [LED_RED_R] = LED_GPIO_PORT,
  [LED_BLUE_NRF] = 0,
};
static unsigned int led_pin[] =
{
  [LED_BLUE_L] = LED_GPIO_BLUE_L,
  [LED_GREEN_L] = LED_GPIO_GREEN_L,
  [LED_RED_L]   = LED_GPIO_RED_L,
  [LED_GREEN_R] = LED_GPIO_GREEN_R,
  [LED_RED_R]   = LED_GPIO_RED_R,
  [LED_BLUE_NRF] = 0,
};
static int led_polarity[] =
{
  [LED_BLUE_L] = LED_POL_BLUE_L,
  [LED_GREEN_L] = LED_POL_GREEN_L,
  [LED_RED_L]   = LED_POL_RED_L,
  [LED_GREEN_R] = LED_POL_GREEN_R,
  [LED_RED_R]   = LED_POL_RED_R,
  [LED_BLUE_NRF] = LED_POL_POS,
};

static bool isInit = 0;
static uint8_t ledControlBitmask;
static uint8_t ledLastState[LED_NUM];
ledSwitch_t ledSwitchState;

static void ledRestoreSavedState(void)
{
  for (int i = 0; i < LED_NUM; i++)
  {
    ledSet(i, ledLastState[i]);
  }
}

static void ledSetForce(led_t led, bool value)
{
  if (led > LED_NUM)
  {
    return;
  }

  if (led_polarity[led] == LED_POL_NEG)
  {
    value = !value;
  }

  if (led == LED_BLUE_NRF && isSyslinkUp())
  {
    SyslinkPacket slp;
    slp.type = value ? SYSLINK_PM_LED_ON : SYSLINK_PM_LED_OFF;
    slp.length = 0;
    syslinkSendPacket(&slp);
  }
  else
  {
    if (value)
    {
      GPIO_SetBits(led_port[led], led_pin[led]);
    }
    else
    {
      GPIO_ResetBits(led_port[led], led_pin[led]);
    }
  }
}

static void ledSetSwitch(ledSwitch_t ledSwitch)
{
  if (ledSwitchState != ledSwitch)
  {
    ledSwitchState = ledSwitch;
    switch (ledSwitch)
    {
      case LED_LEDSEQ:
        ledRestoreSavedState();
        break;
      case LED_PARAM_BITMASK:
        break;
      default:
        break;
    }
  }
}

static void ledBitmaskParamCallback(void)
{

  if (ledControlBitmask & (1 << LED_ENABLE_BITMASK_BIT))
  {
    ledSetSwitch(LED_PARAM_BITMASK);
    for (int i = 0; i < LED_NUM; i++)
    {
      ledSetForce(i, ledControlBitmask & (1<<i));
    }
  }
  else
  {
    ledSetSwitch(LED_LEDSEQ);
  }
}

void ledInit()
{
  int i;

  if(isInit)
    return;

  GPIO_InitTypeDef GPIO_InitStructure;

  // Enable GPIO
  RCC_AHB1PeriphClockCmd(LED_GPIO_PERIF, ENABLE);

  for (i = 0; i < LED_NUM; i++)
  {
    //Initialize the LED pins as an output
    GPIO_InitStructure.GPIO_Pin = led_pin[i];
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_Init(led_port[i], &GPIO_InitStructure);

    //Turn off the LED:s
    ledSet(i, 0);
  }

  ledSwitchState = LED_LEDSEQ;
  isInit = true;
}

bool ledTest(void)
{
  ledSet(LED_GREEN_L, 1);
  ledSet(LED_GREEN_R, 1);
  ledSet(LED_RED_L, 0);
  ledSet(LED_RED_R, 0);
  vTaskDelay(M2T(250));
  ledSet(LED_GREEN_L, 0);
  ledSet(LED_GREEN_R, 0);
  ledSet(LED_RED_L, 1);
  ledSet(LED_RED_R, 1);
  vTaskDelay(M2T(250));

  // LED test end
  ledSet(LED_GREEN_L, 0);
  ledSet(LED_GREEN_R, 0);
  ledSet(LED_RED_L, 0);
  ledSet(LED_RED_R, 0);
  ledSet(LED_BLUE_L, 1);
  ledSet(LED_BLUE_NRF, 1);

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
  ASSERT(led < LED_NUM);
  if (ledSwitchState == LED_LEDSEQ)
  {
    ledSetForce(led, value);
  }

  ledLastState[led] = value;
}

void ledShowFaultPattern(void)
{
  ledSet(LED_GREEN_L, 0);
  ledSet(LED_GREEN_R, 0);
  ledSet(LED_RED_L, 1);
  ledSet(LED_RED_R, 1);
  ledSet(LED_BLUE_L, 0);
}

/**
 * Parameters governing the onboard LEDs
 * */
PARAM_GROUP_START(led)
/**
 * @brief Control onboard LEDs using a bitmask. Enabling it will override the led sequencer.
 *
 * ```
 * | 7:ENABLE | 6:N/A | 5:BLUE_R | 4:RED_R | 3:GREEN_R | 2:RED_L | 1:GREEN_L | 0:BLUE_L |
 * ```
 */
PARAM_ADD_WITH_CALLBACK(PARAM_UINT8, bitmask, &ledControlBitmask, &ledBitmaskParamCallback)

PARAM_GROUP_STOP(led)

