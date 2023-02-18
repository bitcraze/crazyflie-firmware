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
 * led.h - LED functions header file
 */
#ifndef __LED_H__
#define __LED_H__

#include <stdbool.h>

//Led polarity configuration constant
#define LED_POL_POS 0
#define LED_POL_NEG 1

//Hardware configuration
#define LED_GPIO_PERIF   (RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD)
#define LED_GPIO_PORT_BLUE  GPIOD
#define LED_GPIO_BLUE_L  GPIO_Pin_2
#define LED_POL_BLUE_L   LED_POL_POS
#define LED_GPIO_PORT    GPIOC
#define LED_GPIO_GREEN_L GPIO_Pin_1
#define LED_POL_GREEN_L  LED_POL_NEG
#define LED_GPIO_RED_L   GPIO_Pin_0
#define LED_POL_RED_L    LED_POL_NEG
#define LED_GPIO_GREEN_R GPIO_Pin_2
#define LED_POL_GREEN_R  LED_POL_NEG
#define LED_GPIO_RED_R   GPIO_Pin_3
#define LED_POL_RED_R    LED_POL_NEG

#define LINK_LED         LED_GREEN_L
#define CHG_LED          LED_BLUE_L
#define LOWBAT_LED       LED_RED_R
#define LINK_DOWN_LED    LED_RED_L
#define SYS_LED          LED_GREEN_R
#define ERR_LED1         LED_RED_L
#define ERR_LED2         LED_RED_R

#define LED_NUM 6

typedef enum {LED_BLUE_L = 0, LED_GREEN_L, LED_RED_L, LED_GREEN_R, LED_RED_R, LED_BLUE_NRF} led_t;
typedef enum { LED_LEDSEQ, LED_PARAM_BITMASK } ledSwitch_t;

void ledInit();
bool ledTest();

// Clear all configured LEDs, including NRF LED
void ledClearAll(void);

// Set all configured LEDs, including NRF LED
void ledSetAll(void);

// Procedures to set the status of the LEDs
void ledSet(led_t led, bool value);

// Shoes fault pattern (2 Red ON, 2 Green and Blue OFF)
void ledShowFaultPattern(void);

//Legacy functions
#define ledSetRed(VALUE) ledSet(LED_RED, VALUE)
#define ledSetGreen(VALUE) ledSet(LED_GREEN, VALUE)

#endif
