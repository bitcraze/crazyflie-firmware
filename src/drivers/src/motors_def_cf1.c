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

static const MotorPerifDef CONN_M1 =
{
    .drvType       = BRUSHED,
    .gpioPerif     = RCC_APB2Periph_GPIOB,
    .gpioPort      = GPIOB,
    .gpioPin       = GPIO_Pin_1,
    .gpioPinSource = GPIO_PinSource1,
    .gpioOType     = 0,
    .gpioAF        = GPIO_PartialRemap_TIM3,
    .timPerif      = RCC_APB1Periph_TIM3,
    .tim           = TIM3,
    .timPolarity   = TIM_OCPolarity_High,
    .timDbgStop    = DBGMCU_TIM3_STOP,
    .timPeriod     = MOTORS_PWM_PERIOD,
    .timPrescaler  = MOTORS_PWM_PRESCALE,
    .setCompare    = TIM_SetCompare4,
    .getCompare    = TIM_GetCapture4,
    .ocInit        = TIM_OC4Init,
    .preloadConfig = TIM_OC4PreloadConfig,
};

static const MotorPerifDef CONN_M2 =
{
    .drvType       = BRUSHED,
    .gpioPerif     = RCC_APB2Periph_GPIOB,
    .gpioPort      = GPIOB,
    .gpioPin       = GPIO_Pin_0,
    .gpioPinSource = GPIO_PinSource0,
    .gpioOType     = 0,
    .gpioAF        = GPIO_PartialRemap_TIM3,
    .timPerif      = RCC_APB1Periph_TIM3,
    .tim           = TIM3,
    .timPolarity   = TIM_OCPolarity_High,
    .timDbgStop    = DBGMCU_TIM3_STOP,
    .timPeriod     = MOTORS_PWM_PERIOD,
    .timPrescaler  = MOTORS_PWM_PRESCALE,
    .setCompare    = TIM_SetCompare3,
    .getCompare    = TIM_GetCapture3,
    .ocInit        = TIM_OC3Init,
    .preloadConfig = TIM_OC3PreloadConfig,
};

static const MotorPerifDef CONN_M3 =
{
    .drvType       = BRUSHED,
    .gpioPerif     = RCC_APB2Periph_GPIOB,
    .gpioPort      = GPIOB,
    .gpioPin       = GPIO_Pin_9,
    .gpioPinSource = GPIO_PinSource9,
    .gpioOType     = 0,
    .gpioAF        = 0,
    .timPerif      = RCC_APB1Periph_TIM4,
    .tim           = TIM4,
    .timPolarity   = TIM_OCPolarity_High,
    .timDbgStop    = DBGMCU_TIM4_STOP,
    .timPeriod     = MOTORS_PWM_PERIOD,
    .timPrescaler  = MOTORS_PWM_PRESCALE,
    .setCompare    = TIM_SetCompare4,
    .getCompare    = TIM_GetCapture4,
    .ocInit        = TIM_OC4Init,
    .preloadConfig = TIM_OC4PreloadConfig,
};

static const MotorPerifDef CONN_M4 =
{
    .drvType       = BRUSHED,
    .gpioPerif     = RCC_APB2Periph_GPIOB,
    .gpioPort      = GPIOB,
    .gpioPin       = GPIO_Pin_8,
    .gpioPinSource = GPIO_PinSource8,
    .gpioOType     = 0,
    .gpioAF        = 0,
    .timPerif      = RCC_APB1Periph_TIM4,
    .tim           = TIM4,
    .timPolarity   = TIM_OCPolarity_High,
    .timDbgStop    = DBGMCU_TIM4_STOP,
    .timPeriod     = MOTORS_PWM_PERIOD,
    .timPrescaler  = MOTORS_PWM_PRESCALE,
    .setCompare    = TIM_SetCompare3,
    .getCompare    = TIM_GetCapture3,
    .ocInit        = TIM_OC3Init,
    .preloadConfig = TIM_OC3PreloadConfig,
};

static const MotorPerifDef CONN_M1_BL =
{
    .drvType       = BRUSHLESS,
    .gpioPerif     = RCC_APB2Periph_GPIOB,
    .gpioPort      = GPIOB,
    .gpioPin       = GPIO_Pin_1,
    .gpioPinSource = GPIO_PinSource1,
    .gpioOType     = 0,
    .gpioAF        = GPIO_PartialRemap_TIM3,
    .timPerif      = RCC_APB1Periph_TIM3,
    .tim           = TIM3,
    .timPolarity   = TIM_OCPolarity_Low,
    .timDbgStop    = DBGMCU_TIM3_STOP,
    .timPeriod     = MOTORS_BL_PWM_PERIOD,
    .timPrescaler  = MOTORS_BL_PWM_PRESCALE,
    .setCompare    = TIM_SetCompare4,
    .getCompare    = TIM_GetCapture4,
    .ocInit        = TIM_OC4Init,
    .preloadConfig = TIM_OC4PreloadConfig,
};

static const MotorPerifDef CONN_M2_BL =
{
    .drvType       = BRUSHLESS,
    .gpioPerif     = RCC_APB2Periph_GPIOB,
    .gpioPort      = GPIOB,
    .gpioPin       = GPIO_Pin_0,
    .gpioPinSource = GPIO_PinSource0,
    .gpioOType     = 0,
    .gpioAF        = GPIO_PartialRemap_TIM3,
    .timPerif      = RCC_APB1Periph_TIM3,
    .tim           = TIM3,
    .timPolarity   = TIM_OCPolarity_Low,
    .timDbgStop    = DBGMCU_TIM3_STOP,
    .timPeriod     = MOTORS_BL_PWM_PERIOD,
    .timPrescaler  = MOTORS_BL_PWM_PRESCALE,
    .setCompare    = TIM_SetCompare3,
    .getCompare    = TIM_GetCapture3,
    .ocInit        = TIM_OC3Init,
    .preloadConfig = TIM_OC3PreloadConfig,
};

static const MotorPerifDef CONN_M3_BL =
{
    .drvType       = BRUSHLESS,
    .gpioPerif     = RCC_APB2Periph_GPIOB,
    .gpioPort      = GPIOB,
    .gpioPin       = GPIO_Pin_9,
    .gpioPinSource = GPIO_PinSource9,
    .gpioOType     = 0,
    .gpioAF        = 0,
    .timPerif      = RCC_APB1Periph_TIM4,
    .tim           = TIM4,
    .timPolarity   = TIM_OCPolarity_Low,
    .timDbgStop    = DBGMCU_TIM4_STOP,
    .timPeriod     = MOTORS_BL_PWM_PERIOD,
    .timPrescaler  = MOTORS_BL_PWM_PRESCALE,
    .setCompare    = TIM_SetCompare4,
    .getCompare    = TIM_GetCapture4,
    .ocInit        = TIM_OC4Init,
    .preloadConfig = TIM_OC4PreloadConfig,
};

static const MotorPerifDef CONN_M4_BL =
{
    .drvType       = BRUSHLESS,
    .gpioPerif     = RCC_APB2Periph_GPIOB,
    .gpioPort      = GPIOB,
    .gpioPin       = GPIO_Pin_8,
    .gpioPinSource = GPIO_PinSource8,
    .gpioOType     = 0,
    .gpioAF        = 0,
    .timPerif      = RCC_APB1Periph_TIM4,
    .tim           = TIM4,
    .timPolarity   = TIM_OCPolarity_Low,
    .timDbgStop    = DBGMCU_TIM4_STOP,
    .timPeriod     = MOTORS_BL_PWM_PERIOD,
    .timPrescaler  = MOTORS_BL_PWM_PRESCALE,
    .setCompare    = TIM_SetCompare3,
    .getCompare    = TIM_GetCapture3,
    .ocInit        = TIM_OC3Init,
    .preloadConfig = TIM_OC3PreloadConfig,
};

/**
 * Default brushed mapping to M1-M4 connectors.
 */
const MotorPerifDef* motorMapDefaultBrushed[NBR_OF_MOTORS] =
{
  &CONN_M1,
  &CONN_M2,
  &CONN_M3,
  &CONN_M4
};

/**
 * Brushless motors mapped to the standard motor connectors with pull-ups (~1K) to VBAT soldered.
 */
const MotorPerifDef* motorMapDefaltConBrushless[NBR_OF_MOTORS] =
{
  &CONN_M1_BL,
  &CONN_M2_BL,
  &CONN_M3_BL,
  &CONN_M4_BL
};
