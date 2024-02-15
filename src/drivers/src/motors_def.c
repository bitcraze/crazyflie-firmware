/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2022 Bitcraze AB
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
 * motors_def.c - Mapping and configuration of motor outputs
 *
 */

// CF2.X connector M1, PA1, TIM2_CH2
static const MotorPerifDef MOTORS_PA1_TIM2_CH2_BRUSHED =
{
    .drvType       = BRUSHED,
    .gpioPerif     = RCC_AHB1Periph_GPIOA,
    .gpioPort      = GPIOA,
    .gpioPin       = GPIO_Pin_1,
    .gpioPinSource = GPIO_PinSource1,
    .gpioOType     = GPIO_OType_PP,
    .gpioAF        = GPIO_AF_TIM2,
    .timPerif      = RCC_APB1Periph_TIM2,
    .tim           = TIM2,
    .timPolarity   = TIM_OCPolarity_High,
    .timDbgStop    = DBGMCU_TIM2_STOP,
    .timPeriod     = MOTORS_PWM_PERIOD,
    .timPrescaler  = MOTORS_PWM_PRESCALE,
    .setCompare    = TIM_SetCompare2,
    .getCompare    = TIM_GetCapture2,
    .ocInit        = TIM_OC2Init,
    .preloadConfig = TIM_OC2PreloadConfig,
};

// CF2.X connector M2, PB11, TIM2_CH4
static const MotorPerifDef MOTORS_PB11_TIM2_CH4_BRUSHED =
{
    .drvType       = BRUSHED,
    .gpioPerif     = RCC_AHB1Periph_GPIOB,
    .gpioPort      = GPIOB,
    .gpioPin       = GPIO_Pin_11,
    .gpioPinSource = GPIO_PinSource11,
    .gpioOType     = GPIO_OType_PP,
    .gpioAF        = GPIO_AF_TIM2,
    .timPerif      = RCC_APB1Periph_TIM2,
    .tim           = TIM2,
    .timPolarity   = TIM_OCPolarity_High,
    .timDbgStop    = DBGMCU_TIM2_STOP,
    .timPeriod     = MOTORS_PWM_PERIOD,
    .timPrescaler  = MOTORS_PWM_PRESCALE,
    .setCompare    = TIM_SetCompare4,
    .getCompare    = TIM_GetCapture4,
    .ocInit        = TIM_OC4Init,
    .preloadConfig = TIM_OC4PreloadConfig,
};

// CF2.X connector M3, PA15, TIM2_CH1
static const MotorPerifDef MOTORS_PA15_TIM2_CH1_BRUSHED =
{
    .drvType       = BRUSHED,
    .gpioPerif     = RCC_AHB1Periph_GPIOA,
    .gpioPort      = GPIOA,
    .gpioPin       = GPIO_Pin_15,
    .gpioPinSource = GPIO_PinSource15,
    .gpioOType     = GPIO_OType_PP,
    .gpioAF        = GPIO_AF_TIM2,
    .timPerif      = RCC_APB1Periph_TIM2,
    .tim           = TIM2,
    .timPolarity   = TIM_OCPolarity_High,
    .timDbgStop    = DBGMCU_TIM2_STOP,
    .timPeriod     = MOTORS_PWM_PERIOD,
    .timPrescaler  = MOTORS_PWM_PRESCALE,
    .setCompare    = TIM_SetCompare1,
    .getCompare    = TIM_GetCapture1,
    .ocInit        = TIM_OC1Init,
    .preloadConfig = TIM_OC1PreloadConfig,
};

// CF2.X connector M4, PB9, TIM4_CH4
static const MotorPerifDef MOTORS_PB9_TIM4_CH4_BRUSHED =
{
    .drvType       = BRUSHED,
    .gpioPerif     = RCC_AHB1Periph_GPIOB,
    .gpioPort      = GPIOB,
    .gpioPin       = GPIO_Pin_9,
    .gpioPinSource = GPIO_PinSource9,
    .gpioOType     = GPIO_OType_PP,
    .gpioAF        = GPIO_AF_TIM4,
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
// Bolt 1.1 M4, PB10, TIM2_CH3, Brushed config
static const MotorPerifDef MOTORS_PB10_TIM2_CH3_BRUSHED =
{
    .drvType       = BRUSHED,
    .gpioPerif     = RCC_AHB1Periph_GPIOB,
    .gpioPort      = GPIOB,
    .gpioPin       = GPIO_Pin_10,
    .gpioPinSource = GPIO_PinSource10,
    .gpioOType     = GPIO_OType_PP,
    .gpioAF        = GPIO_AF_TIM2,
    .timPerif      = RCC_APB1Periph_TIM2,
    .tim           = TIM2,
    .timPolarity   = TIM_OCPolarity_High,
    .timDbgStop    = DBGMCU_TIM2_STOP,
    .timPeriod     = MOTORS_PWM_PERIOD,
    .timPrescaler  = MOTORS_PWM_PRESCALE,
    .setCompare    = TIM_SetCompare3,
    .getCompare    = TIM_GetCapture3,
    .ocInit        = TIM_OC3Init,
    .preloadConfig = TIM_OC3PreloadConfig,
};

// CF2.X connector M1, PA1, TIM2_CH2, Brushless config, inversed
static const MotorPerifDef MOTORS_PA1_TIM2_CH2_BRUSHLESS_INV_PP =
{
    .drvType       = BRUSHLESS,
    .gpioPerif     = RCC_AHB1Periph_GPIOA,
    .gpioPort      = GPIOA,
    .gpioPin       = GPIO_Pin_1,
    .gpioPinSource = GPIO_PinSource1,
    .gpioOType     = GPIO_OType_PP,
    .gpioAF        = GPIO_AF_TIM2,
    .timPerif      = RCC_APB1Periph_TIM2,
    .tim           = TIM2,
    .timPolarity   = TIM_OCPolarity_Low,
    .timDbgStop    = DBGMCU_TIM2_STOP,
    .timPeriod     = MOTORS_BL_PWM_PERIOD,
    .timPrescaler  = MOTORS_BL_PWM_PRESCALE,
    .setCompare    = TIM_SetCompare2,
    .getCompare    = TIM_GetCapture2,
    .ocInit        = TIM_OC2Init,
    .preloadConfig = TIM_OC2PreloadConfig,
};

// CF2.X connector M2, PB11, TIM2_CH4, Brushless config, inversed
static const MotorPerifDef MOTORS_PB11_TIM2_CH4_BRUSHLESS_INV_PP =
{
    .drvType       = BRUSHLESS,
    .gpioPerif     = RCC_AHB1Periph_GPIOB,
    .gpioPort      = GPIOB,
    .gpioPin       = GPIO_Pin_11,
    .gpioPinSource = GPIO_PinSource11,
    .gpioOType     = GPIO_OType_PP,
    .gpioAF        = GPIO_AF_TIM2,
    .timPerif      = RCC_APB1Periph_TIM2,
    .tim           = TIM2,
    .timPolarity   = TIM_OCPolarity_Low,
    .timDbgStop    = DBGMCU_TIM2_STOP,
    .timPeriod     = MOTORS_BL_PWM_PERIOD,
    .timPrescaler  = MOTORS_BL_PWM_PRESCALE,
    .setCompare    = TIM_SetCompare4,
    .getCompare    = TIM_GetCapture4,
    .ocInit        = TIM_OC4Init,
    .preloadConfig = TIM_OC4PreloadConfig,
};

// CF2.X connector M3, PA15, TIM2_CH1, Brushless config, inversed
static const MotorPerifDef MOTORS_PA15_TIM2_CH1_BRUSHLESS_INV_PP =
{
    .drvType       = BRUSHLESS,
    .gpioPerif     = RCC_AHB1Periph_GPIOA,
    .gpioPort      = GPIOA,
    .gpioPin       = GPIO_Pin_15,
    .gpioPinSource = GPIO_PinSource15,
    .gpioOType     = GPIO_OType_PP,
    .gpioAF        = GPIO_AF_TIM2,
    .timPerif      = RCC_APB1Periph_TIM2,
    .tim           = TIM2,
    .timPolarity   = TIM_OCPolarity_Low,
    .timDbgStop    = DBGMCU_TIM2_STOP,
    .timPeriod     = MOTORS_BL_PWM_PERIOD,
    .timPrescaler  = MOTORS_BL_PWM_PRESCALE,
    .setCompare    = TIM_SetCompare1,
    .getCompare    = TIM_GetCapture1,
    .ocInit        = TIM_OC1Init,
    .preloadConfig = TIM_OC1PreloadConfig,
};

// CF2.X connector M4, PB9, TIM4_CH4, Brushless config, inversed
static const MotorPerifDef MOTORS_PB9_TIM4_CH4_BRUSHLESS_INV_PP =
{
    .drvType       = BRUSHLESS,
    .gpioPerif     = RCC_AHB1Periph_GPIOB,
    .gpioPort      = GPIOB,
    .gpioPin       = GPIO_Pin_9,
    .gpioPinSource = GPIO_PinSource9,
    .gpioOType     = GPIO_OType_PP,
    .gpioAF        = GPIO_AF_TIM4,
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

// Bolt M1, PA1, TIM2_CH2, Brushless config
static const MotorPerifDef MOTORS_PA1_TIM2_CH2_BRUSHLESS_PP =
{
    .drvType       = BRUSHLESS,
    .gpioPerif     = RCC_AHB1Periph_GPIOA,
    .gpioPort      = GPIOA,
    .gpioPin       = GPIO_Pin_1,
    .gpioPinSource = GPIO_PinSource1,
    .gpioOType     = GPIO_OType_PP,
    .gpioAF        = GPIO_AF_TIM2,
    .gpioPowerswitchPerif = RCC_AHB1Periph_GPIOA,
    .gpioPowerswitchPort  = GPIOA,
    .gpioPowerswitchPin   = GPIO_Pin_0,
    .timPerif      = RCC_APB1Periph_TIM2,
    .tim           = TIM2,
    .timPolarity   = TIM_OCPolarity_High,
    .timDbgStop    = DBGMCU_TIM2_STOP,
    .timPeriod     = MOTORS_BL_PWM_PERIOD,
    .timPrescaler  = MOTORS_BL_PWM_PRESCALE,
    .setCompare    = TIM_SetCompare2,
    .getCompare    = TIM_GetCapture2,
    .ocInit        = TIM_OC2Init,
    .preloadConfig = TIM_OC2PreloadConfig,
    .DMA_stream    = DMA1_Stream6,
    .DMA_Channel   = DMA_Channel_3,
    .DMA_PerifAddr = (uint32_t)&TIM2->CCR2,
    .TIM_DMASource = TIM_DMA_CC2,
    .DMA_IRQChannel = DMA1_Stream6_IRQn,
};

// Bolt M2, PB11, TIM2_CH4, Brushless config
static const MotorPerifDef MOTORS_PB11_TIM2_CH4_BRUSHLESS_PP =
{
    .drvType       = BRUSHLESS,
    .gpioPerif     = RCC_AHB1Periph_GPIOB,
    .gpioPort      = GPIOB,
    .gpioPin       = GPIO_Pin_11,
    .gpioPinSource = GPIO_PinSource11,
    .gpioOType     = GPIO_OType_PP,
    .gpioAF        = GPIO_AF_TIM2,
    .gpioPowerswitchPerif = RCC_AHB1Periph_GPIOB,
    .gpioPowerswitchPort  = GPIOB,
    .gpioPowerswitchPin   = GPIO_Pin_12,
    .timPerif      = RCC_APB1Periph_TIM2,
    .tim           = TIM2,
    .timPolarity   = TIM_OCPolarity_High,
    .timDbgStop    = DBGMCU_TIM2_STOP,
    .timPeriod     = MOTORS_BL_PWM_PERIOD,
    .timPrescaler  = MOTORS_BL_PWM_PRESCALE,
    .setCompare    = TIM_SetCompare4,
    .getCompare    = TIM_GetCapture4,
    .ocInit        = TIM_OC4Init,
    .preloadConfig = TIM_OC4PreloadConfig,
    .DMA_stream    = DMA1_Stream7,
    .DMA_Channel   = DMA_Channel_3,
    .DMA_PerifAddr = (uint32_t)&TIM2->CCR4,
    .TIM_DMASource = TIM_DMA_CC4,
    .DMA_IRQChannel = DMA1_Stream7_IRQn,

};

// Bolt M3, PA15, TIM2_CH1, Brushless config
static const MotorPerifDef MOTORS_PA15_TIM2_CH1_BRUSHLESS_PP =
{
    .drvType       = BRUSHLESS,
    .gpioPerif     = RCC_AHB1Periph_GPIOA,
    .gpioPort      = GPIOA,
    .gpioPin       = GPIO_Pin_15,
    .gpioPinSource = GPIO_PinSource15,
    .gpioOType     = GPIO_OType_PP,
    .gpioAF        = GPIO_AF_TIM2,
    .gpioPowerswitchPerif = RCC_AHB1Periph_GPIOC,
    .gpioPowerswitchPort  = GPIOC,
    .gpioPowerswitchPin   = GPIO_Pin_8,
    .timPerif      = RCC_APB1Periph_TIM2,
    .tim           = TIM2,
    .timPolarity   = TIM_OCPolarity_High,
    .timDbgStop    = DBGMCU_TIM2_STOP,
    .timPeriod     = MOTORS_BL_PWM_PERIOD,
    .timPrescaler  = MOTORS_BL_PWM_PRESCALE,
    .setCompare    = TIM_SetCompare1,
    .getCompare    = TIM_GetCapture1,
    .ocInit        = TIM_OC1Init,
    .preloadConfig = TIM_OC1PreloadConfig,
    .DMA_stream    = DMA1_Stream5,
    .DMA_Channel   = DMA_Channel_3,
    .DMA_PerifAddr = (uint32_t)&TIM2->CCR1,
    .TIM_DMASource = TIM_DMA_CC1,
    .DMA_IRQChannel = DMA1_Stream5_IRQn,
};

// Bolt M4, PB9, TIM4_CH4, Brushless config
static const MotorPerifDef MOTORS_PB9_TIM4_CH4_BRUSHLESS_PP =
{
    .drvType       = BRUSHLESS,
    .gpioPerif     = RCC_AHB1Periph_GPIOB,
    .gpioPort      = GPIOB,
    .gpioPin       = GPIO_Pin_9,
    .gpioPinSource = GPIO_PinSource9,
    .gpioOType     = GPIO_OType_PP,
    .gpioAF        = GPIO_AF_TIM4,
    .gpioPowerswitchPerif = RCC_AHB1Periph_GPIOC,
    .gpioPowerswitchPort  = GPIOC,
    .gpioPowerswitchPin   = GPIO_Pin_15,
    .timPerif      = RCC_APB1Periph_TIM4,
    .tim           = TIM4,
    .timPolarity   = TIM_OCPolarity_High,
    .timDbgStop    = DBGMCU_TIM4_STOP,
    .timPeriod     = MOTORS_BL_PWM_PERIOD,
    .timPrescaler  = MOTORS_BL_PWM_PRESCALE,
    .setCompare    = TIM_SetCompare4,
    .getCompare    = TIM_GetCapture4,
    .ocInit        = TIM_OC4Init,
    .preloadConfig = TIM_OC4PreloadConfig,
};

// Bolt 1.1 M4, PB10, TIM2_CH3, Brushless config
static const MotorPerifDef MOTORS_PB10_TIM2_CH3_BRUSHLESS_PP =
{
    .drvType       = BRUSHLESS,
    .gpioPerif     = RCC_AHB1Periph_GPIOB,
    .gpioPort      = GPIOB,
    .gpioPin       = GPIO_Pin_10,
    .gpioPinSource = GPIO_PinSource10,
    .gpioOType     = GPIO_OType_PP,
    .gpioAF        = GPIO_AF_TIM2,
    .gpioPowerswitchPerif = RCC_AHB1Periph_GPIOC,
    .gpioPowerswitchPort  = GPIOC,
    .gpioPowerswitchPin   = GPIO_Pin_15,
    .timPerif      = RCC_APB1Periph_TIM2,
    .tim           = TIM2,
    .timPolarity   = TIM_OCPolarity_High,
    .timDbgStop    = DBGMCU_TIM2_STOP,
    .timPeriod     = MOTORS_BL_PWM_PERIOD,
    .timPrescaler  = MOTORS_BL_PWM_PRESCALE,
    .setCompare    = TIM_SetCompare3,
    .getCompare    = TIM_GetCapture3,
    .ocInit        = TIM_OC3Init,
    .preloadConfig = TIM_OC3PreloadConfig,
    .DMA_stream    = DMA1_Stream1,
    .DMA_Channel   = DMA_Channel_3,
    .DMA_PerifAddr = (uint32_t)&TIM2->CCR3,
    .TIM_DMASource = TIM_DMA_CC3,
    .DMA_IRQChannel = DMA1_Stream1_IRQn,
};

// CF21-BL M1, PA1, TIM2_CH2, Brushless config including DSHOT
static const MotorPerifDef MOTORS_PA1_TIM2_CH2_BRUSHLESS_OD =
{
    .drvType       = BRUSHLESS,
    .gpioPerif     = RCC_AHB1Periph_GPIOA,
    .gpioPort      = GPIOA,
    .gpioPin       = GPIO_Pin_1,
    .gpioPinSource = GPIO_PinSource1,
    .gpioOType     = GPIO_OType_OD,
    .gpioAF        = GPIO_AF_TIM2,
    .gpioPowerswitchPerif = RCC_AHB1Periph_GPIOA,
    .gpioPowerswitchPort  = GPIOA,
    .gpioPowerswitchPin   = GPIO_Pin_0,
    .timPerif      = RCC_APB1Periph_TIM2,
    .tim           = TIM2,
    .timPolarity   = TIM_OCPolarity_High,
    .timDbgStop    = DBGMCU_TIM2_STOP,
    .timPeriod     = MOTORS_BL_PWM_PERIOD,
    .timPrescaler  = MOTORS_BL_PWM_PRESCALE,
    .setCompare    = TIM_SetCompare2,
    .getCompare    = TIM_GetCapture2,
    .ocInit        = TIM_OC2Init,
    .preloadConfig = TIM_OC2PreloadConfig,
    .DMA_stream    = DMA1_Stream6,
    .DMA_Channel   = DMA_Channel_3,
    .DMA_PerifAddr = (uint32_t)&TIM2->CCR2,
    .TIM_DMASource = TIM_DMA_CC2,
    .DMA_IRQChannel = DMA1_Stream6_IRQn,
};

// CF21-BL M2, PB11, TIM2_CH4, Brushless config including DSHOT
static const MotorPerifDef MOTORS_PB11_TIM2_CH4_BRUSHLESS_OD =
{
    .drvType       = BRUSHLESS,
    .gpioPerif     = RCC_AHB1Periph_GPIOB,
    .gpioPort      = GPIOB,
    .gpioPin       = GPIO_Pin_11,
    .gpioPinSource = GPIO_PinSource11,
    .gpioOType     = GPIO_OType_OD,
    .gpioAF        = GPIO_AF_TIM2,
    .gpioPowerswitchPerif = RCC_AHB1Periph_GPIOB,
    .gpioPowerswitchPort  = GPIOB,
    .gpioPowerswitchPin   = GPIO_Pin_12,
    .timPerif      = RCC_APB1Periph_TIM2,
    .tim           = TIM2,
    .timPolarity   = TIM_OCPolarity_High,
    .timDbgStop    = DBGMCU_TIM2_STOP,
    .timPeriod     = MOTORS_BL_PWM_PERIOD,
    .timPrescaler  = MOTORS_BL_PWM_PRESCALE,
    .setCompare    = TIM_SetCompare4,
    .getCompare    = TIM_GetCapture4,
    .ocInit        = TIM_OC4Init,
    .preloadConfig = TIM_OC4PreloadConfig,
    .DMA_stream    = DMA1_Stream7,
    .DMA_Channel   = DMA_Channel_3,
    .DMA_PerifAddr = (uint32_t)&TIM2->CCR4,
    .TIM_DMASource = TIM_DMA_CC4,
    .DMA_IRQChannel = DMA1_Stream7_IRQn,
};

// CF21-BL M3, PA15, TIM2_CH1, Brushless config including DSHOT
    static const MotorPerifDef MOTORS_PA15_TIM2_CH1_BRUSHLESS_OD =
{
    .drvType       = BRUSHLESS,
    .gpioPerif     = RCC_AHB1Periph_GPIOA,
    .gpioPort      = GPIOA,
    .gpioPin       = GPIO_Pin_15,
    .gpioPinSource = GPIO_PinSource15,
    .gpioOType     = GPIO_OType_OD,
    .gpioAF        = GPIO_AF_TIM2,
    .gpioPowerswitchPerif = RCC_AHB1Periph_GPIOC,
    .gpioPowerswitchPort  = GPIOC,
    .gpioPowerswitchPin   = GPIO_Pin_8,
    .timPerif      = RCC_APB1Periph_TIM2,
    .tim           = TIM2,
    .timPolarity   = TIM_OCPolarity_High,
    .timDbgStop    = DBGMCU_TIM2_STOP,
    .timPeriod     = MOTORS_BL_PWM_PERIOD,
    .timPrescaler  = MOTORS_BL_PWM_PRESCALE,
    .setCompare    = TIM_SetCompare1,
    .getCompare    = TIM_GetCapture1,
    .ocInit        = TIM_OC1Init,
    .preloadConfig = TIM_OC1PreloadConfig,
    .DMA_stream    = DMA1_Stream5,
    .DMA_Channel   = DMA_Channel_3,
    .DMA_PerifAddr = (uint32_t)&TIM2->CCR1,
    .TIM_DMASource = TIM_DMA_CC1,
    .DMA_IRQChannel = DMA1_Stream5_IRQn,
};


// CF21-BL M4, PB10, TIM2_CH3, Brushless config including DSHOT
static const MotorPerifDef MOTORS_PB10_TIM2_CH3_BRUSHLESS_OD =
{
    .drvType       = BRUSHLESS,
    .gpioPerif     = RCC_AHB1Periph_GPIOB,
    .gpioPort      = GPIOB,
    .gpioPin       = GPIO_Pin_10,
    .gpioPinSource = GPIO_PinSource10,
    .gpioOType     = GPIO_OType_OD,
    .gpioAF        = GPIO_AF_TIM2,
    .gpioPowerswitchPerif = RCC_AHB1Periph_GPIOC,
    .gpioPowerswitchPort  = GPIOC,
    .gpioPowerswitchPin   = GPIO_Pin_15,
    .timPerif      = RCC_APB1Periph_TIM2,
    .tim           = TIM2,
    .timPolarity   = TIM_OCPolarity_High,
    .timDbgStop    = DBGMCU_TIM2_STOP,
    .timPeriod     = MOTORS_BL_PWM_PERIOD,
    .timPrescaler  = MOTORS_BL_PWM_PRESCALE,
    .setCompare    = TIM_SetCompare3,
    .getCompare    = TIM_GetCapture3,
    .ocInit        = TIM_OC3Init,
    .preloadConfig = TIM_OC3PreloadConfig,
    .DMA_stream    = DMA1_Stream1,
    .DMA_Channel   = DMA_Channel_3,
    .DMA_PerifAddr = (uint32_t)&TIM2->CCR3,
    .TIM_DMASource = TIM_DMA_CC3,
    .DMA_IRQChannel = DMA1_Stream1_IRQn,
};


// Deck TX2, PA2, TIM2_CH3
static const MotorPerifDef MOTORS_PA2_TIM2_CH3_BRUSHLESS_OD =
{
    .drvType       = BRUSHLESS,
    .gpioPerif     = RCC_AHB1Periph_GPIOA,
    .gpioPort      = GPIOA,
    .gpioPin       = GPIO_Pin_2,
    .gpioPinSource = GPIO_PinSource2,
    .gpioOType     = GPIO_OType_OD,
    .gpioAF        = GPIO_AF_TIM2,
    .timPerif      = RCC_APB1Periph_TIM2,
    .tim           = TIM2,
    .timPolarity   = TIM_OCPolarity_High,
    .timDbgStop    = DBGMCU_TIM2_STOP,
    .timPeriod     = MOTORS_BL_PWM_PERIOD,
    .timPrescaler  = MOTORS_BL_PWM_PRESCALE,
    .setCompare    = TIM_SetCompare3,
    .getCompare    = TIM_GetCapture3,
    .ocInit        = TIM_OC3Init,
    .preloadConfig = TIM_OC3PreloadConfig,
};

// Deck TX2, PA2, TIM5_CH3
static const MotorPerifDef MOTORS_PA2_TIM5_CH3_BRUSHLESS_OD =
{
    .drvType       = BRUSHLESS,
    .gpioPerif     = RCC_AHB1Periph_GPIOA,
    .gpioPort      = GPIOA,
    .gpioPin       = GPIO_Pin_2,
    .gpioPinSource = GPIO_PinSource2,
    .gpioOType     = GPIO_OType_OD,
    .gpioAF        = GPIO_AF_TIM5,
    .timPerif      = RCC_APB1Periph_TIM5,
    .tim           = TIM5,
    .timPolarity   = TIM_OCPolarity_High,
    .timDbgStop    = DBGMCU_TIM5_STOP,
    .timPeriod     = MOTORS_BL_PWM_PERIOD,
    .timPrescaler  = MOTORS_BL_PWM_PRESCALE,
    .setCompare    = TIM_SetCompare3,
    .getCompare    = TIM_GetCapture3,
    .ocInit        = TIM_OC3Init,
    .preloadConfig = TIM_OC3PreloadConfig,
};

// Deck RX2, PA3, TIM2_CH4
static const MotorPerifDef MOTORS_PA3_TIM2_CH4_BRUSHLESS_OD =
{
    .drvType       = BRUSHLESS,
    .gpioPerif     = RCC_AHB1Periph_GPIOA,
    .gpioPort      = GPIOA,
    .gpioPin       = GPIO_Pin_3,
    .gpioPinSource = GPIO_PinSource3,
    .gpioOType     = GPIO_OType_OD,
    .gpioAF        = GPIO_AF_TIM2,
    .timPerif      = RCC_APB1Periph_TIM2,
    .tim           = TIM2,
    .timPolarity   = TIM_OCPolarity_High,
    .timDbgStop    = DBGMCU_TIM2_STOP,
    .timPeriod     = MOTORS_BL_PWM_PERIOD,
    .timPrescaler  = MOTORS_BL_PWM_PRESCALE,
    .setCompare    = TIM_SetCompare4,
    .getCompare    = TIM_GetCapture4,
    .ocInit        = TIM_OC4Init,
    .preloadConfig = TIM_OC4PreloadConfig,
};

// Deck RX2, PA3, TIM5_CH4
static const MotorPerifDef MOTORS_PA3_TIM5_CH4_BRUSHLESS_OD =
{
    .drvType       = BRUSHLESS,
    .gpioPerif     = RCC_AHB1Periph_GPIOA,
    .gpioPort      = GPIOA,
    .gpioPin       = GPIO_Pin_3,
    .gpioPinSource = GPIO_PinSource3,
    .gpioOType     = GPIO_OType_OD,
    .gpioAF        = GPIO_AF_TIM5,
    .timPerif      = RCC_APB1Periph_TIM5,
    .tim           = TIM5,
    .timPolarity   = TIM_OCPolarity_High,
    .timDbgStop    = DBGMCU_TIM5_STOP,
    .timPeriod     = MOTORS_BL_PWM_PERIOD,
    .timPrescaler  = MOTORS_BL_PWM_PRESCALE,
    .setCompare    = TIM_SetCompare4,
    .getCompare    = TIM_GetCapture4,
    .ocInit        = TIM_OC4Init,
    .preloadConfig = TIM_OC4PreloadConfig,
};

// Deck IO1, PB8, TIM4_CH3
static const MotorPerifDef MOTORS_PB8_TIM4_CH3_BRUSHLESS_OD =
{
    .drvType       = BRUSHLESS,
    .gpioPerif     = RCC_AHB1Periph_GPIOB,
    .gpioPort      = GPIOB,
    .gpioPin       = GPIO_Pin_8,
    .gpioPinSource = GPIO_PinSource8,
    .gpioOType     = GPIO_OType_OD,
    .gpioAF        = GPIO_AF_TIM4,
    .timPerif      = RCC_APB1Periph_TIM4,
    .tim           = TIM4,
    .timPolarity   = TIM_OCPolarity_High,
    .timDbgStop    = DBGMCU_TIM4_STOP,
    .timPeriod     = MOTORS_BL_PWM_PERIOD,
    .timPrescaler  = MOTORS_BL_PWM_PRESCALE,
    .setCompare    = TIM_SetCompare3,
    .getCompare    = TIM_GetCapture3,
    .ocInit        = TIM_OC3Init,
    .preloadConfig = TIM_OC3PreloadConfig,
};

// Deck IO2, PB5, TIM3_CH2
static const MotorPerifDef MOTORS_PB5_TIM3_CH2_BRUSHLESS_OD =
{
    .drvType       = BRUSHLESS,
    .gpioPerif     = RCC_AHB1Periph_GPIOB,
    .gpioPort      = GPIOB,
    .gpioPin       = GPIO_Pin_5,
    .gpioPinSource = GPIO_PinSource5,
    .gpioOType     = GPIO_OType_OD,
    .gpioAF        = GPIO_AF_TIM3,
    .timPerif      = RCC_APB1Periph_TIM3,
    .tim           = TIM3,
    .timPolarity   = TIM_OCPolarity_High,
    .timDbgStop    = DBGMCU_TIM3_STOP,
    .timPeriod     = MOTORS_BL_PWM_PERIOD,
    .timPrescaler  = MOTORS_BL_PWM_PRESCALE,
    .setCompare    = TIM_SetCompare2,
    .getCompare    = TIM_GetCapture2,
    .ocInit        = TIM_OC2Init,
    .preloadConfig = TIM_OC2PreloadConfig,
};

// Deck IO3, PB4, TIM3_CH1
static const MotorPerifDef MOTORS_PB4_TIM3_CH1_BRUSHLESS_OD =
{
    .drvType       = BRUSHLESS,
    .gpioPerif     = RCC_AHB1Periph_GPIOB,
    .gpioPort      = GPIOB,
    .gpioPin       = GPIO_Pin_4,
    .gpioPinSource = GPIO_PinSource4,
    .gpioOType     = GPIO_OType_OD,
    .gpioAF        = GPIO_AF_TIM3,
    .timPerif      = RCC_APB1Periph_TIM3,
    .tim           = TIM3,
    .timPolarity   = TIM_OCPolarity_High,
    .timDbgStop    = DBGMCU_TIM3_STOP,
    .timPeriod     = MOTORS_BL_PWM_PERIOD,
    .timPrescaler  = MOTORS_BL_PWM_PRESCALE,
    .setCompare    = TIM_SetCompare1,
    .getCompare    = TIM_GetCapture1,
    .ocInit        = TIM_OC1Init,
    .preloadConfig = TIM_OC1PreloadConfig,
};

// Deck SCK, PA5, TIM2_CH1
static const MotorPerifDef MOTORS_PA5_TIM2_CH1_BRUSHLESS_OD =
{
    .drvType       = BRUSHLESS,
    .gpioPerif     = RCC_AHB1Periph_GPIOA,
    .gpioPort      = GPIOA,
    .gpioPin       = GPIO_Pin_5,
    .gpioPinSource = GPIO_PinSource5,
    .gpioOType     = GPIO_OType_OD,
    .gpioAF        = GPIO_AF_TIM2,
    .timPerif      = RCC_APB1Periph_TIM2,
    .tim           = TIM2,
    .timPolarity   = TIM_OCPolarity_High,
    .timDbgStop    = DBGMCU_TIM2_STOP,
    .timPeriod     = MOTORS_BL_PWM_PERIOD,
    .timPrescaler  = MOTORS_BL_PWM_PRESCALE,
    .setCompare    = TIM_SetCompare1,
    .getCompare    = TIM_GetCapture1,
    .ocInit        = TIM_OC1Init,
    .preloadConfig = TIM_OC1PreloadConfig,
};

// Deck MISO, PA6, TIM3_CH1
static const MotorPerifDef MOTORS_PA6_TIM3_CH1_BRUSHLESS_OD =
{
    .drvType       = BRUSHLESS,
    .gpioPerif     = RCC_AHB1Periph_GPIOA,
    .gpioPort      = GPIOA,
    .gpioPin       = GPIO_Pin_6,
    .gpioPinSource = GPIO_PinSource6,
    .gpioOType     = GPIO_OType_OD,
    .gpioAF        = GPIO_AF_TIM3,
    .timPerif      = RCC_APB1Periph_TIM3,
    .tim           = TIM3,
    .timPolarity   = TIM_OCPolarity_High,
    .timDbgStop    = DBGMCU_TIM3_STOP,
    .timPeriod     = MOTORS_BL_PWM_PERIOD,
    .timPrescaler  = MOTORS_BL_PWM_PRESCALE,
    .setCompare    = TIM_SetCompare1,
    .getCompare    = TIM_GetCapture1,
    .ocInit        = TIM_OC1Init,
    .preloadConfig = TIM_OC1PreloadConfig,
};

// Deck MOSI, PA7, TIM14_CH1
static const MotorPerifDef MOTORS_PA7_TIM14_CH1_BRUSHLESS_OD =
{
    .drvType       = BRUSHLESS,
    .gpioPerif     = RCC_AHB1Periph_GPIOA,
    .gpioPort      = GPIOA,
    .gpioPin       = GPIO_Pin_7,
    .gpioPinSource = GPIO_PinSource7,
    .gpioOType     = GPIO_OType_OD,
    .gpioAF        = GPIO_AF_TIM14,
    .timPerif      = RCC_APB1Periph_TIM14,
    .tim           = TIM14,
    .timPolarity   = TIM_OCPolarity_High,
    .timDbgStop    = DBGMCU_TIM14_STOP,
    .timPeriod     = MOTORS_BL_PWM_PERIOD,
    .timPrescaler  = MOTORS_BL_PWM_PRESCALE,
    .setCompare    = TIM_SetCompare1,
    .getCompare    = TIM_GetCapture1,
    .ocInit        = TIM_OC1Init,
    .preloadConfig = TIM_OC1PreloadConfig,
};

/**
 * Mapping for Tags that don't have motors.
 * Actually same mapping as for CF2 but the pins are not physically connected.
 */
const MotorPerifDef* motorMapNoMotors[NBR_OF_MOTORS] =
{
  &MOTORS_PA1_TIM2_CH2_BRUSHED,
  &MOTORS_PB11_TIM2_CH4_BRUSHED,
  &MOTORS_PA15_TIM2_CH1_BRUSHED,
  &MOTORS_PB9_TIM4_CH4_BRUSHED
};

/**
 * Default brushed mapping to M1-M4 connectors.
 */
const MotorPerifDef* motorMapDefaultBrushed[NBR_OF_MOTORS] =
{
  &MOTORS_PA1_TIM2_CH2_BRUSHED,
  &MOTORS_PB11_TIM2_CH4_BRUSHED,
  &MOTORS_PA15_TIM2_CH1_BRUSHED,
  &MOTORS_PB9_TIM4_CH4_BRUSHED
};

/**
 * Brushless motors mapped as on the Big-Quad deck
 * M1 -> TX2
 * M2 -> IO3
 * M3 -> IO2
 * M4 -> RX2
 */
const MotorPerifDef* motorMapBigQuadDeck[NBR_OF_MOTORS] =
{
  &MOTORS_PA2_TIM2_CH3_BRUSHLESS_OD,
  &MOTORS_PB4_TIM3_CH1_BRUSHLESS_OD,
  &MOTORS_PB5_TIM3_CH2_BRUSHLESS_OD,
  &MOTORS_PA3_TIM2_CH4_BRUSHLESS_OD
};

/**
 * Brushless motors mapped to the standard motor connectors with pull-ups (~1K) to VBAT soldered.
 */
const MotorPerifDef* motorMapDefaltConBrushless[NBR_OF_MOTORS] =
{
  &MOTORS_PA1_TIM2_CH2_BRUSHLESS_INV_PP,
  &MOTORS_PB11_TIM2_CH4_BRUSHLESS_INV_PP,
  &MOTORS_PA15_TIM2_CH1_BRUSHLESS_INV_PP,
  &MOTORS_PB9_TIM4_CH4_BRUSHLESS_INV_PP
};

/**
 * Brushless motors mapped to the Bolt PWM outputs.
 */
const MotorPerifDef* motorMapBoltBrushless[NBR_OF_MOTORS] =
{
  &MOTORS_PA1_TIM2_CH2_BRUSHLESS_PP,
  &MOTORS_PB11_TIM2_CH4_BRUSHLESS_PP,
  &MOTORS_PA15_TIM2_CH1_BRUSHLESS_PP,
  &MOTORS_PB9_TIM4_CH4_BRUSHLESS_PP
};

/**
 * Brushless motors mapped to the Bolt 1.1 PWM outputs.
 */
const MotorPerifDef* motorMapBolt11Brushless[NBR_OF_MOTORS] =
{
  &MOTORS_PA1_TIM2_CH2_BRUSHLESS_PP,
  &MOTORS_PB11_TIM2_CH4_BRUSHLESS_PP,
  &MOTORS_PA15_TIM2_CH1_BRUSHLESS_PP,
  &MOTORS_PB10_TIM2_CH3_BRUSHLESS_PP
};

/**
 * Brushed motors mapped to the Bolt 1.1 PWM outputs.
 * 0R resistors 0402 should be mounted at R19, R22, R33, R34 and
 * motor override signals will be disabled (high impedance).
 */
const MotorPerifDef* motorMapBolt11Brushed[NBR_OF_MOTORS] =
{
  &MOTORS_PA1_TIM2_CH2_BRUSHED,
  &MOTORS_PB11_TIM2_CH4_BRUSHED,
  &MOTORS_PA15_TIM2_CH1_BRUSHED,
  &MOTORS_PB10_TIM2_CH3_BRUSHED
};

/**
 * Brushless motors mapped to the Bolt Rev.F PWM outputs.
 */
const MotorPerifDef* motorMapCF21Brushless[NBR_OF_MOTORS] =
{
    &MOTORS_PA1_TIM2_CH2_BRUSHLESS_OD,
    &MOTORS_PB11_TIM2_CH4_BRUSHLESS_OD,
    &MOTORS_PA15_TIM2_CH1_BRUSHLESS_OD,
    &MOTORS_PB10_TIM2_CH3_BRUSHLESS_OD
};

/**
 * Servo mapped to the Bigquad CPPM (MOSI) port
 */
const MotorPerifDef* servoMapMOSI = &MOTORS_PA7_TIM14_CH1_BRUSHLESS_OD;

/**
 * Servo mapped to the Bigquad M1 / TX2 port
 */
const MotorPerifDef* servoMapTX2 = &MOTORS_PA2_TIM5_CH3_BRUSHLESS_OD;

/**
 * Servo mapped to the Bigquad M3 / IO2 port
 */
const MotorPerifDef* servoMapIO2 = &MOTORS_PB5_TIM3_CH2_BRUSHLESS_OD;

/**
 * Servo mapped to the Bigquad M2 / IO3 port
 */
const MotorPerifDef* servoMapIO3 = &MOTORS_PB4_TIM3_CH1_BRUSHLESS_OD;

/**
 * Servo mapped to the Bigquad M4 / RX2 port
 */
const MotorPerifDef* servoMapRX2 = &MOTORS_PA3_TIM5_CH4_BRUSHLESS_OD;

/**
 * Servo mapped to IO1 port
 */
const MotorPerifDef* servoMapIO1 = &MOTORS_PB8_TIM4_CH3_BRUSHLESS_OD;
