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
 * config.h - Main configuration file
 *
 * This file define the default configuration of the copter
 * It contains two types of parameters:
 * - The global parameters are globally defined and independent of any
 *   compilation profile. An example of such define could be some pinout.
 * - The profiled defines, they are parameter that can be specific to each
 *   dev build. The vanilla build is intended to be a "customer" build without
 *   fancy spinning debugging stuff. The developers build are anything the
 *   developer could need to debug and run his code/crazy stuff.
 *
 * The golden rule for the profile is NEVER BREAK ANOTHER PROFILE. When adding a
 * new parameter, one shall take care to modified everything necessary to
 * preserve the behavior of the other profiles.
 *
 * For the flag. T_ means task. H_ means HAL module. U_ would means utils.
 */

#ifndef CONFIG_H_
#define CONFIG_H_
#include "nrf24l01.h"

#ifdef STM32F4XX
  #define P_NAME "Crazyflie 2.0 Rev.C"
  #define QUAD_FORMATION_X

  #define CONFIG_BLOCK_ADDRESS    (2048 * (64-1))
  #define MCU_ID_ADDRESS          0x1FFF7A10
  #define MCU_FLASH_SIZE_ADDRESS  0x1FFF7A22
#else
  #define P_NAME "Crazyflie Rev.F"
  #define CONFIG_BLOCK_ADDRESS    (1024 * (64-1))
  #define MCU_ID_ADDRESS          0x1FFFF7E8
  #define MCU_FLASH_SIZE_ADDRESS  0x1FFFF7E0
#endif


//Task priorities. Higher number higher priority
#define SYSTEM_TASK_PRI         2
#define CRTP_RXTX_TASK_PRI      2
#define LOG_TASK_PRI            1
#define MEM_TASK_PRI            1
#define PARAM_TASK_PRI          1
#define STABILIZER_TASK_PRI     4
#define SYSLINK_TASK_PRI        3
#define USBLINK_TASK_PRI        3
//CF1
#define ADC_TASK_PRI            0
#define PM_TASK_PRI             0
#define NRF24LINK_TASK_PRI      2
#define ESKYLINK_TASK_PRI       1
#define CRTP_TX_TASK_PRI        2
#define CRTP_RX_TASK_PRI        2

//Task names
#define SYSTEM_TASK_NAME        "SYSTEM"
#define ADC_TASK_NAME           "ADC"
#define PM_TASK_NAME            "PWRMGNT"
#define CRTP_TX_TASK_NAME       "CRTP-TX"
#define CRTP_RX_TASK_NAME       "CRTP-RX"
#define CRTP_RXTX_TASK_NAME     "CRTP-RXTX"
#define LOG_TASK_NAME           "LOG"
#define MEM_TASK_NAME           "MEM"
#define PARAM_TASK_NAME         "PARAM"
#define STABILIZER_TASK_NAME    "STABILIZER"
#define NRF24LINK_TASK_NAME     "NRF24LINK"
#define ESKYLINK_TASK_NAME      "ESKYLINK"
#define SYSLINK_TASK_NAME       "SYSLINK"
#define USBLINK_TASK_NAME       "USBLINK"
//Task stack sizes
#define SYSTEM_TASK_STACKSIZE         (2* configMINIMAL_STACK_SIZE)
#define ADC_TASK_STACKSIZE            configMINIMAL_STACK_SIZE
#define PM_TASK_STACKSIZE             configMINIMAL_STACK_SIZE
#define CRTP_TX_TASK_STACKSIZE        configMINIMAL_STACK_SIZE
#define CRTP_RX_TASK_STACKSIZE        configMINIMAL_STACK_SIZE
#define CRTP_RXTX_TASK_STACKSIZE      configMINIMAL_STACK_SIZE
#define LOG_TASK_STACKSIZE            configMINIMAL_STACK_SIZE
#define MEM_TASK_STACKSIZE            configMINIMAL_STACK_SIZE
#define PARAM_TASK_STACKSIZE          configMINIMAL_STACK_SIZE
#define STABILIZER_TASK_STACKSIZE     (3 * configMINIMAL_STACK_SIZE)
#define NRF24LINK_TASK_STACKSIZE      configMINIMAL_STACK_SIZE
#define ESKYLINK_TASK_STACKSIZE       configMINIMAL_STACK_SIZE
#define SYSLINK_TASK_STACKSIZE        configMINIMAL_STACK_SIZE
#define USBLINK_TASK_STACKSIZE        configMINIMAL_STACK_SIZE

//The radio channel. From 0 to 125
#define RADIO_CHANNEL 80
#define RADIO_DATARATE RADIO_RATE_250K
#define RADIO_ADDRESS 0xE7E7E7E7E7ULL

// Define to force initialization of expansion board drivers. For test-rig and programming.
//#define FORCE_EXP_DETECT

//Debug defines
//#define BRUSHLESS_MOTORCONTROLLER
//#define ADC_OUTPUT_RAW_DATA
//#define UART_OUTPUT_TRACE_DATA
//#define UART_OUTPUT_RAW_DATA_ONLY
//#define IMU_OUTPUT_RAW_DATA_ON_UART
//#define T_LAUCH_MOTORS
//#define T_LAUCH_MOTOR_TEST
//#define MOTOR_RAMPUP_TEST
/**
 * \def ADC_OUTPUT_RAW_DATA
 * When defined the gyro data will be written to the UART channel.
 * The UART must be configured to run really fast, e.g. in 2Mb/s.
 */
//#define ADC_OUTPUT_RAW_DATA

#if defined(UART_OUTPUT_TRACE_DATA) && defined(ADC_OUTPUT_RAW_DATA)
#  error "Can't define UART_OUTPUT_TRACE_DATA and ADC_OUTPUT_RAW_DATA at the same time"
#endif

#if defined(UART_OUTPUT_TRACE_DATA) || defined(ADC_OUTPUT_RAW_DATA) || defined(IMU_OUTPUT_RAW_DATA_ON_UART)
#define UART_OUTPUT_RAW_DATA_ONLY
#endif

#if defined(UART_OUTPUT_TRACE_DATA) && defined(T_LAUNCH_ACC)
#  error "UART_OUTPUT_TRACE_DATA and T_LAUNCH_ACC doesn't work at the same time yet due to dma sharing..."
#endif

#endif /* CONFIG_H_ */
