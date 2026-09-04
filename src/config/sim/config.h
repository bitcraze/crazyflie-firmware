/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2022 Bitcraze AB
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
 * config.h for CONFIG_PLATFORM_SIM (Simmyflie) -- a copy of the mainline
 * src/config/config.h's portable content (task names/priorities/stack
 * sizes -- crtp.c and friends reference these directly), picked up via
 * include-path precedence instead of the mainline file (see the Makefile's
 * PLATFORM_SIM INCLUDES comment, same trick as src/config/sim/FreeRTOSConfig.h).
 *
 * Deliberately dropped from the mainline version:
 * - #include "nrf24l01.h"/"trace.h"/"usec_time.h": hardware drivers, not
 *   built under PLATFORM_SIM.
 * - configGENERATE_RUN_TIME_STATS / portCONFIGURE_TIMER_FOR_RUN_TIME_STATS /
 *   portGET_RUN_TIME_COUNTER_VALUE: the FreeRTOS POSIX port's own
 *   portmacro.h already defines these (no-op / ulPortGetRunTime()) --
 *   redefining them here was a real conflict, same class of issue as the
 *   FreeRTOSConfig.h one noted in Phase 1's writeup.
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#define CONFIG_BLOCK_ADDRESS    (2048 * (64-1))
#define MCU_ID_ADDRESS          0x1FFF7A10
#define MCU_FLASH_SIZE_ADDRESS  0x1FFF7A22
#ifndef FREERTOS_HEAP_SIZE
  #define FREERTOS_HEAP_SIZE      30000
#endif
#define FREERTOS_MIN_STACK_SIZE 150       // M4-FPU register setup is bigger so stack needs to be bigger
#define FREERTOS_MCU_CLOCK_HZ   168000000

// Task priorities. Higher number higher priority
#define PASSTHROUGH_TASK_PRI      5
#define STABILIZER_TASK_PRI       5
#define RATE_SUPERVISOR_TASK_PRI  5
#define SENSORS_TASK_PRI          4
#define ADC_TASK_PRI              3
#define FLOW_TASK_PRI             3
#define MULTIRANGER_TASK_PRI      3
#define SYSTEM_TASK_PRI           2
#define CRTP_TX_TASK_PRI          2
#define CRTP_RX_TASK_PRI          2
#define EXTRX_TASK_PRI            2
#define ZRANGER_TASK_PRI          2
#define ZRANGER2_TASK_PRI         2
#define LOG_TASK_PRI              1
#define MEM_TASK_PRI              1
#define PARAM_TASK_PRI            1
#define PROXIMITY_TASK_PRI        0
#define PM_TASK_PRI               0
#define USDLOG_TASK_PRI           1
#define USDWRITE_TASK_PRI         0
#define PCA9685_TASK_PRI          2
#define CMD_HIGH_LEVEL_TASK_PRI   2
#define BQ_OSD_TASK_PRI           1
#define GTGPS_DECK_TASK_PRI       1
#define LIGHTHOUSE_TASK_PRI       3
#define LPS_DECK_TASK_PRI         3
#define OA_DECK_TASK_PRI          3
#define UART1_TEST_TASK_PRI       1
#define UART2_TEST_TASK_PRI       1
#define KALMAN_TASK_PRI           2
#define ERROR_UKF_TASK_PRI        2
#define LEDSEQCMD_TASK_PRI        1
#define FLAPPERDECK_TASK_PRI      2
#define SYSLINK_TASK_PRI          3
#define USBLINK_TASK_PRI          3
#define ACTIVE_MARKER_TASK_PRI    3
#define AI_DECK_TASK_PRI          1
#define UART2_TASK_PRI            3
#define CRTP_SRV_TASK_PRI         0
#define PLATFORM_SRV_TASK_PRI     0
#define COLORLED_TASK_PRIO        1
#define WORKER_TASK_PRI           1
#define SUPERVISOR_TASK_PRI       1

// Task names
#define SYSTEM_TASK_NAME          "SYSTEM"
#define LEDSEQCMD_TASK_NAME       "LEDSEQCMD"
#define ADC_TASK_NAME             "ADC"
#define PM_TASK_NAME              "PWRMGNT"
#define CRTP_TX_TASK_NAME         "CRTP-TX"
#define CRTP_RX_TASK_NAME         "CRTP-RX"
#define CRTP_RXTX_TASK_NAME       "CRTP-RXTX"
#define LOG_TASK_NAME             "LOG"
#define MEM_TASK_NAME             "MEM"
#define PARAM_TASK_NAME           "PARAM"
#define SENSORS_TASK_NAME         "SENSORS"
#define STABILIZER_TASK_NAME      "STABILIZER"
#define RATE_SUPERVISOR_TASK_NAME "RATE_SUPERVISOR"
#define NRF24LINK_TASK_NAME       "NRF24LINK"
#define ESKYLINK_TASK_NAME        "ESKYLINK"
#define SYSLINK_TASK_NAME         "SYSLINK"
#define USBLINK_TASK_NAME         "USBLINK"
#define PROXIMITY_TASK_NAME       "PROXIMITY"
#define EXTRX_TASK_NAME           "EXTRX"
#define UART_RX_TASK_NAME         "UART"
#define ZRANGER_TASK_NAME         "ZRANGER"
#define ZRANGER2_TASK_NAME        "ZRANGER2"
#define FLOW_TASK_NAME            "FLOW"
#define USDLOG_TASK_NAME          "USDLOG"
#define USDWRITE_TASK_NAME        "USDWRITE"
#define PCA9685_TASK_NAME         "PCA9685"
#define CMD_HIGH_LEVEL_TASK_NAME  "CMDHL"
#define MULTIRANGER_TASK_NAME     "MR"
#define BQ_OSD_TASK_NAME          "BQ_OSDTASK"
#define GTGPS_DECK_TASK_NAME      "GTGPS"
#define LIGHTHOUSE_TASK_NAME      "LH"
#define LPS_DECK_TASK_NAME        "LPS"
#define OA_DECK_TASK_NAME         "OA"
#define UART1_TEST_TASK_NAME      "UART1TEST"
#define UART2_TEST_TASK_NAME      "UART2TEST"
#define KALMAN_TASK_NAME          "KALMAN"
#define ERROR_UKF_TASK_NAME       "ERROR_UKF"
#define ACTIVE_MARKER_TASK_NAME   "ACTIVEMARKER-DECK"
#define AI_DECK_GAP_TASK_NAME     "AI-DECK-GAP"
#define AIDECK_ESP_TX_TASK_NAME   "AI-DECK ESP TX"
#define AIDECK_ESP_RX_TASK_NAME   "AI-DECK ESP RX"
#define UART2_TASK_NAME           "UART2"
#define CRTP_SRV_TASK_NAME        "CRTP-SRV"
#define PLATFORM_SRV_TASK_NAME    "PLATFORM-SRV"
#define PASSTHROUGH_TASK_NAME     "PASSTHROUGH"
#define CPX_RT_UART_TASK_NAME     "ROUTER FROM UART2"
#define CPX_RT_INT_TASK_NAME      "ROUTER FROM INTERNAL"
#define CPX_TASK_NAME             "CPX"
#define APP_TASK_NAME             "APP"
#define FLAPPERDECK_TASK_NAME     "FLAPPERDECK"
#define COLORLED_TASK_NAME        "COLORLED-DECK"
#define WORKER_TASK_NAME          "WORKER"
#define SUPERVISOR_TASK_NAME      "SUPERVISOR"

//Task stack sizes
#define SYSTEM_TASK_STACKSIZE           (2* configMINIMAL_STACK_SIZE)
#define LEDSEQCMD_TASK_STACKSIZE        configMINIMAL_STACK_SIZE
#define ADC_TASK_STACKSIZE              configMINIMAL_STACK_SIZE
#define PM_TASK_STACKSIZE               configMINIMAL_STACK_SIZE
#define CRTP_TX_TASK_STACKSIZE          configMINIMAL_STACK_SIZE
#define CRTP_RX_TASK_STACKSIZE          (2* configMINIMAL_STACK_SIZE)
#define CRTP_RXTX_TASK_STACKSIZE        configMINIMAL_STACK_SIZE
#define LOG_TASK_STACKSIZE              (2 * configMINIMAL_STACK_SIZE)
#define MEM_TASK_STACKSIZE              (2 * configMINIMAL_STACK_SIZE)
#define PARAM_TASK_STACKSIZE            (2 * configMINIMAL_STACK_SIZE)
#define SENSORS_TASK_STACKSIZE          (2 * configMINIMAL_STACK_SIZE)
#define STABILIZER_TASK_STACKSIZE       (3 * configMINIMAL_STACK_SIZE)
#define RATE_SUPERVISOR_TASK_STACKSIZE  configMINIMAL_STACK_SIZE
#define NRF24LINK_TASK_STACKSIZE        configMINIMAL_STACK_SIZE
#define ESKYLINK_TASK_STACKSIZE         configMINIMAL_STACK_SIZE
#define SYSLINK_TASK_STACKSIZE          (2 * configMINIMAL_STACK_SIZE)
#define USBLINK_TASK_STACKSIZE          configMINIMAL_STACK_SIZE
#define PROXIMITY_TASK_STACKSIZE        configMINIMAL_STACK_SIZE
#define EXTRX_TASK_STACKSIZE            configMINIMAL_STACK_SIZE
#define UART_RX_TASK_STACKSIZE          configMINIMAL_STACK_SIZE
#define ZRANGER_TASK_STACKSIZE          (2 * configMINIMAL_STACK_SIZE)
#define ZRANGER2_TASK_STACKSIZE         (2 * configMINIMAL_STACK_SIZE)
#define FLOW_TASK_STACKSIZE             (2 * configMINIMAL_STACK_SIZE)
#define USDLOG_TASK_STACKSIZE           (2 * configMINIMAL_STACK_SIZE)
#define USDWRITE_TASK_STACKSIZE         (3 * configMINIMAL_STACK_SIZE)
#define PCA9685_TASK_STACKSIZE          (2 * configMINIMAL_STACK_SIZE)
#define CMD_HIGH_LEVEL_TASK_STACKSIZE   (2 * configMINIMAL_STACK_SIZE)
#define MULTIRANGER_TASK_STACKSIZE      (2 * configMINIMAL_STACK_SIZE)
#define ACTIVEMARKER_TASK_STACKSIZE     configMINIMAL_STACK_SIZE
#define AI_DECK_TASK_STACKSIZE          configMINIMAL_STACK_SIZE
#define UART2_TASK_STACKSIZE            configMINIMAL_STACK_SIZE
#define CRTP_SRV_TASK_STACKSIZE         configMINIMAL_STACK_SIZE
#define PLATFORM_SRV_TASK_STACKSIZE     configMINIMAL_STACK_SIZE
#define PASSTHROUGH_TASK_STACKSIZE      (2 * configMINIMAL_STACK_SIZE)
#define BQ_OSD_TASK_STACKSIZE           configMINIMAL_STACK_SIZE
#define GTGPS_DECK_TASK_STACKSIZE       configMINIMAL_STACK_SIZE
#define UART1_TEST_TASK_STACKSIZE       configMINIMAL_STACK_SIZE
#define UART2_TEST_TASK_STACKSIZE       configMINIMAL_STACK_SIZE
#define LIGHTHOUSE_TASK_STACKSIZE       (2 * configMINIMAL_STACK_SIZE)
#define LPS_DECK_STACKSIZE              (3 * configMINIMAL_STACK_SIZE)
#define OA_DECK_TASK_STACKSIZE          (2 * configMINIMAL_STACK_SIZE)
#define KALMAN_TASK_STACKSIZE           (3 * configMINIMAL_STACK_SIZE)
#define FLAPPERDECK_TASK_STACKSIZE      (2 * configMINIMAL_STACK_SIZE)
#define ERROR_UKF_TASK_STACKSIZE        (4 * configMINIMAL_STACK_SIZE)
#define COLORLED_TASK_STACKSIZE         configMINIMAL_STACK_SIZE
#define WORKER_TASK_STACKSIZE           (2 * configMINIMAL_STACK_SIZE)
#define SUPERVISOR_TASK_STACKSIZE       (2 * configMINIMAL_STACK_SIZE)

#define BAT_LOADING_SAG_THRESHOLD  0.70f

#endif /* CONFIG_H_ */
