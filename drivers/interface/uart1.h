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
 * uart1.h - uart1 driver for deck port
 */
#ifndef UART1_H_
#define UART1_H_

#include <stdbool.h>
#include "eprintf.h"

#define UART1_BAUDRATE         115200

#define UART1_TYPE             UART4
#define UART1_PERIF            RCC_APB1Periph_UART4
#define ENABLE_UART1_RCC       RCC_APB1PeriphClockCmd
#define UART1_IRQ              UART4_IRQn

#define UART1_DMA_IRQ          DMA1_Channel2_IRQn
#define UART1_DMA_IT_TC        DMA1_IT_TC2
#define UART1_DMA_CH           DMA1_Channel2

#define UART1_GPIO_PERIF       RCC_AHB1Periph_GPIOC
#define UART1_GPIO_PORT        GPIOC
#define UART1_GPIO_TX_PIN      GPIO_Pin_10
#define UART1_GPIO_RX_PIN      GPIO_Pin_11
#define UART1_GPIO_AF_TX_PIN   GPIO_PinSource10
#define UART1_GPIO_AF_RX_PIN   GPIO_PinSource11
#define UART1_GPIO_AF_TX       GPIO_AF_UART4
#define UART1_GPIO_AF_RX       GPIO_AF_UART4

/**
 * Initialize the UART.
 */
void uart1Init(void);

/**
 * Test the UART status.
 *
 * @return true if the UART is initialized
 */
bool uart1Test(void);

/**
 * Sends raw data using a lock. Should be used from
 * exception functions and for debugging when a lot of data
 * should be transfered.
 * @param[in] size  Number of bytes to send
 * @param[in] data  Pointer to data
 */
void uart1SendData(uint32_t size, uint8_t* data);

/**
 * Send a single character to the serial port using the uartSendData function.
 * @param[in] ch Character to print. Only the 8 LSB are used.
 *
 * @return Character printed
 */
int uart1Putchar(int ch);

/**
 * Uart printf macro that uses eprintf
 * @param[in] FMT String format
 * @param[in] ... Parameters to print
 *
 * @note If UART Crtp link is activated this function does nothing
 */
#define uart1Printf(FMT, ...) eprintf(uart1Putchar, FMT, ## __VA_ARGS__)

#endif /* UART1_H_ */
