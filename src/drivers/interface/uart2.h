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
 * uart2.h - uart2 driver for deck port
 */
#pragma once

#include <stdbool.h>
#include "eprintf.h"
#include "autoconf.h"

#define UART2_DATA_TIMEOUT_MS    1000
#define UART2_DATA_TIMEOUT_TICKS (UART2_DATA_TIMEOUT_MS / portTICK_RATE_MS)

#define UART2_TYPE             USART2
#define UART2_PERIF            RCC_APB1Periph_USART2
#define ENABLE_UART2_RCC       RCC_APB1PeriphClockCmd
#define UART2_IRQ              USART2_IRQn

#define UART2_DMA_BUFFER_SIZE 128
#define UART2_DMA_IRQ           DMA1_Stream6_IRQn
#define UART2_DMA_IT_TC         DMA_IT_TC4
#define UART2_DMA_STREAM        DMA1_Stream6
#define UART2_DMA_CH            DMA_Channel_4
#define UART2_DMA_FLAG_TCIF     DMA_FLAG_TCIF6

#define UART2_GPIO_PERIF       RCC_AHB1Periph_GPIOA
#define UART2_GPIO_PORT        GPIOA
#define UART2_GPIO_TX_PIN      GPIO_Pin_2
#define UART2_GPIO_RX_PIN      GPIO_Pin_3
#define UART2_GPIO_AF_TX_PIN   GPIO_PinSource2
#define UART2_GPIO_AF_RX_PIN   GPIO_PinSource3
#define UART2_GPIO_AF_TX       GPIO_AF_USART2
#define UART2_GPIO_AF_RX       GPIO_AF_USART2

#define UART2_RX_QUEUE_LENGTH 128

/**
 * Initialize the UART.
 */
void uart2Init(const uint32_t baudrate);

/**
 * Test the UART status.
 *
 * @return true if the UART is initialized
 */
bool uart2Test(void);

/**
 * Sends raw data using a lock. Should be used from
 * exception functions and for debugging when a lot of data
 * should be transfered.
 * @param[in] size  Number of bytes to send
 * @param[in] data  Pointer to data
 */
void uart2SendData(uint32_t size, uint8_t* data);

/**
 * Sends raw data using DMA transfer.
 * @param[in] size  Number of bytes to send
 * @param[in] data  Pointer to data
 */
void uart2SendDataDmaBlocking(uint32_t size, uint8_t* data);

/**
 * Send a single character to the serial port using the uartSendData function.
 * @param[in] ch Character to print. Only the 8 LSB are used.
 *
 * @return Character printed
 */
int uart2Putchar(int ch);

/**
 * Get data from the UART. Blocking until the amount of
 * data has been read
 *
 * @param[in] size  Number of bytes to read
 * @param[out] buffer  Pointer to data
 * 
 * @return number of bytes read
 */
int uart2GetData(size_t size, uint8_t * buffer);

/**
 * Get data from the UART. Blocking until the amount of
 * data has been read or the timeout occurs.
 *
 * @param[in] size  Number of bytes to read
 * @param[out] buffer  Pointer to data
 * @param[in] timeoutTicks timeout in ticks
 * 
 * @return number of bytes read
 */
int uart2GetDataWithTimeout(size_t size, uint8_t * buffer, const uint32_t timeoutTicks);

/**
 * Read a byte of data from the UART with a timeout
 * @param[out] c  Read byte
 * @param[in] timeoutTicks The timeout in sys ticks
 * @return true if data, false if timeout was reached.
 */
bool uart2GetCharWithTimeout(uint8_t *c, const uint32_t timeoutTicks);

/**
 * Read a byte of data from the UART with a timeout defined by UART2_DATA_TIMEOUT_MS
 * @param[out] c  Read byte
 * @return true if data, false if timeout was reached.
 */
bool uart2GetCharWithDefaultTimeout(uint8_t *c);

/**
 * Read a byte of data from the UART. Blocking until a byte
 * can be read. 
 * 
 * @param[out] ch pointer to data
 */
void uart2Getchar(char * ch);

/**
 * Returns true if an overrun condition has happened since initialization or
 * since the last call to this function.
 *
 * @return true if an overrun condition has happened
 */
bool uart2DidOverrun();

/**
 * Uart printf macro that uses eprintf
 * @param[in] FMT String format
 * @param[in] ... Parameters to print
 *
 * @note If UART Crtp link is activated this function does nothing
 */
#define uart2Printf(FMT, ...) eprintf(uart2Putchar, FMT, ## __VA_ARGS__)
