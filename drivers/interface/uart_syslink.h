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
 * uart.h - uart CRTP link and raw access functions
 */
#ifndef UART_H_
#define UART_H_

#include <stdbool.h>

#include "crtp.h"
#include "eprintf.h"

#define NRF_UART

#ifdef NRF_UART
  #define UART_TYPE             USART6
  #define UART_PERIF            RCC_APB2Periph_USART6
  #define ENABLE_UART_RCC       RCC_APB2PeriphClockCmd
  #define UART_IRQ              USART6_IRQn

  #define UART_DMA_IRQ          DMA2_Stream7_IRQn
  #define UART_DMA_IT_TC        DMA_IT_TC
  #define UART_DMA_STREAM       DMA2_Stream7
  #define UART_DMA_CH           DMA_Channel_5
  #define UART_DMA_FLAG_TCIF    DMA_FLAG_TCIF7


  #define UART_GPIO_PERIF       RCC_AHB1Periph_GPIOC
  #define UART_GPIO_PORT        GPIOC
  #define UART_GPIO_TX_PIN      GPIO_Pin_6
  #define UART_GPIO_RX_PIN      GPIO_Pin_7
  #define UART_GPIO_AF_TX_PIN   GPIO_PinSource6
  #define UART_GPIO_AF_RX_PIN   GPIO_PinSource7
  #define UART_GPIO_AF_TX       GPIO_AF_USART6
  #define UART_GPIO_AF_RX       GPIO_AF_USART6

  #define UART_TXEN_PERIF       RCC_AHB1Periph_GPIOA
  #define UART_TXEN_PORT        GPIOA
  #define UART_TXEN_PIN         GPIO_Pin_4
  #define UART_TXEN_EXTI        EXTI_Line4
#elif defined (EXT_UART1)
  #define UART_TYPE             UART4
  #define UART_PERIF            RCC_APB1Periph_UART4
  #define ENABLE_UART_RCC       RCC_APB1PeriphClockCmd
  #define UART_IRQ              UART4_IRQn

  #define UART_DMA_IRQ          DMA1_Channel2_IRQn
  #define UART_DMA_IT_TC        DMA1_IT_TC2
  #define UART_DMA_CH           DMA1_Channel2

  #define UART_GPIO_PERIF       RCC_AHB1Periph_GPIOC
  #define UART_GPIO_PORT        GPIOC
  #define UART_GPIO_TX_PIN      GPIO_Pin_10
  #define UART_GPIO_RX_PIN      GPIO_Pin_11
  #define UART_GPIO_AF_TX_PIN   GPIO_PinSource10
  #define UART_GPIO_AF_RX_PIN   GPIO_PinSource11
  #define UART_GPIO_AF_TX       GPIO_AF_UART4
  #define UART_GPIO_AF_RX       GPIO_AF_UART4
#else
  #define UART_TYPE             USART2
  #define UART_PERIF            RCC_APB1Periph_USART2
  #define ENABLE_UART_RCC       RCC_APB1PeriphClockCmd
  #define UART_IRQ              USART2_IRQn

  #define UART_DMA_IRQ          DMA1_Channel2_IRQn
  #define UART_DMA_IT_TC        DMA1_IT_TC2
  #define UART_DMA_CH           DMA1_Channel2

  #define UART_GPIO_PERIF       RCC_AHB1Periph_GPIOA
  #define UART_GPIO_PORT        GPIOA
  #define UART_GPIO_TX_PIN      GPIO_Pin_2
  #define UART_GPIO_RX_PIN      GPIO_Pin_3
  #define UART_GPIO_AF_TX_PIN   GPIO_PinSource2
  #define UART_GPIO_AF_RX_PIN   GPIO_PinSource3
  #define UART_GPIO_AF_TX       GPIO_AF_USART2
  #define UART_GPIO_AF_RX       GPIO_AF_USART2
#endif

/**
 * Initialize the UART.
 *
 * @note Initialize CRTP link only if USE_CRTP_UART is defined
 */
void uartInit(void);

/**
 * Test the UART status.
 *
 * @return true if the UART is initialized
 */
bool uartTest(void);

/**
 * Get CRTP link data structure
 *
 * @return Address of the crtp link operations structure.
 */
struct crtpLinkOperations * uartGetLink();

/**
 * Get data from rx queue with timeout.
 * @param[out] c  Byte of data
 *
 * @return true if byte received, false if timout reached.
 */
bool uartGetDataWithTimout(uint8_t *c);

/**
 * Sends raw data using a lock. Should be used from
 * exception functions and for debugging when a lot of data
 * should be transfered.
 * @param[in] size  Number of bytes to send
 * @param[in] data  Pointer to data
 *
 * @note If UART Crtp link is activated this function does nothing
 */
void uartSendData(uint32_t size, uint8_t* data);

/**
 * Sends raw data using interrupts and blocking semaphore.
 * @param[in] size  Number of bytes to send
 * @param[in] data  Pointer to data
 */
void uartSendDataIsrBlocking(uint32_t size, uint8_t* data);

/**
 * Send a single character to the serial port using the uartSendData function.
 * @param[in] ch Character to print. Only the 8 LSB are used.
 * @return Character printed
 *
 * @note If UART Crtp link is activated this function does nothing
 */
int uartPutchar(int ch);

/**
 * Uart printf macro that uses eprintf
 * @param[in] FMT String format
 * @param[in] ... Parameters to print
 *
 * @note If UART Crtp link is activated this function does nothing
 */
#define uartPrintf(FMT, ...) eprintf(uartPutchar, FMT, ## __VA_ARGS__)

/**
 * Sends raw data using DMA transfer. Should be used from
 * exception functions and for debugging when a lot of data
 * should be transfered.
 * @param[in] size  Number of bytes to send
 * @param[in] data  Pointer to data
 *
 * @note If UART Crtp link is activated this function does nothing
 */
void uartSendDataDmaBlocking(uint32_t size, uint8_t* data);

/**
 * Interrupt service routine handling UART interrupts.
 */
void uartIsr(void);

/**
 * Interrupt service routine handling UART DMA interrupts.
 */
void uartDmaIsr(void);

void uartTxenFlowctrlIsr();

#endif /* UART_H_ */
