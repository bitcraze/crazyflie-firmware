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
 * uart_syslink.h - uart syslink to nRF51 and raw access functions
 */
#ifndef UART_SYSLINK_H_
#define UART_SYSLINK_H_

#include <stdbool.h>

#include "crtp.h"
#include "eprintf.h"
#include "syslink.h"

#define UARTSLK_TYPE             USART6
#define UARTSLK_PERIF            RCC_APB2Periph_USART6
#define ENABLE_UARTSLK_RCC       RCC_APB2PeriphClockCmd
#define UARTSLK_IRQ              USART6_IRQn

#define UARTSLK_DMA_IRQ          DMA2_Stream7_IRQn
#define UARTSLK_DMA_IT_TC        DMA_IT_TC
#define UARTSLK_DMA_STREAM       DMA2_Stream7
#define UARTSLK_DMA_CH           DMA_Channel_5
#define UARTSLK_DMA_FLAG_TCIF    DMA_FLAG_TCIF7

#define UARTSLK_GPIO_PERIF       RCC_AHB1Periph_GPIOC
#define UARTSLK_GPIO_PORT        GPIOC
#define UARTSLK_GPIO_TX_PIN      GPIO_Pin_6
#define UARTSLK_GPIO_RX_PIN      GPIO_Pin_7
#define UARTSLK_GPIO_AF_TX_PIN   GPIO_PinSource6
#define UARTSLK_GPIO_AF_RX_PIN   GPIO_PinSource7
#define UARTSLK_GPIO_AF_TX       GPIO_AF_USART6
#define UARTSLK_GPIO_AF_RX       GPIO_AF_USART6

#define UARTSLK_TXEN_PERIF       RCC_AHB1Periph_GPIOA
#define UARTSLK_TXEN_PORT        GPIOA
#define UARTSLK_TXEN_PIN         GPIO_Pin_4
#define UARTSLK_TXEN_EXTI        EXTI_Line4

/**
 * Initialize the UART.
 *
 * @note Initialize CRTP link only if USE_CRTP_UART is defined
 */
void uartslkInit(void);

/**
 * Test the UART status.
 *
 * @return true if the UART is initialized
 */
bool uartslkTest(void);

/**
 * Get CRTP link data structure
 *
 * @return Address of the crtp link operations structure.
 */
struct crtpLinkOperations * uartslkGetLink();

/**
 * Get data from rx queue. Blocks until data is available.
 * @param[out] slp Pointer to a complete syslink packet
 */
void uartslkGetPacketBlocking(SyslinkPacket* slp);

/**
 * Sends raw data using a lock. Should be used from
 * exception functions and for debugging when a lot of data
 * should be transfered.
 * @param[in] size  Number of bytes to send
 * @param[in] data  Pointer to data
 */
void uartslkSendData(uint32_t size, uint8_t* data);

/**
 * Sends raw data using interrupts and blocking semaphore.
 * @param[in] size  Number of bytes to send
 * @param[in] data  Pointer to data
 */
void uartslkSendDataIsrBlocking(uint32_t size, uint8_t* data);

/**
 * Send a single character to the serial port using the uartslkSendData function.
 * @param[in] ch Character to print. Only the 8 LSB are used.
 * @return Character printed
 */
int uartslkPutchar(int ch);

/**
 * Sends raw data using DMA transfer.
 * @param[in] size  Number of bytes to send
 * @param[in] data  Pointer to data
 */
void uartslkSendDataDmaBlocking(uint32_t size, uint8_t* data);

/**
 * Interrupt service routine handling UART interrupts.
 */
void uartslkIsr(void);

/**
 * Interrupt service routine handling UART DMA interrupts.
 */
void uartslkDmaIsr(void);

void uartslkTxenFlowctrlIsr();

#endif /* UART_SYSLINK_H_ */
