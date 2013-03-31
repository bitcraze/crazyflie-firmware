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
 * uart.h - uart initialization and functions header file
 */
#ifndef UART_H_
#define UART_H_

#include "crtp.h"
#include "eprintf.h"

#define UART_TYPE       USART3
#define UART_PERIF      RCC_APB1Periph_USART3

#define UART_DMA_IRQ    DMA1_Channel2_IRQn
#define UART_DMA_IT_TC  DMA1_IT_TC2
#define UART_DMA_CH     DMA1_Channel2

#define UART_GPIO_PERIF RCC_APB2Periph_GPIOB
#define UART_GPIO_PORT  GPIOB
#define UART_GPIO_TX    GPIO_Pin_10
#define UART_GPIO_RX    GPIO_Pin_11



/**
 * Initialize the UART.
 */
void uartInit(void);

/**
 * Receive packet from UART. Blocking.
 *
 * @param[out] p The received CRTP packet
 */
int uartReceiveCRTPPacket(CRTPPacket *p);

/**
 * Send CRTP packet.
 *
 * @param[in] p The CRTP packet that shall be sent.
 */
void uartSendCRTPPacket(CRTPPacket *p);

/**
 * Sends raw data using a spinnlock. Should be used from
 * exception functions and for debugging when a lot of data
 * should be transfered.
 * @param[in] size  Number of bytes to send
 * @param[in] data  Pointer to data
 */
void uartSendData(uint32_t size, uint8_t* data);

/**
 * Send a single character to the serial port using the uartSendData function.
 * @param[in] ch Character to print. Only the 8 LSB are used.
 * @return Character printed
 */
int uartPutchar(int ch);

/**
 * Uart printf macro that uses eprintf
 * @param[in] FMT String format
 * @param[in] ... Parameters to print
 */
#define uartPrintf(FMT, ...) eprintf(uartPutchar, FMT, ## __VA_ARGS__)

/**
 * Sends raw data using DMA transfer. Should be used from
 * exception functions and for debugging when a lot of data
 * should be transfered.
 * @param[in] size  Number of bytes to send
 * @param[in] data  Pointer to data
 */
void uartSendDataDma(uint32_t size, uint8_t* data);

/**
 * Interrupt service routine handling UART interrupts.
 */
void uartIsr(void);

/**
 * Interrupt service routine handling UART DMA interrupts.
 */
void uartDmaIsr(void);

#endif /* UART_H_ */
