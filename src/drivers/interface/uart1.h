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
#include <stdint.h>
#include "FreeRTOS.h"
#include "eprintf.h"

#define UART1_BAUDRATE           9600
#define UART1_DATA_TIMEOUT_MS    1000
#define UART1_DATA_TIMEOUT_TICKS (UART1_DATA_TIMEOUT_MS / portTICK_RATE_MS)

#define UART1_TYPE             USART3
#define UART1_PERIF            RCC_APB1Periph_USART3
#define ENABLE_UART1_RCC       RCC_APB1PeriphClockCmd
#define UART1_IRQ              USART3_IRQn

#define UART1_DMA_IRQ          DMA1_Stream3_IRQn
#define UART1_DMA_IT_TC        DMA_IT_TC4
#define UART1_DMA_STREAM       DMA1_Stream3
#define UART1_DMA_CH           DMA_Channel_4
#define UART1_DMA_FLAG_TCIF    DMA_FLAG_TCIF3

#define UART1_GPIO_PERIF       RCC_AHB1Periph_GPIOC
#define UART1_GPIO_PORT        GPIOC
#define UART1_GPIO_TX_PIN      GPIO_Pin_10
#define UART1_GPIO_RX_PIN      GPIO_Pin_11
#define UART1_GPIO_AF_TX_PIN   GPIO_PinSource10
#define UART1_GPIO_AF_RX_PIN   GPIO_PinSource11
#define UART1_GPIO_AF_TX       GPIO_AF_USART3
#define UART1_GPIO_AF_RX       GPIO_AF_USART3

typedef enum {
    uart1ParityNone, uart1ParityEven, uart1ParityOdd
} uart1Parity_t;

typedef void (*uart1RxCallback_t)(uint8_t byte,
                                  BaseType_t *higher_priority_task_woken);
typedef void (*uart1ErrorCallback_t)(BaseType_t *higher_priority_task_woken);

/**
 * Initialize the UART with parity None
 */
void uart1Init(const uint32_t baudrate);

/**
 * Initialize the UART with custom parity
 */
void uart1InitWithParity(const uint32_t baudrate, const uart1Parity_t parity);

/**
 * Change the baudrate of an already-initialized UART.
 *
 * Only reprograms the USART baudrate (parity None, 8N1); the RX queue, DMA and
 * interrupt configuration set up by uart1Init() are left intact. Use this to
 * switch baudrate mid-session instead of re-running uart1Init(), which would
 * tear down and recreate the RX queue. Any bytes in flight across the switch
 * should be drained by the caller.
 *
 * @param[in] baudrate  The new baudrate
 */
void uart1SetBaudrate(const uint32_t baudrate);

/**
 * Test the UART status.
 *
 * @return true if the UART is initialized
 */
bool uart1Test(void);

void uart1SetRxCallback(uart1RxCallback_t callback);

void uart1ClearRxCallback(void);

void uart1SetErrorCallback(uart1ErrorCallback_t callback);

void uart1ClearErrorCallback(void);

void uart1SetCallbacks(uart1RxCallback_t rx_callback,
                       uart1ErrorCallback_t error_callback);

void uart1ClearCallbacks(void);

/**
 * Read a byte of data from incoming queue with a timeout
 * @param[out] c  Read byte
 * @param[in] timeoutTicks The timeout in sys ticks
 * @return true if data, false if timeout was reached.
 */
bool uart1GetDataWithTimeout(uint8_t *c, const uint32_t timeoutTicks);

/**
 * Read a byte of data from incoming queue with a timeout defined by UART1_DATA_TIMEOUT_MS
 * @param[out] c  Read byte
 * @return true if data, false if timeout was reached.
 */
bool uart1GetDataWithDefaultTimeout(uint8_t *c);

/**
 * Get data from the UART connection. Blocking until the amount of
 * data has been read
 *
 * @param[in] size  Number of bytes to read
 * @param[out] data  Pointer to data
 *
 * @return number of bytes read
 */
void uart1GetBytesWithDefaultTimeout(uint32_t size, uint8_t* data);

/**
 * @brief Get the number of bytes available in the UART1 in queue
 *
 * @return uint32_t  Number of bytes available
 */
uint32_t uart1bytesAvailable();

/**
 * @brief Get the maximum number of bytes that can be stored in the UART1 in queue
 *
 * @return uint32_t  The maximum length of the in queue
 */
uint32_t uart1QueueMaxLength();

/**
 * Sends raw data using a lock. Should be used from
 * exception functions and for debugging when a lot of data
 * should be transfered.
 * @param[in] size  Number of bytes to send
 * @param[in] data  Pointer to data
 */
void uart1SendData(uint32_t size, uint8_t* data);

/**
 * Sends all bytes using polling and waits until the final stop bit has left
 * the USART shift register.
 *
 * The STM32 USART exposes no asynchronous TX-error indication in the 8N1,
 * no-flow-control configuration used by UART1. This function can therefore
 * report invalid/uninitialized submission, and otherwise returns only after
 * the hardware Transmission Complete flag is set.
 *
 * @param[in] size  Number of bytes to send
 * @param[in] data  Bytes to send; may be NULL only when size is zero
 * @param[in] timeoutTicks  Maximum ticks for submission and completion
 * @return true after complete local transmission, false on invalid submission
 *         or timeout with potentially partial transmission
 */
bool uart1SendDataWaitComplete(uint32_t size,
                               const uint8_t* data,
                               uint32_t timeoutTicks);

/**
 * Sends raw data using DMA transfer.
 * @param[in] size  Number of bytes to send
 * @param[in] data  Pointer to data
 */
void uart1SendDataDmaBlocking(uint32_t size, uint8_t* data);

/**
 * Send `size` bytes — DMA when this build has it, polling otherwise.
 *
 * When the DMA path is taken the transfer reads directly from `data`, so on
 * STM32F4 the buffer must reside in DMA-reachable memory (Flash or regular
 * RAM); CCM RAM (0x10000000-0x1000FFFF) is not accessible by DMA and will
 * trip an assert (see ASSERT_DMA_SAFE). The buffer must also stay valid until
 * this call returns (the DMA path blocks until the transfer completes).
 *
 * @param[in] size  Number of bytes to send
 * @param[in] data  Pointer to data (must be DMA-safe, see above)
 */
void uart1SendDmaIfAvailable(uint32_t size, uint8_t* data);

/**
 * Send a single character to the serial port using the uartSendData function.
 * @param[in] ch Character to print. Only the 8 LSB are used.
 *
 * @return Character printed
 */
int uart1Putchar(int ch);

void uart1Getchar(char * ch);

/**
 * Returns true if an overrun condition has happened since initialization or
 * since the last call to this function.
 *
 * @return true if an overrun condition has happened
 */
bool uart1DidOverrun();

/**
 * Uart printf macro that uses eprintf
 * @param[in] FMT String format
 * @param[in] ... Parameters to print
 *
 * @note If UART Crtp link is activated this function does nothing
 */
#define uart1Printf(FMT, ...) eprintf(uart1Putchar, FMT, ## __VA_ARGS__)

#endif /* UART1_H_ */
