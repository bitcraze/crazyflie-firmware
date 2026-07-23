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
 * console.h - Used to send console data to the client
 */

#ifndef CONSOLE_H_
#define CONSOLE_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "eprintf.h"

/**
 * Initialize the console
 */
void consoleInit(void);

bool consoleTest(void);

/**
 * Put a character to the console buffer
 *
 * @param ch character that shall be printed
 * @return The character casted to unsigned int or EOF in case of error
 */
int consolePutchar(int ch);

/**
 * Put a character to the console buffer
 *
 * @param ch character that shall be printed
 * @return The character casted to unsigned int or EOF in case of error
 *
 * @note This version can be called by interrup. In such case the internal
 * buffer is going to be used. If a task currently is printing or if the
 * interrupts prints too much the data will be ignored.
 */
int consolePutcharFromISR(int ch);

/**
 * Put a null-terminated string on the console buffer
 *
 * @param str Null terminated string
 * @return a nonnegative number on success, or EOF on error.
 */
int consolePuts(const char *str);

/**
 * Flush the console buffer
 */
void consoleFlush(void);

/**
 * Register a console source in the immutable boot-lifetime source catalog.
 *
 * @param path Null-terminated, valid UTF-8 source path. Colon-separated path
 *             segments must be non-empty, the encoded path must fit in one
 *             Console TOC response. The path is copied before this call
 *             returns, so the caller retains ownership of its storage.
 * @return The nonnegative 8-bit source ID. Registering the same path again
 *         returns its existing ID. Returns -1 when the path is invalid, the
 *         catalog is full, or registration has been frozen.
 */
int consoleSourceRegister(const char *path);

/**
 * Permanently close source registration for the current firmware lifetime.
 *
 * This makes the catalog invariant before clients can query its source IDs and
 * CRC. Calls after the first one have no additional effect.
 */
void consoleSourceFreeze(void);

/**
 * Check whether a ground client has enabled a console source.
 *
 * @param sourceId Boot-lifetime source ID returned by consoleSourceRegister().
 * @return true when sourceId exists and is enabled, otherwise false.
 */
bool consoleSourceIsEnabled(uint8_t sourceId);

/**
 * Try to send one binary chunk for an enabled source without blocking.
 *
 * The bytes are canonically UTF-8 console data, but a chunk may split a code
 * point or contain invalid bytes. The data is copied before this call returns.
 *
 * @param sourceId Boot-lifetime source ID returned by consoleSourceRegister().
 * @param data Non-NULL buffer containing the chunk to send.
 * @param length Number of bytes in data; must fit beside the source ID in one
 *               CRTP packet (at most CRTP_MAX_DATA_SIZE - 1).
 * @return true when the chunk was accepted by the CRTP transmit queue;
 *         false when the source is invalid or disabled, data is NULL, length
 *         is too large, or the queue cannot accept the packet immediately.
 */
bool consoleSourceSend(uint8_t sourceId, const uint8_t *data, size_t length);

/**
 * Macro implementing consolePrintf with eprintf
 *
 * @param FMT String format
 * @param ... Parameters to print
 */
#define consolePrintf(FMT, ...) eprintf(consolePutchar, FMT, ## __VA_ARGS__)

#endif /*CONSOLE_H_*/
