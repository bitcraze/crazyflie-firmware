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
 * ConsoleSource identifies a stream in the CRTP console source catalog. The
 * caller provides the node and its NUL-terminated path. Set only path before
 * registration; Console initializes the other fields on success. Do not
 * change any field after registration. The node and its path must remain valid
 * for the full firmware lifetime, and the path must not change. Console never
 * frees either one.
 */
typedef struct ConsoleSource {
  const char *path;
  struct ConsoleSource *next;  ///< Console-owned list link.
  uint8_t id;                 ///< Console-owned wire ID.
  uint8_t pathLength;         ///< Console-owned encoded path length.
  volatile bool enabled;      ///< Console-owned runtime state.
} ConsoleSource;

/**
 * Register a caller-provided node in the boot-lifetime source catalog.
 *
 * source->path must be valid UTF-8 with non-empty colon-separated segments
 * and at most 27 encoded bytes. On success, source becomes the handle for the
 * functions below. Failure does not change the node or catalog. A node from
 * an earlier successful call remains registered if it is passed again.
 * Call from task context. Console protects the catalog and enabled state with
 * critical sections.
 *
 * @return 0 on success; EINVAL for a NULL source or invalid path, EEXIST for
 *         a previously registered node or path, ENOSPC for 255 entries, or
 *         EBUSY after registration has been frozen. These are positive errno
 *         values from the firmware's C library.
 */
int consoleSourceRegister(ConsoleSource *source);

/**
 * Permanently close source registration for the current firmware lifetime.
 *
 * Clients can then query stable source IDs and the catalog CRC. Later calls
 * have no additional effect.
 */
void consoleSourceFreeze(void);

/**
 * Check whether a client has enabled a console source.
 *
 * @param source Successfully registered node pointer. Any other value violates
 *               the API contract.
 * @return true when the source is enabled, otherwise false.
 */
bool consoleSourceIsEnabled(const ConsoleSource *source);

/**
 * Try to send one binary chunk for an enabled source without blocking.
 *
 * The stream is intended as UTF-8, but a chunk may split a code point or
 * contain invalid bytes. Console copies the data before this call returns.
 * The enabled-state check and CRTP enqueue are ordered with runtime disable:
 * a successful disable response cannot overtake an accepted chunk.
 *
 * @param source Successfully registered node pointer. Any other value violates
 *               the API contract.
 * @param data Non-NULL buffer containing the chunk to send.
 * @param length Number of bytes in data; must fit beside the source ID in one
 *               CRTP packet (at most CRTP_MAX_DATA_SIZE - 1).
 * @return true if the CRTP transmit queue accepted the chunk. This does not
 *         confirm delivery to a client. Returns false if the source is
 *         disabled, data is NULL, length is too large, or the queue cannot
 *         accept the packet immediately.
 */
bool consoleSourceSend(const ConsoleSource *source, const uint8_t *data,
                       size_t length);

#ifdef UNIT_TEST_MODE
/** Reset Console module state between host tests. */
void consoleResetForTest(void);
#endif

/**
 * Macro implementing consolePrintf with eprintf
 *
 * @param FMT String format
 * @param ... Parameters to print
 */
#define consolePrintf(FMT, ...) eprintf(consolePutchar, FMT, ## __VA_ARGS__)

#endif /*CONSOLE_H_*/
