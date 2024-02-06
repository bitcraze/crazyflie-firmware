/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2022 Bitcraze AB
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
 *
 * buf2buf.h - utility for copying buffers when transferring data from one buffer size to another.
 * Incoming buffers have one size while outgoing buffers have a different size.
 */

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    const uint8_t* inBuf;
    uint32_t inBufIndex;
    uint32_t inBufSize;
    uint32_t added;

    uint8_t* outBuf;
    uint32_t outBufIndex;
    uint32_t outBufSize;
    uint32_t consumed;
} Buf2bufContext_t;

/**
 * @brief Initialize a context
 *
 * This function must be called first to initialize the context.
 * A pointer to the out-buffer will be stored in the context and the out-buffer should not be modified until released.
 *
 * @param context A new context
 * @param outBuf Pointer to the out buffer (reused over and over)
 * @param outBufSize The size of the out buffer
 */
void buf2bufInit(Buf2bufContext_t* context, uint8_t* outBuf, const uint32_t outBufSize);

/**
 * @brief Add a new in-buffer
 *
 * Call this function when a new in-buffer is available. The pointer to the in-buffer will be stored in the context
 * and the in-buffer should not be modified until released by a call to buf2bufReleaseInBuf().
 *
 * @param context The context
 * @param inBuf Pointer to the in-buffer
 * @param inBufSize The size of the in-buffer
 */
void buf2bufAddInBuf(Buf2bufContext_t* context, const uint8_t* inBuf, const uint32_t inBufSize);

/**
 * @brief Consume data from the in-buffer
 *
 * Copy data from the in-buffer to the out-buffer, may have to be called multiple times if the out-buffer can not
 * hold all the data from the in-buffer. If this function returns true, the out-buffer is full and is ready to be
 * emptied by the application code. This function should be called repeatedly until it returns false which indicates
 * that the in-buffer is fully consumed.
 *
 * @param context The context
 * @return true The out buffer is full and ready to be used
 * @return false The in buffer is fully consumed
 */
bool buf2bufConsumeInBuf(Buf2bufContext_t* context);

/**
 * @brief Release the in-buffer
 *
 * Call when in-buffer has been fully consumed to release the in-buffer.
 *
 * @param context The context
 */
void buf2bufReleaseInBuf(Buf2bufContext_t* context);

/**
 * @brief Release the out-buffer
 *
 * Will release the out-buffer and return the number of bytes of data that remains to be emptied by the application
 * code.
 *
 * @param context
 * @return uint32_t
 */
uint32_t buf2bufReleaseOutBuf(Buf2bufContext_t* context);

/**
 * @brief The total number of bytes added by in-buffers
 *
 * @param context The context
 * @return uint32_t The number of bytes
 */
static inline uint32_t buf2bufBytesAdded(const Buf2bufContext_t* context) {
    return context->added;
}

/**
 * @brief The total number of bytes consumed from in-buffers
 *
 * @param context The context
 * @return uint32_t The number of bytes
 */
static inline uint32_t buf2bufBytesConsumed(const Buf2bufContext_t* context) {
    return context->consumed;
}
