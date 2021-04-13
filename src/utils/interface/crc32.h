/*
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
 * crc.h - CRC32 implementation
 */

#pragma once

#include <stdint.h>
#include <stddef.h>

/**
 * @brief CRC32 checksum calculation
 * 
 * This module calculates a checksum that is compatible with the zlib
 * CRC32 algorism. For example the CRC32 calculated by Python's
 * "binascii.crc32", "zlib.crc32" and the linux crc32 command line utility.
 * 
 */


/**
 * @brief CRC32 context
 * 
 * Should be considered as an opaque object containing the current
 * state of the CRC32 checksum calculation.
 * 
 */
typedef struct {
    uint32_t remainder;
} crc32Context_t;

/**
 * @brief Initialize CRC32 context
 * 
 * This function must be called before using the context.
 * It can be called on an already initialized context to reset it
 * and calculate a new CRC32.
 * 
 * @param context Context to initialize
 */
void crc32ContextInit(crc32Context_t *context);

/**
 * @brief Update checksum with new data
 * 
 * Can be called multiple times to add data to checksum.
 * 
 * @param context an initialized context
 * @param data Data buffer to add to the checksum
 * @param size Size in byte of the data buffer
 */
void crc32Update(crc32Context_t *context, const void* data, size_t size);

/**
 * @brief Generate and returns the CRC32 checksum
 * 
 * @param context An initialized context
 * @return the CRC32 checksum
 */
uint32_t crc32Out(const crc32Context_t *context);

/**
 * @brief Utility function to calculate the checksum of a buffer in one call
 * 
 * This function calls contextInit, update and out in sequence to generate the
 * checksum of a buffer. It is a conveniance function to calculate the checksum
 * of a buffer that is already complete in memory
 * 
 * @param buffer Data buffer that needs to be checksumed
 * @param size Size in byte of the data buffer
 * @return The CRC32 checksum
 */
uint32_t crc32CalculateBuffer(const void* buffer, size_t size);
