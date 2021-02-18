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
 * crc.c - CRC32 implementation
 */

/**
 * Core calculation code is:
 * 
 * Copyright (C) 2017 Bosch Sensortec GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holder nor the names of the
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 *
 * The information provided is believed to be accurate and reliable.
 * The copyright holder assumes no responsibility
 * for the consequences of use
 * of such information nor for any infringement of patents or
 * other rights of third parties which may result from its use.
 * No license is granted by implication or otherwise under any patent or
 * patent rights of the copyright holder.
 */

#include "crc32.h"

#include <stdbool.h>

#include "static_mem.h"

#define POLYNOMIAL              0xEDB88320
#define CHECK_VALUE             0xCBF43926
#define INITIAL_REMAINDER       0xFFFFFFFF
#define FINAL_XOR_VALUE         0xFFFFFFFF
#define RESIDUE                 0xDEBB20e3

// Internal functions
static uint32_t crcByByte(const uint8_t* message, uint32_t bytesToProcess,
              uint32_t remainder, uint32_t* crcTable);
static uint32_t crcByByte(const uint8_t* message, uint32_t bytesToProcess,
              uint32_t remainder, uint32_t* crcTable);
static void crcTableInit(uint32_t* crcTable);

NO_DMA_CCM_SAFE_ZERO_INIT static uint32_t crcTable[256];
static bool crcTableInitialized = false;

// *** Public API ***

void crc32ContextInit(crc32Context_t *context)
{
  // Lazy static ...
  if (crcTableInitialized == false) {
    // initialize crcTable
    crcTableInit(crcTable);
    crcTableInitialized = true;
  }

  context->remainder = INITIAL_REMAINDER;
}

void crc32Update(crc32Context_t *context, const void* data, size_t size)
{
  context->remainder = crcByByte(data, size, context->remainder, crcTable);
}

uint32_t crc32Out(const crc32Context_t *context)
{
  return context->remainder ^ FINAL_XOR_VALUE;
}

uint32_t crc32CalculateBuffer(const void* buffer, size_t size)
{
  crc32Context_t ctx;

  crc32ContextInit(&ctx);
  crc32Update(&ctx, buffer, size);
  return crc32Out(&ctx);
}

// *** Core calculation from Bosh ***

/* bit-wise crc calculation */
static uint32_t crcByBit(const uint8_t* message, uint32_t bytesToProcess,
             uint32_t remainder)
{
  for (unsigned int byte = 0; byte < bytesToProcess; ++byte)
    {
      remainder ^= *(message+byte);

      for(uint8_t bit = 8; bit > 0; --bit)
        {
          /* reflect is realized by mirroring algorithm
           * LSB is first to be processed */
          if (remainder & 1)
            remainder = (remainder >> 1) ^ POLYNOMIAL;
          else
            remainder = (remainder >> 1);
        }
    }
  return remainder;
}

/* byte-wise crc calculation, requires an initialized crcTable
 * this is factor 8 faster and should be used if multiple crcs
 * have to be calculated */
static uint32_t crcByByte(const uint8_t* message, uint32_t bytesToProcess,
              uint32_t remainder, uint32_t* crcTable)
{
  uint8_t data;
  for (int byte = 0; byte < bytesToProcess; ++byte)
    {
      data = (*(message+byte) ^ remainder);
      remainder = *(crcTable+data) ^ (remainder >> 8);
    }
  return remainder;
}

/* creates a lookup-table which is necessary for the crcByByte function */
static void crcTableInit(uint32_t* crcTable)
{
  uint8_t dividend = ~0;
  /* fill the table by bit-wise calculations of checksums
   * for each possible dividend */
  do {
      *(crcTable+dividend) = crcByBit(&dividend, 1, 0);
  } while(dividend-- > 0);
}
