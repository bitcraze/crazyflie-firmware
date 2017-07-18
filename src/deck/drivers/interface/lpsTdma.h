/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2016 Bitcraze AB
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * lpsTdma.h: Generic TDMA utility defines for LPS algorithms
 */
#ifndef __LPS_TDMA_H__
#define __LPS_TDMA_H__

#ifndef TDMA_NSLOTS_BITS
#ifdef LPS_TDMA_ENABLE
#warning "Number of slots bits for TDMA not defined! Defaulting to 1 (2 slots)."
#endif
#define TDMA_NSLOTS_BITS 1
#endif

#ifndef TDMA_SLOT
#define TDMA_SLOT -1
#endif

#define TDMA_SLOT_BITS 27

#define TDMA_FRAME_BITS (TDMA_SLOT_BITS + TDMA_NSLOTS_BITS)
#define TDMA_SLOT_LEN (1ull<<(unsigned long long)(TDMA_SLOT_BITS+1))
#define TDMA_FRAME_LEN (1ull<<(unsigned long long)(TDMA_FRAME_BITS+1))

#define TDMA_LAST_FRAME(NOW) ( NOW & ~(TDMA_FRAME_LEN-1) )

#endif //__LPS_TDMA_H__
