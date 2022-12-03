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
 */

#include <stdbool.h>
#include <string.h>
#include "buf2buf.h"

void buf2bufInit(Buf2bufContext_t* context, uint8_t* outBuf, const uint32_t outBufSize) {
    context->outBuf = outBuf;
    context->outBufSize = outBufSize;
    context->outBufIndex = 0;

    context->inBuf = 0;
    context->inBufSize = 0;
    context->inBufIndex = 0;

    context->consumed = 0;
    context->added = 0;
}


void buf2bufAddInBuf(Buf2bufContext_t* context, const uint8_t* inBuf, const uint32_t inBufSize) {
    context->inBuf = inBuf;
    context->inBufSize = inBufSize;
    context->inBufIndex = 0;
    context->added += inBufSize;
}


bool buf2bufConsumeInBuf(Buf2bufContext_t* context) {
    const uint32_t dataLeftIn = context->inBufSize - context->inBufIndex;
    const uint32_t spaceLeftOut = context->outBufSize - context->outBufIndex;

    uint32_t copySize = dataLeftIn;
    if (spaceLeftOut < dataLeftIn) {
        copySize = spaceLeftOut;
    }

    memcpy(&context->outBuf[context->outBufIndex], &context->inBuf[context->inBufIndex], copySize);
    context->inBufIndex += copySize;
    context->outBufIndex += copySize;
    context->consumed += copySize;

    const bool isOutBufFull = (context->outBufIndex == context->outBufSize);
    if (isOutBufFull) {
        context->outBufIndex = 0;
    }

    return isOutBufFull;
}


void buf2bufReleaseInBuf(Buf2bufContext_t* context) {
    context->inBuf = 0;
    context->inBufSize = 0;
    context->inBufIndex = 0;
}


uint32_t buf2bufReleaseOutBuf(Buf2bufContext_t* context) {
    context->outBuf = 0;

    return context->outBufIndex;
}


#define OUTBUFSIZE 1000
static uint8_t outBuf[OUTBUFSIZE];
static Buf2bufContext_t context;
void handleInBuffer(const uint32_t memAddr, const uint8_t inBufDataLen, const uint8_t *inBuf, const uint32_t totSize) {
    const bool isFirstBuf = (memAddr == 0);
    if (isFirstBuf) {
        buf2bufInit(&context, outBuf, OUTBUFSIZE);
    }

    buf2bufAddInBuf(&context, inBuf, inBufDataLen);
    while(buf2bufConsumeInBuf(&context)) {
        // send(outbuf, OUTBUFSIZE);
    }
    buf2bufReleaseInBuf(&context);

    const bool isLastBuf = (memAddr + inBufDataLen >= totSize);
    if (isLastBuf) {
        uint32_t size = buf2bufReleaseOutBuf(&context);
        if (size > 0) {
            // send(outbuf, size);
        }
    }
}
