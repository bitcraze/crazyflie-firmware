#ifndef __TDOA_ENGINE_H__
#define __TDOA_ENGINE_H__

#include "tdoaStorage.h"

void tdoaEngineInit();

void tdoaEngineGetAnchorCtxForPacketProcessing(const uint8_t anchorId, const uint32_t currentTime_ms, tdoaAnchorContext_t* anchorCtx);
void tdoaEngineProcessPacket(tdoaAnchorContext_t* anchorCtx, const int64_t txAn_in_cl_An, const int64_t rxAn_by_T_in_cl_T);

#endif // __TDOA_ENGINE_H__
