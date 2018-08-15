#ifndef __TDOA_ENGINE_H__
#define __TDOA_ENGINE_H__

#include "tdoaStorage.h"
#include "tdoaStats.h"

typedef struct {
  tdaoAnchorInfoArray_t anchorInfoArray;
  tdoaStats_t stats;
} tdoaEngineState_t;

void tdoaEngineInit(tdoaEngineState_t* state, const uint32_t now_ms);

void tdoaEngineGetAnchorCtxForPacketProcessing(tdoaEngineState_t* engineState, const uint8_t anchorId, const uint32_t currentTime_ms, tdoaAnchorContext_t* anchorCtx);
void tdoaEngineProcessPacket(tdoaEngineState_t* engineState, tdoaAnchorContext_t* anchorCtx, const int64_t txAn_in_cl_An, const int64_t rxAn_by_T_in_cl_T);

#endif // __TDOA_ENGINE_H__
