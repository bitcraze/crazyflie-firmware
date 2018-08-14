#ifndef __TDOA_ENGINE_H__
#define __TDOA_ENGINE_H__

#include "tdoaStorage.h"

void tdoaEngineInit();

anchorInfo_t* tdoaEngineGetAnchorCtxForPacketProcessing(const uint8_t anchor);
void tdoaEngineProcessPacket(anchorInfo_t* anchorCtx, const int64_t txAn_in_cl_An, const int64_t rxAn_by_T_in_cl_T);

#endif // __TDOA_ENGINE_H__
