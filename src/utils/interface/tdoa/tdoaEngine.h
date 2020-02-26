#ifndef __TDOA_ENGINE_H__
#define __TDOA_ENGINE_H__

#include "tdoaStorage.h"
#include "tdoaStats.h"

typedef void (*tdoaEngineSendTdoaToEstimator)(tdoaMeasurement_t* tdoaMeasurement, const uint8_t idA, const uint8_t idB);

typedef enum {
  TdoaEngineMatchingAlgorithmNone = 0,
  TdoaEngineMatchingAlgorithmRandom,
  TdoaEngineMatchingAlgorithmYoungest,
} tdoaEngineMatchingAlgorithm_t;

typedef struct {
  // State
  tdaoAnchorInfoArray_t anchorInfoArray;
  tdoaStats_t stats;

  // Configuration
  tdoaEngineSendTdoaToEstimator sendTdoaToEstimator;
  double locodeckTsFreq;
  tdoaEngineMatchingAlgorithm_t matchingAlgorithm;

  // Matching algorithm data
  struct {
    uint8_t seqNr[REMOTE_ANCHOR_DATA_COUNT];
    uint8_t id[REMOTE_ANCHOR_DATA_COUNT];
    uint8_t offset;
  } matching;
} tdoaEngineState_t;

void tdoaEngineInit(tdoaEngineState_t* state, const uint32_t now_ms, tdoaEngineSendTdoaToEstimator sendTdoaToEstimator, const double locodeckTsFreq, const tdoaEngineMatchingAlgorithm_t matchingAlgorithm);

void tdoaEngineGetAnchorCtxForPacketProcessing(tdoaEngineState_t* engineState, const uint8_t anchorId, const uint32_t currentTime_ms, tdoaAnchorContext_t* anchorCtx);
void tdoaEngineProcessPacket(tdoaEngineState_t* engineState, tdoaAnchorContext_t* anchorCtx, const int64_t txAn_in_cl_An, const int64_t rxAn_by_T_in_cl_T);

#endif // __TDOA_ENGINE_H__
