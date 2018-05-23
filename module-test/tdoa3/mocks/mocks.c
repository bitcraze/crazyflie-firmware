#include <string.h>
#include <stdio.h>

#include "task.h"
#include "stabilizer_types.h"
#include "lpsTdoaTagStats.h"
#include "lpsTdoaTagEngine.h"

lpsTdoaStats_t lpsTdoaStats;

static uint32_t currentTick;
static tdoaMeasurement_t output;
static bool wasOutputGenerated;


typedef struct {
  unsigned int id;
  unsigned int seqNr;
  unsigned int rxTimeStamp;
  unsigned int distance;
} rangeData_t;

void simulateSetTestCurrentSysTime(const uint32_t tick) {
//  printf("%u\n", tick);
  currentTick = tick;
}

// Replace the FreeRTOS implementation
uint32_t xTaskGetTickCount(void) {
  return currentTick;
}

// Replace the normal implementation in the estimator
bool estimatorKalmanEnqueueTDOA(tdoaMeasurement_t *uwb) {
  memcpy(&output, uwb, sizeof(output));
  wasOutputGenerated = true;

  return true;
}

bool simulateGetEnqueuedResult(tdoaMeasurement_t* uwb) {
  memcpy(uwb, &output, sizeof(*uwb));

  return wasOutputGenerated;
}

static bool isValidTimeStamp(uint64_t ts) {
  return (ts != 0);
}

static void updateRemoteData(anchorInfo_t* anchorCtx, const int rangeDataCount, const rangeData_t* rangeData) {
  for (int i = 0; i < rangeDataCount; i++) {
    uint8_t remoteId = rangeData[i].id;
    int64_t remoteRxTime = rangeData[i].rxTimeStamp;
    uint8_t remoteSeqNr = rangeData[i].seqNr & 0x7f;
    int64_t tof = rangeData[i].distance;

    if (isValidTimeStamp(remoteRxTime)) {
      tdoaEngineSetRemoteRxTime(anchorCtx, remoteId, remoteRxTime, remoteSeqNr);
    }

    if (isValidTimeStamp(tof)) {
      tdoaEngineSetTimeOfFlight(anchorCtx, remoteId, tof);
    }
  }
}

bool callTdoaEngineProcessPacket(uint8_t anchorId,
                                 const int64_t txAn_in_cl_An,
                                 const int64_t rxAn_by_T_in_cl_T,
                                 const uint8_t seqNr,
                                 int rangeDataCount,
                                 rangeData_t* rangeData,
                                 point_t* position) {
  wasOutputGenerated = false;

//  printf("id: %i, tx: %lli, rx: %lli, seq: %i, cnt: %i\n", anchorId, txAn_in_cl_An, rxAn_by_T_in_cl_T, seqNr, rangeDataCount);

  anchorInfo_t* anchorCtx = getAnchorCtxForPacketProcessing(anchorId);
  if (anchorCtx) {
    updateRemoteData(anchorCtx, rangeDataCount, rangeData);
    tdoaEngineProcessPacket(anchorCtx, txAn_in_cl_An, rxAn_by_T_in_cl_T);
    tdoaEngineSetRxTxData(anchorCtx, rxAn_by_T_in_cl_T, txAn_in_cl_An, seqNr);
    if (position) {
      tdoaEngineSetAnchorPosition(anchorCtx, position->x, position->y, position->z);
    }
  }

  return wasOutputGenerated;
}



