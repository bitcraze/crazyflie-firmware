#ifndef __TDOA_STORAGE_H__
#define __TDOA_STORAGE_H__

#include "stabilizer_types.h"
#include "clockCorrectionStorage.h"

#define ANCHOR_STORAGE_COUNT 16
#define REMOTE_ANCHOR_DATA_COUNT 16
#define TOF_PER_ANCHOR_COUNT 16

typedef struct {
  uint8_t id; // Id of remote remote anchor
  uint8_t seqNr; // Sequence number of the packet received in the remote anchor (7 bits)
  int64_t rxTime; // Receive time of packet from anchor id in the remote anchor, in remote DWM clock
  uint32_t endOfLife;
} remoteAnchorData_t;

typedef struct {
  uint8_t id;
  int64_t tof;
  uint32_t endOfLife; // Time stamp when the tof data is outdated, local system time
} timeOfFlight_t;

typedef struct {
  bool isInitialized;
  uint32_t lastUpdateTime; // The time when this anchor was updated the last time
  uint8_t id; // Anchor id

  int64_t txTime; // Transmit time of last packet, in remote DWM clock
  int64_t rxTime; // Receive time of last packet, in local DWM clock
  uint8_t seqNr; // Sequence nr of last packet (7 bits)

  clockCorrectionStorage_t clockCorrectionStorage;

  point_t position; // The coordinates of the anchor

  timeOfFlight_t tof[TOF_PER_ANCHOR_COUNT];
  remoteAnchorData_t remoteAnchorData[REMOTE_ANCHOR_DATA_COUNT];
} anchorInfo_t;

void tdoaStorageInitialize();

anchorInfo_t* tdoaStorageGetAnchorCtx(const uint8_t anchor);
anchorInfo_t* tdoaStorageInitializeNewAnchorContext(const uint8_t anchor);
uint8_t tdoaStorageGetId(const anchorInfo_t* anchorCtx);
int64_t tdoaStorageGetRxTime(const anchorInfo_t* anchorCtx);
int64_t tdoaStorageGetTxTime(const anchorInfo_t* anchorCtx);
uint8_t tdoaStorageGetSeqNr(const anchorInfo_t* anchorCtx);
bool tdoaStorageGetAnchorPosition(const anchorInfo_t* anchorCtx, point_t* position);
void tdoaStorageSetAnchorPosition(anchorInfo_t* anchorCtx, const float x, const float y, const float z);
void tdoaStorageSetRxTxData(anchorInfo_t* anchorCtx, int64_t rxTime, int64_t txTime, uint8_t seqNr);
double tdoaStorageGetClockCorrection(const anchorInfo_t* anchorCtx);
int64_t tdoaStorageGetRemoteRxTime(const anchorInfo_t* anchorCtx, const uint8_t remoteAnchor);
void tdoaStorageSetRemoteRxTime(anchorInfo_t* anchorCtx, const uint8_t remoteAnchor, const int64_t remoteRxTime, const uint8_t remoteSeqNr);
void tdoaStorageGetRemoteSeqNrList(const anchorInfo_t* anchorCtx, int* remoteCount, uint8_t seqNr[], uint8_t id[]);
int64_t tdoaStorageGetTimeOfFlight(const anchorInfo_t* anchorCtx, const uint8_t otherAnchor);
void tdoaStorageSetTimeOfFlight(anchorInfo_t* anchorCtx, const uint8_t remoteAnchor, const int64_t tof);

#endif // __TDOA_STORAGE_H__
