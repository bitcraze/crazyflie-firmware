#ifndef __PEER_LOCALIZATION_H__
#define __PEER_LOCALIZATION_H__

#include <stdbool.h>
#include "math3d.h"
#include "stabilizer_types.h"

void peerLocalizationInit();
bool peerLocalizationTest();

typedef struct peerLocalizationOtherPosition_s {
  uint8_t id; // CF id
  point_t pos; // position and timestamp in m
} peerLocalizationOtherPosition_t;

bool peerLocalizationTellPosition(int id, positionMeasurement_t const *pos);
bool peerLocalizationIsIDActive(uint8_t id);
peerLocalizationOtherPosition_t *peerLocalizationGetPositionByID(uint8_t id);
peerLocalizationOtherPosition_t *peerLocalizationGetPositionByIdx(uint8_t idx);

#endif // __PEER_LOCALIZATION_H__