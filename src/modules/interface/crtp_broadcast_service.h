// [Custmized code]
#ifndef _CRTP_BROADCAST_H_
#define _CRTP_BROADCAST_H_

#include "stabilizer_types.h"
#include "broadcast_data.h"

struct CrtpExtPosition
{
  float x; // in m
  float y; // in m
  float z; // in m
  float yaw; // in rad [CHANGE] yaw estimation
} __attribute__((packed));

// [Debug comment]
void bcPosInit(void);

void bcCmdInit(void);

// [Debug comment]
// // Get the current position from the cache
// bool getExtPositionBC(state_t *state);
// bool getExtPosVelBC(state_t *state);
bool getExtPosVelYawBC(state_t *state); // [CHANGE] yaw estimation

#endif /* _CRTP_BROADCAST_H_ */
