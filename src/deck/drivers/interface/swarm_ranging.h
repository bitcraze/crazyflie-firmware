#ifndef _SWARM_RANGING_H_
#define _SWARM_RANGING_H_
#include "adhocdeck.h"
#include "ranging_struct.h"

/* Function Switch */
#define ENABLE_BUS_BOARDING_SCHEME

/* Queue Constants */
#define RANGING_RX_QUEUE_SIZE 10
#define RANGING_RX_QUEUE_ITEM_SIZE sizeof(Ranging_Message_With_Timestamp_t)

/* Ranging Constants */
#define RANGING_INTERVAL_MIN 20 // default 20
#define RANGING_INTERVAL_MAX 500 // default 500
#define Tf_BUFFER_POOL_SIZE (4 * RANGING_INTERVAL_MAX / RANGING_INTERVAL_MIN)
#define TX_PERIOD_IN_MS 100

/* Ranging Operations */
void rangingInit();
int16_t computeDistance(Timestamp_Tuple_t Tp, Timestamp_Tuple_t Rp,
                        Timestamp_Tuple_t Tr, Timestamp_Tuple_t Rr,
                        Timestamp_Tuple_t Tf, Timestamp_Tuple_t Rf);
void processRangingMessage(Ranging_Message_With_Timestamp_t *rangingMessageWithTimestamp);
int generateRangingMessage(Ranging_Message_t *rangingMessage);
int16_t getDistance(uint16_t neighborAddress);
void setDistance(uint16_t neighborAddress, int16_t distance);

#endif