#ifndef __FLOODING_H__
#define __FLOODING_H__

#include "stdint.h"
#include "adhocdeck.h"
#include "flooding_struct.h"

/* Queue Constants */
#define FLOODING_RX_QUEUE_SIZE 10
#define FLOODING_RX_QUEUE_ITEM_SIZE sizeof (UWB_Packet_t)

/* Flooding Constants */
#define FLOODING_INTERVAL 500
#define FLOODING_TIME_TO_LIVE 6

/* Flooding Operations */
void floodingInit();

int generateFloodingMessage(Flooding_Message_t *floodingMessage);

void processFloodingMessage(Flooding_Message_t *floodingMessage);

bool checkFloodingMessage(Flooding_Message_t *floodingMessage);

#endif