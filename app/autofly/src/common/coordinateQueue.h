#ifndef COORDINATE_QUEUE_H
#define COORDINATE_QUEUE_H

#include "auxiliary_tool.h"
#define MAX_COORDINATEQUEUE 80

typedef struct{
    coordinateF_t data[MAX_COORDINATEQUEUE];
    short front;
    short tail;
    short len;
}CoordinateQueue_t;

void initCoordinateQueue(CoordinateQueue_t *queue);
bool push_CoordinateQueue(CoordinateQueue_t *queue, coordinateF_t data);
coordinateF_t pop_CoordinateQueue(CoordinateQueue_t *queue);
bool isCoordinateQueueEmpty(CoordinateQueue_t *queue);
bool isCoordinateQueueFull(CoordinateQueue_t *queue);

#endif
