#include "stdlib.h"
#include "coordinateQueue.h"
#include "debug.h"

void initCoordinateQueue(CoordinateQueue_t *queue) {
    queue->front = 0;
    queue->tail = 0;
    queue->len = 0;
}

bool push_CoordinateQueue(CoordinateQueue_t *queue, coordinateF_t data){
    if(isCoordinateQueueFull(queue)){
        DEBUG_PRINT("CoordinateQueue is full!\n");
        return false;
    }
    queue->data[queue->tail] = data;
    queue->tail = (queue->tail + 1) % MAX_COORDINATEQUEUE;
    ++queue->len;
    return true;
}

coordinateF_t pop_CoordinateQueue(CoordinateQueue_t *queue){
    if(isCoordinateQueueEmpty(queue)){
        DEBUG_PRINT("CoordinateQueue is empty!\n");
        coordinateF_t data = {-1,-1,-1};
        return data;
    }
    coordinateF_t data = queue->data[queue->front];
    queue->front = (queue->front + 1) % MAX_COORDINATEQUEUE;
    --queue->len;
    return data;
}

bool isCoordinateQueueEmpty(CoordinateQueue_t *queue){
    // return queue->front == queue->tail;
    return queue->len == 0;
}

bool isCoordinateQueueFull(CoordinateQueue_t *queue){
    // return (queue->tail + 1) % MAX_QUEUE_SIZE == queue->front;
    return queue->len == MAX_COORDINATEQUEUE;
}
