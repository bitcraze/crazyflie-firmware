#include <string.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "queue.h"
#include "system.h"
#include "debug.h"

#include "autoflyPacketQueue.h"

#define CHECK_INTERVAL 5

void initAutoflyPacketQueue(Autofly_packet_Queue_t *queue){
    queue->front = 0;
    queue->tail = 0;
    queue->len = 0;
    queue->mutex = xSemaphoreCreateMutex();
    if(queue->mutex == NULL){
        DEBUG_PRINT("[initAutoflyPacketQueue]Create mutex failed\n");
    }
    for(int i = 0; i < MAX_AUTOFLY_PACKET_QUEUE_SIZE; i++){
        queue->data[i].header.sourceId = 0;
        queue->data[i].header.destinationId = 0;
        queue->data[i].header.packetType = 0;
        queue->data[i].header.length = 0;
        memset(queue->data[i].data, 0, AUTOFLY_PACKET_MTU);
    }
}
void pushAutoflyPacketQueue(Autofly_packet_Queue_t *queue, Autofly_packet_t* data){
    // 先占信号量,防止写冲突
    xSemaphoreTake(queue->mutex, portMAX_DELAY);
    // 死等队列空闲
    // todo，考虑优化超时等待
    while (!isAutoflyPacketQueueFull(queue))
    {
        vTaskDelay(M2T(CHECK_INTERVAL));
    }
    
    memcpy(&queue->data[queue->tail], data, sizeof(Autofly_packet_t));
    queue->tail = (queue->tail + 1) % MAX_AUTOFLY_PACKET_QUEUE_SIZE;
    queue->len++;
}
bool popAutoflyPacketQueue(Autofly_packet_Queue_t *queue, Autofly_packet_t* data){
    // 读不需要占用信号量
    // 等待队列非空
    if(isAutoflyPacketQueueEmpty(queue)){
        return false;
    }
    memcpy(data, &queue->data[queue->front], sizeof(Autofly_packet_t));
    queue->front = (queue->front + 1) % MAX_AUTOFLY_PACKET_QUEUE_SIZE;
    queue->len--;
    return true;
}

bool isAutoflyPacketQueueEmpty(Autofly_packet_Queue_t *queue){
    return queue->len == 0;
}
bool isAutoflyPacketQueueFull(Autofly_packet_Queue_t *queue){
    return queue->len == MAX_AUTOFLY_PACKET_QUEUE_SIZE;
}