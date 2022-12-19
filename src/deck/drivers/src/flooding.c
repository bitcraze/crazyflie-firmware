#include <stdint.h>
#include <math.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "system.h"
#include "semphr.h"

#include "autoconf.h"
#include "debug.h"
#include "log.h"
#include "assert.h"
#include "adhocdeck.h"
#include "flooding.h"
#include "flooding_struct.h"
#include "swarm_ranging.h"

static uint16_t MY_UWB_ADDRESS;

static QueueHandle_t rxQueue;
// static Flooding_Check_Table_Set_t floodingCheckTableSet;
static Flooding_Topology_Table_set_t floodingTopologyTableSet;
static UWB_Message_Listener_t listener;
static TaskHandle_t uwbFloodingTxTaskHandle = 0;
static TaskHandle_t uwbFloodingRxTaskHandle = 0;
static SemaphoreHandle_t taskSemaphore;

static int floodingSeqNumber = 1;

uint16_t floodingCheckTable[FLOODING_CHECK_TABLE_SIZE] = {0};

void floodingRxCallback(void *parameters) {
    // DEBUG_PRINT("floodingRxCallback \n");
}

void floodingTxCallback(void *parameters) {
    // DEBUG_PRINT("floodingTxCallback \n");
}

static void uwbFloodingTxTask(void *parameters) {
    systemWaitStart();

    UWB_Packet_t txPacketCache;
    txPacketCache.header.type = FLOODING;

    while (true) {
        // xSemaphoreTake(taskSemaphore, portMAX_DELAY);
        // printFloodingTopologyTableSet(&floodingTopologyTableSet);
        DEBUG_PRINT("Topology size: %u\n", floodingTopologyTableSet.size);
        int msgLen = generateFloodingMessage((Flooding_Message_t *) &txPacketCache.payload);
        if (msgLen > sizeof (Flooding_Message_Header_t)) {
            txPacketCache.header.length = sizeof (Packet_Header_t) + msgLen;
            uwbSendPacketBlock(&txPacketCache);
        }
        // xSemaphoreGive(taskSemaphore);
        vTaskDelay(FLOODING_INTERVAL);
    }
}

static void uwbFloodingRxTask(void *parameters) {
    systemWaitStart();

    UWB_Packet_t rxPacketCache;

    while (true) {
        if (xQueueReceive(rxQueue, &rxPacketCache, portMAX_DELAY)) {
            // xSemaphoreTake(taskSemaphore, portMAX_DELAY);
            Flooding_Message_t *floodingMessage = (Flooding_Message_t *) &rxPacketCache.payload;
            if (checkFloodingMessage(floodingMessage)) {
                // DEBUG_PRINT("Seq: %u\n", floodingMessage->header.msgSequence);
                processFloodingMessage(floodingMessage);
                uwbSendPacketBlock(&rxPacketCache);
            }
            // processFloodingMessage(floodingMessage);
            // xSemaphoreGive(taskSemaphore);
        }
    }
}

void floodingInit() {
    MY_UWB_ADDRESS = getUWBAddress();
    rxQueue = xQueueCreate(FLOODING_RX_QUEUE_SIZE, FLOODING_RX_QUEUE_ITEM_SIZE);
    floodingTopologyTableSetInit(&floodingTopologyTableSet);
    // floodingCheckTableSetInit(&floodingCheckTableSet);

    listener.type = FLOODING;
    listener.rxQueue = rxQueue;
    listener.rxCb = floodingRxCallback;
    listener.txCb = floodingTxCallback;
    uwbRegisterListener(&listener);

    taskSemaphore = xSemaphoreCreateMutex();

    xTaskCreate(uwbFloodingTxTask, ADHOC_DECK_FLOODING_TX_TASK_NAME, 4 * configMINIMAL_STACK_SIZE, NULL,
                ADHOC_DECK_TASK_PRI, &uwbFloodingTxTaskHandle);
    xTaskCreate(uwbFloodingRxTask, ADHOC_DECK_FLOODING_RX_TASK_NAME, 4 * configMINIMAL_STACK_SIZE, NULL,
                ADHOC_DECK_TASK_PRI, &uwbFloodingRxTaskHandle);
}

int generateFloodingMessage(Flooding_Message_t *floodingMessage) {
    // TODO: Complete expiration
    floodingTopologyTableSetClearExpire(&floodingTopologyTableSet);

    int8_t bodyUnitNumber = 0;
    int curSeqNumber = floodingSeqNumber;
    /* generate message body */
    uint16_t addressIndex;
    for (addressIndex = 0; addressIndex < RANGING_TABLE_SIZE; addressIndex++) {
        if (bodyUnitNumber >= MAX_BODY_UNIT_NUMBER) {
            break;
        }
        // Have completed
        /* Use distance to judge whether neighbors exist. If the neighbor does not exist,
           distance will be 0 */
        int16_t distance = getDistance(addressIndex);
        if (distance > 0) {
            floodingMessage->bodyUnits[bodyUnitNumber].srcAddress = MY_UWB_ADDRESS;
            floodingMessage->bodyUnits[bodyUnitNumber].dstAddress = addressIndex;
            floodingMessage->bodyUnits[bodyUnitNumber].distance = distance;
            // Have completed
            floodingTopologyTableSetUpdate(&floodingTopologyTableSet, MY_UWB_ADDRESS,
                                      addressIndex, distance);
            bodyUnitNumber++;
        }
    }
    /* generate message header */
    floodingMessage->header.srcAddress = MY_UWB_ADDRESS;
    floodingMessage->header.msgLength = sizeof(Flooding_Message_Header_t) + sizeof(Flooding_Body_Unit_t) * bodyUnitNumber;
    floodingMessage->header.msgSequence = curSeqNumber;
    floodingMessage->header.timeToLive = FLOODING_TIME_TO_LIVE;

    /* data update */
    if (floodingMessage->header.msgLength > sizeof (Flooding_Message_Header_t)) {
        floodingSeqNumber++;
    }

    return floodingMessage->header.msgLength;
}

void processFloodingMessage(Flooding_Message_t *floodingMessage) {
    int8_t bodyUnitNumberMax = (floodingMessage->header.msgLength - sizeof (Flooding_Message_Header_t)) / sizeof (Flooding_Body_Unit_t);
    // DEBUG_PRINT("Len: %u, Header: %u, Unit: %u\n", floodingMessage->header.msgLength, sizeof (Flooding_Message_Header_t), sizeof (Flooding_Body_Unit_t));
    for (int8_t bodyUnitNumber = 0; bodyUnitNumber < bodyUnitNumberMax; bodyUnitNumber++) {
        Flooding_Body_Unit_t *bodyUnit = &floodingMessage->bodyUnits[bodyUnitNumber];
        // DEBUG_PRINT("SRC: %u\n", bodyUnit->srcAddress);
        floodingTopologyTableSetUpdate(&floodingTopologyTableSet, bodyUnit->srcAddress,
                                      bodyUnit->dstAddress, bodyUnit->distance);
    }
}

bool checkFloodingMessage(Flooding_Message_t *floodingMessage) {
    if (floodingMessage == NULL ||
        floodingMessage->header.srcAddress == MY_UWB_ADDRESS ||
        floodingMessage->header.timeToLive == 0 ||
        floodingMessage->header.msgSequence <= floodingCheckTable[floodingMessage->header.srcAddress]) {
        return false;
    }
    floodingMessage->header.timeToLive--;
    floodingCheckTable[floodingMessage->header.srcAddress] = floodingMessage->header.msgSequence;
    return true;
}
