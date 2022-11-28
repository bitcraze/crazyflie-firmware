#define DEBUG_MODULE "ROUTING"

#include "stm32fxxx.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "autoconf.h"
#include "debug.h"
#include "deck.h"
#include "estimator.h"
#include "log.h"
#include "param.h"
#include "system.h"

#include "adhocdeck.h"
#include "dwTypes.h"
#include "libdw3000.h"
#include "dw3000.h"
#include "routing.h"

static TaskHandle_t uwbRoutingTxTaskHandle = 0;
static TaskHandle_t uwbRoutingRxTaskHandle = 0;
static QueueHandle_t rxQueue;
static int seqNumber = 1;

void routingRxCallback(void *parameters) {
  DEBUG_PRINT("routingRxCallback \n");
}

void routingTxCallback(void *parameters) {
  DEBUG_PRINT("routingTxCallback \n");
}

int generateRoutingDataMessage(MockData_t *message) {
  int msgLen = sizeof(MockData_t);
  message->seqNumber = seqNumber++;
  return msgLen;
}

static void processRoutingDataMessage(UWB_Packet_t *packet) {
  MockData_t *mockData = (MockData_t *) packet->payload;
  DEBUG_PRINT("received routing data, seq number = %d \n", mockData->seqNumber);
}

static void uwbRoutingTxTask(void *parameters) {
  systemWaitStart();

  UWB_Packet_t txPacketCache;
  txPacketCache.header.type = DATA;
//  txPacketCache.header.mac = ? TODO init mac header
  while (true) {
    int msgLen = generateRoutingDataMessage((MockData_t * ) & txPacketCache.payload);
    txPacketCache.header.length = sizeof(Packet_Header_t) + msgLen;
    uwbSendPacketBlock(&txPacketCache);
    vTaskDelay(M2T(100));
  }
}

static void uwbRoutingRxTask(void *parameters) {
  systemWaitStart();

  UWB_Packet_t rxPacketCache;

  while (true) {
    if (uwbReceivePacketBlock(DATA, &rxPacketCache)) {
      processRoutingDataMessage(&rxPacketCache);
    }
  }
}

void routingInit() {
  rxQueue = xQueueCreate(ROUTING_RX_QUEUE_SIZE, ROUTING_RX_QUEUE_ITEM_SIZE);

  UWB_Message_Listener_t listener;
  listener.type = DATA;
  listener.rxQueue = rxQueue;
  listener.rxCb = routingRxCallback;
  listener.txCb = routingTxCallback;
  uwbRegisterListener(&listener);

  xTaskCreate(uwbRoutingTxTask, ADHOC_DECK_ROUTING_TX_TASK_NAME, 4 * configMINIMAL_STACK_SIZE, NULL,
              ADHOC_DECK_TASK_PRI, &uwbRoutingTxTaskHandle); // TODO optimize STACK SIZE
  xTaskCreate(uwbRoutingRxTask, ADHOC_DECK_ROUTING_RX_TASK_NAME, 4 * configMINIMAL_STACK_SIZE, NULL,
              ADHOC_DECK_TASK_PRI, &uwbRoutingRxTaskHandle); // TODO optimize STACK SIZE
}

