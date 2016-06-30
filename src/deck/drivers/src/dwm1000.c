/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2016 Bitcraze AB
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * dwm1000.c: Dwm1000 deck driver. Implements ranging with anchors.
 */

#define DEBUG_MODULE "DWM"

#include <stdint.h>
#include <string.h>
#include "stm32fxxx.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "deck.h"
#include "system.h"
#include "debug.h"
#include "log.h"
#include "nvicconf.h"

#include "libdw1000.h"

// Minimal MAC packet format
#include "mac.h"

#define CS_PIN DECK_GPIO_IO1

// Device handler
static dwDevice_t dwm_device;
static dwDevice_t *dwm = &dwm_device;

// Amount of failed ranging before the ranging value is set as wrong
#define RANGING_FAILED_TH 6

#define RX_TIMEOUT 2000

static xSemaphoreHandle dwm1000Event;
static xSemaphoreHandle spiSemaphore;
static xSemaphoreHandle rangingComplete;

#define N_NODES 9

// Address of the nodes are bc:cf:0:0:0:0:0:node_number
uint8_t addresses[N_NODES+1][8] = {
  {0,0,0,0,0,0,0,0},
  {1,0,0,0,0,0,0xcf,0xbc},
  {2,0,0,0,0,0,0xcf,0xbc},
  {3,0,0,0,0,0,0xcf,0xbc},
  {4,0,0,0,0,0,0xcf,0xbc},
  {5,0,0,0,0,0,0xcf,0xbc},
  {6,0,0,0,0,0,0xcf,0xbc},
  {7,0,0,0,0,0,0xcf,0xbc},
  {8,0,0,0,0,0,0xcf,0xbc},
  {9,0,0,0,0,0,0xcf,0xbc},
};

// Static system configuration
#define N_ANCHORS 8
int anchors[N_ANCHORS] = {1,2,3,4,5,6,7,8};

#define N_TAGS 1
int tags[N_TAGS] = {9};

// Hardcode Crazyflie as board5
static int boardId = 9;

/***** Radio callbacks, Ranging algoritm *******/

// The four packets for ranging
#define POLL 0x01   // Poll is initiated by the tag
#define ANSWER 0x02
#define FINAL 0x03
#define REPORT 0x04 // Report contains all measurement from the anchor

typedef struct {
  uint8_t pollRx[5];
  uint8_t answerTx[5];
  uint8_t finalRx[5];

  float pressure;
  float temperature;
  float asl;
  uint8_t pressure_ok;
} __attribute__((packed)) reportPayload_t;

// Possible error
#define ERROR_SEQ_ANSWER 1
#define ERROR_SEQ_REPORT 2
#define ERROR_NOT_COMPLETE 4

// Timestamps for ranging
static dwTime_t poll_tx;
static dwTime_t poll_rx;
static dwTime_t answer_tx;
static dwTime_t answer_rx;
static dwTime_t final_tx;
static dwTime_t final_rx;

const double C = 299792458.0;       // Speed of light
const double tsfreq = 499.2e6 * 128;  // Timestamp counter frequency

#define ANTENNA_OFFSET 154.6   // In meter
#define ANTENNA_DELAY  (ANTENNA_OFFSET*499.2e6*128)/299792458.0 // In radio tick

static packet_t rxPacket;
static packet_t txPacket;

// The TX callback handles timing TX packets
void txCallback(dwDevice_t *dev)
{
  dwTime_t departure;
  dwGetTransmitTimestamp(dev, &departure);

  switch (txPacket.payload[0]) {
    case POLL:
      poll_tx = departure;
      break;
    case ANSWER:
      answer_tx = departure;
      break;
    case FINAL:
      final_tx = departure;
      break;
    case REPORT:
      break;
  }

}

static float distance[N_ANCHORS];
static float pressures[N_ANCHORS];
static int failedRanging[N_ANCHORS];
static volatile float airtime;
static volatile int current_anchor = 0;
static volatile bool ranging_complete = false;

#define TYPE 0
#define SEQ 1

static volatile uint8_t curr_seq = 0;

void rxCallback(dwDevice_t *dev) {
  dwTime_t arival;
  int dataLength = dwGetDataLength(dev);

  bzero(&rxPacket, MAC802154_HEADER_LENGTH);
  dwGetData(dev, (uint8_t*)&rxPacket, dataLength);

  // Not for me, reactivate receive
  if (memcmp(rxPacket.destAddress, addresses[boardId], 8)) {
    dwNewReceive(dev);
    dwSetDefaults(dev);
    dwStartReceive(dev);
    return;
  }

  dwGetReceiveTimestamp(dev, &arival);

  memcpy(txPacket.destAddress, rxPacket.sourceAddress, 8);
  memcpy(txPacket.sourceAddress, rxPacket.destAddress, 8);

  switch(rxPacket.payload[TYPE]) {
    // Anchor received messages
    case POLL:
      poll_rx = arival;

      txPacket.payload[TYPE] = ANSWER;
      txPacket.payload[SEQ] = rxPacket.payload[SEQ];

      dwNewTransmit(dev);
      dwSetDefaults(dev);
      dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2);

      dwWaitForResponse(dev, true);
      dwStartTransmit(dev);
      break;
    case FINAL:
    {
      reportPayload_t *report = (reportPayload_t *)(txPacket.payload+2);

      final_rx = arival;

      txPacket.payload[TYPE] = REPORT;
      txPacket.payload[SEQ] = rxPacket.payload[SEQ];
      memcpy(&report->pollRx, &poll_rx, 5);
      memcpy(&report->answerTx, &answer_tx, 5);
      memcpy(&report->finalRx, &final_rx, 5);

      dwNewTransmit(dev);
      dwSetDefaults(dev);
      dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2+sizeof(reportPayload_t));

      dwWaitForResponse(dev, true);
      dwStartTransmit(dev);
      break;
    }
    // Tag received messages
    case ANSWER:
      if (rxPacket.payload[SEQ] != curr_seq) {
        xSemaphoreGive(rangingComplete);
        return;
      }

      answer_rx = arival;

      txPacket.payload[0] = FINAL;
      txPacket.payload[SEQ] = rxPacket.payload[SEQ];

      dwNewTransmit(dev);
      dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2);

      dwWaitForResponse(dev, true);
      dwStartTransmit(dev);
      break;
    case REPORT:
    {
      reportPayload_t *report = (reportPayload_t *)(rxPacket.payload+2);
      double tround1, treply1, treply2, tround2, tprop_ctn, tprop;

      if (rxPacket.payload[SEQ] != curr_seq) {
        ranging_complete = false;
        xSemaphoreGive(rangingComplete);
        return;
      }

      memcpy(&poll_rx, &report->pollRx, 5);
      memcpy(&answer_tx, &report->answerTx, 5);
      memcpy(&final_rx, &report->finalRx, 5);

      tround1 = answer_rx.low32 - poll_tx.low32;
      treply1 = answer_tx.low32 - poll_rx.low32;
      tround2 = final_rx.low32 - answer_tx.low32;
      treply2 = final_tx.low32 - answer_rx.low32;

      tprop_ctn = ((tround1*tround2) - (treply1*treply2)) / (tround1 + tround2 + treply1 + treply2);

      tprop = tprop_ctn/tsfreq;
      distance[current_anchor] = C * tprop;
      pressures[current_anchor] = report->asl;

      ranging_complete = true;
      xSemaphoreGive(rangingComplete);

      break;
    }
  }
}

void rxTimeoutCallback(dwDevice_t * dev) {
  // Stop the ranging process on RX timeout
  ranging_complete = false;
  xSemaphoreGive(rangingComplete);
}

/*********** Tasks ************/

volatile static uint16_t rangingState = 0;

static void dwm1000Task(void *param)
{
  //TickType_t xLastWakeTime;
  TickType_t rangingStart;
  TickType_t rangingStop;

  systemWaitStart();

  //xLastWakeTime = xTaskGetTickCount();

  while (1) {
    dwIdle(dwm);

    if (!ranging_complete) {
      rangingState &= ~(1<<current_anchor);
      if (failedRanging[current_anchor] < RANGING_FAILED_TH) {
        failedRanging[current_anchor] ++;
        rangingState |= (1<<current_anchor);
      }
    } else {
      rangingState |= (1<<current_anchor);
      failedRanging[current_anchor] = 0;
    }
    ranging_complete = false;

    current_anchor = (current_anchor+1)%N_ANCHORS;

    txPacket.payload[0] = POLL;
    txPacket.payload[1] = ++curr_seq;

    memcpy(txPacket.sourceAddress, addresses[boardId], 8);
    memcpy(txPacket.destAddress, addresses[anchors[current_anchor]], 8);

    dwNewTransmit(dwm);
    dwSetDefaults(dwm);
    dwSetData(dwm, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2);

    dwWaitForResponse(dwm, true);
    dwStartTransmit(dwm);

    rangingStart = xTaskGetTickCount();
    xSemaphoreTake(rangingComplete, portMAX_DELAY);
    rangingStop = xTaskGetTickCount();

    airtime = rangingStop - rangingStart;

    //vTaskDelayUntil(&xLastWakeTime, M2T(10));
  }
}

static void dwm1000IsrTask(void *param)
{
  systemWaitStart();

  // Service interrupt
  while (1) {
    xSemaphoreTake(dwm1000Event, portMAX_DELAY);
    ITM_SEND(5,0);
    do {
      dwHandleInterrupt(dwm);
    } while(digitalRead(DECK_GPIO_RX1) != 0);
  }
}

static uint8_t spiTxBuffer[196];
static uint8_t spiRxBuffer[196];

/************ Low level ops for libdw **********/
static void spiWrite(dwDevice_t* dev, const void *header, size_t headerLength,
                                      const void* data, size_t dataLength)
{
  xSemaphoreTake(spiSemaphore, portMAX_DELAY);

  digitalWrite(CS_PIN, LOW);
  memcpy(spiTxBuffer, header, headerLength);
  memcpy(spiTxBuffer+headerLength, data, dataLength);
  spiExchange(headerLength+dataLength, spiTxBuffer, spiRxBuffer);
  digitalWrite(CS_PIN, HIGH);

  xSemaphoreGive(spiSemaphore);
}

static void spiRead(dwDevice_t* dev, const void *header, size_t headerLength,
                                     void* data, size_t dataLength)
{
  xSemaphoreTake(spiSemaphore, portMAX_DELAY);

  digitalWrite(CS_PIN, LOW);
  memcpy(spiTxBuffer, header, headerLength);
  memset(spiTxBuffer+headerLength, 0, dataLength);
  spiExchange(headerLength+dataLength, spiTxBuffer, spiRxBuffer);
  memcpy(data, spiRxBuffer+headerLength, dataLength);
  digitalWrite(CS_PIN, HIGH);

  xSemaphoreGive(spiSemaphore);
}

void __attribute__((used)) EXTI11_Callback(void)
{
  portBASE_TYPE  xHigherPriorityTaskWoken = pdFALSE;

  NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
  EXTI_ClearITPendingBit(EXTI_Line11);

  //To unlock RadioTask
  xSemaphoreGiveFromISR(dwm1000Event, &xHigherPriorityTaskWoken);

  if(xHigherPriorityTaskWoken)
    portYIELD();
}

static void spiSetSpeed(dwDevice_t* dev, dwSpiSpeed_t speed)
{
  if (speed == dwSpiSpeedLow)
  {
    spiConfigureSlow();
  }
  else if (speed == dwSpiSpeedHigh)
  {
    spiConfigureFast();
  }
}

static void delayms(dwDevice_t* dev, unsigned int delay)
{
  vTaskDelay(M2T(delay));
}

static dwOps_t dwOps = {
  .spiRead = spiRead,
  .spiWrite = spiWrite,
  .spiSetSpeed = spiSetSpeed,
  .delayms = delayms,
};

/*********** Deck driver initialization ***************/

static bool isInit = false;

static void dwm1000Init(DeckInfo *info)
{
  EXTI_InitTypeDef EXTI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  spiBegin();

  // Init IRQ input
  bzero(&GPIO_InitStructure, sizeof(GPIO_InitStructure));
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  //GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource11);

  EXTI_InitStructure.EXTI_Line = EXTI_Line11;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  // Init reset output
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  // Init CS pin
  pinMode(CS_PIN, OUTPUT);

  // Reset the DW1000 chip
  GPIO_WriteBit(GPIOC, GPIO_Pin_10, 0);
  vTaskDelay(M2T(10));
  GPIO_WriteBit(GPIOC, GPIO_Pin_10, 1);
  vTaskDelay(M2T(10));

  // Semaphore that unlocks the interrupt
  vSemaphoreCreateBinary(dwm1000Event);

  // Semaphore that protect the SPI communication
  spiSemaphore = xSemaphoreCreateMutex();

  vSemaphoreCreateBinary(rangingComplete);

  xTaskCreate(dwm1000IsrTask, "dwmIsrTask", 2*configMINIMAL_STACK_SIZE, NULL,
              /*priority*/5, NULL);
  xTaskCreate(dwm1000Task, "dwmTask", 2*configMINIMAL_STACK_SIZE, NULL,
              /*priority*/3, NULL);

  // Initialize the driver
  dwInit(dwm, &dwOps);       // Init libdw

  int result = dwConfigure(dwm);
  if (result != 0) {
    isInit = false;
    DEBUG_PRINT("Failed to configure DW1000!\r\n");
    return;
  }

  dwEnableAllLeds(dwm);

  dwTime_t delay = {.full = ANTENNA_DELAY/2};
  dwSetAntenaDelay(dwm, delay);

  dwAttachSentHandler(dwm, txCallback);
  dwAttachReceivedHandler(dwm, rxCallback);
  dwAttachReceiveTimeoutHandler(dwm, rxTimeoutCallback);

  dwNewConfiguration(dwm);
  dwSetDefaults(dwm);
  dwEnableMode(dwm, MODE_SHORTDATA_FAST_ACCURACY);
  dwSetChannel(dwm, CHANNEL_2);
  dwSetPreambleCode(dwm, PREAMBLE_CODE_64MHZ_9);

  dwSetReceiveWaitTimeout(dwm, RX_TIMEOUT);

  dwCommitConfiguration(dwm);

  // Enable interrupt
  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_LOW_PRI;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  isInit = true;
}

static bool dwm1000Test()
{
  if (!isInit) {
    DEBUG_PRINT("Error while initializing DWM1000\n");
  }

  return isInit;
}

static const DeckDriver dwm1000_deck = {
  .vid = 0xBC,
  .pid = 0x06,
  .name = "bcDWM1000",

  .usedGpio = 0,  // FIXME: set the used pins

  .init = dwm1000Init,
  .test = dwm1000Test,
};

DECK_DRIVER(dwm1000_deck);

LOG_GROUP_START(ranging)
LOG_ADD(LOG_FLOAT, distance1, &distance[0])
LOG_ADD(LOG_FLOAT, distance2, &distance[1])
LOG_ADD(LOG_FLOAT, distance3, &distance[2])
LOG_ADD(LOG_FLOAT, distance4, &distance[3])
LOG_ADD(LOG_FLOAT, distance5, &distance[4])
LOG_ADD(LOG_FLOAT, distance6, &distance[5])
LOG_ADD(LOG_FLOAT, distance7, &distance[6])
LOG_ADD(LOG_FLOAT, distance8, &distance[7])
LOG_ADD(LOG_FLOAT, pressure1, &pressures[0])
LOG_ADD(LOG_FLOAT, pressure2, &pressures[1])
LOG_ADD(LOG_FLOAT, pressure3, &pressures[2])
LOG_ADD(LOG_FLOAT, pressure4, &pressures[3])
LOG_ADD(LOG_FLOAT, pressure5, &pressures[4])
LOG_ADD(LOG_FLOAT, pressure6, &pressures[5])
LOG_ADD(LOG_FLOAT, pressure7, &pressures[6])
LOG_ADD(LOG_FLOAT, pressure8, &pressures[7])
LOG_ADD(LOG_FLOAT, airtime, &airtime)
LOG_ADD(LOG_UINT16, state, &rangingState)
LOG_GROUP_STOP(ranging)
