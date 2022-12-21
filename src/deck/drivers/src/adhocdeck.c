#define DEBUG_MODULE "DWM3k"

#include <stdint.h>
#include <string.h>

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
#include "swarm_ranging.h"
#include "flooding.h"
#include "routing.h"

#define CS_PIN DECK_GPIO_IO1

// LOCO deck alternative IRQ and RESET pins(IO_2, IO_4) instead of default (RX1, TX1), leaving UART1 free for use
#ifdef CONFIG_DECK_ADHOCDECK_USE_ALT_PINS
#define GPIO_PIN_IRQ      DECK_GPIO_IO2

#ifndef ADHOCDECK_ALT_PIN_RESET
#define GPIO_PIN_RESET    DECK_GPIO_IO4
#else
#define GPIO_PIN_RESET 	ADHOCDECK_ALT_PIN_RESET
#endif

#define EXTI_PortSource EXTI_PortSourceGPIOB
#define EXTI_PinSource    EXTI_PinSource5
#define EXTI_LineN          EXTI_Line5
#elif defined(CONFIG_DECK_ADHOCDECK_USE_UART2_PINS)
#define GPIO_PIN_IRQ 	  DECK_GPIO_TX2
#define GPIO_PIN_RESET 	DECK_GPIO_RX2
#define EXTI_PortSource EXTI_PortSourceGPIOA
#define EXTI_PinSource 	EXTI_PinSource2
#define EXTI_LineN 		  EXTI_Line2
#else
#define GPIO_PIN_IRQ      DECK_GPIO_RX1
#define GPIO_PIN_RESET    DECK_GPIO_TX1
#define EXTI_PortSource EXTI_PortSourceGPIOC
#define EXTI_PinSource    EXTI_PinSource11
#define EXTI_LineN          EXTI_Line11
#endif

#define DEFAULT_RX_TIMEOUT 0xFFFFF

static uint16_t MY_UWB_ADDRESS;
static bool isInit = false;
#ifdef CONFIG_DECK_ADHOCDECK_USE_UART2_PINS
static bool isUWBStart = false;
#endif
static TaskHandle_t uwbTaskHandle = 0;
static TaskHandle_t uwbTxTaskHandle = 0;
static SemaphoreHandle_t irqSemaphore;

static QueueHandle_t txQueue;
static xQueueHandle queues[MESSAGE_TYPE_COUNT];
static UWB_Message_Listener_t listeners[MESSAGE_TYPE_COUNT];
static MESSAGE_TYPE TX_MESSAGE_TYPE;

static int packetSeqNumber = 1;

/* rx buffer used in rx_callback */
static uint8_t rxBuffer[FRAME_LEN_MAX];

static void txCallback() {
  packetSeqNumber++;
  if (TX_MESSAGE_TYPE < MESSAGE_TYPE_COUNT && listeners[TX_MESSAGE_TYPE].txCb) {
    listeners[TX_MESSAGE_TYPE].txCb(NULL); // TODO no parameter passed into txCb now
  }
}

static void rxCallback() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  uint32_t dataLength = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_BIT_MASK;
  if (dataLength != 0 && dataLength <= FRAME_LEN_MAX) {
    dwt_readrxdata(rxBuffer, dataLength - FCS_LEN, 0); /* No need to read the FCS/CRC. */
  }
//  DEBUG_PRINT("rxCallback: data length = %lu \n", dataLength);

  UWB_Packet_t *packet = (UWB_Packet_t *) &rxBuffer;
  MESSAGE_TYPE msgType = packet->header.type;

  ASSERT(msgType < MESSAGE_TYPE_COUNT);

  if (listeners[msgType].rxCb) {
    listeners[msgType].rxCb(packet);
  }

  if (listeners[msgType].rxQueue) {
    xQueueSendFromISR(listeners[msgType].rxQueue, packet, &xHigherPriorityTaskWoken);
  }

  dwt_forcetrxoff();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

static void rxTimeoutCallback() {
  dwt_forcetrxoff();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

static void rxErrorCallback() {
  DEBUG_PRINT("rxErrorCallback: some error occurs when rx\n");
}

uint16_t getUWBAddress() {
  return MY_UWB_ADDRESS;
}

int uwbSendPacket(UWB_Packet_t *packet) {
  xQueueSend(txQueue, packet, 0);
}

int uwbSendPacketBlock(UWB_Packet_t *packet) {
  xQueueSend(txQueue, packet, portMAX_DELAY);
}

int uwbReceivePacket(MESSAGE_TYPE type, UWB_Packet_t *packet) {
  ASSERT(type < MESSAGE_TYPE_COUNT);
  return xQueueReceive(queues[type], packet, 0);
}

int uwbReceivePacketBlock(MESSAGE_TYPE type, UWB_Packet_t *packet) {
  ASSERT(type < MESSAGE_TYPE_COUNT);
  return xQueueReceive(queues[type], packet, portMAX_DELAY);
}

int uwbReceivePacketWait(MESSAGE_TYPE type, UWB_Packet_t *packet, int wait) {
  ASSERT(type < MESSAGE_TYPE_COUNT);
  return xQueueReceive(queues[type], packet, M2T(wait));
}

void uwbRegisterListener(UWB_Message_Listener_t *listener) {
  queues[listener->type] = listener->rxQueue;
  listeners[listener->type] = *listener;
}

static int uwbInit() {
  /* Need to make sure DW IC is in IDLE_RC before proceeding */
  while (!dwt_checkidlerc()) {

  }
#ifdef CONFIG_DECK_ADHOCDECK_USE_UART2_PINS
  while (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
    vTaskDelay(100);
  }
  while (dwt_configure(&config) == DWT_ERROR) {
    vTaskDelay(100);
  }
#else
  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
    return DWT_ERROR;
  }
  if (dwt_configure(&config) == DWT_ERROR) {
    return DWT_ERROR;
  }
#endif
  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

  /* Configure the TX spectrum parameters (power, PG delay and PG count) */
  dwt_configuretxrf(&txconfig_options);

  /* Configure Antenna Delay */
  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);

  /* Auto re-enable receiver after a frame reception failure (except a frame
   * wait timeout), the receiver will re-enable to re-attempt reception. */
  dwt_or32bitoffsetreg(SYS_CFG_ID, 0, SYS_CFG_RXAUTR_BIT_MASK);
  dwt_setrxtimeout(DEFAULT_RX_TIMEOUT);

  dwt_setcallbacks(&txCallback, &rxCallback, &rxTimeoutCallback, &rxErrorCallback, NULL, NULL);
  /* Enable wanted interrupts (TX confirmation, RX good frames, RX timeouts and
   * RX errors). */
  dwt_setinterrupt(SYS_ENABLE_LO_TXFRS_ENABLE_BIT_MASK |
                       SYS_ENABLE_LO_RXFCG_ENABLE_BIT_MASK |
                       SYS_ENABLE_LO_RXFTO_ENABLE_BIT_MASK |
                       SYS_ENABLE_LO_RXPTO_ENABLE_BIT_MASK |
                       SYS_ENABLE_LO_RXPHE_ENABLE_BIT_MASK |
                       SYS_ENABLE_LO_RXFCE_ENABLE_BIT_MASK |
                       SYS_ENABLE_LO_RXFSL_ENABLE_BIT_MASK |
                       SYS_ENABLE_LO_RXSTO_ENABLE_BIT_MASK,
                   0, DWT_ENABLE_INT);

  /* Clearing the SPI ready interrupt */
  dwt_write32bitreg(SYS_STATUS_ID,
                    SYS_STATUS_RCINIT_BIT_MASK | SYS_STATUS_SPIRDY_BIT_MASK);

  irqSemaphore = xSemaphoreCreateMutex();

  return DWT_SUCCESS;
}

static void uwbTxTask(void *parameters) {
  systemWaitStart();
#ifdef CONFIG_DECK_ADHOCDECK_USE_UART2_PINS
  while (!isUWBStart) {
    vTaskDelay(200);
  }
#endif

  UWB_Packet_t packetCache;
  while (true) {
    if (xQueueReceive(txQueue, &packetCache, portMAX_DELAY)) {
      dwt_forcetrxoff();
      dwt_writetxdata(packetCache.header.length, (uint8_t *) &packetCache, 0);
      dwt_writetxfctrl(packetCache.header.length + FCS_LEN, 0, 1);
      /* Start transmission. */
      if (dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED) ==
          DWT_ERROR) {
        DEBUG_PRINT("uwbTxTask:  TX ERROR\n");
      }
    }
  }
}

static void uwbTask(void *parameters) {
  systemWaitStart();
#ifdef CONFIG_DECK_ADHOCDECK_USE_UART2_PINS
  if (uwbInit() == DWT_SUCCESS) {
    isUWBStart = true;
  }
#endif

  while (1) {
    if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY)) {
      do {
        xSemaphoreTake(irqSemaphore, portMAX_DELAY);
        dwt_isr();
        xSemaphoreGive(irqSemaphore);
#ifdef CONFIG_DECK_ADHOCDECK_USE_UART2_PINS
        vTaskDelay(M2T(5)); // TODO check if necessary since increasing FREERTOS_HEAP_SIZE
#endif
      } while (digitalRead(GPIO_PIN_IRQ) != 0);
    }
  }
}

/************ Low level ops for libdw **********/

static uint8_t spiTxBuffer[FRAME_LEN_MAX];
static uint8_t spiRxBuffer[FRAME_LEN_MAX];
static uint16_t spiSpeed = SPI_BAUDRATE_2MHZ;

static void spiWrite(const void *header, size_t headerLength, const void *data,
                     size_t dataLength) {
  spiBeginTransaction(spiSpeed);
  digitalWrite(CS_PIN, LOW);
  memcpy(spiTxBuffer, header, headerLength);
  memcpy(spiTxBuffer + headerLength, data, dataLength);
  spiExchange(headerLength + dataLength, spiTxBuffer, spiRxBuffer);
  digitalWrite(CS_PIN, HIGH);
  spiEndTransaction();
}

static void spiRead(const void *header, size_t headerLength, void *data,
                    size_t dataLength) {
  spiBeginTransaction(spiSpeed);
  digitalWrite(CS_PIN, LOW);
  memcpy(spiTxBuffer, header, headerLength);
  memset(spiTxBuffer + headerLength, 0, dataLength);
  spiExchange(headerLength + dataLength, spiTxBuffer, spiRxBuffer);
  memcpy(data, spiRxBuffer + headerLength, dataLength);
  digitalWrite(CS_PIN, HIGH);
  spiEndTransaction();
}

#ifdef CONFIG_DECK_ADHOCDECK_USE_ALT_PINS
void __attribute__((used)) EXTI5_Callback(void)
#elif defined(CONFIG_DECK_ADHOCDECK_USE_UART2_PINS)
void __attribute__((used)) EXTI2_Callback(void)
#else
void __attribute__((used)) EXTI11_Callback(void)
#endif
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  // Unlock interrupt handling task
  vTaskNotifyGiveFromISR(uwbTaskHandle, &xHigherPriorityTaskWoken);

  if (xHigherPriorityTaskWoken) {
    portYIELD();
  }
}

static void spiSetSpeed(dwSpiSpeed_t speed) {
  if (speed == dwSpiSpeedLow) {
    spiSpeed = SPI_BAUDRATE_2MHZ;
  } else if (speed == dwSpiSpeedHigh) {
    spiSpeed = SPI_BAUDRATE_21MHZ;
  }
}

static void delayms(unsigned int delay) { vTaskDelay(M2T(delay)); }

static void reset(void) {
  digitalWrite(GPIO_PIN_RESET, 0);
  vTaskDelay(M2T(10));
  digitalWrite(GPIO_PIN_RESET, 1);
  vTaskDelay(M2T(10));
}
extern dwOps_t dwt_ops = {
    .spiRead = spiRead,
    .spiWrite = spiWrite,
    .spiSetSpeed = spiSetSpeed,
    .delayms = delayms,
    .reset = reset
};

static void pinInit() {
  EXTI_InitTypeDef EXTI_InitStructure;

  spiBegin();

  // Set up interrupt
  SYSCFG_EXTILineConfig(EXTI_PortSource, EXTI_PinSource);

  EXTI_InitStructure.EXTI_Line = EXTI_LineN;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  // Init pins
  pinMode(CS_PIN, OUTPUT);
  pinMode(GPIO_PIN_RESET, OUTPUT);
  pinMode(GPIO_PIN_IRQ, INPUT);

  //Reset the DW3000 chip
  dwt_ops.reset();
}

static void queueInit() {
  txQueue = xQueueCreate(TX_QUEUE_SIZE, TX_QUEUE_ITEM_SIZE);
}

static void uwbTaskInit() {
  /* Create UWB Task */
  xTaskCreate(uwbTask, ADHOC_DECK_TASK_NAME, 4 * configMINIMAL_STACK_SIZE, NULL,
              ADHOC_DECK_TASK_PRI, &uwbTaskHandle); // TODO optimize STACK SIZE
  xTaskCreate(uwbTxTask, ADHOC_DECK_TX_TASK_NAME, 4 * configMINIMAL_STACK_SIZE, NULL,
              ADHOC_DECK_TASK_PRI, &uwbTxTaskHandle); // TODO optimize STACK SIZE
  rangingInit();
//  routingInit();
  floodingInit();

}
/*********** Deck driver initialization ***************/
static void dwm3000Init(DeckInfo *info) {
  pinInit();
  queueInit();
#ifdef CONFIG_DECK_ADHOCDECK_USE_UART2_PINS
  uwbTaskInit();
  isInit = true;
#else
  if (uwbInit() == DWT_SUCCESS) {
    uwbTaskInit();
    isInit = true;
  } else {
    isInit = false;
  }
#endif
}

static bool dwm3000Test() {
  if (!isInit) {
    DEBUG_PRINT("Error while initializing DWM3000\n");
  }

  return isInit;
}

static const DeckDriver dwm3000_deck = {
    .vid = 0xBC,
    .pid = 0x06,
    .name = "DWM3000",

#ifdef CONFIG_DECK_ADHOCDECK_USE_ALT_PINS
    .usedGpio = DECK_USING_IO_1 | DECK_USING_IO_2 | DECK_USING_IO_4,
#elif defined(CONFIG_DECK_ADHOCDECK_USE_UART2_PINS)
    .usedGpio = DECK_USING_IO_1 | DECK_USING_UART2,
#else
    .usedGpio = DECK_USING_IO_1 | DECK_USING_UART1,
#endif
    .usedPeriph = DECK_USING_SPI,
    .requiredEstimator = kalmanEstimator,
#ifdef ADHOCDECK_NO_LOW_INTERFERENCE
    .requiredLowInterferenceRadioMode = false,
#else
    .requiredLowInterferenceRadioMode = true,
#endif

    .init = dwm3000Init,
    .test = dwm3000Test,
};

DECK_DRIVER(dwm3000_deck);

PARAM_GROUP_START(deck)
        PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, DWM3000, &isInit)
PARAM_GROUP_STOP(deck)

PARAM_GROUP_START(ADHOC)
        PARAM_ADD_CORE(PARAM_UINT16 | PARAM_PERSISTENT, MY_UWB_ADDRESS, &MY_UWB_ADDRESS)
PARAM_GROUP_STOP(ADHOC)