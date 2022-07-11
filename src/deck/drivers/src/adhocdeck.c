#include "stm32fxxx.h"
#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "autoconf.h"
#include "debug.h"
#include "deck.h"
#include "estimator.h"
#include "log.h"
#include "mem.h"
#include "nvicconf.h"
#include "param.h"
#include "statsCnt.h"
#include "system.h"

#include "adhocdeck.h"
#include "dwTypes.h"
#include "libdw3000.h"
#include "dw3000.h"
#include "ranging_struct.h"

#define CS_PIN DECK_GPIO_IO1

// LOCO deck alternative IRQ and RESET pins(IO_2, IO_3) instead of default (RX1,
// TX1), leaving UART1 free for use
#ifdef CONFIG_DECK_LOCODECK_USE_ALT_PINS
#define GPIO_PIN_IRQ DECK_GPIO_IO2

#ifndef LOCODECK_ALT_PIN_RESET
#define GPIO_PIN_RESET DECK_GPIO_IO3
#else
#define GPIO_PIN_RESET LOCODECK_ALT_PIN_RESET
#endif

#define EXTI_PortSource EXTI_PortSourceGPIOB
#define EXTI_PinSource EXTI_PinSource5
#define EXTI_LineN EXTI_Line5
#else
#define GPIO_PIN_IRQ DECK_GPIO_RX1
#define GPIO_PIN_RESET DECK_GPIO_TX1
#define EXTI_PortSource EXTI_PortSourceGPIOC
#define EXTI_PinSource EXTI_PinSource11
#define EXTI_LineN EXTI_Line11
#endif

#define DEFAULT_RX_TIMEOUT 0xFFFFF

static bool isInit = false;
static TaskHandle_t uwbTaskHandle = 0;
static TaskHandle_t uwbTxTaskHandle = 0;
static TaskHandle_t uwbRxTaskHandle = 0;
static TaskHandle_t uwbRangingTaskHandle = 0;
static SemaphoreHandle_t algoSemaphore;
static QueueHandle_t txQueue;
static QueueHandle_t rxQueue;

/* rx buffer used in rx_callback */
static uint8_t rx_buffer[RX_BUFFER_SIZE];
Timestamp_Tuple_t Tf_buffer[Tf_BUFFER_POLL_SIZE] = {0};
static int Tf_buffer_index = 0;
static int seq_number = 0;

static STATS_CNT_RATE_DEFINE(spiWriteCount, 1000);
static STATS_CNT_RATE_DEFINE(spiReadCount, 1000);

static void txCallback() {
  dw_time_t tx_time;
  dwt_readtxtimestamp(&tx_time.raw);
  Tf_buffer_index++;
  Tf_buffer_index %= Tf_BUFFER_POLL_SIZE;
  Tf_buffer[Tf_buffer_index].sequence_number = seq_number;
  Tf_buffer[Tf_buffer_index].timestamp = tx_time;
}

static void rxCallback() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  uint32_t data_length = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_BIT_MASK;
  if (data_length != 0 && data_length <= FRAME_LEN_MAX) {
    dwt_readrxdata(rx_buffer, data_length - FCS_LEN, 0); /* No need to read the FCS/CRC. */
  }
  dw_time_t rx_time;
  dwt_readrxtimestamp(&rx_time.raw);
  Ranging_Message_With_Timestamp_t rx_message_with_timestamp;
  rx_message_with_timestamp.rx_time = rx_time;
  Ranging_Message_t* ranging_message = &rx_buffer;
  rx_message_with_timestamp.ranging_message = *ranging_message;
  xQueueSendFromISR(rxQueue, &rx_message_with_timestamp, &xHigherPriorityTaskWoken);
  dwt_forcetrxoff();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

static void rxTimeoutCallback() {
  dwt_forcetrxoff();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

static void rxErrorCallback() {}

static void uwbTask(void *parameters) {
  while (1) {
    if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY)) {
      do {
        xSemaphoreTake(algoSemaphore, portMAX_DELAY);
        dwt_isr();
        xSemaphoreGive(algoSemaphore);
      } while (digitalRead(GPIO_PIN_IRQ) != 0);
    }
  }
}

static void uwbTxTask(void *parameters) {
  while (!isInit) {
    DEBUG_PRINT("false");
    vTaskDelay(1000);
  }

  Ranging_Message_t packetCache;

  while (true) {
    if (xQueueReceive(txQueue, &packetCache, portMAX_DELAY)) {
      dwt_forcetrxoff();
      dwt_writetxdata(packetCache.header.message_length, &packetCache, 0);
      dwt_writetxfctrl(packetCache.header.message_length + FCS_LEN, 0, 1);
      /* Start transmission. */
      if (dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED) ==
          DWT_ERROR) {
        DEBUG_PRINT("uwbTxTask:  TX ERROR\n");
      }
    }
  }
}

int16_t compute_distance(Ranging_Table_t* table) {

  int64_t tRound1, tReply1, tRound2, tReply2, diff1, diff2, tprop_ctn;
  tRound1 = (table->Rr.timestamp.full - table->Tp.timestamp.full + MAX_TIMESTAMP) % MAX_TIMESTAMP;
  tReply1 = (table->Tr.timestamp.full - table->Rp.timestamp.full + MAX_TIMESTAMP) % MAX_TIMESTAMP;
  tRound2 = (table->Rf.timestamp.full - table->Tr.timestamp.full + MAX_TIMESTAMP) % MAX_TIMESTAMP;
  tReply2 = (table->Tf.timestamp.full - table->Rr.timestamp.full + MAX_TIMESTAMP) % MAX_TIMESTAMP;
  diff1 = tRound1 - tReply1;
  diff2 = tRound2 - tReply2;
  tprop_ctn = (diff1 * tReply2 + diff2 * tReply1 + diff2 * diff1) / (tRound1 + tRound2 + tReply1 + tReply2);

  int16_t distance = (int16_t) tprop_ctn * 0.4691763978616;
  // printf("distance=%d cm\r\n", distance);
  /* update ranging table */
  table->Rp = table->Rf;
  table->Tp = table->Tf;
  table->Rr = table->Re;

  table->Rf.timestamp.full = 0;
  table->Rf.sequence_number = 0;
  
  table->Tf.timestamp.full = 0;
  table->Tf.sequence_number = 0;

  table->Tr.timestamp.full = 0;
  table->Tr.sequence_number = 0;

  return (int16_t)distance;
}

void process_ranging_message(Ranging_Message_With_Timestamp_t* ranging_message_with_timestamp) {
  Ranging_Message_t* ranging_message = &ranging_message_with_timestamp->ranging_message;
  address_t neighbor_addr = ranging_message->header.source_address;
  set_index_t neighbor_index = find_in_ranging_table_set(&ranging_table_set, neighbor_addr);
  /* handle new neighbor */
  if (neighbor_index == -1) {
    if (ranging_table_set.free_queue_entry == -1) {
      /* ranging table set is full */
      return;
    }
    Ranging_Table_t table;
    memset(&table, 0, sizeof(Ranging_Table_t));  // TODO check if necessary
    table.neighbor_address = neighbor_addr;
    table.period = TX_PERIOD_IN_MS;
    table.next_delivery_time = xTaskGetTickCount() + table.period;
    table.expiration_time = xTaskGetTickCount() + M2T(RANGING_TABLE_HOLD_TIME);
    neighbor_index = ranging_table_set_insert(&ranging_table_set, &table);
  }
  Ranging_Table_t* neighbor_ranging_table = &ranging_table_set.set_data[neighbor_index].data;
  /* update Re */
  neighbor_ranging_table->Re.timestamp = ranging_message_with_timestamp->rx_time;
  neighbor_ranging_table->Re.sequence_number = ranging_message->header.message_sequence;

  Timestamp_Tuple_t neighbor_Tr = ranging_message->header.last_tx_timestamp;
  // printf("##########neighbor Tr=%d#########\r\n", neighbor_Tr.sequence_number);
  /* update Tr or Rr*/
  if (neighbor_ranging_table->Tr.timestamp.full == 0) {
    if (neighbor_ranging_table->Rr.sequence_number == neighbor_Tr.sequence_number) {
      neighbor_ranging_table->Tr = neighbor_Tr;
    } else {
      neighbor_ranging_table->Rr = neighbor_ranging_table->Re;
    }
  }
  Timestamp_Tuple_t neighbor_Rf = {.timestamp.full = 0};
  uint8_t body_unit_count = (ranging_message->header.message_length - sizeof(Ranging_Message_Header_t)) / sizeof(Body_Unit_t);
  for (int i = 0; i < body_unit_count; i++) {
    if (ranging_message->body_units[i].address == MY_UWB_ADDRESS) {
      neighbor_Rf = ranging_message->body_units[i].timestamp;
      break;
    }
  }
  /* update Rf and Tf*/
  if (neighbor_Rf.timestamp.full) {
    neighbor_ranging_table->Rf = neighbor_Rf;
    /* find corresponding Tf in Tf_buffer */
    for (int i = 0; i < Tf_BUFFER_POLL_SIZE; i++) {
      if (Tf_buffer[i].sequence_number == neighbor_Rf.sequence_number) {
        neighbor_ranging_table->Tf = Tf_buffer[i];
      }
    }
  }

  if (neighbor_ranging_table->Tr.timestamp.full && neighbor_ranging_table->Rf.timestamp.full && neighbor_ranging_table->Tf.timestamp.full) {
      // printf("===before compute distance===\r\n");
      // print_ranging_table(&ranging_table_set);
      int16_t distance = compute_distance(neighbor_ranging_table);
      printf("distance to neighbor %d = %d cm\r\n", ranging_message->header.source_address, distance);
      // printf("===after compute distance===\r\n");
      // print_ranging_table(&ranging_table_set);
  } else if (neighbor_ranging_table->Rf.timestamp.full && neighbor_ranging_table->Tf.timestamp.full) {
      neighbor_ranging_table->Rp = neighbor_ranging_table->Rf;
      neighbor_ranging_table->Tp = neighbor_ranging_table->Tf;
      neighbor_ranging_table->Rr = neighbor_ranging_table->Re;
      neighbor_ranging_table->Rf.timestamp.full = 0;
      neighbor_ranging_table->Tf.timestamp.full = 0;
      neighbor_ranging_table->Tr.timestamp.full = 0;
  }
  /* update expiration time */
  neighbor_ranging_table->expiration_time = xTaskGetTickCount() + M2T(RANGING_TABLE_HOLD_TIME);
}

static int get_sequence_number() {
  seq_number++;
  return seq_number;
}

static void generate_ranging_message(Ranging_Message_t* ranging_message) {
  /* generate body unit */
  int8_t body_unit_number = 0;
  int cur_seq_number = get_sequence_number();

  for (set_index_t index = ranging_table_set.full_queue_entry; index != -1;
       index = ranging_table_set.set_data[index].next) {
    Ranging_Table_t* table = &ranging_table_set.set_data[index].data;
    if (body_unit_number >= MAX_BODY_UNIT_NUMBER) {
      break;
    }
    if (table->Re.timestamp.full) {
      ranging_message->body_units[body_unit_number].address =
          table->neighbor_address;
      ranging_message->body_units[body_unit_number].timestamp = table->Re;
      body_unit_number++;
      table->Re.sequence_number = 0;
      table->Re.timestamp.full = 0;
    }
  }
  /* generate message header */
  ranging_message->header.source_address = MY_UWB_ADDRESS;
  ranging_message->header.message_length = sizeof(Ranging_Message_Header_t) + sizeof(Body_Unit_t) * body_unit_number;
  // printf("Tx Task Generate: size of ranging_message=%d\r\n", ranging_message->header.message_length);
  // printf("Tx Task Generate: number of body_unit=%d\r\n", body_unit_number);
//   printf("size of message_header=%d, size of body_unit=%d \r\n", sizeof(Ranging_Message_Header_t), sizeof(Body_Unit_t));
  ranging_message->header.message_sequence = cur_seq_number;
  ranging_message->header.last_tx_timestamp = Tf_buffer[Tf_buffer_index];
  ranging_message->header.velocity = 0;
}

static void uwbRxTask(void* parameters) {
  while (!isInit) {
    DEBUG_PRINT("false");
    vTaskDelay(1000);
  }

  Ranging_Message_With_Timestamp_t rx_packet_cache;

  while (true) {
    if (xQueueReceive(rxQueue, &rx_packet_cache, portMAX_DELAY)) {
      process_ranging_message(&rx_packet_cache);
    }
  }
}

static void uwbRangingTask(void* parameters) {
  while (!isInit) {
    printf("false");
    vTaskDelay(1000);
  }

  while (true) {
    Ranging_Message_t tx_packet_cache;
    generate_ranging_message(&tx_packet_cache);
    xQueueSend(txQueue, &tx_packet_cache, portMAX_DELAY);
    vTaskDelay(TX_PERIOD_IN_MS);
  }
}

static uint8_t spiTxBuffer[196];
static uint8_t spiRxBuffer[196];
static uint16_t spiSpeed = SPI_BAUDRATE_2MHZ;

/************ Low level ops for libdw **********/
static void spiWrite(const void *header, size_t headerLength, const void *data,
                     size_t dataLength) {
  spiBeginTransaction(spiSpeed);
  digitalWrite(CS_PIN, LOW);
  memcpy(spiTxBuffer, header, headerLength);
  memcpy(spiTxBuffer + headerLength, data, dataLength);
  spiExchange(headerLength + dataLength, spiTxBuffer, spiRxBuffer);
  digitalWrite(CS_PIN, HIGH);
  spiEndTransaction();
  STATS_CNT_RATE_EVENT(&spiWriteCount);
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
  STATS_CNT_RATE_EVENT(&spiReadCount);
}

#if CONFIG_DECK_LOCODECK_USE_ALT_PINS
void __attribute__((used)) EXTI5_Callback(void)
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


static void uwbInit() {
  // Reset the DW3000 chip
  dwt_ops.reset();
  // dwt_softreset(); // TODO softreset failed, check.
  while (!dwt_checkidlerc()) /* Need to make sure DW IC is in IDLE_RC before proceeding */
  {};
  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
    isInit = false;
    return;
  }
  if (dwt_configure(&config) == DWT_ERROR) {
    while (1) {
      DEBUG_PRINT("dwt_configure error\n");
      vTaskDelay(1000);
    }
    isInit = false;
    return;
  }
  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);
  
  /* Configure the TX spectrum parameters (power, PG delay and PG count) */
  dwt_configuretxrf(&txconfig_options);

  /* Configure Antenna Delay */
  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);

  /* Auto re-enable receiver after a frame reception failure (except a frame
   * wait timeout), the receiver will re-enable to re-attempt reception.*/
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
  dwt_forcetrxoff();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
  algoSemaphore = xSemaphoreCreateMutex();
}

static void pinInit() {
  EXTI_InitTypeDef EXTI_InitStructure;
  // Init pins
  pinMode(CS_PIN, OUTPUT);
  pinMode(GPIO_PIN_RESET, OUTPUT);
  pinMode(GPIO_PIN_IRQ, INPUT);

  spiBegin();
  spiSetSpeed(dwSpiSpeedHigh);

  digitalWrite(GPIO_PIN_RESET, 0);
  vTaskDelay(M2T(10));
  digitalWrite(GPIO_PIN_RESET, 1);
  vTaskDelay(M2T(10));

  // Set up interrupt
  SYSCFG_EXTILineConfig(EXTI_PortSource, EXTI_PinSource);

  EXTI_InitStructure.EXTI_Line = EXTI_LineN;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
}

static void queueInit() {
  txQueue = xQueueCreate(TX_QUEUE_SIZE, TX_QUEUE_ITEM_SIZE);
  rxQueue = xQueueCreate(RX_QUEUE_SIZE, RX_QUEUE_ITEM_SIZE);
}

static void uwbStart() {
 /* Create UWB Task */
  xTaskCreate(uwbTask, ADHOC_DECK_TASK_NAME, 3 * configMINIMAL_STACK_SIZE, NULL,
                    ADHOC_DECK_TASK_PRI, &uwbTaskHandle);
  xTaskCreate(uwbTxTask, ADHOC_DECK_TX_TASK_NAME, 3 * configMINIMAL_STACK_SIZE, NULL,
                    ADHOC_DECK_TASK_PRI, &uwbTxTaskHandle);     
  xTaskCreate(uwbRxTask, ADHOC_DECK_RX_TASK_NAME, 3 * configMINIMAL_STACK_SIZE, NULL,
                    ADHOC_DECK_TASK_PRI, &uwbRxTaskHandle);
  xTaskCreate(uwbRangingTask, ADHOC_DECK_RANGING_TX_TASK_NAME, 3 * configMINIMAL_STACK_SIZE, NULL,
                    ADHOC_DECK_TASK_PRI, &uwbRangingTaskHandle);                
}
/*********** Deck driver initialization ***************/
static void dwm3000Init(DeckInfo *info) {
  pinInit();
  queueInit();
  uwbInit();
  uwbStart();
  isInit = true;
}

static bool dwm3000Test() {
  if (!isInit) {
    DEBUG_PRINT("Error while initializing DWM3000\n");
  }

  return isInit;
}

static const DeckDriver dwm3000_deck = {
    .vid = 0xFF,
    .pid = 0xFF,
    .name = "NetSI_DWM3000",

#ifdef CONFIG_DECK_LOCODECK_USE_ALT_PINS
    .usedGpio = DECK_USING_IO_1 | DECK_USING_IO_2 | DECK_USING_IO_3,
#else
    // (PC10/PC11 is UART1 TX/RX)
    .usedGpio = DECK_USING_IO_1 | DECK_USING_PC10 | DECK_USING_PC11,
#endif
    .usedPeriph = DECK_USING_SPI,
    .requiredEstimator = kalmanEstimator,
#ifdef LOCODECK_NO_LOW_INTERFERENCE
    .requiredLowInterferenceRadioMode = false,
#else
    .requiredLowInterferenceRadioMode = true,
#endif

    .init = dwm3000Init,
    .test = dwm3000Test,
};

DECK_DRIVER(dwm3000_deck);

PARAM_GROUP_START(deck)

PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, NetSI_DWM3000, &isInit)

PARAM_GROUP_STOP(deck)

LOG_GROUP_START(adhoc)
STATS_CNT_RATE_LOG_ADD(spiWr, &spiWriteCount)
STATS_CNT_RATE_LOG_ADD(spiRe, &spiReadCount)
LOG_GROUP_STOP(adhoc)
