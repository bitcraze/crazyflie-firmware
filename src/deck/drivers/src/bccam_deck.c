/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2026 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * bccam_deck.c - Deck driver for the WiFi camera deck
 */

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "system.h"
#include "deck.h"
#include "param.h"
#include "log.h"
#include "i2cdev.h"
#include "deckctrl_gpio.h"
#include "uart1.h"
#include "console.h"

#define DEBUG_MODULE "BCCAM"
#include "debug.h"

#define BCCAM_UART_BAUDRATE       2000000
#define BCCAM_ISP_BAUDRATE        500000

#define BCCAM_FW_RESET_HOLD_MS    50
#define BCCAM_FW_BOOT_SETUP_MS    50
#define BCCAM_FW_BOOT_WAIT_MS     500
#define BCCAM_FW_RX_DRAIN_MS      100

// Deck controller GPIO pin mapping (STM32 deck-ctrl MCU)
#define GPIO_UART1_EN   DECKCTRL_GPIO_PIN_0   // PA0 - UART1 enable
#define GPIO_UART2_EN   DECKCTRL_GPIO_PIN_1   // PA1 - UART2 enable
#define GPIO_QCC_EN     DECKCTRL_GPIO_PIN_2   // PA2 - QCC748 chip enable
#define GPIO_QCC_BOOT   DECKCTRL_GPIO_PIN_7   // PA7 - QCC748 boot select
#define GPIO_PWR_EN     DECKCTRL_GPIO_PIN_12  // PC15 - Power enable

// QCC748 UART ISP protocol constants
#define ISP_HANDSHAKE_BYTE      0x55
#define ISP_HANDSHAKE_COUNT     32
#define ISP_HANDSHAKE_RESP_LEN  16   // "Boot2 ISP Ready\0"
#define ISP_HANDSHAKE_TIMEOUT   M2T(8000)
#define ISP_CMD_TIMEOUT         M2T(5000)

// ISP command IDs
#define ISP_CMD_GET_BOOTINFO    0x10
#define ISP_CMD_CHANGE_RATE     0x20
#define ISP_CMD_RESET           0x21
#define ISP_CMD_CLK_SET         0x22
#define ISP_CMD_FLASH_ERASE     0x30
#define ISP_CMD_FLASH_WRITE     0x31
#define ISP_CMD_FLASH_READ      0x32
#define ISP_CMD_FLASH_SET_PARA  0x3B
#define ISP_CMD_FLASH_CHIPERASE 0x3C
#define ISP_CMD_FLASH_WRITE_CHECK 0x3A
#define ISP_CMD_ISP_MODE        0xA0

// ISP response markers
#define ISP_ACK_OK_L            0x4F  // 'O'
#define ISP_ACK_OK_H            0x4B  // 'K'

static bool isInit = false;
// Written by the deck-memory callbacks / bootloader entry, read by the worker
// task to decide whether a flash session owns the UART — keep it volatile.
static volatile bool isInBootloader = false;
static bool isInFirmware = true;
static uint32_t newFwSize = 0;
static bool flashErased = false;
static DeckInfo *deckInfoG;

// Coalesce host's small CRTP-sized writes into bigger FLASH_WRITE batches.
// QCC748 ROM rejects sequences of tiny writes (returns NACK 0x0006 after
// the first couple). Buffering up to ~1 KiB per FLASH_WRITE avoids that
// small-chunk failure mode.
#define BCCAM_WRITE_BUF_SIZE 1024
static uint8_t writeBuf[BCCAM_WRITE_BUF_SIZE];
static uint16_t writeBufUsed = 0;
static uint32_t writeBufBase = 0;
static uint32_t bytesWritten = 0;

// ---------------------------------------------------------------------------
// UART ISP protocol helpers
// ---------------------------------------------------------------------------

static bool ispRecvBytes(uint8_t *buf, uint32_t len, uint32_t timeoutTicks) {
  for (uint32_t i = 0; i < len; i++) {
    if (!uart1GetDataWithTimeout(&buf[i], timeoutTicks)) {
      return false;
    }
  }
  return true;
}

// Drain the RX queue until the line goes idle. This is not a fixed-time wait:
// it returns as soon as no byte arrives within a 5 ms poll window, and only
// uses timeoutMs as an upper bound. At the baudrates used here a byte takes
// well under 20 us, so 5 ms of silence reliably means the sender has stopped —
// which makes it robust against varying response lengths (boot banners, erase
// progress bytes, etc.) without having to predict the exact byte count.
static void ispDrainRx(uint32_t timeoutMs) {
  uint8_t dummy;
  TickType_t end = xTaskGetTickCount() + M2T(timeoutMs);
  while (xTaskGetTickCount() < end) {
    if (!uart1GetDataWithTimeout(&dummy, M2T(5))) {
      break;
    }
  }
}

static void ispSendCmd(uint8_t cmdId, const uint8_t *payload, uint16_t payloadLen) {
  // Packet: [CMD_ID(1)] [CHECKSUM(1)] [LEN_L(1)] [LEN_H(1)] [PAYLOAD...]
  uint8_t header[4];
  header[0] = cmdId;

  header[2] = (uint8_t)(payloadLen & 0xFF);
  header[3] = (uint8_t)((payloadLen >> 8) & 0xFF);

  // Checksum = sum of len bytes + payload bytes (low byte). The chip is
  // lenient on some commands (e.g. 0x23) but strict on CLK_SET/FLASH_*.
  uint8_t checksum = header[2] + header[3];
  for (uint16_t i = 0; i < payloadLen; i++) {
    checksum += payload[i];
  }
  header[1] = checksum;

  uart1SendData(4, header);
  if (payloadLen > 0 && payload != NULL) {
    uart1SendData(payloadLen, (uint8_t *)payload);
  }
}

// Scan up to ISP_ACK_SCAN_MAX bytes looking for an "OK" or "FL" marker.
// FLASH_ERASE prefixes the response with progress bytes (0x50 0x44 = "PD")
// — about 2 bytes per sector — before the final OK/FL marker. Cap is set
// high enough for a full firmware-region erase.
#define ISP_ACK_SCAN_MAX 2048

static bool ispWaitAck(void) {
  uint8_t prev = 0;
  bool havePrev = false;
  uint32_t scanned = 0;

  for (uint32_t i = 0; i < ISP_ACK_SCAN_MAX; i++) {
    uint8_t b;
    if (!uart1GetDataWithTimeout(&b, ISP_CMD_TIMEOUT)) {
      DEBUG_PRINT("ISP: No ACK after %lu bytes\n", (unsigned long)scanned);
      return false;
    }
    scanned++;

    if (havePrev && prev == ISP_ACK_OK_L && b == ISP_ACK_OK_H) {
      return true;
    }
    if (havePrev && prev == 0x46 /*F*/ && b == 0x4C /*L*/) {
      uint8_t errBytes[2];
      ispRecvBytes(errBytes, 2, M2T(100));
      uint16_t errCode = (uint16_t)errBytes[0] | ((uint16_t)errBytes[1] << 8);
      DEBUG_PRINT("ISP: NACK 0x%04X (after %lu skipped)\n",
                  errCode, (unsigned long)(scanned - 2));
      return false;
    }
    prev = b;
    havePrev = true;
  }

  DEBUG_PRINT("ISP: ACK scan exhausted\n");
  return false;
}

static bool ispFlashErase(uint32_t startAddr, uint32_t endAddr) {
  uint8_t payload[8];
  memcpy(&payload[0], &startAddr, 4);
  memcpy(&payload[4], &endAddr, 4);

  ispSendCmd(ISP_CMD_FLASH_ERASE, payload, 8);
  return ispWaitAck();
}

static bool ispFlashWrite(uint32_t addr, const uint8_t *data, uint16_t len) {
  // Stream header then payload — no large stack copy. ROM accepts up to
  // ~16 KiB per FLASH_WRITE, and we batch up to BCCAM_WRITE_BUF_SIZE
  // (~1 KiB) at the caller.
  uint16_t totalLen = 4 + len;  // addr + data
  uint8_t header[8];
  header[0] = ISP_CMD_FLASH_WRITE;
  header[2] = (uint8_t)(totalLen & 0xFF);
  header[3] = (uint8_t)((totalLen >> 8) & 0xFF);
  memcpy(&header[4], &addr, 4);

  uint8_t cs = header[2] + header[3];
  for (int i = 4; i < 8; i++) cs += header[i];
  for (uint16_t i = 0; i < len; i++) cs += data[i];
  header[1] = cs;

  uart1SendData(sizeof(header), header);
  uart1SendDmaIfAvailable(len, (uint8_t *)data);

  return ispWaitAck();
}

static bool ispFlashRead(uint32_t addr, uint8_t *data, uint16_t len) {
  // Payload: [address(4)] [length(4)]
  uint8_t payload[8];
  uint32_t readLen = len;
  memcpy(&payload[0], &addr, 4);
  memcpy(&payload[4], &readLen, 4);

  ispSendCmd(ISP_CMD_FLASH_READ, payload, 8);
  if (!ispWaitAck()) {
    return false;
  }

  // Response after OK: [data_length(2)] [data(...)]
  uint8_t lenBytes[2];
  if (!ispRecvBytes(lenBytes, 2, ISP_CMD_TIMEOUT)) {
    return false;
  }

  uint16_t respLen = (uint16_t)lenBytes[0] | ((uint16_t)lenBytes[1] << 8);
  if (respLen != len) {
    DEBUG_PRINT("ISP: Read length mismatch (%u vs %u)\n", respLen, len);
    // Drain the announced payload so the queue isn't left desynced for the
    // next command.
    ispDrainRx(100);
    return false;
  }

  return ispRecvBytes(data, len, ISP_CMD_TIMEOUT);
}

// ---------------------------------------------------------------------------
// Deck memory subsystem callbacks (firmware update)
// ---------------------------------------------------------------------------

static bool bcCamWriteFlash(const uint32_t memAddr, const uint8_t writeLen,
                            const uint8_t *buffer, const DeckMemDef_t *memDef) {
  if (!isInBootloader) {
    return false;
  }

  // Auto-erase: on first write, erase the entire region that will be written
  if (!flashErased && newFwSize > 0) {
    uint32_t eraseEnd = memAddr + newFwSize - 1;
    DEBUG_PRINT("ISP: Erasing 0x%08lX - 0x%08lX\n",
                (unsigned long)memAddr, (unsigned long)eraseEnd);
    if (!ispFlashErase(memAddr, eraseEnd)) {
      DEBUG_PRINT("ISP: Flash erase failed\n");
      return false;
    }
    flashErased = true;
    writeBufBase = memAddr;
    writeBufUsed = 0;
    bytesWritten = 0;
  }

  // Non-contiguous write — flush what we have, restart at the new base.
  if (writeBufUsed > 0 && memAddr != writeBufBase + writeBufUsed) {
    if (!ispFlashWrite(writeBufBase, writeBuf, writeBufUsed)) {
      DEBUG_PRINT("ISP: Flush write failed\n");
      return false;
    }
    writeBufBase = memAddr;
    writeBufUsed = 0;
  }
  if (writeBufUsed == 0) {
    writeBufBase = memAddr;
  }

  memcpy(&writeBuf[writeBufUsed], buffer, writeLen);
  writeBufUsed += writeLen;
  bytesWritten += writeLen;

  bool isLast = (newFwSize > 0 && bytesWritten >= newFwSize);
  if (writeBufUsed >= BCCAM_WRITE_BUF_SIZE - 256 || isLast) {
    if (!ispFlashWrite(writeBufBase, writeBuf, writeBufUsed)) {
      DEBUG_PRINT("ISP: write failed @ 0x%08lX (%u B)\n",
                  (unsigned long)writeBufBase, (unsigned)writeBufUsed);
      return false;
    }
    if (isLast) {
      DEBUG_PRINT("ISP: Wrote %lu bytes (full image)\n",
                  (unsigned long)bytesWritten);
      // Drop "in bootloader" so the next flash attempt re-runs the full
      // bootloader entry (cfcli skips reset_to_bootloader if it sees us
      // still active, which would leave our session state stale).
      isInBootloader = false;
      flashErased = false;
      bytesWritten = 0;
      writeBufUsed = 0;
    }
    writeBufBase += writeBufUsed;
    writeBufUsed = 0;
  }

  return true;
}

static bool bcCamReadFlash(const uint32_t memAddr, const uint8_t readLen,
                           uint8_t *buffer) {
  if (!isInBootloader) {
    return false;
  }

  return ispFlashRead(memAddr, buffer, readLen);
}

static uint8_t bcCamPropertiesQuery(void) {
  uint8_t result = DECK_MEMORY_MASK_STARTED | DECK_MEMORY_MASK_SUPPORTS_HOT_RESTART;

  if (isInBootloader) {
    result |= DECK_MEMORY_MASK_BOOT_LOADER_ACTIVE;
  }

  return result;
}

static void bcCamEnterFw(void);

static void bcCamEnterBootloader(void) {
  DEBUG_PRINT("ISP: Bootloader entry starting\n");

  // Reset write-session state — protects against an interrupted previous
  // flash leaving stale counters.
  flashErased = false;
  bytesWritten = 0;
  writeBufUsed = 0;

  uint8_t handshake[ISP_HANDSHAKE_COUNT];
  memset(handshake, ISP_HANDSHAKE_BYTE, ISP_HANDSHAKE_COUNT);

  ispDrainRx(50);

  // Disable QCC748
  deckctrl_gpio_write(deckInfoG, GPIO_QCC_EN, LOW);
  vTaskDelay(M2T(50));

  // Select ROM bootloader (BOOT pin HIGH) before re-enabling
  deckctrl_gpio_write(deckInfoG, GPIO_QCC_BOOT, HIGH);
  vTaskDelay(M2T(10));

  // Switch UART to ROM bootloader baudrate (500000)
  uart1SetBaudrate(BCCAM_ISP_BAUDRATE);

  // Re-enable QCC748 — it will enter ROM bootloader
  deckctrl_gpio_write(deckInfoG, GPIO_QCC_EN, HIGH);
  vTaskDelay(M2T(50));

  // Flood 0x55 and look for "OK" response from ROM bootloader
  bool gotOk = false;
  TickType_t deadline = xTaskGetTickCount() + M2T(5000);

  while (xTaskGetTickCount() < deadline) {
    uart1SendData(ISP_HANDSHAKE_COUNT, handshake);

    uint8_t c;
    if (uart1GetDataWithTimeout(&c, M2T(1))) {
      if (c == ISP_ACK_OK_L) {  // 'O'
        uint8_t c2;
        if (uart1GetDataWithTimeout(&c2, M2T(100))) {
          if (c2 == ISP_ACK_OK_H) {  // 'K'
            gotOk = true;
            break;
          }
        }
      }
    }
  }

  if (!gotOk) {
    DEBUG_PRINT("ISP: Handshake timeout (no OK from ROM)\n");
    goto recover;
  }

  ispDrainRx(50);
  DEBUG_PRINT("ISP: ROM handshake OK\n");

  ispSendCmd(ISP_CMD_GET_BOOTINFO, NULL, 0);
  if (!ispWaitAck()) {
    DEBUG_PRINT("ISP: GET_BOOTINFO failed\n");
    goto recover;
  }
  ispDrainRx(100);
  DEBUG_PRINT("ISP: Got boot info\n");

  // Configure flash SPI pin mux at 500k. Required before any flash op —
  // pin cfg bytes match "set flash cfg: 1014124" from eflash_loader
  // (LE 0x01014124).
  static const uint8_t flashPinCfg[4] = { 0x24, 0x41, 0x01, 0x01 };
  ispSendCmd(ISP_CMD_FLASH_SET_PARA, flashPinCfg, sizeof(flashPinCfg));
  if (!ispWaitAck()) {
    DEBUG_PRINT("ISP: FLASH_SET_PARA NACK at 500k\n");
    goto recover;
  }

  isInBootloader = true;
  DEBUG_PRINT("ISP: ROM bootloader ready\n");
  return;

recover:
  // Couldn't reach the ROM bootloader. Don't leave the deck half-configured
  // (QCC held in reset / BOOT pin high / UART at the ISP baudrate) — drive it
  // back to a known firmware-running state.
  DEBUG_PRINT("ISP: Bootloader entry failed, returning to firmware mode\n");
  bcCamEnterFw();
}

static volatile bool bootloaderRequested = false;

static void bcCamResetToBootloader(void) {
  DEBUG_PRINT("=== BOOTLOADER CALLBACK CALLED ===\n");
  isInFirmware = false;
  isInBootloader = false;
  flashErased = false;
  bootloaderRequested = true;
}

static void bcCamEnterFw(void) {
  // Disable QCC748
  deckctrl_gpio_write(deckInfoG, GPIO_QCC_EN, LOW);
  vTaskDelay(M2T(BCCAM_FW_RESET_HOLD_MS));

  // Set boot select LOW for normal boot
  deckctrl_gpio_write(deckInfoG, GPIO_QCC_BOOT, LOW);
  vTaskDelay(M2T(BCCAM_FW_BOOT_SETUP_MS));

  // Switch back to firmware baudrate
  uart1SetBaudrate(BCCAM_UART_BAUDRATE);

  // Re-enable QCC748 - it will boot firmware
  deckctrl_gpio_write(deckInfoG, GPIO_QCC_EN, HIGH);
  vTaskDelay(M2T(BCCAM_FW_BOOT_WAIT_MS));

  ispDrainRx(BCCAM_FW_RX_DRAIN_MS);

  isInFirmware = true;
  DEBUG_PRINT("QCC748 in firmware mode\n");
}

static volatile bool fwResetRequested = false;

static void bcCamResetToFw(void) {
  isInBootloader = false;
  isInFirmware = false;
  flashErased = false;
  fwResetRequested = true;
}

static const DeckMemDef_t bcCamMemoryDef = {
  .write = bcCamWriteFlash,
  .read = bcCamReadFlash,
  .properties = bcCamPropertiesQuery,
  .supportsUpgrade = true,
  .newFwSizeP = &newFwSize,
  .id = "qcc",
  .commandResetToBootloader = bcCamResetToBootloader,
  .commandResetToFw = bcCamResetToFw,
};

// ---------------------------------------------------------------------------
// Worker task — drives bootloader/firmware transitions off the deck-memory
// callback context and keeps the UART RX queue drained when idle.
// ---------------------------------------------------------------------------

static void bcCamWorkerTask(void *arg) {
  systemWaitStart();

  DEBUG_PRINT("Worker task started\n");

  while (1) {
    if (bootloaderRequested) {
      DEBUG_PRINT("Bootloader requested!\n");
      bootloaderRequested = false;
      bcCamEnterBootloader();
    } else if (fwResetRequested) {
      fwResetRequested = false;
      bcCamEnterFw();
    } else if (isInBootloader) {
      // A flash session is active and the deck-memory callbacks (running on
      // another task) own the UART — they consume the ISP responses. Stay off
      // the RX queue here so we don't steal their bytes, but keep polling the
      // request flags frequently.
      vTaskDelay(M2T(10));
    } else {
      // Drain UART RX to prevent overflow, but check flags frequently
      uint8_t c;
      uart1GetDataWithTimeout(&c, M2T(1));
    }
  }
}

// ---------------------------------------------------------------------------
// Deck driver init / test
// ---------------------------------------------------------------------------

static bool gpioSetup(DeckInfo *info, DeckCtrlGPIOPin pin, uint32_t direction, uint32_t value) {
  if (!deckctrl_gpio_set_direction(info, pin, direction)) {
    return false;
  }
  if (direction == OUTPUT) {
    return deckctrl_gpio_write(info, pin, value);
  }
  return true;
}

static void bcCamDeckInit(DeckInfo *info) {
  if (isInit) {
    return;
  }

  deckInfoG = info;

  // Power on the deck
  if (!gpioSetup(info, GPIO_PWR_EN, OUTPUT, HIGH)) {
    DEBUG_PRINT("Failed to enable deck power\n");
    return;
  }

  vTaskDelay(M2T(50));

  // Boot into normal mode (not bootloader)
  if (!gpioSetup(info, GPIO_QCC_BOOT, OUTPUT, LOW)) {
    DEBUG_PRINT("Failed to set boot select\n");
    return;
  }

  // Enable QCC748
  if (!gpioSetup(info, GPIO_QCC_EN, OUTPUT, HIGH)) {
    DEBUG_PRINT("Failed to enable QCC748\n");
    return;
  }

  vTaskDelay(M2T(100));

  // Enable UART1 pass-through on deck controller
  if (!gpioSetup(info, GPIO_UART1_EN, OUTPUT, HIGH)) {
    DEBUG_PRINT("Failed to enable UART1\n");
    return;
  }

  // Initialize UART1 for QCC748 communication
  uart1Init(BCCAM_UART_BAUDRATE);

  xTaskCreate(bcCamWorkerTask, "bcCam", configMINIMAL_STACK_SIZE * 4,
              NULL, tskIDLE_PRIORITY, NULL);

  isInit = true;
  DEBUG_PRINT("bcCam deck initialized\n");
}

static bool bcCamDeckTest(void) {
  if (!isInit) {
    return false;
  }

  DEBUG_PRINT("bcCam deck test passed\n");
  return true;
}

static const DeckDriver bccam_deck = {
  .vid = 0xBC,
  .pid = 0x15,
  .name = "bcCam",

  .usedPeriph = DECK_USING_UART1,

  .memoryDef = &bcCamMemoryDef,

  .init = bcCamDeckInit,
  .test = bcCamDeckTest,
};

DECK_DRIVER(bccam_deck);

PARAM_GROUP_START(deck)

/**
 * @brief Nonzero if bcCam deck is attached
 */
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcCam, &isInit)

PARAM_GROUP_STOP(deck)
