/*
 * DeckCtrl SPI bridge for deck drivers
 *
 * These functions are intended to be called from deck drivers to use the
 * deck controller as an SPI master, for instance to program an SPI flash on the deck
 *
 * copyright (C) 2026 Bitcraze AB
 */

#include <deck.h>
#include <string.h>

#include "deckctrl_spi.h"
#include "deckctrl.h"
#include "i2cdev.h"

#include "FreeRTOS.h"
#include "semphr.h"

#define DEBUG_MODULE "DECKCTRL_SPI"
#include "debug.h"

#define CTRL_ENABLE     (1 << 0)
#define CTRL_CPOL       (1 << 1)
#define CTRL_CPHA       (1 << 2)
#define CTRL_BR_SHIFT   3

#define EXEC_TRANSFER   (1 << 0)
#define EXEC_KEEP_CS    (1 << 1)

// TX length (2), RX length (2), execute (1)
#define TRANSFER_HEADER_SIZE 5

// Header and TX data are sent in one I2C write, protected by the mutex
static uint8_t transferBuffer[TRANSFER_HEADER_SIZE + DECKCTRL_SPI_MAX_TRANSFER_SIZE];
static SemaphoreHandle_t transferMutex = NULL;
static StaticSemaphore_t transferMutexBuffer;


static bool get_i2c_address(DeckInfo* info, uint8_t* address) {
    if (info->backendContext == NULL || strcmp(info->discoveryBackend->name, "deckctrl") != 0) {
        return false;
    }
    *address = ((DeckCtrlContext*)info->backendContext)->i2cAddress;
    return true;
}

static SemaphoreHandle_t get_transfer_mutex(void) {
    taskENTER_CRITICAL();
    if (transferMutex == NULL) {
        transferMutex = xSemaphoreCreateMutexStatic(&transferMutexBuffer);
    }
    taskEXIT_CRITICAL();
    return transferMutex;
}

bool deckctrl_spi_enable(DeckInfo* info, DeckCtrlSpiMode mode, DeckCtrlSpiBaudRate baudRate) {
    uint8_t i2c_address;
    if (!get_i2c_address(info, &i2c_address)) {
        return false;
    }

    // Deck controller firmware without the SPI bridge reads 0 here
    uint8_t buf_size_bytes[2];
    if (!i2cdevReadReg16(I2C1_DEV, i2c_address, DECKCTRL_SPI_BUF_SIZE_REG, sizeof(buf_size_bytes), buf_size_bytes)) {
        return false;
    }

    const uint16_t buf_size = buf_size_bytes[0] | ((uint16_t)buf_size_bytes[1] << 8);
    if (buf_size < DECKCTRL_SPI_MAX_TRANSFER_SIZE) {
        DEBUG_PRINT("Deck controller has no SPI bridge support (buffer size %d)\n", buf_size);
        return false;
    }

    uint8_t ctrl = CTRL_ENABLE | ((baudRate & 0x07) << CTRL_BR_SHIFT);
    if (mode & 0x02) {
        ctrl |= CTRL_CPOL;
    }
    if (mode & 0x01) {
        ctrl |= CTRL_CPHA;
    }

    return i2cdevWriteReg16(I2C1_DEV, i2c_address, DECKCTRL_SPI_CTRL_REG, 1, &ctrl);
}

bool deckctrl_spi_disable(DeckInfo* info) {
    uint8_t i2c_address;
    if (!get_i2c_address(info, &i2c_address)) {
        return false;
    }

    uint8_t ctrl = 0;
    return i2cdevWriteReg16(I2C1_DEV, i2c_address, DECKCTRL_SPI_CTRL_REG, 1, &ctrl);
}

bool deckctrl_spi_transfer(DeckInfo* info, const uint8_t* txData, uint16_t txLen, uint8_t* rxData, uint16_t rxLen, bool keepCsAsserted) {
    if ((uint32_t)txLen + rxLen > DECKCTRL_SPI_MAX_TRANSFER_SIZE) {
        return false;
    }
    if ((txLen > 0 && txData == NULL) || (rxLen > 0 && rxData == NULL)) {
        return false;
    }

    uint8_t i2c_address;
    if (!get_i2c_address(info, &i2c_address)) {
        return false;
    }

    SemaphoreHandle_t mutex = get_transfer_mutex();
    xSemaphoreTake(mutex, portMAX_DELAY);

    transferBuffer[0] = txLen & 0xFF;
    transferBuffer[1] = (txLen >> 8) & 0xFF;
    transferBuffer[2] = rxLen & 0xFF;
    transferBuffer[3] = (rxLen >> 8) & 0xFF;
    transferBuffer[4] = EXEC_TRANSFER | (keepCsAsserted ? EXEC_KEEP_CS : 0);
    if (txLen > 0) {
        memcpy(&transferBuffer[TRANSFER_HEADER_SIZE], txData, txLen);
    }

    // The deck controller runs the transfer on the I2C STOP ending this write
    bool result = i2cdevWriteReg16(I2C1_DEV, i2c_address, DECKCTRL_SPI_TRANSFER_REG, TRANSFER_HEADER_SIZE + txLen, transferBuffer);

    // Received bytes are stored at the same buffer position as they were clocked in
    if (result && rxLen > 0) {
        result = i2cdevReadReg16(I2C1_DEV, i2c_address, DECKCTRL_SPI_BUFFER_REG + txLen, rxLen, rxData);
    }

    xSemaphoreGive(mutex);

    return result;
}
