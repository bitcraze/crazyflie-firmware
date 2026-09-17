#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "deck.h"

// SPI bridge register addresses
#define DECKCTRL_SPI_CTRL_REG       0x2000
#define DECKCTRL_SPI_STATUS_REG     0x2001
#define DECKCTRL_SPI_BUF_SIZE_REG   0x2002
#define DECKCTRL_SPI_TRANSFER_REG   0x20FB  // TX length, RX length and execute, directly followed by the buffer
#define DECKCTRL_SPI_BUFFER_REG     0x2100

/**
 * @brief Maximum number of bytes (sent + received) in one transfer
 *
 * This is the buffer size of the deck controller firmware that introduced the SPI bridge,
 * deck controllers with a smaller buffer are rejected by deckctrl_spi_enable().
 */
#define DECKCTRL_SPI_MAX_TRANSFER_SIZE 512

typedef enum {
    DECKCTRL_SPI_MODE_0 = 0, // CPOL=0, CPHA=0
    DECKCTRL_SPI_MODE_1 = 1, // CPOL=0, CPHA=1
    DECKCTRL_SPI_MODE_2 = 2, // CPOL=1, CPHA=0
    DECKCTRL_SPI_MODE_3 = 3, // CPOL=1, CPHA=1
} DeckCtrlSpiMode;

/**
 * @brief SPI clock as a divider of the deck controller peripheral clock
 *
 * The frequencies are for the current deck controller firmware which runs at 12 MHz.
 */
typedef enum {
    DECKCTRL_SPI_DIV_2 = 0,   // 6 MHz
    DECKCTRL_SPI_DIV_4 = 1,   // 3 MHz
    DECKCTRL_SPI_DIV_8 = 2,   // 1.5 MHz
    DECKCTRL_SPI_DIV_16 = 3,  // 750 kHz
    DECKCTRL_SPI_DIV_32 = 4,  // 375 kHz
    DECKCTRL_SPI_DIV_64 = 5,  // 187.5 kHz
    DECKCTRL_SPI_DIV_128 = 6, // 93.75 kHz
    DECKCTRL_SPI_DIV_256 = 7, // 46.875 kHz
} DeckCtrlSpiBaudRate;

/**
 * @brief Enable the SPI bridge of the deck controller
 *
 * Configures SPI1 of the deck controller as master and takes over the SPI pins
 * (CS is released/high). While enabled the pins can not be controlled through
 * the deck controller GPIO API.
 *
 * Can be called again while enabled to change mode or baud rate.
 *
 * @param info Pointer to the DeckInfo structure of the deck
 * @param mode SPI mode
 * @param baudRate SPI clock divider
 * @return true if enabled, false on I2C error or if the deck controller firmware
 *         does not support the SPI bridge
 */
bool deckctrl_spi_enable(DeckInfo* info, DeckCtrlSpiMode mode, DeckCtrlSpiBaudRate baudRate);

/**
 * @brief Disable the SPI bridge of the deck controller
 *
 * Releases CS and hands the pins back to the deck controller GPIO configuration
 * (inputs by default), so that another master can use the SPI bus.
 *
 * @param info Pointer to the DeckInfo structure of the deck
 * @return true if the operation was successful, false otherwise
 */
bool deckctrl_spi_disable(DeckInfo* info);

/**
 * @brief Do an SPI transfer through the deck controller
 *
 * Asserts CS, sends txLen bytes from txData and then clocks rxLen bytes (sending 0xFF)
 * into rxData. CS is released at the end unless keepCsAsserted is set, in which case
 * the next transfer continues with CS still asserted. A transfer with no data only
 * sets the CS state.
 *
 * txLen + rxLen must not exceed DECKCTRL_SPI_MAX_TRANSFER_SIZE.
 *
 * @param info Pointer to the DeckInfo structure of the deck
 * @param txData Data to send, can be NULL if txLen is 0
 * @param txLen Number of bytes to send
 * @param rxData Buffer for received data, can be NULL if rxLen is 0
 * @param rxLen Number of bytes to receive after the sent bytes
 * @param keepCsAsserted Keep CS asserted after the transfer
 * @return true if the transfer was successful, false otherwise
 */
bool deckctrl_spi_transfer(DeckInfo* info, const uint8_t* txData, uint16_t txLen, uint8_t* rxData, uint16_t rxLen, bool keepCsAsserted);
