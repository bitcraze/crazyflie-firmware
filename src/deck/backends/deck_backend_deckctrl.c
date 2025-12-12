/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2025 Bitcraze AB
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
 * deck_backend_deckctrl.c - DeckCtrl I2C-based discovery backend
 *
 * This backend discovers decks equipped with intelligent microcontrollers that
 * implement the DeckCtrl protocol. Unlike OneWire decks with passive memory,
 * DeckCtrl decks actively participate in enumeration and can be dynamically
 * assigned unique I2C addresses.
 *
 * Discovery Protocol:
 * 1. Reset all deck controllers (broadcast to 0x41)
 * 2. Put unconfigured controllers in listening mode (0x42)
 * 3. Read next deck's CPU ID from default address (0x43)
 * 4. Assign unique I2C address (0x44+) to that deck
 * 5. Read deck information (VID/PID/name) from assigned address
 * 6. Repeat from step 2 until no more decks respond
 *
 * See docs/functional-areas/deckctrl_protocol.md for full protocol specification.
 */

#include <string.h>
#include <stdlib.h>

#include "deck_discovery.h"
#include "deck.h"
#include "deckctrl.h"
#include "autoconf.h"
#include "i2cdev.h"
#include "deckctrl_gpio.h"
#include "mem.h"

#define DEBUG_MODULE "DECKCTRL"
#include "debug.h"

// Only enable debug prints if CONFIG_DECK_BACKEND_DECKCTRL_DEBUG is set by kconfig
#ifdef CONFIG_DECK_BACKEND_DECKCTRL_DEBUG
    #define DECKCTRL_DEBUG_PRINT DEBUG_PRINT
#else
    #define DECKCTRL_DEBUG_PRINT(...)
#endif

static const DeckDiscoveryBackend_t deckctrlBackend;

/**
 * DeckCtrl I2C address definitions
 *
 * The protocol uses special addresses for broadcast operations and a range
 * of addresses for individually assigned deck controllers.
 */
#define DECKCTRL_I2C_ADDRESS_RESET    DECKCTRL_RESET_I2C_ADDRESS    ///< Broadcast: Reset all deck controllers
#define DECKCTRL_I2C_ADDRESS_LISTEN   DECKCTRL_LISTEN_I2C_ADDRESS   ///< Broadcast: Put unconfigured decks in listening mode
#define DECKCTRL_I2C_ADDRESS_DEFAULT  DECKCTRL_DEFAULT_I2C_ADDRESS  ///< Default: Read CPU ID and assign address

// Address range for assigning I2C addresses to deck controllers
// The limit of possible decks is set by CONFIG_DECK_BACKEND_DECKCTRL_MAX_DECKS in Kconfig
#define DECKCTRL_I2C_ADDRESS_START    DECKCTRL_START_I2C_ADDRESS  ///< First assigned address
#define DECKCTRL_I2C_ADDRESS_END      (DECKCTRL_START_I2C_ADDRESS + CONFIG_DECK_BACKEND_DECKCTRL_MAX_DECKS - 1)  ///< Last assigned address

/**
 * DeckCtrl memory layout at register 0x0000
 *
 * Total size: 21 bytes
 * Format: [MAGIC(2)] [VERSIONS(2)] [VID(1)] [PID(1)] [REV(1)] [NAME(14)]
 */
#define DECKCTRL_MEM_MAGIC_OFFSET_0   0    ///< Magic number byte 0: 0xBC (high byte)
#define DECKCTRL_MEM_MAGIC_OFFSET_1   1    ///< Magic number byte 1: 0xDC (low byte)
#define DECKCTRL_MEM_MAJOR_VER_OFFSET 2    ///< Firmware major version
#define DECKCTRL_MEM_MINOR_VER_OFFSET 3    ///< Firmware minor version
#define DECKCTRL_MEM_VENDOR_ID_OFFSET 4    ///< Vendor ID (matches DeckDriver.vid)
#define DECKCTRL_MEM_PRODUCT_ID_OFFSET 5   ///< Product ID (matches DeckDriver.pid)
#define DECKCTRL_MEM_BOARD_REV_OFFSET 6    ///< Board revision ASCII character
#define DECKCTRL_MEM_PRODUCT_NAME_OFFSET 7 ///< Product name (null-terminated, max 14 chars)

#define DECKCTRL_MEM_PRODUCTION_YEAR_OFFSET 0x16  ///< Production year offset
#define DECKCTRL_MEM_PRODUCTION_MONTH_OFFSET 0x17 ///< Production month offset
#define DECKCTRL_MEM_PRODUCTION_DAY_OFFSET 0x18   ///< Production day offset

#define DECKCTRL_CONFIG_PAGE_SIZE 2048     ///< Size of configuration memory page
#define DECKCTRL_SERIAL_SIZE 12            ///< Size of serial number stored in memory

/**
 * Static storage for discovered decks
 *
 * Memory is allocated at compile time based on CONFIG_DECK_BACKEND_DECKCTRL_MAX_DECKS.
 * Each discovered deck gets an entry in these arrays with its information and context.
 */
static DeckInfo deck_info[CONFIG_DECK_BACKEND_DECKCTRL_MAX_DECKS];           ///< Deck information structures
static char product_names[CONFIG_DECK_BACKEND_DECKCTRL_MAX_DECKS][16];       ///< Product name strings
static DeckCtrlContext deck_contexts[CONFIG_DECK_BACKEND_DECKCTRL_MAX_DECKS]; ///< Backend-specific contexts
static int deck_count = 0;  ///< Number of decks discovered so far

static MemoryHandlerDef_t deckctrl_memory_handlers[CONFIG_DECK_BACKEND_DECKCTRL_MAX_DECKS];

static bool deckctrlMemoryRead(const uint8_t internal_id, const uint32_t memAddr, const uint8_t readLen, uint8_t* buffer) {
    return i2cdevReadReg16(I2C1_DEV, deck_contexts[internal_id].i2cAddress, memAddr, readLen, buffer);
}

static uint32_t deckctrlMemorySize(const uint8_t internal_id) {
    return DECKCTRL_CONFIG_PAGE_SIZE;
}

static bool deckctrlMemorySerialNbr(const uint8_t internal_id, const uint8_t max_length, uint8_t* len, uint8_t* buffer) {
    ASSERT(max_length >= DECKCTRL_SERIAL_SIZE);
    *len = DECKCTRL_SERIAL_SIZE;
    return i2cdevReadReg16(I2C1_DEV, deck_contexts[internal_id].i2cAddress, 0x1900, DECKCTRL_SERIAL_SIZE, buffer);
}

/**
 * @brief Initialize the DeckCtrl discovery backend
 *
 * This function is called once at system startup to initialize the DeckCtrl
 * discovery hardware and reset all deck controllers to a known state.
 *
 * @return true if initialization succeeded (always succeeds even if no decks present)
 */
static bool deckctrl_init(void)
{
    DECKCTRL_DEBUG_PRINT("Initializing DeckCtrl backend with support for %d decks\n", CONFIG_DECK_BACKEND_DECKCTRL_MAX_DECKS);

    // Reset all deck controllers on the bus by reading from broadcast reset address
    // This causes all controllers to return to default state and release their I2C addresses
    uint8_t dummy_buffer[2];
    bool reset_result = i2cdevReadReg16(I2C1_DEV, DECKCTRL_I2C_ADDRESS_RESET, 0x0000, 2, dummy_buffer);

    if (reset_result) {
        DECKCTRL_DEBUG_PRINT("Deck controller reset successful\n");

        // Since this is a hardware reset of the controller MCU, wait for it to restart
        vTaskDelay(M2T(10));
    } else {
        DECKCTRL_DEBUG_PRINT("Deck controller reset failed, no deck controller on the line\n");
    }

    return true;
}

/**
 * @brief Discover and enumerate the next DeckCtrl deck
 *
 * This function is called repeatedly by the deck enumeration system until it
 * returns NULL. Each call attempts to discover one deck using the DeckCtrl protocol.
 *
 * Discovery sequence:
 * 1. Check if we've reached the maximum deck limit
 * 2. Put all unconfigured deck controllers in listening mode (broadcast to 0x42)
 * 3. Read the CPU ID from the next unconfigured deck (from 0x43, register 0x1900)
 * 4. Assign a unique I2C address to that deck (write to 0x43, register 0x1800)
 * 5. Read deck information from the newly assigned address (register 0x0000)
 * 6. Validate magic number and populate DeckInfo structure
 *
 * @return Pointer to DeckInfo for discovered deck, or NULL if no more decks
 */
static DeckInfo* deckctrl_getNextDeck(void)
{
    // Check if we've reached the maximum number of decks
    if (deck_count >= sizeof(deck_info) / sizeof(deck_info[0])) {
        DEBUG_PRINT("WARNING: Maximum number of deck-controller decks reached, no more decks can be enumerated\n");
        return NULL;
    }

    DECKCTRL_DEBUG_PRINT("Starting deck detection\n");

    // Put all unconfigured deck controllers in listening mode (broadcast command)
    // Only controllers that haven't been assigned an address will respond
    uint8_t dummy_buffer[2];
    bool listening_result = i2cdevReadReg16(I2C1_DEV, DECKCTRL_I2C_ADDRESS_LISTEN, 0x0000, 2, dummy_buffer);

    if (!listening_result) {
        DECKCTRL_DEBUG_PRINT("Failed to put deck controllers in listening mode, no more deck to setup.\n");
        return NULL;
    }

    DECKCTRL_DEBUG_PRINT("Deck controllers in listening mode\n");

    // Read the CPU ID from the next available deck
    // The unconfigured deck with the lowest ID will be read, the others will observe a bus collision and back-off
    uint8_t cpu_id[12] = {0};
    bool read_result = i2cdevReadReg16(I2C1_DEV, DECKCTRL_I2C_ADDRESS_DEFAULT, 0x1900, sizeof(cpu_id), cpu_id);

    if (!read_result) {
        DECKCTRL_DEBUG_PRINT("Failed to read deck ID data, no more deck to setup?\n");
        return NULL;
    }

    // Assign a unique I2C address to this deck
    int current_deck = deck_count;
    uint8_t deck_address = DECKCTRL_I2C_ADDRESS_START + current_deck;
    i2cdevWriteReg16(I2C1_DEV, DECKCTRL_I2C_ADDRESS_DEFAULT, 0x1800, 1, &deck_address);
    deck_count++;

    if (deck_count > DECKCTRL_I2C_ADDRESS_END) {
        DEBUG_PRINT("Reached maximum number of decks, will not enumerate any further\n");
    }

    // Print ID and new I2C address of the detected deck if debugging is enabled
    DECKCTRL_DEBUG_PRINT("Found deck with ID: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
                cpu_id[0], cpu_id[1], cpu_id[2], cpu_id[3],
                cpu_id[4], cpu_id[5], cpu_id[6], cpu_id[7],
                cpu_id[8], cpu_id[9], cpu_id[10], cpu_id[11]);
    DECKCTRL_DEBUG_PRINT("Assigned new I2C address: 0x%02x\n", deck_address);

    // Read deck information from the newly assigned address at register 0x0000
    // Format: magic(2) + versions(2) + vendor(1) + product(1) + revision(1) + name(15) + production date(3) + reserved (6) + checksum(1)
    uint8_t deck_mem_data[0x20];
    bool read_info_result = i2cdevReadReg16(I2C1_DEV, deck_address, 0x0000, sizeof(deck_mem_data), deck_mem_data);

    if (!read_info_result) {
        DEBUG_PRINT("Failed to read deck information from I2C address 0x%02x\n", deck_address);
        return NULL;
    }

    // Validate magic number (0xBCDC in big-endian format)
    // This confirms the device is a valid DeckCtrl deck
    uint16_t magic = (deck_mem_data[DECKCTRL_MEM_MAGIC_OFFSET_0] << 8) | deck_mem_data[DECKCTRL_MEM_MAGIC_OFFSET_1];
    if (magic != 0xBCDC) {
        DEBUG_PRINT("Invalid magic number 0x%04x, expected 0xBCDC\n", magic);
        return NULL;
    }

    // Validate checksum (sum of all bytes modulo 256 should be 0)
    uint8_t checksum = 0;
    for (size_t i = 0; i < sizeof(deck_mem_data); i++) {
        checksum += deck_mem_data[i];
    }
    if (checksum != 0) {
        DEBUG_PRINT("Invalid checksum 0x%02x, expected 0x00\n", checksum);
        return NULL;
    }

    // Populate DeckInfo structure with data from deck controller
    DeckInfo* info = &(deck_info[current_deck]);
    memset(info, 0, sizeof(*info));

    // Extract VID/PID which will be used to match with deck drivers
    info->vid = deck_mem_data[DECKCTRL_MEM_VENDOR_ID_OFFSET];
    info->pid = deck_mem_data[DECKCTRL_MEM_PRODUCT_ID_OFFSET];

    info->production_year = deck_mem_data[DECKCTRL_MEM_PRODUCTION_YEAR_OFFSET];
    info->production_month = deck_mem_data[DECKCTRL_MEM_PRODUCTION_MONTH_OFFSET];
    info->production_day = deck_mem_data[DECKCTRL_MEM_PRODUCTION_DAY_OFFSET];

    if (info->production_year == 0 || info->production_month == 0 || info->production_day == 0 ||
        info->production_year == 0xFF || info->production_month == 0xFF || info->production_day == 0xFF)
    {
        DEBUG_PRINT("Invalid production date read from deck controller: %02d-%02d-%02d\n",
            info->production_year, info->production_month, info->production_day);
#ifndef CONFIG_ACCEPT_DECKCTRL_DECKS_UNDER_PRODUCTION
        // Allow a deck without a valid production date to be used.
        return NULL;
#endif
    } else {
        DECKCTRL_DEBUG_PRINT("Production date: 20%02d-%02d-%02d\n",
            info->production_year, info->production_month, info->production_day);
    }

    DECKCTRL_DEBUG_PRINT("Read deck info - VID: 0x%02x, PID: 0x%02x\n", info->vid, info->pid);

    DECKCTRL_DEBUG_PRINT("Firmware version: %d.%d, Board revision: %c\n",
                deck_mem_data[DECKCTRL_MEM_MAJOR_VER_OFFSET],
                deck_mem_data[DECKCTRL_MEM_MINOR_VER_OFFSET],
                deck_mem_data[DECKCTRL_MEM_BOARD_REV_OFFSET]);

    // Copy product name string (null-terminated, max 14 chars + null)
    memcpy(product_names[current_deck], &deck_mem_data[DECKCTRL_MEM_PRODUCT_NAME_OFFSET], 15);
    product_names[current_deck][15] = '\0';  // Ensure null termination
    DECKCTRL_DEBUG_PRINT("Product name: %s\n", product_names[current_deck]);

    info->productName = product_names[current_deck];
    info->discoveryBackend = &deckctrlBackend;

    // Set up backend-specific context with the assigned I2C address
    // Deck drivers can access this via info->backendContext to communicate with the deck
    deck_contexts[current_deck].i2cAddress = deck_address;
    info->backendContext = &deck_contexts[current_deck];

    deckctrl_memory_handlers[current_deck] = (MemoryHandlerDef_t) {
        .type = MEM_TYPE_DECKCTRL,
        .getSize = deckctrlMemorySize,
        .read = deckctrlMemoryRead,
        .write = 0, // Memory is read only
        .getSerialNbr = deckctrlMemorySerialNbr,
        .internal_id = current_deck,
    };

    memoryRegisterHandler(&deckctrl_memory_handlers[current_deck]);

    return info;
}

/**
 * DeckCtrl discovery backend definition
 *
 * This backend is automatically registered via the DECK_DISCOVERY_BACKEND macro
 * and will be called during system initialization to discover DeckCtrl decks.
 */
static const DeckDiscoveryBackend_t deckctrlBackend = {
    .name = "deckctrl",
    .init = deckctrl_init,
    .getNextDeck = deckctrl_getNextDeck,
};

// Register the backend using linker sections
DECK_DISCOVERY_BACKEND(deckctrlBackend);