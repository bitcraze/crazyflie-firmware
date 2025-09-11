/**
 * @file
 *  Backend for deck detection using the deck controller.
 *
 * This backends detects decks on the I2C bus using the deck controller.
 */

#include <string.h>
#include <stdlib.h>

#include "deck_discovery.h"
#include "deck.h"
#include "autoconf.h"
#include "i2cdev.h"


#define DEBUG_MODULE "DECKCTRL"
#include "debug.h"

static const DeckDiscoveryBackend_t deckctrlBackend;

// Addresses of the deck controller on the I2C bus
#define DECKCTRL_I2C_ADDRESS_RESET     0x41
#define DECKCTRL_I2C_ADDRESS_LISTEN    0x42
#define DECKCTRL_I2C_ADDRESS_DEFAULT   0x43

// Address range for assigning I2C addresses to deck controllers
// This puts a limit of 12 decks on the bus
#define DECKCTRL_I2C_ADDRESS_START    0x44
#define DECKCTRL_I2C_ADDRESS_END      0x4F

// Deck controller memory layout offsets in read buffer
#define DECKCTRL_MEM_MAGIC_OFFSET_0   0    // Magic number byte 0 (high byte)
#define DECKCTRL_MEM_MAGIC_OFFSET_1   1    // Magic number byte 1 (low byte)
#define DECKCTRL_MEM_MAJOR_VER_OFFSET 2    // Major version offset
#define DECKCTRL_MEM_MINOR_VER_OFFSET 3    // Minor version offset
#define DECKCTRL_MEM_VENDOR_ID_OFFSET 4    // Vendor ID offset
#define DECKCTRL_MEM_PRODUCT_ID_OFFSET 5   // Product ID offset
#define DECKCTRL_MEM_BOARD_REV_OFFSET 6    // Board revision offset
#define DECKCTRL_MEM_PRODUCT_NAME_OFFSET 7 // Product name start offset

// All possible deck info
static DeckInfo deck_info[DECKCTRL_I2C_ADDRESS_END - DECKCTRL_I2C_ADDRESS_START + 1];
static char product_names[DECKCTRL_I2C_ADDRESS_END - DECKCTRL_I2C_ADDRESS_START + 1][16];
static int deck_count = 0;

static bool deckctrl_init(void)
{
    DEBUG_PRINT("Initializing DECKCTRL backend\n");
    
    // Reset all deck controllers on the bus
    uint8_t dummy_buffer[2];
    bool reset_result = i2cdevReadReg16(I2C1_DEV, 0x41, 0x0000, 2, dummy_buffer);

    // Since this is a hardware reset of the controller MCU, wait a bit to allow the MCU to restart
    vTaskDelay(10);
    
    if (reset_result) {
        DEBUG_PRINT("Deck controller reset successful\n");
    } else {
        DEBUG_PRINT("Deck controller reset failed\n");
    }
    
    return true;
}

static DeckInfo* deckctrl_getNextDeck(void)
{
    if (deck_count >= sizeof(deck_info) / sizeof(deck_info[0])) {
        DEBUG_PRINT("DECKCTRL: Maximum number of decks reached, no more decks can be enumerated\n");
        return NULL;
    }

    DEBUG_PRINT("DECKCTRL: Starting deck detection\n");
    
    // Put all unconfigured deck controllers in listening mode
    uint8_t dummy_buffer[2];
    bool listening_result = i2cdevReadReg16(I2C1_DEV, 0x42, 0x0000, 2, dummy_buffer);
    
    if (!listening_result) {
        DEBUG_PRINT("DECKCTRL: Failed to put deck controllers in listening mode, no more deck to setup.\n");
        return NULL;
    }
    
    DEBUG_PRINT("DECKCTRL: Deck controllers in listening mode\n");
    
    // Read the next available ID
    uint8_t cpu_id[12] = {0};
    bool read_result = i2cdevReadReg16(I2C1_DEV, 0x43, 0x1900, sizeof(cpu_id), cpu_id);
    
    if (!read_result) {
        DEBUG_PRINT("DECKCTRL: Failed to read deck ID data, no more deck to setup?\n");
        return NULL;
    }

    int current_deck = deck_count;
    uint8_t deck_address = DECKCTRL_I2C_ADDRESS_START + current_deck;
    i2cdevWriteReg16(I2C1_DEV, 0x43, 0x1800, 1, &deck_address);
    deck_count++;
    if (deck_count > DECKCTRL_I2C_ADDRESS_END) {
        DEBUG_PRINT("DECKCTRL: Reached maximum number of decks, will not enumerate any further\n");
    }

    // Print ID and new I2C address of the detected deck
    DEBUG_PRINT("DECKCTRL: Found deck with ID: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
                cpu_id[0], cpu_id[1], cpu_id[2], cpu_id[3], 
                cpu_id[4], cpu_id[5], cpu_id[6], cpu_id[7],
                cpu_id[8], cpu_id[9], cpu_id[10], cpu_id[11]);
    DEBUG_PRINT("DECKCTRL: Assigned new I2C address: %02x\n", deck_address);

    // Read deck information from memory address 0x0000
    uint8_t deck_mem_data[21];  // Read magic(2) + versions(2) + vendor(1) + product(1) + revision(1) + name(15)
    bool read_info_result = i2cdevReadReg16(I2C1_DEV, deck_address, 0x0000, sizeof(deck_mem_data), deck_mem_data);
    
    if (!read_info_result) {
        DEBUG_PRINT("DECKCTRL: Failed to read deck information from I2C address %02x\n", deck_address);
        return NULL;
    }

    // Check magic number (0xBCDC) - big endian format
    uint16_t magic = (deck_mem_data[DECKCTRL_MEM_MAGIC_OFFSET_0] << 8) | deck_mem_data[DECKCTRL_MEM_MAGIC_OFFSET_1];
    if (magic != 0xBCDC) {
        DEBUG_PRINT("DECKCTRL: Invalid magic number %04x, expected 0xBCDC\n", magic);
        return NULL;
    }

    // return NULL;

    // Fill DeckInfo structure with matching fields
    DeckInfo* info = &(deck_info[current_deck]);
    memset(info, 0, sizeof(*info));

    // Fill vid and pid from deck controller memory
    info->vid = deck_mem_data[DECKCTRL_MEM_VENDOR_ID_OFFSET];   // Vendor ID
    info->pid = deck_mem_data[DECKCTRL_MEM_PRODUCT_ID_OFFSET];  // Product ID
    
    DEBUG_PRINT("DECKCTRL: Read deck info - VID: 0x%02x, PID: 0x%02x\n", info->vid, info->pid);\

    DEBUG_PRINT("DECKCTRL: Firmware version: %d.%d, Board revision: %c\n", 
                deck_mem_data[DECKCTRL_MEM_MAJOR_VER_OFFSET], 
                deck_mem_data[DECKCTRL_MEM_MINOR_VER_OFFSET], 
                deck_mem_data[DECKCTRL_MEM_BOARD_REV_OFFSET]);
    
    // Print product name (15 bytes starting at product name offset)
    memcpy(product_names[current_deck], &deck_mem_data[DECKCTRL_MEM_PRODUCT_NAME_OFFSET], 15);
    DEBUG_PRINT("DECKCTRL: Product name: %s\n", product_names[current_deck]);

    // TODO
    // info->productName = product_names[current_deck];

    info->discoveryBackend = &deckctrlBackend;
    // TODO
    // info.deckctrl.i2cAddress = deck_address;

    // TODO
    // return info;
    return NULL;
}

static const DeckDiscoveryBackend_t deckctrlBackend = {
    .name = "deckctrl",
    .init = deckctrl_init,
    .getNextDeck = deckctrl_getNextDeck,
};

DECK_DISCOVERY_BACKEND(deckctrlBackend);