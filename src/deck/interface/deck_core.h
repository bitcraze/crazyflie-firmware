/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
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
 * deck_core.h - Definitions and interface to handle deck init and drivers
 *               Implementation is in deck.c, deck_info.c and deck_drivers.c
 */

#ifndef __DECK_CODE_H__
#define __DECK_CODE_H__


#include <stdint.h>
#include <stdbool.h>

#include "estimator.h"

/* Maximum number of decks that can be enumerated */
#define DECK_MAX_COUNT 4

/* Main functions to init and test the decks, called during system initialisation */
void deckInit(void);
bool deckTest(void);

/***** Driver TOC definitions ******/

/* Used peripherals */
#define DECK_USING_UART1   (1<<0)
#define DECK_USING_UART2   (1<<1)
#define DECK_USING_SPI     (1<<2)
#define DECK_USING_TIMER3  (1<<3)
#define DECK_USING_TIMER5  (1<<4)
#define DECK_USING_TIMER14 (1<<5)

/* Used GPIO */
#define DECK_USING_PC11 (1<<0)
#define DECK_USING_RX1  (1<<0)
#define DECK_USING_PC10 (1<<1)
#define DECK_USING_TX1  (1<<1)
#define DECK_USING_PB7  (1<<2)
#define DECK_USING_SDA  (1<<2)
#define DECK_USING_PB6  (1<<3)
#define DECK_USING_SCL  (1<<3)
#define DECK_USING_PB8  (1<<4)
#define DECK_USING_IO_1 (1<<4)
#define DECK_USING_PB5  (1<<5)
#define DECK_USING_IO_2 (1<<5)
#define DECK_USING_PB4  (1<<6)
#define DECK_USING_IO_3 (1<<6)
#define DECK_USING_PC12 (1<<7)
#define DECK_USING_IO_4 (1<<7)
#define DECK_USING_PA2  (1<<8)
#define DECK_USING_TX2  (1<<8)
#define DECK_USING_PA3  (1<<9)
#define DECK_USING_RX2  (1<<9)
#define DECK_USING_PA5  (1<<10)
#define DECK_USING_SCK  (1<<10)
#define DECK_USING_PA6  (1<<11)
#define DECK_USING_MISO (1<<11)
#define DECK_USING_PA7  (1<<12)
#define DECK_USING_MOSI (1<<12)

struct deckInfo_s;
struct deckFwUpdate_s;

/* Structure definition and registering macro */
typedef struct deck_driver {
  /* Identification of the deck (written in the board) */
  uint8_t vid;
  uint8_t pid;
  char *name;

  /* Periphreal and Gpio used _dirrectly_ by the driver */
  uint32_t usedPeriph;
  uint32_t usedGpio;

  /* Required system properties */
  StateEstimatorType requiredEstimator;
  bool requiredLowInterferenceRadioMode;

  // Deck memory access definition
  const struct deckMemDef_s* memoryDef;

  /* Init and test functions */
  void (*init)(struct deckInfo_s *);
  bool (*test)(void);
} DeckDriver;

#define DECK_DRIVER(NAME) const struct deck_driver * driver_##NAME __attribute__((section(".deckDriver." #NAME), used)) = &(NAME)

/****** Deck_info *******/

#define DECK_INFO_HEADER_ID 0xeb
#define DECK_INFO_HEADER_SIZE 7
#define DECK_INFO_TLV_VERSION 0
#define DECK_INFO_TLV_VERSION_POS 8
#define DECK_INFO_TLV_LENGTH_POS 9
#define DECK_INFO_TLV_DATA_POS 10

typedef struct {
  uint8_t *data;
  int length;
} TlvArea;

typedef struct deckInfo_s {

  union {
    struct {
      uint8_t header;
      uint32_t usedPins;
      uint8_t vid;
      uint8_t pid;
      uint8_t crc;

      uint8_t rawTlv[104];
    } __attribute__((packed));

    uint8_t raw[112];
  };

  TlvArea tlv;
  const DeckDriver *driver;
} DeckInfo;

/**
 * @brief Definition of function that is called when a block of a new firmware is uploaded to the deck.
 * The upload will be done in small but continouse pieces.
 * @param address: Address where the buffer should be written. The start of the firmware is at address 0.
 * @param len: Buffer length
 * @param buffer: Buffer to write in the firmware memory
 *
 * @return True if the buffer could be written successully, false otherwise (if the deck if not in bootloader
 *         mode for example)
 */
typedef bool (deckMemoryWrite)(const uint32_t vAddr, const uint8_t len, const uint8_t* buffer);

/**
 * @brief Definition of function to read the firmware
 *
 * @param addr: Address where the data should be read. The start of the firmware is at address 0.
 * @param len: Length to read.
 * @param buffer: Buffer where to output the data
 *
 * @return True if the buffer could be read successully, false otherwise (if the deck if not in bootloader
 *         mode for example)
 */
typedef bool (deckMemoryRead)(const uint32_t vAddr, const uint8_t len, uint8_t* buffer);

#define DECK_MEMORY_MASK_STARTED 1
#define DECK_MEMORY_MASK_UPGRADE_REQUIRED 2
#define DECK_MEMORY_MASK_BOOT_LOADER_ACTIVE 4

/**
 * @brief Definition of function to query a deck for properties related to memory
 *
 * @return bitfield using the DECK_MEMORY_MASK_XXX definitions
 */
typedef uint8_t (deckMemoryProperties)();

/**
 * @brief This struct defines the firmware required by the deck and the function
 * to use to flash new firmware to the deck.
 */
typedef struct deckMemDef_s {
  // Functions that will be called to read or write to the deck memory.
  deckMemoryWrite* write;
  deckMemoryRead* read;

  // Function to query properties of the deck memory
  deckMemoryProperties* properties;

  // True if the deck supports FW upgrades
  bool supportsUpgrade;

  // Definition of the required firmware for the deck (if supported)
  uint32_t requiredHash;
  // TOOD krri rename to length?
  uint32_t requiredSize;

} DeckMemDef_t;

int deckCount(void);

DeckInfo * deckInfo(int i);

/* Key/value area handling */
bool deckTlvHasElement(TlvArea *tlv, int type);

int deckTlvGetString(TlvArea *tlv, int type, char *string, int maxLength);

char* deckTlvGetBuffer(TlvArea *tlv, int type, int *length);

void deckTlvGetTlv(TlvArea *tlv, int type, TlvArea *output);

/* Defined Types */
#define DECK_INFO_NAME 1
#define DECK_INFO_REVISION 2
#define DECK_INFO_CUSTOMDATA 3

/***** Drivers introspection API ******/

/* Returns the number of driver registered */
int deckDriverCount();

/* Returns one driver definition */
const struct deck_driver* deckGetDriver(int i);

/* Find driver by pid/vid */
const struct deck_driver* deckFindDriverByVidPid(uint8_t vid, uint8_t pid);

/*find driver by name */
const struct deck_driver* deckFindDriverByName(char* name);

StateEstimatorType deckGetRequiredEstimator();

bool deckGetRequiredLowInterferenceRadioMode();

#endif //__DECK_CODE_H__
