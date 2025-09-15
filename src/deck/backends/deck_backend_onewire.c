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
 * deck_backend_onewire.c - OneWire deck discovery backend
 */

#include "deck_discovery.h"
#include "deck.h"
#include "ow.h"
#include "crc32.h"

#define DEBUG_MODULE "DECK_BACKEND_OW"
#include "debug.h"

// Uncomment to enable debug prints for OneWire backend
// #define DEBUG_ONEWIRE_BACKEND

#ifdef DEBUG_ONEWIRE_BACKEND
#define OW_BACKEND_DEBUG(fmt, ...) DEBUG_PRINT(fmt, ## __VA_ARGS__)
#else
#define OW_BACKEND_DEBUG(...)
#endif

// OneWire backend state
static uint8_t totalDecks = 0;
static uint8_t currentDeck = 0;
static DeckInfo deckBuffer; // Buffer for returning deck info

// Dummy driver for decks that do not have a driver implemented
extern const DeckDriver dummyDriver;

static bool owBackendInit(void) {
    OW_BACKEND_DEBUG("Initializing OneWire backend\n");
    currentDeck = 0;
    totalDecks = 0;

    // Always initialize OneWire (needed for memory system)
    owInit();

#ifdef CONFIG_DECK_BACKEND_ONEWIRE
    if (!owScan(&totalDecks)) {
        OW_BACKEND_DEBUG("OneWire scan failed\n");
        return false;
    }

    OW_BACKEND_DEBUG("OneWire found %d deck(s)\n", totalDecks);
#else
    OW_BACKEND_DEBUG("Ignoring all OW decks because backend is disabled.\n");
    totalDecks = 0;
#endif
    return true;
}

bool infoDecode(DeckInfo * info)
{
  uint8_t crcHeader;
  uint8_t crcTlv;

  if (info->header != DECK_INFO_HEADER_ID) {
    DEBUG_PRINT("Memory error: wrong header ID\n");
    return false;
  }

  crcHeader = crc32CalculateBuffer(info->raw, DECK_INFO_HEADER_SIZE);
  if(info->crc != crcHeader) {
    DEBUG_PRINT("Memory error: incorrect header CRC\n");
    return false;
  }

  if(info->raw[DECK_INFO_TLV_VERSION_POS] != DECK_INFO_TLV_VERSION) {
    DEBUG_PRINT("Memory error: incorrect TLV version\n");
    return false;
  }

  crcTlv = crc32CalculateBuffer(&info->raw[DECK_INFO_TLV_VERSION_POS], info->raw[DECK_INFO_TLV_LENGTH_POS]+2);
  if(crcTlv != info->raw[DECK_INFO_TLV_DATA_POS + info->raw[DECK_INFO_TLV_LENGTH_POS]]) {
    DEBUG_PRINT("Memory error: incorrect TLV CRC %x!=%x\n", (unsigned int)crcTlv,
                info->raw[DECK_INFO_TLV_DATA_POS + info->raw[DECK_INFO_TLV_LENGTH_POS]]);
    return false;
  }

  info->tlv.data = &info->raw[DECK_INFO_TLV_DATA_POS];
  info->tlv.length = info->raw[DECK_INFO_TLV_LENGTH_POS];

  return true;
}

/****** Key/value area handling ********/
static int findType(TlvArea *tlv, int type) {
  int pos = 0;

  while (pos < tlv->length) {
    if (tlv->data[pos] == type) {
      return pos;
    } else {
      pos += tlv->data[pos+1]+2;
    }
  }
  return -1;
}

bool deckTlvHasElement(TlvArea *tlv, int type) {
  return findType(tlv, type) >= 0;
}

int deckTlvGetString(TlvArea *tlv, int type, char *string, int length) {
  int pos = findType(tlv, type);
  int strlength = 0;

  if (pos >= 0) {
    strlength = tlv->data[pos+1];

    if (strlength > (length-1)) {
      strlength = length-1;
    }

    memcpy(string, &tlv->data[pos+2], strlength);
    string[strlength] = '\0';

    return strlength;
  } else {
    string[0] = '\0';

    return -1;
  }
}

char* deckTlvGetBuffer(TlvArea *tlv, int type, int *length) {
  int pos = findType(tlv, type);
  if (pos >= 0) {
    *length = tlv->data[pos+1];
    return (char*) &tlv->data[pos+2];
  }

  return NULL;
}

void deckTlvGetTlv(TlvArea *tlv, int type, TlvArea *output) {
  output->length = 0;
  output->data = (uint8_t *)deckTlvGetBuffer(tlv, type, &output->length);
}

static DeckInfo* owBackendGetNextDeck(void) {
    if (currentDeck >= totalDecks) {
        return NULL; // No more decks
    }

#ifndef CONFIG_DECK_BACKEND_ONEWIRE
    // Skip all decks when backend is disabled
    currentDeck = totalDecks; // Move to end
    return NULL;
#endif

    OW_BACKEND_DEBUG("Reading OneWire deck %d\n", currentDeck);

    // Read raw deck info
    if (!owRead(currentDeck, 0, sizeof(deckBuffer.raw), (uint8_t*)&deckBuffer)) {
        OW_BACKEND_DEBUG("Failed to read OneWire deck %d\n", currentDeck);
        currentDeck++;
        return NULL;
    }

    // Store backend reference
    deckBuffer.discoveryBackend = NULL; // Backend reference will be set after copying in enumeration code

    // Decode and validate deck info using shared function
    if (infoDecode(&deckBuffer)) {
        // Extract product name and board revision from TLV and populate generic fields
        static char productNames[DECK_MAX_COUNT][30];
        static char boardRevisions[DECK_MAX_COUNT][10];

        if (deckTlvGetString(&deckBuffer.tlv, DECK_INFO_NAME, productNames[currentDeck], 30) > 0) {
            deckBuffer.productName = productNames[currentDeck];
        } else {
            deckBuffer.productName = NULL;
        }

        if (deckTlvGetString(&deckBuffer.tlv, DECK_INFO_REVISION, boardRevisions[currentDeck], 10) > 0) {
            deckBuffer.boardRevision = boardRevisions[currentDeck];
        } else {
            deckBuffer.boardRevision = NULL;
        }

    } else {
#ifdef CONFIG_DEBUG
        OW_BACKEND_DEBUG("OneWire deck %d has corrupt memory. Using dummy driver in DEBUG mode.\n", currentDeck);
        deckBuffer.driver = &dummyDriver;
#else
        OW_BACKEND_DEBUG("OneWire deck %d has corrupt memory. Skipping.\n", currentDeck);
        currentDeck++;
        return NULL;
#endif
    }

    currentDeck++;
    return &deckBuffer;
}

static const DeckDiscoveryBackend_t owBackend = {
    .name = "onewire",
    .init = owBackendInit,
    .getNextDeck = owBackendGetNextDeck,
};

DECK_DISCOVERY_BACKEND(owBackend);