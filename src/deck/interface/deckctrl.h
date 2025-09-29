/*
 * DeckCtrl backend api for deck initialization and control
 * 
 * These functions are intended to be called from deck drivers to
 * power, initialize and control the bootloader state of decks
 * 
 * copyright (C) 2024 Bitcraze AB
 */
#pragma once

#include "deck.h"

struct deckCtrlContext_s {
    // I2C address assigned to the deck by the DeckCtrl during discovery
    uint8_t i2cAddress;
};
typedef struct deckCtrlContext_s DeckCtrlContext;
