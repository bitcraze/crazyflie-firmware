/*
 * DeckCtrl GPIO pin definitions, types and functions
 * 
 * copyright (C) 2024 Bitcraze AB
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "deck.h"

/**
 * @brief GPIO pin enumeration for deck control
 * 
 * Defines the available GPIO pins that can be controlled through the
 * deck control interface. Each pin corresponds to a bit position in
 * the GPIO registers (0x1000 for direction, 0x1002 for value).
 */
typedef enum {
    DECKCTRL_GPIO_PIN_0 = 0,   // GPIO pin 0 (bit 0)
    DECKCTRL_GPIO_PIN_1 = 1,   // GPIO pin 1 (bit 1)
    DECKCTRL_GPIO_PIN_2 = 2,   // GPIO pin 2 (bit 2)
    DECKCTRL_GPIO_PIN_3 = 3,   // GPIO pin 3 (bit 3)
    DECKCTRL_GPIO_PIN_4 = 4,   // GPIO pin 4 (bit 4)
    DECKCTRL_GPIO_PIN_5 = 5,   // GPIO pin 5 (bit 5)
    DECKCTRL_GPIO_PIN_6 = 6,   // GPIO pin 6 (bit 6)
    DECKCTRL_GPIO_PIN_7 = 7,   // GPIO pin 7 (bit 7)
    DECKCTRL_GPIO_PIN_8 = 8,   // GPIO pin 8 (bit 8)
    DECKCTRL_GPIO_PIN_9 = 9,   // GPIO pin 9 (bit 9)
    DECKCTRL_GPIO_PIN_10 = 10, // GPIO pin 10 (bit 10)
    DECKCTRL_GPIO_PIN_11 = 11, // GPIO pin 11 (bit 11)
    DECKCTRL_GPIO_PIN_12 = 12, // GPIO pin 12 (bit 12)
    DECKCTRL_GPIO_PIN_13 = 13, // GPIO pin 13 (bit 13)
    DECKCTRL_GPIO_PIN_14 = 14, // GPIO pin 14 (bit 14)
    DECKCTRL_GPIO_PIN_15 = 15, // GPIO pin 15 (bit 15)
    DECKCTRL_GPIO_PIN_MAX = 16
} DeckCtrlGPIOPin;

// GPIO register addresses
#define DECKCTRL_GPIO_DIRECTION_REG  0x1000
#define DECKCTRL_GPIO_VALUE_REG      0x1002

/**
 * @brief Set the direction of a deck control GPIO pin
 * 
 * Configures the specified GPIO pin as either an input or output pin.
 * 
 * @param info Pointer to the DeckInfo structure of the deck
 * @param pin The GPIO pin to configure
 * @param mode Pin mode. `INPUT` for input `OUTPUT` for output.
 * @return True if the direction was set successfully, false otherwise
 * 
 * @note Implementation details: any direction that is not `INPUT` is considered as output.
 */
bool deckctrl_gpio_set_direction(DeckInfo* info, DeckCtrlGPIOPin pin, uint32_t direction);


/**
 * @brief Write a digital value to a specified deck control GPIO pin
 * 
 * This function sets the output state of a deck control GPIO pin to either
 * high (true) or low (false).
 * 
 * @param info Pointer to the DeckInfo structure of the deck
 * @param pin The deck control GPIO pin to write to
 * @param value The digital value to write. `HIGH` or `LOW`
 * 
 * @return true if the write operation was successful, false otherwise
 * 
 * @note Implementation details: any value that is not `LOW` is considered as high.
 */
bool deckctrl_gpio_write(DeckInfo* info, DeckCtrlGPIOPin pin, uint32_t value);


/**
 * @brief Read the current state of a deck control GPIO pin
 * 
 * @param info Pointer to the DeckInfo structure of the deck
 * @param pin The GPIO pin to read from
 * @param value Pointer to store the read digital value, `HIGH` or `LOW`
 * @return true if the read operation was successful, false otherwise
 */
bool deckctrl_gpio_read(DeckInfo* info, DeckCtrlGPIOPin pin, uint32_t* value);
