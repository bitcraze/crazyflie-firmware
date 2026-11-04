/*
 * DeckCtrl GPIO backend for deck initialization and control
 * 
 * These functions are intended to be called from deck drivers to
 * power, initialize and control the bootloader state of decks
 * 
 * copyright (C) 2025 Bitcraze AB
 */

#include <deck.h>
#include <string.h>

#include "deckctrl_gpio.h"
#include "deckctrl.h"
#include "i2cdev.h"

#include "FreeRTOS.h"
#include "semphr.h"

#define DEBUG_MODULE "DECKCTRL_GPIO"
#include "debug.h"


static bool get_i2c_address(DeckInfo* info, uint8_t* address) {
    if (info->backendContext == NULL || strcmp(info->discoveryBackend->name, "deckctrl") != 0) {
        return false;
    }
    *address = ((DeckCtrlContext*)info->backendContext)->i2cAddress;
    return true;
}

bool deckctrl_gpio_set_direction(DeckInfo* info, DeckCtrlGPIOPin pin, uint32_t direction) {
    if (pin >= DECKCTRL_GPIO_PIN_MAX) {
        return false;
    }

    uint8_t i2c_address;
    if (!get_i2c_address(info, &i2c_address)) {
        return false;
    }

    const uint16_t register_address = DECKCTRL_GPIO_DIRECTION_REG + (pin / 8);
    const uint8_t bit = 1U << (pin % 8);
    uint8_t register_value;

    if (!i2cdevReadReg16(I2C1_DEV, i2c_address, register_address, 1, &register_value)) {
        return false;
    }

    if (direction != INPUT) {
        register_value |= bit;
    } else {
        register_value &= ~bit;
    }

    DEBUG_PRINT("Setting GPIO pin %d direction to %s (byte=0x%02x)\n", pin, (direction != INPUT) ? "output" : "input", register_value);

    return i2cdevWriteReg16(I2C1_DEV, i2c_address, register_address, 1, &register_value);
}

bool deckctrl_gpio_write(DeckInfo* info, DeckCtrlGPIOPin pin, uint32_t value) {
    if (pin >= DECKCTRL_GPIO_PIN_MAX) {
        return false;
    }

    uint8_t i2c_address;
    if (!get_i2c_address(info, &i2c_address)) {
        return false;
    }

    const uint16_t register_address = DECKCTRL_GPIO_VALUE_REG + (pin / 8);
    const uint8_t bit = 1U << (pin % 8);
    uint8_t register_value;

    if (!i2cdevReadReg16(I2C1_DEV, i2c_address, register_address, 1, &register_value)) {
        return false;
    }

    if (value != LOW) {
        register_value |= bit;
    } else {
        register_value &= ~bit;
    }

    return i2cdevWriteReg16(I2C1_DEV, i2c_address, register_address, 1, &register_value);
}

bool deckctrl_gpio_read(DeckInfo* info, DeckCtrlGPIOPin pin, uint32_t * value) {
    if (pin >= DECKCTRL_GPIO_PIN_MAX || value == NULL) {
        return false;
    }

    uint8_t i2c_address;
    if (!get_i2c_address(info, &i2c_address)) {
        return false;
    }

    const uint16_t register_address = DECKCTRL_GPIO_VALUE_REG + (pin / 8);
    const uint8_t bit = 1U << (pin % 8);
    uint8_t register_value;

    if (!i2cdevReadReg16(I2C1_DEV, i2c_address, register_address, 1, &register_value)) {
        return false;
    }

    *value = ((register_value & bit) != 0) ? HIGH : LOW;

    return true;
}
