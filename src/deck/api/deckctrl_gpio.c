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

    char buffer[2];
    // Read current direction register (16-bit)
    if (!i2cdevReadReg16(I2C1_DEV, i2c_address, DECKCTRL_GPIO_DIRECTION_REG, 2, buffer)) {
        return false;
    }

    // Convert bytes to 16-bit value (little endian)
    uint16_t direction_reg = buffer[0] | (buffer[1] << 8);

    // Set or clear the bit for this pin
    if (direction != INPUT) {
        direction_reg |= (1 << pin);
    } else {
        direction_reg &= ~(1 << pin);
    }

    DEBUG_PRINT("Setting GPIO pin %d direction to %s (reg=0x%04x)\n", pin, (direction!=INPUT) ? "output" : "input", direction_reg);

    // Convert back to bytes and write
    buffer[0] = direction_reg & 0xFF;
    buffer[1] = (direction_reg >> 8) & 0xFF;
 
    bool result = i2cdevWriteReg16(I2C1_DEV, i2c_address, DECKCTRL_GPIO_DIRECTION_REG, 2, buffer);

    return result;
}

bool deckctrl_gpio_write(DeckInfo* info, DeckCtrlGPIOPin pin, uint32_t value) {
    if (pin >= DECKCTRL_GPIO_PIN_MAX) {
        return false;
    }

    uint8_t i2c_address;
    if (!get_i2c_address(info, &i2c_address)) {
        return false;
    }

    char buffer[2];
    // Read current value register (16-bit)
    if (!i2cdevReadReg16(I2C1_DEV, i2c_address, DECKCTRL_GPIO_VALUE_REG, 2, buffer)) {
        return false;
    }

    // Convert bytes to 16-bit value (little endian)
    uint16_t value_reg = buffer[0] | (buffer[1] << 8);

    // Set or clear the bit for this pin
    if (value != LOW) {
        value_reg |= (1 << pin);
    } else {
        value_reg &= ~(1 << pin);
    }

    // Convert back to bytes and write
    buffer[0] = value_reg & 0xFF;
    buffer[1] = (value_reg >> 8) & 0xFF;

    return i2cdevWriteReg16(I2C1_DEV, i2c_address, DECKCTRL_GPIO_VALUE_REG, 2, buffer);
}

bool deckctrl_gpio_read(DeckInfo* info, DeckCtrlGPIOPin pin, uint32_t * value) {
    if (pin >= DECKCTRL_GPIO_PIN_MAX || value == NULL) {
        return false;
    }

    uint8_t i2c_address;
    if (!get_i2c_address(info, &i2c_address)) {
        return false;
    }

    char buffer[2];
    // Read current value register (16-bit)
    if (!i2cdevReadReg16(I2C1_DEV, i2c_address, DECKCTRL_GPIO_VALUE_REG, 2, buffer)) {
        return false;
    }

    // Convert bytes to 16-bit value (little endian)
    uint16_t value_reg = buffer[0] | (buffer[1] << 8);

    // Extract the bit for this pin
    *value = ((value_reg & (1 << pin)) != 0)?HIGH:LOW;

    return true;
}
