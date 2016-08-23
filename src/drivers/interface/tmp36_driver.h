#ifndef __tmp36driver_H__
#define __tmp36driver_H__

#include <stdint.h>

/**
 * \def tmp36_driver_ENABLED
 * Enable the tmp36 driver (used by the tmp36 measurement subsystem).
 */
#define tmp36_driver_ENABLED

/**
 * \def tmp36_DECK_GPIO
 * The GPIO pin to use if reading via the analog interface of a tmp36 sensor.
 */
#define tmp36_GPIO DECK_GPIO_TX2

float raw_data_read(uint8_t pin);

#endif
