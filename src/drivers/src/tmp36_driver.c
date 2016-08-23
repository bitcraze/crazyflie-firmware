#include <stddef.h>

#include "config.h"
#include "log.h"
#include "tmp36_driver.h"
#include "deck.h"

/* Internal tracking of last measured data. */
static float raw_data = 0;

/**
 * Reads data measurement from a tmp36 sensor via an analog input interface.
 * @param pin      The GPIO pin to use for ADC conversion.
 * @return The distance measurement in millimeters.
 */
float raw_data_read(uint8_t pin)
{
  raw_data = (float) analogRead(pin);

  return raw_data;
}


