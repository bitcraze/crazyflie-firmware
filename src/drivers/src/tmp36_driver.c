#include <stddef.h>

#include "config.h"
#include "log.h"
#include "tmp36_driver.h"
#include "deck.h"

/* Internal tracking of last measured data. */
static float temp_data = 0;

/**
 * Reads data measurement from a tmp36 sensor via an analog input interface.
 * @param pin      The GPIO pin to use for ADC conversion.
 * @return The distance measurement in millimeters.
 */
float temp_data_read(uint8_t pin)
{
  /*
   * analogRead() returns a 12-bit (0-4095) value scaled to the range between GND (0V) and VREF.
   * The voltage conversion is: V = analogRead() / 4096 * VREF)
   *
   * The tmp36 sensor returns a voltage between GND and VREF, but scaled to VREF / 512 (volts-per-inch).
   * Inches-per-volt is therefore expressed by (512 / VREF).
   *
   * The distance conversion is:             D = (512 / VREF) * V
   * Expanding V, we get:                    D = (512 / VREF) * (analogRead() / 4096 * VREF)
   * Which can be simplified to:             D = analogRead() / 8
   * Last, we convert inches to millimeters: D = 25.4 * analogRead() / 8
   * Which can be written as:                D = IN2MM(analogRead()) / 8
   *   (to retain the sample's LSB)
   *
   * The above conversion assumes the ADC VREF is the same as the LV-MaxSonar-EZ4 VREF. This means
   * that the tmp36 Sensor must have its VCC pin connected to the VCC pin on the deck port.
   */

  temp_data = (float) analogRead(pin);

  return temp_data;
}
