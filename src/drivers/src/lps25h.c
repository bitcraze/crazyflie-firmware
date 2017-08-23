/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011 Fabio Varesano <fvaresano@yahoo.it>
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
 * @file lps25h.c
 * Driver for the lps25h pressure sensor from ST.
 *
 */
#define DEBUG_MODULE "LPS25H"

#include "FreeRTOS.h"
#include "task.h"

#include "lps25h.h"
#include "i2cdev.h"
#include "debug.h"
#include "eprintf.h"

static uint8_t devAddr;
static I2C_Dev *I2Cx;
static bool isInit;

bool lps25hInit(I2C_Dev *i2cPort)
{
  if (isInit)
    return true;

  I2Cx = i2cPort;
  devAddr = LPS25H_I2C_ADDR;

  vTaskDelay(M2T(5));

  isInit = true;

  return true;
}

bool lps25hTestConnection(void)
{
  uint8_t id;
  bool status;

  if (!isInit)
	return false;


  status = i2cdevReadReg8(I2Cx, devAddr, LPS25H_WHO_AM_I, 1, &id);

  if (status == true && id == LPS25H_WAI_ID)
	  return true;

  return false;
}

bool lps25hSelfTest(void)
{
  float pressure;
  float temperature;
  float asl;

  if (!isInit)
    return false;

  lps25hGetData(&pressure, &temperature, &asl);

  if (lps25hEvaluateSelfTest(LPS25H_ST_PRESS_MIN, LPS25H_ST_PRESS_MAX, pressure, "pressure") &&
      lps25hEvaluateSelfTest(LPS25H_ST_TEMP_MIN, LPS25H_ST_TEMP_MAX, temperature, "temperature"))
  {
    return true;
  }
  else
  {
   return false;
  }
}

bool lps25hEvaluateSelfTest(float min, float max, float value, char* string)
{
  if (value < min || value > max)
  {
    DEBUG_PRINT("Self test %s [FAIL]. low: %0.2f, high: %0.2f, measured: %0.2f\n",
                string, (double)min, (double)max, (double)value);
    return false;
  }
  return true;
}

bool lps25hSetEnabled(bool enable)
{
  uint8_t enable_mask;
  bool status;

	if (!isInit)
	  return false;

	if (enable)
	{
	  enable_mask = 0b11000110; // Power on, 25Hz, BDU, reset zero
	  status = i2cdevWriteReg8(I2Cx, devAddr, LPS25H_CTRL_REG1, 1, &enable_mask);
	  enable_mask = 0b00001111; // AVG-P 512, AVG-T 64
	  status = i2cdevWriteReg8(I2Cx, devAddr, LPS25H_RES_CONF, 1, &enable_mask);
	  // FIFO averaging. This requres temp reg to be read in different read as reg auto inc
	  // wraps back to LPS25H_PRESS_OUT_L after LPS25H_PRESS_OUT_H is read.
	  enable_mask = 0b11000011; // FIFO Mean mode, 4 moving average
	  status = i2cdevWriteReg8(I2Cx, devAddr, LPS25H_FIFO_CTRL, 1, &enable_mask);
	  enable_mask = 0b01000000; // FIFO Enable
	  status = i2cdevWriteReg8(I2Cx, devAddr, LPS25H_CTRL_REG2, 1, &enable_mask);
	}
	else
	{
	  enable_mask = 0x00; // Power off and default values
	  status = i2cdevWriteReg8(I2Cx, devAddr, LPS25H_CTRL_REG1, 1, &enable_mask);
	}

	return status;
}

bool lps25hGetData(float* pressure, float* temperature, float* asl)
{
  uint8_t data[5];
  uint32_t rawPressure;
  int16_t rawTemp;
  bool status;

  status =  i2cdevReadReg8(I2Cx, devAddr, LPS25H_PRESS_OUT_XL | LPS25H_ADDR_AUTO_INC, 3, data);
  // If LPS25H moving avg filter is activated the temp must be read out in separate read.
  status &= i2cdevReadReg8(I2Cx, devAddr, LPS25H_TEMP_OUT_L | LPS25H_ADDR_AUTO_INC, 2, &data[3]);

  rawPressure = ((uint32_t)data[2] << 16) | ((uint32_t)data[1] << 8) | data[0];
  *pressure = (float)rawPressure / LPS25H_LSB_PER_MBAR;

  rawTemp = ((int16_t)data[4] << 8) | data[3];
  *temperature = LPS25H_TEMP_OFFSET + ((float)rawTemp / LPS25H_LSB_PER_CELSIUS);

  *asl = lps25hPressureToAltitude(pressure);

  return status;
}

#include "math.h"
// Constants used to determine altitude from pressure
#define CONST_SEA_PRESSURE 102610.f //1026.1f //http://www.meteo.physik.uni-muenchen.de/dokuwiki/doku.php?id=wetter:stadt:messung
#define CONST_PF 0.1902630958f //(1/5.25588f) Pressure factor
#define CONST_PF2 44330.0f
#define FIX_TEMP 25         // Fixed Temperature. ASL is a function of pressure and temperature, but as the temperature changes so much (blow a little towards the flie and watch it drop 5 degrees) it corrupts the ASL estimates.
                            // TLDR: Adjusting for temp changes does more harm than good.

//TODO: pretty expensive function. Rather smooth the pressure estimates and only call this when needed
/**
 * Converts pressure to altitude above sea level (ASL) in meters
 */
float lps25hPressureToAltitude(float* pressure/*, float* ground_pressure, float* ground_temp*/)
{
    if (*pressure > 0)
    {
        //return (1.f - pow(*pressure / CONST_SEA_PRESSURE, CONST_PF)) * CONST_PF2;
        //return ((pow((1015.7 / *pressure), CONST_PF) - 1.0) * (25. + 273.15)) / 0.0065;
        return ((powf((1015.7f / *pressure), CONST_PF) - 1.0f) * (FIX_TEMP + 273.15f)) / 0.0065f;
    }
    else
    {
        return 0;
    }
}
