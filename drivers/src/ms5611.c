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
 * @file ms5611.c
 * Driver for the ms5611 pressure sensor from measurement specialties.
 * Datasheet at http://www.meas-spec.com/downloads/MS5611-01BA03.pdf
 *
 */
#define DEBUG_MODULE "MS5611"

#include "FreeRTOS.h"
#include "task.h"

#include "ms5611.h"
#include "i2cdev.h"
#include "debug.h"
#include "eprintf.h"

#define EXTRA_PRECISION      5 // trick to add more precision to the pressure and temp readings
#define CONVERSION_TIME_MS   100 // conversion time in milliseconds

typedef struct
{
  uint16_t psens;
  uint16_t off;
  uint16_t tcs;
  uint16_t tco;
  uint16_t tref;
  uint16_t tsens;
} CalReg;

static uint8_t devAddr;
static I2C_TypeDef *I2Cx;
static bool isInit;

static CalReg   calReg;
static uint32_t lastPresConv;
static uint32_t lastTempConv;
static int32_t  tempCache;

bool ms5611Init(I2C_TypeDef *i2cPort)
{
  if (isInit)
    return TRUE;

  I2Cx = i2cPort;
  devAddr = MS5611_ADDR_CSB_LOW;

  ms5611Reset(); // reset the device to populate its internal PROM registers
  vTaskDelay(M2T(5));
  if (ms5611ReadPROM() == FALSE) // reads the PROM into object variables for later use
  {
    return FALSE;
  }

  isInit = TRUE;

  return TRUE;
}

bool ms5611SelfTest(void)
{
  bool testStatus = TRUE;
  int32_t rawPress;
  int32_t rawTemp;
  int32_t deltaT;
  float pressure;
  float temperature;

  if (!isInit)
    return FALSE;

  ms5611StartConversion(MS5611_D1 + MS5611_OSR_4096);
  vTaskDelay(M2T(CONVERSION_TIME_MS));
  rawPress = ms5611GetConversion(MS5611_D1 + MS5611_OSR_4096);

  ms5611StartConversion(MS5611_D2 + MS5611_OSR_4096);
  vTaskDelay(M2T(CONVERSION_TIME_MS));
  rawTemp = ms5611GetConversion(MS5611_D2 + MS5611_OSR_4096);

  deltaT = ms5611CalcDeltaTemp(rawTemp);
  temperature = ms5611CalcTemp(deltaT);
  pressure = ms5611CalcPressure(rawPress, deltaT);

  if (ms5611EvaluateSelfTest(MS5611_ST_PRESS_MIN, MS5611_ST_PRESS_MAX, pressure, "pressure") &&
      ms5611EvaluateSelfTest(MS5611_ST_TEMP_MIN, MS5611_ST_TEMP_MAX, temperature, "temperature"))
  {
    DEBUG_PRINT("Self test [OK].\n");
  }
  else
  {
   testStatus = FALSE;
  }

  return testStatus;
}

bool ms5611EvaluateSelfTest(float min, float max, float value, char* string)
{
  if (value < min || value > max)
  {
    DEBUG_PRINT("Self test %s [FAIL]. low: %0.2f, high: %0.2f, measured: %0.2f\n",
                string, min, max, value);
    return FALSE;
  }
  return TRUE;
}

float ms5611GetPressure(uint8_t osr)
{
  // see datasheet page 7 for formulas
  int32_t rawPress = ms5611RawPressure(osr);
  int64_t dT = (int64_t)ms5611GetDeltaTemp(osr);
  if (dT == 0)
  {
    return 0;
  }
  int64_t off = (((int64_t)calReg.off) << 16) + ((calReg.tco * dT) >> 7);
  int64_t sens = (((int64_t)calReg.psens) << 15) + ((calReg.tcs * dT) >> 8);
  if (rawPress != 0)
  {
    return ((((rawPress * sens) >> 21) - off) >> (15 - EXTRA_PRECISION))
        / ((1 << EXTRA_PRECISION) * 100.0);
  }
  else
  {
    return 0;
  }
}

float ms5611CalcPressure(int32_t rawPress, int32_t dT)
{
  int64_t off;
  int64_t sens;

  if (rawPress == 0 || dT == 0)
  {
    return 0;
  }

  off = (((int64_t)calReg.off) << 16) + ((calReg.tco * (int64_t)dT) >> 7);
  sens = (((int64_t)calReg.psens) << 15) + ((calReg.tcs * (int64_t)dT) >> 8);

  return ((((rawPress * sens) >> 21) - off) >> (15 - EXTRA_PRECISION))
          / ((1 << EXTRA_PRECISION) * 100.0);
}

float ms5611GetTemperature(uint8_t osr)
{
  // see datasheet page 7 for formulas
  int32_t dT;

  dT = ms5611GetDeltaTemp(osr);
  if (dT != 0)
  {
    return ms5611CalcTemp(dT);
  }
  else
  {
    return 0;
  }
}

int32_t ms5611GetDeltaTemp(uint8_t osr)
{
  int32_t rawTemp = ms5611RawTemperature(osr);
  if (rawTemp != 0)
  {
    return ms5611CalcDeltaTemp(rawTemp);
  }
  else
  {
    return 0;
  }
}

float ms5611CalcTemp(int32_t deltaT)
{
  if (deltaT == 0)
  {
    return 0;
  }
  else
  {
    return (float)(((1 << EXTRA_PRECISION) * 2000)
            + (((int64_t)deltaT * calReg.tsens) >> (23 - EXTRA_PRECISION)))
            / ((1 << EXTRA_PRECISION)* 100.0);
  }
}

int32_t ms5611CalcDeltaTemp(int32_t rawTemp)
{
  if (rawTemp == 0)
  {
    return 0;
  }
  else
  {
    return rawTemp - (((int32_t)calReg.tref) << 8);
  }
}

int32_t ms5611RawPressure(uint8_t osr)
{
  uint32_t now = xTaskGetTickCount();
  if (lastPresConv != 0 && (now - lastPresConv) >= CONVERSION_TIME_MS)
  {
    lastPresConv = 0;
    return ms5611GetConversion(MS5611_D1 + osr);
  }
  else
  {
    if (lastPresConv == 0 && lastTempConv == 0)
    {
      ms5611StartConversion(MS5611_D1 + osr);
      lastPresConv = now;
    }
    return 0;
  }
}

int32_t ms5611RawTemperature(uint8_t osr)
{
  uint32_t now = xTaskGetTickCount();
  if (lastTempConv != 0 && (now - lastTempConv) >= CONVERSION_TIME_MS)
  {
    lastTempConv = 0;
    tempCache = ms5611GetConversion(MS5611_D2 + osr);
    return tempCache;
  }
  else
  {
    if (lastTempConv == 0 && lastPresConv == 0)
    {
      ms5611StartConversion(MS5611_D2 + osr);
      lastTempConv = now;
    }
    return tempCache;
  }
}

// see page 11 of the datasheet
void ms5611StartConversion(uint8_t command)
{
  // initialize pressure conversion
  i2cdevWriteByte(I2Cx, devAddr, I2CDEV_NO_MEM_ADDR, command);
}

int32_t ms5611GetConversion(uint8_t command)
{
  int32_t conversion = 0;
  uint8_t buffer[MS5611_D1D2_SIZE];

  // start read sequence
  i2cdevWriteByte(I2Cx, devAddr, I2CDEV_NO_MEM_ADDR, 0);
  // Read conversion
  i2cdevRead(I2Cx, devAddr, I2CDEV_NO_MEM_ADDR, MS5611_D1D2_SIZE, buffer);
  conversion = ((int32_t)buffer[0] << 16) |
               ((int32_t)buffer[1] << 8) | buffer[2];

  return conversion;
}

/**
 * Reads factory calibration and store it into object variables.
 */
bool ms5611ReadPROM()
{
  uint8_t buffer[MS5611_PROM_REG_SIZE];
  uint16_t* pCalRegU16 = (uint16_t*)&calReg;
  int32_t i = 0;
  bool status = FALSE;

  for (i = 0; i < MS5611_PROM_REG_COUNT; i++)
  {
    // start read sequence
    status = i2cdevWriteByte(I2Cx, devAddr, I2CDEV_NO_MEM_ADDR,
                             MS5611_PROM_BASE_ADDR + (i * MS5611_PROM_REG_SIZE));
    // Read conversion
    if (status)
    {
      status = i2cdevRead(I2Cx, devAddr, I2CDEV_NO_MEM_ADDR, MS5611_PROM_REG_SIZE, buffer);
      pCalRegU16[i] = ((uint16_t)buffer[0] << 8) | buffer[1];
    }
  }

  return status;
}

/**
 * Send a reset command to the device. With the reset command the device
 * populates its internal registers with the values read from the PROM.
 */
void ms5611Reset()
{
  i2cdevWriteByte(I2Cx, devAddr, I2CDEV_NO_MEM_ADDR, MS5611_RESET);
}
