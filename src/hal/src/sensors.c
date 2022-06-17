/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2018 Bitcraze AB
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
 * sensors.c - Abstraction layer for sensors on a platform. It acts as a
 * proxy to use the correct sensor based on device type.
 */

#define DEBUG_MODULE "SENSORS"

#include "sensors.h"
#include "platform.h"
#include "debug.h"

#include "autoconf.h"

// https://gcc.gnu.org/onlinedocs/cpp/Stringizing.html
#define xstr(s) str(s)
#define str(s) #s

#if defined(CONFIG_SENSORS_BMI088_BMP388) || defined(CONFIG_SENSORS_BMI088_SPI)
  #include "sensors_bmi088_bmp388.h"
#endif

#ifdef CONFIG_SENSORS_MPU9250_LPS25H
  #include "sensors_mpu9250_lps25h.h"
#endif

#ifdef CONFIG_SENSORS_BOSCH
  #include "sensors_bosch.h"
#endif


typedef struct {
  SensorImplementation_t implements;
  void (*init)(void);
  bool (*test)(void);
  bool (*areCalibrated)(void);
  bool (*manufacturingTest)(void);
  void (*acquire)(sensorData_t *sensors, const uint32_t tick);
  void (*waitDataReady)(void);
  bool (*readGyro)(Axis3f *gyro);
  bool (*readAcc)(Axis3f *acc);
  bool (*readMag)(Axis3f *mag);
  bool (*readBaro)(baro_t *baro);
  void (*setAccMode)(accModes accMode);
  void (*dataAvailableCallback)(void);
} sensorsImplementation_t;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
static void nullFunction(void) {}
#pragma GCC diagnostic pop

static const sensorsImplementation_t sensorImplementations[SensorImplementation_COUNT] = {
#ifdef CONFIG_SENSORS_BMI088_BMP388
  {
    .implements = SensorImplementation_bmi088_bmp388,
    .init = sensorsBmi088Bmp388Init_I2C,
    .test = sensorsBmi088Bmp388Test,
    .areCalibrated = sensorsBmi088Bmp388AreCalibrated,
    .manufacturingTest = sensorsBmi088Bmp388ManufacturingTest,
    .acquire = sensorsBmi088Bmp388Acquire,
    .waitDataReady = sensorsBmi088Bmp388WaitDataReady,
    .readGyro = sensorsBmi088Bmp388ReadGyro,
    .readAcc = sensorsBmi088Bmp388ReadAcc,
    .readMag = sensorsBmi088Bmp388ReadMag,
    .readBaro = sensorsBmi088Bmp388ReadBaro,
    .setAccMode = sensorsBmi088Bmp388SetAccMode,
    .dataAvailableCallback = sensorsBmi088Bmp388DataAvailableCallback,
  },
#endif
#ifdef CONFIG_SENSORS_BMI088_SPI
  {
    .implements = SensorImplementation_bmi088_spi_bmp388,
    .init = sensorsBmi088Bmp388Init_SPI,
    .test = sensorsBmi088Bmp388Test,
    .areCalibrated = sensorsBmi088Bmp388AreCalibrated,
    .manufacturingTest = sensorsBmi088Bmp388ManufacturingTest,
    .acquire = sensorsBmi088Bmp388Acquire,
    .waitDataReady = sensorsBmi088Bmp388WaitDataReady,
    .readGyro = sensorsBmi088Bmp388ReadGyro,
    .readAcc = sensorsBmi088Bmp388ReadAcc,
    .readMag = sensorsBmi088Bmp388ReadMag,
    .readBaro = sensorsBmi088Bmp388ReadBaro,
    .setAccMode = sensorsBmi088Bmp388SetAccMode,
    .dataAvailableCallback = sensorsBmi088Bmp388DataAvailableCallback,
  },
#endif
#ifdef CONFIG_SENSORS_MPU9250_LPS25H
  {
    .implements = SensorImplementation_mpu9250_lps25h,
    .init = sensorsMpu9250Lps25hInit,
    .test = sensorsMpu9250Lps25hTest,
    .areCalibrated = sensorsMpu9250Lps25hAreCalibrated,
    .manufacturingTest = sensorsMpu9250Lps25hManufacturingTest,
    .acquire = sensorsMpu9250Lps25hAcquire,
    .waitDataReady = sensorsMpu9250Lps25hWaitDataReady,
    .readGyro = sensorsMpu9250Lps25hReadGyro,
    .readAcc = sensorsMpu9250Lps25hReadAcc,
    .readMag = sensorsMpu9250Lps25hReadMag,
    .readBaro = sensorsMpu9250Lps25hReadBaro,
    .setAccMode = sensorsMpu9250Lps25hSetAccMode,
    .dataAvailableCallback = nullFunction,
  },
#endif
#ifdef CONFIG_SENSORS_BOSCH
  {
    .implements = SensorImplementation_bosch,
    .init = sensorsBoschInit,
    .test = sensorsBoschTest,
    .areCalibrated = sensorsBoschAreCalibrated,
    .manufacturingTest = sensorsBoschManufacturingTest,
    .acquire = sensorsBoschAcquire,
    .waitDataReady = sensorsBoschWaitDataReady,
    .readGyro = sensorsBoschReadGyro,
    .readAcc = sensorsBoschReadAcc,
    .readMag = sensorsBoschReadMag,
    .readBaro = sensorsBoschReadBaro,
    .setAccMode = sensorsBoschSetAccMode,
    .dataAvailableCallback = nullFunction,
  },
#endif
};

static const sensorsImplementation_t* activeImplementation;
static bool isInit = false;
static const sensorsImplementation_t* findImplementation(SensorImplementation_t implementation);

void sensorsInit(void) {
  if (isInit) {
    return;
  }

#ifndef SENSORS_FORCE
  SensorImplementation_t sensorImplementation = platformConfigGetSensorImplementation();
#else
  SensorImplementation_t sensorImplementation = SENSORS_FORCE;
  DEBUG_PRINT("Forcing sensors to " xstr(SENSORS_FORCE) "\n");
#endif

  activeImplementation = findImplementation(sensorImplementation);

  activeImplementation->init();

  isInit = true;
}

bool sensorsTest(void) {
  return activeImplementation->test();
}

bool sensorsAreCalibrated(void) {
  return activeImplementation->areCalibrated();
}

bool sensorsManufacturingTest(void){
  return activeImplementation->manufacturingTest;
}

void sensorsAcquire(sensorData_t *sensors, const uint32_t tick) {
  activeImplementation->acquire(sensors, tick);
}

void sensorsWaitDataReady(void) {
  activeImplementation->waitDataReady();
}

bool sensorsReadGyro(Axis3f *gyro) {
  return activeImplementation->readGyro(gyro);
}

bool sensorsReadAcc(Axis3f *acc) {
  return activeImplementation->readAcc(acc);
}

bool sensorsReadMag(Axis3f *mag) {
  return activeImplementation->readMag(mag);
}

bool sensorsReadBaro(baro_t *baro) {
  return activeImplementation->readBaro(baro);
}

void sensorsSetAccMode(accModes accMode) {
  activeImplementation->setAccMode(accMode);
}

void sensorsSuspend()
{
  NVIC_DisableIRQ(EXTI15_10_IRQn);
}

void sensorsResume()
{
  NVIC_EnableIRQ(EXTI15_10_IRQn);

}

void __attribute__((used)) EXTI14_Callback(void) {
  activeImplementation->dataAvailableCallback();
}

static const sensorsImplementation_t* findImplementation(SensorImplementation_t implementation) {
  const sensorsImplementation_t* result = 0;

  for (int i = 0; i < SensorImplementation_COUNT; i++) {
    if (sensorImplementations[i].implements == implementation) {
      result = &sensorImplementations[i];
      break;
    }
  }

  return result;
}
