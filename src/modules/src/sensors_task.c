/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
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
 * sensors_task.h - Sensors interface using an interrupt-driven task to reduce CPU load
 *
 * 2016.06.15: Initial version by Mike Hamer, http://mikehamer.info
 */

#ifdef PLATFORM_CF1
#error SENSORS = task is only compatible with the Crazyflie 2.0 // due to the IMU initialization
#endif

#include "sensors.h"

#include <math.h>
#include <stm32f4xx.h>

#include "lps25h.h"
#include "mpu6500.h"
#include "ak8963.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "system.h"
#include "debug.h"
#include "imu.h"
#include "nvicconf.h"

#define IMU_MPU6500_DLPF_256HZ
#define IMU_ENABLE_PRESSURE_LPS25H

// TODO: The magnetometer currently doesn't update properly, this is most likely a slave configuration problem
//#define IMU_ENABLE_MAG_AK8963
#define MAG_GAUSS_PER_LSB     666.7f

#define IMU_GYRO_FS_CFG       MPU6500_GYRO_FS_2000
#define IMU_DEG_PER_LSB_CFG   MPU6500_DEG_PER_LSB_2000

#define IMU_ACCEL_FS_CFG      MPU6500_ACCEL_FS_8
#define IMU_G_PER_LSB_CFG     MPU6500_G_PER_LSB_8

static xQueueHandle accelerometerDataQueue;
static xQueueHandle gyroDataQueue;
static xQueueHandle magnetometerDataQueue;
static xQueueHandle barometerDataQueue;
static xSemaphoreHandle sensorsDataReady;

static bool isInit = false;
static sensorData_t sensors;

#define IMU_SENSOR_BIAS_SAMPLES  1024
#define IMU_GYRO_BIAS_CALCULATE_STDDEV
static Axis3f gyroBias;
static Axis3i64 gyroBiasSampleSum;
static float accScaleSum = 0;
static float accScale = 1;
#ifdef IMU_GYRO_BIAS_CALCULATE_STDDEV
static Axis3f gyroBiasStdDev;
static Axis3i64 gyroBiasSampleSumSquares;
#endif
static uint32_t sensorBiasSampleCount = 0;
static bool sensorBiasFound = false;
static bool isBarometerPresent = false;
static bool isMagnetometerPresent = false;

static uint8_t buffer[25] = {0}; // maximum data length is 25 bytes if both slaves are enabled

static void processAccGyroMeasurements(const uint8_t *buffer);
static void processMagnetometerMeasurements(const uint8_t *buffer);
static void processBarometerMeasurements(const uint8_t *buffer);

bool sensorsReadGyro(Axis3f *gyro) {
  return (pdTRUE == xQueueReceive(gyroDataQueue, gyro, 0));
}

bool sensorsReadAcc(Axis3f *acc) {
  return (pdTRUE == xQueueReceive(accelerometerDataQueue, acc, 0));
}

bool sensorsReadMag(Axis3f *mag) {
  return (pdTRUE == xQueueReceive(magnetometerDataQueue, mag, 0));
}

bool sensorsReadBaro(baro_t *baro)
{
  return (pdTRUE == xQueueReceive(barometerDataQueue, baro, 0));
}

void sensorsAcquire(sensorData_t *sensors, const uint32_t tick)
{
  sensorsReadGyro(&sensors->gyro);
  sensorsReadAcc(&sensors->acc);
  sensorsReadMag(&sensors->mag);
  sensorsReadBaro(&sensors->baro);
}

bool sensorsAreCalibrated() {
  return sensorBiasFound;
}

static void sensorsTask(void *param)
{
  systemWaitStart();

  while (1)
  {
    if (pdTRUE == xSemaphoreTake(sensorsDataReady, portMAX_DELAY))
    {
      // data is ready to be read
      uint8_t dataLen = (uint8_t) (14 + (isMagnetometerPresent ? 6 : 0) + (isBarometerPresent ? 5 : 0));

      i2cdevRead(I2C3_DEV, MPU6500_ADDRESS_AD0_HIGH, MPU6500_RA_ACCEL_XOUT_H, dataLen, buffer);
      // these functions process the respective data and queue it on the output queues
      processAccGyroMeasurements(&(buffer[0]));
      if (isMagnetometerPresent) { processMagnetometerMeasurements(&(buffer[14])); }
      if (isBarometerPresent) { processBarometerMeasurements(&(buffer[isMagnetometerPresent ? 20 : 14])); }

      vTaskSuspendAll(); // ensure all queues are populated at the same time
      xQueueOverwrite(accelerometerDataQueue, &sensors.acc);
      xQueueOverwrite(gyroDataQueue, &sensors.gyro);
      if (isMagnetometerPresent) { xQueueOverwrite(magnetometerDataQueue, &sensors.mag); }
      if (isBarometerPresent) { xQueueOverwrite(barometerDataQueue, &sensors.baro); }
      xTaskResumeAll();
    }
  }
}

void processBarometerMeasurements(const uint8_t *buffer)
{
  static uint32_t rawPressure = 0;
  static int16_t rawTemp = 0;

  rawPressure = ((uint32_t) buffer[2] << 16) | ((uint32_t) buffer[1] << 8) | buffer[0];
  rawTemp = ((int16_t) buffer[4] << 8) | buffer[3];

  sensors.baro.pressure = (float) rawPressure / LPS25H_LSB_PER_MBAR;
  sensors.baro.temperature = LPS25H_TEMP_OFFSET + ((float) rawTemp / LPS25H_LSB_PER_CELSIUS);
  sensors.baro.asl = lps25hPressureToAltitude(&sensors.baro.pressure);
}

void processMagnetometerMeasurements(const uint8_t *buffer)
{
  static int16_t headingx = 0;
  static int16_t headingy = 0;
  static int16_t headingz = 0;

  headingx = (((int16_t) buffer[1]) << 8) | buffer[0];
  headingy = (((int16_t) buffer[3]) << 8) | buffer[2];
  headingz = (((int16_t) buffer[5]) << 8) | buffer[4];

  sensors.mag.x = (float)headingx / MAG_GAUSS_PER_LSB;
  sensors.mag.y = (float)headingy / MAG_GAUSS_PER_LSB;
  sensors.mag.z = (float)headingz / MAG_GAUSS_PER_LSB;
}

void processAccGyroMeasurements(const uint8_t *buffer)
{
  // Note the ordering to correct the rotated 90º IMU coordinate system
  int16_t ay = (((int16_t) buffer[0]) << 8) | buffer[1];
  int16_t ax = (((int16_t) buffer[2]) << 8) | buffer[3];
  int16_t az = (((int16_t) buffer[4]) << 8) | buffer[5];
  int16_t gy = (((int16_t) buffer[8]) << 8) | buffer[9];
  int16_t gx = (((int16_t) buffer[10]) << 8) | buffer[11];
  int16_t gz = (((int16_t) buffer[12]) << 8) | buffer[13];

  if (!sensorBiasFound) { // If the gyro has not yet been calibrated:
    // Add the current sample to the running mean and variance
    gyroBiasSampleSum.x += gx;
    gyroBiasSampleSum.y += gy;
    gyroBiasSampleSum.z += gz;
#ifdef IMU_GYRO_BIAS_CALCULATE_STDDEV
    gyroBiasSampleSumSquares.x += gx * gx;
    gyroBiasSampleSumSquares.y += gy * gy;
    gyroBiasSampleSumSquares.z += gz * gz;
#endif
    accScaleSum += sqrtf(powf(ax * IMU_G_PER_LSB_CFG, 2) + powf(ay * IMU_G_PER_LSB_CFG, 2) + powf(az * IMU_G_PER_LSB_CFG, 2));
    sensorBiasSampleCount += 1;

    // If we then have enough samples, calculate the mean and standard deviation
    if (sensorBiasSampleCount == IMU_SENSOR_BIAS_SAMPLES) {
      gyroBias.x = (float)(gyroBiasSampleSum.x) / IMU_SENSOR_BIAS_SAMPLES;
      gyroBias.y = (float)(gyroBiasSampleSum.y) / IMU_SENSOR_BIAS_SAMPLES;
      gyroBias.z = (float)(gyroBiasSampleSum.z) / IMU_SENSOR_BIAS_SAMPLES;

#ifdef IMU_GYRO_BIAS_CALCULATE_STDDEV
      gyroBiasStdDev.x = sqrtf((float)(gyroBiasSampleSumSquares.x) / IMU_SENSOR_BIAS_SAMPLES - (gyroBias.x * gyroBias.x));
      gyroBiasStdDev.y = sqrtf((float)(gyroBiasSampleSumSquares.y) / IMU_SENSOR_BIAS_SAMPLES - (gyroBias.y * gyroBias.y));
      gyroBiasStdDev.z = sqrtf((float)(gyroBiasSampleSumSquares.z) / IMU_SENSOR_BIAS_SAMPLES - (gyroBias.z * gyroBias.z));
#endif
      accScale = accScaleSum / IMU_SENSOR_BIAS_SAMPLES;
      sensorBiasFound = true;
    }
  }

  sensors.gyro.x = -(gx - gyroBias.x) * IMU_DEG_PER_LSB_CFG;
  sensors.gyro.y =  (gy - gyroBias.y) * IMU_DEG_PER_LSB_CFG;
  sensors.gyro.z =  (gz - gyroBias.z) * IMU_DEG_PER_LSB_CFG;

  sensors.acc.x = -(ax) * IMU_G_PER_LSB_CFG / accScale;
  sensors.acc.y =  (ay) * IMU_G_PER_LSB_CFG / accScale;
  sensors.acc.z =  (az) * IMU_G_PER_LSB_CFG / accScale;
}

static void sensorsDeviceInit(void) {
  isMagnetometerPresent = false;
  isBarometerPresent = false;

  // Wait for sensors to startup
  while (xTaskGetTickCount() < 1000);

  i2cdevInit(I2C3_DEV);
  mpu6500Init(I2C3_DEV);
  if (mpu6500TestConnection() == true)
  {
    DEBUG_PRINT("MPU9250 I2C connection [OK].\n");
  }
  else
  {
    DEBUG_PRINT("MPU9250 I2C connection [FAIL].\n");
  }

  mpu6500Reset();
  vTaskDelay(M2T(50));
  // Activate MPU6500
  mpu6500SetSleepEnabled(false);
  // Enable temp sensor
  mpu6500SetTempSensorEnabled(true);
  // Disable interrupts
  mpu6500SetIntEnabled(false);
  // Connect the HMC5883L to the main I2C bus
  mpu6500SetI2CBypassEnabled(true);
  // Set x-axis gyro as clock source
  mpu6500SetClockSource(MPU6500_CLOCK_PLL_XGYRO);
  // Set gyro full scale range
  mpu6500SetFullScaleGyroRange(IMU_GYRO_FS_CFG);
  // Set accelerometer full scale range
  mpu6500SetFullScaleAccelRange(IMU_ACCEL_FS_CFG);
#ifdef IMU_MPU6500_DLPF_256HZ
  // 256Hz digital low-pass filter only works with little vibrations
  // Set output rate (15): 8000 / (1 + 15) = 500Hz
  mpu6500SetRate(15);
  // Set digital low-pass bandwidth
  mpu6500SetDLPFMode(MPU6500_DLPF_BW_256);
#else
  // To low DLPF bandwidth might cause instability and decrease agility
  // but it works well for handling vibrations and unbalanced propellers
  // Set output rate (1): 1000 / (1 + 1) = 500Hz
  mpu6500SetRate(1);
  // Set digital low-pass bandwidth
  mpu6500SetDLPFMode(MPU6500_DLPF_BW_98);
#endif

  // delay 3 seconds until the quad has stabilized enough to pass the test
  bool mpu6500SelfTestPassed = false;
  for (int i=0; i<300; i++)
  {
    if(mpu6500SelfTest() == true)
    {
      mpu6500SelfTestPassed = true;
      break;
    }
    else
    {
      vTaskDelay(M2T(10));
    }
  }
  configASSERT(mpu6500SelfTestPassed);

  // Now begin to set up the slaves
  mpu6500SetSlave4MasterDelay(4); // read slaves at 100Hz = (500Hz / (1 + 4))

#ifdef IMU_ENABLE_MAG_AK8963
  ak8963Init(I2C3_DEV);
  if (ak8963TestConnection() == true)
  {
    isMagnetometerPresent = true;
    ak8963SetMode(AK8963_MODE_16BIT | AK8963_MODE_CONT2); // 16bit 100Hz
    configASSERT(ak8963SelfTest());
    DEBUG_PRINT("AK8963 I2C connection [OK].\n");
    mpu6500SetSlaveAddress(0, 0x80 | AK8963_ADDRESS_00); // set the magnetometer to Slave 0, enable read
    mpu6500SetSlaveRegister(0, AK8963_RA_HXL); // read the magnetometer heading register
    mpu6500SetSlaveDataLength(0, 6); // read 6 bytes (x, y, z heading)
    mpu6500SetSlaveDelayEnabled(0, true);
    mpu6500SetSlaveEnabled(0, true);
  }
  else
  {
    DEBUG_PRINT("AK8963 I2C connection [FAIL].\n");
  }
#endif

#ifdef IMU_ENABLE_PRESSURE_LPS25H
  lps25hInit(I2C3_DEV);
  if (lps25hTestConnection() == true)
  {
    lps25hSetEnabled(true);
    isBarometerPresent = true;
    configASSERT(lps25hSelfTest());
    DEBUG_PRINT("LPS25H I2C connection [OK].\n");
    mpu6500SetSlaveAddress(1, 0x80 | LPS25H_I2C_ADDR); // set the barometer to Slave 1, enable read
    mpu6500SetSlaveRegister(1, LPS25H_PRESS_OUT_XL | LPS25H_ADDR_AUTO_INC);
    mpu6500SetSlaveDataLength(1, 5);
    mpu6500SetSlaveDelayEnabled(1, true);
    mpu6500SetSlaveEnabled(1, true);
  }
  else
  {
    //TODO: Should sensor test fail hard if no connection
    DEBUG_PRINT("LPS25H I2C connection [FAIL].\n");
  }
#endif

  mpu6500SetI2CBypassEnabled(false);
  mpu6500SetI2CMasterModeEnabled(true);
  mpu6500SetWaitForExternalSensorEnabled(false); // the slave data isn't so important for the state estimation
  mpu6500SetInterruptMode(0); // active high
  mpu6500SetInterruptDrive(0); // push pull
  mpu6500SetInterruptLatch(0); // latched until clear
  mpu6500SetInterruptLatchClear(1); // cleared on any register read
  mpu6500SetIntDataReadyEnabled(true);
}

static void sensorsTaskInit(void) {
  accelerometerDataQueue = xQueueCreate(1, sizeof(Axis3f));
  gyroDataQueue = xQueueCreate(1, sizeof(Axis3f));
  magnetometerDataQueue = xQueueCreate(1, sizeof(Axis3f));
  barometerDataQueue = xQueueCreate(1, sizeof(baro_t));

  xTaskCreate(sensorsTask, SENSORS_TASK_NAME, SENSORS_TASK_STACKSIZE, NULL, SENSORS_TASK_PRI, NULL);
}

static void sensorsInterruptInit(void) {
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;

  // FSYNC "shall not be floating, must be set high or low by the MCU"
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOC, GPIO_Pin_14);

  // Enable the MPU6500 interrupt on PC13
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource13);

  EXTI_InitStructure.EXTI_Line = EXTI_Line13;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  portDISABLE_INTERRUPTS();
  EXTI_Init(&EXTI_InitStructure);
  EXTI_ClearITPendingBit(EXTI_Line13);
  portENABLE_INTERRUPTS();
}

void sensorsInit(void)
{
  if (isInit) { return; }

  sensorsDataReady = xSemaphoreCreateBinary();

  sensorsDeviceInit();
  sensorsInterruptInit();
  sensorsTaskInit();

  isInit = true;
}

bool sensorsTest(void)
{
  if (!isInit)
  {
    DEBUG_PRINT("Error while initializing sensor task\r\n");
  }

  return isInit;
}

void __attribute__((used)) EXTI13_Callback(void) {
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(sensorsDataReady, &xHigherPriorityTaskWoken);

  if (xHigherPriorityTaskWoken)
  {
    portYIELD();
  }
}
