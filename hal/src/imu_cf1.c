/**
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
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
 *
 * imu.c - inertial measurement unit
 */
#define DEBUG_MODULE "IMU"

#include <math.h>

#include "stm32f10x_conf.h"
#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"
#include "configblock.h"
#include "cfassert.h"
#include "imu.h"
#include "i2cdev.h"
#include "mpu6050.h"
#include "hmc5883l.h"
#include "ms5611.h"
#include "ledseq.h"
#include "uart.h"
#include "param.h"

#define IMU_ENABLE_MAG_HMC5883
#define IMU_ENABLE_PRESSURE_MS5611
//#define IMU_MPU6050_DLPF_256HZ

#define IMU_GYRO_FS_CFG       MPU6050_GYRO_FS_2000
#define IMU_DEG_PER_LSB_CFG   MPU6050_DEG_PER_LSB_2000
#define IMU_ACCEL_FS_CFG      MPU6050_ACCEL_FS_8
#define IMU_G_PER_LSB_CFG     MPU6050_G_PER_LSB_8
#define IMU_1G_RAW            (int16_t)(1.0 / MPU6050_G_PER_LSB_8)

#define IMU_STARTUP_TIME_MS   1000

#define GYRO_NBR_OF_AXES 3
#define GYRO_X_SIGN      (-1)
#define GYRO_Y_SIGN      (-1)
#define GYRO_Z_SIGN      (-1)
#define GYRO_NBR_OF_AXES            3
#define GYRO_MIN_BIAS_TIMEOUT_MS    M2T(1*1000)

#define IMU_NBR_OF_BIAS_SAMPLES  128

#define GYRO_VARIANCE_BASE        4000
#define GYRO_VARIANCE_THRESHOLD_X (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Y (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Z (GYRO_VARIANCE_BASE)

#define MAG_GAUSS_PER_LSB_CFG    HMC5883L_GAIN_660
#define MAG_GAUSS_PER_LSB        660.0


typedef struct
{
  Axis3i16   bias;
  bool       isBiasValueFound;
  bool       isBufferFilled;
  Axis3i16*  bufHead;
  Axis3i16   buffer[IMU_NBR_OF_BIAS_SAMPLES];
} BiasObj;

static BiasObj    gyroBias;
static BiasObj    accelBias;
static int32_t    varianceSampleTime;
static Axis3i16   gyroMpu;
static Axis3i16   accelMpu;
static Axis3i16   accelLPF;
static Axis3i16   accelLPFAligned;
static Axis3i16   mag;
static Axis3i32   accelStoredFilterValues;
static uint8_t    imuAccLpfAttFactor;
static bool       isHmc5883lPresent;
static bool       isMs5611Present;

static bool isMpu6050TestPassed;
static bool isHmc5883lTestPassed;
static bool isMs5611TestPassed;

// Pre-calculated values for accelerometer alignment
static float cosPitch;
static float sinPitch;
static float cosRoll;
static float sinRoll;

/**
 * MPU6050 selt test function. If the chip is moved to much during the self test
 * it will cause the test to fail.
 */
static void imuBiasInit(BiasObj* bias);
static void imuCalculateBiasMean(BiasObj* bias, Axis3i32* meanOut);
static void imuCalculateVarianceAndMean(BiasObj* bias, Axis3i32* varOut, Axis3i32* meanOut);
static bool imuFindBiasValue(BiasObj* bias);
static void imuAddBiasValue(BiasObj* bias, Axis3i16* dVal);
static void imuAccIIRLPFilter(Axis3i16* in, Axis3i16* out,
                              Axis3i32* storedValues, int32_t attenuation);
static void imuAccAlignToGravity(Axis3i16* in, Axis3i16* out);

static bool isInit;

void imu6Init(void)
{
  if(isInit)
    return;

 isHmc5883lPresent = false;
 isMs5611Present = false;

  // Wait for sensors to startup
  while (xTaskGetTickCount() < M2T(IMU_STARTUP_TIME_MS));

  i2cdevInit(I2C1);
  mpu6050Init(I2C1);
  if (mpu6050TestConnection() == true)
  {
    DEBUG_PRINT("MPU6050 I2C connection [OK].\n");
  }
  else
  {
    DEBUG_PRINT("MPU6050 I2C connection [FAIL].\n");
  }

  mpu6050Reset();
  vTaskDelay(M2T(50));
  // Activate MPU6050
  mpu6050SetSleepEnabled(false);
  // Enable temp sensor
  mpu6050SetTempSensorEnabled(true);
  // Disable interrupts
  mpu6050SetIntEnabled(false);
  // Connect the HMC5883L to the main I2C bus
  mpu6050SetI2CBypassEnabled(true);
  // Set x-axis gyro as clock source
  mpu6050SetClockSource(MPU6050_CLOCK_PLL_XGYRO);
  // Set gyro full scale range
  mpu6050SetFullScaleGyroRange(IMU_GYRO_FS_CFG);
  // Set accelerometer full scale range
  mpu6050SetFullScaleAccelRange(IMU_ACCEL_FS_CFG);

#ifdef IMU_MPU6050_DLPF_256HZ
  // 256Hz digital low-pass filter only works with little vibrations
  // Set output rate (15): 8000 / (1 + 15) = 500Hz
  mpu6050SetRate(15);
  // Set digital low-pass bandwidth
  mpu6050SetDLPFMode(MPU6050_DLPF_BW_256);
#else
  // To low DLPF bandwidth might cause instability and decrease agility
  // but it works well for handling vibrations and unbalanced propellers
  // Set output rate (1): 1000 / (1 + 1) = 500Hz
  mpu6050SetRate(1);
  // Set digital low-pass bandwidth
  mpu6050SetDLPFMode(MPU6050_DLPF_BW_98);
#endif


#ifdef IMU_ENABLE_MAG_HMC5883
  hmc5883lInit(I2C1);
	if (hmc5883lTestConnection() == true)
	{
		isHmc5883lPresent = true;
    DEBUG_PRINT("HMC5883 I2C connection [OK].\n");
  }
  else
  {
    DEBUG_PRINT("HMC5883L I2C connection [FAIL].\n");
  }
#endif

#ifdef IMU_ENABLE_PRESSURE_MS5611
  if (ms5611Init(I2C1) == true)
  {
    isMs5611Present = true;
    DEBUG_PRINT("MS5611 I2C connection [OK].\n");
  }
  else
  {
    DEBUG_PRINT("MS5611 I2C connection [FAIL].\n");
  }
#endif

  imuBiasInit(&gyroBias);
  imuBiasInit(&accelBias);
  varianceSampleTime = -GYRO_MIN_BIAS_TIMEOUT_MS + 1;
  imuAccLpfAttFactor = IMU_ACC_IIR_LPF_ATT_FACTOR;

  cosPitch = cos(configblockGetCalibPitch() * M_PI/180);
  sinPitch = sin(configblockGetCalibPitch() * M_PI/180);
  cosRoll = cos(configblockGetCalibRoll() * M_PI/180);
  sinRoll = sin(configblockGetCalibRoll() * M_PI/180);

  isInit = true;
}

bool imu6Test(void)
{
  bool testStatus = true;

  if (!isInit)
  {
    DEBUG_PRINT("Uninitialized");
    testStatus = false;
  }
  // Test for CF 10-DOF variant with none responding sensor
  if((isHmc5883lPresent && !isMs5611Present) ||
     (!isHmc5883lPresent && isMs5611Present))
  {
    DEBUG_PRINT("HMC5883L or MS5611 is not responding");
    testStatus = false;
  }
  if (testStatus)
  {
    isMpu6050TestPassed = mpu6050SelfTest();
    testStatus = isMpu6050TestPassed ;
  }
  if (testStatus && isHmc5883lPresent)
  {
    isHmc5883lTestPassed = hmc5883lSelfTest();
    testStatus = isHmc5883lTestPassed;
  }
  if (testStatus && isMs5611Present)
  {
    isMs5611TestPassed = ms5611SelfTest();
    testStatus = isMs5611TestPassed;
  }

  return testStatus;
}


void imu6Read(Axis3f* gyroOut, Axis3f* accOut)
{
  mpu6050GetMotion6(&accelMpu.x, &accelMpu.y, &accelMpu.z, &gyroMpu.x, &gyroMpu.y, &gyroMpu.z);

  imuAddBiasValue(&gyroBias, &gyroMpu);
  if (!accelBias.isBiasValueFound)
  {
    imuAddBiasValue(&accelBias, &accelMpu);
  }
  if (!gyroBias.isBiasValueFound)
  {
    imuFindBiasValue(&gyroBias);
    if (gyroBias.isBiasValueFound)
    {
      ledseqRun(LED_RED, seq_calibrated);
//      uartPrintf("Gyro bias: %i, %i, %i\n",
//                  gyroBias.bias.x, gyroBias.bias.y, gyroBias.bias.z);
    }
  }

#ifdef IMU_TAKE_ACCEL_BIAS
  if (gyroBias.isBiasValueFound &&
      !accelBias.isBiasValueFound)
  {
    Axis3i32 mean;

    imuCalculateBiasMean(&accelBias, &mean);
    accelBias.bias.x = mean.x;
    accelBias.bias.y = mean.y;
    accelBias.bias.z = mean.z - IMU_1G_RAW;
    accelBias.isBiasValueFound = true;
    //uartPrintf("Accel bias: %i, %i, %i\n",
    //            accelBias.bias.x, accelBias.bias.y, accelBias.bias.z);
  }
#endif

  imuAccIIRLPFilter(&accelMpu, &accelLPF, &accelStoredFilterValues,
                    (int32_t)imuAccLpfAttFactor);

  imuAccAlignToGravity(&accelLPF, &accelLPFAligned);

  // Re-map outputs
  gyroOut->x = (gyroMpu.x - gyroBias.bias.x) * IMU_DEG_PER_LSB_CFG;
  gyroOut->y = (gyroMpu.y - gyroBias.bias.y) * IMU_DEG_PER_LSB_CFG;
  gyroOut->z = (gyroMpu.z - gyroBias.bias.z) * IMU_DEG_PER_LSB_CFG;
  accOut->x = (accelLPFAligned.x - accelBias.bias.x) * IMU_G_PER_LSB_CFG;
  accOut->y = (accelLPFAligned.y - accelBias.bias.y) * IMU_G_PER_LSB_CFG;
  accOut->z = (accelLPFAligned.z - accelBias.bias.z) * IMU_G_PER_LSB_CFG;
}

void imu9Read(Axis3f* gyroOut, Axis3f* accOut, Axis3f* magOut)
{
  imu6Read(gyroOut, accOut);

  if (isHmc5883lPresent)
  {
    hmc5883lGetHeading(&mag.x, &mag.y, &mag.z);
    magOut->x = (float)mag.x / MAG_GAUSS_PER_LSB;
    magOut->y = (float)mag.y / MAG_GAUSS_PER_LSB;
    magOut->z = (float)mag.z / MAG_GAUSS_PER_LSB;
  }
  else
  {
    magOut->x = 0.0;
    magOut->y = 0.0;
    magOut->z = 0.0;
  }
}

bool imu6IsCalibrated(void)
{
  bool status;

  status = gyroBias.isBiasValueFound;
#ifdef IMU_TAKE_ACCEL_BIAS
  status &= accelBias.isBiasValueFound;
#endif

  return status;
}

bool imuHasBarometer(void)
{
  return isMs5611Present;
}

bool imuHasMangnetometer(void)
{
  return isHmc5883lPresent;
}

static void imuBiasInit(BiasObj* bias)
{
  bias->isBufferFilled = false;
  bias->bufHead = bias->buffer;
}

/**
 * Calculates the variance and mean for the bias buffer.
 */
static void imuCalculateVarianceAndMean(BiasObj* bias, Axis3i32* varOut, Axis3i32* meanOut)
{
  uint32_t i;
  int32_t sum[GYRO_NBR_OF_AXES] = {0};
  int64_t sumSq[GYRO_NBR_OF_AXES] = {0};

  for (i = 0; i < IMU_NBR_OF_BIAS_SAMPLES; i++)
  {
    sum[0] += bias->buffer[i].x;
    sum[1] += bias->buffer[i].y;
    sum[2] += bias->buffer[i].z;
    sumSq[0] += bias->buffer[i].x * bias->buffer[i].x;
    sumSq[1] += bias->buffer[i].y * bias->buffer[i].y;
    sumSq[2] += bias->buffer[i].z * bias->buffer[i].z;
  }

  varOut->x = (sumSq[0] - ((int64_t)sum[0] * sum[0]) / IMU_NBR_OF_BIAS_SAMPLES);
  varOut->y = (sumSq[1] - ((int64_t)sum[1] * sum[1]) / IMU_NBR_OF_BIAS_SAMPLES);
  varOut->z = (sumSq[2] - ((int64_t)sum[2] * sum[2]) / IMU_NBR_OF_BIAS_SAMPLES);

  meanOut->x = sum[0] / IMU_NBR_OF_BIAS_SAMPLES;
  meanOut->y = sum[1] / IMU_NBR_OF_BIAS_SAMPLES;
  meanOut->z = sum[2] / IMU_NBR_OF_BIAS_SAMPLES;

  isInit = true;
}

/**
 * Calculates the mean for the bias buffer.
 */
static void __attribute__((used)) imuCalculateBiasMean(BiasObj* bias, Axis3i32* meanOut)
{
  uint32_t i;
  int32_t sum[GYRO_NBR_OF_AXES] = {0};

  for (i = 0; i < IMU_NBR_OF_BIAS_SAMPLES; i++)
  {
    sum[0] += bias->buffer[i].x;
    sum[1] += bias->buffer[i].y;
    sum[2] += bias->buffer[i].z;
  }

  meanOut->x = sum[0] / IMU_NBR_OF_BIAS_SAMPLES;
  meanOut->y = sum[1] / IMU_NBR_OF_BIAS_SAMPLES;
  meanOut->z = sum[2] / IMU_NBR_OF_BIAS_SAMPLES;

}

/**
 * Adds a new value to the variance buffer and if it is full
 * replaces the oldest one. Thus a circular buffer.
 */
static void imuAddBiasValue(BiasObj* bias, Axis3i16* dVal)
{
  bias->bufHead->x = dVal->x;
  bias->bufHead->y = dVal->y;
  bias->bufHead->z = dVal->z;
  bias->bufHead++;

  if (bias->bufHead >= &bias->buffer[IMU_NBR_OF_BIAS_SAMPLES])
  {
    bias->bufHead = bias->buffer;
    bias->isBufferFilled = true;
  }
}

/**
 * Checks if the variances is below the predefined thresholds.
 * The bias value should have been added before calling this.
 * @param bias  The bias object
 */
static bool imuFindBiasValue(BiasObj* bias)
{
  bool foundBias = false;

  if (bias->isBufferFilled)
  {
    Axis3i32 variance;
    Axis3i32 mean;

    imuCalculateVarianceAndMean(bias, &variance, &mean);

    //uartSendData(sizeof(variance), (uint8_t*)&variance);
    //uartSendData(sizeof(mean), (uint8_t*)&mean);
    //uartPrintf("%i, %i, %i", variance.x, variance.y, variance.z);
    //uartPrintf("    %i, %i, %i\n", mean.x, mean.y, mean.z);

    if (variance.x < GYRO_VARIANCE_THRESHOLD_X &&
        variance.y < GYRO_VARIANCE_THRESHOLD_Y &&
        variance.z < GYRO_VARIANCE_THRESHOLD_Z &&
        (varianceSampleTime + GYRO_MIN_BIAS_TIMEOUT_MS < xTaskGetTickCount()))
    {
      varianceSampleTime = xTaskGetTickCount();
      bias->bias.x = mean.x;
      bias->bias.y = mean.y;
      bias->bias.z = mean.z;
      foundBias = true;
      bias->isBiasValueFound = true;
    }
  }

  return foundBias;
}

static void imuAccIIRLPFilter(Axis3i16* in, Axis3i16* out, Axis3i32* storedValues, int32_t attenuation)
{
  out->x = iirLPFilterSingle(in->x, attenuation, &storedValues->x);
  out->y = iirLPFilterSingle(in->y, attenuation, &storedValues->y);
  out->z = iirLPFilterSingle(in->z, attenuation, &storedValues->z);
}


/**
 * Compensate for a miss-aligned accelerometer. It uses the trim
 * data gathered from the UI and written in the config-block to
 * rotate the accelerometer to be aligned with gravity.
 */
static void imuAccAlignToGravity(Axis3i16* in, Axis3i16* out)
{
  Axis3i16 rx;
  Axis3i16 ry;

  // Rotate around x-axis
  rx.x = in->x;
  rx.y = in->y * cosRoll - in->z * sinRoll;
  rx.z = in->y * sinRoll + in->z * cosRoll;

  // Rotate around y-axis
  ry.x = rx.x * cosPitch - rx.z * sinPitch;
  ry.y = rx.y;
  ry.z = -rx.x * sinPitch + rx.z * cosPitch;

  out->x = ry.x;
  out->y = ry.y;
  out->z = ry.z;
}

PARAM_GROUP_START(imu_acc_lpf)
PARAM_ADD(PARAM_UINT8, factor, &imuAccLpfAttFactor)
PARAM_GROUP_STOP(imu_acc_lpf)

PARAM_GROUP_START(imu_sensors)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, HMC5883L, &isHmc5883lPresent)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, MS5611, &isMs5611Present)
PARAM_GROUP_STOP(imu_sensors)

PARAM_GROUP_START(imu_tests)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, MPU6050, &isMpu6050TestPassed)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, HMC5883L, &isHmc5883lTestPassed)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, MS5611, &isMs5611TestPassed)
PARAM_GROUP_STOP(imu_tests)
