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
 * sensors_bst.c - sensors Bosch
 */
#define DEBUG_MODULE "IMU"

#include <math.h>
#include "stm32fxxx.h"

#include "sensors.h"
#include "imu.h"
// BST Drivers
#include "bmi160.h"
#include "bmp280.h"
#include "bma2x2.h"
#include "bmg160.h"
#include "bstdr_comm_support.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "system.h"
#include "configblock.h"
#include "param.h"
#include "debug.h"
#include "imu.h"
#include "nvicconf.h"
#include "ledseq.h"
#include "sound.h"
#include "filter.h"

#define SENSORS_ENABLE_BMI055
#define SENSORS_ENABLE_BMI160
#define SENSORS_ENABLE_PRESSURE_BMP280

#define SENSORS_READ_RATE_HZ      1000
#define SENSORS_STARTUP_TIME_MS   1000

#define SENSORS_BMI160_G_CFG             16
#define SENSORS_BMI160_G_PER_LSB_CFG     (2.0f * SENSORS_BMI160_G_CFG) / 65536.0f
#define SENSORS_BMI160_DEG_PER_LSB_CFG   (2.0f * 2000.0f) / 65536.0f
#define SENSORS_1G_RAW            (int16_t)(1.0f / (float)((2.0f * SENSORS_BMI160_G_CFG) / 65536.0f))

#define SENSORS_BMI055_1G_PER_LSB_CFG   (2.0f * 8.0f / (4096))
#define SENSORS_BMI055_1G_IN_LSB		    (int16_t)(1.0f / SENSORS_BMI055_1G_PER_LSB_CFG)

#define SENSORS_VARIANCE_MAN_TEST_TIMEOUT M2T(1000) // Timeout in ms
#define SENSORS_MAN_TEST_LEVEL_MAX        5.0f      // Max degrees off

#define GYRO_NBR_OF_AXES            3
#define GYRO_MIN_BIAS_TIMEOUT_MS    M2T(1*1000)

#define SENSORS_TAKE_ACCEL_BIAS

// Number of samples used in variance calculation. Changing this effects the threshold
#define SENSORS_NBR_OF_BIAS_SAMPLES  512

// Variance threshold to take zero bias for gyro
#define GYRO_VARIANCE_BASE        2000
#define GYRO_VARIANCE_THRESHOLD_X (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Y (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Z (GYRO_VARIANCE_BASE)

typedef enum
{
  SENSORS_BMI055 = 1,
  SENSORS_BMI160 = 2
} sensorsTypes_e;

// holds the IMU type
static sensorsTypes_e usedImuType = SENSORS_BMI160;

typedef struct
{
  Axis3f bias;
  bool isBiasValueFound;
  bool isBufferFilled;
  Axis3i16* bufHead;
  Axis3i16 buffer[SENSORS_NBR_OF_BIAS_SAMPLES];
} BiasObj;

static xQueueHandle accelerometerDataQueue;
static xQueueHandle gyroDataQueue;
static xQueueHandle barometerDataQueue;

static bool isInit = false;
static sensorData_t sensors;

static int32_t varianceSampleTime;
static Axis3i16 accelLPF;
static Axis3i32 accelStoredFilterValues;
static uint8_t sensorsAccLpfAttFactor;

static bool isBarometerPresent = false;

// Bosch sensors support
#ifdef SENSORS_ENABLE_BMI160
static BiasObj gyroBiasBmi160;
static BiasObj accelBiasBmi160;
static Axis3i16 gyroBmi160;
static Axis3i16 accelBmi160;
static struct bmi160_dev bmi160Dev;
//static bool isBmi160TestPassed = true;
#endif

#ifdef SENSORS_ENABLE_BMI055
static BiasObj gyroBiasBmi055;
static BiasObj accelBiasBmi055;
static Axis3i16 gyroBmi055;
static Axis3i16 accelBmi055;
static bma2x2_t bmi055DevAcc;
static bmg160_t bmi055DevGyro;
//static bool isBmi055TestPassed = true;
#endif

//
#ifdef SENSORS_ENABLE_PRESSURE_BMP280
static bmp280_t bmp280Dev;
//static bool isBMP280TestPassed = true;
#endif

// Pre-calculated values for accelerometer alignment
float cosPitch;
float sinPitch;
float cosRoll;
float sinRoll;

static void sensorsBiasInit(BiasObj* bias);
static void sensorsCalculateBiasMean(BiasObj* bias, Axis3i32* meanOut);
static void sensorsCalculateVarianceAndMean(BiasObj* bias, Axis3f* varOut, Axis3f* meanOut);
static bool sensorsFindBiasValue(BiasObj* bias);
static void sensorsAddBiasValue(BiasObj* bias, Axis3i16* dVal);
static void sensorsAccIIRLPFilter(Axis3i16* in, Axis3i16* out, Axis3i32* storedValues, int32_t attenuation);
static void sensorsAccAlignToGravity(Axis3f* in, Axis3f* out);
static void sensorsRead(Axis3f* gyroOut, Axis3f* accOut);
static void sensorsTaskInit(void);

bool sensorsReadGyro(Axis3f *gyro)
{
  return (pdTRUE == xQueueReceive(gyroDataQueue, gyro, 0));
}

bool sensorsReadAcc(Axis3f *acc)
{
  return (pdTRUE == xQueueReceive(accelerometerDataQueue, acc, 0));
}

bool sensorsReadMag(Axis3f *mag)
{
  return 0;
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

bool sensorsAreCalibrated()
{
  bool gyroBiasFound;

  switch (usedImuType)
  {
    case SENSORS_BMI055:
#ifdef SENSORS_ENABLE_BMI055
      gyroBiasFound = gyroBiasBmi055.isBiasValueFound;
#endif
      break;
    case SENSORS_BMI160:
#ifdef SENSORS_ENABLE_BMI160
      gyroBiasFound = gyroBiasBmi160.isBiasValueFound;
#endif
      break;
    default:
      gyroBiasFound = false;
      break;
  }

  return gyroBiasFound;
}

static void sensorsTask(void *param)
{
  uint32_t lastWakeTime = xTaskGetTickCount();

  systemWaitStart();

  while (1)
  {
    vTaskDelayUntil(&lastWakeTime, F2T(SENSORS_READ_RATE_HZ));

    sensorsRead(&sensors.gyro, &sensors.acc);

    // ensure all queues are populated at the same time
    vTaskSuspendAll();
    xQueueOverwrite(accelerometerDataQueue, &sensors.acc);
    xQueueOverwrite(gyroDataQueue, &sensors.gyro);
    if (isBarometerPresent)
    {
      xQueueOverwrite(barometerDataQueue, &sensors.baro);
    }
    xTaskResumeAll();
  }
}

void sensorsDeviceInit(void)
{
  if (isInit)
    return;

  isBarometerPresent = false;

  // Wait for sensors to startup
  while (xTaskGetTickCount() < M2T(SENSORS_STARTUP_TIME_MS))
    ;

  // BMI160 Initialization
#ifdef SENSORS_ENABLE_BMI160
  bstdr_ret_t ret_res;

  bmi160Dev.read = (bmi160_com_fptr_t)bstdr_burst_read;  // assign bus read function for bst devices
  bmi160Dev.write = (bmi160_com_fptr_t)bstdr_burst_write;  // assign bus write function for bst devices
  bmi160Dev.delay_ms = (bmi160_delay_fptr_t)bstdr_ms_delay; // assign delay function
  bmi160Dev.id = 0x69;  // I2C device address
  ret_res = bmi160_init(&bmi160Dev); // initialize the device

  if (ret_res == BSTDR_OK)
  {
    DEBUG_PRINT("BMI160 I2C connection [OK].\n");
    /* Select the Output data rate, range of accelerometer sensor ~92Hz BW by OSR4 @ODR=800Hz */
    bmi160Dev.accel_cfg.odr = BMI160_ACCEL_ODR_800HZ;
    bmi160Dev.accel_cfg.range = BMI160_ACCEL_RANGE_16G;
    bmi160Dev.accel_cfg.bw = BMI160_ACCEL_BW_OSR4_AVG1;
    /* Select the power mode of accelerometer sensor */
    bmi160Dev.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

    /* Select the Output data rate, range of Gyroscope sensor ~92Hz BW by OSR4 @ODR=800Hz */
    bmi160Dev.gyro_cfg.odr = BMI160_GYRO_ODR_800HZ;
    bmi160Dev.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
    bmi160Dev.gyro_cfg.bw = BMI160_GYRO_BW_OSR4_MODE;

    /* Select the power mode of Gyroscope sensor */
    bmi160Dev.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

    /* Set the sensor configuration */
    ret_res = bmi160_set_sens_conf(&bmi160Dev);
  }
  else
  {
    DEBUG_PRINT("BMI160 I2C connection [FAIL].\n");
  }

#ifdef SENSORS_TAKE_ACCEL_BIAS
  sensorsBiasInit(&accelBiasBmi160);
#endif
  sensorsBiasInit(&gyroBiasBmi160);

#endif

#ifdef SENSORS_ENABLE_BMI055
  bmi055DevAcc.bus_read = bstdr_burst_read;	// assign bus read function for bst devices
  bmi055DevAcc.bus_write = bstdr_burst_write;	// assign bus write function for bst devices
  bmi055DevAcc.delay = bstdr_ms_delay;	// assign delay function
  bmi055DevAcc.dev_addr = 0x18;

  bmi055DevGyro.bus_read = bstdr_burst_read;	// assign bus read function for bst devices
  bmi055DevGyro.bus_write = bstdr_burst_write;	// assign bus write function for bst devices
  bmi055DevGyro.delay = bstdr_ms_delay;	// assign delay function
  bmi055DevGyro.dev_addr = 0x68;

  bma2x2_init(&bmi055DevAcc); // initialize the device

  if (bma2x2_check_connection() == BSTDR_OK)
  {

    DEBUG_PRINT("BMI055 Accel I2C connection [OK].\n");
    bma2x2_set_power_mode(BMA2x2_MODE_NORMAL);
    bma2x2_set_bw(BMA2x2_BW_125HZ);
    bma2x2_set_range(BMA2x2_RANGE_8G);

    bmi055DevAcc.delay(10);
    bma2x2_xyz_t acc;
    bma2x2_read_accel_xyz(&acc);
  }
  else
  {
    DEBUG_PRINT("BMI055 Accel I2C connection [FAIL].\n");
  }

  bmg160_init(&bmi055DevGyro); // initialize the device

  if (bmg160_check_connection() == BSTDR_OK)
  {
    DEBUG_PRINT("BMI055 Gyro I2C connection [OK].\n");
    bmg160_set_power_mode(BMG160_MODE_NORMAL);
    bmi055DevGyro.delay(50);
    bmg160_set_bw(BMG160_BW_116_HZ);
    bmg160_set_range(BMG160_RANGE_2000);
    bmi055DevGyro.delay(50);
    bmg160_xyz_t gyr;
    bmg160_get_data_XYZ(&gyr);
  }
  else
  {
    DEBUG_PRINT("BMI055 Gyro I2C connection [FAIL].\n");
  }

  sensorsBiasInit(&gyroBiasBmi055);
#ifdef SENSORS_TAKE_ACCEL_BIAS
  sensorsBiasInit(&accelBiasBmi055);
#endif

#endif

// Bosch Sensortec Sensor
#ifdef SENSORS_ENABLE_PRESSURE_BMP280
  bstdr_ret_t comtest = BSTDR_E_GEN_ERROR;
  bmp280Dev.bus_read = bstdr_burst_read;	// assign bus read function for bst devices
  bmp280Dev.bus_write = bstdr_burst_write;	// assign bus write function for bst devices
  bmp280Dev.delay = bstdr_ms_delay;	// assign delay function
  bmp280Dev.dev_addr = 0x76;
  bmp280_init(&bmp280Dev); // initialize the device
  comtest = bmp280_test_connection();
  if (comtest == BSTDR_OK)
  {
    isBarometerPresent = true;
    DEBUG_PRINT("BME280 I2C connection [OK].\n");
    bmp280_set_filter(0x03);
    bmp280_set_work_mode(BMP280_STANDARD_RESOLUTION_MODE);
    bmp280_set_standby_durn(BMP280_STANDBY_TIME_1_MS);
    bmp280_set_power_mode(BMP280_NORMAL_MODE);
    bmp280Dev.delay(20);	// wait before first read out
    // read out data
    int32_t v_temp_s32;
    uint32_t v_pres_u32;
    bmp280_read_data(&v_pres_u32, &v_temp_s32);
  }
#endif

  varianceSampleTime = -GYRO_MIN_BIAS_TIMEOUT_MS + 1;
  sensorsAccLpfAttFactor = IMU_ACC_IIR_LPF_ATT_FACTOR;

  cosPitch = cosf(configblockGetCalibPitch() * (float) M_PI / 180);
  sinPitch = sinf(configblockGetCalibPitch() * (float) M_PI / 180);
  cosRoll = cosf(configblockGetCalibRoll() * (float) M_PI / 180);
  sinRoll = sinf(configblockGetCalibRoll() * (float) M_PI / 180);

  isInit = true;
}

void sensorsInit(void)
{
  if (isInit)
  {
    return;
  }

  sensorsDeviceInit();
  sensorsTaskInit();

  isInit = true;
}

bool sensorsTest(void)
{
  bool testStatus = true;

  if (!isInit)
  {
    DEBUG_PRINT("Uninitialized\n");
    testStatus = false;
  }

  return testStatus;
}

bool sensorsManufacturingTest(void)
{
  return true;
}

static void sensorsTaskInit(void)
{
  accelerometerDataQueue = xQueueCreate(1, sizeof(Axis3f));
  gyroDataQueue = xQueueCreate(1, sizeof(Axis3f));
  barometerDataQueue = xQueueCreate(1, sizeof(baro_t));

  xTaskCreate(sensorsTask, SENSORS_TASK_NAME, SENSORS_TASK_STACKSIZE, NULL, SENSORS_TASK_PRI, NULL);
}

static void sensorsRead(Axis3f* gyroOut, Axis3f* accOut)
{
  Axis3f accScaled;

  // read out the data
  switch (usedImuType)
  {
    case SENSORS_BMI055:
#ifdef SENSORS_ENABLE_BMI055
    {
      bma2x2_xyz_t bmi055acc;
      bmg160_xyz_t bmi055gyr;
      bma2x2_read_accel_xyz(&bmi055acc);
      bmg160_get_data_XYZ(&bmi055gyr);
      // re-align the axes
      accelBmi055.x = bmi055acc.x;
      accelBmi055.y = bmi055acc.y;
      accelBmi055.z = bmi055acc.z;
      gyroBmi055.x = bmi055gyr.x;
      gyroBmi055.y = bmi055gyr.y;
      gyroBmi055.z = bmi055gyr.z;
    }
#endif
      break;
    case SENSORS_BMI160:
#ifdef SENSORS_ENABLE_BMI160
    {
      struct bmi160_sensor_data bmi160acc;
      struct bmi160_sensor_data bmi160gyr;
      bmi160_get_sensor_data(BMI160_BOTH_ACCEL_AND_GYRO, &bmi160acc, &bmi160gyr, &bmi160Dev);
      // re-align the axes
      accelBmi160.x = bmi160acc.x;
      accelBmi160.y = bmi160acc.y;
      accelBmi160.z = bmi160acc.z;
      gyroBmi160.x = bmi160gyr.x;
      gyroBmi160.y = bmi160gyr.y;
      gyroBmi160.z = bmi160gyr.z;
    }
#endif
      break;
  }

  // output depends on the
  switch (usedImuType)
  {
    case SENSORS_BMI160:
      sensorsAddBiasValue(&gyroBiasBmi160, &gyroBmi160);
#ifdef SENSORS_TAKE_ACCEL_BIAS
      if (!accelBiasBmi160.isBiasValueFound)
      {
        sensorsAddBiasValue(&accelBiasBmi160, &accelBmi160);
      }
#endif
      if (!gyroBiasBmi160.isBiasValueFound)
      {
        sensorsFindBiasValue(&gyroBiasBmi160);
        if (gyroBiasBmi160.isBiasValueFound)
        {
          soundSetEffect(SND_CALIB);
          ledseqRun(SYS_LED, seq_calibrated);
        }
      }

#ifdef SENSORS_TAKE_ACCEL_BIAS
      if (gyroBiasBmi160.isBiasValueFound && !accelBiasBmi160.isBiasValueFound)
      {
        Axis3i32 mean;

        sensorsCalculateBiasMean(&accelBiasBmi160, &mean);
        accelBiasBmi160.bias.x = mean.x;
        accelBiasBmi160.bias.y = mean.y;
        accelBiasBmi160.bias.z = mean.z - SENSORS_1G_RAW;
        accelBiasBmi160.isBiasValueFound = true;
      }
#endif

#ifdef SENSORS_ENABLE_BMI160
      sensorsAccIIRLPFilter(&accelBmi160, &accelLPF, &accelStoredFilterValues, (int32_t) sensorsAccLpfAttFactor);
      gyroOut->x = ((float) gyroBmi160.x - gyroBiasBmi160.bias.x) * SENSORS_BMI160_DEG_PER_LSB_CFG;
      gyroOut->y = ((float) gyroBmi160.y - gyroBiasBmi160.bias.y) * SENSORS_BMI160_DEG_PER_LSB_CFG;
      gyroOut->z = ((float) gyroBmi160.z - gyroBiasBmi160.bias.z) * SENSORS_BMI160_DEG_PER_LSB_CFG;
      accScaled.x = (accelLPF.x - accelBiasBmi160.bias.x) * SENSORS_BMI160_G_PER_LSB_CFG;
      accScaled.y = (accelLPF.y - accelBiasBmi160.bias.y) * SENSORS_BMI160_G_PER_LSB_CFG;
      accScaled.z = (accelLPF.z - accelBiasBmi160.bias.z) * SENSORS_BMI160_G_PER_LSB_CFG;
      sensorsAccAlignToGravity(&accScaled, accOut);
#endif
      break;
    case SENSORS_BMI055:
      sensorsAddBiasValue(&gyroBiasBmi055, &gyroBmi055);
#ifdef SENSORS_TAKE_ACCEL_BIAS
      if (!accelBiasBmi055.isBiasValueFound)
      {
        sensorsAddBiasValue(&accelBiasBmi055, &accelBmi055);
      }
#endif
      if (!gyroBiasBmi055.isBiasValueFound)
      {
        sensorsFindBiasValue(&gyroBiasBmi055);
        if (gyroBiasBmi055.isBiasValueFound)
        {
          soundSetEffect(SND_CALIB);
          ledseqRun(SYS_LED, seq_calibrated);
        }
      }

#ifdef SENSORS_TAKE_ACCEL_BIAS
      if (gyroBiasBmi055.isBiasValueFound && !accelBiasBmi055.isBiasValueFound)
      {
        Axis3i32 mean;

        sensorsCalculateBiasMean(&accelBiasBmi055, &mean);
        accelBiasBmi055.bias.x = mean.x;
        accelBiasBmi055.bias.y = mean.y;
        accelBiasBmi055.bias.z = mean.z - SENSORS_BMI055_1G_IN_LSB;
        accelBiasBmi055.isBiasValueFound = true;
      }
#endif
#ifdef SENSORS_ENABLE_BMI055
      sensorsAccIIRLPFilter(&accelBmi055, &accelLPF, &accelStoredFilterValues, (int32_t) sensorsAccLpfAttFactor);
      gyroOut->x = ((float) gyroBmi055.x - gyroBiasBmi055.bias.x) * SENSORS_BMI160_DEG_PER_LSB_CFG;
      gyroOut->y = ((float) gyroBmi055.y - gyroBiasBmi055.bias.y) * SENSORS_BMI160_DEG_PER_LSB_CFG;
      gyroOut->z = ((float) gyroBmi055.z - gyroBiasBmi055.bias.z) * SENSORS_BMI160_DEG_PER_LSB_CFG;
      accScaled.x = (accelLPF.x - accelBiasBmi055.bias.x) * SENSORS_BMI055_1G_PER_LSB_CFG;
      accScaled.y = (accelLPF.y - accelBiasBmi055.bias.y) * SENSORS_BMI055_1G_PER_LSB_CFG;
      accScaled.z = (accelLPF.z - accelBiasBmi055.bias.z) * SENSORS_BMI055_1G_PER_LSB_CFG;
      sensorsAccAlignToGravity(&accScaled, accOut);
#endif
      break;
  }
}

bool sensorsIsCalibrated(void)
{
  bool status;

  // output depends on the
  switch (usedImuType)
  {
    case SENSORS_BMI160:
      status = gyroBiasBmi160.isBiasValueFound;
#ifdef SENSORS_TAKE_ACCEL_BIAS
      status &= accelBiasBmi160.isBiasValueFound;
#endif
      break;
    case SENSORS_BMI055:
      status = gyroBiasBmi055.isBiasValueFound;
#ifdef SENSORS_TAKE_ACCEL_BIAS
      status &= accelBiasBmi055.isBiasValueFound;
#endif
      break;
    default:
      status = false;
      break;
  }

  return status;
}

bool sensorsHasBarometer(void)
{
  return isBarometerPresent;
}

bool sensorsHasMangnetometer(void)
{
  return false;
}

static void sensorsBiasInit(BiasObj* bias)
{
  bias->isBufferFilled = false;
  bias->bufHead = bias->buffer;
}

/**
 * Calculates the variance and mean for the bias buffer.
 */
static void sensorsCalculateVarianceAndMean(BiasObj* bias, Axis3f* varOut, Axis3f* meanOut)
{
  uint32_t i;
  int64_t sum[GYRO_NBR_OF_AXES] =  { 0 };
  int64_t sumSq[GYRO_NBR_OF_AXES] =  { 0 };

  for (i = 0; i < SENSORS_NBR_OF_BIAS_SAMPLES; i++)
  {
    sum[0] += bias->buffer[i].x;
    sum[1] += bias->buffer[i].y;
    sum[2] += bias->buffer[i].z;
    sumSq[0] += bias->buffer[i].x * bias->buffer[i].x;
    sumSq[1] += bias->buffer[i].y * bias->buffer[i].y;
    sumSq[2] += bias->buffer[i].z * bias->buffer[i].z;
  }

  meanOut->x = (float) sum[0] / SENSORS_NBR_OF_BIAS_SAMPLES;
  meanOut->y = (float) sum[1] / SENSORS_NBR_OF_BIAS_SAMPLES;
  meanOut->z = (float) sum[2] / SENSORS_NBR_OF_BIAS_SAMPLES;

  varOut->x = fabs(sumSq[0] / SENSORS_NBR_OF_BIAS_SAMPLES - meanOut->x * meanOut->x);
  varOut->y = fabs(sumSq[1] / SENSORS_NBR_OF_BIAS_SAMPLES - meanOut->y * meanOut->y);
  varOut->z = fabs(sumSq[2] / SENSORS_NBR_OF_BIAS_SAMPLES - meanOut->z * meanOut->z);
}

/**
 * Calculates the mean for the bias buffer.
 */
static void __attribute__((used)) sensorsCalculateBiasMean(BiasObj* bias, Axis3i32* meanOut)
{
  uint32_t i;
  int32_t sum[GYRO_NBR_OF_AXES] =  { 0 };

  for (i = 0; i < SENSORS_NBR_OF_BIAS_SAMPLES; i++)
  {
    sum[0] += bias->buffer[i].x;
    sum[1] += bias->buffer[i].y;
    sum[2] += bias->buffer[i].z;
  }

  meanOut->x = sum[0] / SENSORS_NBR_OF_BIAS_SAMPLES;
  meanOut->y = sum[1] / SENSORS_NBR_OF_BIAS_SAMPLES;
  meanOut->z = sum[2] / SENSORS_NBR_OF_BIAS_SAMPLES;
}

/**
 * Adds a new value to the variance buffer and if it is full

 * replaces the oldest one. Thus a circular buffer.
 */
static void sensorsAddBiasValue(BiasObj* bias, Axis3i16* dVal)
{
  bias->bufHead->x = dVal->x;
  bias->bufHead->y = dVal->y;
  bias->bufHead->z = dVal->z;
  bias->bufHead++;

  if (bias->bufHead >= &bias->buffer[SENSORS_NBR_OF_BIAS_SAMPLES])
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
static bool sensorsFindBiasValue(BiasObj* bias)
{
  bool foundBias = false;

  if (bias->isBufferFilled)
  {
    Axis3f variance;
    Axis3f mean;

    sensorsCalculateVarianceAndMean(bias, &variance, &mean);

    if (variance.x < GYRO_VARIANCE_THRESHOLD_X && variance.y < GYRO_VARIANCE_THRESHOLD_Y && variance.z < GYRO_VARIANCE_THRESHOLD_Z
        && (varianceSampleTime + GYRO_MIN_BIAS_TIMEOUT_MS < xTaskGetTickCount()))
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

static void sensorsAccIIRLPFilter(Axis3i16* in, Axis3i16* out, Axis3i32* storedValues, int32_t attenuation)
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
static void sensorsAccAlignToGravity(Axis3f* in, Axis3f* out)
{
  Axis3f rx;
  Axis3f ry;

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

PARAM_GROUP_START(imu_sensors)
PARAM_ADD(PARAM_UINT8, BoschIMUSelect, &usedImuType)
PARAM_GROUP_STOP(imu_sensors)
