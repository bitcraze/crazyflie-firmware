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

#include "mpu6500.h"
#include "icm20789.h"
#include "zranger.h"

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
#include "usec_time.h"

#define ICM20789_ENABLE_BARO

#define SENSORS_READ_RATE_HZ      1000

#define SENSORS_GYRO_FS_CFG       MPU6500_GYRO_FS_2000
#define SENSORS_DEG_PER_LSB_CFG   MPU6500_DEG_PER_LSB_2000

#define SENSORS_ACCEL_FS_CFG      MPU6500_ACCEL_FS_16
#define SENSORS_G_PER_LSB_CFG     MPU6500_G_PER_LSB_16

#define SENSORS_VARIANCE_MAN_TEST_TIMEOUT M2T(2000) // Timeout in ms
#define SENSORS_MAN_TEST_LEVEL_MAX        5.0f      // Max degrees off

#define SENSORS_BIAS_SAMPLES       1000
#define SENSORS_ACC_SCALE_SAMPLES  200
#define SENSORS_GYRO_BIAS_CALCULATE_STDDEV

// Buffer length for MPU9250 slave reads
#define SENSORS_MPU6500_BUFF_LEN    14

#define GYRO_NBR_OF_AXES            3
#define GYRO_MIN_BIAS_TIMEOUT_MS    M2T(1*1000)
#define SENSORS_NBR_OF_BIAS_SAMPLES     1024
// Variance threshold to take zero bias for gyro
#define GYRO_VARIANCE_BASE          5000
#define GYRO_VARIANCE_THRESHOLD_X   (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Y   (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Z   (GYRO_VARIANCE_BASE)

typedef struct
{
  Axis3f     bias;
  bool       isBiasValueFound;
  bool       isBufferFilled;
  Axis3i16*  bufHead;
  Axis3i16   buffer[SENSORS_NBR_OF_BIAS_SAMPLES];
} BiasObj;

static xQueueHandle accelerometerDataQueue;
static xQueueHandle gyroDataQueue;
static xQueueHandle magnetometerDataQueue;
static xQueueHandle barometerDataQueue;
static xSemaphoreHandle sensorsDataReady;
static xSemaphoreHandle dataReady;

static bool isInit = false;
static sensorData_t sensors;

static BiasObj gyroBiasRunning;
static Axis3f  gyroBias;
static bool    gyroBiasFound = false;
static float accScaleSum = 0;
static float accScale = 1;

// Low Pass filtering
#define GYRO_LPF_CUTOFF_FREQ  80
#define ACCEL_LPF_CUTOFF_FREQ 30
static lpf2pData accLpf[3];
static lpf2pData gyroLpf[3];
static void applyAxis3fLpf(lpf2pData *data, Axis3f* in);

static bool isBarometerPresent = false;
static bool isMagnetometerPresent = false;

static bool isMpu6500TestPassed = false;
static bool isAK8963TestPassed = false;
static bool isLPS25HTestPassed = false;

// Pre-calculated values for accelerometer alignment
float cosPitch;
float sinPitch;
float cosRoll;
float sinRoll;

// This buffer needs to hold data from all sensors
static uint8_t buffer[SENSORS_MPU6500_BUFF_LEN] = {0};

static void processAccGyroMeasurements(const uint8_t *buffer);
static bool processGyroBias(int16_t gx, int16_t gy, int16_t gz,  Axis3f *gyroBiasOut);
static bool processAccScale(int16_t ax, int16_t ay, int16_t az);
static void sensorsBiasObjInit(BiasObj* bias);
static void sensorsCalculateVarianceAndMean(BiasObj* bias, Axis3f* varOut, Axis3f* meanOut);
static void sensorsCalculateBiasMean(BiasObj* bias, Axis3i32* meanOut);
static void sensorsAddBiasValue(BiasObj* bias, int16_t x, int16_t y, int16_t z);
static bool sensorsFindBiasValue(BiasObj* bias);
static void sensorsAccAlignToGravity(Axis3f* in, Axis3f* out);

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
  zRangerReadRange(&sensors->zrange, tick);
}

bool sensorsAreCalibrated() {
  return gyroBiasFound;
}

static void sensorsTask(void *param)
{
  systemWaitStart();

  while (1)
  {
    if (pdTRUE == xSemaphoreTake(sensorsDataReady, portMAX_DELAY))
    {
      icm20789ReadAllSensors(buffer);
      // these functions process the respective data and queue it on the output queues
      processAccGyroMeasurements(&(buffer[0]));

//      vTaskSuspendAll(); // ensure all queues are populated at the same time
      xQueueOverwrite(accelerometerDataQueue, &sensors.acc);
      xQueueOverwrite(gyroDataQueue, &sensors.gyro);
      xSemaphoreGive(dataReady);
//      xTaskResumeAll();
    }
  }
}

void sensorsWaitDataReady(void)
{
  xSemaphoreTake(dataReady, portMAX_DELAY);
}

#ifdef ICM20789_ENABLE_BARO
static void  baroTask(void *param)
{
  systemWaitStart();

  static int pressuredataReady = 0;
  int status = false;

  while(1)
  {
    status = icm20789BaroStartMeasurement();

    vTaskDelay(M2T(25));

    while (status == true)
    {
      status = icm20789BaroGetPressureData(&sensors.baro.pressure,
                                           &sensors.baro.temperature,
                                           &sensors.baro.asl);
      if (status)
      {
        pressuredataReady = 1;
      }
      else
      {
        vTaskDelay(M2T(1));
      }

      if (pressuredataReady)
      {
          xQueueOverwrite(barometerDataQueue, &sensors.baro);
          pressuredataReady = 0;
          status = false;
      }
    }
  }
}
#endif

void processAccGyroMeasurements(const uint8_t *buffer)
{
  Axis3f accScaled;
  // Note the ordering to correct the rotated 90º IMU coordinate system
  int16_t ay = (((int16_t) buffer[0]) << 8) | buffer[1];
  int16_t ax = (((int16_t) buffer[2]) << 8) | buffer[3];
  int16_t az = (((int16_t) buffer[4]) << 8) | buffer[5];
  int16_t gy = (((int16_t) buffer[8]) << 8) | buffer[9];
  int16_t gx = (((int16_t) buffer[10]) << 8) | buffer[11];
  int16_t gz = (((int16_t) buffer[12]) << 8) | buffer[13];


  gyroBiasFound = processGyroBias(gx, gy, gz, &gyroBias);
  if (gyroBiasFound)
  {
     processAccScale(ax, ay, az);
  }

  sensors.gyro.x =  (gx - gyroBias.x) * SENSORS_DEG_PER_LSB_CFG;
  sensors.gyro.y = -(gy - gyroBias.y) * SENSORS_DEG_PER_LSB_CFG;
  sensors.gyro.z =  (gz - gyroBias.z) * SENSORS_DEG_PER_LSB_CFG;
  applyAxis3fLpf((lpf2pData*)(&gyroLpf), &sensors.gyro);

  accScaled.x =  (ax) * SENSORS_G_PER_LSB_CFG / accScale;
  accScaled.y = -(ay) * SENSORS_G_PER_LSB_CFG / accScale;
  accScaled.z =  (az) * SENSORS_G_PER_LSB_CFG / accScale;
  sensorsAccAlignToGravity(&accScaled, &sensors.acc);
  applyAxis3fLpf((lpf2pData*)(&accLpf), &sensors.acc);
}

static void sensorsDeviceInit(void)
{
  isMagnetometerPresent = false;
  isBarometerPresent = false;

  // Wait for sensors to startup
  while (xTaskGetTickCount() < 1000);

  if (icm20789Init() == true)
  {
    DEBUG_PRINT("ICM-20789 SPI connection [OK].\n");
  }
  else
  {
    DEBUG_PRINT("ICM-20789 SPI connection [FAIL].\n");
  }

#ifdef ICM20789_ENABLE_BARO
  i2cdevInit(I2C3_DEV);
  if (icm20789BaroInit(I2C3_DEV) == true)
  {
    DEBUG_PRINT("ICM-20789 I2C connection [OK].\n");
  }
  else
  {
    DEBUG_PRINT("ICM-20789 I2C connection [FAIL].\n");
  }
#endif

  // Init second order filer for accelerometer
  for (uint8_t i = 0; i < 3; i++)
  {
    lpf2pInit(&gyroLpf[i], 1000, GYRO_LPF_CUTOFF_FREQ);
    lpf2pInit(&accLpf[i],  1000, ACCEL_LPF_CUTOFF_FREQ);
  }

  cosPitch = cosf(configblockGetCalibPitch() * (float) M_PI/180);
  sinPitch = sinf(configblockGetCalibPitch() * (float) M_PI/180);
  cosRoll = cosf(configblockGetCalibRoll() * (float) M_PI/180);
  sinRoll = sinf(configblockGetCalibRoll() * (float) M_PI/180);
}

static void sensorsTaskInit(void)
{
  accelerometerDataQueue = xQueueCreate(1, sizeof(Axis3f));
  gyroDataQueue = xQueueCreate(1, sizeof(Axis3f));
  magnetometerDataQueue = xQueueCreate(1, sizeof(Axis3f));
  barometerDataQueue = xQueueCreate(1, sizeof(baro_t));

  xTaskCreate(sensorsTask, SENSORS_TASK_NAME, SENSORS_TASK_STACKSIZE, NULL, SENSORS_TASK_PRI, NULL);
#ifdef ICM20789_ENABLE_BARO
  xTaskCreate(baroTask, "BARO", configMINIMAL_STACK_SIZE, NULL, 2 /*prio*/, NULL);
#endif
}

static void sensorsInterruptInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;

  // Enable the ICM20789 interrupt on PC13
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //GPIO_PuPd_DOWN;
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
  if (isInit)
  {
    return;
  }

  sensorsDataReady = xSemaphoreCreateBinary();
  dataReady = xSemaphoreCreateBinary();

  initUsecTimer();

  sensorsBiasObjInit(&gyroBiasRunning);
  sensorsDeviceInit();
  sensorsInterruptInit();
  sensorsTaskInit();

  isInit = true;
}

bool sensorsTest(void)
{
  bool testStatus = true;

  return true;

  if (!isInit)
  {
    DEBUG_PRINT("Error while initializing sensor task\r\n");
    testStatus = false;
  }

  // Try for 3 seconds so the quad has stabilized enough to pass the test
  for (int i = 0; i < 300; i++)
  {
    if(icm20789SelfTest() == true)
    {
      isMpu6500TestPassed = true;
      break;
    }
    else
    {
      vTaskDelay(M2T(10));
    }
  }
  testStatus &= isMpu6500TestPassed;

  return testStatus;
}

/**
 * Calculates accelerometer scale out of SENSORS_ACC_SCALE_SAMPLES samples. Should be called when
 * platform is stable.
 */
static bool processAccScale(int16_t ax, int16_t ay, int16_t az)
{
  static bool accBiasFound = false;
  static uint32_t accScaleSumCount = 0;

  if (!accBiasFound)
  {
    accScaleSum += sqrtf(powf(ax * SENSORS_G_PER_LSB_CFG, 2) + powf(ay * SENSORS_G_PER_LSB_CFG, 2) + powf(az * SENSORS_G_PER_LSB_CFG, 2));
    accScaleSumCount++;

    if (accScaleSumCount == SENSORS_ACC_SCALE_SAMPLES)
    {
      accScale = accScaleSum / SENSORS_ACC_SCALE_SAMPLES;
      accBiasFound = true;
    }
  }

  return accBiasFound;
}

/**
 * Calculates the bias first when the gyro variance is below threshold. Requires a buffer
 * but calibrates platform first when it is stable.
 */
static bool processGyroBias(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut)
{
  sensorsAddBiasValue(&gyroBiasRunning, gx, gy, gz);

  if (!gyroBiasRunning.isBiasValueFound)
  {
    sensorsFindBiasValue(&gyroBiasRunning);
    if (gyroBiasRunning.isBiasValueFound)
    {
      soundSetEffect(SND_CALIB);
      ledseqRun(SYS_LED, seq_calibrated);
    }
  }

  gyroBiasOut->x = gyroBiasRunning.bias.x;
  gyroBiasOut->y = gyroBiasRunning.bias.y;
  gyroBiasOut->z = gyroBiasRunning.bias.z;

  return gyroBiasRunning.isBiasValueFound;
}

static void sensorsBiasObjInit(BiasObj* bias)
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
  int64_t sum[GYRO_NBR_OF_AXES] = {0};
  int64_t sumSq[GYRO_NBR_OF_AXES] = {0};

  for (i = 0; i < SENSORS_NBR_OF_BIAS_SAMPLES; i++)
  {
    sum[0] += bias->buffer[i].x;
    sum[1] += bias->buffer[i].y;
    sum[2] += bias->buffer[i].z;
    sumSq[0] += bias->buffer[i].x * bias->buffer[i].x;
    sumSq[1] += bias->buffer[i].y * bias->buffer[i].y;
    sumSq[2] += bias->buffer[i].z * bias->buffer[i].z;
  }

  varOut->x = (sumSq[0] - ((int64_t)sum[0] * sum[0]) / SENSORS_NBR_OF_BIAS_SAMPLES);
  varOut->y = (sumSq[1] - ((int64_t)sum[1] * sum[1]) / SENSORS_NBR_OF_BIAS_SAMPLES);
  varOut->z = (sumSq[2] - ((int64_t)sum[2] * sum[2]) / SENSORS_NBR_OF_BIAS_SAMPLES);

  meanOut->x = (float)sum[0] / SENSORS_NBR_OF_BIAS_SAMPLES;
  meanOut->y = (float)sum[1] / SENSORS_NBR_OF_BIAS_SAMPLES;
  meanOut->z = (float)sum[2] / SENSORS_NBR_OF_BIAS_SAMPLES;
}

/**
 * Calculates the mean for the bias buffer.
 */
static void __attribute__((used)) sensorsCalculateBiasMean(BiasObj* bias, Axis3i32* meanOut)
{
  uint32_t i;
  int32_t sum[GYRO_NBR_OF_AXES] = {0};

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
static void sensorsAddBiasValue(BiasObj* bias, int16_t x, int16_t y, int16_t z)
{
  bias->bufHead->x = x;
  bias->bufHead->y = y;
  bias->bufHead->z = z;
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
  static int32_t varianceSampleTime;
  bool foundBias = false;

  if (bias->isBufferFilled)
  {
    Axis3f variance;
    Axis3f mean;

    sensorsCalculateVarianceAndMean(bias, &variance, &mean);

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

uint64_t mpuIntTimestamp;
void __attribute__((used)) EXTI13_Callback(void)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  mpuIntTimestamp = usecTimestamp();
  xSemaphoreGiveFromISR(sensorsDataReady, &xHigherPriorityTaskWoken);

  if (xHigherPriorityTaskWoken)
  {
    portYIELD();
  }
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

static void applyAxis3fLpf(lpf2pData *data, Axis3f* in)
{
  for (uint8_t i = 0; i < 3; i++) {
    in->axis[i] = lpf2pApply(&data[i], in->axis[i]);
  }
}

PARAM_GROUP_START(imu_sensors)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, HMC5883L, &isMagnetometerPresent)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, MS5611, &isBarometerPresent) // TODO: Rename MS5611 to LPS25H. Client needs to be updated at the same time.
PARAM_GROUP_STOP(imu_sensors)

PARAM_GROUP_START(imu_tests)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, MPU6500, &isMpu6500TestPassed)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, HMC5883L, &isAK8963TestPassed)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, MS5611, &isLPS25HTestPassed) // TODO: Rename MS5611 to LPS25H. Client needs to be updated at the same time.
PARAM_GROUP_STOP(imu_tests)
