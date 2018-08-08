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
 * sensors_bmi088_bmp388.c: IMU sensor driver for the *88 bosch sensors
 */

#define DEBUG_MODULE "IMU"

#include "sensors_bosch.h"
#include "i2cdev.h"

#define SENSORS_TAKE_ACCEL_BIAS

/* available sensors */
#define SENSORS_BMI055          0x01
#define SENSORS_BMI160          0x02
#define SENSORS_BMI088          0x04
#define SENSORS_BMM150          0x08
#define SENSORS_BMP280          0x10
#define SENSORS_BMP388          0x20

/* configure sensor's use
 * PRIMARIES are the sensors which are used for stabilization
 * SECONDARIES are only added to the log if compilations is
 * done with CFLAGS += -DLOG_SEC_IMU */
static uint8_t gyroPrimInUse =          SENSORS_BMI088;
static uint8_t accelPrimInUse =         SENSORS_BMI088;
//static uint8_t baroPrimInUse =          SENSORS_BMP280;

typedef struct {
  Axis3i16      value;
  Axis3i16*     bufStart;
  Axis3i16*     bufPtr;
  uint8_t       found : 1;
  uint8_t       ongoing : 1;
  uint8_t       bufIsFull : 1;
} BiasObj;

/* initialize necessary variables */
static struct bmi088_dev bmi088Dev;
static struct bmp3_dev   bmp388Dev;

static xQueueHandle accelPrimDataQueue;
static xQueueHandle gyroPrimDataQueue;
static xQueueHandle baroPrimDataQueue;
static xQueueHandle magPrimDataQueue;

static bool isInit = false;
static bool allSensorsAreCalibrated = false;
static sensorData_t sensors;

static int32_t varianceSampleTime;
static uint8_t sensorsAccLpfAttFactor;

static bool isBarometerPresent = false;
static bool isMagnetometerPresent = false;
static uint8_t baroMeasDelayMin = SENSORS_DELAY_BARO;

// Pre-calculated values for accelerometer alignment
float cosPitch;
float sinPitch;
float cosRoll;
float sinRoll;

static void sensorsDeviceInit(void);
static void sensorsTaskInit(void);
static void sensorsTask(void *param);
static void sensorsApplyBiasAndScale(Axis3f* scaled, Axis3i16* aligned,
                                     Axis3i16* bias, float scale);
static void sensorsScaleBaro(baro_t* baroScaled, float pressure,
                             float temperature);
static bool processGyroBias(BiasObj* bias);
static void processAccelBias(BiasObj* bias);

static void sensorsAccIIRLPFilter(Axis3i16* in, Axis3i16* out,
                                  Axis3i32* storedValues, int32_t attenuation);
static void sensorsAccAlignToGravity(Axis3f* in, Axis3f* out);
static void sensorsBiasReset(BiasObj* bias);
static void sensorsBiasMalloc(BiasObj* bias);
static void sensorsBiasFree(BiasObj* bias);
static void sensorsBiasBufPtrIncrement(BiasObj* bias);
static bstdr_ret_t bstdr_bmi088_comm_init(void);
static bstdr_ret_t bstdr_bmi088_burst_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
static bstdr_ret_t bstdr_bmi088_burst_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
static void bstdr_bmi088_ms_delay(uint32_t period);

void sensorsInit(void)
{
  if (isInit)
    {
      return;
    }

  sensorsDeviceInit();
  sensorsTaskInit();

  //isInit = true;
}

static void sensorsDeviceInit(void)
{
  if (isInit)
    return;

  bstdr_ret_t rslt;
  isBarometerPresent = false;

  // Wait for sensors to startup
  vTaskDelay(M2T(SENSORS_STARTUP_TIME_MS));

  bstdr_bmi088_comm_init();

  /* BMI088 */
  bmi088Dev.accel_id = BMI088_ACCEL_I2C_ADDR_PRIMARY;
  bmi088Dev.gyro_id = BMI088_GYRO_I2C_ADDR_SECONDARY;
  bmi088Dev.interface = BMI088_I2C_INTF;
  bmi088Dev.read = bstdr_bmi088_burst_read;
  bmi088Dev.write = bstdr_bmi088_burst_write;
  bmi088Dev.delay_ms = bstdr_bmi088_ms_delay;

  /* BMI088 GYRO */
  rslt = bmi088_gyro_init(&bmi088Dev); // initialize the device
  if (rslt == BSTDR_OK)
    {
      DEBUG_PRINT("BMI088 Gyro I2C connection [OK].\n");
      /* set power mode of gyro */
      bmi088Dev.gyro_cfg.power = BMI088_GYRO_PM_NORMAL;
      rslt |= bmi088_set_gyro_power_mode(&bmi088Dev);
      /* set bandwidth and range of gyro */
      bmi088Dev.gyro_cfg.bw = BMI088_GYRO_BW_116_ODR_1000_HZ;
      bmi088Dev.gyro_cfg.range = SENSORS_BMI088_GYRO_FS_CFG;
      bmi088Dev.gyro_cfg.odr = BMI088_GYRO_BW_116_ODR_1000_HZ;
      rslt |= bmi088_set_gyro_meas_conf(&bmi088Dev);

      bmi088Dev.delay_ms(50);
      struct bmi088_sensor_data gyr;
      rslt |= bmi088_get_gyro_data(&gyr, &bmi088Dev);
    }
  else
    {
#ifndef SENSORS_IGNORE_IMU_FAIL
      DEBUG_PRINT("BMI088 Gyro I2C connection [FAIL]\n");
      isInit = false;
#endif
    }

  /* BMI088 ACCEL */
  rslt |= bmi088_accel_switch_control(&bmi088Dev, BMI088_ACCEL_POWER_ENABLE);
  bmi088Dev.delay_ms(5);

  rslt = bmi088_accel_init(&bmi088Dev); // initialize the device
  if (rslt == BSTDR_OK)
    {
      DEBUG_PRINT("BMI088 Accel I2C connection [OK]\n");
      /* set power mode of accel */
      bmi088Dev.accel_cfg.power = BMI088_ACCEL_PM_ACTIVE;
      rslt |= bmi088_set_accel_power_mode(&bmi088Dev);
      bmi088Dev.delay_ms(10);

      /* set bandwidth and range of accel */
      bmi088Dev.accel_cfg.bw = BMI088_ACCEL_BW_OSR4;
      bmi088Dev.accel_cfg.range = SENSORS_BMI088_ACCEL_FS_CFG;
      bmi088Dev.accel_cfg.odr = BMI088_ACCEL_ODR_1600_HZ;
      rslt |= bmi088_set_accel_meas_conf(&bmi088Dev);

      struct bmi088_sensor_data acc;
      rslt |= bmi088_get_accel_data(&acc, &bmi088Dev);
    }
  else
    {
#ifndef SENSORS_IGNORE_IMU_FAIL
      DEBUG_PRINT("BMI088 Accel I2C connection [FAIL]\n");
      isInit = false;
#endif
    }

  /* BMP388 */
  bmp388Dev.dev_id = BMP3_I2C_ADDR_SEC;
  bmp388Dev.intf = BMP3_I2C_INTF;
  bmp388Dev.read = bstdr_bmi088_burst_read;
  bmp388Dev.write = bstdr_bmi088_burst_write;
  bmp388Dev.delay_ms = bstdr_bmi088_ms_delay;

  int i = 3;
  do {
    bmp388Dev.delay_ms(1);
    rslt = bmp3_init(&bmp388Dev);
    // consolePrintf(" %d", rslt);
  } while (rslt != BMP3_OK && i-- > 0);

  if (rslt == BMP3_OK)
  {
    isBarometerPresent = true;
    DEBUG_PRINT("BMP388 I2C connection [OK]\n");
    /* Used to select the settings user needs to change */
    uint16_t settings_sel;
    /* Select the pressure and temperature sensor to be enabled */
    bmp388Dev.settings.press_en = BMP3_ENABLE;
    bmp388Dev.settings.temp_en = BMP3_ENABLE;
    /* Select the output data rate and oversampling settings for pressure and temperature */
    bmp388Dev.settings.odr_filter.press_os = BMP3_OVERSAMPLING_8X;
    bmp388Dev.settings.odr_filter.temp_os = BMP3_NO_OVERSAMPLING;
    bmp388Dev.settings.odr_filter.odr = BMP3_ODR_50_HZ;
    bmp388Dev.settings.odr_filter.iir_filter = BMP3_IIR_FILTER_COEFF_3;
    /* Assign the settings which needs to be set in the sensor */
    settings_sel = BMP3_PRESS_EN_SEL | BMP3_TEMP_EN_SEL | BMP3_PRESS_OS_SEL | BMP3_TEMP_OS_SEL | BMP3_ODR_SEL | BMP3_IIR_FILTER_SEL;
    rslt = bmp3_set_sensor_settings(settings_sel, &bmp388Dev);

    /* Set the power mode to normal mode */
    bmp388Dev.settings.op_mode = BMP3_NORMAL_MODE;
    rslt = bmp3_set_op_mode(&bmp388Dev);


    bmp388Dev.delay_ms(20); // wait before first read out
    // read out data
    /* Variable used to select the sensor component */
    uint8_t sensor_comp;
    /* Variable used to store the compensated data */
    struct bmp3_data data;

    /* Sensor component selection */
    sensor_comp = BMP3_PRESS | BMP3_TEMP;
    /* Temperature and Pressure data are read and stored in the bmp3_data instance */
    rslt = bmp3_get_sensor_data(sensor_comp, &data, &bmp388Dev);

    /* Print the temperature and pressure data */
//    DEBUG_PRINT("BMP388 T:%0.2f  P:%0.2f\n",data.temperature, data.pressure/100.0f);
    baroMeasDelayMin = SENSORS_DELAY_BARO;
  }
  else
  {
#ifndef SENSORS_IGNORE_BAROMETER_FAIL
    DEBUG_PRINT("BMP388 I2C connection [FAIL]\n");
    isInit = false;
    return;
#endif
  }

  varianceSampleTime = -GYRO_MIN_BIAS_TIMEOUT_MS + 1;
  sensorsAccLpfAttFactor = IMU_ACC_IIR_LPF_ATT_FACTOR;

  cosPitch = cosf(configblockGetCalibPitch() * (float) M_PI / 180);
  sinPitch = sinf(configblockGetCalibPitch() * (float) M_PI / 180);
  cosRoll = cosf(configblockGetCalibRoll() * (float) M_PI / 180);
  sinRoll = sinf(configblockGetCalibRoll() * (float) M_PI / 180);

  isInit = true;
}

static void sensorsTaskInit(void)
{
  accelPrimDataQueue = xQueueCreate(1, sizeof(Axis3f));
  gyroPrimDataQueue = xQueueCreate(1, sizeof(Axis3f));
  magPrimDataQueue = xQueueCreate(1, sizeof(Axis3f));
  baroPrimDataQueue = xQueueCreate(1, sizeof(baro_t));

  xTaskCreate(sensorsTask, SENSORS_TASK_NAME, SENSORS_TASK_STACKSIZE,
              NULL, SENSORS_TASK_PRI, NULL);
}

static void sensorsGyroGet(Axis3i16* dataOut, uint8_t device) {
  bmi088_get_gyro_data((struct bmi088_sensor_data*)dataOut, &bmi088Dev);
}

static void sensorsAccelGet(Axis3i16* dataOut, uint8_t device) {
  bmi088_get_accel_data((struct bmi088_sensor_data*)dataOut, &bmi088Dev);
}

static void sensorsGyroCalibrate(BiasObj* gyro, uint8_t type) {
  if (gyro->found == 0)
    {
      if (gyro->ongoing == 0)
        {
          sensorsBiasMalloc(gyro);
        }
      /* write directly into buffer */
      sensorsGyroGet(gyro->bufPtr, type);
      /* FIXME: for sensor deck v1 realignment has to be added her */
      sensorsBiasBufPtrIncrement(gyro);

      if (gyro->bufIsFull == 1)
        {
          if (processGyroBias(gyro))
            sensorsBiasFree(gyro);
        }
    }
}

static void __attribute__((used))
sensorsAccelCalibrate(BiasObj* accel, BiasObj* gyro, uint8_t type) {
  if (accel->found == 0)
    {
      if (accel->ongoing == 0)
        {
          sensorsBiasMalloc(accel);
        }
      /* write directly into buffer */
      sensorsAccelGet(accel->bufPtr, type);
      /* FIXME: for sensor deck v1 realignment has to be added her */
      sensorsBiasBufPtrIncrement(accel);
      if ( (accel->bufIsFull == 1) && (gyro->found == 1) )
        {
          processAccelBias(accel);
          switch(type) {
            case SENSORS_BMI160:
              accel->value.z -= SENSORS_BMI160_1G_IN_LSB;
              break;
            case SENSORS_BMI055:
              accel->value.z -= SENSORS_BMI055_1G_IN_LSB;
              break;
            case SENSORS_BMI088:
              accel->value.z -= SENSORS_BMI088_1G_IN_LSB;
              break;
          }
          sensorsBiasFree(accel);
        }
    }
}

static void sensorsTask(void *param)
{
  systemWaitStart();

  uint32_t lastWakeTime = xTaskGetTickCount();
  static BiasObj bmi088GyroBias;
#ifdef SENSORS_TAKE_ACCEL_BIAS
  static BiasObj bmi088AccelBias;
#endif
  Axis3i16 gyroPrim;
  Axis3i16 accelPrim;
  Axis3f accelPrimScaled;
  Axis3i16 accelPrimLPF;
  Axis3i32 accelPrimStoredFilterValues;
  /* wait an additional second the keep bus free
   * this is only required by the z-ranger, since the
   * configuration will be done after system start-up */
  //vTaskDelayUntil(&lastWakeTime, M2T(1500));
  while (1)
    {
      vTaskDelayUntil(&lastWakeTime, F2T(SENSORS_READ_RATE_HZ));
      /* calibrate if necessary */
      if (!allSensorsAreCalibrated)
      {
        if (!bmi088GyroBias.found)
        {
          sensorsGyroCalibrate(&bmi088GyroBias, SENSORS_BMI088);
  #ifdef SENSORS_TAKE_ACCEL_BIAS
          sensorsAccelCalibrate(&bmi088AccelBias,
                                &bmi088GyroBias, SENSORS_BMI088);
  #endif
        }

        if (bmi088GyroBias.found
  #ifdef SENSORS_TAKE_ACCEL_BIAS
            && bmi088AccelBias.found
  #endif
        )
        {
          // soundSetEffect(SND_CALIB);
          DEBUG_PRINT("Sensor calibration [OK].\n");
          ledseqRun(SYS_LED, seq_calibrated);
          allSensorsAreCalibrated = true;
        }
      }
    else
    {
      /* get data from chosen sensors */
      sensorsGyroGet(&gyroPrim, gyroPrimInUse);
      sensorsAccelGet(&accelPrim, accelPrimInUse);
      sensorsApplyBiasAndScale(&sensors.gyro, &gyroPrim,
                               &bmi088GyroBias.value,
                               SENSORS_BMI088_DEG_PER_LSB_CFG);

      sensorsAccIIRLPFilter(&accelPrim, &accelPrimLPF,
                            &accelPrimStoredFilterValues,
                            (int32_t)sensorsAccLpfAttFactor);
      sensorsApplyBiasAndScale(&accelPrimScaled, &accelPrimLPF,
                               &bmi088AccelBias.value,
                               SENSORS_BMI088_G_PER_LSB_CFG);

      sensorsAccAlignToGravity(&accelPrimScaled, &sensors.acc);

      }
      if (isBarometerPresent)
        {
          static uint8_t baroMeasDelay = SENSORS_DELAY_BARO;
          if (--baroMeasDelay == 0)
          {
            uint8_t sensor_comp = BMP3_PRESS | BMP3_TEMP;
            struct bmp3_data data;
            baro_t* baro388 = &sensors.baro;
            /* Temperature and Pressure data are read and stored in the bmp3_data instance */
            bmp3_get_sensor_data(sensor_comp, &data, &bmp388Dev);
            sensorsScaleBaro(baro388, data.pressure, data.temperature);
            baroMeasDelay = baroMeasDelayMin;
          }
        }
      /* ensure all queues are populated at the same time */
      vTaskSuspendAll();
      xQueueOverwrite(accelPrimDataQueue, &sensors.acc);
      xQueueOverwrite(gyroPrimDataQueue, &sensors.gyro);

      if (isBarometerPresent)
      {
        xQueueOverwrite(baroPrimDataQueue, &sensors.baro);
      }

      if (isMagnetometerPresent)
      {
        xQueueOverwrite(magPrimDataQueue, &sensors.mag);
      }
      xTaskResumeAll();
    }
}

static void sensorsBiasMalloc(BiasObj* bias)
{
  /* allocate memory for buffer */
  bias->bufStart =
      pvPortMalloc(SENSORS_NBR_OF_BIAS_SAMPLES * sizeof(Axis3i16));
  bias->bufPtr = bias->bufStart;
  /* set ongoing bit */
  bias->ongoing = 1;
}

static void __attribute__((used)) sensorsBiasReset(BiasObj* bias)
{
  /* unset bias found and buffer full status bits */
  bias->found = 0;
  bias->bufIsFull = 0;
  /* set bufPtr to start to ensure the buffer has to be refilled
   * completly before bufferIsFull Status bit will be set */
  bias->bufPtr = bias->bufStart;
  /* clear any exisiting bias value */
  bias->value.x = 0;
  bias->value.y = 0;
  bias->value.z = 0;
  allSensorsAreCalibrated = false;
}

static void sensorsBiasFree(BiasObj* bias)
{
  /* unset buffer is full */
  bias->bufIsFull = 0;
  /* free buffer memory */
  vPortFree(bias->bufStart);
  bias->bufStart = NULL;
  bias->bufPtr = NULL;
}

/**
 * Adds a new value to the variance buffer and if it is full
 * replaces the oldest one. Thus a circular buffer.
 */
static void sensorsBiasBufPtrIncrement(BiasObj* bias)
{
  bias->bufPtr++;
  if (bias->bufPtr >= bias->bufStart+SENSORS_NBR_OF_BIAS_SAMPLES)
    {
      bias->bufPtr = bias->bufStart;
      bias->bufIsFull = 1;
    }
}

/**
 * Calculates the mean for the bias buffer.
 */
static void calcMean(BiasObj* bias, Axis3f* mean) {
  Axis3i16* elem;
  int64_t sum[GYRO_NBR_OF_AXES] = {0};

  for (elem = bias->bufStart;
       elem != (bias->bufStart+SENSORS_NBR_OF_BIAS_SAMPLES); elem++)
    {
      sum[0] += elem->x;
      sum[1] += elem->y;
      sum[2] += elem->z;
    }

  mean->x = (float)sum[0] / SENSORS_NBR_OF_BIAS_SAMPLES;
  mean->y = (float)sum[1] / SENSORS_NBR_OF_BIAS_SAMPLES;
  mean->z = (float)sum[2] / SENSORS_NBR_OF_BIAS_SAMPLES;
}

/**
 * Calculates the variance and mean for the bias buffer.
 */
static void calcVarianceAndMean(BiasObj* bias, Axis3f* variance, Axis3f* mean)
{
  Axis3i16* elem;
  int64_t sumSquared[GYRO_NBR_OF_AXES] = {0};

  for (elem = bias->bufStart;
      elem != (bias->bufStart+SENSORS_NBR_OF_BIAS_SAMPLES); elem++)
    {
      sumSquared[0] += elem->x * elem->x;
      sumSquared[1] += elem->y * elem->y;
      sumSquared[2] += elem->z * elem->z;
    }
  calcMean(bias, mean);

  variance->x = fabs(sumSquared[0] / SENSORS_NBR_OF_BIAS_SAMPLES
                     - mean->x * mean->x);
  variance->y = fabs(sumSquared[1] / SENSORS_NBR_OF_BIAS_SAMPLES
                     - mean->y * mean->y);
  variance->z = fabs(sumSquared[2] / SENSORS_NBR_OF_BIAS_SAMPLES
                     - mean->z * mean->z);
}

/**
 * Checks if the variances is below the predefined thresholds.
 * The bias value should have been added before calling this.
 * @param bias  The bias object
 */
static bool processGyroBias(BiasObj* bias)
{
  Axis3f mean, variance;
  calcVarianceAndMean(bias, &variance, &mean);

  if (variance.x < GYRO_VARIANCE_THRESHOLD_X
      && variance.y < GYRO_VARIANCE_THRESHOLD_Y
      && variance.z < GYRO_VARIANCE_THRESHOLD_Z
      && (varianceSampleTime + GYRO_MIN_BIAS_TIMEOUT_MS < xTaskGetTickCount()))
    {
      varianceSampleTime = xTaskGetTickCount();
      bias->value.x = (int16_t)(mean.x + 0.5f);
      bias->value.y = (int16_t)(mean.y + 0.5f);
      bias->value.z = (int16_t)(mean.z + 0.5f);
      bias->found = 1;
      return true;
    }
  return false;
}

static void processAccelBias(BiasObj* bias)
{
  Axis3f mean;
  calcMean(bias, &mean);

  varianceSampleTime = xTaskGetTickCount();
  bias->value.x = (int16_t)(mean.x + 0.5f);
  bias->value.y = (int16_t)(mean.y + 0.5f);
  bias->value.z = (int16_t)(mean.z + 0.5f);
  bias->found = 1;
}
static void sensorsApplyBiasAndScale(Axis3f* scaled, Axis3i16* aligned,
                                     Axis3i16* bias, float scale) {
  scaled->x = ((float)aligned->x - (float)bias->x) * scale;
  scaled->y = ((float)aligned->y - (float)bias->y) * scale;
  scaled->z = ((float)aligned->z - (float)bias->z) * scale;
}

static void sensorsScaleBaro(baro_t* baroScaled, float pressure,
                             float temperature) {
  baroScaled->pressure = pressure*0.01f;
  baroScaled->temperature = temperature;
  baroScaled->asl = ((powf((1015.7f / baroScaled->pressure), 0.1902630958f)
      - 1.0f) * (25.0f + 273.15f)) / 0.0065f;
}

bool sensorsReadGyro(Axis3f *gyro)
{
  return (pdTRUE == xQueueReceive(gyroPrimDataQueue, gyro, 0));
}

bool sensorsReadAcc(Axis3f *acc)
{
  return (pdTRUE == xQueueReceive(accelPrimDataQueue, acc, 0));
}

bool sensorsReadMag(Axis3f *mag)
{
  return (pdTRUE == xQueueReceive(magPrimDataQueue, mag, 0));
}

bool sensorsReadBaro(baro_t *baro)
{
  return (pdTRUE == xQueueReceive(baroPrimDataQueue, baro, 0));
}

void sensorsAcquire(sensorData_t *sensors, const uint32_t tick)
{
  sensorsReadGyro(&sensors->gyro);
  sensorsReadAcc(&sensors->acc);
  sensorsReadMag(&sensors->mag);
  sensorsReadBaro(&sensors->baro);
  zRangerReadRange(&sensors->zrange, tick);
}

bool sensorsAreCalibrated()
{
  return allSensorsAreCalibrated;
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

bool sensorsHasBarometer(void)
{
  return isBarometerPresent;
}

bool sensorsHasMangnetometer(void)
{
  return isMagnetometerPresent;
}

static void sensorsAccIIRLPFilter(Axis3i16* in, Axis3i16* out,
                                  Axis3i32* storedValues, int32_t attenuation)
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

// Communication routines

/*!
 * @brief Communication initialization
 *
 * This is optional. Depends on the system you are using
 *
 * @return Zero if successful, otherwise an error code
 */
static bstdr_ret_t bstdr_bmi088_comm_init(void)
{
	/**< Communication initialization --Optional!*/
	i2cdevInit(I2C3_DEV);
	return (bstdr_ret_t)0;
}


/*!
 * @brief Generic burst read
 *
 * @param [out] dev_id I2C address, SPI chip select or user desired identifier
 *
 * @return Zero if successful, otherwise an error code
 */
static bstdr_ret_t bstdr_bmi088_burst_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
	/**< Burst read code comes here */
	if (i2cdevRead(I2C3_DEV, dev_id, reg_addr, (uint16_t) len, reg_data))
	{
	  return BSTDR_OK;
	}
	else
	{
    return BSTDR_E_CON_ERROR;
	}
}


/*!
 * @brief Generic burst write
 *
 * @param [out] dev_id I2C address, SPI chip select or user desired identifier
 *
 * @return Zero if successful, otherwise an error code
 */
static bstdr_ret_t bstdr_bmi088_burst_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
	/**< Burst write code comes here */
	if (i2cdevWrite(I2C3_DEV, dev_id,reg_addr,(uint16_t) len, reg_data))
  {
    return BSTDR_OK;
  }
  else
  {
    return BSTDR_E_CON_ERROR;
  }
}


/*!
 * @brief Generic burst read
 *
 * @param [in] period Delay period in milliseconds
 *
 * @return None
 */
static void bstdr_bmi088_ms_delay(uint32_t period)
{
	/**< Delay code comes */
	vTaskDelay(M2T(period)); // Delay a while to let the device stabilize
}



PARAM_GROUP_START(imu_sensors)
PARAM_ADD(PARAM_UINT8, BoschGyrSel, &gyroPrimInUse)
PARAM_ADD(PARAM_UINT8, BoschAccSel, &accelPrimInUse)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, BMM150, &isMagnetometerPresent)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, BMP285, &isBarometerPresent)
PARAM_GROUP_STOP(imu_sensors)
