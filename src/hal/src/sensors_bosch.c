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
 * sensors_bosch.c
 */

/*********************************************
 * IMPORTANT NOTE relating to sensor_deck v1 *
 * Don't miss to align the axes correctly!	 *
 *********************************************/

#define DEBUG_MODULE "IMU"

#include <math.h>
#include "stm32fxxx.h"

#include "sensors.h"
#include "imu.h"
// BST Drivers
#include "bmi055.h"
#include "bmi160.h"
#include "bmi160_filter.h"
#include "bmp285.h"
#include "bmm150.h"
#include "bma455.h"
#include "bstdr_comm_support.h"

#include "vl53l0x.h"

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
//#define SENSORS_ENABLE_BMI160
#define SENSORS_ENABLE_BMM150
//#define SENSORS_ENABLE_BMA455
#define SENSORS_ENABLE_PRESSURE_BMP285

typedef enum
{
	SENSORS_BMI055 = 1,
	SENSORS_BMI160 = 2
} sensorsImuTypes_e;

// holds the used IMU type
static sensorsImuTypes_e usedImuType = SENSORS_BMI055;

#define SENSORS_READ_RATE_HZ		1000
#define SENSORS_STARTUP_TIME_MS		1000
#define SENSORS_READ_BARO_HZ		50
#define SENSORS_READ_MAG_HZ			20
#define SENSORS_REQ_TICKS_BARO		F2T((SENSORS_READ_RATE_HZ/SENSORS_READ_BARO_HZ))
#define SENSORS_REQ_TICKS_MAG		F2T((SENSORS_READ_RATE_HZ/SENSORS_READ_MAG_HZ))

#ifdef SENSORS_ENABLE_BMI160
#define SENSORS_BMI160_ACCEL_CFG			16
#define SENSORS_BMI160_ACCEL_FS_CFG			BMI160_ACCEL_RANGE_16G
#define SENSORS_BMI160_G_PER_LSB_CFG		(2.0f * SENSORS_BMI160_ACCEL_CFG) / 65536.0f
#define SENSORS_BMI160_1G_IN_LSB		    65536 / SENSORS_BMI160_ACCEL_CFG / 2

#define SENSORS_BMI160_GYRO_FS_CFG			BMI160_GYRO_RANGE_2000_DPS
#define SENSORS_BMI160_DEG_PER_LSB_CFG		(2.0f *2000.0f) / 65536.0f
#endif

#ifdef SENSORS_ENABLE_BMI055
#define SENSORS_BMI055_ACCEL_CFG			16
#define SENSORS_BMI055_ACCEL_FS_CFG			BMI055_ACCEL_RANGE_16G
#define SENSORS_BMI055_G_PER_LSB_CFG		(2.0f * SENSORS_BMI055_ACCEL_CFG) / 4096.0f
#define SENSORS_BMI055_1G_IN_LSB		    4096 / SENSORS_BMI055_ACCEL_CFG / 2

#define SENSORS_BMI055_GYRO_FS_CFG			BMI055_GYRO_RANGE_2000_DPS
#define SENSORS_BMI055_DEG_PER_LSB_CFG		(2.0f *2000.0f) / 65536.0f
#endif

#define SENSORS_VARIANCE_MAN_TEST_TIMEOUT M2T(1000) // Timeout in ms#define SENSORS_MAN_TEST_LEVEL_MAX        5.0f      // Max degrees off#define GYRO_NBR_OF_AXES            3
#define GYRO_MIN_BIAS_TIMEOUT_MS    M2T(1*1000)

#define SENSORS_TAKE_ACCEL_BIAS

// Number of samples used in variance calculation. Changing this effects the threshold
#define SENSORS_NBR_OF_BIAS_SAMPLES  512

// Variance threshold to take zero bias for gyro
#define GYRO_VARIANCE_BASE        2000
#define GYRO_VARIANCE_THRESHOLD_X (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Y (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Z (GYRO_VARIANCE_BASE)

typedef struct BiasObj_s
{
	Axis3f bias;
	bool isBiasValueFound;
	bool isBufferFilled;
	Axis3i16* bufHead;
	Axis3i16 buffer[SENSORS_NBR_OF_BIAS_SAMPLES];
} BiasObj_t;

static xQueueHandle accelerometerDataQueue;
static xQueueHandle gyroDataQueue;
#ifdef LOG_SEC_IMU
static xQueueHandle accelerometerSpareDataQueue;
static xQueueHandle gyroSpareDataQueue;
#endif
static xQueueHandle barometerDataQueue;
static xQueueHandle magnetometerDataQueue;

static bool isInit = false;
static sensorData_t sensors;

static int32_t varianceSampleTime;
static Axis3i16 accelLPF;
static uint8_t sensorsAccLpfAttFactor;

static bool isBarometerPresent = false;
static bool isMagnetometerPresent = false;
#ifdef SENSORS_ENABLE_BMA455
static bool isAccelerometerPresent = false;
#endif
#ifdef SENSORS_ENABLE_BMI160
static BiasObj_t gyroBiasBmi160;
static BiasObj_t accelBiasBmi160;
static Axis3i16 gyroBmi160;
static Axis3i16 accelBmi160;
static struct bmi160_dev bmi160Dev;
struct iir_filter_1d iir4Accx;
struct iir_filter_1d iir4Accy;
struct iir_filter_1d iir4Accz;
#endif

#ifdef SENSORS_ENABLE_BMI055
static BiasObj_t gyroBiasBmi055;
static BiasObj_t accelBiasBmi055;
static Axis3i16 gyroBmi055;
static Axis3i16 accelBmi055;
static Axis3i32 accelStoredFilterValues;
static struct bmi055_dev bmi055Dev;
#endif

//
#ifdef SENSORS_ENABLE_PRESSURE_BMP285
static struct bmp285_t bmp285Dev;
#endif

#ifdef SENSORS_ENABLE_BMM150
struct bmm150_dev bmm150Dev;
#endif

#ifdef SENSORS_ENABLE_BMA455
struct bma4_dev bma455Dev;
#endif

// Pre-calculated values for accelerometer alignment
float cosPitch;
float sinPitch;
float cosRoll;
float sinRoll;

static void sensorsDeviceInit(void);
static void sensorsTaskInit(void);
static void sensorsTask(void *param);
static void sensorsBiasInit(BiasObj_t* bias);
static void sensorsRead(Axis3f* gyroUsed, Axis3f* accUsed, baro_t *baroUsed, Axis3f* magUsed
#ifdef LOG_SEC_IMU
		, Axis3f* gyroSpare, Axis3f* accSpare
#endif
#ifdef LOG_SEC_BARO
		, baro_t *baroSpare
#endif
		);
static void sensorsCalculateBiasMean(BiasObj_t* bias, Axis3i32* meanOut);
static void sensorsCalculateVarianceAndMean(BiasObj_t* bias, Axis3f* varOut, Axis3f* meanOut);
static bool sensorsFindBiasValue(BiasObj_t* bias);
static void sensorsAddBiasValue(BiasObj_t* bias, Axis3i16* dVal);
static void sensorsAccIIRLPFilter(Axis3i16* in, Axis3i16* out, Axis3i32* storedValues, int32_t attenuation);
static void sensorsAccAlignToGravity(Axis3f* in, Axis3f* out);

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

static void sensorsDeviceInit(void)
{
	if (isInit)
		return;

	bstdr_ret_t rslt;
	isBarometerPresent = false;

	// Wait for sensors to startup
	vTaskDelay(M2T(SENSORS_STARTUP_TIME_MS));

	// BMI160 Initialization
#ifdef SENSORS_ENABLE_BMI160

	// initialize filters
	bmi160_lfilter_init(&iir4Accx);
	bmi160_lfilter_init(&iir4Accy);
	bmi160_lfilter_init(&iir4Accz);

	bmi160Dev.read = (bmi160_com_fptr_t)bstdr_burst_read;  // assign bus read function for bst devices
	bmi160Dev.write = (bmi160_com_fptr_t)bstdr_burst_write;  // assign bus write function for bst devices
	bmi160Dev.delay_ms = (bmi160_delay_fptr_t)bstdr_ms_delay; // assign delay function
	bmi160Dev.id = BMI160_I2C_ADDR+1;  // I2C device address

	rslt = bmi160_init(&bmi160Dev); // initialize the device
	if (rslt == BSTDR_OK)
	{
		DEBUG_PRINT("BMI160 I2C connection [OK].\n");
		/* Select the Output data rate, range of accelerometer sensor ~92Hz BW by OSR4 @ODR=800Hz */
		bmi160Dev.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
		bmi160Dev.accel_cfg.range = SENSORS_BMI160_ACCEL_FS_CFG;
		bmi160Dev.accel_cfg.bw = BMI160_ACCEL_BW_OSR4_AVG1;
		/* Select the power mode of accelerometer sensor */
		bmi160Dev.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

		/* Select the Output data rate, range of Gyroscope sensor ~92Hz BW by OSR4 @ODR=800Hz */
		bmi160Dev.gyro_cfg.odr = BMI160_GYRO_ODR_800HZ;
		bmi160Dev.gyro_cfg.range = SENSORS_BMI160_GYRO_FS_CFG;
		bmi160Dev.gyro_cfg.bw = BMI160_GYRO_BW_OSR4_MODE;

		/* Select the power mode of Gyroscope sensor */
		bmi160Dev.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

		/* Set the sensor configuration */
		rslt |= bmi160_set_sens_conf(&bmi160Dev);
		bmi160Dev.delay_ms(50);

		struct bmi160_sensor_data acc, gyr;
		rslt |= bmi160_get_sensor_data(BMI160_BOTH_ACCEL_AND_GYRO, &acc, &gyr, &bmi160Dev);
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
	bmi055Dev.accel_id = BMI055_ACCEL_I2C_ADDR;
	bmi055Dev.gyro_id = BMI055_GYRO_I2C_ADDR;
	bmi055Dev.interface = BMI055_I2C_INTF;
	bmi055Dev.read = (bmi055_com_fptr_t)bstdr_burst_read;
	bmi055Dev.write = (bmi055_com_fptr_t)bstdr_burst_write;
	bmi055Dev.delay_ms = (bmi055_delay_fptr_t)bstdr_ms_delay;

	rslt = bmi055_accel_init(&bmi055Dev); // initialize the device
	if (rslt == BSTDR_OK)
	{
		DEBUG_PRINT("BMI055 Accel I2C connection [OK].\n");
		/* set power mode of accel */
		bmi055Dev.accel_cfg.power = BMI055_ACCEL_PM_NORMAL;
		rslt |= bmi055_set_accel_power_mode(&bmi055Dev);
		/* set bandwidth and range of accel */
		bmi055Dev.accel_cfg.bw = BMI055_ACCEL_BW_125_HZ;
		bmi055Dev.accel_cfg.range = SENSORS_BMI055_ACCEL_FS_CFG;
		rslt |= bmi055_set_accel_sensor_config(CONFIG_ALL, &bmi055Dev);

		bmi055Dev.delay_ms(10);
		struct bmi055_sensor_data acc;
		rslt |= bmi055_get_accel_data(&acc, &bmi055Dev);
	}
	else
	{
		DEBUG_PRINT("BMI055 Accel I2C connection [FAIL].\n");
	}

	rslt = bmi055_gyro_init(&bmi055Dev); // initialize the device
	if (rslt == BSTDR_OK)
	{
		DEBUG_PRINT("BMI055 Gyro I2C connection [OK].\n");
		/* set power mode of gyro */
		bmi055Dev.gyro_cfg.power = BMI055_GYRO_PM_NORMAL;
		rslt |= bmi055_set_gyro_power_mode(&bmi055Dev);
		/* set bandwidth and range of gyro */
		bmi055Dev.gyro_cfg.bw = BMI055_GYRO_BW_116_HZ;
		bmi055Dev.gyro_cfg.range = SENSORS_BMI055_GYRO_FS_CFG;
		rslt |= bmi055_set_gyro_sensor_config(CONFIG_ALL, &bmi055Dev);

		bmi055Dev.delay_ms(50);
		struct bmi055_sensor_data gyr;
		rslt |= bmi055_get_gyro_data(&gyr, &bmi055Dev);
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

	// Bosch Sensortec Sensor BMP285 connection check
#ifdef SENSORS_ENABLE_PRESSURE_BMP285
	rslt = BSTDR_E_GEN_ERROR;

	bmp285Dev.bus_read = bstdr_burst_read;	// assign bus read function for bst devices
	bmp285Dev.bus_write = bstdr_burst_write;	// assign bus write function for bst devices
	bmp285Dev.delay_ms = bstdr_ms_delay;	// assign delay function
	bmp285Dev.dev_addr = BMP285_I2C_ADDRESS1;
	rslt = bmp285_init(&bmp285Dev); // initialize the device
	if (rslt == BSTDR_OK)
	{
		isBarometerPresent = true;
		DEBUG_PRINT("BME285 I2C connection [OK].\n");
		bmp285_set_filter(BMP285_FILTER_COEFF_2);
		bmp285_set_work_mode(BMP285_STANDARD_RESOLUTION_MODE);
		bmp285_set_standby_durn(BMP285_STANDBY_TIME_1_MS);
		bmp285_set_power_mode(BMP285_NORMAL_MODE);
		bmp285Dev.delay_ms(20);	// wait before first read out
		// read out data
		int32_t v_temp_s32;
		uint32_t v_pres_u32;
		bmp285_read_pressure_temperature(&v_pres_u32, &v_temp_s32);
	}
#endif

	// Bosch Sensortec Sensor BMM150 connection check
#ifdef SENSORS_ENABLE_BMM150
	rslt = BSTDR_OK;

	/* Sensor interface over I2C */
	bmm150Dev.id = BMM150_DEFAULT_I2C_ADDRESS;
	bmm150Dev.interface = BMM150_I2C_INTF;
	bmm150Dev.read = (bmm150_com_fptr_t)bstdr_burst_read;
	bmm150Dev.write = (bmm150_com_fptr_t)bstdr_burst_write;
	bmm150Dev.delay_ms = bstdr_ms_delay;

	rslt |= bmm150_init(&bmm150Dev);
	bmm150Dev.settings.pwr_mode = BMM150_NORMAL_MODE;
	rslt |= bmm150_set_op_mode(&bmm150Dev);
	bmm150Dev.settings.preset_mode = BMM150_PRESETMODE_HIGHACCURACY;
	rslt |= bmm150_set_presetmode(&bmm150Dev);

	if (rslt == BMM150_OK){
		DEBUG_PRINT("BMM150 I2C connection [OK].\n");
		isMagnetometerPresent = true;
	}
#endif

	// Bosch Sensortec Sensor BMA455 connection check
#ifdef SENSORS_ENABLE_BMA455
	uint16_t bma4_rslt = BSTDR_E_GEN_ERROR;

	/* Modify the parameters */
	bma455Dev.dev_addr = BMA4_I2C_ADDR_SECONDARY;
	bma455Dev.interface = BMA4_I2C_INTERFACE;
	bma455Dev.bus_read = (bma4_com_fptr_t)bstdr_burst_read;
	bma455Dev.bus_write = (bma4_com_fptr_t)bstdr_burst_write;
	bma455Dev.delay = bstdr_ms_delay;
	bma455Dev.read_write_len = 8;

	/* Initialize the instance */
	bma4_rslt = bma455_init(&bma455Dev);
	if (bma4_rslt == BMA4_OK){
		DEBUG_PRINT("BMA455 I2C connection [OK].\n");
		isAccelerometerPresent = true;
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

static void sensorsTaskInit(void)
{
	accelerometerDataQueue = xQueueCreate(1, sizeof(Axis3f));
	gyroDataQueue = xQueueCreate(1, sizeof(Axis3f));
#ifdef LOG_SEC_IMU
	accelerometerSpareDataQueue = xQueueCreate(1, sizeof(Axis3f));
	gyroSpareDataQueue = xQueueCreate(1, sizeof(Axis3f));
#endif
	magnetometerDataQueue = xQueueCreate(1, sizeof(Axis3f));
	barometerDataQueue = xQueueCreate(1, sizeof(baro_t));

	xTaskCreate(sensorsTask, SENSORS_TASK_NAME, SENSORS_TASK_STACKSIZE, NULL, SENSORS_TASK_PRI, NULL);
}

static void sensorsTask(void *param)
{
	systemWaitStart();

	uint32_t lastWakeTime = xTaskGetTickCount();
	/* wait an additional second the keep bus free
	 * this is only required by the z-ranger, since the
	 * configuration will be done after system start-up */
	//vTaskDelayUntil(&lastWakeTime, M2T(1500));
	while (1)
	{
		vTaskDelayUntil(&lastWakeTime, F2T(SENSORS_READ_RATE_HZ));

		sensorsRead(&sensors.gyro, &sensors.acc, &sensors.baro, &sensors.mag
#ifdef LOG_SEC_IMU
				, &sensors.gyroSpare, &sensors.accSpare
#endif
				);

		// ensure all queues are populated at the same time
		vTaskSuspendAll();
		xQueueOverwrite(accelerometerDataQueue, &sensors.acc);
		xQueueOverwrite(gyroDataQueue, &sensors.gyro);

#ifdef LOG_SEC_IMU
		xQueueOverwrite(gyroSpareDataQueue, &sensors.gyroSpare);
		xQueueOverwrite(accelerometerSpareDataQueue, &sensors.accSpare);
#endif

		if (isBarometerPresent)
		{
			xQueueOverwrite(barometerDataQueue, &sensors.baro);
		}

		if (isMagnetometerPresent)
		{
			xQueueOverwrite(magnetometerDataQueue, &sensors.mag);
		}
		xTaskResumeAll();
	}
}

static void sensorsRead(Axis3f* gyroUsed, Axis3f* accUsed, baro_t *baroUsed, Axis3f* magUsed
#ifdef LOG_SEC_IMU
		, Axis3f* gyroSpare, Axis3f* accSpare
#endif
		)
{
	static Axis3f accScaled;

	// read out the data
#ifdef SENSORS_ENABLE_BMI160
		static struct bmi160_sensor_data bmi160acc;
		static struct bmi160_sensor_data bmi160gyr;
		bmi160_get_sensor_data(BMI160_BOTH_ACCEL_AND_GYRO, &bmi160acc, &bmi160gyr, &bmi160Dev);
		// re-align the axes
		accelBmi160.x = bmi160acc.x;
		accelBmi160.y = bmi160acc.y;
		accelBmi160.z = bmi160acc.z;
		gyroBmi160.x = bmi160gyr.x;
		gyroBmi160.y = bmi160gyr.y;
		gyroBmi160.z = bmi160gyr.z;

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
				// soundSetEffect(SND_CALIB);
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
			accelBiasBmi160.bias.z = mean.z - SENSORS_BMI160_1G_IN_LSB;
			accelBiasBmi160.isBiasValueFound = true;
		}
#endif

		//sensorsAccIIRLPFilter(&accelBmi160, &accelLPF, &accelStoredFilterValues, (int32_t) sensorsAccLpfAttFactor);
		// software filter to improve performance

		bmi160_lfilter(&iir4Accx,accelBmi160.x);
		bmi160_lfilter(&iir4Accy,accelBmi160.y);
		bmi160_lfilter(&iir4Accz,accelBmi160.z);

		accelLPF.x = iir4Accx.out[0];
		accelLPF.y = iir4Accy.out[0];
		accelLPF.z = iir4Accz.out[0];

		static Axis3f* gyro160;
		static Axis3f* acc160;
#ifndef LOG_SEC_IMU
		gyro160 = gyroUsed;
		acc160 = accUsed;
#else
		gyro160 = ( (usedImuType == SENSORS_BMI160) ? gyroUsed : gyroSpare);
		acc160 = ( (usedImuType == SENSORS_BMI160) ? accUsed : accSpare);
#endif
		gyro160->x = ((float) gyroBmi160.x - gyroBiasBmi160.bias.x) * SENSORS_BMI160_DEG_PER_LSB_CFG;
		gyro160->y = ((float) gyroBmi160.y - gyroBiasBmi160.bias.y) * SENSORS_BMI160_DEG_PER_LSB_CFG;
		gyro160->z = ((float) gyroBmi160.z - gyroBiasBmi160.bias.z) * SENSORS_BMI160_DEG_PER_LSB_CFG;
		accScaled.x = (accelLPF.x - accelBiasBmi160.bias.x) * SENSORS_BMI160_G_PER_LSB_CFG;
		accScaled.y = (accelLPF.y - accelBiasBmi160.bias.y) * SENSORS_BMI160_G_PER_LSB_CFG;
		accScaled.z = (accelLPF.z - accelBiasBmi160.bias.z) * SENSORS_BMI160_G_PER_LSB_CFG;
		sensorsAccAlignToGravity(&accScaled, acc160);
#endif

#ifdef SENSORS_ENABLE_BMI055
		static struct bmi055_sensor_data bmi055acc;
		static struct bmi055_sensor_data bmi055gyr;
		bmi055_get_accel_data(&bmi055acc, &bmi055Dev);
		bmi055_get_gyro_data(&bmi055gyr, &bmi055Dev);
		// re-align the axes
		accelBmi055.x = bmi055acc.x;
		accelBmi055.y = bmi055acc.y;
		accelBmi055.z = bmi055acc.z;
		gyroBmi055.x = bmi055gyr.x;
		gyroBmi055.y = bmi055gyr.y;
		gyroBmi055.z = bmi055gyr.z;
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
				// soundSetEffect(SND_CALIB);
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

		static Axis3f* gyro055;
		static Axis3f* acc055;
#ifndef LOG_SEC_IMU
		gyro055 = gyroUsed;
		acc055 = accUsed;
#else
		gyro055 = ( (usedImuType == SENSORS_BMI055) ? gyroUsed : gyroSpare);
		acc055 = ( (usedImuType == SENSORS_BMI055) ? accUsed : accSpare);
#endif

		sensorsAccIIRLPFilter(&accelBmi055, &accelLPF, &accelStoredFilterValues, (int32_t) sensorsAccLpfAttFactor);
		gyro055->x = ((float) gyroBmi055.x - gyroBiasBmi055.bias.x) * SENSORS_BMI055_DEG_PER_LSB_CFG;
		gyro055->y = ((float) gyroBmi055.y - gyroBiasBmi055.bias.y) * SENSORS_BMI055_DEG_PER_LSB_CFG;
		gyro055->z = ((float) gyroBmi055.z - gyroBiasBmi055.bias.z) * SENSORS_BMI055_DEG_PER_LSB_CFG;
		accScaled.x = (accelLPF.x - accelBiasBmi055.bias.x) * SENSORS_BMI055_G_PER_LSB_CFG;
		accScaled.y = (accelLPF.y - accelBiasBmi055.bias.y) * SENSORS_BMI055_G_PER_LSB_CFG;
		accScaled.z = (accelLPF.z - accelBiasBmi055.bias.z) * SENSORS_BMI055_G_PER_LSB_CFG;
		sensorsAccAlignToGravity(&accScaled, acc055);
#endif

#if SENSORS_DISABLE_IMUS
	}
#endif

#ifdef SENSORS_ENABLE_BMM150
	static uint8_t magMeasDelay = SENSORS_REQ_TICKS_MAG;

	if (--magMeasDelay == 0){
		bmm150_read_mag_data(&bmm150Dev);
		magUsed->x = bmm150Dev.data.x;
		magUsed->y = bmm150Dev.data.y;
		magUsed->z = bmm150Dev.data.z;
		magMeasDelay = SENSORS_REQ_TICKS_MAG;
	}
#endif

#ifdef SENSORS_ENABLE_PRESSURE_BMP285
	static int32_t v_temp_s32;
	static uint32_t v_pres_u32;
	static uint8_t baroMeasDelay = SENSORS_REQ_TICKS_BARO;

	if (--baroMeasDelay == 0){
		bmp285_read_pressure_temperature(&v_pres_u32, &v_temp_s32);
		baroUsed->pressure = v_pres_u32/100.0;
		baroUsed->temperature = v_temp_s32/100.0;
		baroUsed->asl = ((powf((1015.7f / baroUsed->pressure), 0.1902630958f) - 1.0f) * (25 + 273.15f)) / 0.0065f;
		baroMeasDelay = SENSORS_REQ_TICKS_BARO;
	}
#endif
}

bool sensorsReadGyro(Axis3f *gyro)
{
	return (pdTRUE == xQueueReceive(gyroDataQueue, gyro, 0));
}

#ifdef LOG_SEC_IMU
bool sensorsReadGyroSpare(Axis3f *gyro)
{
	return (pdTRUE == xQueueReceive(gyroSpareDataQueue, gyro, 0));
}

bool sensorsReadAccSpare(Axis3f *acc)
{
	return (pdTRUE == xQueueReceive(accelerometerSpareDataQueue, acc, 0));
}
#endif

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
	vl53l0xReadRange(&sensors->zrange, tick);
#ifdef LOG_SEC_IMU
	sensorsReadGyroSpare(&sensors->gyroSpare);
	sensorsReadAccSpare(&sensors->accSpare);
#endif
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

bool imuIsCalibrated(void)
{
	bool status;

	// output depends on the
	switch (usedImuType)
	{
	case SENSORS_BMI160:
#ifdef SENSORS_ENABLE_BMI160
		status = gyroBiasBmi160.isBiasValueFound;
#ifdef SENSORS_TAKE_ACCEL_BIAS
		status &= accelBiasBmi160.isBiasValueFound;
#endif
#endif
		break;
	case SENSORS_BMI055:
#ifdef SENSORS_ENABLE_BMI055
		status = gyroBiasBmi055.isBiasValueFound;
#ifdef SENSORS_TAKE_ACCEL_BIAS
		status &= accelBiasBmi055.isBiasValueFound;
#endif
#endif
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
	return isMagnetometerPresent;
}

static void sensorsBiasInit(BiasObj_t* bias)
{
	bias->isBufferFilled = false;
	bias->bufHead = bias->buffer;
}

/**
 * Calculates the variance and mean for the bias buffer.
 */
static void sensorsCalculateVarianceAndMean(BiasObj_t* bias, Axis3f* varOut, Axis3f* meanOut)
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
static void __attribute__((used)) sensorsCalculateBiasMean(BiasObj_t* bias, Axis3i32* meanOut)
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
static void sensorsAddBiasValue(BiasObj_t* bias, Axis3i16* dVal)
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
static bool sensorsFindBiasValue(BiasObj_t* bias)
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
