/**
 * Copyright (C) 2015 - 2016 Bosch Sensortec GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holder nor the names of the
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 *
 * The information provided is believed to be accurate and reliable.
 * The copyright holder assumes no responsibility
 * for the consequences of use
 * of such information nor for any infringement of patents or
 * other rights of third parties which may result from its use.
 * No license is granted by implication or otherwise under any patent or
 * patent rights of the copyright holder.
 *
 * @file	bmi055_gyro.c
 * @date	16 Feb 2017
 * @version	0.2.0
 * @brief
 *
 */

/*!
 * @defgroup bmi055_gyro
 * @brief    Sensor driver for BMI055 sensor
 * @{*/

/***************************************************************************/
/**\name        Header files
****************************************************************************/
#include "bmi055.h"

/***************************************************************************/
/**\name        Local Function Prototypes
****************************************************************************/
/*!
 * @brief This API sets the range of accel sensor.
 *
 * @param[in] dev  : Structure instance of bmi055_dev.
 * @note : Refer user guide for detailed info.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static int8_t bmi055_set_gyro_range(const struct bmi055_dev *dev);

/*!
 * @brief This API sets the bandwidth of gyroscope sensor.
 *
 * @param[in] dev  : Structure instance of bmi055_dev.
 * @note : Refer user guide for detailed info.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static int8_t bmi055_set_gyro_bandwidth(const struct bmi055_dev *dev);

/*!
 * @brief This internal API is used to validate the device pointer for
 * null conditions.
 *
 * @param[in] dev : Structure instance of bmi055_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */

static int8_t bmi055_gyro_null_ptr_check(const struct bmi055_dev *dev);
/*!
 * @brief This API configures the pins to fire the
 * interrupt signal when it occurs.
 *
 * @param[in] int_config  : Structure instance of bmi055_int_sett.
 * @param[in] dev         : Structure instance of bmi055_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */

#if CODE_UNDER_MODIFICATION
static int8_t set_int_pin_config(struct bmi055_int_sett *int_config, struct bmi055_dev *dev);

/*!
 * @brief This API sets the any motion/slope interrupt of the sensor.
 * This interrupt occurs when slope bet. gyro values exceeds preset
 * threshold.
 *
 * @param[in] int_config  : Structure instance of bmi055_int_sett.
 * @param[in] dev         : Structure instance of bmi055_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static int8_t set_gyro_any_motion_int(struct bmi055_int_sett *int_config, struct bmi055_dev *dev);

/*!
 * @brief This API sets the high-rate interrupt.This interrupt occurs
 * if gyro values exceeds a preset threshold and is used for the
 * detection of shock or other high-angular rate events
 *
 * @param[in] int_config  : Structure instance of bmi055_int_sett.
 * @param[in] dev         : Structure instance of bmi055_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static int8_t set_gyro_high_rate_int(struct bmi055_int_sett *int_config, struct bmi055_dev *dev);

/*!
 * @brief This API sets the data ready interrupt for gyro.
 * This interrupt occurs when new z-axis gyro data comes.
 *
 * @param[in] int_config  : Structure instance of bmi055_int_sett.
 * @param[in] dev         : Structure instance of bmi055_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static int8_t set_gyro_data_ready_int(struct bmi055_int_sett *int_config, struct bmi055_dev *dev);

/*!
 * @brief This API sets the auto-offset tap interrupt for gyro.
 *
 * @param[in] int_config  : Structure instance of bmi055_int_sett.
 * @param[in] dev         : Structure instance of bmi055_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static int8_t set_gyro_auto_offset_tap_int(struct bmi055_int_sett *int_config, struct bmi055_dev *dev);

/*!
 * @brief This API sets the slow-offset interrupt for gyro.
 *
 * @param[in] int_config  : Structure instance of bmi055_int_sett.
 * @param[in] dev         : Structure instance of bmi055_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static int8_t set_gyro_slow_offset_int(struct bmi055_int_sett *int_config, struct bmi055_dev *dev);

/*!
 * @brief This API sets the fast-offset interrupt for gyro.
 *
 * @param[in] int_config  : Structure instance of bmi055_int_sett.
 * @param[in] dev         : Structure instance of bmi055_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static int8_t set_gyro_fast_offset_int(struct bmi055_int_sett *int_config, struct bmi055_dev *dev);
#endif

/***************************************************************************/
/**\name        User Function Definitions
****************************************************************************/
/*!
 *  @brief This API is the entry point for gyroscope sensor. It reads the
 *  chip-id and initializes  gyroscope parameters with default values.
 */
int8_t bmi055_gyro_init(struct bmi055_dev *dev)
{
	/* Variable to define error */
	int8_t rslt;
	/* Variable to assign  chip id */
	uint8_t chip_id = 0;

	/* Null-pointer check */
	rslt = bmi055_gyro_null_ptr_check(dev);
	if (rslt == BMI055_OK) {
		/* Read chip-id of the gyroscope */
		rslt = bmi055_get_gyro_regs(BMI055_GYR_CHIP_ID_ADDR, &chip_id, 1, dev);
		if (rslt == BMI055_OK) {
			/* Validate chip-id */
			if (chip_id == BMI055_GYRO_CHIP_ID) {
				/* Assign chip id to the structure */
				dev->gyro_chip_id = chip_id;
				/* Reset the gyroscope sensor */
				rslt = bmi055_gyro_soft_reset(dev);
				/* waiting time required prior to any
				 *configuration register access */
				dev->delay_ms(5);
			} else {
				/* Communication fail */
				rslt = BMI055_E_DEV_NOT_FOUND;
			}
		} else {
			/* Invalid sensor */
			rslt =  BMI055_E_COM_FAIL;
		}

		/* Initializing gyroscope parameters with default values */
		dev->gyro_cfg.bw = BMI055_GYRO_BW_523_HZ;
		dev->gyro_cfg.power = BMI055_GYRO_PM_NORMAL;
		dev->gyro_cfg.range = BMI055_GYRO_RANGE_2000_DPS;
	}

	return rslt;
}

/*!
 * @brief This API reads the data from the given register address of the
 *gyroscope sensor.
 */
int8_t bmi055_get_gyro_regs(uint8_t reg_addr, uint8_t *data, uint16_t len, const struct bmi055_dev *dev)
{
	/* Variable to define error */
	int8_t rslt;

	/* Null-pointer check */
	rslt = bmi055_gyro_null_ptr_check(dev);
	if (rslt == BMI055_OK) {
		/* Configuring reg_addr for SPI Interface */
		if (dev->interface == BMI055_SPI_INTF)
			reg_addr = (reg_addr | BMI055_SPI_RD_MASK);

		/* Read the gyroscope register */
		rslt = dev->read(dev->gyro_id, reg_addr, data, len);
		dev->delay_ms(1);
		if (rslt != 0)
			/* Communication fail */
			rslt = BMI055_E_COM_FAIL;
	}

	return rslt;
}

/*!
 * @brief This API writes the given data to the register address of the
 *gyroscope sensor.
 */
int8_t bmi055_set_gyro_regs(uint8_t reg_addr, uint8_t *data, uint16_t len, const struct bmi055_dev *dev)
{
	int8_t rslt;

	/* Null-pointer check */
	rslt = bmi055_gyro_null_ptr_check(dev);
	if (rslt == BMI055_OK) {
		/* Configuring reg_addr for SPI Interface */
		if (dev->interface == BMI055_SPI_INTF)
			reg_addr = (reg_addr & BMI055_SPI_WR_MASK);

		/* Write to the gyroscope register */
		rslt = dev->write(dev->gyro_id, reg_addr, data, len);
		dev->delay_ms(1);
		if (rslt != 0)
			rslt = BMI055_E_COM_FAIL;
	}

	return rslt;
}

/*!
 * @brief This API resets and restarts the gyroscope sensor. All register values
 * are overwritten with default values.
 */
int8_t bmi055_gyro_soft_reset(const struct bmi055_dev *dev)
{
	int8_t rslt;
	/* Variable to define the soft reset Value */
	uint8_t data = BMI055_SOFT_RESET_VAL;

	/* Null-pointer check */
	rslt = bmi055_gyro_null_ptr_check(dev);
	if (rslt == BMI055_OK) {
		/* Reset the  gyroscope */
		rslt = bmi055_set_gyro_regs(BMI055_GYR_SOFTRESET_ADDR, &data, 1, dev);
	}

	return rslt;
}

/*!
 * @brief This API sets the power mode of the gyroscope sensor.
 */
int8_t bmi055_set_gyro_power_mode(const struct bmi055_dev *dev)
{
	/* Variable to define error */
	int8_t rslt;
	uint8_t reg_addr = 0,  data = 0;
	/* Variable to define power */
	uint8_t power = 0;
	/* Variable to define register to store data */
	uint8_t reg_data = 0;

	/* Null-pointer check */
	rslt = bmi055_gyro_null_ptr_check(dev);
	if (rslt == BMI055_OK) {
		/* Configure gyroscope power */
		if (dev->gyro_cfg.power <= BMI055_GYRO_PM_SUSPEND) {
			/* Define address for gyroscope power */
			reg_addr = BMI055_GYR_LPM1_ADDR;

			/* Get gyroscope power from the register*/
			rslt = bmi055_get_gyro_regs(reg_addr, &data, 1, dev);
			if (rslt == BMI055_OK) {
				/* Write the configured power mode */
				switch (dev->gyro_cfg.power) {
				/* Value of power on normal power mode */
				case BMI055_GYRO_PM_NORMAL:
					power = 0;
					break;
				/* Value of power on deep suspend power mode */
				case BMI055_GYRO_PM_DEEP_SUSPEND:
					power = 0x20;
					break;
				/* Value of power on suspend power mode */
				case BMI055_GYRO_PM_SUSPEND:
					power = 0x80;
					break;
				default:
					break;
				}

				/* Write the configured power mode */
				reg_data = BMI055_SET_BIT_POS0(data, BMI055_GYRO_POWER, power);

				/* Set the configured power */
				rslt = bmi055_set_gyro_regs(reg_addr, &reg_data, 1, dev);
			} else {
				/* Communication fail */
				rslt = BMI055_E_COM_FAIL;
			}
		} else {
			/* Out of range */
			rslt = BMI055_E_OUT_OF_RANGE;
		}
	}

	return rslt;
}

/*!
 * @brief This API sets the bandwidth of gyroscope sensor.
 */
static int8_t bmi055_set_gyro_bandwidth(const struct bmi055_dev *dev)
{
	/* Variable to define error */
	int8_t rslt;
	/* Variable to define reg address, temp value and buffer */
	uint8_t reg_addr = 0;
	/* Variable to define bandwidth */
	uint8_t bw = 0;
	/* Variable to define register to store data */
	uint8_t reg_data = 0;

	/* Null-pointer check */
	rslt = bmi055_gyro_null_ptr_check(dev);
	if (rslt == BMI055_OK) {
			/* Bandwidth range check */
			if (dev->gyro_cfg.bw <= BMI055_GYRO_BW_32_HZ) {
				/* Address to get gyroscope bandwidth */
				reg_addr = BMI055_GYR_BW_ADDR;

				/* Get gyroscope bandwidth from the register*/
				rslt = bmi055_get_gyro_regs(reg_addr, &bw, 1, dev);
				if (rslt == BMI055_OK) {
					/* Write the corresponding Bandwidth
					   mode in the register variable */
					reg_data = BMI055_SET_BIT_POS0(bw, BMI055_GYRO_BW, dev->gyro_cfg.bw);

					/* Set the configured gyroscope
					bandwidth */
					rslt = bmi055_set_gyro_regs(reg_addr, &reg_data, 1, dev);
				} else {
					/* Communication fail */
					rslt = BMI055_E_COM_FAIL;
				}
			} else{
				/* Not within range */
				rslt = BMI055_E_OUT_OF_RANGE;
			}
	}

	return rslt;
}

/*!
 * @brief This API sets the range of gyroscope sensor.
 */
static int8_t bmi055_set_gyro_range(const struct bmi055_dev *dev)
{
	/* Variable to define error */
	int8_t rslt;
	/* Variable to define reg address, temp value and buffer */
	uint8_t reg_addr = 0;
	/* Variable to define range */
	uint8_t range = 0;
	/* Variable to define register to store data */
	uint8_t reg_data = 0;

	/* Null-pointer check */
	rslt = bmi055_gyro_null_ptr_check(dev);
	if (rslt == BMI055_OK) {
		/* Range check */
		if (dev->gyro_cfg.range <= BMI055_GYRO_RANGE_125_DPS) {
			/* Address to get range */
			reg_addr = BMI055_GYR_RANGE_ADDR;

			/* Get gyroscope range from the register*/
			rslt = bmi055_get_gyro_regs(reg_addr, &range, 1, dev);
			if (rslt == BMI055_OK) {
				/* Write the corresponding range in the
				   register variable */
				reg_data = BMI055_SET_BIT_POS0(range, BMI055_GYRO_RANGE, dev->gyro_cfg.range);

				/* Set the configured range */
				rslt = bmi055_set_gyro_regs(reg_addr, &reg_data, 1, dev);
			} else {
				/* Communication fail */
				rslt = BMI055_E_COM_FAIL;
			}
		} else {
			/* Not within range */
			rslt = BMI055_E_OUT_OF_RANGE;
		}
	}

	return rslt;
}

/*!
 * @brief This API sets the bandwidth and range for bmi055 gyroscope
 */
int8_t bmi055_set_gyro_sensor_config(uint8_t bmi055_config, const struct bmi055_dev *dev)
{
	int8_t rslt;

	/* Check for null pointer in the device structure*/
	rslt = bmi055_gyro_null_ptr_check(dev);
	if (rslt == BMI055_OK) {
		/* Check whether configuration is valid) */
		if (bmi055_config & (BANDWIDTH_CONFIG | RANGE_CONFIG)) {
			/* Set accelerometer bandwidth */
			if (bmi055_config & BANDWIDTH_CONFIG)
				rslt = bmi055_set_gyro_bandwidth(dev);
			/* Set accelerometer range */
			if (bmi055_config & RANGE_CONFIG)
				rslt = bmi055_set_gyro_range(dev);
		} else {
			/* Configuration setting failed */
			rslt = BMI055_E_CONFIG_FAIL;
		}
	}

	return rslt;
}

/*! @brief This API reads the gyroscope data from the sensor, and stores it in the
 * bmi055_sensor_data structure instance passed by the user.
 */
int8_t bmi055_get_gyro_data(struct bmi055_sensor_data *gyro, const struct bmi055_dev *dev)
{
	int8_t rslt;
	uint8_t index = 0, data_array[6] = {0};
	uint16_t lsb = 0, msb = 0, msblsb = 0;

	/* Null-pointer check */
	rslt = bmi055_gyro_null_ptr_check(dev);
	if (rslt == BMI055_OK) {
		/* Read accelerometer sensor data */
		rslt = bmi055_get_gyro_regs(BMI055_GYR_X_L_ADDR, data_array, 6, dev);
		if (rslt == BMI055_OK) {
			/* Parse X-axis accelerometer data */
			lsb = (uint16_t)data_array[index++];
			msb = ((uint16_t)data_array[index++]) << 8;
			msblsb = msb | lsb;
			gyro->x = (int16_t)msblsb;

			/* Parse Y-axis gyroscope data */
			lsb = (uint16_t)data_array[index++];
			msb = ((uint16_t)data_array[index++]) << 8;
			msblsb = msb | lsb;
			gyro->y = (int16_t)msblsb;

			/* Parse Z-axis accelerometer data */
			lsb = (uint16_t)data_array[index++];
			msb = ((uint16_t)data_array[index++]) << 8;
			msblsb = msb | lsb;
			gyro->z = (int16_t)msblsb;
		} else {
			/* Communication fail */
			rslt = BMI055_E_COM_FAIL;
		}
	}

	return rslt;
}

/*!
 * @brief This internal API is used to validate the device structure pointer for
 * null conditions.
 */
static int8_t bmi055_gyro_null_ptr_check(const struct bmi055_dev *dev)
{
	int8_t rslt;

	if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_ms == NULL)) {
		/* Device structure pointer is not valid */
		rslt = BMI055_E_NULL_PTR;
	} else {
		/* Device structure is fine */
		rslt = BMI055_OK;
	}

	return rslt;
}

/* TODO Code Cleanup */

#if 0
/*!
 * @brief This API configures the necessary gyroscope interrupt
 * based on the user settings in the bmi055_int_sett
 * structure instance.
 */
int8_t bmi055_set_gyro_int_config(struct bmi055_int_sett *int_config, struct bmi055_dev *dev)
{
	int8_t rslt = BMI055_OK;

	switch (int_config->gyr_int_type)
	{
	case BMI055_GYRO_ANY_MOTION_INT:
		/* Any motion interrupt */
		rslt = set_gyro_any_motion_int(int_config, dev);
		break;
	case BMI055_GYRO_HIGH_RATE_INT:
		/* High rate Interrupt */
		rslt = set_gyro_high_rate_int(int_config, dev);
		break;
	case BMI055_GYRO_DATA_RDY_INT:
		/* Data Ready interrupt */
		rslt = set_gyro_data_ready_int(int_config, dev);
		break;
	case BMI055_GYRO_AUTO_OFFSET_INT:
		/* Auto-offset interrupt */
		rslt = set_gyro_auto_offset_tap_int(int_config, dev);
		break;
	case BMI055_GYRO_SLOW_OFFSET_INT:
		/* Slow-offset interrupt */
		rslt = set_gyro_slow_offset_int(int_config, dev);
		break;
	case BMI055_GYRO_FAST_OFFSET_INT:
		/* Fast-offset interrupt */
		rslt = set_gyro_fast_offset_int(int_config, dev);
		break;
	 default:
		break;
	}

	return rslt;
}

/***************************************************************************/
/**\name        Local function definitions
****************************************************************************/
/*!
 * @brief This API sets the any motion/slope interrupt of the sensor.
 * This interrupt occurs when slope bet. gyro values exceeds preset
 * threshold.
 */
static int8_t set_gyro_any_motion_int(struct bmi055_int_sett *int_config, struct bmi055_dev *dev)
{
    int8_t rslt = BMI055_OK;
    uint8_t reg_addr, data, temp, data_array[3];

    /* Null-pointer check */
    if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_ms == NULL) || ((int_config == NULL)))
        return BMI055_E_NULL_PTR;

    /* updating the interrupt structure to local structure */
    struct bmi055_gyr_any_mot_int_cfg *any_mot_int = &(int_config->int_type_cfg.gyr_any_mot_int);

    /* Configuring interrupt pins */
    rslt = set_int_pin_config(int_config, dev);

    /* Configure Int Map register to map interrupt pin to any motion interrupt */
    if (int_config->gyr_int_channel == BMI055_INT_CHANNEL_3) {
         reg_addr = BMI055_GYR_INT_MAP_0_ADDR;
         rslt = bmi055_get_gyro_regs(reg_addr, &data, 1, dev);
         if (rslt != BMI055_OK)
             return rslt;
         temp = data & ~BMI055_GYR_INT1_MAP_0_ANY_MOT_MASK;
         data = temp | ((1 << 1) & BMI055_GYR_INT1_MAP_0_ANY_MOT_MASK);//Mapping INT3 pin to any motion interrupt
    }
    else {
        reg_addr = BMI055_GYR_INT_MAP_2_ADDR;
        rslt = bmi055_get_gyro_regs(reg_addr, &data, 1, dev);
        if (rslt != BMI055_OK)
            return rslt;
        temp = data & ~BMI055_GYR_INT2_MAP_2_ANY_MOT_MASK;
        data = temp | ((1 << 1) & BMI055_GYR_INT2_MAP_2_ANY_MOT_MASK);//Mapping INT4 pin to any motion interrupt
    }

    rslt = bmi055_set_gyro_regs(reg_addr, &data, 1, dev);/* write to appropriate Map register */
    if (rslt != BMI055_OK)
        return rslt;

    /* Configuring any motion registers */
    reg_addr = BMI055_GYRO_0_REG;
    rslt = bmi055_get_gyro_regs(reg_addr, data_array, 3, dev);
    dev->delay_ms(1);
    if (rslt != BMI055_OK)
        return rslt;

    /* Write to INT_SRC register */
    data = data_array[0];
    temp = data & ~BMI055_GYR_SRC_ANY_MOT_MASK;
    data = temp | ((0 << 1) & BMI055_GYR_SRC_ANY_MOT_MASK);// // Adding data source for Any-motion Interrupt
    data_array[0] = data;

    data = data_array[1];
    temp = data & ~BMI055_GYR_ANY_THRES_MASK;
    data = temp | (any_mot_int->any_thr & BMI055_GYR_ANY_THRES_MASK);// Adding any motion threshold
    data_array[1] = data;

    data = data_array[2];
    temp = data & ~BMI055_GYR_ANY_EN_X_MASK;
    data = temp | ((any_mot_int->any_en_x) & BMI055_GYR_ANY_EN_X_MASK);// Adding any motion interrupt x flag
    temp = data & ~BMI055_GYR_ANY_EN_Y_MASK;
    data = temp | ((any_mot_int->any_en_y << 1) & BMI055_GYR_ANY_EN_Y_MASK);// Adding any motion interrupt y flag
    temp = data & ~BMI055_GYR_ANY_EN_Z_MASK;
    data = temp | ((any_mot_int->any_en_z << 2) & BMI055_GYR_ANY_EN_Z_MASK);// Adding any motion interrupt z flag
    temp = data & ~BMI055_GYR_ANY_DURSAMPLE_MASK;
    data = temp | ((any_mot_int->any_dursample << 4) & BMI055_GYR_ANY_DURSAMPLE_MASK);// Adding any motion sample duration
    temp = data & ~BMI055_GYR_AWAKE_DUR_MASK;
    data = temp | ((any_mot_int->awake_dur << 6) & BMI055_GYR_AWAKE_DUR_MASK);// Adding any motion awake duration
    data_array[2] = data;
    /* Writing data to any motion interrupt registers simultaneously since required registers are located consecutively */
    rslt = bmi055_set_gyro_regs(reg_addr, data_array, 3, dev);
    dev->delay_ms(1);
    return rslt;

}

/*!
 * @brief This API sets the high-rate interrupt.This interrupt occurs
 * if gyro values exceeds a preset threshold and is used for the
 * detection of shock or other high-angular rate events
 */
static int8_t set_gyro_high_rate_int(struct bmi055_int_sett *int_config, struct bmi055_dev *dev)
{
    int8_t rslt = BMI055_OK;
    uint8_t reg_addr, data, temp, data_array[6];

    /* Null-pointer check */
    if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_ms == NULL) || ((int_config == NULL)))
        return BMI055_E_NULL_PTR;

    /* updating the interrupt structure to local structure */
    struct bmi055_gyr_high_rate_int_cfg *high_rate_int = &(int_config->int_type_cfg.gyr_high_rate_int);

    /* Configuring interrupt pins */
    rslt = set_int_pin_config(int_config, dev);
    if (rslt != BMI055_OK)
        return rslt;

    /* Configure Int Map register to map interrupt pin to High-rate interrupt */
    if (int_config->gyr_int_channel == BMI055_INT_CHANNEL_3) {
         reg_addr = BMI055_GYR_INT_MAP_0_ADDR;
         rslt = bmi055_get_gyro_regs(reg_addr, &data, 1, dev);
         if (rslt != BMI055_OK)
             return rslt;
         temp = data & ~BMI055_GYR_INT1_MAP_0_HIGHRATE_MASK;
         data = temp | ((1 << 3) & BMI055_GYR_INT1_MAP_0_HIGHRATE_MASK);//Mapping INT3 pin to High-rate interrupt
    }
    else {
        reg_addr = BMI055_GYR_INT_MAP_2_ADDR;
        rslt = bmi055_get_gyro_regs(reg_addr, &data, 1, dev);
        if (rslt != BMI055_OK)
            return rslt;
        temp = data & ~BMI055_GYR_INT2_MAP_2_HIGHRATE_MASK;
        data = temp | ((1 << 3) & BMI055_GYR_INT2_MAP_2_HIGHRATE_MASK);//Mapping INT4 pin to High-rate interrupt
    }

    rslt = bmi055_set_gyro_regs(reg_addr, &data, 1, dev);/* write to appropriate Map register */
    if(rslt != BMI055_OK)
        return rslt;

    /* Write to INT_SRC register */
    reg_addr = BMI055_GYRO_0_REG;
    rslt = bmi055_get_gyro_regs(reg_addr, &data, 1, dev);
    if (rslt != BMI055_OK)
        return rslt;

    temp = data & ~BMI055_GYR_SRC_HIGHRATE_MASK;
    data = temp | ((0 << 3) & BMI055_GYR_SRC_HIGHRATE_MASK);// Adding data source for High-rate Interrupt
    rslt = bmi055_set_gyro_regs(reg_addr, &data, 1, dev);
    if (rslt != BMI055_OK)
        return rslt;

    /* Configuring High rate Interrupt registers */
    reg_addr = BMI055_GYR_INT_HR_0_ADDR;
    /* Adding High-rate interrupt for x-axis, High-rate threshold for x-axis and high-rate hysteresis for x-axis */
    data_array[0] = (high_rate_int->high_en_x | (high_rate_int->high_thr_x << 1) | (high_rate_int->high_hy_x << 6));
    /* Adding High-rate duration for x-axis */
    data_array[1] = high_rate_int->high_dur_x;
    /* Adding High-rate interrupt for y-axis, High-rate threshold for y-axis and high-rate hysteresis for y-axis */
    data_array[2] =  (high_rate_int->high_en_y | (high_rate_int->high_thr_y << 1) | (high_rate_int->high_hy_y << 6));
    /* Adding High-rate duration for y-axis */
    data_array[3] = high_rate_int->high_dur_y;
    /* Adding High-rate interrupt for z-axis, High-rate threshold for z-axis and high-rate hysteresis for z-axis */
    data_array[4] = (high_rate_int->high_en_z | (high_rate_int->high_thr_z << 1) | (high_rate_int->high_hy_z << 6));
    /* Adding High-rate duration for z-axis */
    data_array[5] = high_rate_int->high_dur_z;
    /* Writing data to High-rate interrupt registers simultaneously since required registers are located consecutively */
    rslt = bmi055_set_gyro_regs(reg_addr, data_array, 6, dev);
    dev->delay_ms(2);
    return rslt;

}

/*!
 * @brief This API sets the data ready interrupt for gyro.
 * This interrupt occurs when new z-axis gyro data comes.
 */
static int8_t set_gyro_data_ready_int(struct bmi055_int_sett *int_config, struct bmi055_dev *dev)
{
    int8_t rslt = BMI055_OK;
    uint8_t reg_addr, data = 0, temp = 0;

    /* Null-pointer check */
    if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_ms == NULL) || ((int_config == NULL)))
        return BMI055_E_NULL_PTR;

    /* Writing data to Int Enable 0 register */
    reg_addr = BMI055_GYR_INT_EN_0_ADDR;
    rslt = bmi055_get_gyro_regs(reg_addr, &data, 1, dev);
    if (rslt != BMI055_OK)
        return rslt;
    temp = data & ~BMI055_GYR_DATA_INT_EN_0_MASK;
    data = temp | ((1 << 7) & BMI055_GYR_DATA_INT_EN_0_MASK);// Adding new data interrupt flag
    rslt = bmi055_set_gyro_regs(reg_addr, &data, 1, dev);/* write data to Int Enable 0 register */

    if(rslt != BMI055_OK)
        return rslt;

    /* Configuring interrupt pins */
    rslt = set_int_pin_config(int_config, dev);
    if (rslt != BMI055_OK)
        return rslt;

    /* Configure Int Map register to map interrupt pin to new data interrupt */
    reg_addr = BMI055_GYR_INT_MAP_1_ADDR;
    rslt = bmi055_get_gyro_regs(reg_addr, &data, 1, dev);
    if (rslt != BMI055_OK)
        return rslt;

    if (int_config->gyr_int_channel == BMI055_INT_CHANNEL_3) {
         temp = data & ~BMI055_GYR_INT1_MAP_1_DATA_MASK;
         data = temp | (1 & BMI055_GYR_INT1_MAP_1_DATA_MASK);//Mapping INT3 pin to new data interrupt
    }
    else {
        temp = data & ~BMI055_GYR_INT2_MAP_1_DATA_MASK;
        data = temp | ((1 << 7) & BMI055_GYR_INT2_MAP_1_DATA_MASK);//Mapping INT4 pin to new data interrupt
    }

    rslt = bmi055_set_gyro_regs(reg_addr, &data, 1, dev);/* write to Map register */
    return rslt;

}

/*!
 * @brief This API sets the auto-offset tap interrupt for gyro.
 */
static int8_t set_gyro_auto_offset_tap_int(struct bmi055_int_sett *int_config, struct bmi055_dev *dev)
{
    int8_t rslt = BMI055_OK;
    uint8_t reg_addr, data, temp;

    /* Null-pointer check */
    if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_ms == NULL) || ((int_config == NULL)))
        return BMI055_E_NULL_PTR;

    /* updating the interrupt structure to local structure */
    struct bmi055_gyr_auto_offset_int_cfg *auto_offset_int = &(int_config->int_type_cfg.gyr_auto_offset_int);

    /* Writing data to Int Enable 0 register */
    reg_addr = BMI055_GYR_INT_EN_0_ADDR;
    rslt = bmi055_get_gyro_regs(reg_addr, &data, 1, dev);
    if (rslt != BMI055_OK)
        return rslt;
    temp = data & ~BMI055_GYR_AUTO_OFF_INT_EN_0_MASK;
    data = temp | ((1 << 2) & BMI055_GYR_AUTO_OFF_INT_EN_0_MASK);// Adding auto-offset flag
    rslt = bmi055_set_gyro_regs(reg_addr, &data, 1, dev);/* write data to Int Enable 0 register */
    if(rslt != BMI055_OK)
        return rslt;

    /* Configuring interrupt pins */
    rslt = set_int_pin_config(int_config, dev);
    if (rslt != BMI055_OK)
        return rslt;

    /* Configure Int Map register to map interrupt pin to new data interrupt */
    reg_addr = BMI055_GYR_INT_MAP_1_ADDR;
    rslt = bmi055_get_gyro_regs(reg_addr, &data, 1, dev);
    if (rslt != BMI055_OK)
        return rslt;


    if (int_config->gyr_int_channel == BMI055_INT_CHANNEL_3) {
         temp = data & ~BMI055_GYR_INT1_MAP_1_AUTO_OFF_MASK;
         data = temp | ((1 << 3) & BMI055_GYR_INT1_MAP_1_AUTO_OFF_MASK);//Mapping INT3 pin to auto-offset interrupt
    }
    else {
        temp = data & ~BMI055_GYR_INT2_MAP_1_AUTO_OFF_MASK;
        data = temp | ((1 << 4) & BMI055_GYR_INT2_MAP_1_AUTO_OFF_MASK);//Mapping INT4 pin to auto-offset interrupt
    }

    rslt = bmi055_set_gyro_regs(reg_addr, &data, 1, dev);/* write to Map register */
    if(rslt != BMI055_OK)
        return rslt;

    /* Writing data to register to configure auto-offset word length */
    reg_addr = BMI055_GYR_A_FOC_ADDR;
    rslt = bmi055_get_gyro_regs(reg_addr, &data, 1, dev);
    if (rslt != BMI055_OK)
        return rslt;

    temp = data & ~BMI055_GYR_AUTO_OFF_WORD_LEN_MASK;
    data = temp | ((auto_offset_int->auto_offset_wordlen << 6) & BMI055_GYR_AUTO_OFF_WORD_LEN_MASK);// Adding auto-offset word length
    rslt = bmi055_set_gyro_regs(reg_addr, &data, 1, dev);/* write data to register */
    return rslt;
}

/*!
 * @brief This API sets the slow-offset interrupt for gyro.
 */
static int8_t set_gyro_slow_offset_int(struct bmi055_int_sett *int_config, struct bmi055_dev *dev)
{
    int8_t rslt = BMI055_OK;
    uint8_t reg_addr, data, temp;

    /* Null-pointer check */
    if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_ms == NULL) || ((int_config == NULL)))
        return BMI055_E_NULL_PTR;

    /* updating the interrupt structure to local structure */
    struct bmi055_gyr_slow_offset_int_cfg *gyr_slow_offset_int = &(int_config->int_type_cfg.gyr_slow_offset_int);

    /* Configuring interrupt pins */
    rslt = set_int_pin_config(int_config, dev);
    if (rslt != BMI055_OK)
        return rslt;

    /* Write to data source register */
    reg_addr = BMI055_GYRO_0_REG;
    rslt = bmi055_get_gyro_regs(reg_addr, &data, 1, dev);
    if (rslt != BMI055_OK)
        return rslt;

    temp = data & ~BMI055_GYR_SRC_SLOW_OFFSET_MASK;
    data = temp | ((0 << 5) & BMI055_GYR_SRC_SLOW_OFFSET_MASK);// Adding data source for slow-offset interrupt
    rslt = bmi055_set_gyro_regs(reg_addr, &data, 1, dev);//Writing data to src register
    if (rslt != BMI055_OK)
        return rslt;

    /* Configuring Slow-offset register */
    reg_addr = BMI055_GYR_SOC_ADDR;
    rslt = bmi055_get_gyro_regs(reg_addr, &data, 1, dev);
    if (rslt != BMI055_OK)
        return rslt;

    temp = data & ~BMI055_GYR_SLOW_OFF_EN_X_MASK;
    data = temp | (gyr_slow_offset_int->slow_offset_en_x & BMI055_GYR_SLOW_OFF_EN_X_MASK);// Adding slow-offset x-axis flag
    temp = data & ~BMI055_GYR_SLOW_OFF_EN_Y_MASK;
    data = temp | ((gyr_slow_offset_int->slow_offset_en_y << 1) & BMI055_GYR_SLOW_OFF_EN_Y_MASK);// Adding slow-offset y-axis flag
    temp = data & ~BMI055_GYR_SLOW_OFF_EN_Z_MASK;
    data = temp | ((gyr_slow_offset_int->slow_offset_en_z << 2) & BMI055_GYR_SLOW_OFF_EN_Z_MASK);// Adding slow-offset z-axis flag
    temp = data & ~BMI055_GYR_SLOW_OFF_DUR_MASK;
    data = temp | ((gyr_slow_offset_int->slow_offset_dur << 3) & BMI055_GYR_SLOW_OFF_DUR_MASK);// Adding slow-offset duration
    temp = data & ~BMI055_GYR_SLOW_OFF_THR_MASK;
    data = temp | ((gyr_slow_offset_int->slow_offset_thres << 6) & BMI055_GYR_SLOW_OFF_THR_MASK);// Adding slow-offset threshold
    rslt = bmi055_set_gyro_regs(reg_addr, &data, 1, dev);//writing data to slow-offset register
    return rslt;


}

/*!
 * @brief This API sets the fast-offset interrupt for gyro.
 */
static int8_t set_gyro_fast_offset_int(struct bmi055_int_sett *int_config, struct bmi055_dev *dev)
{
    int8_t rslt = BMI055_OK;
    uint8_t reg_addr, data, temp;

    /* Null-pointer check */
    if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_ms == NULL) || ((int_config == NULL)))
        return BMI055_E_NULL_PTR;

    /* updating the interrupt structure to local structure */
    struct bmi055_gyr_fast_offset_int_cfg *gyr_fast_offset_int = &(int_config->int_type_cfg.gyr_fast_offset_int);

    /* Configuring interrupt pins */
    rslt = set_int_pin_config(int_config, dev);
    if (rslt != BMI055_OK)
        return rslt;

    /* Configure Int Map register to map interrupt pin to fast-offset interrupt */
    reg_addr = BMI055_GYR_INT_MAP_1_ADDR;
    rslt = bmi055_get_gyro_regs(reg_addr, &data, 1, dev);
    if (rslt != BMI055_OK)
        return rslt;

    if (int_config->gyr_int_channel == BMI055_INT_CHANNEL_3) {
         temp = data & ~BMI055_GYR_INT1_MAP_1_FAST_OFF_MASK;
         data = temp | ((1 << 1) & BMI055_GYR_INT1_MAP_1_FAST_OFF_MASK);//Mapping INT3 pin to fast-offset interrupt
    }
    else {
        temp = data & ~BMI055_GYR_INT2_MAP_1_FAST_OFF_MASK;
        data = temp | ((1 << 6) & BMI055_GYR_INT2_MAP_1_FAST_OFF_MASK);//Mapping INT4 pin to fast-offset interrupt
    }

    rslt = bmi055_set_gyro_regs(reg_addr, &data, 1, dev);/* write to Map register */
    if(rslt != BMI055_OK)
        return rslt;

    /* Write to data source register */
    reg_addr = BMI055_GYRO_1_REG;
    rslt = bmi055_get_gyro_regs(reg_addr, &data, 1, dev);
    if (rslt != BMI055_OK)
        return rslt;

    temp = data & ~BMI055_GYR_SRC_FAST_OFFSET_MASK;
    data = temp | ((0 << 7) & BMI055_GYR_SRC_FAST_OFFSET_MASK);// Adding data source for fast-offset interrupt
    rslt = bmi055_set_gyro_regs(reg_addr, &data, 1, dev);//Writing to data source register
    if (rslt != BMI055_OK)
        return rslt;

    /* Configuring fast-offset register */
    reg_addr = BMI055_GYR_A_FOC_ADDR;
    rslt = bmi055_get_gyro_regs(reg_addr, &data, 1, dev);
    if(rslt != BMI055_OK)
        return rslt;

    temp = data & ~BMI055_GYR_FAST_OFF_EN_X_MASK;
    data = temp | (gyr_fast_offset_int->fast_offset_en_x & BMI055_GYR_FAST_OFF_EN_X_MASK);// Adding fast-offset enable x-axis
    temp = data & ~BMI055_GYR_FAST_OFF_EN_Y_MASK;
    data = temp | ((gyr_fast_offset_int->fast_offset_en_y << 1) & BMI055_GYR_FAST_OFF_EN_Y_MASK);// Adding fast-offset enable y-axis
    temp = data & ~BMI055_GYR_FAST_OFF_EN_Z_MASK;
    data = temp | ((gyr_fast_offset_int->fast_offset_en_z << 2) & BMI055_GYR_FAST_OFF_EN_Z_MASK);// Adding fast-offset enable z-axis
    temp = data & ~BMI055_GYR_FAST_OFF_EN_MASK;
    data = temp | ((gyr_fast_offset_int->fast_offset_en << 3) & BMI055_GYR_FAST_OFF_EN_MASK);// Adding fast-offset enable
    temp = data & ~BMI055_GYR_FAST_OFF_WORD_LEN_MASK;
    data = temp | ((gyr_fast_offset_int->fast_offset_wordlen << 4) & BMI055_GYR_FAST_OFF_WORD_LEN_MASK);// Adding fast-offset word length
    rslt = bmi055_set_gyro_regs(reg_addr, &data, 1, dev);/* write data to fast-offset register */
    return rslt;

}

/*!
 * @brief This API configures the pins to fire the
 * interrupt signal when it occurs.
 */
static int8_t set_int_pin_config(struct bmi055_int_sett *int_config, struct bmi055_dev *dev)
{
    int8_t rslt = BMI055_OK;
    uint8_t reg_addr, data, temp;

    /* updating the interrupt structure to local structure */
    struct bmi055_int_pin_sett *int_pin_cfg = &(int_config->int_pin_sett);

    /* Configuration of output interrupt signals on pins INT1 and INT2 are done in Int Enable 1 register*/
    reg_addr = BMI055_GYR_INT_EN_1_ADDR;
    rslt = bmi055_get_gyro_regs(reg_addr, &data, 1, dev);
    if (rslt != BMI055_OK)
        return rslt;

    if (int_config->gyr_int_channel == BMI055_INT_CHANNEL_3) {
        temp = data & ~BMI055_GYR_INT1_LVL_INT_EN_1_MASK;
        data = temp | (int_pin_cfg->output_type & BMI055_GYR_INT1_LVL_INT_EN_1_MASK);// Adding output_type to INT3 pin
        temp = data & ~BMI055_GYR_INT1_OD_INT_EN_1_MASK;
        data = temp | ((int_pin_cfg->output_mode << 1) & BMI055_GYR_INT1_OD_INT_EN_1_MASK);// Adding output_mode to INT3 pin
    }
    else {
        temp = data & ~BMI055_GYR_INT2_LVL_INT_EN_1_MASK;
        data = temp | ((int_pin_cfg->output_type << 2) & BMI055_GYR_INT2_LVL_INT_EN_1_MASK);// Adding output_type to INT4 pin
        temp = data & ~BMI055_GYR_INT2_OD_INT_EN_1_MASK;
        data = temp | ((int_pin_cfg->output_mode << 3) & BMI055_GYR_INT2_OD_INT_EN_1_MASK);// Adding output_mode to INT4 pin
    }

    /* Write to Int Enable 1 register */
    rslt = bmi055_set_gyro_regs(reg_addr, &data, 1, dev);
    if (rslt != BMI055_OK)
        return rslt;

    /* Configuring INT_RST_LATCH register */
    reg_addr = BMI055_GYR_INT_LATCH_ADDR;
    rslt = bmi055_get_gyro_regs(reg_addr, &data, 1, dev);
    if (rslt != BMI055_OK)
        return rslt;

    temp = data & ~BMI055_GYR_LATCH_DUR_MASK;
    data = temp | ((int_pin_cfg->latch_dur) & BMI055_GYR_LATCH_DUR_MASK);// Adding Latch duration
    temp = data & ~BMI055_GYR_RESET_INT_MASK;
    data = temp | ((int_pin_cfg->reset_int << 7) & BMI055_GYR_RESET_INT_MASK);// Adding reset_int

    // Adding Offset_reset in case of auto-offset, slow-offset and fast-offset interrupts
    if ((int_config->gyr_int_type == BMI055_GYRO_AUTO_OFFSET_INT) || (int_config->gyr_int_type == BMI055_GYRO_SLOW_OFFSET_INT)
        || (int_config->gyr_int_type == BMI055_GYRO_FAST_OFFSET_INT)) {
        temp = data & ~BMI055_GYR_OFFSET_RESET_MASK;
        data = temp | ((int_pin_cfg->offset_reset << 6) & BMI055_GYR_OFFSET_RESET_MASK);
    }

    /* Write to INT_RST_LATCH register */
    rslt = bmi055_set_gyro_regs(reg_addr, &data, 1, dev);
    return rslt;
}

#endif

/** @}*/
