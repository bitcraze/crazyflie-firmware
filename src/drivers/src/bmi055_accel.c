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
 * @file	bmi055_accel.c
 * @date	19 Apr, 2017
 * @version	1.1.0
 * @brief	Sensor driver for BMI055 sensor
 *
 */

/***************************************************************************/
/**\name        Header files
****************************************************************************/
#include "bmi055.h"

/***************************************************************************/
/**\name        Local Function Prototypes
****************************************************************************/
/*!
 * @brief This internal API sets the range of accelerometer sensor.
 *
 * @param[in] dev  : Structure instance of bmi055_dev.
 *
 * @note : Refer user guide for detailed info.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static int8_t bmi055_set_accel_range(const struct bmi055_dev *dev);

/*!
 * @brief This internal API sets the bandwidth of accelerometer sensor.
 *
 * @param[in] dev  : Structure instance of bmi055_dev.
 *
 * @note : Refer user guide for detailed info.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static int8_t bmi055_set_accel_bandwidth(const struct bmi055_dev *dev);

/*!
 * @brief This internal API is used to validate the device pointer for
 * null conditions.
 *
 * @param[in] dev : Structure instance of bmi055_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
static int8_t bmi055_accel_null_ptr_check(const struct bmi055_dev *dev);

/*!
 * @brief This internal API enables/disables all accelerometer interrupts.
 *
 * @param[in] set_en_bit : Enable or disable bit
 * @param[in] dev        : Structure instance of bmi055_dev.
 *
* @note set_en_bit = 0 -> disable
*	set_en_bit = 1 -> enable
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
static int8_t enable_disable_all_accel_int(uint8_t set_en_bit,  const struct bmi055_dev *dev);

/*!
 * @brief This internal API configures the pins to fire the
 * interrupt signal when it occurs.
 *
 * @param[in] acc_int_config  : Structure instance of bmi055_accel_int_sett.
 * @param[in] dev             : Structure instance of bmi055_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static int8_t set_int_pin_config(struct bmi055_accel_int_sett *acc_int_config, const struct bmi055_dev *dev);

/*!
 * @brief This internal API configures the input output configuration of the
 * interrupt pins
 *
 * @param[in] int_pin_cfg   : Structure instance of bmi055_int_pin_sett.
 * @param[in] acc_int_config    : Structure instance of bmi055_accel_int_sett.
 * @param[in] temp_data     : Input data given
 * @param[out]temp_out_data : Output data from the register
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static int8_t config_in_out_int(const struct bmi055_int_pin_sett *int_pin_cfg,
					const struct bmi055_accel_int_sett *acc_int_config,
						uint8_t temp_data, uint8_t *temp_out_data);

/*!
 * @brief This internal API configures the reset latch configuration of the
 * interrupt pins
 *
 * @param[in] int_pin_cfg   : Structure instance of bmi055_int_pin_sett.
 * @param[in] temp_data     : Input data given
 * @param[out]temp_out_data : Output data from the register
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static void config_reset_latch_int(const struct bmi055_int_pin_sett *int_pin_cfg,
						uint8_t temp_data, uint8_t *temp_out_data);

/*!
 * @brief This internal API sets the orientation interrupt of the sensor.This
 * interrupt occurs when there is orientation change in the sensor
 * with respect to gravitational field vector g.
 *
 * @param[in] acc_int_config  : Structure instance of bmi055_accel_int_sett.
 * @param[in] dev             : Structure instance of bmi055_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 *
 */
static int8_t set_accel_orientation_int(struct bmi055_accel_int_sett *acc_int_config, const struct bmi055_dev *dev);

/*!
 * @brief This internal API enables the orient interrupt
 *
 * @param[in] acc_int_config  : Structure instance of bmi055_accel_int_sett.
 * @param[in] dev             : Structure instance of bmi055_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static int8_t enable_accel_orient_int(struct bmi055_accel_int_sett *acc_int_config, const struct bmi055_dev *dev);

/*!
 * @brief This internal API maps the orient interrupt to either
 * INT pin 1 or 2
 *
 * @param[in] acc_int_config  : Structure instance of bmi055_accel_int_sett.
 * @param[in] dev             : Structure instance of bmi055_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static int8_t map_accel_orient_int(const struct bmi055_accel_int_sett *acc_int_config, const struct bmi055_dev *dev);

/*!
 * @brief This internal API configures registers for orient interrupt
 *
 * @param[in] acc_int_config  : Structure instance of bmi055_accel_int_sett.
 * @param[in] dev	      : Structure instance of bmi055_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static int8_t config_orient_reg(struct bmi055_accel_int_sett *acc_int_config, const struct bmi055_dev *dev);

/*!
 * @brief This internal API sets the slope interrupt of the sensor.This
 * interrupt occurs when slope(absolute value of acceleration difference)
 * exceeds a preset threshold.
 *
 * @param[in] acc_int_config  : Structure instance of bmi055_accel_int_sett.
 * @param[in] dev         : Structure instance of bmi055_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static int8_t set_accel_slope_int(struct bmi055_accel_int_sett *acc_int_config, const struct bmi055_dev *dev);
/*!
 * @brief This internal API enables the slope interrupt
 *
 * @param[in] acc_int_config  : Structure instance of bmi055_accel_int_sett.
 * @param[in] dev         : Structure instance of bmi055_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static int8_t enable_accel_slope_int(struct bmi055_accel_int_sett *acc_int_config, const struct bmi055_dev *dev);

/*!
 * @brief This internal API maps the slope interrupt to either
 * INT pin 1 or 2
 *
 * @param[in] acc_int_config  : Structure instance of bmi055_accel_int_sett.
 * @param[in] dev         : Structure instance of bmi055_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static int8_t map_accel_slope_int(const struct bmi055_accel_int_sett *acc_int_config, const struct bmi055_dev *dev);

/*!
 * @brief This internal API writes the data source definition for
 * slope interrupt - filtered or unfiltered.
 *
 * @param[in] acc_int_config  : Structure instance of bmi055_accel_int_sett.
 * @param[in] dev         : Structure instance of bmi055_dev.
 *
 * @note Value = 0 for filtered data , 1 for unfiltered data
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static int8_t config_int_slope_src(const struct bmi055_accel_int_sett *acc_int_config, const struct bmi055_dev *dev);

/*!
 * @brief This internal API configures registers for slope interrupt
 *
 * @param[in] acc_int_config  : Structure instance of bmi055_accel_int_sett.
 * @param[in] dev         : Structure instance of bmi055_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static int8_t config_slope_reg(struct bmi055_accel_int_sett *acc_int_config, const struct bmi055_dev *dev);

/*!
 * @brief This internal API sets the slow/no-motion interrupt of the sensor.
 * Slow motion interrupt occurs when slope of atleast one enabled axis exceeds
 * preset threshold for programmable number of samples. No motion interrupt
 * occurs when when slope of all enabled axes is less than the preset threshold
 * for a programmable time.
 *
 * @param[in] acc_int_config  : Structure instance of bmi055_accel_int_sett.
 * @param[in] dev         : Structure instance of bmi055_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static int8_t set_accel_slow_no_motion_int(struct bmi055_accel_int_sett *acc_int_config, const struct bmi055_dev *dev);

/*!
 * @brief This internal API enables the slow/no-motion interrupt
 *
 * @param[in] acc_int_config  : Structure instance of bmi055_accel_int_sett.
 * @param[in] dev         : Structure instance of bmi055_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static int8_t enable_accel_slow_no_motion_int(struct bmi055_accel_int_sett *acc_int_config,
							const struct bmi055_dev *dev);

/*!
 * @brief This internal API maps the slow/no-motion interrupt to either
 * INT pin 1 or 2
 *
 * @param[in] acc_int_config  : Structure instance of bmi055_accel_int_sett.
 * @param[in] dev         : Structure instance of bmi055_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static int8_t map_accel_slow_no_motion_int(const struct bmi055_accel_int_sett *acc_int_config,
							const struct bmi055_dev *dev);

/*!
 * @brief This internal API writes the data source definition for
 * slow/no-motion interrupt - filtered or unfiltered.
 *
 * @param[in] acc_int_config  : Structure instance of bmi055_accel_int_sett.
 * @param[in] dev         : Structure instance of bmi055_dev.
 *
 * @note Value = 0 for filtered data , 1 for unfiltered data
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static int8_t config_int_slow_no_motion_src(const struct bmi055_accel_int_sett *acc_int_config,
							const struct bmi055_dev *dev);

/*!
 * @brief This internal API configures registers for slow/no-motion interrupt
 *
 * @param[in] acc_int_config  : Structure instance of bmi055_accel_int_sett.
 * @param[in] dev         : Structure instance of bmi055_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static int8_t config_slow_no_motion_reg(struct bmi055_accel_int_sett *acc_int_config, const struct bmi055_dev *dev);

/*!
 * @brief This internal API configures duration for slow/no-motion interrupt
 *
 * @param[in] acc_int_config  : Structure instance of bmi055_accel_int_sett.
 * @param[in] dev         : Structure instance of bmi055_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static int8_t config_slow_no_motion_dur(struct bmi055_accel_int_sett *acc_int_config, const struct bmi055_dev *dev);

/*!
 * @brief This internal API configures threshold for slow/no-motion interrupt
 *
 * @param[in] acc_int_config  : Structure instance of bmi055_accel_int_sett.
 * @param[in] dev         : Structure instance of bmi055_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static int8_t config_slow_no_motion_thres(struct bmi055_accel_int_sett *acc_int_config, const struct bmi055_dev *dev);

/*!
 * @brief This internal API sets the low-g interrupt of the sensor.This
 * interrupt occurs when the absolute value of sum of all accelerations or
 * acceleration of each axis are lower than the preset threshold.
 *
 * @param[in] acc_int_config  : Structure instance of bmi055_accel_int_sett.
 * @param[in] dev         : Structure instance of bmi055_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static int8_t set_accel_low_g_int(struct bmi055_accel_int_sett *acc_int_config, const struct bmi055_dev *dev);

/*!
 * @brief This internal API enables the low g interrupt
 *
 * @param[in] acc_int_config  : Structure instance of bmi055_accel_int_sett.
 * @param[in] dev         : Structure instance of bmi055_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static int8_t enable_accel_low_g_int(struct bmi055_accel_int_sett *acc_int_config, const struct bmi055_dev *dev);

/*!
 * @brief This internal API maps the low-g interrupt to either
 * INT pin 1 or 2
 *
 * @param[in] acc_int_config  : Structure instance of bmi055_accel_int_sett.
 * @param[in] dev         : Structure instance of bmi055_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static int8_t map_accel_low_g_int(const struct bmi055_accel_int_sett *acc_int_config, const struct bmi055_dev *dev);

/*!
 * @brief This internal API writes the data source definition for
 * low-g interrupt - filtered or unfiltered.
 *
 * @param[in] acc_int_config  : Structure instance of bmi055_accel_int_sett.
 * @param[in] dev         : Structure instance of bmi055_dev.
 *
 * @note Value = 0 for filtered data , 1 for unfiltered data
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static int8_t config_int_low_g_src(const struct bmi055_accel_int_sett *acc_int_config, const struct bmi055_dev *dev);

/*!
 * @brief This internal API configures registers for low-g interrupt
 *
 * @param[in] acc_int_config  : Structure instance of bmi055_accel_int_sett.
 * @param[in] dev         : Structure instance of bmi055_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static int8_t config_low_g_reg(struct bmi055_accel_int_sett *acc_int_config, const struct bmi055_dev *dev);

/*!
 * @brief This internal API sets the high-g interrupt of the sensor.This
 * interrupt occurs when accel values exceeds a preset threshold and it is
 * used for the detection of shock or other high-acceleration events.
 *
 * @param[in] acc_int_config  : Structure instance of bmi055_accel_int_sett.
 * @param[in] dev         : Structure instance of bmi055_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static int8_t set_accel_high_g_int(struct bmi055_accel_int_sett *acc_int_config, const struct bmi055_dev *dev);

/*!
 * @brief This internal API enables the high-g interrupt
 *
 * @param[in] acc_int_config  : Structure instance of bmi055_accel_int_sett.
 * @param[in] dev         : Structure instance of bmi055_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static int8_t enable_accel_high_g_int(struct bmi055_accel_int_sett *acc_int_config, const struct bmi055_dev *dev);

/*!
 * @brief This internal API maps the high-g interrupt to either
 * INT pin 1 or 2
 *
 * @param[in] acc_int_config  : Structure instance of bmi055_accel_int_sett.
 * @param[in] dev         : Structure instance of bmi055_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static int8_t map_accel_high_g_int(const struct bmi055_accel_int_sett *acc_int_config, const struct bmi055_dev *dev);

/*!
 * @brief This internal API writes the data source definition for
 * high-g interrupt - filtered or unfiltered.
 *
 * @param[in] acc_int_config  : Structure instance of bmi055_accel_int_sett.
 * @param[in] dev         : Structure instance of bmi055_dev.
 *
 * @note Value = 0 for filtered data , 1 for unfiltered data
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static int8_t config_int_high_g_src(const struct bmi055_accel_int_sett *acc_int_config, const struct bmi055_dev *dev);

/*!
 * @brief This internal API configures registers for high-g interrupt
 *
 * @param[in] acc_int_config  : Structure instance of bmi055_accel_int_sett.
 * @param[in] dev         : Structure instance of bmi055_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static int8_t config_high_g_reg(struct bmi055_accel_int_sett *acc_int_config, const struct bmi055_dev *dev);

/*!
 * @brief This API sets the data ready interrupt for accel.
 * This interrupt occurs when new z-axis accel data comes.
 *
 * @param[in] acc_int_config  : Structure instance of bmi055_accel_int_sett.
 * @param[in] dev         : Structure instance of bmi055_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static int8_t set_accel_new_data_int(struct bmi055_accel_int_sett *acc_int_config, const struct bmi055_dev *dev);

/*!
 * @brief This internal API enables the data ready interrupt
 *
 * @param[in] dev         : Structure instance of bmi055_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static int8_t enable_accel_new_data_int(const struct bmi055_accel_int_sett *acc_int_config,
						const struct bmi055_dev *dev);

/*!
 * @brief This internal API maps the data ready interrupt to either
 * INT pin 1 or 2
 *
 * @param[in] acc_int_config  : Structure instance of bmi055_accel_int_sett.
 * @param[in] dev         : Structure instance of bmi055_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static int8_t map_accel_new_data_int(const struct bmi055_accel_int_sett *acc_int_config,
						const struct bmi055_dev *dev);

/*!
 * @brief This internal API writes the data source definition for
 * data ready interrupt - filtered or unfiltered.
 *
 * @note Value = 0 for filtered data , 1 for unfiltered data
 *
 * @param[in] acc_int_config  : Structure instance of bmi055_accel_int_sett.
 * @param[in] dev         : Structure instance of bmi055_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static int8_t config_int_new_data_src(const struct bmi055_accel_int_sett *acc_int_config,
						const struct bmi055_dev *dev);

/***************************************************************************/
/**\name        User Function Definitions
****************************************************************************/
/*!
 *  @brief This API is the entry point for accelerometer sensor. It reads
 *  the chip-id and initializes  accelerometer parameters with default values.
 */
int8_t bmi055_accel_init(struct bmi055_dev *dev)
{
	/* Variable to define error */
	int8_t rslt;
	/* Variable to assign  chip id */
	uint8_t chip_id = 0;

	/* Null-pointer check */
	rslt = bmi055_accel_null_ptr_check(dev);
	if (rslt == BMI055_OK) {
		/* Read chip-id of the accelerometer */
		rslt = bmi055_get_accel_regs(BMI055_ACC_CHIP_ID_ADDR, &chip_id, 1, dev);
		if (rslt == BMI055_OK) {
			/* Validate chip-id */
			if (chip_id == BMI055_ACCEL_CHIP_ID) {
				/* Assign chip id to the structure */
				dev->accel_chip_id = chip_id;
				/* Reset the accelerometer sensor */
				rslt = bmi055_accel_soft_reset(dev);
				/* waiting time required prior to any
				configuration register access */
				dev->delay_ms(5);
			} else {
				rslt = BMI055_E_DEV_NOT_FOUND;
			}
		} else {
			rslt =  BMI055_E_COM_FAIL;
		}

		/* Initializing accelerometer sensor parameters
		with default values */
		dev->accel_cfg.bw = BMI055_ACCEL_BW_1000_HZ;
		dev->accel_cfg.power = BMI055_ACCEL_PM_NORMAL;
		dev->accel_cfg.range = BMI055_ACCEL_RANGE_2G;
	}

	return rslt;
}

/*!
 * @brief This API reads the data from the given register address
 * of accelerometer sensor.
 */
int8_t bmi055_get_accel_regs(uint8_t reg_addr, uint8_t *data, uint16_t len, const struct bmi055_dev *dev)
{
	/* Variable to define error */
	int8_t rslt;

	/* Null-pointer check */
	rslt = bmi055_accel_null_ptr_check(dev);
	if (rslt == BMI055_OK) {
		/* Configuring reg_addr for SPI Interface */
		if (dev->interface == BMI055_SPI_INTF)
			reg_addr = (reg_addr | BMI055_SPI_RD_MASK);

		/* Read accelerometer register */
		rslt = dev->read(dev->accel_id, reg_addr, data, len);
		dev->delay_ms(1);
		if (rslt != 0)
			rslt = BMI055_E_COM_FAIL;
	}

	return rslt;
}

/*!
 * @brief This API writes the given data to the register address of
 * accelerometer sensor.
 */
int8_t bmi055_set_accel_regs(uint8_t reg_addr, uint8_t *data, uint16_t len, const struct bmi055_dev *dev)
{
	/* Variable to define error */
	int8_t rslt;
	uint8_t burst_cnt;

	/* Null-pointer check */
	rslt = bmi055_accel_null_ptr_check(dev);
	if (rslt == BMI055_OK) {
		/* Configuring reg_addr for SPI Interface */
		if (dev->interface == BMI055_SPI_INTF)
			reg_addr = (reg_addr & BMI055_SPI_WR_MASK);

		/* Burst write */
		for (burst_cnt = 0; burst_cnt < len; burst_cnt++) {
			/* Write to an accelerometer register */
			rslt = dev->write(dev->accel_id, reg_addr, &data[burst_cnt], len);
			reg_addr++;
			dev->delay_ms(1);
		}

		if (rslt != 0)
			rslt = BMI055_E_COM_FAIL;
	}

	return rslt;
}

/*!
 * @brief This API resets and restarts the accelerometer sensor. All register
 * values are overwritten with default parameters.
 */
int8_t bmi055_accel_soft_reset(const struct bmi055_dev *dev)
{
	/* Variable to define error */
	int8_t rslt;
	/* Variable to define soft reset value */
	uint8_t data = BMI055_SOFT_RESET_VAL;

	/* Null-pointer check */
	rslt = bmi055_accel_null_ptr_check(dev);
	if (rslt == BMI055_OK) {
		/* Reset accelerometer device */
		rslt = bmi055_set_accel_regs(BMI055_ACC_SOFTRESET_ADDR, &data, 1, dev);
	}

	return rslt;
}

/*!
 * @brief This API sets the power mode of the accelerometer sensor.
 */
int8_t bmi055_set_accel_power_mode(const struct bmi055_dev *dev)
{
	/* Variable to define error */
	int8_t rslt;
	/* Variable to define register address of power */
	uint8_t reg_addr = BMI055_ACC_PMU_LPW_ADDR;
	/* Variable defined to store data in the  register*/
	uint8_t reg_data = 0;
	/* Variable to define power */
	uint8_t power = 0;

	/* Null-pointer check */
	rslt = bmi055_accel_null_ptr_check(dev);
	if (rslt == BMI055_OK) {
		/* Power range check */
		if ((dev->accel_cfg.power < BMI055_ACC_POWER_MAX)
			&& (dev->accel_cfg.power != BMI055_ACCEL_INVALID_POWER)) {
			/* Get power mode from the register */
			rslt = bmi055_get_accel_regs(reg_addr, &power, 1, dev);
			if (rslt == BMI055_OK) {
				/* Write the corresponding power mode in the
				register variable */
				reg_data = BMI055_SET_BITS(power, BMI055_ACC_POWER, dev->accel_cfg.power);

				/* Set the configured power to the address */
				rslt = bmi055_set_accel_regs(reg_addr, &reg_data, 1, dev);
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
 * @brief This API sets the bandwidth and range for bmi055 accelerometer
 */
int8_t bmi055_set_accel_sensor_config(uint8_t bmi055_config, const struct bmi055_dev *dev)
{
    /* Variable to define error */
	int8_t rslt;

	/* Check for null pointer in the device structure*/
	rslt = bmi055_accel_null_ptr_check(dev);
	if (rslt == BMI055_OK) {
		/* Check whether configuration is valid) */
		if (bmi055_config & (BANDWIDTH_CONFIG | RANGE_CONFIG)) {
			/* Set accelerometer bandwidth */
			if (bmi055_config & BANDWIDTH_CONFIG)
				rslt = bmi055_set_accel_bandwidth(dev);
			/* Set accelerometer range */
			if (bmi055_config & RANGE_CONFIG)
				rslt = bmi055_set_accel_range(dev);
		} else {
			/* Configuration setting failed */
			rslt = BMI055_E_CONFIG_FAIL;
		}
	}

	return rslt;
}

/*!
 * @brief This API reads the accelerometer data from the sensor, and stores it
 * in the bmi055_sensor_data structure instance passed by the user.
 */
int8_t bmi055_get_accel_data(struct bmi055_sensor_data *accel, const struct bmi055_dev *dev)
{
    /* Variable to define error */
	int8_t rslt;
	/* Variable to define index for array */
	uint8_t index = 0;
	/* Array to store accelerometer data */
	uint8_t data_array[6] = {0};
	/* Variable to store LSB */
	uint16_t lsb = 0;
	/* Variable to store MSB */
	uint16_t msb = 0;
	/* Variable to store MSB-LSB value*/
	uint16_t msblsb = 0;

	/* Null-pointer check */
	rslt = bmi055_accel_null_ptr_check(dev);
	if (rslt == BMI055_OK) {
		/* Read accelerometer sensor data */
		rslt = bmi055_get_accel_regs(BMI055_ACC_X_L_ADDR, data_array, 6, dev);
		if (rslt == BMI055_OK) {
			/* Parse X-axis accelerometer data */
			lsb = (uint16_t)data_array[index++];
			msb = ((uint16_t)data_array[index++]) << 8;
			msblsb = msb | lsb;
			/* Right shift by 4 since resolution is 12 bit */
			accel->x = (int16_t)(msblsb) / (int16_t)(0x10);

			/*Parse Y-axis accelerometer data */
			lsb = (uint16_t)data_array[index++];
			msb = ((uint16_t)data_array[index++]) << 8;
			msblsb = msb | lsb;
			/* Right shift by 4 since resolution is 12 bit */
			accel->y = (int16_t)(msblsb) / (int16_t)(0x10);

			/* Parse Z-axis accelerometer data */
			lsb = (uint16_t)data_array[index++];
			msb = ((uint16_t)data_array[index++]) << 8;
			msblsb = msb | lsb;
			/* Right shift by 4 since resolution is 12 bit */
			accel->z = (int16_t)(msblsb) / (int16_t)(0x10);

		} else {
			/* Communication fail */
			rslt = BMI055_E_COM_FAIL;
		}
	}

	return rslt;
}

/* TODO Code Cleanup */
/*!
 * @brief This API configures the necessary accelerometer interrupt based on
 * the user settings in the bmi055_accel_int_sett structure instance.
 */
int8_t bmi055_set_accel_int_config(struct bmi055_accel_int_sett *acc_int_config, const struct bmi055_dev *dev)
{
	/* Variable to define error */
	int8_t rslt;

	 /* Null pointer error check */
	rslt = bmi055_accel_null_ptr_check(dev);
	if ((rslt == BMI055_OK) && (acc_int_config != NULL)) {
		switch (acc_int_config->acc_int_types) {
		case BMI055_ACC_SLOPE_INT:
			/*Any motion or slope interrupt*/
			rslt = set_accel_slope_int(acc_int_config, dev);
			break;
#if CODE_UNDER_MODIFICATION
		case BMI055_ACC_DOUBLE_TAP_INT:
		case BMI055_ACC_SINGLE_TAP_INT:
			/* Double tap and single tap Interrupt */
			rslt = set_accel_tap_int(acc_int_config, dev);
			break;
		case BMI055_ACC_FLAT_INT:
			/* Flat detection interrupt */
			rslt = set_accel_flat_detect_int(acc_int_config, dev);
			break;
#endif
		case BMI055_ACC_ORIENT_INT:
			/* Orientation interrupt */
			rslt = set_accel_orientation_int(acc_int_config, dev);
			break;
		case BMI055_ACC_SLOW_NO_MOTION_INT:
			/* Slow or no motion interrupt */
			rslt = set_accel_slow_no_motion_int(acc_int_config, dev);
			break;

		case BMI055_ACC_NEW_DATA_INT:
			/* New data interrupt */
			rslt = set_accel_new_data_int(acc_int_config, dev);
			break;
		case BMI055_ACC_LOW_G_INT:
			/* Low-g interrupt */
			rslt = set_accel_low_g_int(acc_int_config, dev);
			break;
		case BMI055_ACC_HIGH_G_INT:
			/* High-g interrupt */
			rslt = set_accel_high_g_int(acc_int_config, dev);
			break;
		default:
			break;
		}
	} else {
		rslt = BMI055_E_NULL_PTR;
	}

	return rslt;
}

/*! This API retrieves the status of accelerometer interrupts
 * when set
 */
int8_t bmi055_get_accel_int_status(uint32_t *intr_status, const struct bmi055_dev *dev)
{
	/* Variable to define error */
	int8_t rslt;
	/* Variable to declare status register address */
	uint8_t status_reg_addr = BMI055_ACC_INT_STATUS_0_ADDR;
	/* Array to store the status of the interrupts */
	uint8_t status_array[4] = {0};
	/* Variable for loop */
	uint8_t i = 0;

	/* Null pointer error check */
	rslt = bmi055_accel_null_ptr_check(dev);
	if ((rslt == BMI055_OK) && (intr_status != NULL)) {
		/* Get the interrupt status from the register*/
		rslt = bmi055_get_accel_regs(status_reg_addr, status_array, 4, dev);
		if (rslt == BMI055_OK) {
			while (i < 4) {
				*((uint8_t *)intr_status + i) = status_array[i];
				i++;
			}
		}
	} else {
		rslt = BMI055_E_NULL_PTR;
	}

	return rslt;
}

/*!
 * @brief This API enables/disables individual or all accelerometer interrupts.
 */
int8_t bmi055_set_reset_accel_int(struct bmi055_accel_int_sett *acc_int_config, const struct bmi055_dev *dev,
						uint8_t set_en_bit)
{
	/* Variable to define error */
	int8_t rslt;
	/* Variable to define accelerometer interrupt */
	uint8_t accel_int = acc_int_config->acc_int_types;

	/* Null pointer error check */
	rslt = bmi055_accel_null_ptr_check(dev);
	if ((rslt == BMI055_OK) && (acc_int_config != NULL)) {
		switch (accel_int) {
		case BMI055_ACC_NEW_DATA_INT:
			/* Flag to enable/disable new-data interrupt */
			acc_int_config->acc_int_type_cfg.accel_new_data_en = set_en_bit;

			/* Set the interrupt configuration */
			rslt = bmi055_set_accel_int_config(acc_int_config, dev);
			break;
		case BMI055_ACC_LOW_G_INT:
			/* Flag to enable/disable low-g interrupt */
			acc_int_config->acc_int_type_cfg.acc_low_g_int.low_g_en = set_en_bit;

			/* Set the interrupt configuration */
			rslt = bmi055_set_accel_int_config(acc_int_config, dev);
			break;
		case BMI055_ACC_HIGH_G_INT:
			/* Flag to enable/disable x, y and z axis for
			high-g interrupt */
			acc_int_config->acc_int_type_cfg.acc_high_g_int.high_g_en_x = set_en_bit;
			acc_int_config->acc_int_type_cfg.acc_high_g_int.high_g_en_y = set_en_bit;
			acc_int_config->acc_int_type_cfg.acc_high_g_int.high_g_en_z = set_en_bit;

			/* Set the interrupt configuration */
			rslt = bmi055_set_accel_int_config(acc_int_config, dev);
			break;
		case BMI055_ACC_SLOW_NO_MOTION_INT:
			/* Flag to enable/disable x, y and z axis for
			slow/no-motion interrupt */
			acc_int_config->acc_int_type_cfg.acc_slow_no_mot_int.slow_no_mot_en_x = set_en_bit;
			acc_int_config->acc_int_type_cfg.acc_slow_no_mot_int.slow_no_mot_en_y = set_en_bit;
			acc_int_config->acc_int_type_cfg.acc_slow_no_mot_int.slow_no_mot_en_z = set_en_bit;

			/* Set the interrupt configuration */
			rslt = bmi055_set_accel_int_config(acc_int_config, dev);
			break;
		case BMI055_ACC_SLOPE_INT:
			/* Flag to enable/disable x, y and z axis for
			slope interrupt */
			acc_int_config->acc_int_type_cfg.acc_slope_int.slope_en_x = set_en_bit;
			acc_int_config->acc_int_type_cfg.acc_slope_int.slope_en_y = set_en_bit;
			acc_int_config->acc_int_type_cfg.acc_slope_int.slope_en_z = set_en_bit;

			/* Set the interrupt configuration */
			rslt = bmi055_set_accel_int_config(acc_int_config, dev);
			break;
		case BMI055_ACC_ORIENT_INT:
			/* Flag to enable/disable orient interrupt */
			acc_int_config->acc_int_type_cfg.acc_orient_int.orient_en = set_en_bit;

			/* Set the interrupt configuration */
			rslt = bmi055_set_accel_int_config(acc_int_config, dev);
			break;
		case BMI055_ALL_ACCEL_INT:
			rslt = enable_disable_all_accel_int(set_en_bit, dev);
			break;
		default:
			break;
		}
	} else {
		rslt = BMI055_E_NULL_PTR;
	}

	return rslt;
}

/*!
 * @brief This API clears all the latched interrupts of the accelerometer.
 */
int8_t bmi055_reset_accel_latch_int(struct bmi055_accel_int_sett *acc_int_config, const struct bmi055_dev *dev)
{

	/* Variable to define error */
	int8_t rslt;
	/* Variable to define register address */
	uint8_t reg_addr = BMI055_ACC_INT_LATCH_ADDR;
	/* Variable defined to store register data temporarily */
	uint8_t temp_reg_data;
	/* Variable defined to store data temporarily */
	uint8_t temp_data = 0;

	/* Null pointer error check */
	rslt = bmi055_accel_null_ptr_check(dev);

	if ((rslt == BMI055_OK) && (acc_int_config != NULL)) {
		/* Update the local structure with the pin configuration
		structure */
		struct bmi055_int_pin_sett *int_pin_cfg = &(acc_int_config->int_pin_sett);

		/* Get the reset latch status */
		rslt = bmi055_get_accel_regs(reg_addr, &temp_data, 1, dev);
		if (rslt == BMI055_OK) {
			/* Configuring interrupt reset - '1' clears all latched
			interrupts and '0' keeps latched interrupt active */
			temp_reg_data = BMI055_SET_BIT_POS0(temp_data, BMI055_ACC_RESET_INT,
					int_pin_cfg->reset_int);

			/* Write reset status to INT_RST_LATCH address */
			rslt = bmi055_set_accel_regs(reg_addr, &temp_reg_data, 1, dev);
		}
	} else {
		rslt = BMI055_E_NULL_PTR;
	}

	return rslt;
}

/***************************************************************************/
/**\name        Local Function Definitions
****************************************************************************/

/*!
 * @brief This internal API sets the bandwidth of accelerometer sensor.
 */
static int8_t bmi055_set_accel_bandwidth(const struct bmi055_dev *dev)
{
	/* Variable to define error */
	int8_t rslt;
	/* Variable to define register address of bandwidth */
	uint8_t reg_addr = BMI055_ACC_BW_ADDR;
	/* Variable defined to store data in the  register*/
	uint8_t reg_data = 0;
	/* Variable to define bandwidth */
	uint8_t bw = 0;

	/* Null-pointer check */
	rslt = bmi055_accel_null_ptr_check(dev);
	if (rslt == BMI055_OK) {
		/* Bandwidth range check */
		if ((dev->accel_cfg.bw >= BMI055_ACCEL_BW_7_81_HZ) && (dev->accel_cfg.bw <= BMI055_ACCEL_BW_1000_HZ)) {
			/* Get accelerometer bandwidth from the register*/
			rslt = bmi055_get_accel_regs(reg_addr, &bw, 1, dev);
			if (rslt == BMI055_OK) {
				/* Write the corresponding Bandwidth mode in the
				register variable */
				reg_data = BMI055_SET_BIT_POS0(bw, BMI055_ACC_BW, dev->accel_cfg.bw);

				/* Set the configured accelerometer bandwidth */
				rslt = bmi055_set_accel_regs(reg_addr, &reg_data, 1, dev);
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
 * @brief This internal API sets the range of accelerometer sensor.
 */
static int8_t bmi055_set_accel_range(const struct bmi055_dev *dev)
{
	/* Variable to define error */
	int8_t rslt;
	/* Variable to define register address of range */
	uint8_t reg_addr = BMI055_ACC_RANGE_ADDR;
	/* Variable defined to store data in the  register*/
	uint8_t reg_data = 0;
	/* Variable to define range */
	uint8_t range = 0;

	/* Null-pointer check */
	rslt = bmi055_accel_null_ptr_check(dev);
	if (rslt == BMI055_OK) {
		/* Range check */
		if ((dev->accel_cfg.range == BMI055_ACCEL_RANGE_2G) ||
			(dev->accel_cfg.range == BMI055_ACCEL_RANGE_4G) ||
			(dev->accel_cfg.range == BMI055_ACCEL_RANGE_8G) ||
			(dev->accel_cfg.range == BMI055_ACCEL_RANGE_16G)) {

			/* Get accelerometer range from the register*/
			rslt = bmi055_get_accel_regs(reg_addr, &range, 1, dev);
			if (rslt == BMI055_OK) {
				/* Write the corresponding range in the register
				variable */
				reg_data = BMI055_SET_BIT_POS0(range, BMI055_ACC_RANGE, dev->accel_cfg.range);

				/* Set the configured range */
				rslt = bmi055_set_accel_regs(reg_addr, &reg_data, 1, dev);
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
 * @brief This internal API sets the new data interrupt for accelerometer.
 * This interrupt occurs when new z-axis accelerometer data is stored
 * in the register.
 */
static int8_t set_accel_new_data_int(struct bmi055_accel_int_sett *acc_int_config, const struct bmi055_dev *dev)
{
	/* Variable to define error */
	int8_t rslt;

	/* Enable new data interrupt */
	rslt = enable_accel_new_data_int(acc_int_config, dev);
	if (rslt == BMI055_OK) {
		/* Map new data interrupt to either INT 1 or 2 */
		rslt = map_accel_new_data_int(acc_int_config, dev);
		if (rslt == BMI055_OK) {
			/* Write data source definition for new data
			interrupt */
			rslt = config_int_new_data_src(acc_int_config, dev);
			if (rslt == BMI055_OK) {
				/* Configuring interrupt pins */
				rslt = set_int_pin_config(acc_int_config, dev);
			}
		}
	}

	return rslt;
}

/*!
 * @brief This internal API enables the new data interrupt
 */
static int8_t enable_accel_new_data_int(const struct bmi055_accel_int_sett *acc_int_config,
						const struct bmi055_dev *dev)
{
	/* Variable to define error */
	int8_t rslt;
	/* Variable to define address of Interrupt enable 1 register*/
	uint8_t reg_addr = BMI055_ACC_INT_EN_1_ADDR;
	/* Variable defined to retrieve data */
	uint8_t data = 0;
	/* Variable defined to set data in register */
	uint8_t reg_data;

	/* Get interrupt enable status from the register */
	rslt = bmi055_get_accel_regs(reg_addr, &data, 1, dev);
	if (rslt == BMI055_OK) {
		/* Write interrupt enable bits in the variable */
		reg_data = BMI055_SET_BITS(data, BMI055_ACC_NEW_DATA_INT_EN_1,
							acc_int_config->acc_int_type_cfg.accel_new_data_en);

		/* Enable new data interrupt in Int Enable 1 register */
		rslt = bmi055_set_accel_regs(reg_addr, &reg_data, 1, dev);
	}

	return rslt;
}

/*!
 * @brief This internal API maps the new data interrupt to either
 * INT pin 1 or 2
 */
static int8_t map_accel_new_data_int(const struct bmi055_accel_int_sett *acc_int_config, const struct bmi055_dev *dev)
{
	/* Variable to define error */
	int8_t rslt;
	/* Variable to define register address of Map1 register */
	uint8_t reg_addr = BMI055_ACC_INT_MAP_1_ADDR;
	/* Variable defined to retrieve data */
	uint8_t data = 0;
	/* Variable defined to set data in register */
	uint8_t reg_data;

	/* Get interrupt map status from the register */
	rslt = bmi055_get_accel_regs(reg_addr, &data, 1, dev);
	if (rslt == BMI055_OK) {
		if (acc_int_config->acc_int_channel == BMI055_INT_CHANNEL_1) {
			/* Map new data interrupt to INT1 pin */
			reg_data = BMI055_SET_BIT_POS0(data, BMI055_ACC_INT1_MAP_1_NEW_DATA, BMI055_ENABLE);

		} else if (acc_int_config->acc_int_channel == BMI055_INT_CHANNEL_2) {
			/* Map new data interrupt to INT2 pin */
			reg_data = BMI055_SET_BITS(data, BMI055_ACC_INT2_MAP_1_NEW_DATA, BMI055_ENABLE);

		} else {
			rslt = BMI055_E_INVALID_CHANNEL;
		}

		if (rslt == BMI055_OK) {
			/* Configure the Interrupt map register to map new data
			interrupt to either INT1 or INT2 pin*/
			rslt = bmi055_set_accel_regs(reg_addr, &reg_data, 1, dev);
		}
	}

	return rslt;
}

/*!
 * @brief This internal API writes the data source definition for
 * new data interrupt - filtered or unfiltered.
 *
 * @note Value = 0(BMI055_FALSE) for filtered data , 1(BMI055_TRUE)
 * for unfiltered data
 */
static int8_t config_int_new_data_src(const struct bmi055_accel_int_sett *acc_int_config, const struct bmi055_dev *dev)
{
	/* Variable to define error */
	int8_t rslt;
	/* Variable to define register address of Interrupt source */
	uint8_t reg_addr = BMI055_ACC_INT_SRC_ADDR;
	/* Variable defined to retrieve data */
	uint8_t data = 0;
	/* Variable defined to set data in register */
	uint8_t reg_data;

	/* Get interrupt source data from the register */
	rslt = bmi055_get_accel_regs(reg_addr, &data, 1, dev);
	if (rslt == BMI055_OK) {
		/* Write the interrupt source data in a variable*/
		reg_data = BMI055_SET_BITS(data, BMI055_ACC_SRC_NEW_DATA, acc_int_config->int_data_src_type);

		/* Write data source definition for new data interrupt */
		rslt = bmi055_set_accel_regs(reg_addr, &reg_data, 1, dev);
	}

	return rslt;
}

/*!
 * @brief This internal API sets the low-g interrupt of the sensor.This
 * interrupt occurs when the absolute value of sum of all accelerations or
 * acceleration of each axis are lower than the preset threshold.
 */
static int8_t set_accel_low_g_int(struct bmi055_accel_int_sett *acc_int_config,  const struct bmi055_dev *dev)
{
	/* Variable to define error */
	int8_t rslt;

	/* Enable low-g interrupt */
	rslt = enable_accel_low_g_int(acc_int_config, dev);
	if (rslt == BMI055_OK) {
		/* Map low-g interrupt to either INT 1 or 2 */
		rslt = map_accel_low_g_int(acc_int_config, dev);
		if (rslt == BMI055_OK) {
			/* Write data source definition for low-g
			interrupt */
			rslt = config_int_low_g_src(acc_int_config, dev);
			if (rslt == BMI055_OK) {
				/* Configuring interrupt pins */
				rslt = set_int_pin_config(acc_int_config, dev);
				if (rslt == BMI055_OK) {
					/* Configure Low-g registers */
					rslt = config_low_g_reg(acc_int_config, dev);
				}
			}
		}
	}

	return rslt;
}

/*!
 * @brief This internal API enables the low-g interrupt
 */
static int8_t enable_accel_low_g_int(struct bmi055_accel_int_sett *acc_int_config, const struct bmi055_dev *dev)
{
	/* Variable to define error */
	int8_t rslt;
	/* Variable to define address of Interrupt enable 1 register*/
	uint8_t reg_addr = BMI055_ACC_INT_EN_1_ADDR;
	/* Variable defined to retrieve data */
	uint8_t data = 0;
	/* Variable defined to set data in register */
	uint8_t reg_data;

	/* Updating the interrupt structure to local structure */
	struct bmi055_acc_low_g_int_cfg *low_g_int_cfg = &(acc_int_config->acc_int_type_cfg.acc_low_g_int);

	/* Get interrupt enable status from the register */
	rslt = bmi055_get_accel_regs(reg_addr, &data, 1, dev);
	if (rslt == BMI055_OK) {
		/* Write interrupt enable bits in the variable */
		reg_data = BMI055_SET_BITS(data, BMI055_ACC_LOW_G_INT_EN_1, low_g_int_cfg->low_g_en);

		/* Enable low-g interrupt in Interrupt Enable 1 register */
		rslt = bmi055_set_accel_regs(reg_addr, &reg_data, 1, dev);
	}

	return rslt;
}

/*!
 * @brief This internal API maps the low-g interrupt to either
 * INT pin 1 or 2
 */
static int8_t map_accel_low_g_int(const struct bmi055_accel_int_sett *acc_int_config, const struct bmi055_dev *dev)
{
	/* Variable to define error */
	int8_t rslt;
	/* Variable to define register address */
	uint8_t reg_addr = 0;
	/* Variable defined to retrieve data */
	uint8_t data = 0;
	/* Variable defined to set data in register */
	uint8_t reg_data = 0;

	if (acc_int_config->acc_int_channel == BMI055_INT_CHANNEL_1) {
		/* Define Interrupt map 0 register */
		reg_addr = BMI055_ACC_INT_MAP_0_ADDR;

		/* Get interrupt map status from the register */
		rslt = bmi055_get_accel_regs(reg_addr, &data, 1, dev);
		if (rslt == BMI055_OK) {
			/* Map low-g interrupt to INT1 pin */
			reg_data = BMI055_SET_BIT_POS0(data, BMI055_ACC_INT1_MAP_0_LOW_G, BMI055_ENABLE);
		}
	} else if (acc_int_config->acc_int_channel == BMI055_INT_CHANNEL_2) {
		/* Define Interrupt map 2 register */
		reg_addr = BMI055_ACC_INT_MAP_2_ADDR;

		/* Get interrupt map status from the register */
		rslt = bmi055_get_accel_regs(reg_addr, &data, 1, dev);
		if (rslt == BMI055_OK) {
			/* Map low-g interrupt to INT2 pin */
			reg_data = BMI055_SET_BIT_POS0(data, BMI055_ACC_INT2_MAP_2_LOW_G, BMI055_ENABLE);
		}
	} else {
		rslt = BMI055_E_INVALID_CHANNEL;
	}


	if (rslt == BMI055_OK) {
		/* Configure the Interrupt map register to map low-g
		interrupt to either INT1 or INT2 pin*/
		rslt = bmi055_set_accel_regs(reg_addr, &reg_data, 1, dev);
	}

	return rslt;
}
/*!
 * @brief This internal API writes the data source definition for
 * low-g interrupt - filtered or unfiltered.
 *
 * @note Value = 0(BMI055_FALSE) for filtered data , 1(BMI055_TRUE)
 * for unfiltered data
 */
static int8_t config_int_low_g_src(const struct bmi055_accel_int_sett *acc_int_config, const struct bmi055_dev *dev)
{
	/* Variable to define error */
	int8_t rslt;
	/* Variable to define register address of Interrupt source */
	uint8_t reg_addr = BMI055_ACC_INT_SRC_ADDR;
	/* Variable defined to retrieve data */
	uint8_t data = 0;
	/* Variable defined to set data in register */
	uint8_t reg_data;

	/* Get interrupt source data from the register */
	rslt = bmi055_get_accel_regs(reg_addr, &data, 1, dev);
	if (rslt == BMI055_OK) {
		/* Write the interrupt source data in a variable */
		reg_data = BMI055_SET_BIT_POS0(data, BMI055_ACC_SRC_LOW_G, acc_int_config->int_data_src_type);

		/* Write data source definition for low-g interrupt */
		rslt = bmi055_set_accel_regs(reg_addr, &reg_data, 1, dev);
	}

	return rslt;
}
/*!
 * @brief This internal API configures registers for low-g interrupt
 */
static int8_t config_low_g_reg(struct bmi055_accel_int_sett *acc_int_config, const struct bmi055_dev *dev)
{
	/* Variable to define error */
	int8_t rslt;
	/* Variable to define register address */
	uint8_t reg_addr;
	/* Variable defined to set data in register */
	uint8_t reg_data = 0;
	/* Variable defined to retrieve data */
	uint8_t data = 0;
	/* Array defined to store data */
	uint8_t data_array[3] = {0};

	/* Updating the interrupt structure to local structure */
	struct bmi055_acc_low_g_int_cfg *low_g_int_cfg = &(acc_int_config->acc_int_type_cfg.acc_low_g_int);

	/* Configuring Low-g registers */
	/* Store the low duration in the array */
	data_array[0] = low_g_int_cfg->low_dur;
	/* Store the low threshold in the array */
	data_array[1] = low_g_int_cfg->low_thres;

	reg_addr = BMI055_ACC_INT_LH_2_ADDR;

	/*Get the low-g interrupt mode and hysteresis setting */
	rslt = bmi055_get_accel_regs(reg_addr, &data, 1, dev);
	if (rslt == BMI055_OK) {
		/* Get low hysteresis value */
		reg_data = BMI055_SET_BIT_POS0(data, BMI055_ACC_LOW_HYST, low_g_int_cfg->low_hyst);
		/* Get Low mode */
		reg_data = BMI055_SET_BITS(reg_data, BMI055_ACC_LOW_MODE, low_g_int_cfg->low_mode);

		/* Store low hysteresis and low mode in the array */
		data_array[2] = reg_data;

		reg_addr = BMI055_ACC_INT_LH_0_ADDR;

		/* Write the data simultaneously since addresses are located in
		consecutive positions.*/
		rslt = bmi055_set_accel_regs(reg_addr, data_array, 3, dev);
		dev->delay_ms(1);
	}

	return rslt;
}

/*!
 * @brief This internal  API sets the high-g interrupt of the sensor.This
 * interrupt occurs when either of the accel values exceeds a preset threshold.
 * It is used for the detection of shock or other high-acceleration events.
 */
static int8_t set_accel_high_g_int(struct bmi055_accel_int_sett *acc_int_config,  const struct bmi055_dev *dev)
{
	/* Variable to define error */
	int8_t rslt;

	/* Enable high-g interrupt */
	rslt = enable_accel_high_g_int(acc_int_config, dev);
	if (rslt == BMI055_OK) {
		/* Map high-g interrupt to either INT 1 or 2 */
		rslt = map_accel_high_g_int(acc_int_config, dev);
		if (rslt == BMI055_OK) {
			/* Write data source definition for high-g
			interrupt */
			rslt = config_int_high_g_src(acc_int_config, dev);
			if (rslt == BMI055_OK) {
				/* Configuring interrupt pins */
				rslt = set_int_pin_config(acc_int_config, dev);
				if (rslt == BMI055_OK) {
					/* Configure high-g registers */
					rslt = config_high_g_reg(acc_int_config, dev);
				}
			}
		}
	}

	return rslt;
}

/*!
 * @brief This internal API enables the high-g interrupt
 */
static int8_t enable_accel_high_g_int(struct bmi055_accel_int_sett *acc_int_config, const struct bmi055_dev *dev)
{
	/* Variable to define error */
	int8_t rslt;
	/* Variable to define register address of enable 1 register*/
	uint8_t reg_addr = BMI055_ACC_INT_EN_1_ADDR;
	/* Variable defined to retrieve data */
	uint8_t data = 0;
	/* Variable defined to set data in register */
	uint8_t reg_data;

	struct bmi055_acc_high_g_int_cfg  *high_g_int_cfg = &(acc_int_config->acc_int_type_cfg.acc_high_g_int);

	/* Get interrupt enable status from the register */
	rslt = bmi055_get_accel_regs(reg_addr, &data, 1, dev);
	if (rslt == BMI055_OK) {
		/* Enabling High-g x axis */
		reg_data = BMI055_SET_BIT_POS0(data, BMI055_ACC_HIGH_G_X_INT_EN_1, high_g_int_cfg->high_g_en_x);

		/* Enabling High-g y axis */
		reg_data = BMI055_SET_BITS(reg_data, BMI055_ACC_HIGH_G_Y_INT_EN_1, high_g_int_cfg->high_g_en_y);

		/* Enabling High-g z axis */
		reg_data = BMI055_SET_BITS(reg_data, BMI055_ACC_HIGH_G_Z_INT_EN_1, high_g_int_cfg->high_g_en_z);

		/* Enable high-g interrupt in Int Enable 1 register */
		rslt = bmi055_set_accel_regs(reg_addr, &reg_data, 1, dev);
	}

	return rslt;
}

/*!
 * @brief This internal API maps the high-g interrupt to either
 * INT pin 1 or 2
 */
static int8_t map_accel_high_g_int(const struct bmi055_accel_int_sett *acc_int_config, const struct bmi055_dev *dev)
{
	/* Variable to define error */
	int8_t rslt;
	/* Variable to define register address */
	uint8_t reg_addr = 0;
	/* Variable defined to retrieve data */
	uint8_t data = 0;
	/* Variable defined to set data in register */
	uint8_t reg_data;

	if (acc_int_config->acc_int_channel == BMI055_INT_CHANNEL_1) {
		/* Define Interrupt map 0 register */
		reg_addr = BMI055_ACC_INT_MAP_0_ADDR;

		/* Get interrupt map status from the register */
		rslt = bmi055_get_accel_regs(reg_addr, &data, 1, dev);
		if (rslt == BMI055_OK) {
			/* Map high-g interrupt to INT1 pin */
			reg_data = BMI055_SET_BITS(data, BMI055_ACC_INT1_MAP_0_HIGH_G, BMI055_ENABLE);
		}
	} else if (acc_int_config->acc_int_channel == BMI055_INT_CHANNEL_2) {
		/* Define Interrupt map 2 register */
		reg_addr = BMI055_ACC_INT_MAP_2_ADDR;

		/* Get interrupt map status from the register */
		rslt = bmi055_get_accel_regs(reg_addr, &data, 1, dev);
		if (rslt == BMI055_OK) {
			/* Map high-g interrupt to INT2 pin */
			reg_data = BMI055_SET_BITS(data, BMI055_ACC_INT2_MAP_2_HIGH_G, BMI055_ENABLE);
		}
	} else {
		rslt = BMI055_E_INVALID_CHANNEL;
	}

	if (rslt == BMI055_OK) {
		/* Configure the Interrupt map register to map high-g interrupt
		to either INT1 or INT2 pin*/
		rslt = bmi055_set_accel_regs(reg_addr, &reg_data, 1, dev);
	}

	return rslt;
}
/*!
 * @brief This internal API writes the data source definition for
 * high-g interrupt - filtered or unfiltered.
 *
 * @note Value = 0(BMI055_FALSE) for filtered data , 1(BMI055_TRUE)
 * for unfiltered data
 */
static int8_t config_int_high_g_src(const struct bmi055_accel_int_sett *acc_int_config, const struct bmi055_dev *dev)
{
	/* Variable to define error */
	int8_t rslt;
	/* Variable to define register address of Interrupt source */
	uint8_t reg_addr = BMI055_ACC_INT_SRC_ADDR;
	/* Variable defined to retrieve data */
	uint8_t data = 0;
	/* Variable defined to set data in register */
	uint8_t reg_data;

	/* Get interrupt source data from the register */
	rslt = bmi055_get_accel_regs(reg_addr, &data, 1, dev);
	if (rslt == BMI055_OK) {
		/* Write the interrupt source data in a variable*/
		reg_data = BMI055_SET_BITS(data, BMI055_ACC_SRC_HIGH_G, acc_int_config->int_data_src_type);

		/* Write data source definition for high-g interrupt */
		rslt = bmi055_set_accel_regs(reg_addr, &reg_data, 1, dev);
	}

	return rslt;
}
/*!
 * @brief This internal API configures registers for high-g interrupt.
 */
static int8_t config_high_g_reg(struct bmi055_accel_int_sett *acc_int_config, const struct bmi055_dev *dev)
{
	/* Variable to define error */
	int8_t rslt;
	/* Variable to define register address */
	uint8_t reg_addr = BMI055_ACC_INT_LH_2_ADDR;
	/* Variable defined to set data in register */
	uint8_t reg_data = 0;
	/* Variable defined to retrieve data */
	uint8_t data = 0;
	/* Array defined to store data */
	uint8_t data_array[3] = {0};

	/* Updating the interrupt structure to local structure */
	struct bmi055_acc_high_g_int_cfg *high_g_int_cfg = &(acc_int_config->acc_int_type_cfg.acc_high_g_int);

	/*Get the high-g interrupt hysteresis setting */
	rslt = bmi055_get_accel_regs(reg_addr, &data, 1, dev);
	if (rslt == BMI055_OK) {
		/* Set default high hysteresis value */
		reg_data = BMI055_SET_BITS(data, BMI055_ACC_HIGH_HYST, high_g_int_cfg->high_hy);

		/* Store high hysteresis value in the array */
		data_array[0] = reg_data;
		/* Store the high duration in the array */
		data_array[1] = high_g_int_cfg->high_dur;
		/* Store the low threshold in the array */
		data_array[2] = high_g_int_cfg->high_thres;

		/* Write the data simultaneously since addresses are located in
		consecutive positions.*/
		rslt = bmi055_set_accel_regs(reg_addr, data_array, 3, dev);
		dev->delay_ms(1);
	}

	return rslt;
}

/*!
 * @brief This internal API sets the slow/no-motion interrupt of the sensor.
 * Slow motion interrupt occurs when slope of atleast one enabled axis exceeds
 * preset threshold for programmable number of samples. No motion interrupt
 * occurs when when slope of all enabled axes is less than the preset threshold
 * for a programmable time.
 */
static int8_t set_accel_slow_no_motion_int(struct bmi055_accel_int_sett *acc_int_config, const struct bmi055_dev *dev)
{
	/* Variable to define error */
	int8_t rslt;

	/* Enable slow/no-motion interrupt */
	rslt = enable_accel_slow_no_motion_int(acc_int_config, dev);
	if (rslt == BMI055_OK) {
		/* Map slow/no-motion interrupt to either INT 1 or 2 */
		rslt = map_accel_slow_no_motion_int(acc_int_config, dev);
		if (rslt == BMI055_OK) {
			/* Write data source definition for slow/no-motion
			interrupt */
			rslt = config_int_slow_no_motion_src(acc_int_config, dev);
			if (rslt == BMI055_OK) {
				/* Configuring interrupt pins */
				rslt = set_int_pin_config(acc_int_config, dev);
				if (rslt == BMI055_OK) {
					/* Configure slow-motion duration
					and threshold */
					rslt = config_slow_no_motion_reg(acc_int_config, dev);
				}
			}
		}
	}

	return rslt;
}

/*!
 * @brief This internal API enables the slow/no-motion interrupt
 *
 * @note For slow_no_mot_sel: 1 -> No-motion; 0-> Slow-motion
 */
static int8_t enable_accel_slow_no_motion_int(struct bmi055_accel_int_sett *acc_int_config,
							const struct bmi055_dev *dev)
{
	/* Variable to define error */
	int8_t rslt;
	/* Variable to define address of Interrupt enable 2 register */
	uint8_t reg_addr = BMI055_ACC_INT_EN_2_ADDR;
	/* Variable defined to retrieve data */
	uint8_t data = 0;
	/* Variable defined to set data in register */
	uint8_t reg_data;

	struct bmi055_acc_slow_no_mot_int_cfg  *slow_no_mot_int_cfg =
			&(acc_int_config->acc_int_type_cfg.acc_slow_no_mot_int);

	/* Get interrupt enable status from the register */
	rslt = bmi055_get_accel_regs(reg_addr, &data, 1, dev);
	if (rslt == BMI055_OK) {
		/* Select between slow and no-motion interrupt */
		reg_data = BMI055_SET_BITS(data,
				BMI055_ACC_SLOW_NO_MOT_SEL_INT_EN_2, slow_no_mot_int_cfg->slow_no_mot_sel);

		/* Enabling slow/no-motion x axis */
		reg_data = BMI055_SET_BIT_POS0(reg_data,
				BMI055_ACC_SLOW_NO_MOT_X_INT_EN_2, slow_no_mot_int_cfg->slow_no_mot_en_x);

		/* Enabling slow/no-motion y axis */
		reg_data = BMI055_SET_BITS(reg_data,
				BMI055_ACC_SLOW_NO_MOT_Y_INT_EN_2, slow_no_mot_int_cfg->slow_no_mot_en_y);

		/* Enabling slow/no-motion z axis */
		reg_data = BMI055_SET_BITS(reg_data,
				BMI055_ACC_SLOW_NO_MOT_Z_INT_EN_2, slow_no_mot_int_cfg->slow_no_mot_en_z);

		/* Enable slow/no-motion interrupt in Int Enable 2 register */
		rslt = bmi055_set_accel_regs(reg_addr, &reg_data, 1, dev);
	}

	return rslt;
}

/*!
 * @brief This internal API maps the slow/no-motion interrupt to either
 * INT pin 1 or 2
 */
static int8_t map_accel_slow_no_motion_int(const struct bmi055_accel_int_sett *acc_int_config,
							const struct bmi055_dev *dev)
{
	/* Variable to define error */
	int8_t rslt;
	/* Variable to define register address */
	uint8_t reg_addr = 0;
	/* Variable defined to retrieve data */
	uint8_t data = 0;
	/* Variable defined to set data in register */
	uint8_t reg_data;


	if (acc_int_config->acc_int_channel == BMI055_INT_CHANNEL_1) {
		/* Define Interrupt map 0 register */
		reg_addr = BMI055_ACC_INT_MAP_0_ADDR;

		/* Get interrupt map status from the register */
		rslt = bmi055_get_accel_regs(reg_addr, &data, 1, dev);
		if (rslt == BMI055_OK) {
			/* Map slow/no-motion interrupt to INT1 pin */
			reg_data = BMI055_SET_BITS(data,
					BMI055_ACC_INT1_MAP_0_SLOW_NO_MOT, BMI055_ENABLE);
		}
	} else if (acc_int_config->acc_int_channel == BMI055_INT_CHANNEL_2) {
		/* Define Interrupt map 2 register */
		reg_addr = BMI055_ACC_INT_MAP_2_ADDR;

		/* Get interrupt map status from the register */
		rslt = bmi055_get_accel_regs(reg_addr, &data, 1, dev);
		if (rslt == BMI055_OK) {
			/* Map slow/no-motion interrupt to INT2 pin */
			reg_data = BMI055_SET_BITS(data,
				BMI055_ACC_INT2_MAP_2_SLOW_NO_MOT, BMI055_ENABLE);
		}
	} else {
		rslt = BMI055_E_INVALID_CHANNEL;
	}

	if (rslt == BMI055_OK) {
		/* Configure the Interrupt map register to map
		slow/no-motion interrupt to either INT1 or INT2 pin */
		rslt = bmi055_set_accel_regs(reg_addr, &reg_data, 1, dev);
	}

	return rslt;
}

/*!
 * @brief This internal API writes the data source definition for
 * slow/no-motion interrupt - filtered or unfiltered.
 *
 * @note Value = 0(BMI055_FALSE) for filtered data , 1(BMI055_TRUE)
 * for unfiltered data
 */
static int8_t config_int_slow_no_motion_src(const struct bmi055_accel_int_sett *acc_int_config,
								const struct bmi055_dev *dev)
{
	/* Variable to define error */
	int8_t rslt;
	/* Variable to define register address of Interrupt source */
	uint8_t reg_addr = BMI055_ACC_INT_SRC_ADDR;
	/* Variable defined to retrieve data */
	uint8_t data = 0;
	/* Variable defined to set data in register */
	uint8_t reg_data;

	/* Get interrupt source data from the register */
	rslt = bmi055_get_accel_regs(reg_addr, &data, 1, dev);
	if (rslt == BMI055_OK) {
		/* Write the interrupt source data in a variable*/
		reg_data = BMI055_SET_BITS(data, BMI055_ACC_SRC_SLOW_NO_MOT,
							acc_int_config->int_data_src_type);

		/* Write data source definition for slow/no-motion interrupt */
		rslt = bmi055_set_accel_regs(reg_addr, &reg_data, 1, dev);
	}

	return rslt;
}

/*!
 * @brief This internal API configures registers for slow/no-motion interrupt.
 */
static int8_t config_slow_no_motion_reg(struct bmi055_accel_int_sett *acc_int_config, const struct bmi055_dev *dev)
{
	/* Variable to define error */
	int8_t rslt;

	/* Configure duration for slow/no-motion interrupt */
	rslt = config_slow_no_motion_dur(acc_int_config, dev);
	if (rslt == BMI055_OK) {
		/* Configure threshold for slow/no-motion interrupt */
		rslt = config_slow_no_motion_thres(acc_int_config, dev);
	}

	return rslt;
}

/*!
 * @brief This internal API configures duration for slow/no-motion interrupt.
 */
static int8_t config_slow_no_motion_dur(struct bmi055_accel_int_sett *acc_int_config, const struct bmi055_dev *dev)
{
	/* Variable to define error */
	int8_t rslt;
	/* Variable to define register address */
	uint8_t reg_addr = BMI055_ACC_INT_MOT_0_ADDR;
	/* Variable defined to set data in register */
	uint8_t reg_data = 0;
	/* Variable defined to retrieve data */
	uint8_t data = 0;

	/* Updating the interrupt structure to local structure */
	struct bmi055_acc_slow_no_mot_int_cfg  *slow_no_mot_int_cfg =
			&(acc_int_config->acc_int_type_cfg.acc_slow_no_mot_int);

	/*Get the slow/no-motion duration setting */
	rslt = bmi055_get_accel_regs(reg_addr, &data, 1, dev);
	if (rslt == BMI055_OK) {
		/* If slow-motion */
		if (slow_no_mot_int_cfg->slow_no_mot_sel == 0) {
			/* Set slow motion duration value */
			reg_data = BMI055_SET_BITS(data,
				BMI055_ACC_SLOW_DUR, slow_no_mot_int_cfg->slow_mot_dur);

			rslt = bmi055_set_accel_regs(reg_addr, &reg_data, 1, dev);
		} else if (slow_no_mot_int_cfg->slow_no_mot_sel == 1) {
			/* Set no-motion duration value */
			reg_data = BMI055_SET_BITS(data,
				BMI055_ACC_NO_MOTION_DUR, slow_no_mot_int_cfg->no_mot_dur);

			rslt = bmi055_set_accel_regs(reg_addr, &reg_data, 1, dev);
		} else {
			rslt = BMI055_E_INVALID_INPUT;
		}
	}

	return rslt;
}

/*!
 * @brief This internal API configures threshold for slow/no-motion interrupt.
 */
static int8_t config_slow_no_motion_thres(struct bmi055_accel_int_sett *acc_int_config, const struct bmi055_dev *dev)
{
	/* Variable to define error */
	int8_t rslt;
	/* Variable to define register address */
	uint8_t reg_addr = BMI055_ACC_INT_MOT_2_ADDR;
	/* Variable defined to set data in register */
	uint8_t reg_data;

	/* Updating the interrupt structure to local structure */
	struct bmi055_acc_slow_no_mot_int_cfg  *slow_no_mot_int_cfg =
			&(acc_int_config->acc_int_type_cfg.acc_slow_no_mot_int);

	/* Set slow/no-motion threshold value */
	reg_data = slow_no_mot_int_cfg->slow_no_mot_thres;

	rslt = bmi055_set_accel_regs(reg_addr, &reg_data, 1, dev);

	return rslt;
}

/*!
 * @brief This internal API sets the slope interrupt of the sensor.This
 * interrupt occurs when slope(absolute value of acceleration difference)
 * exceeds a preset threshold.
 */
static int8_t set_accel_slope_int(struct bmi055_accel_int_sett *acc_int_config,  const struct bmi055_dev *dev)
{
	/* Variable to define error */
	int8_t rslt;

	/* Enable slope interrupt */
	rslt = enable_accel_slope_int(acc_int_config, dev);
	if (rslt == BMI055_OK) {
		/* Map slope interrupt to either INT 1 or 2 */
		rslt = map_accel_slope_int(acc_int_config, dev);
		if (rslt == BMI055_OK) {
			/* Write data source definition for slope
			interrupt */
			rslt = config_int_slope_src(acc_int_config, dev);
			if (rslt == BMI055_OK) {
				/* Configuring interrupt pins */
				rslt = set_int_pin_config(acc_int_config, dev);
				if (rslt == BMI055_OK) {
					/* Configure slope registers */
					rslt = config_slope_reg(acc_int_config, dev);
				}
			}
		}
	}

	return rslt;
}

/*!
 * @brief This internal API enables the slope interrupt
 */
static int8_t enable_accel_slope_int(struct bmi055_accel_int_sett *acc_int_config, const struct bmi055_dev *dev)
{
	/* Variable to define error */
	int8_t rslt;
	/* Variable to define address of Interrupt enable 0 register */
	uint8_t reg_addr = BMI055_ACC_INT_EN_0_ADDR;
	/* Variable defined to retrieve data */
	uint8_t data = 0;
	/* Variable defined to set data in register */
	uint8_t reg_data;

	struct bmi055_acc_slop_int_cfg  *slope_int_cfg = &(acc_int_config->acc_int_type_cfg.acc_slope_int);

	/* Get interrupt enable status from the register */
	rslt = bmi055_get_accel_regs(reg_addr, &data, 1, dev);
	if (rslt == BMI055_OK) {
		/* Enabling slope x axis */
		reg_data = BMI055_SET_BIT_POS0(data, BMI055_ACC_ANY_MOTION_X_INT_EN_0, slope_int_cfg->slope_en_x);

		/* Enabling slope y axis */
		reg_data = BMI055_SET_BITS(reg_data, BMI055_ACC_ANY_MOTION_Y_INT_EN_0, slope_int_cfg->slope_en_y);

		/* Enabling slope z axis */
		reg_data = BMI055_SET_BITS(reg_data, BMI055_ACC_ANY_MOTION_Z_INT_EN_0, slope_int_cfg->slope_en_z);

		/* Enable slope interrupt in Int Enable 0 register */
		rslt = bmi055_set_accel_regs(reg_addr, &reg_data, 1, dev);
	}

	return rslt;
}

/*!
 * @brief This internal API maps the slope interrupt to either
 * INT pin 1 or 2
 */
static int8_t map_accel_slope_int(const struct bmi055_accel_int_sett *acc_int_config, const struct bmi055_dev *dev)
{
	/* Variable to define error */
	int8_t rslt;
	/* Variable to define register address */
	uint8_t reg_addr = 0;
	/* Variable defined to retrieve data */
	uint8_t data = 0;
	/* Variable defined to set data in register */
	uint8_t reg_data;

	if (acc_int_config->acc_int_channel == BMI055_INT_CHANNEL_1) {
		/* Define Interrupt map 0 register */
		reg_addr = BMI055_ACC_INT_MAP_0_ADDR;

		/* Get interrupt map status from the register */
		rslt = bmi055_get_accel_regs(reg_addr, &data, 1, dev);
		if (rslt == BMI055_OK) {
			/* Map slope interrupt to INT1 pin */
			reg_data = BMI055_SET_BITS(data, BMI055_ACC_INT1_MAP_0_SLOPE, BMI055_ENABLE);
		}
	} else if (acc_int_config->acc_int_channel == BMI055_INT_CHANNEL_2) {
		/* Define Interrupt map 2 register */
		reg_addr = BMI055_ACC_INT_MAP_2_ADDR;

		/* Get interrupt map status from the register */
		rslt = bmi055_get_accel_regs(reg_addr, &data, 1, dev);
		if (rslt == BMI055_OK) {
			/* Map slope interrupt to INT2 pin */
			reg_data = BMI055_SET_BITS(data, BMI055_ACC_INT2_MAP_2_SLOPE, BMI055_ENABLE);
		}
	} else {
		rslt = BMI055_E_INVALID_CHANNEL;
	}

	if (rslt == BMI055_OK) {
		/* Configure the Interrupt map register to map slope
		interrupt to either INT1 or INT2 pin*/
		rslt = bmi055_set_accel_regs(reg_addr, &reg_data, 1, dev);
	}

	return rslt;
}
/*!
 * @brief This internal API writes the data source definition for
 * slope interrupt - filtered or unfiltered.
 *
 * @note Value = 0(BMI055_FALSE) for filtered data , 1(BMI055_TRUE)
 * for unfiltered data
 */
static int8_t config_int_slope_src(const struct bmi055_accel_int_sett *acc_int_config, const struct bmi055_dev *dev)
{
	/* Variable to define error */
	int8_t rslt;
	/* Variable to define register address of Interrupt source */
	uint8_t reg_addr = BMI055_ACC_INT_SRC_ADDR;
	/* Variable defined to retrieve data */
	uint8_t data = 0;
	/* Variable defined to set data in register */
	uint8_t reg_data;

	/* Get interrupt source data from the register */
	rslt = bmi055_get_accel_regs(reg_addr, &data, 1, dev);
	if (rslt == BMI055_OK) {
		/* Write the interrupt source data in a variable*/
		reg_data = BMI055_SET_BITS(data, BMI055_ACC_SRC_SLOPE, acc_int_config->int_data_src_type);

		/* Write data source definition for slope interrupt */
		rslt = bmi055_set_accel_regs(reg_addr, &reg_data, 1, dev);
	}

	return rslt;
}
/*!
 * @brief This internal API configures registers for slope interrupt.
 */
static int8_t config_slope_reg(struct bmi055_accel_int_sett *acc_int_config, const struct bmi055_dev *dev)
{
	/* Variable to define error */
	int8_t rslt;
	/* Variable to define register address */
	uint8_t reg_addr = BMI055_ACC_INT_MOT_0_ADDR;
	/* Variable defined to set data in register */
	uint8_t reg_data = 0;
	/* Variable defined to retrieve data */
	uint8_t data = 0;
	/* Array defined to store data */
	uint8_t data_array[2] = {0};

	/* Updating the interrupt structure to local structure */
	struct bmi055_acc_slop_int_cfg  *slope_int_cfg = &(acc_int_config->acc_int_type_cfg.acc_slope_int);

	/*Get the slope interrupt duration */
	rslt = bmi055_get_accel_regs(reg_addr, &data, 1, dev);
	if (rslt == BMI055_OK) {
		/* Set the slope duration */
		reg_data = BMI055_SET_BIT_POS0(data, BMI055_ACC_SLOPE_DUR, slope_int_cfg->slope_dur);

		/* Store the duration in the array */
		data_array[0] = reg_data;

		/* Store the threshold in the array */
		data_array[1] = slope_int_cfg->slope_thr;

		/* Write the data simultaneously since addresses are located in
		consecutive positions.*/
		rslt = bmi055_set_accel_regs(reg_addr, data_array, 2, dev);
		dev->delay_ms(1);
	}

	return rslt;
}

/*!
  *@brief This internal API sets the orientation interrupt of the sensor.This
  * interrupt occurs when there is orientation change in the sensor
  * with respect to gravitational field vector g.
 */
static int8_t set_accel_orientation_int(struct bmi055_accel_int_sett *acc_int_config,  const struct bmi055_dev *dev)
{
	/* Variable to define error */
	int8_t rslt;

	/* Enable orientation interrupt */
	rslt = enable_accel_orient_int(acc_int_config, dev);
	if (rslt == BMI055_OK) {
		/* Map orientation interrupt to either INT 1 or 2 */
		rslt = map_accel_orient_int(acc_int_config, dev);
		if (rslt == BMI055_OK) {
			/* Configuring interrupt pins */
			rslt = set_int_pin_config(acc_int_config, dev);
			if (rslt == BMI055_OK) {
				/* Configure orientation registers */
				rslt = config_orient_reg(acc_int_config, dev);
			}
		}
	}

	return rslt;
}

/*!
 * @brief This internal API enables the orientation interrupt
 */
static int8_t enable_accel_orient_int(struct bmi055_accel_int_sett *acc_int_config, const struct bmi055_dev *dev)
{
	/* Variable to define error */
	int8_t rslt;
	/* Variable to define address of Interrupt enable 0 register */
	uint8_t reg_addr = BMI055_ACC_INT_EN_0_ADDR;
	/* Variable defined to retrieve data */
	uint8_t data = 0;
	/* Variable defined to set data in register */
	uint8_t reg_data;

	/* Updating the interrupt structure to local structure */
	struct bmi055_acc_orient_int_cfg *orient_int_cfg = &(acc_int_config->acc_int_type_cfg.acc_orient_int);

	/* Get interrupt enable status from the register */
	rslt = bmi055_get_accel_regs(reg_addr, &data, 1, dev);
	if (rslt == BMI055_OK) {
		/* Write interrupt enable bits in the variable */
		reg_data = BMI055_SET_BITS(data, BMI055_ACC_ORIENT_INT_EN_0, orient_int_cfg->orient_en);

		/* Enable orientation interrupt in Interrupt Enable 1
		register */
		rslt = bmi055_set_accel_regs(reg_addr, &reg_data, 1, dev);
	}

	return rslt;
}

/*!
 * @brief This internal API maps the orientation interrupt to either
 * INT pin 1 or 2
 */
static int8_t map_accel_orient_int(const struct bmi055_accel_int_sett *acc_int_config, const struct bmi055_dev *dev)
{
	/* Variable to define error */
	int8_t rslt;
	/* Variable to define register address */
	uint8_t reg_addr = 0;
	/* Variable defined to retrieve data */
	uint8_t data = 0;
	/* Variable defined to set data in register */
	uint8_t reg_data = 0;


	if (acc_int_config->acc_int_channel == BMI055_INT_CHANNEL_1) {
		/* Define Interrupt map 0 register */
		reg_addr = BMI055_ACC_INT_MAP_0_ADDR;

		/* Get interrupt map status from the register */
		rslt = bmi055_get_accel_regs(reg_addr, &data, 1, dev);
		if (rslt == BMI055_OK) {
			/* Map orientation interrupt to INT1 pin */
			reg_data = BMI055_SET_BITS(data, BMI055_ACC_INT1_MAP_0_ORIENT, BMI055_ENABLE);
		}
	} else if (acc_int_config->acc_int_channel == BMI055_INT_CHANNEL_2) {
		/* Define Interrupt map 2 register */
		reg_addr = BMI055_ACC_INT_MAP_2_ADDR;

		/* Get interrupt map status from the register */
		rslt = bmi055_get_accel_regs(reg_addr, &data, 1, dev);
		if (rslt == BMI055_OK) {
			/* Map orientation interrupt to INT2 pin */
			reg_data = BMI055_SET_BITS(data, BMI055_ACC_INT2_MAP_2_ORIENT, BMI055_ENABLE);
		}
	} else {
		rslt = BMI055_E_INVALID_CHANNEL;
	}

	if (rslt == BMI055_OK) {
		/* Configure the Interrupt map register to map orientation
		interrupt to either INT1 or INT2 pin*/
		rslt = bmi055_set_accel_regs(reg_addr, &reg_data, 1, dev);
	}

	return rslt;
}

/*!
 * @brief This internal API configures registers for orientation interrupt
 */
static int8_t config_orient_reg(struct bmi055_accel_int_sett *acc_int_config, const struct bmi055_dev *dev)
{
	/* Variable to define error */
	int8_t rslt;
	/* Variable to define register address */
	uint8_t reg_addr;
	/* Variable defined to set data in register */
	uint8_t reg_data = 0;
	/* Variable defined to retrieve data */
	uint8_t data = 0;
	/* Array defined to store data */
	uint8_t data_array[2] = {0};

	/* Updating the interrupt structure to local structure */
	struct bmi055_acc_orient_int_cfg *orient_int_cfg = &(acc_int_config->acc_int_type_cfg.acc_orient_int);

	/* Configuring orientation registers */
	reg_addr = BMI055_ACC_INT_ORIENT_0_ADDR;

	/* Get the previous settings */
	rslt = bmi055_get_accel_regs(reg_addr, &data, 1, dev);
	if (rslt == BMI055_OK) {
		/* Set orientation hysteresis */
		reg_data = BMI055_SET_BITS(data, BMI055_ACC_ORIENT_HYST, orient_int_cfg->orient_hyst);
		/* Set orient blocking */
		reg_data = BMI055_SET_BITS(reg_data, BMI055_ACC_ORIENT_BLOCKING, orient_int_cfg->orient_blocking);
		/* Set orient mode */
		reg_data = BMI055_SET_BIT_POS0(reg_data, BMI055_ACC_ORIENT_MODE, orient_int_cfg->orient_mode);

		/* Store orientation hysteresis, blocking and
		mode in the array */
		data_array[0] = reg_data;

		/* Configuring rest of the settings in another
		register */
		reg_addr = BMI055_ACC_INT_ORIENT_1_ADDR;

		/* Get the previous settings */
		rslt = bmi055_get_accel_regs(reg_addr, &data, 1, dev);
		if (rslt == BMI055_OK) {
			/* Set up/down masking : '0' -> generates interrupt
			only with x-y plane */
			reg_data = BMI055_SET_BITS(data, BMI055_ACC_ORIENT_UD_EN, orient_int_cfg->orient_ud_en);
			/* Set orient theta */
			reg_data = BMI055_SET_BIT_POS0(reg_data, BMI055_ACC_ORIENT_THETA, orient_int_cfg->orient_theta);

			/* Store up/down enable and theta angle in the
			next byte */
			data_array[1] = reg_data;

			/* Write the data simultaneously since addresses are
			located in consecutive positions.*/
			rslt = bmi055_set_accel_regs(BMI055_ACC_INT_ORIENT_0_ADDR, data_array, 2, dev);
			dev->delay_ms(1);
		}
	}

	return rslt;
}

/*!
 * @brief This internal API configures the pins to fire the interrupt signal
 * when it occurs.
 */
static int8_t set_int_pin_config(struct bmi055_accel_int_sett *acc_int_config, const struct bmi055_dev *dev)
{
	/* Variable to define error */
	int8_t rslt;
	/* Variable to define register address */
	uint8_t reg_addr = BMI055_ACC_INT_OUT_CTRL_ADDR;
	/* Variable defined to retrieve and set data in the register */
	uint8_t reg_data_array[2] = {0};
	/* Variable defined to store data temporarily */
	uint8_t temp_data = 0;
	/* Variable defined to get data temporarily */
	uint8_t temp_out_data = 0;

	/* Update the local structure with the pin configuration
	structure */
	struct bmi055_int_pin_sett *int_pin_cfg = &(acc_int_config->int_pin_sett);

	/* Get the electrical behavior configuration of interrupt pin */
	rslt = bmi055_get_accel_regs(reg_addr, reg_data_array, 2, dev);
	if (rslt == BMI055_OK) {
		/* Get the first byte of the data array */
		temp_data = reg_data_array[0];

		/* Configure input output configuration of interrupt pins */
		rslt = config_in_out_int(int_pin_cfg, acc_int_config, temp_data, &temp_out_data);
		if (rslt == BMI055_OK) {
			/* Copy register value to the first byte of array */
			reg_data_array[0] = temp_out_data;

			/* Get the second byte of the data array */
			temp_data = reg_data_array[1];

			/* Configure latch and reset configuration of
			interrupt pins */
			config_reset_latch_int(int_pin_cfg, temp_data, &temp_out_data);

			/* Copy register value to the second byte of array */
			reg_data_array[1] = temp_out_data;

			/* Write data to INT_OUT_CTRL and INT_RST_LATCH
			simultaneously as they lie in consecutive places */
			rslt = bmi055_set_accel_regs(reg_addr, reg_data_array, 2, dev);
		}
	}

	return rslt;
}

/*!
 * @brief This internal API configures the input output configuration of the
 * interrupt pins
 */
static int8_t config_in_out_int(const struct bmi055_int_pin_sett *int_pin_cfg,
					const struct bmi055_accel_int_sett *acc_int_config, uint8_t temp_data,
						uint8_t *temp_out_data)
{
	/* Variable to define error */
	int8_t rslt = 0;

	if (acc_int_config->acc_int_channel == BMI055_INT_CHANNEL_1) {
		/* Configuring output type for INT1 pin */
		temp_data = BMI055_SET_BIT_POS0(temp_data, BMI055_ACC_INT1_LVL,
							int_pin_cfg->output_type);

		/* Configuring output mode for INT1 pin */
		temp_data = BMI055_SET_BITS(temp_data, BMI055_ACC_INT1_OD,
							int_pin_cfg->output_mode);
	} else if (acc_int_config->acc_int_channel == BMI055_INT_CHANNEL_2) {
		/* Configuring output type for INT2 pin */
		temp_data = BMI055_SET_BITS(temp_data, BMI055_ACC_INT2_LVL,
							int_pin_cfg->output_type);

		/* Configuring output mode for INT2 pin */
		temp_data = BMI055_SET_BITS(temp_data, BMI055_ACC_INT2_OD,
							int_pin_cfg->output_mode);
	} else {
		rslt = BMI055_E_INVALID_CHANNEL;
	}

	if (rslt == BMI055_OK)
		*temp_out_data = temp_data;

	return rslt;
}

/*!
 * @brief This internal API configures the latch and the reset configuration of
 * the interrupt pins
 */
static void config_reset_latch_int(const struct bmi055_int_pin_sett *int_pin_cfg,
						uint8_t temp_data, uint8_t *temp_out_data)
{
	/* Configuring Latch duration */
	temp_data = BMI055_SET_BIT_POS0(temp_data, BMI055_ACC_LATCH_DUR,
						int_pin_cfg->latch_dur);

	/* Configuring interrupt reset - '1' clears all latched
	interrupts and '0' keeps latched interrupt active */
	temp_data = BMI055_SET_BITS(temp_data, BMI055_ACC_RESET_INT,
						int_pin_cfg->reset_int);

	*temp_out_data = temp_data;
}

/*!
 * @brief This internal API enables/disables all accelerometer interrupts.
 */
static int8_t enable_disable_all_accel_int(uint8_t set_en_bit, const struct bmi055_dev *dev)
{
	/* Variable to define error */
	int8_t rslt;
	/* Variable to define register address */
	uint8_t reg_addr = BMI055_ACC_INT_EN_0_ADDR;
	/* Variable defined to store data in the register*/
	uint8_t reg_data[3] = {0};
	/* Variable defined for loop*/
	uint8_t count = 0;

	if (set_en_bit == 1) {
		while (count < 3)
			reg_data[count++] = 0xFF;
	}

	/* Write '0XFF' to enable or '0' to  disable
	all interrupt registers */
	rslt = bmi055_set_accel_regs(reg_addr, reg_data, 3, dev);

	return rslt;
}

/*!
 * @brief This internal API is used to validate the device structure pointer for
 * null conditions.
 */
static int8_t bmi055_accel_null_ptr_check(const struct bmi055_dev *dev)
{
	/* Variable to define error */
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

/** @}*/
