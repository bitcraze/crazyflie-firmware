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
 * @file    bmi160.h
 * @date    6 Apr 2017
 * @version 3.4.0
 * @brief
 *
 */

/*!
 * @defgroup bmi160
 * @brief
 * @{*/

#ifndef BMI160_H_
#define BMI160_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "bmi160_defs.h"
#ifdef __KERNEL__
#include <bmi160_math.h>
#else
#include <math.h>
#include <string.h>
#include <stdlib.h>
#endif


/*********************** User function prototypes ************************/

/*!
 *  @brief This API is the entry point for sensor.It performs
 *  the selection of I2C/SPI read mechanism according to the
 *  selected interface and reads the chip-id of bmi160 sensor.
 *
 *  @param[in,out] dev : Structure instance of bmi160_dev
 *  @note : Refer user guide for detailed info.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
int8_t bmi160_init(struct bmi160_dev *dev);

/*!
 * @brief This API reads the data from the given register address of sensor.
 *
 * @param[in] reg_addr  : Register address from where the data to be read
 * @param[out] data     : Pointer to data buffer to store the read data.
 * @param[in] len       : No of bytes of data to be read.
 * @param[in] dev       : Structure instance of bmi160_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
int8_t bmi160_get_regs(uint8_t reg_addr, uint8_t *data, uint16_t len, const struct bmi160_dev *dev);

/*!
 * @brief This API writes the given data to the register address
 * of sensor.
 *
 * @param[in] reg_addr  : Register address from where the data to be written.
 * @param[in] data      : Pointer to data buffer which is to be written
 * in the sensor.
 * @param[in] len       : No of bytes of data to write..
 * @param[in] dev       : Structure instance of bmi160_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
int8_t bmi160_set_regs(uint8_t reg_addr, uint8_t *data, uint16_t len, const struct bmi160_dev *dev);

/*!
 * @brief This API resets and restarts the device.
 * All register values are overwritten with default parameters.
 *
 * @param[in] dev  : Structure instance of bmi160_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
int8_t bmi160_soft_reset(const struct bmi160_dev *dev);

/*!
 * @brief This API configures the power mode, range and bandwidth
 * of sensor.
 *
 * @param[in] dev    : Structure instance of bmi160_dev.
 * @note : Refer user guide for detailed info.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
int8_t bmi160_set_sens_conf(struct bmi160_dev *dev);

/*!
 * @brief This API sets the power mode of the sensor.
 *
 * @param[in] dev  : Structure instance of bmi160_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
int8_t bmi160_set_power_mode(struct bmi160_dev *dev);

/*!
 * @brief This API reads sensor data, stores it in
 * the bmi160_sensor_data structure pointer passed by the user.
 * The user can ask for accel data ,gyro data or both sensor
 * data using bmi160_select_sensor enum
 *
 * @param[in] select_sensor    : enum to choose accel,gyro or both sensor data
 * @param[out] accel    : Structure pointer to store accel data
 * @param[out] gyro     : Structure pointer to store gyro data
 * @param[in] dev       : Structure instance of bmi160_dev.
 * @note : Refer user guide for detailed info.
 *
 * @return Result of API execution status
 * @retval zero -> Success  / -ve value -> Error
 */
int8_t bmi160_get_sensor_data(uint8_t select_sensor, struct bmi160_sensor_data *accel,
				struct bmi160_sensor_data *gyro, const struct bmi160_dev *dev);

/*!
 * @brief This API configures the necessary interrupt based on
 *  the user settings in the bmi160_int_settg structure instance.
 *
 * @param[in] int_config  : Structure instance of bmi160_int_settg.
 * @param[in] dev         : Structure instance of bmi160_dev.
 * @note : Refer user guide for detailed info.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
int8_t bmi160_set_int_config(struct bmi160_int_settg *int_config, struct bmi160_dev *dev);

/*!
 * @brief This API enables the step counter feature.
 *
 * @param[in] step_cnt_enable	: value to enable or disable
 * @param[in] dev		: Structure instance of bmi160_dev.
 * @note : Refer user guide for detailed info.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
int8_t bmi160_set_step_counter(uint8_t step_cnt_enable, const struct bmi160_dev *dev);

/*!
 * @brief This API reads the step counter value.
 *
 * @param[in] step_val	  : Pointer to store the step counter value.
 * @param[in] dev         : Structure instance of bmi160_dev.
 * @note : Refer user guide for detailed info.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
int8_t bmi160_read_step_counter(uint16_t *step_val, const struct bmi160_dev *dev);

/*!
 * @brief This API reads the mention no of byte of data from the given
 * register address of auxiliary sensor.
 *
 * @param[in] reg_addr	  : Address of register to read.
 * @param[in] aux_data	  : Pointer to store the read data.
 * @param[in] len	  : No of bytes to read.
 * @param[in] dev         : Structure instance of bmi160_dev.
 * @note : Refer user guide for detailed info.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
int8_t bmi160_aux_read(uint8_t reg_addr, uint8_t *aux_data, uint16_t len, const struct bmi160_dev *dev);

/*!
 * @brief This API writes the mention no of byte of data to the given
 * register address of auxiliary sensor.
 *
 * @param[in] reg_addr	  : Address of register to write.
 * @param[in] aux_data	  : Pointer to write data.
 * @param[in] len	  : No of bytes to write.
 * @param[in] dev         : Structure instance of bmi160_dev.
 * @note : Refer user guide for detailed info.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
int8_t bmi160_aux_write(uint8_t reg_addr, uint8_t *aux_data, uint16_t len, const struct bmi160_dev *dev);

/*!
 * @brief This API initialize the auxiliary sensor
 * in order to access it.
 *
 * @param[in] dev         : Structure instance of bmi160_dev.
 * @note : Refer user guide for detailed info.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
int8_t bmi160_aux_init(const struct bmi160_dev *dev);

/*!
 * @brief This API is used to setup the auxiliary sensor of bmi160 in auto mode
 * Thus enabling the auto update of 8 bytes of data from auxiliary sensor
 * to BMI160 register address 0x04 to 0x0B
 *
 * @param[in] data_addr	   : Starting address of aux. sensor's data register
 *                           (BMI160 registers 0x04 to 0x0B will be updated
 *                           with 8 bytes of data from auxiliary sensor 
 *                           starting from this register address.)
 * @param[in] dev	   : Structure instance of bmi160_dev.
 * 
 * @note : Set the value of auxiliary polling rate by setting
 *         dev->aux_cfg.aux_odr to the required value from the table
 *         before calling this API
 *
 *   dev->aux_cfg.aux_odr  |   Auxiliary ODR (Hz)
 *  -----------------------|-----------------------
 *  BMI160_AUX_ODR_0_78HZ  |        25/32
 *  BMI160_AUX_ODR_1_56HZ  |        25/16 
 *  BMI160_AUX_ODR_3_12HZ  |        25/8 
 *  BMI160_AUX_ODR_6_25HZ  |        25/4 
 *  BMI160_AUX_ODR_12_5HZ  |        25/2 
 *  BMI160_AUX_ODR_25HZ    |        25 
 *  BMI160_AUX_ODR_50HZ    |        50 
 *  BMI160_AUX_ODR_100HZ   |        100 
 *  BMI160_AUX_ODR_200HZ   |        200 
 *  BMI160_AUX_ODR_400HZ   |        400 
 *  BMI160_AUX_ODR_800HZ   |        800 
 *
 * @note : Other values of  dev->aux_cfg.aux_odr are reserved and not for use
 *
 * @return Result of API execution status
 * @retval zero -> Success  / -ve value -> Error
 */
int8_t bmi160_set_aux_auto_mode(uint8_t* data_addr, struct bmi160_dev *dev);

/*!
 * @brief This API configures the 0x4C register and settings like
 * Auxiliary sensor manual enable/ disable and aux burst read length.
 *
 * @param[in] dev    : Structure instance of bmi160_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success  / -ve value -> Error
 */
int8_t bmi160_config_aux_mode(const struct bmi160_dev *dev);

/*!
 * @brief This API is used to read the raw uncompensated auxiliary sensor 
 * data of 8 bytes from BMI160 register address 0x04 to 0x0B
 *
 * @param[in] aux_data	     : Pointer to user array of length 8 bytes
 *                             Ensure that the aux_data array is of 
 *                             length 8 bytes
 * @param[in] dev	     : Structure instance of bmi160_dev
 *
 * @return Result of API execution status
 * @retval zero -> Success  / -ve value -> Error
 */
int8_t bmi160_read_aux_data_auto_mode(uint8_t *aux_data, const struct bmi160_dev *dev);

/*!
 *  @brief This API reads the data from fifo buffer.
 *
 *  @note User has to allocate the FIFO buffer along with
 *  corresponding fifo length from his side before calling this API
 *  as mentioned in the readme.md
 *
 *  @note User must specify the number of bytes to read from the FIFO in
 *  dev->fifo->length , It will be updated by the number of bytes actually
 *  read from FIFO after calling this API
 *
 *  @param[in] dev     : Structure instance of bmi160_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 *
 */
int8_t bmi160_get_fifo_data(struct bmi160_dev const *dev);

/*!
 *  @brief This API writes fifo_flush command to command register.This
 *  action clears all data in the Fifo without changing fifo configuration
 *  settings.
 *
 *  @param[in] dev     : Structure instance of bmi160_dev
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 *
 */
int8_t bmi160_set_fifo_flush(const struct bmi160_dev *dev);

/*! @brief This API sets the FIFO configuration in the sensor.
 *
 *  @param[in] config : variable used to specify the FIFO
 *  configurations which are to be enabled or disabled in the sensor.
 *
 *  @note : User can set either set one or more or all FIFO configurations
 *  by ORing the below mentioned macros.
 *      config                  |   Value
 *      ------------------------|---------------------------
 *      BMI160_FIFO_TIME        |   0x02
 *      BMI160_FIFO_TAG_INT2    |   0x04
 *      BMI160_FIFO_TAG_INT1    |   0x08
 *      BMI160_FIFO_HEADER      |   0x10
 *      BMI160_FIFO_AUX         |   0x20
 *      BMI160_FIFO_ACCEL	|   0x40
 *      BMI160_FIFO_GYRO        |   0x80
 *
 *  @param[in] enable : Parameter used to enable or disable the above
 *  FIFO configuration
 *  @param[in] dev : Structure instance of bmi160_dev.
 *
 *  @return status of bus communication result
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 *
 */
int8_t bmi160_set_fifo_config(uint8_t config, uint8_t enable, struct bmi160_dev const *dev);

/*! @brief This API is used to configure the down sampling ratios of
 *  the accel and gyro data for FIFO.Also, it configures filtered or
 *  pre-filtered data for the fifo for accel and gyro.
 *
 *  @param[in] fifo_down : variable used to specify the FIFO down
 *  configurations which are to be enabled or disabled in the sensor.
 *
 *  @note The user must select one among the following macros to
 *  select down-sampling ratio for accel
 *      config                               |   Value
 *      -------------------------------------|---------------------------
 *      BMI160_ACCEL_FIFO_DOWN_ZERO          |   0x00
 *      BMI160_ACCEL_FIFO_DOWN_ONE           |   0x10
 *      BMI160_ACCEL_FIFO_DOWN_TWO           |   0x20
 *      BMI160_ACCEL_FIFO_DOWN_THREE         |   0x30
 *      BMI160_ACCEL_FIFO_DOWN_FOUR          |   0x40
 *      BMI160_ACCEL_FIFO_DOWN_FIVE          |   0x50
 *      BMI160_ACCEL_FIFO_DOWN_SIX           |   0x60
 *      BMI160_ACCEL_FIFO_DOWN_SEVEN         |   0x70
 *
 *  @note The user must select one among the following macros to
 *  select down-sampling ratio for gyro
 *
 *      config                               |   Value
 *      -------------------------------------|---------------------------
 *      BMI160_GYRO_FIFO_DOWN_ZERO           |   0x00
 *      BMI160_GYRO_FIFO_DOWN_ONE            |   0x01
 *      BMI160_GYRO_FIFO_DOWN_TWO            |   0x02
 *      BMI160_GYRO_FIFO_DOWN_THREE          |   0x03
 *      BMI160_GYRO_FIFO_DOWN_FOUR           |   0x04
 *      BMI160_GYRO_FIFO_DOWN_FIVE           |   0x05
 *      BMI160_GYRO_FIFO_DOWN_SIX            |   0x06
 *      BMI160_GYRO_FIFO_DOWN_SEVEN          |   0x07
 *
 *  @note The user can enable filtered accel data by the following macro
 *      config                               |   Value
 *      -------------------------------------|---------------------------
 *      BMI160_ACCEL_FIFO_FILT_EN            |   0x80
 *
 *  @note The user can enable filtered gyro data by the following macro
 *      config                               |   Value
 *      -------------------------------------|---------------------------
 *      BMI160_GYRO_FIFO_FILT_EN             |   0x08
 *
 *  @note : By ORing the above mentioned macros, the user can select
 *  the required FIFO down config settings
 *
 *  @param[in] dev : Structure instance of bmi160_dev.
 *
 *  @return status of bus communication result
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 *
 */
int8_t bmi160_set_fifo_down(uint8_t fifo_down, const struct bmi160_dev *dev);

/*!
 *  @brief This API sets the FIFO watermark level in the sensor.
 *
 *  @note The FIFO watermark is issued when the FIFO fill level is
 *  equal or above the watermark level and units of watermark is 4 bytes.
 *
 *  @param[in]  fifo_wm        : Variable used to set the FIFO water mark level
 *  @param[in]  dev            : Structure instance of bmi160_dev
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 *
 */
int8_t bmi160_set_fifo_wm(uint8_t fifo_wm, const struct bmi160_dev *dev);

/*!
 *  @brief This API parses and extracts the accelerometer frames from
 *  FIFO data read by the "bmi160_get_fifo_data" API and stores it in
 *  the "accel_data" structure instance.
 *
 *  @note The bmi160_extract_accel API should be called only after
 *  reading the FIFO data by calling the bmi160_get_fifo_data() API.
 *
 *  @param[out] accel_data    : Structure instance of bmi160_sensor_data
 *                              where the accelerometer data in FIFO is stored.
 *  @param[in,out] accel_length  : Number of valid accelerometer frames
 *                              (x,y,z axes data) read out from fifo.
 *  @param[in] dev            : Structure instance of bmi160_dev.
 *
 *  @note accel_length is updated with the number of valid accelerometer
 *  frames extracted from fifo (1 accel frame   = 6 bytes) at the end of
 *  execution of this API.
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 *
 */
int8_t bmi160_extract_accel(struct bmi160_sensor_data *accel_data, uint8_t *accel_length, struct bmi160_dev const *dev);

/*!
 *  @brief This API parses and extracts the gyro frames from
 *  FIFO data read by the "bmi160_get_fifo_data" API and stores it in
 *  the "gyro_data" structure instance.
 *
 *  @note The bmi160_extract_accel API should be called only after
 *  reading the FIFO data by calling the bmi160_get_fifo_data() API.
 *
 *  @param[out] gyro_data    : Structure instance of bmi160_sensor_data
 *                             where the gyro data in FIFO is stored.
 *  @param[in,out] gyro_length  : Number of valid gyro frames
 *                             (x,y,z axes data) read out from fifo.
 *  @param[in] dev           : Structure instance of bmi160_dev.
 *
 *  @note gyro_length is updated with the number of valid gyro
 *  frames extracted from fifo (1 gyro frame   = 6 bytes) at the end of
 *  execution of this API.
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 *
 */
int8_t bmi160_extract_gyro(struct bmi160_sensor_data *gyro_data, uint8_t *gyro_length, struct bmi160_dev const *dev);

#ifdef __cplusplus
}
#endif

#endif /* BMI160_H_ */

/** @}*/
