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
 * @date	Dec 1, 2016
 * @version 3.0.0
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
int8_t bmi160_get_regs(uint8_t reg_addr, uint8_t *data, uint16_t len, struct bmi160_dev *dev);

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
int8_t bmi160_set_regs(uint8_t reg_addr, uint8_t *data, uint16_t len, struct bmi160_dev *dev);

/*!
 * @brief This API resets and restarts the device.All register
 * values are overwritten with default parameters.
 *
 * @param[in] dev  : Structure instance of bmi160_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
int8_t bmi160_soft_reset(struct bmi160_dev *dev);

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
 * @param[in] sensor    : enum to choose accel,gyro or both sensor data
 * @param[out] accel    : Structure pointer to store accel data
 * @param[out] gyro     : Structure pointer to store gyro data
 * @param[in] dev       : Structure instance of bmi160_dev.
 * @note : Refer user guide for detailed info.
 *
 * @return Result of API execution status
 * @retval zero -> Success  / -ve value -> Error
 */
int8_t bmi160_get_sensor_data(enum bmi160_select_sensor sensor, struct bmi160_sensor_data *accel, struct bmi160_sensor_data *gyro, struct bmi160_dev *dev);

/*!
 * @brief This API configures the necessary interrupt based on
 *  the user settings in the bmi160_intr_sett structure instance.
 *
 * @param[in] int_config  : Structure instance of bmi160_intr_sett.
 * @param[in] dev         : Structure instance of bmi160_dev.
 * @note : Refer user guide for detailed info.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
int8_t bmi160_set_intr_config(struct bmi160_intr_sett *int_config, struct bmi160_dev *dev);

#ifdef __cplusplus
}
#endif

#endif /* BMI160_H_ */

/** @}*/
