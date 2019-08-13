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
 * @file	bmi055.h
 * @date	19 Apr, 2017
 * @version	1.1.0
 * @brief	Sensor Driver for BMI055 sensor
 *
 */

#ifndef BMI055_H_
#define BMI055_H_

#ifdef __cplusplus
extern "C"
{
#endif

/********************************************************/
/* header includes */
#include "../interface/bmi055_defs.h"

/***************************************************************************/
/**\name        BMI055 User Accelerometer function prototypes
****************************************************************************/
/*!
 *  @brief This API is the entry point for accelerometer sensor.
 *  It performs the selection of I2C/SPI read mechanism according to the
 *  selected interface and reads the chip-id of accelerometer sensor.
 *
 *  @param[in,out] dev  : Structure instance of bmi055_dev.
 *  @note : Refer user guide for detailed info.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
int8_t bmi055_accel_init(struct bmi055_dev *dev);

/*!
 * @brief This API reads the data from the given register address of
 * accelerometer sensor.
 *
 * @param[in] reg_addr  : Register address from where the data to be read
 * @param[out] data     : Pointer to data buffer to store the read data.
 * @param[in] len       : No. of bytes of data to be read.
 * @param[in] dev       : Structure instance of bmi055_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
int8_t bmi055_get_accel_regs(uint8_t reg_addr, uint8_t *data, uint16_t len, const struct bmi055_dev *dev);

/*!
 * @brief This API writes the given data to the register address
 * of accel sensor.
 *
 * @param[in] reg_addr  : Register address to where the data to be written.
 * @param[in] data      : Pointer to data buffer which is to be written
 * in the sensor.
 * @param[in] len       : No. of bytes of data to write.
 * @param[in] dev       : Structure instance of bmi055_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
int8_t bmi055_set_accel_regs(uint8_t reg_addr, uint8_t *data, uint16_t len, const struct bmi055_dev *dev);

/*!
 * @brief This API resets and restarts the accelerometer sensor.All register
 * values are overwritten with default parameters.
 *
 * @param[in] dev  : Structure instance of bmi055_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
int8_t bmi055_accel_soft_reset(const struct bmi055_dev *dev);

/*!
 * @brief This API sets the power mode of the accelerometer sensor.
 *
 * @param[in] dev  : Structure instance of bmi055_dev.
 *       [in]  power : power to be set
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
int8_t bmi055_set_accel_power_mode(const struct bmi055_dev *dev);

/*!
 * @brief This API sets the range and bandwidth
 * of accel sensor.
 *
 * @param[in] dev  : Structure instance of bmi055_dev.
 * @param[in] bmi055_config  : Decides the parameter to be tested
 *
 * @note : Refer user guide for detailed info.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
int8_t bmi055_set_accel_sensor_config(uint8_t bmi055_config, const struct bmi055_dev *dev);

/*!
 * @brief This API reads the accelerometer data from the sensor,
 * store it in the bmi055_sensor_data structure instance
 * passed by the user.
 *
 * @param[out] accel  : Structure pointer to store accelerometer data
 * @param[in]  dev    : Structure instance of bmi055_dev.
 *
 *
 * @return Result of API execution status
 * @retval zero -> Success  / -ve value -> Error
 */
int8_t bmi055_get_accel_data(struct bmi055_sensor_data *accel, const struct bmi055_dev *dev);

/*!
 * @brief This API configures the necessary accelerometer interrupt
 * based on the user settings in the bmi055_accel_int_sett
 * structure instance.
 *
 * @param[in] acc_int_config  : Structure instance of bmi055_accel_int_sett.
 * @param[in] dev         : Structure instance of bmi055_dev.
 *
 * @note : Refer user guide for detailed info.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
int8_t bmi055_set_accel_int_config(struct bmi055_accel_int_sett *acc_int_config, const struct bmi055_dev *dev);

/*!
 * @brief This API retrieves the status of accelerometer interrupts
 * when set
 *
 * @param[in, out] intr_status : Pointer to get interrupt status.
 * @param[in]      dev         : Structure instance of bmi055_dev.
 *
 * @note : Refer bmi055_defs.h for status masks :
 *
 * BMI055_ACC_LOW_G_INT_STATUS_MASK       - For low-g status
 * BMI055_ACC_HIGH_G_INT_STATUS_MASK      - For high-g status
 * BMI055_ACC_SLOPE_INT_STATUS_MASK       - For slope status
 * BMI055_ACC_SLOW_NO_MOT_INT_STATUS_MASK - For slow-motion interrupt
 * BMI055_ACC_DOUBLE_TAP_INT_STATUS_MASK  - For double tap
 * BMI055_ACC_SINGLE_TAP_INT_STATUS_MASK  - For single tap
 * BMI055_ACC_ORIENT_INT_STATUS_MASK      - For orient interrupt
 * BMI055_ACC_TAP_INT_STATUS_MASK         - For tap interrupt
 * BMI055_ACC_FIFO_FULL_INT_STATUS_MASK   - For fifo full interrupt
 * BMI055_ACC_FIFO_WM_INT_STATUS_MASK     - For fifo watermark interrupt
 * BMI055_ACC_NEW_DATA_INT_STATUS_MASK    - For new data interrupt
 * BMI055_ACC_SLOPE_X_INT_STATUS_MASK     - For slope interrupt in x-axis
 * BMI055_ACC_SLOPE_Y_INT_STATUS_MASK     - For slope interrupt in y-axis
 * BMI055_ACC_SLOPE_Z_INT_STATUS_MASK     - For slope interrupt in z-axis
 * BMI055_ACC_SLOPE_SIGN_INT_STATUS_MASK  - direction of slope interrupt
 * BMI055_ACC_TAP_X_INT_STATUS_MASK       - For tap interrupt in x-axis
 * BMI055_ACC_TAP_Y_INT_STATUS_MASK       - For tap interrupt in y-axis
 * BMI055_ACC_TAP_Z_INT_STATUS_MASK       - For tap interrupt in z-axis
 * BMI055_ACC_TAP_SIGN_INT_STATUS_MASK    - direction of slope interrupt
 * BMI055_ACC_HIGH_G_X_INT_STATUS_MASK    - For high-g interrupt in x-axis
 * BMI055_ACC_HIGH_G_Y_INT_STATUS_MASK    - For high-g interrupt in y-axis
 * BMI055_ACC_HIGH_G_Z_INT_STATUS_MASK    - For high-g interrupt in z-axis
 * BMI055_ACC_HIGH_G_SIGN_INT_STATUS_MASK - direction of high-g interrupt
 * BMI055_ACC_ORIENT_BIT_MASK             - interrupt for orient modes
 * BMI055_ACC_ORIENT_PORTRAIT_UPRIGHT_MASK- orientation is portrait upright
 * BMI055_ACC_ORIENT_PORTRAIT_DOWN_MASK   - orientation is portrait down
 * BMI055_ACC_ORIENT_LANDSACPE_LEFT_MASK  - orientation is landscape left
 * BMI055_ACC_ORIENT_LANDSACPE_RIGHT_MASK - orientation is landscape right
 * BMI055_ACC_ORIENT_Z_AXIS_DIR_MASK      - direction of z axis
 * BMI055_ACC_FLAT_INT_STATUS_MASK        - For flat interrupt
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
int8_t bmi055_get_accel_int_status(uint32_t *intr_status, const struct bmi055_dev *dev);

/*!
 * @brief This  API clears all the latched interrupts of the accelerometer.
 *
 * @param[in] acc_int_config  : Structure instance of bmi055_accel_int_sett.
 * @param[in] dev             : Structure instance of bmi055_dev.
 *
 * @note : Refer user guide for detailed info.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
int8_t bmi055_reset_accel_latch_int(struct bmi055_accel_int_sett *acc_int_config, const struct bmi055_dev *dev);

/*!
 * @brief This API enables/disables individual or all accelerometer interrupts.
 *
 * @param[in] acc_int_config  : Structure instance of bmi055_accel_int_sett.
 * @param[in] dev             : Structure instance of bmi055_dev.
 * @param[in] set_en_bit      : Enable or disable bit.
 *
 * @note : Refer user guide for detailed info.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
int8_t bmi055_set_reset_accel_int(struct bmi055_accel_int_sett *acc_int_config, const struct bmi055_dev *dev,
						uint8_t set_en_bit);


/***************************************************************************/
/**\name        BMI055 Gyroscope User function prototypes
****************************************************************************/
/*!
 *  @brief This API is the entry point for gyroscope sensor.
 *  It performs the selection of I2C/SPI read mechanism according to the
 *  selected interface and reads the chip-id of gyroscope sensor.
 *
 *  @param[in,out] dev  : Structure instance of bmi055_dev.
 *  @note : Refer user guide for detailed info.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
int8_t bmi055_gyro_init(struct bmi055_dev *dev);

/*!
 * @brief This API reads the data from the given register address of \
 * gyroscope sensor.
 *
 * @param[in] reg_addr  : Register address from where the data to be read
 * @param[out] data     : Pointer to data buffer to store the read data.
 * @param[in] len       : No. of bytes of data to be read.
 * @param[in] dev       : Structure instance of bmi055_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
int8_t bmi055_get_gyro_regs(uint8_t reg_addr, uint8_t *data, uint16_t len, const struct bmi055_dev *dev);

/*!
 * @brief This API writes the given data to the register address
 * of gyroscope sensor.
 *
 * @param[in] reg_addr  : Register address to where the data to be written.
 * @param[in] data      : Pointer to data buffer which is to be written
 * in the sensor.
 * @param[in] len       : No. of bytes of data to write.
 * @param[in] dev       : Structure instance of bmi055_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
int8_t bmi055_set_gyro_regs(uint8_t reg_addr, uint8_t *data, uint16_t len, const struct bmi055_dev *dev);

/*!
 * @brief This API resets and restarts the gyroscope sensor.All
 * register values are overwritten with default values.
 *
 * @param[in] dev : Structure instance of bmi055_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
int8_t bmi055_gyro_soft_reset(const struct bmi055_dev *dev);

/*!
 * @brief This API sets the power mode of the gyroscope sensor.
 *
 * @param[in] dev : Structure instance of bmi055_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
int8_t bmi055_set_gyro_power_mode(const struct bmi055_dev *dev);

/*!
 * @brief This API sets the range and bandwidth
 * of gyroscope sensor.
 *
 * @param[in] dev  : Structure instance of bmi055_dev.
 * @param[in] bmi055_config  : Decides the parameter to be tested
 *
 * @note : Refer user guide for detailed info.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
int8_t bmi055_set_gyro_sensor_config(uint8_t bmi055_config, const struct bmi055_dev *dev);

/*!
 * @brief This API reads the gyroscope data from the sensor,
 * store it in the bmi055_sensor_data structure instance
 * passed by the user.
 *
 * @param[out] gyro   : Structure pointer to store gyroscope data
 * @param[in] dev     : Structure instance of bmi055_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success  / -ve value -> Error
 */
int8_t bmi055_get_gyro_data(struct bmi055_sensor_data *gyro, const struct bmi055_dev *dev);

/*!
 * @brief This API configures the necessary gyroscope interrupt
 * based on the user settings in the bmi055_accel_int_sett
 * structure instance.
 *
 * @param[in, out] acc_int_config : Structure instance of bmi055_accel_int_sett.
 * @param[in] dev                 : Structure instance of bmi055_dev.
 * @note : Refer user guide for detailed info.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
int8_t bmi055_set_gyro_int_config(struct bmi055_gyro_int_sett *gyr_int_config, struct bmi055_dev *dev);

#ifdef __cplusplus
}
#endif

#endif /* BMI055_H_ */

