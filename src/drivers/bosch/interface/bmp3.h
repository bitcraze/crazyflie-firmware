/**
 * Copyright (C) 2016 - 2017 Bosch Sensortec GmbH
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
 * @file	bmp3.h
 * @date	04 Dec 2017
 * @version	1.0.0
 * @brief
 *
 */

/*! @file bmp3.h
    @brief Sensor driver for BMP3 sensor */
/*!
 * @defgroup BMP3 SENSOR API
 * @{*/
#ifndef BMP3_H_
#define BMP3_H_

/* Header includes */
#include "bmp3_defs.h"

extern uint8_t bmp3_chip_id;

/*!
 *  @brief This API is the entry point.
 *  It performs the selection of I2C/SPI read mechanism according to the
 *  selected interface and reads the chip-id and calibration data of the sensor.
 *
 *  @param[in,out] dev : Structure instance of bmp3_dev
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bmp3_init(struct bmp3_dev *dev);

/*!
 * @brief This API performs the soft reset of the sensor.
 *
 * @param[in] dev : Structure instance of bmp3_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error.
 */
int8_t bmp3_soft_reset(const struct bmp3_dev *dev);

/*!
 * @brief This API sets the power control(pressure enable and
 * temperature enable), over sampling, odr and filter
 * settings in the sensor.
 *
 * @param[in] dev : Structure instance of bmp3_dev.
 * @param[in] desired_settings : Variable used to select the settings which
 * are to be set in the sensor.
 *
 * @note : Below are the macros to be used by the user for selecting the
 * desired settings. User can do OR operation of these macros for configuring
 * multiple settings.
 *
 * Macros		  |   Functionality
 * -----------------------|----------------------------------------------
 * BMP3_PRESS_EN_SEL    |   Enable/Disable pressure.
 * BMP3_TEMP_EN_SEL     |   Enable/Disable temperature.
 * BMP3_PRESS_OS_SEL    |   Set pressure oversampling.
 * BMP3_TEMP_OS_SEL     |   Set temperature oversampling.
 * BMP3_IIR_FILTER_SEL  |   Set IIR filter.
 * BMP3_ODR_SEL         |   Set ODR.
 * BMP3_OUTPUT_MODE_SEL |   Set either open drain or push pull
 * BMP3_LEVEL_SEL       |   Set interrupt pad to be active high or low
 * BMP3_LATCH_SEL       |   Set interrupt pad to be latched or nonlatched.
 * BMP3_DRDY_EN_SEL     |   Map/Unmap the drdy interrupt to interrupt pad.
 * BMP3_I2C_WDT_EN_SEL  |   Enable/Disable I2C internal watch dog.
 * BMP3_I2C_WDT_SEL_SEL |   Set I2C watch dog timeout delay.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error.
 */
int8_t bmp3_set_sensor_settings(uint32_t desired_settings, struct bmp3_dev *dev);

/*!
 * @brief This API gets the power control(power mode, pressure enable and
 * temperature enable), over sampling, odr, filter, interrupt control and
 * advance settings from the sensor.
 *
 * @param[in,out] dev : Structure instance of bmp3_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error.
 */
int8_t bmp3_get_sensor_settings(struct bmp3_dev *dev);

/*!
 * @brief This API sets the power mode of the sensor.
 *
 * @param[in] dev : Structure instance of bmp3_dev.
 *
 * dev->settings.op_mode |   Macros
 * ----------------------|-------------------
 *     0                 | BMP3_SLEEP_MODE
 *     1                 | BMP3_FORCED_MODE
 *     3                 | BMP3_NORMAL_MODE
 *
 *
 * @note : Before setting normal mode, valid odr and osr settings should be set
 * in the sensor by using 'bmp3_set_sensor_settings' function.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bmp3_set_op_mode(struct bmp3_dev *dev);

/*!
 * @brief This API gets the power mode of the sensor.
 *
 * @param[in] dev : Structure instance of bmp3_dev.
 * @param[out] op_mode : Pointer variable to store the op-mode.
 *
 *   op_mode             |   Macros
 * ----------------------|-------------------
 *     0                 | BMP3_SLEEP_MODE
 *     1                 | BMP3_FORCED_MODE
 *     3                 | BMP3_NORMAL_MODE
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bmp3_get_op_mode(uint8_t *op_mode, const struct bmp3_dev *dev);

/*!
 * @brief This API reads the pressure, temperature or both data from the
 * sensor, compensates the data and store it in the bmp3_data structure
 * instance passed by the user.
 *
 * @param[in] sensor_comp : Variable which selects which data to be read from
 * the sensor.
 *
 * sensor_comp |   Macros
 * ------------|-------------------
 *     1       | BMP3_PRESS
 *     2       | BMP3_TEMP
 *     3       | BMP3_ALL
 *
 * @param[out] data : Structure instance of bmp3_data.
 * @param[in] dev : Structure instance of bmp3_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bmp3_get_sensor_data(uint8_t sensor_comp, struct bmp3_data *data, struct bmp3_dev *dev);

/*!
 * @brief This API writes the given data to the register address
 * of the sensor.
 *
 * @param[in] reg_addr : Register address from where the data to be written.
 * @param[in] reg_data : Pointer to data buffer which is to be written
 * in the sensor.
 * @param[in] len : No of bytes of data to write..
 * @param[in] dev : Structure instance of bmp3_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bmp3_set_regs(uint8_t *reg_addr, const uint8_t *reg_data, uint8_t len, const struct bmp3_dev *dev);

/*!
 * @brief This API reads the data from the given register address of the sensor.
 *
 * @param[in] reg_addr : Register address from where the data to be read
 * @param[out] reg_data : Pointer to data buffer to store the read data.
 * @param[in] length : No of bytes of data to be read.
 * @param[in] dev : Structure instance of bmp3_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bmp3_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t length, const struct bmp3_dev *dev);

/*!
 * @brief This API sets the fifo_config_1(fifo_mode,
 * fifo_stop_on_full, fifo_time_en, fifo_press_en, fifo_temp_en),
 * fifo_config_2(fifo_subsampling, data_select) and int_ctrl(fwtm_en, ffull_en)
 * settings in the sensor.
 *
 * @param[in] dev : Structure instance of bmp3_dev.
 * @param[in] desired_settings : Variable used to select the FIFO settings which
 * are to be set in the sensor.
 *
 * @note : Below are the macros to be used by the user for selecting the
 * desired settings. User can do OR operation of these macros for configuring
 * multiple settings.
 *
 * Macros                          |  Functionality
 * --------------------------------|----------------------------
 * BMP3_FIFO_MODE_SEL            |  Enable/Disable FIFO
 * BMP3_FIFO_STOP_ON_FULL_EN_SEL |  Set FIFO stop on full interrupt
 * BMP3_FIFO_TIME_EN_SEL         |  Enable/Disable FIFO time
 * BMP3_FIFO_PRESS_EN_SEL        |  Enable/Disable pressure
 * BMP3_FIFO_TEMP_EN_SEL         |  Enable/Disable temperature
 * BMP3_FIFO_DOWN_SAMPLING_SEL   |  Set FIFO downsampling
 * BMP3_FIFO_FILTER_EN_SEL       |  Enable/Disable FIFO filter
 * BMP3_FIFO_FWTM_EN_SEL         |  Enable/Disable FIFO watermark interrupt
 * BMP3_FIFO_FFULL_EN_SEL        |  Enable/Disable FIFO full interrupt
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error.
 */
int8_t bmp3_set_fifo_settings(uint16_t desired_settings, const struct bmp3_dev *dev);

/*!
 * @brief This API gets the fifo_config_1(fifo_mode,
 * fifo_stop_on_full, fifo_time_en, fifo_press_en, fifo_temp_en),
 * fifo_config_2(fifo_subsampling, data_select) and int_ctrl(fwtm_en, ffull_en)
 * settings from the sensor.
 *
 * @param[in,out] dev : Structure instance of bmp3_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error.
 */
int8_t bmp3_get_fifo_settings(const struct bmp3_dev *dev);

/*!
 * @brief This API gets the fifo data from the sensor.
 *
 * @param[in,out] dev : Structure instance of bmp3 device, where the fifo
 * data will be stored in fifo buffer.
 *
 * @return Result of API execution status.
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error.
 */
int8_t bmp3_get_fifo_data(const struct bmp3_dev *dev);

/*!
 * @brief This API gets the fifo length from the sensor.
 *
 * @param[out] fifo_length : Variable used to store the fifo length.
 * @param[in] dev : Structure instance of bmp3_dev.
 *
 * @return Result of API execution status.
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error.
 */
int8_t bmp3_get_fifo_length(uint16_t *fifo_length, const struct bmp3_dev *dev);

/*!
 * @brief This API extracts the temperature and/or pressure data from the FIFO
 * data which is already read from the fifo.
 *
 * @param[out] data : Array of bmp3_data structures where the temperature
 * and pressure frames will be stored.
 * @param[in,out] dev : Structure instance of bmp3_dev which contains the
 * fifo buffer to parse the temperature and pressure frames.
 *
 * @return Result of API execution status.
 * @retval zero -> Success / -ve value -> Error.
 */
int8_t bmp3_extract_fifo_data(struct bmp3_data *data, struct bmp3_dev *dev);

/*!
 * @brief This API gets the command ready, data ready for pressure and
 * temperature and interrupt (fifo watermark, fifo full, data ready) and
 * error status from the sensor.
 *
 * @param[in,out] dev : Structure instance of bmp3_dev
 *
 * @return Result of API execution status.
 * @retval zero -> Success / -ve value -> Error.
 */
int8_t bmp3_get_status(struct bmp3_dev *dev);

/*!
 * @brief This API sets the fifo watermark length according to the frames count
 * set by the user in the device structure. Refer below for usage.
 *
 * @note: dev->fifo->data.req_frames = 50;
 *
 * @param[in] dev : Structure instance of bmp3_dev
 *
 * @return Result of API execution status.
 * @retval zero -> Success / -ve value -> Error.
 */
int8_t bmp3_set_fifo_watermark(const struct bmp3_dev *dev);

#endif /* BMP3_H_ */
/** @}*/
