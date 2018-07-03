/*
 *
 ****************************************************************************
 * Copyright (C) 2015 - 2016 Bosch Sensortec GmbH
 *
 * File : bmi088_fifo.h
 *
 * Date: 30 Oct 2017
 *
 * Revision:
 *
 * Usage: Sensor Driver for BMI088 family of sensors
 *
 ****************************************************************************
 *
 * Disclaimer
 *
 * Common:
 * Bosch Sensortec products are developed for the consumer goods industry.
 * They may only be used within the parameters of the respective valid
 * product data sheet.  Bosch Sensortec products are provided with the
 * express understanding that there is no warranty of fitness for a
 * particular purpose.They are not fit for use in life-sustaining,
 * safety or security sensitive systems or any system or device
 * that may lead to bodily harm or property damage if the system
 * or device malfunctions. In addition,Bosch Sensortec products are
 * not fit for use in products which interact with motor vehicle systems.
 * The resale and or use of products are at the purchasers own risk and
 * his own responsibility. The examination of fitness for the intended use
 * is the sole responsibility of the Purchaser.
 *
 * The purchaser shall indemnify Bosch Sensortec from all third party
 * claims, including any claims for incidental, or consequential damages,
 * arising from any product use not covered by the parameters of
 * the respective valid product data sheet or not approved by
 * Bosch Sensortec and reimburse Bosch Sensortec for all costs in
 * connection with such claims.
 *
 * The purchaser must monitor the market for the purchased products,
 * particularly with regard to product safety and inform Bosch Sensortec
 * without delay of all security relevant incidents.
 *
 * Engineering Samples are marked with an asterisk (*) or (e).
 * Samples may vary from the valid technical specifications of the product
 * series. They are therefore not intended or fit for resale to third
 * parties or for use in end products. Their sole purpose is internal
 * client testing. The testing of an engineering sample may in no way
 * replace the testing of a product series. Bosch Sensortec assumes
 * no liability for the use of engineering samples.
 * By accepting the engineering samples, the Purchaser agrees to indemnify
 * Bosch Sensortec from all claims arising from the use of engineering
 * samples.
 *
 * Special:
 * This software module (hereinafter called "Software") and any information
 * on application-sheets (hereinafter called "Information") is provided
 * free of charge for the sole purpose to support your application work.
 * The Software and Information is subject to the following
 * terms and conditions:
 *
 * The Software is specifically designed for the exclusive use for
 * Bosch Sensortec products by personnel who have special experience
 * and training. Do not use this Software if you do not have the
 * proper experience or training.
 *
 * This Software package is provided `` as is `` and without any expressed
 * or implied warranties,including without limitation, the implied warranties
 * of merchantability and fitness for a particular purpose.
 *
 * Bosch Sensortec and their representatives and agents deny any liability
 * for the functional impairment
 * of this Software in terms of fitness, performance and safety.
 * Bosch Sensortec and their representatives and agents shall not be liable
 * for any direct or indirect damages or injury, except as
 * otherwise stipulated in mandatory applicable law.
 *
 * The Information provided is believed to be accurate and reliable.
 * Bosch Sensortec assumes no responsibility for the consequences of use
 * of such Information nor for any infringement of patents or
 * other rights of third parties which may result from its use.
 * No license is granted by implication or otherwise under any patent or
 * patent rights of Bosch. Specifications mentioned in the Information are
 * subject to change without notice.
 **************************************************************************/
/*! \file bmi088_fifo.h
 \brief Sensor Driver for BMI088 family of sensors */
#ifndef BMI088_FIFO_H_
#define BMI088_FIFO_H_

#ifdef __cplusplus
extern "C"
{
#endif
#ifdef USE_FIFO
    /*********************************************************************/
    /* header files */
#include "bmi088.h"

    /*********************************************************************/
    /* (extern) variable declarations */

    /*********************************************************************/
    /* function prototype declarations */

    /*!
     *  @brief This API reads the FIFO data of Accel sensor.
     *
     *  @param[in,out] dev  : Structure instance of bmi088_dev
     *
     *  @return Result of API execution status
     *  @retval 0 -> Success
     *  @retval Any non zero value -> Fail
     *
     */
    uint16_t bmi088_get_accel_fifo_data(struct bmi088_dev *dev);

    /*!
     *  @brief This API parses and extracts the accelerometer frames from
     *  FIFO data read by the "bmi088_get_accel_fifo_data" API and stores
     *  it in the "accel_data" structure instance.
     *
     *  @note The bmi088_extract_accel API should be called only after reading
     *  the FIFO data by calling the bmi088_get_accel_fifo_data() API
     *
     *  @param[in,out] accel_data   : Structure instance of bmi088_sensor_data
     *   where the accelerometer data in FIFO is stored.
     *  @param[in,out] accel_length : Number of accelerometer frames
     *   (x,y,z axes data)
     *  @param[in,out] dev          : Structure instance of bmi088_dev.
     *
     *  @note accel_length has the number of accelerometer frames
     *  (1 accel frame   = 6 bytes) which the user needs to extract and store, is
     *  provided as input parameter by the user and the Number of valid
     *  accelerometer frames extracted and stored is updated in
     *  "accel_length" at the end of execution of this API.
     *
     *  @return Result of API execution status
     *  @retval 0 -> Success
     *  @retval Any non zero value -> Fail
     *
     */
    uint16_t bmi088_extract_accel(struct bmi088_sensor_data *accel_data, uint16_t *accel_length, const struct bmi088_dev *dev);

    /*!
     *  @brief This API reads the FIFO water mark level which is set
     *  in the accel sensor.
     *
     *  @note The FIFO watermark is issued when the accel FIFO fill level
     *  is equal or above the watermark level.
     *
     *  @param[out] fifo_wm : Pointer variable to store FIFO water mark level
     *  @param[in] dev      : Structure instance of bmi088_dev
     *
     *  @return Result of API execution status
     *  @retval 0 -> Success
     *  @retval Any non zero value -> Fail
     *
     */
    uint16_t bmi088_get_accel_fifo_wm(uint16_t *fifo_wm, struct bmi088_dev *dev);

    /*!
     *  @brief This API sets the FIFO watermark level in the accel sensor.
     *
     *  @note The FIFO watermark is issued when the accel FIFO fill level is
     *  equal or above the watermark level.
     *
     *  @param[in] fifo_wm  : Variable used to set the FIFO water mark level
     *  @param[out] dev     : Structure instance of bmi088_dev
     *
     *  @return Result of API execution status
     *  @retval 0 -> Success
     *  @retval Any non zero value -> Fail
     *
     */
    uint16_t bmi088_set_accel_fifo_wm(uint16_t fifo_wm, struct bmi088_dev *dev);

    /*!
     *  @brief This API checks whether the Accel FIFO data is set for filtered
     *  or unfiltered mode.
     *
     *  @param[out] accel_fifo_filter : Variable used to check whether the Accel
     *  data is filtered or unfiltered.
     *  Value    |  accel_fifo_filter
     *  ---------|-------------------------
     *  0x00     |  Unfiltered data
     *  0x01     |  Filtered data
     *
     *  @param[in] dev                : structure instance of bmi088_dev
     *
     *  @return Result of API execution status
     *  @retval 0 -> Success
     *  @retval Any non zero value -> Fail
     *
     */
    uint16_t bmi088_get_accel_fifo_filt_data(uint8_t *accel_fifo_filter, struct bmi088_dev *dev);

    /*!
     *  @brief This API sets the condition of Accel FIFO data either to
     *  filtered or unfiltered mode.
     *
     *  @param[in] accel_fifo_filter : Variable used to set the filtered or
     *  unfiltered condition of Accel FIFO data.
     *  value      |  accel_fifo_filter_data
     *  -----------|-------------------------
     *    0x00     |  Unfiltered data
     *    0x01     |  Filtered data
     *
     *  @param[out] dev              : Structure instance of bmi088_dev
     *
     *  @return Result of API execution status
     *  @retval 0 -> Success
     *  @retval Any non zero value -> Fail
     *
     */
    uint16_t bmi088_set_accel_fifo_filt_data(uint8_t accel_fifo_filter, struct bmi088_dev *dev);

    /*!
     *  @brief This API reads the down sampling rates which is configured
     *  for Accel FIFO data.
     *
     *  @param[out] fifo_down : Variable used to specify the Accel FIFO
     *  down-sampling rates
     *  @param[in] dev        : Structure instance of bmi088_dev
     *
     *  @return Result of API execution status
     *  @retval 0 -> Success
     *  @retval Any non zero value -> Fail
     *
     */
    uint16_t bmi088_get_fifo_down_accel(uint8_t *fifo_down, struct bmi088_dev *dev);

    /*!
     *  @brief This API sets the down-sampling rates for Accel FIFO.
     *
     *  @param[in] fifo_down : Variable used to specify the Accel FIFO
     *  down-sampling rates.
     *  @param[in] dev       : Structure instance of bmi088_dev
     *
     *  @return Result of API execution status
     *  @retval 0 -> Success
     *  @retval Any non zero value -> Fail
     *
     */
    uint16_t bmi088_set_fifo_down_accel(uint8_t fifo_down, struct bmi088_dev *dev);

    /*!
     *  @brief This API reads the length of Accel FIFO data in the units of bytes.
     *
     *  @note This byte counter is updated each time a complete frame is read
     *  or written
     *
     *  @param[in] fifo_length : Pointer variable used to store the value of
     *  fifo byte counter
     *  @param[in] dev         : Structure instance of bmi088_dev
     *
     *  @return Result of API execution status
     *  @retval 0 -> Success
     *  @retval Any non zero value -> Fail
     *
     */
    uint16_t bmi088_get_accel_fifo_length(uint16_t *fifo_length, struct bmi088_dev *dev);

    /*!
     *  @brief This API sets the FIFO full interrupt of the accel sensor.
     *
     *  @param[in] int_config        : Structure instance of bmi088_int_cfg.
     *  @param[in] dev               : Structure instance of bmi088_dev.
     *
     *  @return Result of API execution status
     *  @retval zero -> Success
     *  @retval Any non zero value -> Fail
     *
     */
    uint16_t bmi088_set_accel_fifo_full_int(struct bmi088_int_cfg *int_config, struct bmi088_dev *dev);

    /*!
     *  @brief This API sets the FIFO watermark interrupt of the accel sensor.
     *
     *  @param[in] int_config      : Structure instance of bmi088_int_cfg.
     *  @param[in] dev             : Structure instance of bmi088_dev.
     *
     *  @return Result of API execution status
     *  @retval zero -> Success
     *  @retval Any non zero value -> Fail
     *
     */
    uint16_t bmi088_set_accel_fifo_wm_int(struct bmi088_int_cfg *int_config, struct bmi088_dev *dev);

    /*!
     *  @brief This API reads the FIFO data of Gyro sensor
     *
     *  @param[in,out] dev  : Structure instance of bmi088_dev
     *
     *  @return Result of API execution status
     *  @retval 0 -> Success
     *  @retval Any non zero value -> Fail
     *
     */
    uint16_t bmi088_get_gyro_fifo_data(struct bmi088_dev *dev);

    /*!
     *  @brief This API parses and extracts the gyroscope frames from
     *  FIFO data read by the "bmi088_get_gyro_fifo_data" API and stores
     *  it in the "gyro_data" structure instance parameter of this API
     *
     *  @note The bmi088_extract_gyro API should be called only after reading
     *  the FIFO data by calling the bmi088_get_gyro_fifo_data() API
     *
     *  @param[in,out] gyro_data   : Structure instance of bmi088_sensor_data
     *  where the gyroscope data in FIFO is stored.
     *  @param[in,out] gyro_length : Number of gyroscope frames (x,y,z,r data)
     *  @param[in,out] dev         : Structure instance of bmi088_dev.
     *
     *  @note gyro_length has the number of gyroscope frames(x,y,z,r data)
     *  which the user needs to extract and store. It is provided as input
     *  parameter by the user and the number of valid gyroscope frames
     *  extracted and stored is updated in "gyro_length" at the end of
     *  execution of this API.
     *
     *  @return Result of API execution status
     *  @retval 0 -> Success
     *  @retval Any non zero value -> Fail
     *
     */
    uint16_t bmi088_extract_gyro(struct bmi088_sensor_data *gyro_data, uint16_t *gyro_length, const struct bmi088_dev *dev);

    /*!
     *  @brief This API reads the FIFO water mark level which is set
     *  in the gyro sensor.
     *
     *  @note The FIFO watermark is issued when the FIFO fill level is
     *  equal or above the watermark level.
     *
     *  @param[out]  fifo_wm : Pointer variable to store FIFO water mark level
     *  @param[in]   dev     : Structure instance of bmi088_dev
     *
     *  @return Result of API execution status
     *  @retval 0 -> Success
     *  @retval Any non zero value -> Fail
     *
     */
    uint16_t bmi088_get_gyro_fifo_wm(uint8_t *fifo_wm, struct bmi088_dev *dev);

    /*!
     *  @brief This API sets the FIFO watermark level in the gyro sensor.
     *
     *  @note The FIFO watermark is issued when the FIFO fill level is
     *  equal or above the watermark level.
     *
     *  @param[in]  fifo_wm : Variable used to set the FIFO water mark level
     *  @param[in]  dev     : Structure instance of bmi088_dev
     *
     *  @return Result of API execution status
     *  @retval 0 -> Success
     *  @retval Any non zero value -> Fail
     *
     */
    uint16_t bmi088_set_gyro_fifo_wm(uint8_t fifo_wm, struct bmi088_dev *dev);

    /*!
     *  @brief This API reads the FIFO operating mode which is set
     *  in the gyro sensor.
     *
     *  @param[out] fifo_mode  : Pointer variable used to get the FIFO
     *  operating mode
     *            value                 |  fifo_mode
     *  --------------------------------|-------------------------
     *   BMI088_GYRO_BYPASS_OP_MODE     |  Bypass Mode
     *   BMI088_GYRO_FIFO_OP_MODE       |  Fifo Mode
     *   BMI088_GYRO_STREAM_OP_MODE     |  Stream Mode
     *
     *  @param[in] dev         : Structure instance of bmi088_dev
     *
     *  @return Result of API execution status
     *  @retval 0 -> Success
     *  @retval Any non zero value -> Fail
     *
     */
    uint16_t bmi088_get_gyro_fifo_mode(uint8_t *fifo_mode, struct bmi088_dev *dev);

    /*!
     *  @brief This API sets the FIFO operating mode in the gyro sensor.
     *
     *  @param[in]  fifo_mode : Variable used to set the FIFO operating mode
     *            value                 |  Fifo Mode
     *  --------------------------------|-------------------------
     *   BMI088_GYRO_BYPASS_OP_MODE     |  Bypass Mode
     *   BMI088_GYRO_FIFO_OP_MODE       |  Fifo Mode
     *   BMI088_GYRO_STREAM_OP_MODE     |  Stream Mode
     *
     *  @param[in] dev        : Structure instance of bmi088_dev
     *
     *  @return Result of API execution status
     *  @retval 0 -> Success
     *  @retval Any non zero value -> Fail
     *
     */
    uint16_t bmi088_set_gyro_fifo_mode(uint8_t fifo_mode, struct bmi088_dev *dev);

    /*!
     *  @brief This API reads the data of axes to be stored in the FIFO in the gyro sensor.
     *
     *  @param[out]  fifo_data_select : Pointer variable used to get the selected data
     *  axes for FIFO data storage.
     *            value                 |  fifo_data_select
     *  --------------------------------|-------------------------
     *   BMI088_GYRO_ALL_INT_DATA       |  X,Y,Z plus INT_status 0 and 1
     *   BMI088_GYRO_X_DATA             |  X only
     *   BMI088_GYRO_Y_DATA             |  Y only
     *   BMI088_GYRO_Z_DATA             |  Z only
     *
     *  @param[in] dev                : Structure instance of bmi088_dev
     *
     *  @return Result of API execution status
     *  @retval 0 -> Success
     *  @retval Any non zero value -> Fail
     *
     */
    uint16_t bmi088_get_gyro_fifo_data_sel(uint8_t *fifo_data_select, struct bmi088_dev *dev);

    /*!
     *  @brief This API sets the data of axes to be stored in the FIFO in the gyro sensor.
     *
     *  @param[in]  fifo_data_select : Variable used to select data of axis to be stored
     *  in fifo
     *            value                 |  Data of axis stored in FIFO
     *  --------------------------------|-------------------------
     *   BMI088_GYRO_ALL_INT_DATA       |  X,Y,Z plus INT_status 0 and 1
     *   BMI088_GYRO_X_DATA             |  X only
     *   BMI088_GYRO_Y_DATA             |  Y only
     *   BMI088_GYRO_Z_DATA             |  Z only
     *
     *  @param[in] dev               : Structure instance of bmi088_dev
     *
     *  @return Result of API execution status
     *  @retval 0 -> Success
     *  @retval Any non zero value -> Fail
     *
     */
    uint16_t bmi088_set_gyro_fifo_data_sel(uint8_t fifo_data_select, struct bmi088_dev *dev);

    /*!
     *  @brief This API gets the length of Gyro FIFO data in the units of bytes.
     *
     *  @note This byte counter is updated each time a complete frame is read
     *  or written
     *
     *  @param[out] fifo_length : Pointer variable used to store the value of
     *  fifo byte counter
     *  @param[in] dev          : Structure instance of bmi088_dev
     *
     *  @return Result of API execution status
     *  @retval 0 -> Success
     *  @retval Any non zero value -> Fail
     *
     */
    uint16_t bmi088_get_gyro_fifo_length(uint8_t *fifo_length, struct bmi088_dev *dev);

    /*!
     *  @brief This API gets the gyro FIFO overrun status.
     *
     *  @param[out] fifo_overrun_status : Pointer variable used to store the status
     *  of FIFO overrun.
     *  @param[in] dev                  : Structure instance of bmi088_dev
     *
     *  @return Result of API execution status
     *  @retval 0 -> Success
     *  @retval Any non zero value -> Fail
     *
     */
    uint16_t bmi088_get_gyro_fifo_overrun(uint8_t *fifo_overrun_status, struct bmi088_dev *dev);

    /*!
     *  @brief This API gets the fifo tag status which is set in
     *  gyro sensor.
     *
     *  @param[out] fifo_tag_status :  Pointer variable used to store the value of
     *  fifo tag status.
     *  @param[in] dev              : Structure instance of bmi088_dev
     *
     *  @return Result of API execution status
     *  @retval 0 -> Success
     *  @retval Any non zero value -> Fail
     *
     */
    uint16_t bmi088_get_gyro_fifo_tag(uint8_t *fifo_tag_status, struct bmi088_dev *dev);

    /*!
     *  @brief This API enables or disables fifo tag in gyro sensor.
     *
     *  @param[in] fifo_tag :  Variable used to enable or disable fifo tag
     *            value           |  fifo_tag
     *  --------------------------|-----------------------------------
     *   BMI088_DISABLE           | Do not collect Interrupts in FIFO
     *   BMI088_ENABLE            | Collects Interrupts in FIFO
     *
     *  @param[in] dev      : Structure instance of bmi088_dev
     *
     *  @return Result of API execution status
     *  @retval 0 -> Success
     *  @retval Any non zero value -> Fail
     *
     */
    uint16_t bmi088_set_gyro_fifo_tag(uint8_t fifo_tag, struct bmi088_dev *dev);

    /*!
     *  @brief This API enables or disables the FIFO full interrupt of the gyro sensor.
     *
     *  @param[in] int_config        : Structure instance of bmi088_int_cfg.
     *  @param[in] dev               : Structure instance of bmi088_dev.
     *  @param[in] fifo_full_enable  : Variable used to enable or disable fifo
     *  full interrupt
     *         value              |  fifo_full_enable
     *  --------------------------|-----------------------------------
     *   BMI088_DISABLE           | Disables fifo full interrupt
     *   BMI088_ENABLE            | Enables fifo full interrupt
     *
     *  @return Result of API execution status
     *  @retval zero -> Success
     *  @retval Any non zero value -> Fail
     *
     */
    uint16_t bmi088_set_gyro_fifo_full_int(struct bmi088_int_cfg *int_config, struct bmi088_dev *dev, uint8_t fifo_full_enable);

    /*!
     *  @brief This API enables or disables the FIFO watermark interrupt of the gyro sensor.
     *
     *  @param[in] int_config      : Structure instance of bmi088_int_cfg.
     *  @param[in] dev             : Structure instance of bmi088_dev.
     *  @param[in] fifo_wm_enable  : Variable used to enable or disable fifo
     *  watermark interrupt
     *         value              |  fifo_full_enable
     *  --------------------------|-----------------------------------
     *   BMI088_DISABLE           | Disables fifo watermark interrupt
     *   BMI088_ENABLE            | Enables fifo watermark interrupt
     *
     *  @return Result of API execution status
     *  @retval zero -> Success
     *  @retval Any non zero value -> Fail
     *
     */
    uint16_t bmi088_set_gyro_fifo_wm_int(struct bmi088_int_cfg *int_config, struct bmi088_dev *dev, uint8_t fifo_wm_enable);

#ifdef __cplusplus
}
#endif
#endif
#endif
/* End of BMI088_FIFO_H_ */
