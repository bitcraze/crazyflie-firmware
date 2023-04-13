/*
 *
 ****************************************************************************
 * Copyright (C) 2015 - 2016 Bosch Sensortec GmbH
 *
 * File : bmi088.h
 *
 * Date: 30 Oct 2017
 *
 * Revision:  $
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
/*! \file bmi088.h
 \brief Sensor Driver for BMI088 family of sensors */
#ifndef BMI088_H_
#define BMI088_H_

#ifdef __cplusplus
extern "C"
{
#endif

    /*********************************************************************/
    /* header files */
#include "bmi088_defs.h"
    /*********************************************************************/
    /* (extern) variable declarations */

    /*********************************************************************/
    /* function prototype declarations */

    /*********************** BMI088 Accelerometer function prototypes ************************/

    /*!
     *  @brief This API is the entry point for accel sensor.
     *  It performs the selection of I2C/SPI read mechanism according to the
     *  selected interface and reads the chip-id of accel sensor.
     *
     *  @param[in,out] dev  : Structure instance of bmi088_dev.
     *  @note : Refer user guide for detailed info.
     *
     *  @return Result of API execution status
     *  @retval zero -> Success
     *  @retval Any non zero value -> Fail
     */
    uint16_t bmi088_accel_init(struct bmi088_dev *dev);

    /*!
     *  @brief This API is used to write the binary configuration in the sensor
     *
     *  @param[in] dev : Structure instance of bmi088_dev.
     *
     *  @return Result of API execution status
     *  @retval 0 -> Success
     *  @retval Any non zero value -> Fail
     *
     */
    uint16_t bmi088_write_config_file(struct bmi088_dev *dev);

    /*!
     *  @brief This API reads the data from the given register address of accel sensor.
     *
     *  @param[in] reg_addr  : Register address from where the data to be read
     *  @param[out] data     : Pointer to data buffer to store the read data.
     *  @param[in] len       : No. of bytes of data to be read.
     *  @param[in] dev       : Structure instance of bmi088_dev.
     *
     *  @return Result of API execution status
     *  @retval zero -> Success
     *  @retval Any non zero value -> Fail
     */
    uint16_t bmi088_get_accel_regs(uint8_t reg_addr, uint8_t *data, uint16_t len, struct bmi088_dev *dev);

    /*!
     *  @brief This API writes the given data to the register address
     *  of accel sensor.
     *
     *  @param[in] reg_addr  : Register address to where the data to be written.
     *  @param[in] data      : Pointer to data buffer which is to be written
     *  in the sensor.
     *  @param[in] len       : No. of bytes of data to write.
     *  @param[in] dev       : Structure instance of bmi088_dev.
     *
     *  @return Result of API execution status
     *  @retval zero -> Success
     *  @retval Any non zero value -> Fail
     */
    uint16_t bmi088_set_accel_regs(uint8_t reg_addr, uint8_t *data, uint16_t len, struct bmi088_dev *dev);

    /*!
     *  @brief This API reads the error status from the accel sensor.
     *
     *  Below table mention the types of error which can occur in the sensor
     *@verbatim
     *************************************************************************
     *        Error           |       Description
     *************************|***********************************************
     *                        |       Fatal Error, chip is not in operational
     *        fatal           |       state (Boot-, power-system).
     *                        |       This flag will be reset only by
     *                        |       power-on-reset or soft reset.
     *************************|***********************************************
     *        cmd             |       Command execution failed.
     *************************|***********************************************
     *                        |       Value        Name       Description
     *        error_code      |       000        no_error     no error
     *                        |       001        accel_err      error in
     *                        |                               ACCEL_CONF
     *************************************************************************
     *@endverbatim
     *
     *  @param[in,out] err_reg : Pointer to structure variable which stores the
     *  error status read from the sensor.
     *  @param[in] dev : Structure instance of bmi088_dev.
     *
     *  @return Result of API execution status
     *  @retval 0 -> Success
     *  @retval Any non zero value -> Fail
     */
    uint16_t bmi088_get_accel_error_status(struct bmi088_err_reg *err_reg, struct bmi088_dev *dev);

    /*!
     *  @brief This API reads the status of the accel sensor.
     *
     *  Below table lists the sensor status flags
     *@verbatim
     *************************************************************************
     *        Status                    |       Description
     ***********************************|*************************************
     *        drdy_accel                | Data ready for Accel.
     *************************************************************************
     *@endverbatim
     *
     *  @param[in] status : Variable used to store the sensor status flags
     *  which is read from the sensor.
     *  @param[in] dev : Structure instance of bmi088_dev.
     *
     *  @return Result of API execution status
     *  @retval 0 -> Success
     *  @retval Any non zero value -> Fail
     *
     */
    uint16_t bmi088_get_accel_status(uint8_t *status, struct bmi088_dev *dev);

    /*!
     *  @brief This API resets the accel sensor.
     *
     *  @param[in] dev  : Structure instance of bmi088_dev.
     *
     *  @return Result of API execution status
     *  @retval zero -> Success
     *  @retval Any non zero value -> Fail
     */
    uint16_t bmi088_accel_soft_reset(struct bmi088_dev *dev);

    /*!
     *  @brief This API sets the Output data rate, range and bandwidth
     *  of accel sensor.
     *
     *  @param[in] dev  : Structure instance of bmi088_dev.
     *  @note : Refer user guide for detailed info.
     *
     *  @return Result of API execution status
     *  @retval zero -> Success
     *  @retval Any non zero value -> Fail
     */
    uint16_t bmi088_set_accel_meas_conf(struct bmi088_dev *dev);

    /*!
     *  @brief This API sets the power mode of the accel sensor.
     *
     *  @param[in] dev  : Structure instance of bmi088_dev.
     *
     *  @return Result of API execution status
     *  @retval zero -> Success
     *  @retval Any non zero value -> Fail
     */
    uint16_t bmi088_set_accel_power_mode(struct bmi088_dev *dev);

    /*!
     *  @brief This API reads the accel data from the sensor,
     *  store it in the bmi088_sensor_data structure instance
     *  passed by the user.
     *
     *  @param[out] accel  : Structure pointer to store accel data
     *  @param[in]  dev    : Structure instance of bmi088_dev.
     *
     *
     *  @return Result of API execution status
     *  @retval zero -> Success
     *  @retval Any non zero value -> Fail
     */
    uint16_t bmi088_get_accel_data(struct bmi088_sensor_data *accel, struct bmi088_dev *dev);

    /*!
     *  @brief This API configures the necessary accel interrupt
     *  based on the user settings in the bmi088_int_cfg
     *  structure instance.
     *
     *  @param[in] int_config  : Structure instance of bmi088_int_cfg.
     *  @param[in] dev         : Structure instance of bmi088_dev.
     *  @note : Refer user guide for detailed info.
     *
     *  @return Result of API execution status
     *  @retval zero -> Success
     *  @retval Any non zero value -> Fail
     */
    uint16_t bmi088_set_accel_int_config(const struct bmi088_int_cfg *int_config, struct bmi088_dev *dev);

    /*!
     *  @brief This API switches accel sensor on or off.
     *
     *  @param[in]  dev             : Structure instance of bmi088_dev.
     *  @param[in]  switch_input    : Input to switch accel on or off
     *  Value    |  Description
     *  ---------|--------------------
     *   0       | BMI088_ACCEL_POWER_DISABLE
     *   4       | BMI088_ACCEL_POWER_ENABLE
     *
     *
     *  @return Result of API execution status
     *  @retval zero -> Success
     *  @retval Any non zero value -> Fail
     */
    uint16_t bmi088_accel_switch_control(struct bmi088_dev *dev, uint8_t switch_input);

    /*!
     *  @brief This API reads the temperature of the sensor in � Celcius.
     *
     *  @param[in]  dev             : Structure instance of bmi088_dev.
     *  @param[out] sensor_temp     : Pointer to store sensor temperature in � Celcius
     *
     *  @return Result of API execution status
     *  @retval zero -> Success
     *  @retval Any non zero value -> Fail
     */
    uint16_t bmi088_get_sensor_temperature(struct bmi088_dev *dev, float *sensor_temp);

    /*!
     *  @brief This API reads the sensor time of the sensor.
     *
     *  @param[in]  dev             : Structure instance of bmi088_dev.
     *  @param[out] sensor_time     : Pointer to store sensor time
     *
     *  @return Result of API execution status
     *  @retval zero -> Success
     *  @retval Any non zero value -> Fail
     */
    uint16_t bmi088_get_sensor_time(struct bmi088_dev *dev, uint32_t *sensor_time);

    /*!
     *  @brief This API checks whether the self test functionality of the sensor
     *  is working or not
     *
     *  @param[in] result : Pointer variable used to store the result of self test
     *  operation.
     *  result   |  Description
     *  ---------|--------------------
     *   0       | BMI088_SELFTEST_PASS
     *  -1       | BMI088_SELFTEST_FAIL
     *
     *  @param[in] dev    : Structure instance of bmi088_dev
     *
     *  @return results of self test
     *  @retval 0 -> Success
     *  @retval Any non zero value -> Fail
     */
    uint16_t bmi088_perform_accel_selftest(int8_t *result, struct bmi088_dev *dev);

    /*!
     *  @brief This API enables or disables the Accel Self test feature in the
     *  sensor.
     *
     *  @param[in] selftest : Variable used to enable or disable
     *  the Accel self test feature
     *  Value   |  Description
     *  --------|---------------
     *  0x00    | BMI088_ACCEL_SWITCH_OFF_SELF_TEST
     *  0x0D    | BMI088_ACCEL_POSITIVE_SELF_TEST
     *  0x09    | BMI088_ACCEL_NEGATIVE_SELF_TEST
     *
     *  @param[in] dev      : Structure instance of bmi088_dev
     *
     *  @return Result of API execution status
     *  @retval 0 -> Success
     *  @retval Any non zero value -> Fail
     */
    uint16_t bmi088_set_accel_selftest(uint8_t selftest, struct bmi088_dev *dev);

    /*********************** BMI088 Gyroscope function prototypes ************************/

    /*!
     *  @brief This API is the entry point for gyro sensor.
     *  It performs the selection of I2C/SPI read mechanism according to the
     *  selected interface and reads the chip-id of gyro sensor.
     *
     *  @param[in,out] dev  : Structure instance of bmi088_dev.
     *  @note : Refer user guide for detailed info.
     *
     *  @return Result of API execution status
     *  @retval zero -> Success
     *  @retval Any non zero value -> Fail
     */
    uint16_t bmi088_gyro_init(struct bmi088_dev *dev);

    /*!
     *  @brief This API reads the data from the given register address of gyro sensor.
     *
     *  @param[in] reg_addr  : Register address from where the data to be read
     *  @param[out] data     : Pointer to data buffer to store the read data.
     *  @param[in] len       : No. of bytes of data to be read.
     *  @param[in] dev       : Structure instance of bmi088_dev.
     *
     *  @return Result of API execution status
     *  @retval zero -> Success
     *  @retval Any non zero value -> Fail
     */
    uint16_t bmi088_get_gyro_regs(uint8_t reg_addr, uint8_t *data, uint16_t len, struct bmi088_dev *dev);

    /*!
     *  @brief This API writes the given data to the register address
     *  of gyro sensor.
     *
     *  @param[in] reg_addr  : Register address to where the data to be written.
     *  @param[in] data      : Pointer to data buffer which is to be written
     *  in the sensor.
     *  @param[in] len       : No. of bytes of data to write.
     *  @param[in] dev       : Structure instance of bmi088_dev.
     *
     *  @return Result of API execution status
     *  @retval zero -> Success
     *  @retval Any non zero value -> Fail
     */
    uint16_t bmi088_set_gyro_regs(uint8_t reg_addr, uint8_t *data, uint16_t len, struct bmi088_dev *dev);

    /*!
     *  @brief This API resets the gyro sensor.
     *
     *  @param[in] dev : Structure instance of bmi088_dev.
     *
     *  @return Result of API execution status
     *  @retval zero -> Success
     *  @retval Any non zero value -> Fail
     */
    uint16_t bmi088_gyro_soft_reset(struct bmi088_dev *dev);

    /*!
     *  @brief This API sets the output data rate, range and bandwidth
     *  of gyro sensor.
     *
     *  @param[in] dev : Structure instance of bmi088_dev.
     *  @note : Refer user guide for detailed info.
     *
     *  @return Result of API execution status
     *  @retval zero -> Success
     *  @retval Any non zero value -> Fail
     */
    uint16_t bmi088_set_gyro_meas_conf(struct bmi088_dev *dev);

    /*!
     *  @brief This API sets the power mode of the gyro sensor.
     *
     *  @param[in] dev : Structure instance of bmi088_dev.
     *
     *  @return Result of API execution status
     *  @retval zero -> Success
     *  @retval Any non zero value -> Fail
     */
    uint16_t bmi088_set_gyro_power_mode(struct bmi088_dev *dev);

    /*!
     *  @brief This API reads the gyro data from the sensor,
     *  store it in the bmi088_sensor_data structure instance
     *  passed by the user.
     *
     *  @param[out] gyro   : Structure pointer to store gyro data
     *  @param[in] dev     : Structure instance of bmi088_dev.
     *
     *  @return Result of API execution status
     *  @retval zero -> Success
     *  @retval Any non zero value -> Fail
     */
    uint16_t bmi088_get_gyro_data(struct bmi088_sensor_data *gyro, struct bmi088_dev *dev);

    /*!
     *  @brief This API configures the necessary gyro interrupt
     *  based on the user settings in the bmi088_int_cfg
     *  structure instance.
     *
     *  @param[in] int_config  : Structure instance of bmi088_int_cfg.
     *  @param[in] dev         : Structure instance of bmi088_dev.
     *  @note : Refer user guide for detailed info.
     *
     *  @return Result of API execution status
     *  @retval zero -> Success
     *  @retval Any non zero value -> Fail
     */
    uint16_t bmi088_set_gyro_int_config(const struct bmi088_int_cfg *int_config, struct bmi088_dev *dev);

    /*!
     *  @brief This API enables or disables the Gyro Self test feature in the
     *  sensor.
     *
     *  @param[in] selftest : Variable used to enable or disable
     *  the Gyro self test feature
     *  Value   |  Description
     *  --------|---------------
     *  0x00    | BMI088_DISABLE
     *  0x01    | BMI088_ENABLE
     *
     *  @param[in] dev      : Structure instance of bmi088_dev
     *
     *  @return Result of API execution status
     *  @retval 0 -> Success
     *  @retval Any non zero value -> Fail
     *
     */
    uint16_t bmi088_set_gyro_selftest(uint8_t selftest, struct bmi088_dev *dev);

    /*!
     *  @brief This API checks whether the self test functionality of the
     *  gyro sensor is working or not
     *
     *  @param[in] result : Pointer variable used to store the result of
     *  self test operation
     *  result   |  Description
     *  ---------|--------------------
     *   0       | BMI088_SELFTEST_PASS
     *  -1       | BMI088_SELFTEST_FAIL
     *
     *  @param[in]  dev    : Structure instance of bmi088_dev.
     *
     *  @return Result of API execution status
     *  @retval zero -> Success
     *  @retval Any non zero value -> Fail
     */
    uint16_t bmi088_perform_gyro_selftest(int8_t *result, struct bmi088_dev *dev);

#ifdef __cplusplus
}
#endif

#endif
/* End of BMI088_H_ */
