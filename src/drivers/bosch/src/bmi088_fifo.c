/*
 ****************************************************************************
 * Copyright (C) 2015 - 2016 Bosch Sensortec GmbH
 *
 * File :bmi088_fifo.c
 *
 * Date: 30 Oct 2017
 *
 * Revision:
 *
 * Usage: Sensor Driver for BMI088 family of sensors
 *
 ****************************************************************************
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
/*! \file bmi088_fifo.c
 \brief Sensor Driver for BMI088 family of sensors */
/***************************************************************************/
/**\name        Header files
 ****************************************************************************/
#ifdef USE_FIFO
#include "bmi088_fifo.h"

/***************************************************************************/
/**\name        Local structures
 ****************************************************************************/

/***************************************************************************/
/*! Static Function Declarations
 ****************************************************************************/

/*!
 * @brief This API is used to validate the device structure pointer for
 * null conditions.
 *
 * @param[in] dev         : Structure instance of bmi088_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success
 * @retval Any non zero value -> Fail
 *
 */
static uint16_t null_ptr_check(const struct bmi088_dev *dev);

/*!
 * @brief This API is used to reset the accel FIFO related configurations
 * in the fifo_frame structure.
 *
 * @param dev[in,out]  : Structure instance of bmi088_dev
 *
 * @return None
 */
static void reset_accel_fifo_data_structure(const struct bmi088_dev *dev);

/*!
 * @brief This API computes the number of bytes of accel FIFO data
 * which is to be parsed in header-less mode
 *
 * @param[out] start_idx   : The start index for parsing data
 * @param[out] len         : Number of bytes to be parsed
 * @param[in]  accel_count   : Number of accelerometer frames to be read
 * @param[in]  dev         : Structure instance of bmi088_dev.
 *
 * @return None
 */
static void get_accel_len_to_parse(uint16_t *start_idx, uint16_t *len, const uint16_t *accel_count, const struct bmi088_dev *dev);

/*!
 * @brief This API checks the accel fifo read data as empty frame, if it
 * is empty frame then moves the index to last byte.
 *
 * @param[in,out] data_index   : The index of the current data to
 *                               be parsed from accelerometer fifo data
 * @param[in]  dev             : Structure instance of bmi088_dev.
 *
 * @return None
 */
static void check_empty_accel_fifo(uint16_t *data_index, const struct bmi088_dev *dev);

/*!
 * @brief This API is used to parse the accelerometer data from the
 * FIFO data in header mode.
 *
 * @param[in,out] accel_data   : Structure instance of bmi088_sensor_data where
 *                                the accelerometer data in FIFO is stored.
 * @param[in,out] accel_length : Number of accelerometer frames
 *                                (x,y,z axes data)
 * @param[in,out] dev          : Structure instance of bmi088_dev.
 *
 * @return None
 */
static void extract_accel_header_mode(struct bmi088_sensor_data *accel_data, uint16_t *accel_length, const struct bmi088_dev *dev);

/*!
 * @brief This API is used to parse the accelerometer data from the
 * FIFO data in both header mode and header-less mode.
 * It updates the idx value which is used to store the index of
 * the current data byte which is parsed.
 *
 * @param[in,out] accel       : Structure instance of bmi088_sensor_data.
 * @param[in,out] idx       : Index value of number of bytes parsed
 * @param[in,out] accel_idx   : Index value of accelerometer data
 *                             (x,y,z axes) frame to be parsed
 * @param[in] frm           : It consists of either fifo_data_enable
 *                            parameter (Accel enabled in FIFO)
 *                            in header-less mode or frame header data
 *                            in header mode
 * @param[in] dev           : Structure instance of bmi088_dev.
 *
 * @return None
 */
static void unpack_accel_frm(struct bmi088_sensor_data *accel, uint16_t *idx, uint16_t *accel_idx, uint8_t frm, const struct bmi088_dev *dev);

/*!
 * @brief This API is used to parse the accelerometer data from the
 * FIFO data and store it in the instance of the structure bmi088_sensor_data.
 *
 * @param[out] accel_data        : Structure instance of bmi088_sensor_data where
 *                                 the parsed accel data bytes are stored.
 * @param[in] data_start_index   : Index value of the accel data bytes
 *                                 which is to be parsed from the fifo data.
 * @param[in] dev                : Structure instance of bmi088_dev.
 *
 * @return None
 *
 */
static void unpack_accel_data(struct bmi088_sensor_data *accel_data, uint16_t data_start_index, const struct bmi088_dev *dev);

/*!
 * @brief This API is used to parse and store the sensor time from the
 * FIFO data in the structure instance dev.
 *
 * @param[in,out] data_index : Index of the FIFO data which
 *                             has the sensor time.
 * @param[in,out] dev        : Structure instance of bmi088_dev.
 *
 * @return None
 */
static void unpack_sensortime_frame(uint16_t *data_index, const struct bmi088_dev *dev);

/*!
 * @brief This API is used to parse and store the skipped_frame_count from
 * the accelerometer FIFO data in the structure instance dev.
 *
 * @param[in,out] data_index       : Index of the FIFO data which
 *                                   has the skipped frame count.
 * @param[in,out] dev              : Structure instance of bmi088_dev.
 *
 * @return None
 */
static void unpack_accel_skipped_frame(uint16_t *data_index, const struct bmi088_dev *dev);

/*!
 * @brief This API is used to parse and store the dropped_frame_count from
 * the accelerometer FIFO data in the structure instance dev.
 *
 * @param[in,out] data_index       : Index of the FIFO data which
 *                                   has the dropped frame data.
 * @param[in,out] dev              : Structure instance of bmi088_dev.
 *
 * @return None
 */
static void unpack_accel_dropped_frame(uint16_t *data_index, const struct bmi088_dev *dev);

/*!
 * @brief This API is used to move the data index ahead of the
 * current_frame_length parameter when unnecessary FIFO data appears while
 * extracting the accelerometer data.
 *
 * @param[in,out] data_index       : Index of the FIFO data which
 *                                    is to be moved ahead of the
 *                                    current_frame_length
 * @param[in] current_frame_length : Number of bytes in a particular frame
 * @param[in] dev                  : Structure instance of bmi088_dev.
 *
 * @return None
 */
static void move_to_next_accel_frame(uint16_t *data_index, uint8_t current_frame_length, const struct bmi088_dev *dev);

/*!
 * @brief This API is used to reset the gyro FIFO related configurations
 * in the fifo_frame structure.
 *
 * @param dev[in,out]  : Structure instance of bmi088_dev
 *
 * @return None
 */
static void reset_gyro_fifo_data_structure(const struct bmi088_dev *dev);

/*!
 * @brief This API computes the number of bytes of Gyro FIFO data which is
 * to be parsed in header-less mode
 *
 * @param[out] start_idx   : The start index for parsing data
 * @param[out] len         : Number of bytes to be parsed
 * @param[in]  gyro_count  : Number of gyroscope frames to be read
 * @param[in]  dev         : Structure instance of bmi088_dev.
 *
 * @return None
 */
static void get_gyro_len_to_parse(uint16_t *start_idx, uint16_t *len, const uint16_t *gyro_count, const struct bmi088_dev *dev);

/*!
 *  @brief This API is used to parse the gyroscope data from the
 *  FIFO data in header-less mode and update the idx value
 *  which is used to store the index of the current data byte
 *  which is parsed.
 *
 *  @param[in,out] data     : Structure instance of bmi088_sensor_data.
 *  @param[in,out] idx      : Index value of number of bytes parsed
 *  @param[in,out] gyro_idx : Index value gyroscope data frame (x,y,z,r)
 *                            to be parsed
 *  @param[in] frm          : It consists of either the fifo_data_enable parameter
 *                            (Gyro data enabled in FIFO) in header-less mode
 *  @param[in] dev          : Structure instance of bmi088_dev.
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 *
 */
static uint16_t unpack_gyro_frm(struct bmi088_sensor_data *data, uint16_t *idx, uint16_t *gyro_idx, uint8_t frm, const struct bmi088_dev *dev);

/*!
 * @brief This API is used to parse the  gyroscope data from
 * the FIFO data and store it in the instance of the structure
 * gyro_data.
 *
 * @param[out] gyro_data        : Structure instance of bmi088_sensor_data where the
 *                                 parsed gyroscope data bytes are stored.
 * @param[in] start_idx         : Index value of the gyroscope data bytes
 *                                 which is to be parsed from the FIFO data
 * @param[in] dev               : Structure instance of bmi088_dev.
 *
 * @return None
 */
static void unpack_gyro_data(struct bmi088_sensor_data *gyro_data, uint16_t start_idx, const struct bmi088_dev *dev);

/*!
 * @brief This API checks the gyro fifo read data as empty frame, if it
 * is empty frame then moves the index to last byte.
 *
 * @param[in,out] data_index   : The index of the current data to
 *                                be parsed from gyroscope fifo data
 * @param[in]  dev             : Structure instance of bmi088_dev.
 *
 * @return None
 */
static void check_empty_gyro_fifo(uint16_t *data_index, const struct bmi088_dev *dev);

/***************************************************************************/
/**\name        Extern Declarations
 ****************************************************************************/

/***************************************************************************/
/**\name        Globals
 ****************************************************************************/

/***************************************************************************/
/**\name        Function definitions
 ****************************************************************************/

/*!
 *  @brief This API reads the FIFO data of Accel sensor.
 */
uint16_t bmi088_get_accel_fifo_data(struct bmi088_dev *dev)
{
    uint16_t rslt = 0;
    uint8_t data = 0;
    uint8_t reg_addr = BMI088_ACCEL_FIFO_DATA_REG;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    /* Proceed if null check is fine */
    if (rslt == BMI088_OK)
    {
        /* Reset the Accel FIFO related configurations */
        reset_accel_fifo_data_structure(dev);

        if (dev->interface == BMI088_SPI_INTF)
        {
            reg_addr = reg_addr | BMI088_SPI_RD_MASK;
        }

        /* Read Accel FIFO data*/
        rslt |= dev->read(dev->accel_id, reg_addr, dev->accel_fifo->data, dev->accel_fifo->length);
        reg_addr = BMI088_ACCEL_FIFO_CONFIG_1_REG;
        /* Read fifo frame content configuration*/
        rslt |= bmi088_get_accel_regs(reg_addr, &data, BMI088_ONE, dev);
        /* Filter fifo header enabled status */
        dev->accel_fifo->fifo_header_enable = data & BMI088_FIFO_HEADER;
        /* Filter accel data enabled status */
        dev->accel_fifo->fifo_data_enable = data & BMI088_FIFO_A_ENABLE;
    }

    return rslt;
}

/*!
 *  @brief This API parses and extracts the accelerometer frames from
 *  FIFO data read by the "bmi088_get_accel_fifo_data" API and stores
 *  it in the "accel_data" structure instance.
 */
uint16_t bmi088_extract_accel(struct bmi088_sensor_data *accel_data, uint16_t *accel_length, const struct bmi088_dev *dev)
{
    uint16_t rslt = 0;
    uint16_t data_index = 0;
    uint16_t accel_index = 0;
    uint16_t data_read_length = 0;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    /* Proceed if null check is fine */
    if (rslt == BMI088_OK)
    {
        /* Parsing the Accel FIFO data in header-less mode */
        if (dev->accel_fifo->fifo_header_enable == 0)
        {
            /* Collects the number of bytes of Accel FIFO data
             * which is to be parsed in header-less mode */
            get_accel_len_to_parse(&data_index, &data_read_length, accel_length, dev);

            for (; data_index < data_read_length;)
            {
                /* Unpack Accel data from fifo buffer */
                unpack_accel_frm(accel_data, &data_index, &accel_index, dev->accel_fifo->fifo_data_enable, dev);
                /*Check for the availability of next
                 two bytes of accel FIFO data */
                check_empty_accel_fifo(&data_index, dev);
            }
            /* update number of Accel data read*/
            *accel_length = accel_index;
            /*update the Accel byte index*/
            dev->accel_fifo->byte_start_idx = data_index;
        }
        else
        {
            /* Parsing the accel FIFO data in header mode */
            extract_accel_header_mode(accel_data, accel_length, dev);
        }
    }

    return rslt;
}

/*!
 *  @brief This API reads the accel FIFO water mark level which is set
 *  in the sensor.
 */
uint16_t bmi088_get_accel_fifo_wm(uint16_t *fifo_wm, struct bmi088_dev *dev)
{
    uint16_t rslt = 0;
    uint8_t data[2] = { 0, 0 };
    uint8_t reg_addr;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    /* Proceed if null check is fine */
    if (rslt == BMI088_OK)
    {
        reg_addr = BMI088_ACCEL_FIFO_WTM_0_REG;
        /* Read the Accel FIFO watermark level*/
        rslt |= bmi088_get_accel_regs(reg_addr, data, BMI088_TWO, dev);

        if (rslt == BMI088_OK)
        {
            /* Update the fifo watermark level */
            *fifo_wm = (data[1] << BMI088_EIGHT) | (data[0]);
        }
    }

    return rslt;
}

/*!
 *  @brief This API sets the accel FIFO watermark level in the sensor.
 */
uint16_t bmi088_set_accel_fifo_wm(uint16_t fifo_wm, struct bmi088_dev *dev)
{
    uint16_t rslt = 0;
    uint8_t data[2] = { 0, 0 };
    uint8_t reg_addr;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    /* Proceed if null check is fine */
    if (rslt == BMI088_OK)
    {
        data[0] = BMI088_GET_LSB(fifo_wm);
        data[1] = BMI088_GET_MSB(fifo_wm);

        /* consecutive write is not possible in suspend mode hence
         separate write is used with delay of 1 ms*/
        /* Write the Accel fifo watermark level*/
        reg_addr = BMI088_ACCEL_FIFO_WTM_0_REG;
        rslt |= bmi088_set_accel_regs(reg_addr, &data[0], BMI088_ONE, dev);
        dev->delay_ms(BMI088_ONE);
        /* Write the Accel fifo watermark level */
        rslt |= bmi088_set_accel_regs((reg_addr + BMI088_ONE), &data[1], BMI088_ONE, dev);
    }

    return rslt;
}

/*!
 *  @brief This API checks whether the Accel FIFO data is set for filtered
 *  or unfiltered mode.
 */
uint16_t bmi088_get_accel_fifo_filt_data(uint8_t *accel_fifo_filter, struct bmi088_dev *dev)
{
    uint16_t rslt = 0;
    uint8_t data = 0, reg_addr;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    /* Proceed if null check is fine */
    if (rslt == BMI088_OK)
    {
        reg_addr = BMI088_ACCEL_FIFO_DOWN_REG;
        /* Read the Accel FIFO filter data */
        rslt |= bmi088_get_accel_regs(reg_addr, &data, BMI088_ONE, dev);

        if (rslt == BMI088_OK)
        {
            /* Update the accel fifo filter info */
            *accel_fifo_filter = BMI088_GET_BITSLICE(data, BMI088_ACCEL_FIFO_FILT_DATA);
        }
    }

    return rslt;
}

/*!
 *  @brief This API sets the condition of Accel FIFO data either to
 *  filtered or unfiltered mode.
 */
uint16_t bmi088_set_accel_fifo_filt_data(uint8_t accel_fifo_filter, struct bmi088_dev *dev)
{
    uint16_t rslt = 0;
    uint8_t data = 0, reg_addr;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    /* Proceed if null check is fine */
    if (rslt == BMI088_OK)
    {
        if (accel_fifo_filter <= BMI088_MAX_VALUE_FIFO_FILTER)
        {
            /* Read accel fifo down register */
            reg_addr = BMI088_ACCEL_FIFO_DOWN_REG;
            rslt |= bmi088_get_accel_regs(reg_addr, &data, BMI088_ONE, dev);

            if (rslt == BMI088_OK)
            {
                /* Write Accel FIFO filter data */
                data = BMI088_SET_BITSLICE(data, BMI088_ACCEL_FIFO_FILT_DATA, accel_fifo_filter);
                rslt |= bmi088_set_accel_regs(reg_addr, &data, BMI088_ONE, dev);
            }
        }
        else
        {
            rslt |= BMI088_E_OUT_OF_RANGE;
        }
    }
    return rslt;
}

/*!
 *  @brief This API reads the down sampling rates which is configured
 *  for Accel FIFO data.
 */
uint16_t bmi088_get_fifo_down_accel(uint8_t *fifo_down, struct bmi088_dev *dev)
{
    uint16_t rslt = 0;
    uint8_t data = 0, reg_addr;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    /* Proceed if null check is fine */
    if (rslt == BMI088_OK)
    {
        reg_addr = BMI088_ACCEL_FIFO_DOWN_REG;
        /* Read the Accel FIFO down data */
        rslt |= bmi088_get_accel_regs(reg_addr, &data, BMI088_ONE, dev);

        if (rslt == BMI088_OK)
        {
            /* Update the fifo down value */
            *fifo_down = BMI088_GET_BITSLICE(data, BMI088_ACCEL_FIFO_FILT_DOWN);
        }
    }

    return rslt;
}

/*!
 *  @brief This API sets the down-sampling rates for Accel FIFO.
 */
uint16_t bmi088_set_fifo_down_accel(uint8_t fifo_down, struct bmi088_dev *dev)
{
    uint16_t rslt = 0;
    uint8_t data = 0, reg_addr;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    /* Proceed if null check is fine */
    if (rslt == BMI088_OK)
    {
        if (fifo_down <= BMI088_MAX_VALUE_FIFO_DOWN)
        {
            reg_addr = BMI088_ACCEL_FIFO_DOWN_REG;
            /* Read the Accel FIFO down register */
            rslt |= bmi088_get_accel_regs(reg_addr, &data, BMI088_ONE, dev);

            if (rslt == BMI088_OK)
            {
                data = BMI088_SET_BITSLICE(data, BMI088_ACCEL_FIFO_FILT_DOWN, fifo_down);
                /* Write the Accel FIFO down data */
                rslt |= bmi088_set_accel_regs(reg_addr, &data, BMI088_ONE, dev);
            }
        }
        else
        {
            rslt |= BMI088_E_OUT_OF_RANGE;
        }
    }
    return rslt;
}

/*!
 *  @brief This API reads the length of FIFO data available in the
 *  Accel sensor in the units of bytes.
 */
uint16_t bmi088_get_accel_fifo_length(uint16_t *fifo_length, struct bmi088_dev *dev)
{
    uint16_t rslt = 0;
    uint8_t index = 0, reg_addr;
    uint8_t data[2] = { 0, 0 };

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);
    /* Proceed if null check is fine */
    if (rslt == BMI088_OK)
    {
        reg_addr = BMI088_ACCEL_FIFO_LENGTH_0_REG;
        /* Read Accel FIFO length */
        rslt |= bmi088_get_accel_regs(reg_addr, data, BMI088_TWO, dev);

        if (rslt == BMI088_OK)
        {
            /* Update fifo length */
            index = BMI088_ONE;
            data[index] = BMI088_GET_BITSLICE(data[index], BMI088_FIFO_BYTE_COUNTER_MSB);
            *fifo_length = ((data[index] << BMI088_EIGHT) | data[index - BMI088_ONE]);
        }
    }

    return rslt;
}

/*!
 *  @brief This API sets the FIFO full interrupt of the accel sensor.
 */
uint16_t bmi088_set_accel_fifo_full_int(struct bmi088_int_cfg *int_config, struct bmi088_dev *dev)
{
    uint16_t rslt = 0;
    uint8_t data = 0, reg_addr;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    /* Proceed if null check is fine */
    if (rslt == BMI088_OK)
    {
        /* update reg_addr based on channel inputs */
        if (int_config->accel_int_channel == BMI088_INT_CHANNEL_1)
        {
            reg_addr = BMI088_ACCEL_INT1_IO_CONF_REG;
        }

        if (int_config->accel_int_channel == BMI088_INT_CHANNEL_2)
        {
            reg_addr = BMI088_ACCEL_INT2_IO_CONF_REG;
        }

        /* Read interrupt pin configuration register */
        rslt = bmi088_get_accel_regs(reg_addr, &data, BMI088_ONE, dev);

        if (rslt == BMI088_OK)
        {
            /* Update data with user configured bmi088_int_cfg structure */
            data = BMI088_SET_BITSLICE(data, BMI088_ACCEL_INT2_LVL, int_config->accel_int_pin_cfg.lvl);
            data = BMI088_SET_BITSLICE(data, BMI088_ACCEL_INT2_OD, int_config->accel_int_pin_cfg.output_mode);
            data = BMI088_SET_BITSLICE(data, BMI088_ACCEL_INT2_IO, int_config->accel_int_pin_cfg.enable_int_pin);

            /* Write to interrupt pin configuration register */
            rslt = bmi088_set_accel_regs(reg_addr, &data, BMI088_ONE, dev);

            if (rslt == BMI088_OK)
            {
                reg_addr = BMI088_ACCEL_INT1_INT2_MAP_DATA_REG;
                /* Read the Accel Interrupt Map Register */
                rslt = bmi088_get_accel_regs(reg_addr, &data, BMI088_ONE, dev);

                if (rslt == BMI088_OK)
                {
                    /* Update data to map fifo full interrupt based on channel inputs */
                    if (int_config->accel_int_channel == BMI088_INT_CHANNEL_1)
                    {
                        data = BMI088_SET_BITSLICE(data, BMI088_ACCEL_INT1_FIFO_FULL, BMI088_ONE);
                    }
                    else if (int_config->accel_int_channel == BMI088_INT_CHANNEL_2)
                    {
                        data = BMI088_SET_BITSLICE(data, BMI088_ACCEL_INT2_FIFO_FULL, BMI088_ONE);
                    }
                    else
                    {
                        /* do nothing */
                    }

                    /* Write the Accel Interrupt Map Register */
                    rslt = bmi088_set_gyro_regs(reg_addr, &data, BMI088_ONE, dev);

                }
            }
        }
    }

    return rslt;

}

/*!
 *  @brief This API sets the FIFO watermark interrupt of the accel sensor.
 */
uint16_t bmi088_set_accel_fifo_wm_int(struct bmi088_int_cfg *int_config, struct bmi088_dev *dev)
{
    uint16_t rslt = 0;
    uint8_t data = 0, reg_addr;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    /* Proceed if null check is fine */
    if (rslt == BMI088_OK)
    {
        /* update reg_addr based on channel inputs */
        if (int_config->accel_int_channel == BMI088_INT_CHANNEL_1)
        {
            reg_addr = BMI088_ACCEL_INT1_IO_CONF_REG;
        }

        if (int_config->accel_int_channel == BMI088_INT_CHANNEL_2)
        {
            reg_addr = BMI088_ACCEL_INT2_IO_CONF_REG;
        }

        /* Read interrupt pin configuration register */
        rslt = bmi088_get_accel_regs(reg_addr, &data, BMI088_ONE, dev);

        if (rslt == BMI088_OK)
        {
            /* Update data with user configured bmi088_int_cfg structure */
            data = BMI088_SET_BITSLICE(data, BMI088_ACCEL_INT2_LVL, int_config->accel_int_pin_cfg.lvl);
            data = BMI088_SET_BITSLICE(data, BMI088_ACCEL_INT2_OD, int_config->accel_int_pin_cfg.output_mode);
            data = BMI088_SET_BITSLICE(data, BMI088_ACCEL_INT2_IO, int_config->accel_int_pin_cfg.enable_int_pin);

            /* Write to interrupt pin configuration register */
            rslt = bmi088_set_accel_regs(reg_addr, &data, BMI088_ONE, dev);

            if (rslt == BMI088_OK)
            {
                reg_addr = BMI088_ACCEL_INT1_INT2_MAP_DATA_REG;
                /* Read the Accel Interrupt Map Register */
                rslt = bmi088_get_accel_regs(reg_addr, &data, BMI088_ONE, dev);

                if (rslt == BMI088_OK)
                {
                    /* Update data to map watermark interrupt based on channel inputs */
                    if (int_config->accel_int_channel == BMI088_INT_CHANNEL_1)
                    {
                        data = BMI088_SET_BITSLICE(data, BMI088_ACCEL_INT1_FIFO_WM, BMI088_ONE);
                    }
                    else if (int_config->accel_int_channel == BMI088_INT_CHANNEL_2)
                    {
                        data = BMI088_SET_BITSLICE(data, BMI088_ACCEL_INT2_FIFO_WM, BMI088_ONE);
                    }
                    else
                    {
                        /* do nothing */
                    }

                    /* Write to the Accel Interrupt Map Register */
                    rslt = bmi088_set_gyro_regs(reg_addr, &data, BMI088_ONE, dev);
                }
            }
        }
    }

    return rslt;

}

/*!
 *  @brief This API reads the FIFO data of Gyro sensor.
 */
uint16_t bmi088_get_gyro_fifo_data(struct bmi088_dev *dev)
{
    uint16_t rslt = 0;
    uint8_t reg_addr = BMI088_GYRO_FIFO_DATA_REG;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    /* Proceed if null check is fine */
    if (rslt == BMI088_OK)
    {
        /* Reset the gyro FIFO related configurations */
        reset_gyro_fifo_data_structure(dev);

        /* Read FIFO data*/
        if (dev->interface == BMI088_SPI_INTF)
        {
            reg_addr = reg_addr | BMI088_SPI_RD_MASK;
        }

        rslt |= dev->read(dev->gyro_id, reg_addr, dev->gyro_fifo->data, dev->gyro_fifo->length);

        /* Update the data select axes info in fifo_data_enable variable found in device structure */
        rslt |= bmi088_get_gyro_fifo_data_sel(&dev->gyro_fifo->fifo_data_enable, dev);

    }

    return rslt;
}

/*!
 *  @brief This API parses and extracts the gyroscope frames from
 *  FIFO data read by the "bmi088_get_gyro_fifo_data" API and
 *  stores it in the "gyro_data" structure instance parameter of
 *  this API.
 */
uint16_t bmi088_extract_gyro(struct bmi088_sensor_data *gyro_data, uint16_t *gyro_length, const struct bmi088_dev *dev)
{
    uint16_t rslt = 0;
    uint16_t data_index = 0;
    uint16_t gyro_index = 0;
    uint16_t data_read_length = 0;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    /* Proceed if null check is fine */
    if (rslt == BMI088_OK)
    {
        /* Collects the number of bytes of Gyro FIFO data
         * which is to be parsed */
        get_gyro_len_to_parse(&data_index, &data_read_length, gyro_length, dev);

        for (; data_index < data_read_length;)
        {
            /* Unpack Gyro data from fifo buffer */
            rslt |= unpack_gyro_frm(gyro_data, &data_index, &gyro_index, dev->gyro_fifo->fifo_data_enable, dev);
            /* Check for the availability of next
             two bytes of FIFO data */
            check_empty_gyro_fifo(&data_index, dev);
        }

        /* Update number of gyro data read*/
        *gyro_length = gyro_index;
        /* Update the Gyro byte index*/
        dev->gyro_fifo->byte_start_idx = data_index;
    }

    return rslt;

}

/*!
 *  @brief This API reads the Gyro FIFO water mark level which is set
 *  in the sensor.
 */
uint16_t bmi088_get_gyro_fifo_wm(uint8_t *fifo_wm, struct bmi088_dev *dev)
{
    uint16_t rslt = 0;
    uint8_t data, reg_addr;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    /* Proceed if null check is fine */
    if (rslt == BMI088_OK)
    {
        reg_addr = BMI088_GYRO_FIFO_CONFIG_0_REG;
        /* Read the Gyro FIFO water mark level*/
        rslt = bmi088_get_gyro_regs(reg_addr, &data, BMI088_ONE, dev);

        if (rslt == BMI088_OK)
        {
            /* Update the fifo watermark level */
            *fifo_wm = BMI088_GET_BITSLICE(data, BMI088_GYRO_FIFO_WM);
        }
    }

    return rslt;
}

/*!
 *  @brief This API sets the gyro FIFO watermark level in the sensor.
 */
uint16_t bmi088_set_gyro_fifo_wm(uint8_t fifo_wm, struct bmi088_dev *dev)
{
    uint16_t rslt = 0;
    uint8_t data, reg_addr;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    /* Proceed if null check is fine */
    if (rslt == BMI088_OK)
    {
        if (fifo_wm <= BMI088_GYRO_FIFO_WM_MAX)
        {
            /* Read the fifo config 0 register */
            reg_addr = BMI088_GYRO_FIFO_CONFIG_0_REG;
            rslt |= bmi088_get_gyro_regs(reg_addr, &data, BMI088_ONE, dev);

            /* Write the fifo watermark level*/
            data = BMI088_SET_BITSLICE(data, BMI088_GYRO_FIFO_WM, fifo_wm);
            rslt |= bmi088_set_gyro_regs(reg_addr, &data, BMI088_ONE, dev);
        }
        else
        {
            rslt = BMI088_E_OUT_OF_RANGE;
        }
    }

    return rslt;
}

/*!
 *  @brief This API gets the FIFO operating mode in the gyro sensor.
 */
uint16_t bmi088_get_gyro_fifo_mode(uint8_t *fifo_mode, struct bmi088_dev *dev)
{
    uint16_t rslt = 0;
    uint8_t data, reg_addr;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    /* Proceed if null check is fine */
    if (rslt == BMI088_OK)
    {
        reg_addr = BMI088_GYRO_FIFO_CONFIG_1_REG;
        /* Read the Gyro fifo config 1 register */
        rslt = bmi088_get_gyro_regs(reg_addr, &data, BMI088_ONE, dev);

        if (rslt == BMI088_OK)
        {
            /* Update the fifo mode */
            *fifo_mode = BMI088_GET_BITSLICE(data, BMI088_GYRO_FIFO_MODE);
        }
    }

    return rslt;

}

/*!
 *  @brief This API sets the FIFO operating mode in the gyro sensor.
 */
uint16_t bmi088_set_gyro_fifo_mode(uint8_t fifo_mode, struct bmi088_dev *dev)
{
    uint16_t rslt = 0;
    uint8_t data, reg_addr;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    /* Proceed if null check is fine */
    if (rslt == BMI088_OK)
    {
        if (fifo_mode <= BMI088_GYRO_STREAM_OP_MODE)
        {
            /* Read the fifo config 1 register */
            reg_addr = BMI088_GYRO_FIFO_CONFIG_1_REG;
            rslt |= bmi088_get_gyro_regs(reg_addr, &data, BMI088_ONE, dev);

            /* Write the fifo operating mode */
            data = BMI088_SET_BITSLICE(data, BMI088_GYRO_FIFO_MODE, fifo_mode);
            rslt |= bmi088_set_gyro_regs(reg_addr, &data, BMI088_ONE, dev);
        }
        else
        {
            rslt = BMI088_E_OUT_OF_RANGE;
        }
    }

    return rslt;

}

/*!
 *  @brief This API gets the data of axes to be stored in the FIFO in the gyro sensor.
 */
uint16_t bmi088_get_gyro_fifo_data_sel(uint8_t *fifo_data_select, struct bmi088_dev *dev)
{
    uint16_t rslt = 0;
    uint8_t data, reg_addr;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    /* Proceed if null check is fine */
    if (rslt == BMI088_OK)
    {
        reg_addr = BMI088_GYRO_FIFO_CONFIG_1_REG;
        /* Read the Gyro Fifo config 1 register  */
        rslt = bmi088_get_gyro_regs(reg_addr, &data, BMI088_ONE, dev);

        if (rslt == BMI088_OK)
        {
            /* Update the fifo data select */
            *fifo_data_select = BMI088_GET_BITSLICE(data, BMI088_GYRO_FIFO_DATA_SELECT);
        }
    }

    return rslt;

}

/*!
 *  @brief This API sets the data of axes to be stored in the FIFO in the gyro sensor.
 */
uint16_t bmi088_set_gyro_fifo_data_sel(uint8_t fifo_data_select, struct bmi088_dev *dev)
{
    uint16_t rslt = 0;
    uint8_t data, reg_addr;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    /* Proceed if null check is fine */
    if (rslt == BMI088_OK)
    {
        if (fifo_data_select <= BMI088_GYRO_Z_DATA)
        {
            /* Read the fifo config 1 register */
            reg_addr = BMI088_GYRO_FIFO_CONFIG_1_REG;
            rslt |= bmi088_get_gyro_regs(reg_addr, &data, BMI088_ONE, dev);

            /* Write the fifo operating mode */
            data = BMI088_SET_BITSLICE(data, BMI088_GYRO_FIFO_DATA_SELECT, fifo_data_select);
            rslt |= bmi088_set_gyro_regs(reg_addr, &data, BMI088_ONE, dev);
        }
        else
        {
            rslt = BMI088_E_OUT_OF_RANGE;
        }
    }

    return rslt;

}

/*!
 *  @brief This API reads the length of FIFO data available in the
 *  Gyro sensor in the units of bytes.
 */
uint16_t bmi088_get_gyro_fifo_length(uint8_t *fifo_length, struct bmi088_dev *dev)
{
    uint16_t rslt = 0;
    uint8_t data = 0, reg_addr;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    /* Proceed if null check is fine */
    if (rslt == BMI088_OK)
    {
        reg_addr = BMI088_GYRO_FIFO_STAT_REG;
        /* Read Gyro FIFO length*/
        rslt |= bmi088_get_gyro_regs(reg_addr, &data, BMI088_ONE, dev);

        if (rslt == BMI088_OK)
        {
            /* Update the fifo length */
            *fifo_length = BMI088_GET_BITSLICE(data, BMI088_GYRO_FIFO_COUNTER);
        }
    }

    return rslt;
}

/*!
 *  @brief This API gets the gyro FIFO overrun status.
 */
uint16_t bmi088_get_gyro_fifo_overrun(uint8_t *fifo_overrun_status, struct bmi088_dev *dev)
{
    uint16_t rslt = 0;
    uint8_t data = 0, reg_addr;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    /* Proceed if null check is fine */
    if (rslt == BMI088_OK)
    {
        reg_addr = BMI088_GYRO_FIFO_STAT_REG;
        /* Read Gyro fifo Overrun status */
        rslt |= bmi088_get_gyro_regs(reg_addr, &data, BMI088_ONE, dev);

        if (rslt == BMI088_OK)
        {
            /* Update the fifo overrun status */
            *fifo_overrun_status = BMI088_GET_BITSLICE(data, BMI088_GYRO_FIFO_OVERRUN);
        }
    }

    return rslt;

}

/*!
 *  @brief This API gets the fifo tag status which is set in gyro sensor.
 */
uint16_t bmi088_get_gyro_fifo_tag(uint8_t *fifo_tag_status, struct bmi088_dev *dev)
{
    uint16_t rslt = 0;
    uint8_t data = 0, reg_addr;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    /* Proceed if null check is fine */
    if (rslt == BMI088_OK)
    {
        /* Read the fifo config 0 register */
        reg_addr = BMI088_GYRO_FIFO_CONFIG_0_REG;
        rslt = bmi088_get_gyro_regs(reg_addr, &data, BMI088_ONE, dev);

        if (rslt == BMI088_OK)
        {
            /* Update the fifo tag status */
            *fifo_tag_status = BMI088_GET_BITSLICE(data, BMI088_GYRO_FIFO_TAG);
        }
    }

    return rslt;

}

/*!
 *  @brief This API enables or disable fifo tag in gyro sensor.
 */
uint16_t bmi088_set_gyro_fifo_tag(uint8_t fifo_tag, struct bmi088_dev *dev)
{
    uint16_t rslt = 0;
    uint8_t data, reg_addr;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    /* Proceed if null check is fine */
    if (rslt == BMI088_OK)
    {
        if (fifo_tag <= BMI088_ENABLE)
        {
            /* Read the fifo config 0 register */
            reg_addr = BMI088_GYRO_FIFO_CONFIG_0_REG;
            rslt |= bmi088_get_gyro_regs(reg_addr, &data, BMI088_ONE, dev);

            /* Write the fifo tag */
            data = BMI088_SET_BITSLICE(data, BMI088_GYRO_FIFO_TAG, fifo_tag);
            rslt |= bmi088_set_gyro_regs(reg_addr, &data, BMI088_ONE, dev);
        }
        else
        {
            rslt = BMI088_E_OUT_OF_RANGE;
        }
    }

    return rslt;

}

/*!
 *  @brief This API enables or disables the FIFO full interrupt of the gyro sensor.
 */
uint16_t bmi088_set_gyro_fifo_full_int(struct bmi088_int_cfg *int_config, struct bmi088_dev *dev, uint8_t fifo_full_enable)
{
    uint16_t rslt = 0;
    uint8_t data[2] = { 0, 0 };
    uint8_t reg_addr;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    /* Proceed if null check is fine */
    if (rslt == BMI088_OK)
    {
        /* Read the Interrupt control register and Interrupt configuration register together */
        reg_addr = BMI088_GYRO_INT_CTRL_REG;
        rslt |= bmi088_get_gyro_regs(reg_addr, data, BMI088_TWO, dev);

        if (rslt == BMI088_OK)
        {
            data[0] = BMI088_SET_BITSLICE(data[0], BMI088_GYRO_FIFO_EN, fifo_full_enable);

            /* Interrupt pin or channel 3 */
            if (int_config->gyro_int_channel == BMI088_INT_CHANNEL_3)
            {
                /* Update data with user configured bmi088_int_cfg structure */
                data[1] = BMI088_SET_BITSLICE(data[1], BMI088_GYRO_INT3_LVL, int_config->gyro_int_pin_3_cfg.lvl);
                data[1] = BMI088_SET_BITSLICE(data[1], BMI088_GYRO_INT3_OD, int_config->gyro_int_pin_3_cfg.output_mode);
            }

            /* Interrupt pin or channel 4 */
            if (int_config->gyro_int_channel == BMI088_INT_CHANNEL_4)
            {
                /* Update data with user configured bmi088_int_cfg structure */
                data[1] = BMI088_SET_BITSLICE(data[1], BMI088_GYRO_INT4_LVL, int_config->gyro_int_pin_4_cfg.lvl);
                data[1] = BMI088_SET_BITSLICE(data[1], BMI088_GYRO_INT4_OD, int_config->gyro_int_pin_4_cfg.output_mode);
            }

            /* Both Interrupt pins or channels */
            if (int_config->gyro_int_channel == BMI088_INT_CHANNEL_BOTH)
            {
                /* Update data with user configured bmi088_int_cfg structure */
                data[1] = BMI088_SET_BITSLICE(data[1], BMI088_GYRO_INT3_LVL, int_config->gyro_int_pin_3_cfg.lvl);
                data[1] = BMI088_SET_BITSLICE(data[1], BMI088_GYRO_INT3_OD, int_config->gyro_int_pin_3_cfg.output_mode);
                data[1] = BMI088_SET_BITSLICE(data[1], BMI088_GYRO_INT4_LVL, int_config->gyro_int_pin_4_cfg.lvl);
                data[1] = BMI088_SET_BITSLICE(data[1], BMI088_GYRO_INT4_OD, int_config->gyro_int_pin_4_cfg.output_mode);
            }

            /* Write to Interrupt control register and Interrupt configuration register together */
            rslt |= bmi088_set_gyro_regs(reg_addr, data, BMI088_TWO, dev);

            if (rslt == BMI088_OK)
            {
                /* Read the Interrupt Map register */
                reg_addr = BMI088_GYRO_INT3_INT4_IO_MAP_REG;
                rslt |= bmi088_get_gyro_regs(reg_addr, &data[0], BMI088_ONE, dev);

                /* Interrupt pin or channel 3 */
                if (int_config->gyro_int_channel == BMI088_INT_CHANNEL_3)
                {
                    /* Map Interrupt pin 3 */
                    data[0] = BMI088_SET_BITSLICE(data[0], BMI088_GYRO_INT1_FIFO, BMI088_ONE);
                }

                /* Interrupt pin or channel 4 */
                if (int_config->gyro_int_channel == BMI088_INT_CHANNEL_4)
                {
                    /* Map Interrupt pin 4 */
                    data[0] = BMI088_SET_BITSLICE(data[0], BMI088_GYRO_INT2_FIFO, BMI088_ONE);
                }

                /* Both Interrupt pins or channels */
                if (int_config->gyro_int_channel == BMI088_INT_CHANNEL_BOTH)
                {
                    /* Map both Interrupt pins */
                    data[0] = BMI088_SET_BITSLICE(data[0], BMI088_GYRO_INT1_FIFO, BMI088_ONE);
                    data[0] = BMI088_SET_BITSLICE(data[0], BMI088_GYRO_INT2_FIFO, BMI088_ONE);
                }

                /* Write to Interrupt Map register */
                rslt |= bmi088_set_gyro_regs(reg_addr, &data[0], BMI088_ONE, dev);
            }
        }
    }

    return rslt;

}

/*!
 *  @brief This API enables or disables the FIFO watermark interrupt of the gyro sensor.
 */
uint16_t bmi088_set_gyro_fifo_wm_int(struct bmi088_int_cfg *int_config, struct bmi088_dev *dev, uint8_t fifo_wm_enable)
{
    uint16_t rslt = 0;
    uint8_t data, reg_addr;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    /* Proceed if null check is fine */
    if (rslt == BMI088_OK)
    {
        reg_addr = BMI088_GYRO_INT3_INT4_IO_CONF_REG;
        /* Read interrupt configuration register */
        rslt = bmi088_get_gyro_regs(reg_addr, &data, BMI088_ONE, dev);

        if (rslt == BMI088_OK)
        {
            /* Interrupt pin or channel 3 */
            if (int_config->gyro_int_channel == BMI088_INT_CHANNEL_3)
            {
                /* Update data with user configured bmi088_int_cfg structure */
                data = BMI088_SET_BITSLICE(data, BMI088_GYRO_INT3_LVL, int_config->gyro_int_pin_3_cfg.lvl);
                data = BMI088_SET_BITSLICE(data, BMI088_GYRO_INT3_OD, int_config->gyro_int_pin_3_cfg.output_mode);
            }

            /* Interrupt pin or channel 4 */
            if (int_config->gyro_int_channel == BMI088_INT_CHANNEL_4)
            {
                /* Update data with user configured bmi088_int_cfg structure */
                data = BMI088_SET_BITSLICE(data, BMI088_GYRO_INT4_LVL, int_config->gyro_int_pin_4_cfg.lvl);
                data = BMI088_SET_BITSLICE(data, BMI088_GYRO_INT4_OD, int_config->gyro_int_pin_4_cfg.output_mode);
            }

            /* Both Interrupt pins or channels */
            if (int_config->gyro_int_channel == BMI088_INT_CHANNEL_BOTH)
            {
                /* Update data with user configured bmi088_int_cfg structure */
                data = BMI088_SET_BITSLICE(data, BMI088_GYRO_INT3_LVL, int_config->gyro_int_pin_3_cfg.lvl);
                data = BMI088_SET_BITSLICE(data, BMI088_GYRO_INT3_OD, int_config->gyro_int_pin_3_cfg.output_mode);
                data = BMI088_SET_BITSLICE(data, BMI088_GYRO_INT4_LVL, int_config->gyro_int_pin_4_cfg.lvl);
                data = BMI088_SET_BITSLICE(data, BMI088_GYRO_INT4_OD, int_config->gyro_int_pin_4_cfg.output_mode);
            }

            /* write to interrupt configuration register */
            rslt = bmi088_set_gyro_regs(reg_addr, &data, BMI088_ONE, dev);
        }

    }

    /* Read the Interrupt Map register */
    reg_addr = BMI088_GYRO_INT3_INT4_IO_MAP_REG;
    rslt |= bmi088_get_gyro_regs(reg_addr, &data, BMI088_ONE, dev);

    /* Interrupt pin or channel 3 */
    if (int_config->gyro_int_channel == BMI088_INT_CHANNEL_3)
    {
        /* Map Interrupt pin 3 */
        data = BMI088_SET_BITSLICE(data, BMI088_GYRO_INT1_FIFO, BMI088_ONE);
    }

    /* Interrupt pin or channel 4 */
    if (int_config->gyro_int_channel == BMI088_INT_CHANNEL_4)
    {
        /* Map Interrupt pin 4 */
        data = BMI088_SET_BITSLICE(data, BMI088_GYRO_INT2_FIFO, BMI088_ONE);
    }

    /* Both Interrupt pins or channels */
    if (int_config->gyro_int_channel == BMI088_INT_CHANNEL_BOTH)
    {
        /* Map both Interrupt pins */
        data = BMI088_SET_BITSLICE(data, BMI088_GYRO_INT1_FIFO, BMI088_ONE);
        data = BMI088_SET_BITSLICE(data, BMI088_GYRO_INT2_FIFO, BMI088_ONE);
    }

    /* Write to Interrupt Map register */
    rslt |= bmi088_set_gyro_regs(reg_addr, &data, BMI088_ONE, dev);

    /* Read the Interrupt enable register */
    reg_addr = BMI088_GYRO_INT_EN_REG;
    rslt |= bmi088_get_gyro_regs(reg_addr, &data, BMI088_ONE, dev);

    /* Write to Interrupt enable register */
    data = BMI088_SET_BITSLICE(data, BMI088_GYRO_INT_EN, fifo_wm_enable);
    rslt |= bmi088_set_gyro_regs(reg_addr, &data, BMI088_ONE, dev);

    return rslt;

}

/*****************************************************************************/
/* Static function definition */
/*!
 * @brief This API is used to validate the device structure pointer for
 * null conditions.
 */
static uint16_t null_ptr_check(const struct bmi088_dev *dev)
{
    uint16_t rslt;

    if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_ms == NULL))
    {
        /* Device structure pointer is not valid */
        rslt = BMI088_E_NULL_PTR;
    }
    else
    {
        /* Device structure is fine */
        rslt = BMI088_OK;
    }

    return rslt;
}

/*!
 *  @brief This API is used to reset the FIFO related configurations
 *  in the Accel fifo_frame structure.
 */
static void reset_accel_fifo_data_structure(const struct bmi088_dev *dev)
{
    /* Prepare for next accel FIFO read by resetting accel FIFO's
     internal data structures*/
    dev->accel_fifo->byte_start_idx = 0;
    dev->accel_fifo->sensor_time = 0;
    dev->accel_fifo->skipped_frame_count = 0;
    dev->accel_fifo->dropped_frame_count = 0;

}

/*!
 *  @brief This API computes the number of bytes of accel FIFO data
 *  which is to be parsed in header-less mode.
 */
static void get_accel_len_to_parse(uint16_t *start_idx, uint16_t *len, const uint16_t *accel_count, const struct bmi088_dev *dev)
{
    uint8_t dummy_byte_spi = 0;

    /*Check if this is the first iteration of data unpacking
     if yes, then consider dummy byte on SPI*/
    if (dev->accel_fifo->byte_start_idx == 0)
    {
        dummy_byte_spi = dev->dummy_byte;
    }

    /*Data start index*/
    *start_idx = dev->accel_fifo->byte_start_idx + dummy_byte_spi;

    if (dev->accel_fifo->fifo_data_enable == BMI088_FIFO_A_ENABLE)
    {
        /* Len has the number of bytes to loop for */
        *len = (uint16_t)(((*accel_count) * BMI088_FIFO_A_LENGTH) + dummy_byte_spi);
    }
    else
    {
        /* No sensor is enabled in FIFO,
         so there will be no accel data.
         Update the data index as complete*/
        *start_idx = dev->accel_fifo->length;
    }

    if ((*len) > dev->accel_fifo->length)
    {
        /* Handling the case where more data is requested
         than available */
        *len = dev->accel_fifo->length;
    }
}

/*!
 *  @brief This API checks the accel fifo read data as empty frame, if it
 *  is empty frame then moves the index to last byte.
 */
static void check_empty_accel_fifo(uint16_t *data_index, const struct bmi088_dev *dev)
{
    if ((*data_index + 2) < dev->accel_fifo->length)
    {
        /* Check if FIFO is empty */
        if ((dev->accel_fifo->data[*data_index] == FIFO_MSB_CONFIG_CHECK)
            && (dev->accel_fifo->data[*data_index + BMI088_ONE] == FIFO_LSB_CONFIG_CHECK))
        {
            /* Update the data index as complete*/
            *data_index = dev->accel_fifo->length;
        }
    }
}

/*!
 *  @brief This API is used to parse the accelerometer data from the
 *  FIFO data in header mode.
 */
static void extract_accel_header_mode(struct bmi088_sensor_data *accel_data, uint16_t *accel_length, const struct bmi088_dev *dev)
{
    uint8_t frame_header = 0;
    uint16_t data_index;
    uint16_t accel_index = 0;
    uint16_t frame_to_read = *accel_length;

    /*Check if this is the first iteration of data unpacking
     if yes, then consider dummy byte on SPI*/
    if (dev->accel_fifo->byte_start_idx == 0)
    {
        dev->accel_fifo->byte_start_idx = dev->dummy_byte;
    }

    for (data_index = dev->accel_fifo->byte_start_idx; data_index < dev->accel_fifo->length;)
    {
        /* Header byte is stored in the variable frame_header */
        frame_header = dev->accel_fifo->data[data_index];
        /* Get the frame details from header */
        frame_header = frame_header & BMI088_FIFO_TAG_INTR_MASK;
        /* Index is moved to next byte where the data is starting */
        data_index++;

        switch (frame_header)
        {
            /* Accel frame */
            case FIFO_HEAD_A:
                {
                unpack_accel_frm(accel_data, &data_index, &accel_index, frame_header, dev);
            }
                break;

                /* Sensor time frame */
            case FIFO_HEAD_SENSOR_TIME:
                {
                unpack_sensortime_frame(&data_index, dev);
            }
                break;

                /* Skip frame */
            case FIFO_HEAD_SKIP_FRAME:
                {
                unpack_accel_skipped_frame(&data_index, dev);
            }
                break;

                /* Input config frame */
            case FIFO_HEAD_INPUT_CONFIG:
                {
                move_to_next_accel_frame(&data_index, BMI088_ONE, dev);
            }
                break;

                /* Sample drop frame */
            case FIFO_HEAD_SAMPLE_DROP:
                {
                unpack_accel_dropped_frame(&data_index, dev);
            }
                break;

                /* Over read FIFO data */
            case FIFO_HEAD_OVER_READ_MSB:
                {
                /* Update the data index as complete */
                data_index = dev->accel_fifo->length;
            }
                break;

            default:
                break;
        }

        if (frame_to_read == accel_index)
        {
            /* Number of frames to be read is completed */
            break;
        }
    }

    /* Update number of accel data read */
    *accel_length = accel_index;
    /* Update the accel frame index */
    dev->accel_fifo->byte_start_idx = data_index;
}

/*!
 *  @brief This API is used to parse the accelerometer data from the
 *  FIFO data in both header mode and header-less mode.
 *  It updates the idx value which is used to store the index of
 *  the current data byte which is parsed.
 */
static void unpack_accel_frm(struct bmi088_sensor_data *accel, uint16_t *idx, uint16_t *accel_idx, uint8_t frm, const struct bmi088_dev *dev)
{
    uint8_t is_buffer_end_reached = 0;

    if ((frm == FIFO_HEAD_A) || (frm == BMI088_FIFO_A_ENABLE))
    {
        /* Partial read, then skip the data */
        if ((*idx + BMI088_FIFO_A_LENGTH) > dev->accel_fifo->length)
        {
            /* Update the data index as complete */
            *idx = dev->accel_fifo->length;
            is_buffer_end_reached = 1;
        }

        if (!is_buffer_end_reached)
        {
            /* Unpack the data array into the structure instance "accel" */
            unpack_accel_data(&accel[*accel_idx], *idx, dev);
            /* Move the data index */
            *idx = *idx + BMI088_FIFO_A_LENGTH;
            (*accel_idx)++;
        }
    }

}

/*!
 *  @brief This API is used to parse the accelerometer data from the
 *  FIFO data and store it in the instance of the structure bmi088_sensor_data.
 */
static void unpack_accel_data(struct bmi088_sensor_data *accel_data, uint16_t data_start_index, const struct bmi088_dev *dev)
{
    uint16_t data_lsb;
    uint16_t data_msb;

    /* Accel raw x data */
    data_lsb = dev->accel_fifo->data[data_start_index++];
    data_msb = dev->accel_fifo->data[data_start_index++];
    accel_data->x = (int16_t)((data_msb << BMI088_EIGHT) | data_lsb);

    /* Accel raw y data */
    data_lsb = dev->accel_fifo->data[data_start_index++];
    data_msb = dev->accel_fifo->data[data_start_index++];
    accel_data->y = (int16_t)((data_msb << BMI088_EIGHT) | data_lsb);

    /* Accel raw z data */
    data_lsb = dev->accel_fifo->data[data_start_index++];
    data_msb = dev->accel_fifo->data[data_start_index++];
    accel_data->z = (int16_t)((data_msb << BMI088_EIGHT) | data_lsb);

}

/*!
 *  @brief This API is used to parse and store the sensor time from the
 *  FIFO data in the structure instance dev.
 */
static void unpack_sensortime_frame(uint16_t *data_index, const struct bmi088_dev *dev)
{
    uint32_t sensor_time_byte3 = 0;
    uint16_t sensor_time_byte2 = 0;
    uint8_t sensor_time_byte1 = 0;

    /* Partial read, then move the data index to last data */
    if ((*data_index + BMI088_SENSOR_TIME_LENGTH) > dev->accel_fifo->length)
    {
        /* Update the data index as complete */
        *data_index = dev->accel_fifo->length;
    }
    else
    {
        sensor_time_byte3 = dev->accel_fifo->data[(*data_index) + BMI088_SENSOR_TIME_MSB_BYTE] << BMI088_SIXTEEN;
        sensor_time_byte2 = dev->accel_fifo->data[(*data_index) + BMI088_SENSOR_TIME_XLSB_BYTE] << BMI088_EIGHT;
        sensor_time_byte1 = dev->accel_fifo->data[(*data_index) + BMI088_SENSOR_TIME_LSB_BYTE];
        /* Sensor time */
        dev->accel_fifo->sensor_time = (uint32_t)(sensor_time_byte3 | sensor_time_byte2 | sensor_time_byte1);
        *data_index = (*data_index) + BMI088_SENSOR_TIME_LENGTH;
    }
}

/*!
 *  @brief This API is used to parse and store the skipped_frame_count from
 *  the accelerometer FIFO data in the structure instance dev.
 */
static void unpack_accel_skipped_frame(uint16_t *data_index, const struct bmi088_dev *dev)
{
    /* Partial read, then move the data index to last data */
    if (*data_index >= dev->accel_fifo->length)
    {
        /* Update the data index as complete */
        *data_index = dev->accel_fifo->length;
    }
    else
    {
        dev->accel_fifo->skipped_frame_count = dev->accel_fifo->data[*data_index];
        /* Move the data index */
        *data_index = (*data_index) + 1;
    }
}

/*!
 *  @brief This API is used to parse and store the dropped_frame_count from
 *  the accelerometer FIFO data in the structure instance dev.
 */
static void unpack_accel_dropped_frame(uint16_t *data_index, const struct bmi088_dev *dev)
{
    uint8_t dropped_frame = 0;
    /* Partial read, then move the data index to last data */
    if (*data_index >= dev->accel_fifo->length)
    {
        /*Update the data index as complete*/
        *data_index = dev->accel_fifo->length;
    }
    else
    {
        /* Extract accel dropped frame count */
        dropped_frame = dev->accel_fifo->data[*data_index] & ACCEL_FIFO_DROP;

        /* Move the data index and update the dropped frame count */
        if (dropped_frame == ACCEL_FIFO_DROP)
        {
            *data_index = (*data_index) + BMI088_FIFO_A_LENGTH;
            dev->accel_fifo->dropped_frame_count = dev->accel_fifo->dropped_frame_count + 1;
        }
    }
}

/*!
 *  @brief This API is used to move the data index ahead of the
 *  current_frame_length parameter when unnecessary FIFO data appears while
 *  extracting the accel data.
 */
static void move_to_next_accel_frame(uint16_t *data_index, uint8_t current_frame_length, const struct bmi088_dev *dev)
{
    /* Partial read, then move the data index to last data */
    if ((*data_index + current_frame_length) > dev->accel_fifo->length)
    {
        /* Update the data index as complete */
        *data_index = dev->accel_fifo->length;
    }
    else
    {
        /* Move the data index to next frame */
        *data_index = *data_index + current_frame_length;
    }
}

/*!
 *  @brief This API is used to reset the gyro FIFO related configurations
 *  in the fifo_frame structure.
 */
static void reset_gyro_fifo_data_structure(const struct bmi088_dev *dev)
{
    /* Prepare for next gyro FIFO read by resetting gyro FIFO's
     internal data structures */
    dev->gyro_fifo->byte_start_idx = 0;
    dev->gyro_fifo->sensor_time = 0;
    dev->gyro_fifo->skipped_frame_count = 0;
    dev->gyro_fifo->dropped_frame_count = 0;

}

/*!
 *  @brief This API computes the number of bytes of Gyro FIFO data which is
 *  to be parsed in header-less mode.
 */
static void get_gyro_len_to_parse(uint16_t *start_idx, uint16_t *len, const uint16_t *gyro_count, const struct bmi088_dev *dev)
{
    /* Data start index */
    *start_idx = dev->gyro_fifo->byte_start_idx;

    if (dev->gyro_fifo->fifo_data_enable == BMI088_GYRO_ALL_INT_DATA)
    {
        dev->gyro_fifo->frame_length = BMI088_FIFO_G_ALL_DATA_LENGTH;
    }
    else if (dev->gyro_fifo->fifo_data_enable == BMI088_GYRO_X_DATA)
    {
        dev->gyro_fifo->frame_length = BMI088_FIFO_G_X_LENGTH;
    }
    else if (dev->gyro_fifo->fifo_data_enable == BMI088_GYRO_Y_DATA)
    {
        dev->gyro_fifo->frame_length = BMI088_FIFO_G_Y_LENGTH;
    }
    else if (dev->gyro_fifo->fifo_data_enable == BMI088_GYRO_Z_DATA)
    {
        dev->gyro_fifo->frame_length = BMI088_FIFO_G_Z_LENGTH;
    }
    else
    {
        /* No sensor is enabled in FIFO,
         so there will be no gyro data.
         Update the data index as complete */
        *start_idx = dev->gyro_fifo->length;
        dev->gyro_fifo->frame_length = 0;
    }

    /* Len has the number of bytes to loop for */
    *len = (uint16_t)((*gyro_count) * dev->gyro_fifo->frame_length);

    /* Handling the case where more data is requested than available */
    if ((*len) > dev->gyro_fifo->length)
    {
        /* Len is equal to the FIFO length */
        *len = dev->gyro_fifo->length;
    }
}

/*!
 *  @brief This API is used to parse the gyroscope data from the
 *  FIFO data in header-less mode and update the data_index value
 *  which is used to store the index of the current data byte
 *  which is parsed.
 */
static uint16_t unpack_gyro_frm(struct bmi088_sensor_data *data, uint16_t *idx, uint16_t *gyro_idx, uint8_t frm,
                                const struct bmi088_dev *dev)
{
    uint16_t rslt = 0;
    uint8_t is_buffer_end_reached = 0;

    if ((frm == BMI088_GYRO_ALL_INT_DATA) || (frm == BMI088_GYRO_X_DATA)
        || (frm == BMI088_GYRO_Y_DATA)
        || (frm == BMI088_GYRO_Z_DATA))
    {
        /* Partial read, then skip the data */
        if ((*idx + dev->gyro_fifo->frame_length) > dev->gyro_fifo->length)
        {
            /*update the data index as complete*/
            *idx = dev->gyro_fifo->length;
            is_buffer_end_reached = 1;
        }

        if (!is_buffer_end_reached)
        {
            /* Unpack the data array into Gyro sensor data structure */
            unpack_gyro_data(&data[*gyro_idx], *idx, dev);
            /* move the data index */
            *idx = *idx + dev->gyro_fifo->frame_length;
            (*gyro_idx)++;
        }

    }

    return rslt;
}

/*!
 *  @brief This API is used to parse the gyroscope data from
 *  the FIFO data and store it in the instance of the structure
 *  gyro_data.
 */
static void unpack_gyro_data(struct bmi088_sensor_data *gyro_data, uint16_t start_idx, const struct bmi088_dev *dev)
{
    uint16_t data_lsb;
    uint16_t data_msb;

    /* Gyro raw x data */
    data_lsb = dev->gyro_fifo->data[start_idx++];
    data_msb = dev->gyro_fifo->data[start_idx++];
    gyro_data->x = (int16_t)((data_msb << BMI088_EIGHT) | data_lsb);

    /* Gyro raw y data */
    data_lsb = dev->gyro_fifo->data[start_idx++];
    data_msb = dev->gyro_fifo->data[start_idx++];
    gyro_data->y = (int16_t)((data_msb << BMI088_EIGHT) | data_lsb);

    /* Gyro raw z data */
    data_lsb = dev->gyro_fifo->data[start_idx++];
    data_msb = dev->gyro_fifo->data[start_idx++];
    gyro_data->z = (int16_t)((data_msb << BMI088_EIGHT) | data_lsb);

}

/*!
 *  @brief This API checks the gyro fifo read data as empty frame, if it
 *  is empty frame then moves the index to last byte.
 */
static void check_empty_gyro_fifo(uint16_t *data_index, const struct bmi088_dev *dev)
{
    if ((*data_index + 2) < dev->gyro_fifo->length)
    {
        /* Check if FIFO is empty */
        if ((dev->gyro_fifo->data[*data_index] == FIFO_MSB_CONFIG_CHECK)
            && (dev->gyro_fifo->data[*data_index + BMI088_ONE] == FIFO_LSB_CONFIG_CHECK))
        {
            /* Update the data index as complete */
            *data_index = dev->gyro_fifo->length;
        }
    }
}

#endif
/** @}*/
