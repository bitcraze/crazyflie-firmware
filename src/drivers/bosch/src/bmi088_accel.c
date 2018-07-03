/*
 ****************************************************************************
 * Copyright (C) 2015 - 2016 Bosch Sensortec GmbH
 *
 * File :bmi088_accel.c
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
/*! \file bmi088_accel.c
 \brief Sensor Driver for BMI088 family of sensors */
/***************************************************************************/
/**\name        Header files
 ****************************************************************************/
#include "bmi088.h"

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
 */
static uint16_t null_ptr_check(struct bmi088_dev *dev);

/*!
 * @brief This API configures the pins which fire the
 * interrupt signal when any interrupt occurs.
 *
 * @param[in] int_config  : Structure instance of bmi088_int_cfg.
 * @param[in] dev         : Structure instance of bmi088_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success
 * @retval Any non zero value -> Fail
 */
static uint16_t set_int_pin_config(const struct bmi088_int_cfg *int_config, struct bmi088_dev *dev);

/*!
 * @brief This API sets the data ready interrupt for accel sensor
 *
 * @param[in] int_config  : Structure instance of bmi088_int_cfg.
 * @param[in] dev         : Structure instance of bmi088_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success
 * @retval Any non zero value -> Fail
 */
static uint16_t set_accel_data_ready_int(const struct bmi088_int_cfg *int_config, struct bmi088_dev *dev);

/*!
 * @brief This API writes the config stream data in memory using burst mode
 *
 * @param[in] stream_data : Pointer to store data of 32 bytes
 * @param[in] index       : Represents value in multiple of 32 bytes
 * @param[in] dev         : Structure instance of bmi088_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval Any non zero value -> Fail
 */
static uint16_t stream_transfer_write(const uint8_t *stream_data, uint16_t index, struct bmi088_dev *dev);

/*!
 * @brief This function enables and configures the Accel which is needed
 * for Self test operation.
 *
 * @param[in] dev : Structure instance of bmi088_dev
 *
 * @return results of self test
 * @retval zero -> Success
 * @retval Any non zero value -> Fail
 */
static uint16_t set_accel_selftest_config(struct bmi088_dev *dev);

/*!
 * @brief This function validates the Accel Self test data and decides the
 * result of Self test operation.
 *
 * @param[in] accel_data_diff : Pointer to structure variable which holds
 * the Accel data difference of Self test operation
 *
 * @return results of self test operation
 * @retval zero -> Success
 * @retval Any non zero value -> Fail
 */
static uint16_t validate_selftest(const struct bmi088_sensor_data *accel_data_diff);

/*!
 * @brief This API converts lsb value of axes to mg for self-test
 *
 * @param[in] accel_data_diff     : Pointer variable used to pass accel difference
 * values in g
 * @param[out] accel_data_diff_mg : Pointer variable used to store accel
 * difference values in mg
 *
 * @return None
 */
static void convert_lsb_g(const struct bmi088_sensor_data *accel_data_diff,
                          struct bmi088_sensor_data *accel_data_diff_mg);

/*!
 * @brief This API is used to calculate the power of given
 * base value.
 *
 * @param[in] base       : value of base
 * @param[in] resolution : resolution of the sensor
 *
 * @return : returns the value of base^resolution
 */
static int32_t power(int16_t base, uint8_t resolution);

/***************************************************************************/
/**\name        Extern Declarations
 ****************************************************************************/

/***************************************************************************/
/**\name        Globals
 ****************************************************************************/

/* Copy of accel_cfg structure of device structure.It
 * prevents overwriting same accel configuration data */
static struct bmi088_cfg accel_cfg_copy;

/***************************************************************************/
/**\name        Function definitions
 ****************************************************************************/
/*!
 *  @brief This API is the entry point for accel sensor.
 *  It performs the selection of I2C/SPI read mechanism according to the
 *  selected interface and reads the chip-id of accel sensor.
 */
uint16_t bmi088_accel_init(struct bmi088_dev *dev)
{
    uint16_t rslt = BMI088_OK;
    uint8_t data = 0, reg_addr;
    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    /* Proceed if null check is fine */
    if (rslt == BMI088_OK)
    {
        if (dev->interface == BMI088_SPI_INTF)
        {
            /* Set dummy byte in case of SPI interface*/
            dev->dummy_byte = 1;
        }
        else
        {
            /* Make dummy byte 0 in case of I2C interface*/
            dev->dummy_byte = 0;
        }

        reg_addr = BMI088_ACCEL_CHIP_ID_REG;
        rslt |= bmi088_get_accel_regs(reg_addr, &data, BMI088_ONE, dev);

        if (rslt == BMI088_OK)
        {
            /* Assign Chip Id */
            dev->accel_chip_id = data;

            /* Initializing accel sensor parameters with default values */
            dev->accel_cfg.bw = BMI088_ACCEL_ODR_100_HZ;
            dev->accel_cfg.power = BMI088_ACCEL_PM_SUSPEND;
            dev->accel_cfg.range = BMI088_ACCEL_RANGE_6G;

            /* Copying accel_cfg parameters of device structure to
             * accel_cfg_copy structure to maintain a copy */
            accel_cfg_copy.bw = dev->accel_cfg.bw;
            accel_cfg_copy.power = dev->accel_cfg.power;
            accel_cfg_copy.range = dev->accel_cfg.range;

        }
        else
        {
            rslt = BMI088_E_COM_FAIL;
        }

    }

    return rslt;
}

/*!
 *  @brief This API is used to write the binary configuration in the sensor.
 */
uint16_t bmi088_write_config_file(struct bmi088_dev *dev)
{
    uint16_t rslt = 0;
    /* Disable advance power save*/
    uint8_t adv_power_save = 0, reg_addr;
    /* Config loading disable*/
    uint8_t config_load = BMI088_DISABLE;
    uint16_t index = 0;
    uint8_t config_stream_status = 0;

    reg_addr = BMI088_ACCEL_PWR_CONF_REG;
    /* Disable advanced power save*/
    rslt |= bmi088_set_accel_regs(reg_addr, &adv_power_save, BMI088_ONE, dev);

    /* Wait for sensor time synchronization. Refer the data-sheet for
     more information*/
    dev->delay_ms(BMI088_ONE);

    if (rslt == BMI088_OK)
    {
        reg_addr = BMI088_ACCEL_INIT_CTRL_REG;
        /* Disable config loading*/
        rslt |= bmi088_set_accel_regs(reg_addr, &config_load, BMI088_ONE, dev);

        /* Write the config stream */
        for (index = 0; index < BMI088_CONFIG_STREAM_SIZE; index += dev->read_write_len)
        {
            rslt |= stream_transfer_write((dev->config_file_ptr + index), index, dev);
        }

        /* Enable config loading and FIFO mode */
        config_load = BMI088_ENABLE;

        rslt |= bmi088_set_accel_regs(reg_addr, &config_load, BMI088_ONE, dev);

        /* Wait till ASIC is initialized. Refer the data-sheet for
         more information*/
        dev->delay_ms(BMI088_ONE_FIFTY);

        reg_addr = BMI088_ACCEL_INTERNAL_STAT_REG;
        /* Read the status of config stream operation */
        rslt |= bmi088_get_accel_regs(reg_addr, &config_stream_status, BMI088_ONE, dev);

        if (config_stream_status != BMI088_ASIC_INITIALIZED)
        {
            rslt |= BMI088_E_CONFIG_STREAM_ERROR;
        }
    }

    return rslt;

}

/*!
 *  @brief This API reads the data from the given register address of accel sensor.
 */
uint16_t bmi088_get_accel_regs(uint8_t reg_addr, uint8_t *data, uint16_t len,
                               struct bmi088_dev *dev)
{
    /* variable used to return the status of communication result*/
    uint16_t rslt = BMI088_OK;
    uint16_t temp_len = len + dev->dummy_byte;
    uint16_t i;
    uint8_t temp_buff[temp_len];

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    /* Proceed if null check is fine */
    if (rslt == BMI088_OK)
    {
        /* Configuring reg_addr for SPI Interface */
        if (dev->interface == BMI088_SPI_INTF)
        {
            reg_addr = reg_addr | BMI088_SPI_RD_MASK;
        }
        /* Read the data from the register */
        rslt |= dev->read(dev->accel_id, reg_addr, temp_buff, temp_len);

        for (i = 0; i < len; i++)
        {
            data[i] = temp_buff[i + dev->dummy_byte];
        }
    }

    return rslt;

}

/*!
 *  @brief This API writes the given data to the register address
 *  of accel sensor.
 */
uint16_t bmi088_set_accel_regs(uint8_t reg_addr, uint8_t *data, uint16_t len,
                               struct bmi088_dev *dev)
{
    uint16_t rslt = BMI088_OK;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    /* Proceed if null check is fine */
    if ((rslt == BMI088_OK) && (data != NULL) && (len != 0))
    {
        /* Configuring reg_addr for SPI Interface */
        if (dev->interface == BMI088_SPI_INTF)
        {
            reg_addr = (reg_addr & BMI088_SPI_WR_MASK);
        }

        /* write to an accel register */
        rslt = dev->write(dev->accel_id, reg_addr, data, len);

        if (rslt != BMI088_OK)
        {
            rslt = BMI088_E_COM_FAIL;
        }
    }

    return rslt;
}

/*!
 *  @brief This API reads the error status from the accel sensor.
 */
uint16_t bmi088_get_accel_error_status(struct bmi088_err_reg *err_reg, struct bmi088_dev *dev)
{
    uint16_t rslt = 0;
    uint8_t data = 0, reg_addr;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    /* Proceed if null check is fine */
    if (rslt == BMI088_OK)
    {
        reg_addr = BMI088_ACCEL_ERR_REG;
        /* Read the error codes */
        rslt = bmi088_get_accel_regs(reg_addr, &data, BMI088_ONE, dev);

        if (rslt == BMI088_OK)
        {
            /* Fatal error */
            err_reg->fatal_err = BMI088_GET_BITSLICE(data, BMI088_FATAL_ERR);
            /* Cmd error */
            err_reg->cmd_err = BMI088_GET_BITSLICE(data, BMI088_CMD_ERR);
            /* User error */
            err_reg->err_code = BMI088_GET_BITSLICE(data, BMI088_ERR_CODE);
        }
    }

    return rslt;
}

/*!
 *  @brief This API reads the status of the accel sensor.
 */
uint16_t bmi088_get_accel_status(uint8_t *status, struct bmi088_dev *dev)
{
    uint16_t rslt = 0;
    uint8_t data = 0, reg_addr;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    /* Proceed if null check is fine */
    if (rslt == BMI088_OK)
    {
        reg_addr = BMI088_ACCEL_STATUS_REG;
        /* Read the status */
        rslt = bmi088_get_accel_regs(reg_addr, &data, BMI088_ONE, dev);

        if (rslt == BMI088_OK)
        {
            *status = data;
        }
    }

    return rslt;

}

/*!
 *  @brief This API resets the accel sensor.
 */
uint16_t bmi088_accel_soft_reset(struct bmi088_dev *dev)
{
    uint16_t rslt = BMI088_OK;
    uint8_t reg_addr, data;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    /* Proceed if null check is fine */
    if (rslt == BMI088_OK)
    {
        reg_addr = BMI088_ACCEL_SOFTRESET_REG;
        data = BMI088_SOFT_RESET_VAL;
        /* Reset accel device */
        rslt = bmi088_set_accel_regs(reg_addr, &data, BMI088_ONE, dev);
        if (rslt == BMI088_OK)
        {
            /* Delay 1 ms after reset value is written to its register */
            dev->delay_ms(BMI088_ACCEL_SOFTRESET_DELAY);
        }
    }

    return rslt;
}

/*!
 * @brief This API sets the output data rate, range and bandwidth
 * of accel sensor.
 */
uint16_t bmi088_set_accel_meas_conf(struct bmi088_dev *dev)
{
    uint16_t rslt = BMI088_OK;
    uint8_t reg_addr, data;
    uint8_t bw, range, odr;
    bool is_odr_invalid = false, is_bw_invalid = false, is_range_invalid = false;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    /* Proceed if null check is fine */
    if (rslt == BMI088_OK)
    {
        odr = dev->accel_cfg.odr;
        bw = dev->accel_cfg.bw;
        range = dev->accel_cfg.range;

        /* Check if odr and bandwidth are not previously configured odr and bandwidth */
        if ((odr != accel_cfg_copy.odr) || (bw != accel_cfg_copy.bw))
        {
            /* Check for valid odr */
            if ((odr < BMI088_ACCEL_ODR_12_5_HZ) || (odr > BMI088_ACCEL_ODR_1600_HZ))
            {
                is_odr_invalid = true;
                /* Since odr and bandwidth are written to the same
                 * register, use previous odr in case of invalid ODR.
                 * This will be helpful if valid bandwidth arrives */
                odr = accel_cfg_copy.odr;
            }

            /* Check for valid bandwidth */
            if (bw > BMI088_ACCEL_BW_NORMAL)
            {
                is_bw_invalid = true;
                /* Since bandwidth and odr are written to the same
                 * register, use previous bandwidth in case of
                 * invalid bandwidth.This will be helpful if valid
                 * odr arrives */
                bw = accel_cfg_copy.bw;
            }

            /* If either odr or bw is valid, write it to accel config. registers */
            if ((!is_odr_invalid) || (!is_bw_invalid))
            {
                reg_addr = BMI088_ACCEL_CONF_REG;
                /* Read accel config. register */
                rslt = bmi088_get_accel_regs(reg_addr, &data, BMI088_ONE, dev);
                if (rslt == BMI088_OK)
                {
                    /* Update data with new odr and bw values */
                    data = BMI088_SET_BITSLICE(data, BMI088_ACCEL_ODR, odr);
                    data = BMI088_SET_BITSLICE(data, BMI088_ACCEL_BW, bw);
                    /* write to accel config. register */
                    rslt = bmi088_set_accel_regs(reg_addr, &data, BMI088_ONE, dev);

                    /* If rslt is ok, copy the current odr
                     * and bw to accel_cfg_copy structure to
                     * maintain a copy */
                    if (rslt == BMI088_OK)
                    {
                        accel_cfg_copy.odr = odr;
                        accel_cfg_copy.bw = bw;
                    }
                }
            }
        }

        /* Check if range is not previously configured range */
        if (range != accel_cfg_copy.range)
        {
            /* Check if range is valid */
            if (range <= BMI088_ACCEL_RANGE_24G)
            {
                reg_addr = BMI088_ACCEL_RANGE_REG;
                /* Read range register */
                rslt = bmi088_get_accel_regs(reg_addr, &data, BMI088_ONE, dev);
                if (rslt == BMI088_OK)
                {
                    /* Update data with current range values */
                    data = BMI088_SET_BITSLICE(data, BMI088_ACCEL_RANGE, range);
                    /* write to range register */
                    rslt = bmi088_set_accel_regs(reg_addr, &data, BMI088_ONE, dev);

                    /* If rslt is ok, copy the current range
                     * to accel_cfg_copy structure to
                     * maintain a copy */
                    if (rslt == BMI088_OK)
                    {
                        accel_cfg_copy.range = range;
                    }
                }
            }
            else
            {
                /* Range is invalid */
                is_range_invalid = true;
            }
        }
    }

    /* If invalid odr or bw or range arrive, make rslt invalid input */
    if (is_odr_invalid || is_bw_invalid || is_range_invalid)
    {
        rslt = BMI088_E_INVALID_INPUT;
    }

    return rslt;
}

/*!
 * @brief This API sets the power mode of the accel sensor.
 */
uint16_t bmi088_set_accel_power_mode(struct bmi088_dev *dev)
{
    uint16_t rslt = BMI088_OK;
    uint8_t reg_addr, power;
    uint8_t data[2];

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    /* Proceed if null check is fine */
    if (rslt == BMI088_OK)
    {
        power = dev->accel_cfg.power;
        /* Check if current power is not previously configured power */
        if (power != accel_cfg_copy.power)
        {
            /* Configure data array to write to accel power configuration register */
            if (power == BMI088_ACCEL_PM_ACTIVE)
            {
                data[0] = BMI088_ACCEL_PM_ACTIVE;
                data[1] = BMI088_ACCEL_POWER_ENABLE;
            }
            else if (power == BMI088_ACCEL_PM_SUSPEND)
            {
                data[0] = BMI088_ACCEL_PM_SUSPEND;
                data[1] = BMI088_ACCEL_POWER_DISABLE;
            }
            else
            {
                /* Invalid power input */
                rslt = BMI088_E_INVALID_INPUT;
            }

            if (rslt == BMI088_OK)
            {
                reg_addr = BMI088_ACCEL_PWR_CONF_REG;
                /* write to accel power configuration register */
                rslt = bmi088_set_accel_regs(reg_addr, data, BMI088_TWO, dev);

                /* If rslt is ok, copy the current power
                 * to accel_cfg_copy structure to maintain
                 * a copy */
                if (rslt == BMI088_OK)
                {
                    accel_cfg_copy.power = power;
                }
            }

        }
    }

    return rslt;
}

/*!
 * @brief This API reads the accel data from the sensor,
 * store it in the bmi088_sensor_data structure instance
 * passed by the user.
 */
uint16_t bmi088_get_accel_data(struct bmi088_sensor_data *accel,
                               struct bmi088_dev *dev)
{
    uint16_t rslt = BMI088_OK;
    uint8_t index = 0, reg_addr, data[6];
    uint32_t lsb, msb, msblsb;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    /* Proceed if null check is fine */
    if ((rslt == BMI088_OK) && (accel != NULL))
    {
        /* Read accel sensor data */
        reg_addr = BMI088_ACCEL_X_LSB_REG;
        rslt = bmi088_get_accel_regs(reg_addr, data, BMI088_SIX, dev);

        if (rslt == BMI088_OK)
        {
            lsb = (uint32_t)data[index++];
            msb = (uint32_t)data[index++];
            msblsb = (msb << BMI088_EIGHT) | lsb;
            accel->x = ((int16_t)msblsb); /* Data in X axis */

            lsb = (uint32_t)data[index++];
            msb = (uint32_t)data[index++];
            msblsb = (msb << BMI088_EIGHT) | lsb;
            accel->y = ((int16_t)msblsb); /* Data in Y axis */

            lsb = (uint32_t)data[index++];
            msb = (uint32_t)data[index++];
            msblsb = (msb << BMI088_EIGHT) | lsb;
            accel->z = ((int16_t)msblsb); /* Data in Z axis */
        }
    }

    return rslt;
}

/*!
 * @brief This API configures the necessary accel interrupt
 * based on the user settings in the bmi088_int_cfg
 * structure instance.
 */
uint16_t bmi088_set_accel_int_config(const struct bmi088_int_cfg *int_config, struct bmi088_dev *dev)
{
    uint16_t rslt = BMI088_OK;

    switch (int_config->accel_int_type)
    {
        case BMI088_ACCEL_DATA_RDY_INT:
            {
            /* Data ready interrupt */
            rslt = set_accel_data_ready_int(int_config, dev);
        }
            break;
        default:
            break;
    }

    return rslt;
}

/*!
 * @brief This API switches accel sensor on or off.
 */
uint16_t bmi088_accel_switch_control(struct bmi088_dev *dev, uint8_t switch_input)
{
    uint16_t rslt = BMI088_OK;
    uint8_t reg_addr, data = switch_input;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    /* Proceed if null check is fine */
    if (rslt == BMI088_OK)
    {
        /* Check if switch input is valid */
        if ((data == BMI088_ACCEL_POWER_DISABLE) || (data == BMI088_ACCEL_POWER_ENABLE))
        {
            reg_addr = BMI088_ACCEL_PWR_CTRL_REG;
            /* write to accel power control register  */
            rslt = bmi088_set_accel_regs(reg_addr, &data, BMI088_ONE, dev);
        }
        else
        {
            rslt = BMI088_E_INVALID_INPUT;
        }
    }

    return rslt;
}

/*!
 * @brief This API reads the temperature of the sensor in ° Celcius.
 */
uint16_t bmi088_get_sensor_temperature(struct bmi088_dev *dev, float *sensor_temp)
{
    uint16_t rslt = BMI088_OK;
    uint8_t reg_addr, data[2] = { 0 };
    uint16_t msb, lsb;
    int16_t msblsb;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    /* Proceed if null check is fine */
    if (rslt == BMI088_OK)
    {
        reg_addr = BMI088_TEMP_MSB_REG;
        /* Read sensor temperature */
        rslt = bmi088_get_accel_regs(reg_addr, data, BMI088_TWO, dev);

        if (rslt == BMI088_OK)
        {
            msb = (data[0] << BMI088_THREE); /* MSB data */
            lsb = (data[1] >> BMI088_FIVE); /* LSB data */
            msblsb = (int16_t)(msb + lsb);

            if (msblsb > 1023)
            {
                msblsb = msblsb - 2048;
            }
            /* sensor temperature */
            *sensor_temp = (msblsb * 0.125) + 23;
        }
    }

    return rslt;

}

/*!
 *  @brief This API reads the sensor time of the sensor.
 */
uint16_t bmi088_get_sensor_time(struct bmi088_dev *dev, uint32_t *sensor_time)
{
    uint16_t rslt = BMI088_OK;
    uint8_t reg_addr, data[3] = { 0 };
    uint32_t byte2, byte1, byte0;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    /* Proceed if null check is fine */
    if (rslt == BMI088_OK)
    {
        reg_addr = BMI088_ACCEL_SENSORTIME_0_REG;
        /* Read 3-byte sensor time */
        rslt = bmi088_get_accel_regs(reg_addr, data, BMI088_THREE, dev);
        if (rslt == BMI088_OK)
        {
            byte0 = data[0]; 						/* Lower byte */
            byte1 = (data[1] << BMI088_EIGHT); 		/* Middle byte */
            byte2 = (data[2] << BMI088_SIXTEEN); 	/* Higher byte */

            /* Sensor time */
            *sensor_time = (byte2 | byte1 | byte0);
        }
    }

    return rslt;
}

/*!
 *  @brief This API checks whether the self test functionality of the sensor
 *  is working or not.
 */
uint16_t bmi088_perform_accel_selftest(int8_t *result, struct bmi088_dev *dev)
{
    uint16_t rslt = BMI088_OK;
    struct bmi088_sensor_data positive = { 0 };
    struct bmi088_sensor_data negative = { 0 };
    /*! Structure for difference of accel values in g*/
    struct bmi088_sensor_data accel_data_diff = { 0 };
    /*! Structure for difference of accel values in mg*/
    struct bmi088_sensor_data accel_data_diff_mg = { 0 };

    *result = BMI088_SELFTEST_FAIL;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    /* Proceed if null check is fine */
    if (rslt == BMI088_OK)
    {
        rslt |= set_accel_selftest_config(dev);
        dev->delay_ms(20);
        rslt |= bmi088_set_accel_selftest(BMI088_ACCEL_POSITIVE_SELF_TEST, dev);

        if (rslt == BMI088_OK)
        {
            dev->delay_ms(BMI088_HUNDRED);
            rslt |= bmi088_get_accel_data(&positive, dev);
            rslt |= bmi088_set_accel_selftest(BMI088_ACCEL_NEGATIVE_SELF_TEST, dev);

            if (rslt == BMI088_OK)
            {
                dev->delay_ms(100);
                rslt |= bmi088_get_accel_data(&negative, dev);
                rslt |= bmi088_set_accel_selftest(BMI088_ACCEL_SWITCH_OFF_SELF_TEST, dev);

                accel_data_diff.x = ABS(positive.x) + ABS(negative.x);
                accel_data_diff.y = ABS(positive.y) + ABS(negative.y);
                accel_data_diff.z = ABS(positive.z) + ABS(negative.z);

                /*! Converting LSB of the differences of
                 accel values to mg */
                convert_lsb_g(&accel_data_diff, &accel_data_diff_mg);
                /*! Validating self test for
                 accel values in mg */
                rslt |= validate_selftest(&accel_data_diff_mg);

                if (rslt == BMI088_OK)
                {
                    *result = BMI088_SELFTEST_PASS;
                    dev->delay_ms(BMI088_HUNDRED);
                }

                /* Triggers a soft reset */
                rslt |= bmi088_accel_soft_reset(dev);
            }
        }
    }

    return rslt;
}

/*!
 *  @brief This API enables or disables the Accel Self test feature in the
 *  sensor.
 */
uint16_t bmi088_set_accel_selftest(uint8_t selftest, struct bmi088_dev *dev)
{
    uint16_t rslt = BMI088_OK;
    uint8_t reg_addr, data = 0;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    /* Proceed if null check is fine */
    if (rslt == BMI088_OK)
    {
        reg_addr = BMI088_ACCEL_SELF_TEST_REG;
        data = selftest;
        /* Write to accel selftest register */
        rslt = bmi088_set_accel_regs(reg_addr, &data, BMI088_ONE, dev);
    }

    return rslt;
}

/*****************************************************************************/
/* Static function definition */
/*!
 * @brief This API is used to validate the device structure pointer for
 * null conditions.
 */
static uint16_t null_ptr_check(struct bmi088_dev *dev)
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
 * @brief This API configures the pins which fire the
 * interrupt signal when any interrupt occurs.
 */
static uint16_t set_int_pin_config(const struct bmi088_int_cfg *int_config, struct bmi088_dev *dev)
{
    uint16_t rslt = BMI088_OK;
    uint8_t reg_addr, data;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    /* Proceed if null check is fine */
    if ((rslt == BMI088_OK) && (int_config != NULL))
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
        }
    }

    return rslt;

}

/*!
 * @brief This API sets the data ready interrupt for accel sensor.
 */
static uint16_t set_accel_data_ready_int(const struct bmi088_int_cfg *int_config, struct bmi088_dev *dev)
{
    uint16_t rslt = BMI088_OK;
    uint8_t reg_addr, data;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    /* Proceed if null check is fine */
    if ((rslt == BMI088_OK) && (int_config != NULL))
    {
        reg_addr = BMI088_ACCEL_INT1_INT2_MAP_DATA_REG;
        /* Read interrupt map register */
        rslt = bmi088_get_accel_regs(reg_addr, &data, BMI088_ONE, dev);

        if (rslt == BMI088_OK)
        {
            /* Update data to map data ready interrupt to appropriate interrupt pins */
            if (int_config->accel_int_channel == BMI088_INT_CHANNEL_1)
            {
                data = BMI088_SET_BITSLICE(data, BMI088_ACCEL_INT1_DRDY, BMI088_ENABLE);
            }

            if (int_config->accel_int_channel == BMI088_INT_CHANNEL_2)
            {
                data = BMI088_SET_BITSLICE(data, BMI088_ACCEL_INT2_DRDY, BMI088_ENABLE);
            }

            /* Write to interrupt map register */
            rslt = bmi088_set_accel_regs(reg_addr, &data, BMI088_ONE, dev);
            /* Configure interrupt pins */
            rslt |= set_int_pin_config(int_config, dev);
        }
    }

    return rslt;

}

/*!
 *  @brief This API writes the config stream data in memory using burst mode.
 */
static uint16_t stream_transfer_write(const uint8_t *stream_data, uint16_t index, struct bmi088_dev *dev)
{
    uint16_t rslt = 0;
    uint8_t asic_msb = (uint8_t)((index / 2) >> 4);
    uint8_t asic_lsb = ((index / 2) & 0x0F);
    uint8_t reg_addr;

    /* Write to feature config register */
    reg_addr = BMI088_ACCEL_RESERVED_5B_REG;
    rslt |= bmi088_set_accel_regs(reg_addr, &asic_lsb, BMI088_ONE, dev);

    if (rslt == BMI088_OK)
    {
        /* Write to feature config register */
        reg_addr = BMI088_ACCEL_RESERVED_5C_REG;
        rslt |= bmi088_set_accel_regs(reg_addr, &asic_msb, BMI088_ONE, dev);

        if (rslt == BMI088_OK)
        {
            /* Write to feature config registers */
            reg_addr = BMI088_ACCEL_FEATURE_CFG_REG;
            rslt |= bmi088_set_accel_regs(reg_addr, (uint8_t *)stream_data, dev->read_write_len, dev);
        }
    }

    return rslt;

}

/*!
 *  @brief This function enables and configures the Accel which is needed
 *  for Self test operation.
 */
static uint16_t set_accel_selftest_config(struct bmi088_dev *dev)
{
    uint16_t rslt = BMI088_OK;

    /* Configuring sensors to perform accel self test */
    dev->accel_cfg.odr = BMI088_ACCEL_ODR_1600_HZ;
    dev->accel_cfg.bw = BMI088_ACCEL_BW_NORMAL;
    dev->accel_cfg.range = BMI088_ACCEL_RANGE_24G;

    /* Enable Accel sensor */
    rslt = bmi088_accel_switch_control(dev, BMI088_ACCEL_POWER_ENABLE);
    /* Configure sensors with above configured settings */
    rslt |= bmi088_set_accel_meas_conf(dev);

    return rslt;
}

/*!
 *  @brief This function validates the Accel Self test data and decides the
 *  result of Self test operation.
 */
static uint16_t validate_selftest(const struct bmi088_sensor_data *accel_data_diff)
{
    uint16_t rslt;

    /* Validating accel data by comparing with minimum value of the axes in mg */
    /* x axis limit 1000mg, y axis limit 1000mg and z axis limit 500mg */
    if (accel_data_diff->x >= 1000 && accel_data_diff->y >= 1000 && accel_data_diff->z >= 500)
    {
        rslt = BMI088_OK;
    }
    else
    {
        rslt = BMI088_E_SELF_TEST_FAIL;
    }

    return rslt;
}

/*!
 *  @brief This API converts lsb value of axes to mg for self-test.
 */
static void convert_lsb_g(const struct bmi088_sensor_data *accel_data_diff,
                          struct bmi088_sensor_data *accel_data_diff_mg)
{
    uint32_t lsb_per_g;
    /* Range considered for self-test is 24g */
    uint8_t range = 16;

    /* lsb_per_g for the 16-bit resolution and 24g range*/
    lsb_per_g = (uint32_t)(power(2, BMI088_16_BIT_RESOLUTION) / (2 * range));
    /* accel x value in mg */
    accel_data_diff_mg->x = (accel_data_diff->x / (int32_t)lsb_per_g) * 1000;
    /* accel y value in mg */
    accel_data_diff_mg->y = (accel_data_diff->y / (int32_t)lsb_per_g) * 1000;
    /* accel z value in mg */
    accel_data_diff_mg->z = (accel_data_diff->z / (int32_t)lsb_per_g) * 1000;
}

/*!
 * @brief This API is used to calculate the power of given
 * base value.
 */
static int32_t power(int16_t base, uint8_t resolution)
{
    uint8_t i = 1;
    /* Initialize variable to store the power of 2 value */
    int32_t value = 1;

    for (; i <= resolution; i++)
    {
        value = (int32_t)(value * base);
    }

    return value;
}

/** @}*/

