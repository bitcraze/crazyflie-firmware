/*
 ****************************************************************************
 * Copyright (C) 2015 - 2016 Bosch Sensortec GmbH
 *
 * File :bmi088_gyro.c
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
/*! \file bmi088_gyro.c
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
 * @brief This API sets the data ready interrupt for gyro sensor.
 *
 * @param[in] int_config  : Structure instance of bmi088_int_cfg.
 * @param[in] dev         : Structure instance of bmi088_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success
 * @retval Any non zero value -> Fail
 */
static uint16_t set_gyro_data_ready_int(const struct bmi088_int_cfg *int_config, struct bmi088_dev *dev);

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

/***************************************************************************/
/**\name        Extern Declarations
 ****************************************************************************/

/***************************************************************************/
/**\name        Globals
 ****************************************************************************/

/* Copy of gyro_cfg structure of device structure. It
 * prevents overwriting same gyro configuration data */
static struct bmi088_cfg gyro_cfg_copy;

/***************************************************************************/
/**\name        Function definitions
 ****************************************************************************/

/*!
 *  @brief This API is the entry point for gyro sensor.
 *  It performs the selection of I2C/SPI read mechanism according to the
 *  selected interface and reads the chip-id of gyro sensor.
 */
uint16_t bmi088_gyro_init(struct bmi088_dev *dev)
{
    uint16_t rslt = BMI088_OK;
    uint8_t reg_addr, data = 0;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    /* Proceed if null check is fine */
    if (rslt == BMI088_OK)
    {
        reg_addr = BMI088_GYRO_CHIP_ID_REG;
        /* Read gyro chip id */
        rslt |= bmi088_get_gyro_regs(reg_addr, &data, BMI088_ONE, dev);

        if (rslt == BMI088_OK)
        {
            /* Assign Chip Id */
            dev->gyro_chip_id = data;

            /* Initializing gyro sensor parameters with default values */
            dev->gyro_cfg.bw = BMI088_GYRO_ODR_RESET_VAL;
            dev->gyro_cfg.power = BMI088_GYRO_PM_NORMAL;
            dev->gyro_cfg.range = BMI088_GYRO_RANGE_2000_DPS;

            /* Copying gyro_cfg parameters of device structure to
             * gyro_cfg_copy structure to maintain a copy. This will
             * help us to prevent writing same configuration data
             * again and again */
            gyro_cfg_copy.bw = dev->gyro_cfg.bw;
            gyro_cfg_copy.power = dev->gyro_cfg.power;
            gyro_cfg_copy.range = dev->gyro_cfg.range;

        }
        else
        {
            rslt = BMI088_E_COM_FAIL;
        }

    }

    return rslt;
}

/*!
 * @brief This API reads the data from the given register address
 * of gyro sensor.
 */
uint16_t bmi088_get_gyro_regs(uint8_t reg_addr, uint8_t *data, uint16_t len, struct bmi088_dev *dev)
{
    uint16_t rslt = BMI088_OK;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    /* Proceed if null check is fine */
    if (rslt == BMI088_OK)
    {
        /* Configuring reg_addr for SPI Interface */
        if (dev->interface == BMI088_SPI_INTF)
        {
            reg_addr = (reg_addr | BMI088_SPI_RD_MASK);
        }

        /* read a gyro register */
        rslt = dev->read(dev->gyro_id, reg_addr, data, len);

        if (rslt != BMI088_OK)
        {
            rslt = BMI088_E_COM_FAIL;
        }
    }

    return rslt;
}

/*!
 * @brief This API writes the given data to the register address
 * of gyro sensor.
 */
uint16_t bmi088_set_gyro_regs(uint8_t reg_addr, uint8_t *data, uint16_t len, struct bmi088_dev *dev)
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

        /* write to a gyro register */
        rslt = dev->write(dev->gyro_id, reg_addr, data, len);

        if (rslt != BMI088_OK)
        {
            rslt = BMI088_E_COM_FAIL;
        }
    }

    return rslt;
}

/*!
 * @brief This API resets the gyro sensor.
 */
uint16_t bmi088_gyro_soft_reset(struct bmi088_dev *dev)
{
    uint16_t rslt = BMI088_OK;
    uint8_t data, reg_addr;
    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    /* Proceed if null check is fine */
    if (rslt == BMI088_OK)
    {
        /* Reset gyro device */
        reg_addr = BMI088_GYRO_SOFTRESET_REG;
        data = BMI088_SOFT_RESET_VAL;
        rslt = bmi088_set_gyro_regs(reg_addr, &data, BMI088_ONE, dev);

        if (rslt == BMI088_OK)
        {
            /* delay 30 ms after writing reset value to its register */
            dev->delay_ms(BMI088_GYRO_SOFTRESET_DELAY);
        }
    }

    return rslt;
}

/*!
 * @brief This API sets the power mode, range and bandwidth
 * of gyro sensor.
 */
uint16_t bmi088_set_gyro_meas_conf(struct bmi088_dev *dev)
{
    uint16_t rslt = BMI088_OK;
    uint8_t reg_addr;
    bool is_range_invalid = false, is_odr_invalid = false;
    uint8_t odr, range;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    /* Proceed if null check is fine */
    if (rslt == BMI088_OK)
    {
        odr = dev->gyro_cfg.odr;
        range = dev->gyro_cfg.range;

        /* Check if range is not previously configured range */
        if (range != gyro_cfg_copy.range)
        {
            /* Check for valid range */
            if (range <= BMI088_GYRO_RANGE_125_DPS)
            {
                reg_addr = BMI088_GYRO_RANGE_REG;
                /* Write range value to range register */
                rslt = bmi088_set_gyro_regs(reg_addr, &range, BMI088_ONE, dev);

                /* If rslt is ok, copy the current range to previous range to maintain a copy */
                if (rslt == BMI088_OK)
                {
                    gyro_cfg_copy.range = range;
                }
            }
            else
            {
                /* Set range as invalid */
                is_range_invalid = true;
            }
        }

        /* Check if odr is not previously configured odr */
        if (odr != gyro_cfg_copy.odr)
        {
            /* Check for valid odr */
            if (odr <= BMI088_GYRO_BW_32_ODR_100_HZ)
            {
                reg_addr = BMI088_GYRO_BANDWIDTH_REG;
                /* Write odr value to odr register */
                rslt = bmi088_set_gyro_regs(reg_addr, &odr, BMI088_ONE, dev);

                /* If rslt is ok, copy the current odr to previous odr to maintain a copy */
                if (rslt == BMI088_OK)
                {
                    gyro_cfg_copy.odr = odr;
                }
            }
            else
            {
                /* Set odr as invalid */
                is_odr_invalid = true;
            }
        }
    }

    /* If invalid conditions take place, make rslt as invalid */
    if ((is_range_invalid) || (is_odr_invalid))
    {
        rslt = BMI088_E_INVALID_INPUT;
    }

    return rslt;
}

/*!
 * @brief This API sets the power mode of the gyro sensor.
 */
uint16_t bmi088_set_gyro_power_mode(struct bmi088_dev *dev)
{
    uint16_t rslt = BMI088_OK;
    uint8_t reg_addr, power;
    bool is_power_switching_mode_valid = true;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    /* Proceed if null check is fine */
    if (rslt == BMI088_OK)
    {
        power = dev->gyro_cfg.power;

        /* Check if power is not previously configured power */
        if (power != gyro_cfg_copy.power)
        {
            /* Check for invalid power switching (i.e) deep suspend to suspend */
            if ((power == BMI088_GYRO_PM_SUSPEND) && (gyro_cfg_copy.power == BMI088_GYRO_PM_DEEP_SUSPEND))
            {
                is_power_switching_mode_valid = false;
            }

            /* Check for invalid power switching (i.e) from suspend to deep suspend */
            if ((power == BMI088_GYRO_PM_DEEP_SUSPEND) && (gyro_cfg_copy.power == BMI088_GYRO_PM_SUSPEND))
            {
                is_power_switching_mode_valid = false;
            }

            /* Check if power switching mode is valid*/
            if (is_power_switching_mode_valid)
            {
                reg_addr = BMI088_GYRO_LPM1_REG;
                /* Write power to power register */
                rslt = bmi088_set_gyro_regs(reg_addr, &power, BMI088_ONE, dev);

                /* If rslt is fine, copy current power to previous power to maintain a copy */
                if (rslt == BMI088_OK)
                {
                    gyro_cfg_copy.power = power;
                }
            }
        }

    }

    return rslt;
}

/*!
 * @brief This API reads the gyro data from the sensor,
 * store it in the bmi088_sensor_data structure instance
 * passed by the user.
 */
uint16_t bmi088_get_gyro_data(struct bmi088_sensor_data *gyro, struct bmi088_dev *dev)
{
    uint16_t rslt = BMI088_OK;
    uint8_t index = 0, reg_addr, data[6];
    uint32_t lsb, msb, msblsb;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    /* Proceed if null check is fine */
    if ((rslt == BMI088_OK) && (gyro != NULL))
    {
        reg_addr = BMI088_GYRO_X_LSB_REG;
        /* read gyro sensor data */
        rslt = bmi088_get_gyro_regs(reg_addr, data, BMI088_SIX, dev);

        if (rslt == BMI088_OK)
        {
            lsb = (uint32_t)data[index++];
            msb = (uint32_t)data[index++];
            msblsb = (msb << BMI088_EIGHT) | lsb;
            gyro->x = (int16_t)msblsb; /* Data in X axis */

            lsb = (uint32_t)data[index++];
            msb = (uint32_t)data[index++];
            msblsb = (msb << BMI088_EIGHT) | lsb;
            gyro->y = (int16_t)msblsb; /* Data in Y axis */

            lsb = (uint32_t)data[index++];
            msb = (uint32_t)data[index++];
            msblsb = (msb << BMI088_EIGHT) | lsb;
            gyro->z = (int16_t)msblsb; /* Data in Z axis */
        }
    }

    return rslt;
}

/*!
 * @brief This API configures the necessary gyro interrupt
 * based on the user settings in the bmi088_int_cfg
 * structure instance.
 */
uint16_t bmi088_set_gyro_int_config(const struct bmi088_int_cfg *int_config, struct bmi088_dev *dev)
{
    uint16_t rslt = BMI088_OK;

    switch (int_config->gyro_int_type)
    {
        case BMI088_GYRO_DATA_RDY_INT:
            {
            /* Data ready interrupt */
            rslt = set_gyro_data_ready_int(int_config, dev);
        }
            break;
        default:
            break;
    }

    return rslt;

}

/*!
 *  @brief This API enables or disables the Gyro Self test feature in the
 *  sensor.
 */
uint16_t bmi088_set_gyro_selftest(uint8_t selftest, struct bmi088_dev *dev)
{
    uint16_t rslt = BMI088_OK;
    uint8_t reg_addr, data = 0;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    /* Proceed if null check is fine */
    if (rslt == BMI088_OK)
    {
        /* Check for valid selftest input */
        if ((selftest == BMI088_ENABLE) || (selftest == BMI088_DISABLE))
        {
            reg_addr = BMI088_GYRO_SELF_TEST_REG;
            /* Read self test register */
            rslt = bmi088_get_gyro_regs(reg_addr, &data, BMI088_ONE, dev);

            if (rslt == BMI088_OK)
            {
                /* Enable self-test */
                data = BMI088_SET_BITSLICE(data, BMI088_GYRO_SELF_TEST_EN, selftest);
                /* write self test input value to self-test register */
                rslt = bmi088_set_gyro_regs(reg_addr, &data, BMI088_ONE, dev);
            }
        }
        else
        {
            rslt = BMI088_E_INVALID_INPUT;
        }
    }

    return rslt;
}

/*!
 *  @brief This API checks whether the self test functionality of the
 *  gyro sensor is working or not.
 */
uint16_t bmi088_perform_gyro_selftest(int8_t *result, struct bmi088_dev *dev)
{
    uint16_t rslt = BMI088_OK;
    uint8_t reg_addr, data = 0, loop_break = 1;

    *result = BMI088_SELFTEST_FAIL;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    /* Proceed if null check is fine */
    if (rslt == BMI088_OK)
    {
        /* Enable the gyro self-test */
        rslt = bmi088_set_gyro_selftest(BMI088_ENABLE, dev);

        if (rslt == BMI088_OK)
        {
            /* Loop till self-test ready bit is set */
            while (loop_break)
            {
                reg_addr = BMI088_GYRO_SELF_TEST_REG;
                /* Read self-test register to check if self-test ready bit is set */
                rslt = bmi088_get_gyro_regs(reg_addr, &data, BMI088_ONE, dev);

                if (rslt == BMI088_OK)
                {
                    data = BMI088_GET_BITSLICE(data, BMI088_GYRO_SELF_TEST_RDY);

                    /* If self-test ready bit is set, exit the loop */
                    if (data)
                    {
                        loop_break = 0;
                    }
                }
                else
                {
                    /* Exit the loop in case of communication failure */
                    loop_break = 0;
                }
            }

            if (rslt == BMI088_OK)
            {
                /* Read self-test register to check for self-test Ok bit */
                rslt = bmi088_get_gyro_regs(reg_addr, &data, BMI088_ONE, dev);

                if (rslt == BMI088_OK)
                {
                    data = BMI088_GET_BITSLICE(data, BMI088_GYRO_SELF_TEST_RESULT);

                    /* Update the self-test result based on self-test Ok bit */
                    if (!data)
                    {
                        *result = BMI088_SELFTEST_PASS;
                    }
                }
            }
        }

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
 * @brief This API sets the data ready interrupt for gyro sensor.
 */
static uint16_t set_gyro_data_ready_int(const struct bmi088_int_cfg *int_config, struct bmi088_dev *dev)
{
    uint16_t rslt = BMI088_OK;
    uint8_t reg_addr, data;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    /* Proceed if null check is fine */
    if ((rslt == BMI088_OK) && (int_config != NULL))
    {
        reg_addr = BMI088_GYRO_INT_CTRL_REG;
        /* Data to enable new data ready interrupt */
        data = BMI088_GYRO_DRDY_INT_ENABLE_VAL;

        /* write data to interrupt control register */
        rslt = bmi088_set_gyro_regs(reg_addr, &data, BMI088_ONE, dev);

        if (rslt == BMI088_OK)
        {
            reg_addr = BMI088_GYRO_INT3_INT4_IO_MAP_REG;

            /* update data to map data ready interrupt to appropriate pins */
            if (int_config->gyro_int_channel == BMI088_INT_CHANNEL_3)
            {
                data = BMI088_GYRO_MAP_DRDY_TO_INT3;
            }

            if (int_config->gyro_int_channel == BMI088_INT_CHANNEL_4)
            {
                data = BMI088_GYRO_MAP_DRDY_TO_INT4;
            }

            if (int_config->gyro_int_channel == BMI088_INT_CHANNEL_BOTH)
            {
                data = BMI088_GYRO_MAP_DRDY_TO_BOTH_INT3_INT4;
            }

            /* write data to interrupt map register */
            rslt = bmi088_set_gyro_regs(reg_addr, &data, BMI088_ONE, dev);
            /* Configure interrupt pin */
            rslt |= set_int_pin_config(int_config, dev);

        }
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

    return rslt;

}

/** @}*/

