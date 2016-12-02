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
 * @file    bmi160.c
 * @date	Dec 1, 2016
 * @version 3.0.0
 * @brief
 *
 */

/*!
 * @defgroup bmi160
 * @brief
 * @{*/

#include "bmi160.h"

static struct bmi160_cfg accel_cfg_copy;
static struct bmi160_cfg gyro_cfg_copy;

/*********************** Local function prototypes ************************/

/*!
 * @brief This API configures the pins to fire the
 * interrupt signal when it occurs.
 *
 * @param[in] int_config  : Structure instance of bmi160_intr_sett.
 * @param[in] dev         : Structure instance of bmi160_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static int8_t set_intr_pin_config(struct bmi160_intr_sett *int_config, struct bmi160_dev *dev);

/*!
 * @brief This API sets the any motion/slope interrupt of the sensor.
 * This interrupt occurs when accel values exceeds preset threshold
 * for a certain period of time.
 *
 * @param[in] int_config  : Structure instance of bmi160_intr_sett.
 * @param[in] dev         : Structure instance of bmi160_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static int8_t set_accel_slope_int(struct bmi160_intr_sett *int_config, struct bmi160_dev *dev);

/*!
 * @brief This API sets tap interrupts.Interrupt is fired when
 * tap movements happen.
 *
 * @param[in] int_config  : Structure instance of bmi160_intr_sett.
 * @param[in] dev         : Structure instance of bmi160_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static int8_t set_accel_tap_int(struct bmi160_intr_sett *int_config, struct bmi160_dev *dev);

/*!
 * @brief This API sets the data ready interrupt for both accel and gyro.
 * This interrupt occurs when new accel and gyro data come.
 *
 * @param[in] int_config  : Structure instance of bmi160_intr_sett.
 * @param[in] dev         : Structure instance of bmi160_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static int8_t set_accel_gyro_data_ready_int(struct bmi160_intr_sett *int_config, struct bmi160_dev *dev);

/*!
 * @brief This API sets the significant motion interrupt of the sensor.This
 * interrupt occurs when there is change in user location.
 *
 * @param[in] int_config  : Structure instance of bmi160_intr_sett.
 * @param[in] dev         : Structure instance of bmi160_dev.

 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static int8_t set_accel_sig_motion_int(struct bmi160_intr_sett *int_config, struct bmi160_dev *dev);

/*!
 * @brief This API sets the no motion/slow motion interrupt of the sensor.
 * Slow motion is similar to any motion interrupt.No motion interrupt
 * occurs when slope bet. two accel values falls below preset threshold
 * for preset duration.
 *
 * @param[in] int_config  : Structure instance of bmi160_intr_sett.
 * @param[in] dev         : Structure instance of bmi160_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static int8_t set_accel_no_motion_int(struct bmi160_intr_sett *int_config, struct bmi160_dev *dev);

/*!
 * @brief This API sets the step detection interrupt.This interrupt
 * occurs when the single step causes accel values to go above
 * preset threshold.
 *
 * @param[in] int_config  : Structure instance of bmi160_intr_sett.
 * @param[in] dev         : Structure instance of bmi160_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static int8_t set_accel_step_detect_int(struct bmi160_intr_sett *int_config, struct bmi160_dev *dev);

/*!
 * @brief This API sets the orientation interrupt of the sensor.This
 * interrupt occurs when there is orientation change in the sensor
 * with respect to gravitational field vector g.
 *
 * @param[in] int_config  : Structure instance of bmi160_intr_sett.
 * @param[in] dev         : Structure instance of bmi160_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static int8_t set_accel_orientation_int(struct bmi160_intr_sett *int_config, struct bmi160_dev *dev);

/*!
 * @brief This API sets the flat interrupt of the sensor.This interrupt
 * occurs in case of flat orientation
 *
 * @param[in] int_config  : Structure instance of bmi160_intr_sett.
 * @param[in] dev         : Structure instance of bmi160_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static int8_t set_accel_flat_detect_int(struct bmi160_intr_sett *int_config, struct bmi160_dev *dev);

/*!
 * @brief This API sets the low-g interrupt of the sensor.This interrupt
 * occurs during free-fall.
 *
 * @param[in] int_config  : Structure instance of bmi160_intr_sett.
 * @param[in] dev         : Structure instance of bmi160_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static int8_t set_accel_low_g_int(struct bmi160_intr_sett *int_config, struct bmi160_dev *dev);

/*!
 * @brief This API sets the high-g interrupt of the sensor.The interrupt
 * occurs if the absolute value of acceleration data of any enabled axis
 * exceeds the programmed threshold and the sign of the value does not
 * change for a preset duration.
 *
 * @param[in] int_config  : Structure instance of bmi160_intr_sett.
 * @param[in] dev         : Structure instance of bmi160_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static int8_t set_accel_high_g_int(struct bmi160_intr_sett *int_config, struct bmi160_dev *dev);


/*********************** User function definitions ************************/

/*!
 * @brief This API reads the data from the given register address
 * of sensor.
 */
int8_t bmi160_get_regs(uint8_t reg_addr, uint8_t *data, uint16_t len, struct bmi160_dev *dev)
{
    int8_t rslt = BMI160_OK;
    /* Null-pointer check */
    if ((dev == NULL) || (dev->read == NULL))
        return BMI160_E_NULL_PTR;

    /* Configuring reg_addr for SPI Interface */
    if (dev->interface == BMI160_SPI_INTF)
        reg_addr = (reg_addr | BMI160_SPI_RD_MASK);

    rslt = dev->read(dev->id, reg_addr, data, len);
    if (rslt != BMI160_OK)
        rslt = BMI160_E_COM_FAIL;

    return rslt;
}

/*!
 * @brief This API writes the given data to the register address
 * of sensor.
 */
int8_t bmi160_set_regs(uint8_t reg_addr, uint8_t *data, uint16_t len, struct bmi160_dev *dev)
{
    int8_t rslt = BMI160_OK;
    /* Null-pointer check */
    if ((dev == NULL) || (dev->write == NULL))
        return BMI160_E_NULL_PTR;

    /* Configuring reg_addr for SPI Interface */
    if (dev->interface == BMI160_SPI_INTF)
        reg_addr = (reg_addr & BMI160_SPI_WR_MASK);

    rslt = dev->write(dev->id, reg_addr, data, len);
    if (rslt != BMI160_OK)
        rslt = BMI160_E_COM_FAIL;

    return rslt;
}


/*!
 *  @brief This API is the entry point for sensor.It performs
 *  the selection of I2C/SPI read mechanism according to the
 *  selected interface and reads the chip-id of bmi160 sensor.
 */
int8_t bmi160_init(struct bmi160_dev *dev)
{
    int8_t rslt = BMI160_OK;
    uint8_t data;

    /* Null-pointer check */
    if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_ms == NULL))
        return BMI160_E_NULL_PTR;

    /* reading 0x7F register to enable SPI Interface if SPI is used */
    if (dev->interface == BMI160_SPI_INTF)
        rslt = bmi160_get_regs(BMI160_SPI_COMM_TEST_ADDR, &data, 1, dev);

    rslt = bmi160_get_regs(BMI160_CHIP_ID_ADDR, &dev->chip_id, 1, dev);//read chip ID
    if (rslt != BMI160_OK)
        return rslt;

    if ((rslt == BMI160_OK) && (dev->chip_id == BMI160_CHIP_ID)) {
        rslt = bmi160_soft_reset(dev);//Soft-reset
        if (rslt != BMI160_OK)
            return rslt;
    }
    else {
        return BMI160_E_DEV_NOT_FOUND;
    }

    /* Allocating memory for prev_accel_cfg_ptr.This is to hold the copy of accel_cfg parameters */
    dev->prev_accel_cfg_ptr = &accel_cfg_copy;
    if (dev->prev_accel_cfg_ptr == NULL)
        return BMI160_E_NULL_PTR;

    /* Allocating memory for prev_gyro_cfg_ptr.This is to hold the copy of gyro_cfg parameters */
    dev->prev_gyro_cfg_ptr = &gyro_cfg_copy;
    if (dev->prev_gyro_cfg_ptr == NULL)
        return BMI160_E_NULL_PTR;


    /* Initializing accel and gyro params with default values */
    dev->accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;
    dev->accel_cfg.odr = BMI160_ACCEL_ODR_100HZ;
    dev->accel_cfg.power = BMI160_ACCEL_SUSPEND_MODE;
    dev->accel_cfg.range = BMI160_ACCEL_RANGE_2G;
    dev->gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;
    dev->gyro_cfg.odr = BMI160_GYRO_ODR_100HZ;
    dev->gyro_cfg.power = BMI160_GYRO_SUSPEND_MODE;
    dev->gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;

    /* Copying accel_cfg structure to similar structure to maintain a copy */
    memcpy(dev->prev_accel_cfg_ptr, &dev->accel_cfg, sizeof(struct bmi160_cfg));
    /* Copying gyro_cfg structure to similar structure to maintain a copy */
    memcpy(dev->prev_gyro_cfg_ptr, &dev->gyro_cfg, sizeof(struct bmi160_cfg));

    return rslt;
}

/*!
 * @brief This API resets and restarts the device.All register
 * values are overwritten with default parameters.
 */
int8_t bmi160_soft_reset(struct bmi160_dev *dev)
{
    int8_t rslt = BMI160_OK;
    uint8_t data = BMI160_SOFT_RESET_CMD;

    /* Null-pointer check */
    if ((dev == NULL) || (dev->delay_ms == NULL))
        return BMI160_E_NULL_PTR;

    rslt = bmi160_set_regs(BMI160_COMMAND_REG_ADDR, &data, 1, dev); /* Reset the device */
    dev->delay_ms(BMI160_SOFT_RESET_DELAY_MS);
    if (rslt != BMI160_OK)
        return rslt;

    /* reading 0x7F register to enable SPI Interface if SPI is used */
    if (dev->interface == BMI160_SPI_INTF)
        rslt = bmi160_get_regs(BMI160_SPI_COMM_TEST_ADDR, &data, 1, dev);

    return rslt;
}


/*!
 * @brief This API configures the power mode, range and bandwidth
 * of sensor.
 */
int8_t bmi160_set_sens_conf(struct bmi160_dev *dev)
{
    int8_t rslt = BMI160_OK;
    uint8_t reg_addr, data, temp;
    uint8_t odr, bw, range;
    struct bmi160_cfg *prev_accel_cfg, *prev_gyro_cfg;

    /* Null-pointer check */
    if ((dev == NULL) || (dev->delay_ms == NULL) || (dev->prev_accel_cfg_ptr == NULL) || (dev->prev_gyro_cfg_ptr == NULL))
        return BMI160_E_NULL_PTR;

    /* type-casting void pointer to appropriate local structure pointer */
    prev_accel_cfg = (struct bmi160_cfg *)dev->prev_accel_cfg_ptr;
    prev_gyro_cfg = (struct bmi160_cfg *)dev->prev_gyro_cfg_ptr;

    if (((dev->accel_cfg.odr <= BMI160_ACCEL_ODR_MAX) && (dev->accel_cfg.odr != prev_accel_cfg->odr))
        || ((dev->accel_cfg.bw <= BMI160_ACCEL_BW_MAX) && (dev->accel_cfg.bw != prev_accel_cfg->bw))) {

        /* write accel Output data rate and bandwidth */
        reg_addr = BMI160_ACCEL_CONFIG_ADDR;
        rslt = bmi160_get_regs(reg_addr, &data, 1, dev);
        if (rslt != BMI160_OK)
            return rslt;
        if ((dev->accel_cfg.odr <= BMI160_ACCEL_ODR_MAX) && (dev->accel_cfg.odr != prev_accel_cfg->odr)) {
            odr = (uint8_t)dev->accel_cfg.odr;
            temp = data & ~BMI160_ACCEL_ODR_MASK;
            data = temp | (odr & BMI160_ACCEL_ODR_MASK);//Adding output data rate
            prev_accel_cfg->odr = dev->accel_cfg.odr;
        }
        if ((dev->accel_cfg.bw <= BMI160_ACCEL_BW_MAX) && (dev->accel_cfg.bw != prev_accel_cfg->bw)) {
            bw = (uint8_t)dev->accel_cfg.bw;
            temp = data & ~BMI160_ACCEL_BW_MASK;
            data = temp | ((bw << 4) & BMI160_ACCEL_ODR_MASK);//Adding bandwidth
            prev_accel_cfg->bw = dev->accel_cfg.bw;
        }
        rslt = bmi160_set_regs(reg_addr, &data, 1, dev);/* Write output data rate and bandwidth */
        if (rslt != BMI160_OK)
            return rslt;
        dev->delay_ms(BMI160_ONE_MS_DELAY);
    }

    if ((dev->accel_cfg.range <= BMI160_ACCEL_RANGE_MAX) && (dev->accel_cfg.range != prev_accel_cfg->range)) {
        /* write accel range */
        reg_addr = BMI160_ACCEL_RANGE_ADDR;
        rslt = bmi160_get_regs(reg_addr, &data, 1, dev);
        if (rslt != BMI160_OK)
            return rslt;

        range = (uint8_t)dev->accel_cfg.range;
        temp = data & ~BMI160_ACCEL_RANGE_MASK;
        data = temp | (range & BMI160_ACCEL_RANGE_MASK);//Adding range
        rslt = bmi160_set_regs(reg_addr, &data, 1, dev);/* Write range */
        if (rslt != BMI160_OK)
            return rslt;
        prev_accel_cfg->range = dev->accel_cfg.range;
    }

    if (((dev->gyro_cfg.odr <= BMI160_GYRO_ODR_MAX) && (dev->gyro_cfg.odr != prev_gyro_cfg->odr))
        || ((dev->gyro_cfg.bw <= BMI160_GYRO_BW_MAX) && (dev->gyro_cfg.bw != prev_gyro_cfg->bw))) {
        /* write gyro Output data rate and bandwidth */
        reg_addr = BMI160_GYRO_CONFIG_ADDR;
        rslt = bmi160_get_regs(reg_addr, &data, 1, dev);
        if (rslt != BMI160_OK)
            return rslt;
        if ((dev->gyro_cfg.odr <= BMI160_GYRO_ODR_MAX) && (dev->gyro_cfg.odr != prev_gyro_cfg->odr)) {
            odr = (uint8_t)dev->gyro_cfg.odr;
            temp = (data & ~BMI160_GYRO_ODR_MASK);
            data = temp | (odr & BMI160_GYRO_ODR_MASK);//Adding output data rate
            prev_gyro_cfg->odr = dev->gyro_cfg.odr;
        }
        if (dev->gyro_cfg.bw < BMI160_GYRO_BW_MAX) {
            bw = (uint8_t)dev->gyro_cfg.bw;
            temp = data & ~BMI160_GYRO_BW_MASK;
            data = temp | ((bw << 4) & BMI160_GYRO_BW_MASK);//Adding bandwidth
            prev_gyro_cfg->bw = dev->gyro_cfg.bw;
        }
        rslt = bmi160_set_regs(reg_addr, &data, 1, dev);/* Write output data rate and bandwidth */
        if (rslt != BMI160_OK)
            return rslt;
        dev->delay_ms(BMI160_ONE_MS_DELAY);
    }

    if ((dev->gyro_cfg.range <= BMI160_GYRO_RANGE_MAX) && (dev->gyro_cfg.range != prev_gyro_cfg->range)) {
        /* write the gyro range value */
        reg_addr = BMI160_GYRO_RANGE_ADDR;
        rslt = bmi160_get_regs(reg_addr, &data, 1, dev);
        if (rslt != BMI160_OK)
            return rslt;

        range = (uint8_t)dev->gyro_cfg.range;
        temp = data & ~BMI160_GYRO_RANGE_MSK;
        data = temp | (range & BMI160_GYRO_RANGE_MSK);//Adding range
        rslt = bmi160_set_regs(reg_addr, &data, 1, dev);/* Write range */
        if (rslt != BMI160_OK)
            return rslt;
        prev_gyro_cfg->range = dev->gyro_cfg.range;
    }

    /* write power mode for accel and gyro */
    rslt = bmi160_set_power_mode(dev);
    return rslt;
}

/*!
 * @brief This API sets the power mode of the sensor.
 */
int8_t bmi160_set_power_mode(struct bmi160_dev *dev)
{
    int8_t rslt = BMI160_OK;
    uint8_t reg_addr, temp, data;
    struct bmi160_cfg *prev_accel_cfg, *prev_gyro_cfg;

    /* Null-pointer check */
    if ((dev == NULL) || (dev->delay_ms == NULL) || (dev->prev_accel_cfg_ptr == NULL) || (dev->prev_gyro_cfg_ptr == NULL))
       return BMI160_E_NULL_PTR;

    /* type-casting void pointer to appropriate local structure pointer */
    prev_accel_cfg = (struct bmi160_cfg *)dev->prev_accel_cfg_ptr;
    prev_gyro_cfg = (struct bmi160_cfg *)dev->prev_gyro_cfg_ptr;

    if ((dev->accel_cfg.power != prev_accel_cfg->power) &&
        ((dev->accel_cfg.power >= BMI160_ACCEL_SUSPEND_MODE) && (dev->accel_cfg.power <= BMI160_ACCEL_LOWPOWER_MODE))) {

        reg_addr = BMI160_ACCEL_CONFIG_ADDR;
        rslt = bmi160_get_regs(reg_addr, &data, 1, dev);

        if (dev->accel_cfg.power == BMI160_ACCEL_LOWPOWER_MODE) {
            temp = data & ~BMI160_ACCEL_UNDERSAMPLING_MASK;
            data = temp | ((1 << 7) & BMI160_ACCEL_UNDERSAMPLING_MASK);//Set under-sampling parameter
            rslt = bmi160_set_regs(reg_addr, &data, 1, dev);/* Write data */
            if (rslt != BMI160_OK)
                return rslt;
        }
        else {
            if (data & BMI160_ACCEL_UNDERSAMPLING_MASK) {
                temp = data & ~BMI160_ACCEL_UNDERSAMPLING_MASK;
                data = temp | ((0 << 7) & BMI160_ACCEL_UNDERSAMPLING_MASK);//disable under-sampling parameter if already enabled
                rslt = bmi160_set_regs(reg_addr, &data, 1, dev);/* Write data */
                if (rslt != BMI160_OK)
                    return rslt;
            }
        }

        reg_addr = BMI160_COMMAND_REG_ADDR;
        rslt = bmi160_set_regs(reg_addr, &dev->accel_cfg.power, 1, dev);/* Write accel power */
        if (prev_accel_cfg->power == BMI160_ACCEL_SUSPEND_MODE)
            dev->delay_ms(BMI160_ACCEL_DELAY_MS);//Original delay is 4.1 ms
        prev_accel_cfg->power = dev->accel_cfg.power;
    }

    if (dev->gyro_cfg.power != prev_gyro_cfg->power)
    {
        switch(dev->gyro_cfg.power)
        {
           case BMI160_GYRO_SUSPEND_MODE:
           case BMI160_GYRO_NORMAL_MODE:
           case BMI160_GYRO_FASTSTARTUP_MODE:
               break;
           default:
               return BMI160_E_INVALID_INPUT;
        }
        reg_addr = BMI160_COMMAND_REG_ADDR;
        rslt = bmi160_set_regs(reg_addr, &dev->gyro_cfg.power, 1, dev);/* Write gyro power */
        if (prev_gyro_cfg->power == BMI160_GYRO_SUSPEND_MODE) {
            dev->delay_ms(BMI160_GYRO_DELAY_MS);//Original delay is 80.3 ms
        }
        else if ((prev_gyro_cfg->power == BMI160_GYRO_FASTSTARTUP_MODE)
            && (dev->accel_cfg.power == BMI160_GYRO_NORMAL_MODE)) {
            dev->delay_ms(10);/* This delay is required for transition from fast-startup mode to normal mode */
        }
        else {
            /* do nothing */
        }
        prev_gyro_cfg->power = dev->gyro_cfg.power;
    }

    return rslt;

}

/*!
 * @brief This API reads sensor data, stores it in
 * the bmi160_sensor_data structure pointer passed by the user.
 */
int8_t bmi160_get_sensor_data(enum bmi160_select_sensor sensor, struct bmi160_sensor_data *accel, struct bmi160_sensor_data *gyro, struct bmi160_dev *dev)
{
    int8_t rslt = BMI160_OK;
    uint8_t index = 0, data_array[12];
    uint32_t lsb, msb, msblsb;

    /* Null-pointer check */
    if (dev != NULL) {

       switch (sensor) {
            case BMI160_ACCEL_ONLY:
            {
                /* Null-pointer check */
                if (accel == NULL)
                    return BMI160_E_NULL_PTR;

                //read accel sensor data only
                rslt = bmi160_get_regs(BMI160_ACCEL_DATA_ADDR, data_array, 6, dev);
                if (rslt != BMI160_OK)
                   return rslt;

                /* Accel Data */
                lsb = (uint32_t)data_array[index++];
                msb = (uint32_t)data_array[index++];
                msblsb = (msb << 8) | lsb;
                accel->x = (int16_t)msblsb; /* Data in X axis */

                lsb = (uint32_t)data_array[index++];
                msb = (uint32_t)data_array[index++];
                msblsb = (msb << 8) | lsb;
                accel->y = (int16_t)msblsb; /* Data in Y axis */

                lsb = (uint32_t)data_array[index++];
                msb = (uint32_t)data_array[index++];
                msblsb = (msb << 8) | lsb;
                accel->z = (int16_t)msblsb; /* Data in Z axis */
            }
            break;
            case BMI160_GYRO_ONLY:
            {
                /* Null-pointer check */
                if (gyro == NULL)
                    return BMI160_E_NULL_PTR;

                rslt = bmi160_get_regs(BMI160_GYRO_DATA_ADDR, data_array, 6, dev); //read gyro sensor data only
                if (rslt != BMI160_OK)
                   return rslt;

                /* Gyro Data */
                lsb = (uint32_t)data_array[index++];
                msb = (uint32_t)data_array[index++];
                msblsb = (msb << 8) | lsb;
                gyro->x = (int16_t)msblsb; /* Data in X axis */

                lsb = (uint32_t)data_array[index++];
                msb = (uint32_t)data_array[index++];
                msblsb = (msb << 8) | lsb;
                gyro->y = (int16_t)msblsb; /* Data in Y axis */

                lsb = (uint32_t)data_array[index++];
                msb = (uint32_t)data_array[index++];
                msblsb = (msb << 8) | lsb;
                gyro->z = (int16_t)msblsb; /* Data in Z axis */
            }
            break;
            case BMI160_BOTH_ACCEL_AND_GYRO:
            {
                /* Null-pointer check */
                if ((gyro == NULL) || (accel == NULL))
                    return BMI160_E_NULL_PTR;

                /* read both accel and gyro sensor data */
                rslt = bmi160_get_regs(BMI160_GYRO_DATA_ADDR, data_array, 12, dev);
                if (rslt != BMI160_OK)
                  return rslt;

                /* Gyro Data */
                lsb = (uint32_t)data_array[index++];
                msb = (uint32_t)data_array[index++];
                msblsb = (msb << 8) | lsb;
                gyro->x = (int16_t)msblsb; /* gyro X axis data */

                lsb = (uint32_t)data_array[index++];
                msb = (uint32_t)data_array[index++];
                msblsb = (msb << 8) | lsb;
                gyro->y = (int16_t)msblsb; /* gyro Y axis data */

                lsb = (uint32_t)data_array[index++];
                msb = (uint32_t)data_array[index++];
                msblsb = (msb << 8) | lsb;
                gyro->z = (int16_t)msblsb; /* gyro Z axis data */

                /* Accel Data */
                lsb = (uint32_t)data_array[index++];
                msb = (uint32_t)data_array[index++];
                msblsb = (msb << 8) | lsb;
                accel->x = (int16_t)msblsb; /* accel X axis data */

                lsb = (uint32_t)data_array[index++];
                msb = (uint32_t)data_array[index++];
                msblsb = (msb << 8) | lsb;
                accel->y = (int16_t)msblsb; /* accel Y axis data */

                lsb = (uint32_t)data_array[index++];
                msb = (uint32_t)data_array[index++];
                msblsb = (msb << 8) | lsb;
                accel->z = (int16_t)msblsb; /* accel Z axis data */
            }
            break;
            default:
                break;
       }

    }
    else {
        rslt = BMI160_E_NULL_PTR;
    }

    return rslt;

}

/*!
 * @brief This API configures the necessary interrupt based on
 *  the user settings in the bmi160_intr_sett structure instance.
 */
int8_t bmi160_set_intr_config(struct bmi160_intr_sett *int_config, struct bmi160_dev *dev)
{
    int8_t rslt = BMI160_OK;

    switch (int_config->int_type)
    {
        case BMI160_ACC_SLOPE_INT:
            rslt = set_accel_slope_int(int_config, dev); /*Any motion or slope interrupt*/
            break;
        case BMI160_ACC_SIG_MOTION_INT:
            rslt = set_accel_sig_motion_int(int_config, dev); /* Significant motion interrupt */
            break;
        case BMI160_ACC_SLOW_NO_MOTION_INT:
            rslt = set_accel_no_motion_int(int_config, dev); /* Slow or no motion interrupt */
            break;
        case BMI160_ACC_DOUBLE_TAP_INT:
            case BMI160_ACC_SINGLE_TAP_INT:
                rslt = set_accel_tap_int(int_config, dev); /* Double tap and single tap Interrupt */
            break;
        case BMI160_STEP_DETECT_INT:
                rslt = set_accel_step_detect_int(int_config, dev); /* Step detector interrupt */
             break;
        case BMI160_ACC_ORIENT_INT:
            rslt = set_accel_orientation_int(int_config, dev); /* Orientation interrupt */
            break;
        case BMI160_ACC_FLAT_INT:
            rslt = set_accel_flat_detect_int(int_config, dev); /* Flat detection interrupt */
            break;
        case BMI160_ACC_LOW_G_INT:
            rslt = set_accel_low_g_int(int_config, dev); /* Low-g interrupt */
            break;
        case BMI160_ACC_HIGH_G_INT:
            rslt = set_accel_high_g_int(int_config, dev); /* High-g interrupt */
            break;
        case BMI160_ACC_GYRO_DATA_RDY_INT:
            rslt = set_accel_gyro_data_ready_int(int_config, dev); /* Data ready interrupt */
            break;
         default:
            break;
    }
    return rslt;
}

/*********************** Local function definitions ***********************/

/*!
 * @brief This API sets the any motion/slope interrupt of the sensor.
 * This interrupt occurs when accel values exceeds preset threshold
 * for a certain period of time.
 */
static int8_t set_accel_slope_int(struct bmi160_intr_sett *int_config, struct bmi160_dev *dev)
{
    int8_t rslt = BMI160_OK;
    uint8_t reg_addr, data, temp, dur, data_array[2], count = 0;

    /* Null-pointer check */
    if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_ms == NULL) || ((int_config == NULL)))
        return BMI160_E_NULL_PTR;

    /* updating the interrupt structure to local structure */
    struct bmi160_acc_slop_int_cfg *slope_int_cfg = &(int_config->int_type_cfg.acc_slope_int);

    /* Enable any motion x,any motion y,any motion z in Int Enable 0 register */
    reg_addr = BMI160_INT_ENABLE_0_ADDR;
    rslt = bmi160_get_regs(reg_addr, &data, 1, dev);
    if (rslt != BMI160_OK)
        return rslt;

    if (slope_int_cfg->slope_x == 1) {
        temp = data & ~BMI160_ANY_MOTION_X_INT_EN_MASK;
        data = temp | (1 & BMI160_ANY_MOTION_X_INT_EN_MASK);// Adding Any_motion x axis
    }
    if (slope_int_cfg->slope_y == 1) {
        temp = data & ~BMI160_ANY_MOTION_Y_INT_EN_MASK;
        data = temp | ((1 << 1) & BMI160_ANY_MOTION_Y_INT_EN_MASK);// Adding Any_motion y axis
    }
    if (slope_int_cfg->slope_z == 1) {
        temp = data & ~BMI160_ANY_MOTION_Z_INT_EN_MASK;
        data = temp | ((1 << 2) & BMI160_ANY_MOTION_Z_INT_EN_MASK);// Adding Any_motion z axis
    }
    rslt = bmi160_set_regs(reg_addr, &data, 1, dev);/* write data to Int Enable 0 register */
    if (rslt != BMI160_OK)
        return rslt;

    /* Configure Interrupt pins */
    rslt = set_intr_pin_config(int_config, dev);
    if (rslt != BMI160_OK)
        return rslt;

    /* Disabling Significant motion interrupt if enabled */
    reg_addr = BMI160_INT_MOTION_3_ADDR;
    rslt = bmi160_get_regs(reg_addr, &data, 1, dev);
    if (rslt != BMI160_OK)
        return rslt;

    temp = (data & BMI160_SIG_MOTION_SEL_MASK);

    if (temp) {
        temp = data & ~BMI160_SIG_MOTION_SEL_MASK;
        data = temp | ((0 << 1) & BMI160_SIG_MOTION_SEL_MASK);
        rslt = bmi160_set_regs(reg_addr, &data, 1, dev);// Write data to register
        if (rslt != BMI160_OK)
            return rslt;
    }

    /* Configure Int Map register to map interrupt pin to Slope/Any motion interrupt */
    if (int_config->int_channel == BMI160_INT_CHANNEL_1) {
        reg_addr = BMI160_INT_MAP_0_ADDR;
        rslt = bmi160_get_regs(reg_addr, &data, 1, dev);
        if (rslt != BMI160_OK)
            return rslt;

        temp = data & ~BMI160_INT1_SLOPE_MASK;
        data = temp | ((1 << 2) & BMI160_INT1_SLOPE_MASK);
    }
    else {
        reg_addr = BMI160_INT_MAP_2_ADDR;
        rslt = bmi160_get_regs(reg_addr, &data, 1, dev);
        if (rslt != BMI160_OK)
            return rslt;

        temp = data & ~BMI160_INT2_SLOPE_MASK;
        data = temp | ((1 << 2)& BMI160_INT2_SLOPE_MASK);
    }

    rslt = bmi160_set_regs(reg_addr, &data, 1, dev);// Write data to appropriate MAP address
    if (rslt != BMI160_OK)
        return rslt;


    /* Configure Int data 1 register to add source of interrupt */
    reg_addr = BMI160_INT_DATA_1_ADDR;
    rslt = bmi160_get_regs(reg_addr, &data, 1, dev);
    if (rslt != BMI160_OK)
        return rslt;

    temp = data & ~BMI160_MOTION_SRC_INT_MASK;
    data = temp | ((0 << 7)& BMI160_MOTION_SRC_INT_MASK);
    rslt = bmi160_set_regs(reg_addr, &data, 1, dev);// Write data to DATA 1 address
    if (rslt != BMI160_OK)
        return rslt;

    /* Configure Int Motion 0 register */
    reg_addr = BMI160_INT_MOTION_0_ADDR;
    rslt = bmi160_get_regs(reg_addr, &data, 1, dev);
    if (rslt != BMI160_OK)
        return rslt;

    dur = (uint8_t)slope_int_cfg->slope_dur;/* slope duration */
    temp = data & ~BMI160_SLOPE_INT_DUR_MASK;
    data = temp | (dur & BMI160_MOTION_SRC_INT_MASK);
    data_array[count++] = data;

    data_array[count++] = slope_int_cfg->slope_thr;/* add slope threshold */

    /* INT MOTION 0 and INT MOTION 1 address lie consecutively,hence writing data to respective registers at one go */
    rslt = bmi160_set_regs(reg_addr, data_array, count, dev);//Writing to Int_motion 0 and Int_motion 1 Address simultaneously
    return rslt;

}

/*!
 * @brief This API sets tap interrupts.Interrupt is fired when
 * tap movements happen.
 */
static int8_t set_accel_tap_int(struct bmi160_intr_sett *int_config, struct bmi160_dev *dev)
{
    int8_t rslt = BMI160_OK;
    uint8_t reg_addr, data, temp, data_array[2], count = 0;
    uint8_t dur, shock, quiet, thres;

    /* Null-pointer check */
    if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_ms == NULL) || ((int_config == NULL)))
        return BMI160_E_NULL_PTR;

    /* updating the interrupt structure to local structure */
    struct bmi160_acc_tap_int_cfg *tap_int_cfg = &(int_config->int_type_cfg.acc_tap_int);

    /* Enable single tap or double tap interrupt in Int Enable 0 register */
    reg_addr = BMI160_INT_ENABLE_0_ADDR;
    rslt = bmi160_get_regs(reg_addr, &data, 1, dev);
    if (rslt != BMI160_OK)
        return rslt;

    if (int_config->int_type == BMI160_ACC_SINGLE_TAP_INT) {
        temp = data & ~BMI160_SINGLE_TAP_INT_EN_MASK;
        data = temp | ((1 << 5) & BMI160_SINGLE_TAP_INT_EN_MASK);
    }
    else {
        temp = data & ~BMI160_DOUBLE_TAP_INT_EN_MASK;
        data = temp | ((1 << 4) & BMI160_DOUBLE_TAP_INT_EN_MASK);
    }
    rslt = bmi160_set_regs(reg_addr, &data, 1, dev);/* Write to Enable 0 Address */
    if (rslt != BMI160_OK)
        return rslt;

    /* Configure Interrupt pins */
    rslt = set_intr_pin_config(int_config, dev);
    if (rslt != BMI160_OK)
        return rslt;

    /* Configure Map register to map interrupt pin to single or double tap interrupt*/
    if (int_config->int_channel == BMI160_INT_CHANNEL_1) {
        reg_addr = BMI160_INT_MAP_0_ADDR;
        rslt = bmi160_get_regs(reg_addr, &data, 1, dev);
        if (rslt != BMI160_OK)
            return rslt;
        if (int_config->int_type == BMI160_ACC_SINGLE_TAP_INT) {
            temp = data & ~BMI160_INT1_SINGLE_TAP_MASK;
            data = temp | ((1 << 5) & BMI160_INT1_SINGLE_TAP_MASK);
        }
        else {
            temp = data & ~BMI160_INT1_DOUBLE_TAP_MASK;
            data = temp | ((1 << 4) & BMI160_INT1_DOUBLE_TAP_MASK);
        }
    }
    else {
        reg_addr = BMI160_INT_MAP_2_ADDR;
        rslt = bmi160_get_regs(reg_addr, &data, 1, dev);
        if (rslt != BMI160_OK)
            return rslt;

        if (int_config->int_type == BMI160_ACC_SINGLE_TAP_INT) {
            temp = data & ~BMI160_INT2_SINGLE_TAP_MASK;
            data = temp | ((1 << 5) & BMI160_INT2_SINGLE_TAP_MASK);
        }
        else {
            temp = data & ~BMI160_INT2_DOUBLE_TAP_MASK;
            data = temp | ((1 << 4) & BMI160_INT2_DOUBLE_TAP_MASK);
        }
    }

    rslt = bmi160_set_regs(reg_addr, &data, 1, dev);// Write data to appropriate MAP address
    if (rslt != BMI160_OK)
        return rslt;


    /* Configure Int data 0 register to add source of interrupt */
    reg_addr = BMI160_INT_DATA_0_ADDR;
    rslt = bmi160_get_regs(reg_addr, &data, 1, dev);
    if (rslt != BMI160_OK)
        return rslt;

    temp = data & ~BMI160_TAP_SRC_INT_MASK;
    data = temp | ((0 << 3) & BMI160_TAP_SRC_INT_MASK);
    rslt = bmi160_set_regs(reg_addr, &data, 1, dev);/* Write data to Data 0 address*/
    if (rslt != BMI160_OK)
        return rslt;

    /* Configure tap 0 register for tap shock,tap quiet duration in case of single tap interrupt */
    reg_addr = BMI160_INT_TAP_0_ADDR;
    rslt = bmi160_get_regs(reg_addr, data_array, 2, dev);
    if (rslt != BMI160_OK)
        return rslt;

    data = data_array[count];

    if (int_config->int_type == BMI160_ACC_DOUBLE_TAP_INT) {
        dur  = (uint8_t)tap_int_cfg->tap_dur;
        temp = (data & ~BMI160_TAP_DUR_MASK);
        data = temp | (dur & BMI160_TAP_DUR_MASK); //Add tap duration data in case of double tap interrupt
    }

    shock = (uint8_t)tap_int_cfg->tap_shock;/* tap shock */
    temp = data & ~BMI160_TAP_SHOCK_DUR_MASK;
    data = temp | ((shock << 6) & BMI160_TAP_SHOCK_DUR_MASK);

    quiet = (uint8_t)tap_int_cfg->tap_quiet;/* tap quiet */
    temp = data & ~BMI160_TAP_QUIET_DUR_MASK;
    data = temp | ((quiet << 7) & BMI160_TAP_QUIET_DUR_MASK);

    data_array[count++] = data;

    data = data_array[count];
    thres = (uint8_t)tap_int_cfg->tap_thr;/* tap threshold */
    temp = data & ~BMI160_TAP_THRES_MASK;
    data = temp | (thres & BMI160_TAP_THRES_MASK);

    data_array[count++] = data;
    /* TAP 0 and TAP 1 address lie consecutively,hence writing data to respective registers at one go */
    rslt = bmi160_set_regs(reg_addr, data_array, count, dev);//Writing to Tap 0 and Tap 1 Address simultaneously
    return rslt;
}

/*!
 * @brief This API sets the data ready interrupt for both accel and gyro.
 * This interrupt occurs when new accel and gyro data comes.
 */
static int8_t set_accel_gyro_data_ready_int(struct bmi160_intr_sett *int_config, struct bmi160_dev *dev)
{
    int8_t rslt = BMI160_OK;
    uint8_t reg_addr, data, temp;

    /* Null-pointer check */
    if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_ms == NULL) || ((int_config == NULL)))
        return BMI160_E_NULL_PTR;

    /* Enable data ready interrupt in Int Enable 1 register */
    reg_addr = BMI160_INT_ENABLE_1_ADDR;
    rslt = bmi160_get_regs(reg_addr, &data, 1, dev);
    if (rslt != BMI160_OK)
        return rslt;

    temp = data & ~BMI160_DATA_RDY_INT_EN_MASK;
    data = temp | ((1 << 4) & BMI160_DATA_RDY_INT_EN_MASK);

    rslt = bmi160_set_regs(reg_addr, &data, 1, dev);/* Writing data to INT ENABLE 1 Address */
    if (rslt != BMI160_OK)
        return rslt;

    /* Configure Interrupt pins */
    rslt = set_intr_pin_config(int_config, dev);
    if (rslt != BMI160_OK)
        return rslt;

    /* Configure Map register to map interrupt pin to data ready interrupt*/
    reg_addr = BMI160_INT_MAP_1_ADDR;
    rslt = bmi160_get_regs(reg_addr, &data, 1, dev);
    if (rslt != BMI160_OK)
        return rslt;

    if (int_config->int_channel == BMI160_INT_CHANNEL_1) {
        temp = data & ~BMI160_INT1_DATA_READY_MASK;
        data = temp | ((1 << 7) & BMI160_INT1_DATA_READY_MASK);
    }
    else {
        temp = data & ~BMI160_INT2_DATA_READY_MASK;
        data = temp | ((1 << 3) & BMI160_INT2_DATA_READY_MASK);
    }
    rslt = bmi160_set_regs(reg_addr, &data, 1, dev);/* Writing data to Map 1 address */

    return rslt;

}

/*!
 * @brief This API sets the significant motion interrupt of the sensor.This
 * interrupt occurs when there is change in user location.
 */
static int8_t set_accel_sig_motion_int(struct bmi160_intr_sett *int_config, struct bmi160_dev *dev)
{
    int8_t rslt = BMI160_OK;
    uint8_t reg_addr, data, temp;

    /* Null-pointer check */
    if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_ms == NULL) || ((int_config == NULL)))
        return BMI160_E_NULL_PTR;

    /* updating the interrupt structure to local structure */
    struct bmi160_acc_sig_mot_int_cfg *sig_mot_int_cfg = &(int_config->int_type_cfg.acc_sig_motion_int);

    /* For significant motion,enable any motion x,any motion y,any motion z in Int Enable 0 register */
    reg_addr = BMI160_INT_ENABLE_0_ADDR;
    rslt = bmi160_get_regs(reg_addr, &data, 1, dev);
    if (rslt != BMI160_OK)
        return rslt;

    temp = data & ~BMI160_SIG_MOTION_INT_EN_MASK;
    data = temp | (7 & BMI160_SIG_MOTION_INT_EN_MASK);
    rslt = bmi160_set_regs(reg_addr, &data, 1, dev);/* write data to Int Enable 0 register */
    if (rslt != BMI160_OK)
        return rslt;

    /* Configure Interrupt pins */
    rslt = set_intr_pin_config(int_config, dev);
    if (rslt != BMI160_OK)
        return rslt;

    /* Configure Int Map register to map interrupt pin to Significant motion interrupt */
    if (int_config->int_channel == BMI160_INT_CHANNEL_1) {
        reg_addr = BMI160_INT_MAP_0_ADDR;
        rslt = bmi160_get_regs(reg_addr, &data, 1, dev);
        if (rslt != BMI160_OK)
            return rslt;

        temp = data & ~BMI160_INT1_SLOPE_MASK;
        data = temp | ((1 << 2) & BMI160_INT1_SLOPE_MASK);
    }
    else {
        reg_addr = BMI160_INT_MAP_2_ADDR;
        rslt = bmi160_get_regs(reg_addr, &data, 1, dev);
        if (rslt != BMI160_OK)
            return rslt;

        temp = data & ~BMI160_INT2_SLOPE_MASK;
        data = temp | ((1 << 2) & BMI160_INT2_SLOPE_MASK);
    }

    rslt = bmi160_set_regs(reg_addr, &data, 1, dev);//Write data to appropriate MAP address
    if (rslt != BMI160_OK)
        return rslt;

    /* Configure Int data 1 register to add source of interrupt */
    reg_addr = BMI160_INT_DATA_1_ADDR;
    rslt = bmi160_get_regs(reg_addr, &data, 1, dev);
    if (rslt != BMI160_OK)
        return rslt;

    temp = data & ~BMI160_MOTION_SRC_INT_MASK;
    data = temp | ((0 << 7) & BMI160_MOTION_SRC_INT_MASK);
    rslt = bmi160_set_regs(reg_addr, &data, 1, dev);// Write data to DATA 1 address
    if (rslt != BMI160_OK)
        return rslt;

    /* Configuring INT_MOTION registers */
    /* Write significant motion threshold .This threshold is same as any motion threshold */
    reg_addr = BMI160_INT_MOTION_1_ADDR;
    data = sig_mot_int_cfg->sig_mot_thres;
    rslt = bmi160_set_regs(reg_addr, &data, 1, dev);// Write data to INT_MOTION 1 address
    if (rslt != BMI160_OK)
        return rslt;

    reg_addr = BMI160_INT_MOTION_3_ADDR;
    rslt = bmi160_get_regs(reg_addr, &data, 1, dev);
    if (rslt != BMI160_OK)
        return rslt;

    temp = data & ~BMI160_SIG_MOTION_SKIP_MASK;
    data = temp | ((sig_mot_int_cfg->sig_mot_skip << 2) & BMI160_SIG_MOTION_SKIP_MASK); /* adding skip time of sig_motion interrupt*/
    temp = data & ~BMI160_SIG_MOTION_PROOF_MASK;
    data = temp | ((sig_mot_int_cfg->sig_mot_proof << 4) & BMI160_SIG_MOTION_PROOF_MASK); /* adding proof time of sig_motion interrupt */
    data = data | 0x02; /* enabling int_sig_mot_sel bit to select significant motion interrupt */
    rslt = bmi160_set_regs(reg_addr, &data, 1, dev);

    return rslt;
}

/*!
 * @brief This API sets the no motion/slow motion interrupt of the sensor.
 * Slow motion is similar to any motion interrupt.No motion interrupt
 * occurs when slope bet. two accel values falls below preset threshold
 * for preset duration.
 */
static int8_t set_accel_no_motion_int(struct bmi160_intr_sett *int_config, struct bmi160_dev *dev)
{
    int8_t rslt = BMI160_OK;
    uint8_t reg_addr, data, temp, data_array[2];

    /* Null-pointer check */
    if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_ms == NULL) || ((int_config == NULL)))
        return BMI160_E_NULL_PTR;

    /* updating the interrupt structure to local structure */
    struct bmi160_acc_no_motion_int_cfg *no_mot_int_cfg = &(int_config->int_type_cfg.acc_no_motion_int);

    /* Enable no motion x,no motion y,no motion z in Int Enable 2 register */
    reg_addr = BMI160_INT_ENABLE_2_ADDR;
    rslt = bmi160_get_regs(reg_addr, &data, 1, dev);
    if (rslt != BMI160_OK)
        return rslt;

    if (no_mot_int_cfg->no_motion_x == 1) {
        temp = data & ~BMI160_NO_MOTION_X_INT_EN_MASK;
        data = temp | (1 & BMI160_NO_MOTION_X_INT_EN_MASK);//Adding No_motion x axis
    }
    if (no_mot_int_cfg->no_motion_y == 1) {
        temp = data & ~BMI160_NO_MOTION_Y_INT_EN_MASK;
        data = temp | ((1 << 1) & BMI160_NO_MOTION_Y_INT_EN_MASK);//Adding No_motion x axis
    }
    if (no_mot_int_cfg->no_motion_z == 1) {
        temp = data & ~BMI160_NO_MOTION_Z_INT_EN_MASK;
        data = temp | ((1 << 2) & BMI160_NO_MOTION_Z_INT_EN_MASK);//Adding No_motion x axis
    }
    rslt = bmi160_set_regs(reg_addr, &data, 1, dev);/* write data to Int Enable 2 register */
    if (rslt != BMI160_OK)
        return rslt;

    /* Configure Interrupt pins */
    rslt = set_intr_pin_config(int_config, dev);
    if (rslt != BMI160_OK)
        return rslt;

    /* Configure Int Map register to map interrupt pin to No motion interrupt */
    if (int_config->int_channel == BMI160_INT_CHANNEL_1) {
        reg_addr = BMI160_INT_MAP_0_ADDR;
        rslt = bmi160_get_regs(reg_addr, &data, 1, dev);
        if (rslt != BMI160_OK)
            return rslt;

        temp = data & ~BMI160_INT1_NO_MOTION_MASK;
        data = temp | ((1 << 3) & BMI160_INT1_NO_MOTION_MASK);
    }
    else {
        reg_addr = BMI160_INT_MAP_2_ADDR;
        rslt = bmi160_get_regs(reg_addr, &data, 1, dev);
        if (rslt != BMI160_OK)
            return rslt;

        temp = data & ~BMI160_INT2_NO_MOTION_MASK;
        data = temp | ((1 << 3) & BMI160_INT2_NO_MOTION_MASK);
    }

    rslt = bmi160_set_regs(reg_addr, &data, 1, dev);//Write data to appropriate MAP address
    if (rslt != BMI160_OK)
        return rslt;

    /* Configure Int data 1 register to add source of interrupt */
    reg_addr = BMI160_INT_DATA_1_ADDR;
    rslt = bmi160_get_regs(reg_addr, &data, 1, dev);
    if (rslt != BMI160_OK)
        return rslt;

    temp = data & ~BMI160_MOTION_SRC_INT_MASK;
    data = temp | ((0 << 7) & BMI160_MOTION_SRC_INT_MASK);
    rslt = bmi160_set_regs(reg_addr, &data, 1, dev);// Write data to DATA 1 address
    if (rslt != BMI160_OK)
        return rslt;

    /* Configuring INT_MOTION register */
    reg_addr = BMI160_INT_MOTION_0_ADDR;
    rslt = bmi160_get_regs(reg_addr, &data, 1, dev);
    if (rslt != BMI160_OK)
        return rslt;

    temp = data & ~BMI160_NO_MOTION_INT_DUR_MASK;
    data = temp | ((no_mot_int_cfg->no_motion_dur << 2) & BMI160_NO_MOTION_INT_DUR_MASK);// Adding no_motion duration
    rslt = bmi160_set_regs(reg_addr, &data, 1, dev);// Write data to NO_MOTION 0 address
    if (rslt != BMI160_OK)
        return rslt;

    reg_addr = BMI160_INT_MOTION_3_ADDR;
    rslt = bmi160_get_regs(reg_addr, &data, 1, dev);
    if (rslt != BMI160_OK)
        return rslt;

    temp = data & ~BMI160_NO_MOTION_SEL_BIT_MASK;
    data = temp | (no_mot_int_cfg->no_motion_sel & BMI160_NO_MOTION_SEL_BIT_MASK);// Adding no_motion_sel bit

    data_array[1] = data;
    data_array[0] = no_mot_int_cfg->no_motion_thres; // Adding no motion threshold
    reg_addr = BMI160_INT_MOTION_2_ADDR;
    rslt = bmi160_set_regs(reg_addr, data_array, 2, dev);// writing data to INT_MOTION 2 and INT_MOTION 3 address simultaneously
    return rslt;

}

/*!
 * @brief This API sets the step detection interrupt.This interrupt
 * occurs when the single step causes accel values to go above
 * preset threshold.
 */
static int8_t set_accel_step_detect_int(struct bmi160_intr_sett *int_config, struct bmi160_dev *dev)
{
    int8_t rslt = BMI160_OK;
    uint8_t reg_addr, data, temp, data_array[2];

    /* Null-pointer check */
    if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_ms == NULL) || ((int_config == NULL)))
        return BMI160_E_NULL_PTR;

    /* updating the interrupt structure to local structure */
    struct bmi160_acc_step_detect_int_cfg *step_detect_int_cfg = &(int_config->int_type_cfg.acc_step_detect_int);

    /* Enable data ready interrupt in Int Enable 2 register */
    reg_addr = BMI160_INT_ENABLE_2_ADDR;
    rslt = bmi160_get_regs(reg_addr, &data, 1, dev);
    if (rslt != BMI160_OK)
        return rslt;

    temp = data & ~BMI160_STEP_DETECT_INT_EN_MASK;
    data = temp | ((1 << 3) & BMI160_STEP_DETECT_INT_EN_MASK);

    rslt = bmi160_set_regs(reg_addr, &data, 1, dev);/* Writing data to INT ENABLE 2 Address */
    if (rslt != BMI160_OK)
        return rslt;

    /* Configure Interrupt pins */
    rslt = set_intr_pin_config(int_config, dev);
    if (rslt != BMI160_OK)
        return rslt;

    /* Configure Int Map register to map interrupt pin to step detector */
    if (int_config->int_channel == BMI160_INT_CHANNEL_1) {
        reg_addr = BMI160_INT_MAP_0_ADDR;
        rslt = bmi160_get_regs(reg_addr, &data, 1, dev);
        if (rslt != BMI160_OK)
            return rslt;

        temp = data & ~BMI160_INT1_STEP_DETECT_MASK;
        data = temp | (1 & BMI160_INT1_STEP_DETECT_MASK);
    }
    else {
        reg_addr = BMI160_INT_MAP_2_ADDR;
        rslt = bmi160_get_regs(reg_addr, &data, 1, dev);
        if (rslt != BMI160_OK)
            return rslt;

        temp = data & ~BMI160_INT2_STEP_DETECT_MASK;
        data = temp | (1 & BMI160_INT2_STEP_DETECT_MASK);
    }

    rslt = bmi160_set_regs(reg_addr, &data, 1, dev);// Write data to appropriate MAP address
    if (rslt != BMI160_OK)
        return rslt;

    /* Configuring STEP_CONFIG register */
    reg_addr = BMI160_INT_STEP_CONFIG_0_ADDR;
    rslt = bmi160_get_regs(reg_addr, &data, 1, dev);
    if (rslt != BMI160_OK)
        return rslt;

    temp = data & ~BMI160_STEP_DETECT_MIN_THRES_MASK;
    data = temp | ((step_detect_int_cfg->min_threshold << 3) & BMI160_STEP_DETECT_MIN_THRES_MASK); // Adding min_threshold

    temp = data & ~BMI160_STEP_DETECT_STEPTIME_MIN_MASK;
    data = temp | ((step_detect_int_cfg->steptime_min) & BMI160_STEP_DETECT_STEPTIME_MIN_MASK);// Adding steptime_min

    data_array[0] = data;

    /* Normal mode setting */
    if (step_detect_int_cfg->normal_mode_en == 1) {
       data_array[1] = 0x03;
    }
    /* Sensitive mode setting */
    if (step_detect_int_cfg->sensitive_mode_en == 1) {
        data_array[1] = 0x00;
    }
    /* Robust mode setting */
    if (step_detect_int_cfg->robust_mode_en == 1) {
        data_array[1] = 0x07;
    }

    rslt = bmi160_set_regs(reg_addr, data_array, 2, dev);// Write data to STEP_CONFIG register
    return rslt;
}

/*!
 * @brief This API sets the orientation interrupt of the sensor.This
 * interrupt occurs when there is orientation change in the sensor
 * with respect to gravitational field vector g.
 */
static int8_t set_accel_orientation_int(struct bmi160_intr_sett *int_config, struct bmi160_dev *dev)
{
    int8_t rslt = BMI160_OK;
    uint8_t reg_addr, data, temp, data_array[2];

    /* Null-pointer check */
    if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_ms == NULL) || ((int_config == NULL)))
        return BMI160_E_NULL_PTR;

    /* updating the interrupt structure to local structure */
    struct bmi160_acc_orient_int_cfg *orient_int_cfg = &(int_config->int_type_cfg.acc_orient_int);

    /* Enable data ready interrupt in Int Enable 0 register */
    reg_addr = BMI160_INT_ENABLE_0_ADDR;
    rslt = bmi160_get_regs(reg_addr, &data, 1, dev);
    if (rslt != BMI160_OK)
        return rslt;

    temp = data & ~BMI160_ORIENT_INT_EN_MASK;
    data = temp | ((1 << 6) & BMI160_ORIENT_INT_EN_MASK);

    rslt = bmi160_set_regs(reg_addr, &data, 1, dev);/* Writing data to INT ENABLE 0 Address */
    if (rslt != BMI160_OK)
        return rslt;

    /* Configure Interrupt pins */
    rslt = set_intr_pin_config(int_config, dev);
    if (rslt != BMI160_OK)
        return rslt;

    /* Configure Int Map register to map interrupt pin to orientation interrupt */
    if (int_config->int_channel == BMI160_INT_CHANNEL_1) {
        reg_addr = BMI160_INT_MAP_0_ADDR;
        rslt = bmi160_get_regs(reg_addr, &data, 1, dev);
        if (rslt != BMI160_OK)
            return rslt;

        temp = data & ~BMI160_INT1_ORIENT_MASK;
        data = temp | ((1 << 6) & BMI160_INT1_ORIENT_MASK);
    }
    else {
        reg_addr = BMI160_INT_MAP_2_ADDR;
        rslt = bmi160_get_regs(reg_addr, &data, 1, dev);
        if (rslt != BMI160_OK)
            return rslt;

        temp = data & ~BMI160_INT2_ORIENT_MASK;
        data = temp | ((1 << 6) & BMI160_INT2_ORIENT_MASK);
    }

    rslt = bmi160_set_regs(reg_addr, &data, 1, dev);// Write data to appropriate MAP address
    if (rslt != BMI160_OK)
        return rslt;

    /* Configuring INT_ORIENT registers */
    reg_addr = BMI160_INT_ORIENT_0_ADDR;
    rslt = bmi160_get_regs(reg_addr, data_array, 2, dev);
    if (rslt != BMI160_OK)
        return rslt;

    data = data_array[0];
    temp = data & ~BMI160_ORIENT_MODE_MASK;
    data = temp | ((orient_int_cfg->orient_mode) & BMI160_ORIENT_MODE_MASK); // Adding Orientation mode
    temp = data & ~BMI160_ORIENT_BLOCK_MASK;
    data = temp | ((orient_int_cfg->orient_blocking << 2) & BMI160_ORIENT_BLOCK_MASK); // Adding Orientation blocking
    temp = data & ~BMI160_ORIENT_HYST_MASK;
    data = temp | ((orient_int_cfg->orient_hyst << 4) & BMI160_ORIENT_HYST_MASK); // Adding Orientation hysteresis
    data_array[0] = data;

    data = data_array[1];
    temp = data & ~BMI160_ORIENT_THETA_MASK;
    data = temp | ((orient_int_cfg->orient_theta) & BMI160_ORIENT_THETA_MASK); // Adding Orientation threshold
    temp = data & ~BMI160_ORIENT_UD_ENABLE;
    data = temp | ((orient_int_cfg->orient_ud_en << 6) & BMI160_ORIENT_UD_ENABLE);// Adding Orient_ud_en
    temp = data & ~BMI160_AXES_EN_MASK;
    data = temp | ((orient_int_cfg->axes_ex << 7) & BMI160_AXES_EN_MASK); // Adding axes_en
    data_array[1] = data;
    rslt = bmi160_set_regs(reg_addr, data_array, 2, dev);// Writing data to INT_ORIENT 0 and INT_ORIENT 1 registers simultaneously

    return rslt;
}

/*!
 * @brief This API sets the flat interrupt of the sensor.This interrupt
 * occurs in case of flat orientation
 */
static int8_t set_accel_flat_detect_int(struct bmi160_intr_sett *int_config, struct bmi160_dev *dev)
{
    int8_t rslt = BMI160_OK;
    uint8_t reg_addr, data, temp, data_array[2];

    /* Null-pointer check */
    if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_ms == NULL) || ((int_config == NULL)))
        return BMI160_E_NULL_PTR;

    /* updating the interrupt structure to local structure */
    struct bmi160_acc_flat_detect_int_cfg *flat_detect_int = &(int_config->int_type_cfg.acc_flat_int);

    /* Enable flat interrupt in Int Enable 0 register */
    reg_addr = BMI160_INT_ENABLE_0_ADDR;
    rslt = bmi160_get_regs(reg_addr, &data, 1, dev);
    if (rslt != BMI160_OK)
        return rslt;

    temp = data & ~BMI160_FLAT_INT_EN_MASK;
    data = temp | ((1 << 7) & BMI160_FLAT_INT_EN_MASK);
    rslt = bmi160_set_regs(reg_addr, &data, 1, dev);/* Writing data to INT ENABLE 0 Address */
    if (rslt != BMI160_OK)
        return rslt;

    /* Configure Interrupt pins */
    rslt = set_intr_pin_config(int_config, dev);
    if (rslt != BMI160_OK)
        return rslt;

    /* Configure Map register to map interrupt pin to flat interrupt*/
    if (int_config->int_channel == BMI160_INT_CHANNEL_1) {
        reg_addr = BMI160_INT_MAP_0_ADDR;
        rslt = bmi160_get_regs(reg_addr, &data, 1, dev);
        if (rslt != BMI160_OK)
            return rslt;

        temp = data & ~BMI160_INT1_FLAT_MASK;
        data = temp | ((1 << 7) & BMI160_INT1_FLAT_MASK);
    }
    else {
        reg_addr = BMI160_INT_MAP_2_ADDR;
        rslt = bmi160_get_regs(reg_addr, &data, 1, dev);
        if (rslt != BMI160_OK)
            return rslt;
        temp = data & ~BMI160_INT2_FLAT_MASK;
        data = temp | ((1 << 7) & BMI160_INT2_FLAT_MASK);
    }

    rslt = bmi160_set_regs(reg_addr, &data, 1, dev);//Write data to appropriate MAP address
    if (rslt != BMI160_OK)
        return rslt;

    /* Configuring INT_FLAT register */
    reg_addr = BMI160_INT_FLAT_0_ADDR;
    rslt = bmi160_get_regs(reg_addr, data_array, 2, dev);
    if (rslt != BMI160_OK)
        return rslt;

    data = data_array[0];
    temp = data & ~BMI160_FLAT_THRES_MASK;
    data = temp | ((flat_detect_int->flat_theta) & BMI160_FLAT_THRES_MASK); // Adding flat theta
    data_array[0] = data;

    data = data_array[1];
    temp = data & ~BMI160_FLAT_HOLD_TIME_MASK;
    data = temp | ((flat_detect_int->flat_hold_time << 4) & BMI160_FLAT_HOLD_TIME_MASK); // Adding flat hold time
    temp = data & ~BMI160_FLAT_HYST_MASK;
    data = temp | ((flat_detect_int->flat_hy) & BMI160_FLAT_HYST_MASK); // Adding flat hysteresis
    data_array[1] = data;
    rslt = bmi160_set_regs(reg_addr, data_array, 2, dev);// Writing data to INT_FLAT 0 and INT_FLAT 1 registers simultaneously
    return rslt;
}

/*!
 * @brief This API sets the low-g interrupt of the sensor.This interrupt
 * occurs during free-fall.
 */
static int8_t set_accel_low_g_int(struct bmi160_intr_sett *int_config, struct bmi160_dev *dev)
{
    int8_t rslt = BMI160_OK;
    uint8_t reg_addr, data, temp, data_array[3];

    /* Null-pointer check */
    if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_ms == NULL) || ((int_config == NULL)))
        return BMI160_E_NULL_PTR;

    /* updating the interrupt structure to local structure */
    struct bmi160_acc_low_g_int_cfg *low_g_int = &(int_config->int_type_cfg.acc_low_g_int);

    /* Enable low-g interrupt in Int Enable 1 register */
    reg_addr = BMI160_INT_ENABLE_1_ADDR;
    rslt = bmi160_get_regs(reg_addr, data_array, 2, dev);
    if (rslt != BMI160_OK)
        return rslt;

    data = data_array[0];
    temp = data & ~BMI160_LOW_G_INT_EN_MASK;
    data = temp | ((1 << 3) & BMI160_LOW_G_INT_EN_MASK);
    data_array[0] = data;

    data = data_array[1];
    temp = (data & BMI160_STEP_DETECT_EN_MASK);
    // Disabling step detection if enabled already
    if (temp) {
        temp = data & ~BMI160_STEP_DETECT_EN_MASK;
        data = temp | ((0 << 3) & BMI160_STEP_DETECT_EN_MASK);
        data_array[2] = data;
        rslt = bmi160_set_regs(reg_addr, data_array, 2, dev);/* Writing data to INT ENABLE 1 Address */
    }
    else
        rslt = bmi160_set_regs(reg_addr, data_array, 1, dev);/* Writing data to INT ENABLE 1 Address */

    if (rslt != BMI160_OK)
        return rslt;

    /* Configure STEP_CONF register */
    reg_addr = BMI160_INT_STEP_CONFIG_1_ADDR;
    rslt = bmi160_get_regs(reg_addr, &data, 1, dev);
    if (rslt != BMI160_OK)
        return rslt;

    if (data) {
        temp = data & ~BMI160_STEP_COUNT_EN_BIT_MASK;
        data = temp | ((0 << 3) & BMI160_STEP_COUNT_EN_BIT_MASK);// Disabling step_count_en bit
        rslt = bmi160_set_regs(reg_addr, &data, 1, dev);// Writing to STEP_CONF register
        if (rslt != BMI160_OK)
            return rslt;
    }

    /* Configure Interrupt pins */
    rslt = set_intr_pin_config(int_config, dev);
    if (rslt != BMI160_OK)
        return rslt;

    /* Configure Map register to map interrupt pin to low-g interrupt*/
    if (int_config->int_channel == BMI160_INT_CHANNEL_1) {
        reg_addr = BMI160_INT_MAP_0_ADDR;
        rslt = bmi160_get_regs(reg_addr, &data, 1, dev);
        if (rslt != BMI160_OK)
            return rslt;

        temp = data & ~BMI160_INT1_LOW_G_MASK;
        data = temp | (1 & BMI160_INT1_LOW_G_MASK);
    }
    else {
        reg_addr = BMI160_INT_MAP_2_ADDR;
        rslt = bmi160_get_regs(reg_addr, &data, 1, dev);
        if (rslt != BMI160_OK)
            return rslt;

        temp = data & ~BMI160_INT2_LOW_G_MASK;
        data = temp | (1 & BMI160_INT2_LOW_G_MASK);
    }

    rslt = bmi160_set_regs(reg_addr, &data, 1, dev);//Write data to appropriate MAP address
    if (rslt != BMI160_OK)
        return rslt;


    /* Configure Int data 0 register to add source of interrupt */
    reg_addr = BMI160_INT_DATA_0_ADDR;
    rslt = bmi160_get_regs(reg_addr, &data, 1, dev);
    if (rslt != BMI160_OK)
        return rslt;

    temp = data & ~BMI160_LOW_HIGH_SRC_INT_MASK;
    data = temp | ((0 << 7) & BMI160_LOW_HIGH_SRC_INT_MASK);
    rslt = bmi160_set_regs(reg_addr, &data, 1, dev);/* Write data to Data 0 address*/
    if (rslt != BMI160_OK)
        return rslt;

    /* Configuring INT_LOWHIGH register for low-g interrupt */
    reg_addr = BMI160_INT_LOWHIGH_2_ADDR;
    rslt = bmi160_get_regs(reg_addr, &data, 1, dev);
    if (rslt != BMI160_OK)
        return rslt;

    temp = data & ~BMI160_LOW_G_HYST_MASK;
    data = temp | (low_g_int->low_hyst & BMI160_LOW_G_HYST_MASK); //Adding low-g hysteresis
    temp = data & ~BMI160_LOW_G_LOW_MODE_MASK;
    data = temp | ((low_g_int->low_mode << 2) & BMI160_LOW_G_LOW_MODE_MASK); //Adding low-mode
    data_array[2] = data;
    data_array[1] = low_g_int->low_thres;// Adding low-g threshold
    data_array[0] = low_g_int->low_dur;// Adding low-g interrupt delay
    reg_addr = BMI160_INT_LOWHIGH_0_ADDR;
    rslt = bmi160_set_regs(reg_addr, data_array, 3, dev);// Writing data to INT_LOWHIGH 0,1,2 registers simultaneously

    return rslt;
}

/*!
 * @brief This API sets the high-g interrupt of the sensor.The interrupt
 * occurs if the absolute value of acceleration data of any enabled axis
 * exceeds the programmed threshold and the sign of the value does not
 * change for a preset duration.
 */
static int8_t set_accel_high_g_int(struct bmi160_intr_sett *int_config, struct bmi160_dev *dev)
{
    int8_t rslt = BMI160_OK;
    uint8_t reg_addr, data, temp, data_array[3];

    /* Null-pointer check */
    if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_ms == NULL) || ((int_config == NULL)))
        return BMI160_E_NULL_PTR;

    /* updating the interrupt structure to local structure */
    struct bmi160_acc_high_g_int_cfg *high_g_int_cfg = &(int_config->int_type_cfg.acc_high_g_int);

    /* Enable high-g x,high-g y,high-g z in Int Enable 1 register */
    reg_addr = BMI160_INT_ENABLE_1_ADDR;
    rslt = bmi160_get_regs(reg_addr, &data, 1, dev);
    if (rslt != BMI160_OK)
        return rslt;

    if (high_g_int_cfg->high_g_x == 1) {
        temp = data & ~BMI160_HIGH_G_X_INT_EN_MASK;
        data = temp | (1 & BMI160_HIGH_G_X_INT_EN_MASK);// Adding high-g X-axis
    }
    if (high_g_int_cfg->high_g_y == 1) {
        temp = data & ~BMI160_HIGH_G_Y_INT_EN_MASK;
        data = temp | ((1 << 1) & BMI160_HIGH_G_Y_INT_EN_MASK);// Adding high-g Y-axis
    }
    if (high_g_int_cfg->high_g_z == 1) {
        temp = data & ~BMI160_HIGH_G_Z_INT_EN_MASK;
        data = temp | ((1 << 2) & BMI160_HIGH_G_Z_INT_EN_MASK);// Adding high-g Z-axis
    }
    rslt = bmi160_set_regs(reg_addr, &data, 1, dev);/* write data to Int Enable 1 register */
    if (rslt != BMI160_OK)
        return rslt;

    /* Configure Interrupt pins */
    rslt = set_intr_pin_config(int_config, dev);
    if (rslt != BMI160_OK)
        return rslt;

    /* Configure Map register to map interrupt pin to high-g interrupt*/
    if (int_config->int_channel == BMI160_INT_CHANNEL_1) {
        reg_addr = BMI160_INT_MAP_0_ADDR;
        rslt = bmi160_get_regs(reg_addr, &data, 1, dev);
        if (rslt != BMI160_OK)
            return rslt;

        temp = data & ~BMI160_INT1_HIGH_G_MASK;
        data = temp | ((1 << 1) & BMI160_INT1_HIGH_G_MASK);
    }
    else {
        reg_addr = BMI160_INT_MAP_2_ADDR;
        rslt = bmi160_get_regs(reg_addr, &data, 1, dev);
        if (rslt != BMI160_OK)
            return rslt;
        temp = data & ~BMI160_INT2_HIGH_G_MASK;
        data = temp | ((1 << 1) & BMI160_INT2_HIGH_G_MASK);
    }

    rslt = bmi160_set_regs(reg_addr, &data, 1, dev);//Write data to appropriate MAP address
    if (rslt != BMI160_OK)
        return rslt;


    /* Configure Int data 0 register to add source of interrupt */
    reg_addr = BMI160_INT_DATA_0_ADDR;
    rslt = bmi160_get_regs(reg_addr, &data, 1, dev);
    if (rslt != BMI160_OK)
        return rslt;

    temp = data & ~BMI160_LOW_HIGH_SRC_INT_MASK;
    data = temp | ((0 << 7) & BMI160_LOW_HIGH_SRC_INT_MASK);
    rslt = bmi160_set_regs(reg_addr, &data, 1, dev);/* Write data to Data 0 address*/
    if (rslt != BMI160_OK)
        return rslt;

    /* Configure INT_LOWHIGH register for high-g interrupt */
    reg_addr = BMI160_INT_LOWHIGH_2_ADDR;
    rslt = bmi160_get_regs(reg_addr, &data, 1, dev);
    if (rslt != BMI160_OK)
        return rslt;

    temp = data & ~BMI160_HIGH_G_HYST_MASK;
    data = temp | ((high_g_int_cfg->high_hy << 6) & BMI160_HIGH_G_HYST_MASK);// Adding high-g hysteresis
    data_array[0] = data;
    data_array[1] = high_g_int_cfg->high_dur;// Adding high-g duration
    data_array[2] = high_g_int_cfg->high_thres;// Adding high-g threshold
    rslt = bmi160_set_regs(reg_addr, data_array, 3, dev);

    return rslt;
}


/*!
 * @brief This API configures the pins to fire the
 * interrupt signal when it occurs.
 */
static int8_t set_intr_pin_config(struct bmi160_intr_sett *int_config, struct bmi160_dev *dev)
{
    int8_t rslt = BMI160_OK;
    uint8_t reg_addr, temp, data, data_array[2], count = 0;

    /* Configuration of output interrupt signals on pins INT1 and INT2 are done in BMI160_INT_OUT_CTRL_ADDR register*/
    reg_addr = BMI160_INT_OUT_CTRL_ADDR;
    rslt = bmi160_get_regs(reg_addr, data_array, 2, dev);
    if (rslt != BMI160_OK)
        return rslt;

    /* updating the interrupt pin structure to local structure */
    struct bmi160_intr_pin_sett *intr_pin_sett = &(int_config->int_pin_sett);

    data = data_array[count];
    if (int_config->int_channel == BMI160_INT_CHANNEL_1) {
        /* Configuring channel 1 */
        temp = data & ~BMI160_INT1_OUTPUT_EN_MASK;
        data = temp | ((intr_pin_sett->output_en << 3) & BMI160_INT1_OUTPUT_EN_MASK);/* Output enable */

        temp = data & ~BMI160_INT1_OUTPUT_MODE_MASK;
        data = temp | ((intr_pin_sett->output_mode << 2) & BMI160_INT1_OUTPUT_MODE_MASK);/* Output mode */

        temp = data & ~BMI160_INT1_EDGE_CTRL_MASK;
        data = temp | ((intr_pin_sett->edge_ctrl) & BMI160_INT1_EDGE_CTRL_MASK);/* edge control */
    }
    else {
        /* Configuring channel 2 */
        temp = data & ~BMI160_INT2_OUTPUT_EN_MASK;
        data = temp | ((intr_pin_sett->output_en << 7) & BMI160_INT2_OUTPUT_EN_MASK);/* Output enable */

        temp = data & ~BMI160_INT2_OUTPUT_MODE_MASK;
        data = temp | ((intr_pin_sett->output_mode << 6) & BMI160_INT2_OUTPUT_MODE_MASK);/* Output mode */

        temp = data & ~BMI160_INT2_EDGE_CTRL_MASK;
        data = temp | ((intr_pin_sett->edge_ctrl << 4) & BMI160_INT2_EDGE_CTRL_MASK);/* edge control */
    }
    data_array[count++] = data;

    data = data_array[count];

    if (int_config->int_channel == BMI160_INT_CHANNEL_1) {
        /* Configuring channel 1 */
        temp = data & ~BMI160_INT1_INPUT_EN_MASK;
        data = temp | ((intr_pin_sett->input_en << 4) & BMI160_INT1_INPUT_EN_MASK);/* Input enable */
    }
    else {
        /* Configuring channel 2 */
        temp = data & ~BMI160_INT2_INPUT_EN_MASK;
        data = temp | ((intr_pin_sett->input_en << 5) & BMI160_INT2_INPUT_EN_MASK);/* Input enable */
    }

    /* In case of latch interrupt,update the latch duration */
    /* Latching holds the interrupt for the amount of latch duration time */
    temp = data & ~BMI160_INT_LATCH_MASK;
    data = temp | (intr_pin_sett->latch_dur & BMI160_INT_LATCH_MASK);/*latch duration */
    data_array[count++] = data;

    /* OUT_CTRL_INT and LATCH_INT address lie consecutively,hence writing data to respective registers at one go */
    rslt = bmi160_set_regs(reg_addr, data_array, count, dev);//Write to Out_ctrl and Latch_int register simultaneously
    return rslt;
}

/** @}*/


