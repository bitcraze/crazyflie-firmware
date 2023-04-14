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
 * @file    bmi088_defs.h
 * @date	Oct 5, 2016
 * @version
 * @brief
 *
 */

/*!
 * @defgroup bmi088_defs
 * @brief
 * @{*/

#ifndef BMI088_DEFS_H_
#define BMI088_DEFS_H_

/*********************************************************************/
/**\ header files */
#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#endif

/*********************************************************************/
/* macro definitions */

#ifdef __cplusplus
extern "C"
{
#endif

    /*
     #if (LONG_MAX) > 0x7fffffff
     #define __have_long64 1
     #elif (LONG_MAX) == 0x7fffffff
     #define __have_long32 1
     #endif
     */

#if !defined(UINT8_C)
#define INT8_C(x)       x
#if (INT_MAX) > 0x7f
#define UINT8_C(x)      x
#else
#define UINT8_C(x)      x##U
#endif
#endif

#if !defined(UINT16_C)
#define INT16_C(x)      x
#if (INT_MAX) > 0x7fff
#define UINT16_C(x)     x
#else
#define UINT16_C(x)     x##U
#endif
#endif

#if !defined(INT32_C) && !defined(UINT32_C)
#if __have_long32
#define INT32_C(x)      x##L
#define UINT32_C(x)     x##UL
#else
#define INT32_C(x)      x
#define UINT32_C(x)     x##U
#endif
#endif

#if !defined(INT64_C) && !defined(UINT64_C)
#if __have_long64
#define INT64_C(x)      x##L
#define UINT64_C(x)     x##UL
#else
#define INT64_C(x)      x##LL
#define UINT64_C(x)     x##ULL
#endif
#endif

    /*************************** Sensor macros   *****************************/

    /* Test for an endian machine */
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define LITTLE_ENDIAN   1
#elif __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
#define BIG_ENDIAN   1
#else
#error "Code does not support Endian format of the processor"
#endif

    /*************************** BMI088 Accelerometer Macros *****************************/

    /** Register map */
    /* Accel registers */

    /**\name	Accel Chip Id register */
#define BMI088_ACCEL_CHIP_ID_REG                    UINT8_C(0x00)

    /**\name	Accel Error condition register */
#define BMI088_ACCEL_ERR_REG                        UINT8_C(0x02)

    /**\name	Accel Status flag register */
#define BMI088_ACCEL_STATUS_REG                     UINT8_C(0x03)

    /**\name	Accel X LSB data register */
#define BMI088_ACCEL_X_LSB_REG                      UINT8_C(0x12)

    /**\name	Accel X MSB data register */
#define BMI088_ACCEL_X_MSB_REG                      UINT8_C(0x13)

    /**\name	Accel Y LSB data register */
#define BMI088_ACCEL_Y_LSB_REG                      UINT8_C(0x14)

    /**\name	Accel Y MSB data register */
#define BMI088_ACCEL_Y_MSB_REG                      UINT8_C(0x15)

    /**\name	Accel Z LSB data register */
#define BMI088_ACCEL_Z_LSB_REG                      UINT8_C(0x16)

    /**\name	Accel Z MSB data register */
#define BMI088_ACCEL_Z_MSB_REG                      UINT8_C(0x17)

    /**\name	Sensor time byte 0 register */
#define BMI088_ACCEL_SENSORTIME_0_REG               UINT8_C(0x18)

    /**\name	Sensor time byte 1 register */
#define BMI088_ACCEL_SENSORTIME_1_REG               UINT8_C(0x19)

    /**\name	Sensor time byte 2 register */
#define BMI088_ACCEL_SENSORTIME_2_REG               UINT8_C(0x1A)

    /**\name	Accel Interrupt status register */
#define BMI088_ACCEL_INT_STAT_1_REG                 UINT8_C(0x1D)

    /**\name	Sensor temperature MSB data register */
#define BMI088_TEMP_MSB_REG                         UINT8_C(0x22)

    /**\name	Sensor temperature LSB data register */
#define BMI088_TEMP_LSB_REG                         UINT8_C(0x23)

    /**\name    Accel Fifo Length 0 register */
#define BMI088_ACCEL_FIFO_LENGTH_0_REG              UINT8_C(0x24)

    /**\name    Accel Fifo Length 1 register */
#define BMI088_ACCEL_FIFO_LENGTH_1_REG              UINT8_C(0x25)

    /**\name    Accel Fifo data register */
#define BMI088_ACCEL_FIFO_DATA_REG                  UINT8_C(0x26)

    /**\name    Accel Internal status register */
#define BMI088_ACCEL_INTERNAL_STAT_REG              UINT8_C(0x2A)

    /**\name	Accel configuration register */
#define BMI088_ACCEL_CONF_REG                       UINT8_C(0x40)

    /**\name	Accel range setting register */
#define BMI088_ACCEL_RANGE_REG                      UINT8_C(0x41)

    /**\name    Accel Fifo Down register */
#define BMI088_ACCEL_FIFO_DOWN_REG                  UINT8_C(0x45)

    /**\name    Accel Fifo Watermark 0 register */
#define BMI088_ACCEL_FIFO_WTM_0_REG                 UINT8_C(0x46)

    /**\name    Accel Fifo Watermark 1 register */
#define BMI088_ACCEL_FIFO_WTM_1_REG                 UINT8_C(0x47)

    /**\name    Accel Fifo Config 0 register */
#define BMI088_ACCEL_FIFO_CONFIG_0_REG              UINT8_C(0x48)

    /**\name    Accel Fifo Config 1 register */
#define BMI088_ACCEL_FIFO_CONFIG_1_REG              UINT8_C(0x49)

    /**\name	Accel Interrupt pin 1 configuration register */
#define BMI088_ACCEL_INT1_IO_CONF_REG               UINT8_C(0x53)

    /**\name	Accel Interrupt pin 2 configuration register */
#define BMI088_ACCEL_INT2_IO_CONF_REG               UINT8_C(0x54)

    /**\name	Accel Interrupt map register */
#define BMI088_ACCEL_INT1_INT2_MAP_DATA_REG         UINT8_C(0x58)

    /**\name    Accel Init control register */
#define BMI088_ACCEL_INIT_CTRL_REG                  UINT8_C(0x59)

    /**\name	Accel Self test register */
#define BMI088_ACCEL_SELF_TEST_REG                  UINT8_C(0x6D)

    /**\name	Accel Power mode configuration register */
#define BMI088_ACCEL_PWR_CONF_REG                   UINT8_C(0x7C)

    /**\name	Accel Power control (switch on or off ) register */
#define BMI088_ACCEL_PWR_CTRL_REG                   UINT8_C(0x7D)

    /**\name	Accel Soft reset register */
#define BMI088_ACCEL_SOFTRESET_REG                  UINT8_C(0x7E)

    /**\name	Accel unique chip identifier */
#define BMI088_ACCEL_CHIP_ID                        UINT8_C(0x1E)

	/**\name	Accel I2C slave address */
#define BMI088_ACCEL_I2C_ADDR_PRIMARY               UINT8_C(0x18)
#define BMI088_ACCEL_I2C_ADDR_SECONDARY             UINT8_C(0x19)

    /**\name    Feature Config related Registers */
#define BMI088_ACCEL_RESERVED_5B_REG                UINT8_C(0x5B)
#define BMI088_ACCEL_RESERVED_5C_REG                UINT8_C(0x5C)
#define BMI088_ACCEL_FEATURE_CFG_REG                UINT8_C(0x5E)

    /**\name    Interrupt masks */
#define BMI088_ACCEL_DATA_READY_INT                 UINT8_C(0x80)

    /**\name	Accel Bandwidth */
#define BMI088_ACCEL_BW_OSR4                        UINT8_C(0x00)
#define BMI088_ACCEL_BW_OSR2                        UINT8_C(0x01)
#define BMI088_ACCEL_BW_NORMAL                      UINT8_C(0x02)

    /**\name	 Accel Range */
#define BMI088_ACCEL_RANGE_3G                       UINT8_C(0x00)
#define BMI088_ACCEL_RANGE_6G                       UINT8_C(0x01)
#define BMI088_ACCEL_RANGE_12G                      UINT8_C(0x02)
#define BMI088_ACCEL_RANGE_24G                      UINT8_C(0x03)

    /**\name	Accel Output data rate */
#define BMI088_ACCEL_ODR_12_5_HZ                    UINT8_C(0x05)
#define BMI088_ACCEL_ODR_25_HZ                      UINT8_C(0x06)
#define BMI088_ACCEL_ODR_50_HZ                      UINT8_C(0x07)
#define BMI088_ACCEL_ODR_100_HZ                     UINT8_C(0x08)
#define BMI088_ACCEL_ODR_200_HZ                     UINT8_C(0x09)
#define BMI088_ACCEL_ODR_400_HZ                     UINT8_C(0x0A)
#define BMI088_ACCEL_ODR_800_HZ                     UINT8_C(0x0B)
#define BMI088_ACCEL_ODR_1600_HZ                    UINT8_C(0x0C)

    /**\name    Accel Self test */
#define BMI088_ACCEL_SWITCH_OFF_SELF_TEST           UINT8_C(0x00)
#define BMI088_ACCEL_POSITIVE_SELF_TEST             UINT8_C(0x0D)
#define BMI088_ACCEL_NEGATIVE_SELF_TEST             UINT8_C(0x09)

    /**\name	Accel Power mode */
#define BMI088_ACCEL_PM_ACTIVE                      UINT8_C(0x00)
#define BMI088_ACCEL_PM_SUSPEND                     UINT8_C(0x03)

    /**\name	Accel Power control settings */
#define BMI088_ACCEL_POWER_DISABLE                  UINT8_C(0x00)
#define BMI088_ACCEL_POWER_ENABLE                   UINT8_C(0x04)

    /**\name	Accel Soft reset delay */
#define BMI088_ACCEL_SOFTRESET_DELAY                UINT8_C(1)

    /**\name    Mask definitions for ACCEL_ERR_REG register */
#define BMI088_FATAL_ERR_MASK                     UINT8_C(0x01)
#define BMI088_CMD_ERR_MASK                       UINT8_C(0x02)
#define BMI088_ERR_CODE_MASK                      UINT8_C(0x1C)

    /**\name    Position definitions for ACCEL_ERR_REG register */
#define BMI088_FATAL_ERR_POS                      UINT8_C(0)
#define BMI088_CMD_ERR_POS                        UINT8_C(1)
#define BMI088_ERR_CODE_POS                       UINT8_C(2)

    /**\name    Mask definition for FIFO BYTE COUNTER register */
#define BMI088_FIFO_BYTE_COUNTER_MSB_MASK         UINT8_C(0x3F)

    /**\name    Position definition for FIFO BYTE COUNTER register */
#define BMI088_FIFO_BYTE_COUNTER_MSB_POS          UINT8_C(0)

    /**\name    Mask definitions for odr, bandwidth and range */
#define BMI088_ACCEL_ODR_MASK                       UINT8_C(0x0F)
#define BMI088_ACCEL_BW_MASK                        UINT8_C(0x70)
#define BMI088_ACCEL_RANGE_MASK                     UINT8_C(0x03)

    /**\name    Position definitions for odr, bandwidth and range */
#define BMI088_ACCEL_ODR_POS                        UINT8_C(0)
#define BMI088_ACCEL_BW_POS                         UINT8_C(4)
#define BMI088_ACCEL_RANGE_POS                      UINT8_C(0)

    /**\name    Mask definitions for FIFO_DOWNS register */
#define BMI088_ACCEL_FIFO_FILT_DOWN_MASK            UINT8_C(0x70)
#define BMI088_ACCEL_FIFO_FILT_DATA_MASK            UINT8_C(0x80)

    /**\name    Position definitions for FIFO_DOWNS register */
#define BMI088_ACCEL_FIFO_FILT_DOWN_POS             UINT8_C(4)
#define BMI088_ACCEL_FIFO_FILT_DATA_POS             UINT8_C(7)

    /**\name    Mask definitions for INT1_IO_CONF register */
#define BMI088_ACCEL_INT1_LVL_MASK                  UINT8_C(0x02)
#define BMI088_ACCEL_INT1_OD_MASK                   UINT8_C(0x04)
#define BMI088_ACCEL_INT1_IO_MASK                   UINT8_C(0x08)

    /**\name    Position definitions for INT1_IO_CONF register */
#define BMI088_ACCEL_INT1_LVL_POS                   UINT8_C(1)
#define BMI088_ACCEL_INT1_OD_POS                    UINT8_C(2)
#define BMI088_ACCEL_INT1_IO_POS                    UINT8_C(3)

    /**\name    Mask definitions for INT2_IO_CONF register */
#define BMI088_ACCEL_INT2_LVL_MASK                  UINT8_C(0x02)
#define BMI088_ACCEL_INT2_OD_MASK                   UINT8_C(0x04)
#define BMI088_ACCEL_INT2_IO_MASK                   UINT8_C(0x08)

    /**\name    Position definitions for INT2_IO_CONF register */
#define BMI088_ACCEL_INT2_LVL_POS                   UINT8_C(1)
#define BMI088_ACCEL_INT2_OD_POS                    UINT8_C(2)
#define BMI088_ACCEL_INT2_IO_POS                    UINT8_C(3)

    /**\name    Mask definitions for INT1_INT2_MAP_DATA register */
#define BMI088_ACCEL_INT1_FIFO_FULL_MASK            UINT8_C(0x01)
#define BMI088_ACCEL_INT1_FIFO_WM_MASK              UINT8_C(0x02)
#define BMI088_ACCEL_INT1_DRDY_MASK                 UINT8_C(0x04)
#define BMI088_ACCEL_INT2_FIFO_FULL_MASK            UINT8_C(0x10)
#define BMI088_ACCEL_INT2_FIFO_WM_MASK              UINT8_C(0x20)
#define BMI088_ACCEL_INT2_DRDY_MASK                 UINT8_C(0x40)

    /**\name    Position definitions for INT1_INT2_MAP_DATA register */
#define BMI088_ACCEL_INT1_FIFO_FULL_POS             UINT8_C(0)
#define BMI088_ACCEL_INT1_FIFO_WM_POS               UINT8_C(1)
#define BMI088_ACCEL_INT1_DRDY_POS                  UINT8_C(2)
#define BMI088_ACCEL_INT2_FIFO_FULL_POS             UINT8_C(4)
#define BMI088_ACCEL_INT2_FIFO_WM_POS               UINT8_C(5)
#define BMI088_ACCEL_INT2_DRDY_POS                  UINT8_C(6)

    /**\name    Maximum Accel Fifo filter value */
#define BMI088_MAX_VALUE_FIFO_FILTER              UINT8_C(1)

    /**\name    Maximum Accel Fifo down value */
#define BMI088_MAX_VALUE_FIFO_DOWN                UINT8_C(7)

    /**\name    Asic Initialization value */
#define BMI088_ASIC_INITIALIZED                   UINT8_C(0x01)

    /*************************** BMI088 Gyroscope Macros *****************************/

    /** Register map */
    /* Gyro registers */

    /**\name    Gyro Chip Id register */
#define BMI088_GYRO_CHIP_ID_REG                    UINT8_C(0x00)

    /**\name    Gyro X LSB data register */
#define BMI088_GYRO_X_LSB_REG                      UINT8_C(0x02)

    /**\name    Gyro X MSB data register */
#define BMI088_GYRO_X_MSB_REG                      UINT8_C(0x03)

    /**\name    Gyro Y LSB data register */
#define BMI088_GYRO_Y_LSB_REG                      UINT8_C(0x04)

    /**\name    Gyro Y MSB data register */
#define BMI088_GYRO_Y_MSB_REG                      UINT8_C(0x05)

    /**\name    Gyro Z LSB data register */
#define BMI088_GYRO_Z_LSB_REG                      UINT8_C(0x06)

    /**\name    Gyro Z MSB data register */
#define BMI088_GYRO_Z_MSB_REG                      UINT8_C(0x07)

    /**\name    Gyro Interrupt status register */
#define BMI088_GYRO_INT_STAT_1_REG                 UINT8_C(0x0A)

    /**\name    Gyro  Fifo status register */
#define BMI088_GYRO_FIFO_STAT_REG                  UINT8_C(0x0E)

    /**\name    Gyro Range register */
#define BMI088_GYRO_RANGE_REG                      UINT8_C(0x0F)

    /**\name    Gyro Bandwidth register */
#define BMI088_GYRO_BANDWIDTH_REG                  UINT8_C(0x10)

    /**\name    Gyro Power register */
#define BMI088_GYRO_LPM1_REG                       UINT8_C(0x11)

    /**\name    Gyro Soft reset register */
#define BMI088_GYRO_SOFTRESET_REG                  UINT8_C(0x14)

    /**\name    Gyro Interrupt control register */
#define BMI088_GYRO_INT_CTRL_REG                   UINT8_C(0x15)

    /**\name    Gyro Interrupt Pin configuration register */
#define BMI088_GYRO_INT3_INT4_IO_CONF_REG          UINT8_C(0x16)

    /**\name    Gyro Interrupt Map register */
#define BMI088_GYRO_INT3_INT4_IO_MAP_REG           UINT8_C(0x18)

    /**\name    Gyro Interrupt Enable Register */
#define BMI088_GYRO_INT_EN_REG                     UINT8_C(0x1E)

    /**\name    Gyro Self test register */
#define BMI088_GYRO_SELF_TEST_REG                  UINT8_C(0x3C)

    /**\name    Gyro Fifo Config 0 register */
#define BMI088_GYRO_FIFO_CONFIG_0_REG              UINT8_C(0x3D)

    /**\name    Gyro Fifo Config 1 register */
#define BMI088_GYRO_FIFO_CONFIG_1_REG              UINT8_C(0x3E)

    /**\name    Gyro Fifo Data register */
#define BMI088_GYRO_FIFO_DATA_REG                  UINT8_C(0x3F)

    /**\name	Gyro unique chip identifier */
#define BMI088_GYRO_CHIP_ID                        UINT8_C(0x0F)

	/**\name	Gyro I2C slave address */
#define BMI088_GYRO_I2C_ADDR_PRIMARY               UINT8_C(0x68)
#define BMI088_GYRO_I2C_ADDR_SECONDARY             UINT8_C(0x69)

    /**\name	Gyro Range */
#define BMI088_GYRO_RANGE_2000_DPS                 UINT8_C(0x00)
#define BMI088_GYRO_RANGE_1000_DPS                 UINT8_C(0x01)
#define BMI088_GYRO_RANGE_500_DPS                  UINT8_C(0x02)
#define BMI088_GYRO_RANGE_250_DPS                  UINT8_C(0x03)
#define BMI088_GYRO_RANGE_125_DPS                  UINT8_C(0x04)

    /**\name	Gyro Output data rate and bandwidth */
#define BMI088_GYRO_BW_532_ODR_2000_HZ             UINT8_C(0x00)
#define BMI088_GYRO_BW_230_ODR_2000_HZ             UINT8_C(0x01)
#define BMI088_GYRO_BW_116_ODR_1000_HZ             UINT8_C(0x02)
#define BMI088_GYRO_BW_47_ODR_400_HZ               UINT8_C(0x03)
#define BMI088_GYRO_BW_23_ODR_200_HZ               UINT8_C(0x04)
#define BMI088_GYRO_BW_12_ODR_100_HZ               UINT8_C(0x05)
#define BMI088_GYRO_BW_64_ODR_200_HZ               UINT8_C(0x06)
#define BMI088_GYRO_BW_32_ODR_100_HZ               UINT8_C(0x07)
#define BMI088_GYRO_ODR_RESET_VAL                  UINT8_C(0x80)

    /**\name	Gyro Power mode */
#define BMI088_GYRO_PM_NORMAL                      UINT8_C(0x00)
#define BMI088_GYRO_PM_DEEP_SUSPEND                UINT8_C(0x20)
#define BMI088_GYRO_PM_SUSPEND                     UINT8_C(0x80)

    /**\name	Gyro data ready interrupt enable value */
#define BMI088_GYRO_DRDY_INT_ENABLE_VAL            UINT8_C(0x80)

    /**\name	Gyro data ready map values */
#define BMI088_GYRO_MAP_DRDY_TO_INT3               UINT8_C(0x01)
#define BMI088_GYRO_MAP_DRDY_TO_INT4               UINT8_C(0x80)
#define BMI088_GYRO_MAP_DRDY_TO_BOTH_INT3_INT4     UINT8_C(0x81)

    /**\name    Gyro Fifo Operating Mode selection values */
#define BMI088_GYRO_BYPASS_OP_MODE                 UINT8_C(0x00)
#define BMI088_GYRO_FIFO_OP_MODE                   UINT8_C(0x01)
#define BMI088_GYRO_STREAM_OP_MODE                 UINT8_C(0x02)

    /**\name    Gyro Fifo data select values */
#define BMI088_GYRO_ALL_INT_DATA                   UINT8_C(0x00)//(X,Y,Z plus INT_status 0 AND 1 )
#define BMI088_GYRO_X_DATA                         UINT8_C(0x01)
#define BMI088_GYRO_Y_DATA                         UINT8_C(0x02)
#define BMI088_GYRO_Z_DATA                         UINT8_C(0x03)

    /**\name	Gyro Soft reset delay */
#define BMI088_GYRO_SOFTRESET_DELAY                UINT8_C(30)

    /**\name    Gyro Fifo watermark mark maximum value */
#define BMI088_GYRO_FIFO_WM_MAX                    UINT8_C(0x7F)

    /**\name    Mask definitions for BMI088_GYRO_FIFO_STAT_REG register */
#define BMI088_GYRO_FIFO_COUNTER_MASK              UINT8_C(0x7F)
#define BMI088_GYRO_FIFO_OVERRUN_MASK              UINT8_C(0x80)

    /**\name    Position definitions for BMI088_GYRO_FIFO_STAT_REG register */
#define BMI088_GYRO_FIFO_COUNTER_POS               UINT8_C(0)
#define BMI088_GYRO_FIFO_OVERRUN_POS               UINT8_C(7)

    /**\name    Mask definitions for BMI088_GYRO_INT_CTRL_REG register */
#define BMI088_GYRO_FIFO_EN_MASK                   UINT8_C(0x40)
#define BMI088_GYRO_DATA_EN_MASK                   UINT8_C(0x80)

    /**\name    Position definitions for BMI088_GYRO_INT_CTRL_REG register */
#define BMI088_GYRO_FIFO_EN_POS                    UINT8_C(6)
#define BMI088_GYRO_DATA_EN_POS                    UINT8_C(7)

    /**\name    Mask definitions for BMI088_GYRO_INT3_INT4_IO_CONF_REG register */
#define BMI088_GYRO_INT3_LVL_MASK                  UINT8_C(0x01)
#define BMI088_GYRO_INT3_OD_MASK                   UINT8_C(0x02)
#define BMI088_GYRO_INT4_LVL_MASK                  UINT8_C(0x04)
#define BMI088_GYRO_INT4_OD_MASK                   UINT8_C(0x08)

    /**\name    Position definitions for BMI088_GYRO_INT3_INT4_IO_CONF_REG register */
#define BMI088_GYRO_INT3_LVL_POS                   UINT8_C(0)
#define BMI088_GYRO_INT3_OD_POS                    UINT8_C(1)
#define BMI088_GYRO_INT4_LVL_POS                   UINT8_C(2)
#define BMI088_GYRO_INT4_OD_POS                    UINT8_C(3)

    /**\name    Mask definitions for BMI088_GYRO_INT3_INT4_IO_MAP_REG register */
#define BMI088_GYRO_INT1_FIFO_MASK                 UINT8_C(0x04)
#define BMI088_GYRO_INT2_FIFO_MASK                 UINT8_C(0x20)

    /**\name    Position definitions for BMI088_GYRO_INT3_INT4_IO_MAP_REG register */
#define BMI088_GYRO_INT1_FIFO_POS                  UINT8_C(2)
#define BMI088_GYRO_INT2_FIFO_POS                  UINT8_C(5)

    /**\name    Mask definitions for BMI088_GYRO_INT_CTRL_REG register */
#define BMI088_GYRO_INT_EN_MASK                    UINT8_C(0x80)

    /**\name    Position definitions for BMI088_GYRO_INT_CTRL_REG register */
#define BMI088_GYRO_INT_EN_POS                     UINT8_C(7)

    /**\name    Mask definitions for GYRO_SELF_TEST register */
#define BMI088_GYRO_SELF_TEST_EN_MASK              UINT8_C(0x01)
#define BMI088_GYRO_SELF_TEST_RDY_MASK             UINT8_C(0x02)
#define BMI088_GYRO_SELF_TEST_RESULT_MASK          UINT8_C(0x04)
#define BMI088_GYRO_SELF_TEST_FUNCTION_MASK        UINT8_C(0x08)

    /**\name    Position definitions for GYRO_SELF_TEST register */
#define BMI088_GYRO_SELF_TEST_EN_POS               UINT8_C(0)
#define BMI088_GYRO_SELF_TEST_RDY_POS              UINT8_C(1)
#define BMI088_GYRO_SELF_TEST_RESULT_POS           UINT8_C(2)
#define BMI088_GYRO_SELF_TEST_FUNCTION_POS         UINT8_C(3)

    /**\name    Mask definitions for BMI088_GYRO_FIFO_CONFIG_0_REG register */
#define BMI088_GYRO_FIFO_WM_MASK                   UINT8_C(0x7F)
#define BMI088_GYRO_FIFO_TAG_MASK                  UINT8_C(0x80)

    /**\name    Position definitions for BMI088_GYRO_FIFO_CONFIG_0_REG register */
#define BMI088_GYRO_FIFO_WM_POS                    UINT8_C(0)
#define BMI088_GYRO_FIFO_TAG_POS                   UINT8_C(7)

    /**\name    Mask definitions for BMI088_GYRO_FIFO_CONFIG_1_REG register */
#define BMI088_GYRO_FIFO_DATA_SELECT_MASK          UINT8_C(0x03)
#define BMI088_GYRO_FIFO_MODE_MASK                 UINT8_C(0xC0)

    /**\name    Position definitions for BMI088_GYRO_FIFO_CONFIG_1_REG register */
#define BMI088_GYRO_FIFO_DATA_SELECT_POS           UINT8_C(0)
#define BMI088_GYRO_FIFO_MODE_POS                  UINT8_C(6)

    /*************************** Common Macros for both Accel and Gyro *****************************/

    /**\name    Interface settings */
#define BMI088_SPI_INTF                           UINT8_C(1)
#define BMI088_I2C_INTF                           UINT8_C(0)

    /**\name    SPI read/write mask to configure address */
#define BMI088_SPI_RD_MASK                        UINT8_C(0x80)
#define BMI088_SPI_WR_MASK                        UINT8_C(0x7F)

    /**\name    Error code definitions */
#define BMI088_OK                                 UINT16_C(0)
#define BMI088_E_NULL_PTR                         UINT16_C(1)
#define BMI088_E_COM_FAIL                         UINT16_C(1 << 2)
#define BMI088_E_DEV_NOT_FOUND                    UINT16_C(1 << 3)
#define BMI088_E_OUT_OF_RANGE                     UINT16_C(1 << 4)
#define BMI088_E_INVALID_INPUT                    UINT16_C(1 << 5)
#define BMI088_E_SELF_TEST_FAIL                   UINT16_C(1 << 6)
#define BMI088_E_CONFIG_STREAM_ERROR              UINT16_C(1 << 7)

    /***\name    Soft reset Value */
#define BMI088_SOFT_RESET_VAL                     UINT8_C(0xB6)

    /**\name	Enable/disable values */
#define BMI088_ENABLE                             UINT8_C(0x01)
#define BMI088_DISABLE                            UINT8_C(0x00)

    /**\name	Constant values macros */
#define BMI088_ONE                                UINT8_C(1)
#define BMI088_TWO                                UINT8_C(2)
#define BMI088_THREE                              UINT8_C(3)
#define BMI088_FOUR                               UINT8_C(4)
#define BMI088_FIVE                               UINT8_C(5)
#define BMI088_SIX                                UINT8_C(6)
#define BMI088_SEVEN                              UINT8_C(7)
#define BMI088_EIGHT                              UINT8_C(8)
#define BMI088_NINE                               UINT8_C(9)
#define BMI088_TEN                                UINT8_C(10)
#define BMI088_ELEVEN                             UINT8_C(11)
#define BMI088_TWELVE                             UINT8_C(12)
#define BMI088_SIXTEEN                            UINT8_C(16)
#define BMI088_FIFTY                              UINT8_C(50)
#define BMI088_HUNDRED                            UINT8_C(100)
#define BMI088_ONE_FIFTY                          UINT8_C(150)
#define BMI088_CONFIG_STREAM_SIZE                 UINT16_C(6144)

    /**\name    Sensor time array parameter definitions */
#define BMI088_SENSOR_TIME_MSB_BYTE               UINT8_C(2)
#define BMI088_SENSOR_TIME_XLSB_BYTE              UINT8_C(1)
#define BMI088_SENSOR_TIME_LSB_BYTE               UINT8_C(0)

    /**\name    FIFO header data definitions    */
#define FIFO_HEAD_A                               UINT8_C(0x84)
#define FIFO_HEAD_G                               UINT8_C(0x90)
#define FIFO_HEAD_G_A                             UINT8_C(0x94)
#define FIFO_HEAD_SENSOR_TIME                     UINT8_C(0x44)
#define FIFO_HEAD_INPUT_CONFIG                    UINT8_C(0x48)
#define FIFO_HEAD_SKIP_FRAME                      UINT8_C(0x40)
#define FIFO_HEAD_OVER_READ_MSB                   UINT8_C(0x80)
#define FIFO_HEAD_SAMPLE_DROP                     UINT8_C(0x50)

    /**\name    FIFO headerless mode data enable definitions   */
#define BMI088_FIFO_A_ENABLE                      UINT8_C(0x40)

    /**\name    FIFO configuration selection    */
#define BMI088_FIFO_STOP_ON_FULL                  UINT8_C(0x01)
#define BMI088_FIFO_TIME                          UINT8_C(0x02)
#define BMI088_FIFO_TAG_INTR2                     UINT8_C(0x04)
#define BMI088_FIFO_TAG_INTR1                     UINT8_C(0x08)
#define BMI088_FIFO_HEADER                        UINT8_C(0x10)
#define BMI088_FIFO_ACCEL                         UINT8_C(0x40)
#define BMI088_FIFO_ALL                           UINT8_C(0x7F)
#define BMI088_FIFO_CONFIG_0_MASK                 UINT8_C(0x03)
#define BMI088_FIFO_CONFIG_1_MASK                 UINT8_C(0xFC)

    /**\name    FIFO frame count definition     */
#define FIFO_LSB_CONFIG_CHECK                     UINT8_C(0x00)
#define FIFO_MSB_CONFIG_CHECK                     UINT8_C(0x80)
#define BMI088_FIFO_TAG_INTR_MASK                 UINT8_C(0xFC)

    /**\name    FIFO dropped frame definition    */
#define ACCEL_FIFO_DROP                           UINT8_C(0x01)

    /**\name    FIFO sensor time length definitions*/
#define BMI088_SENSOR_TIME_LENGTH                 UINT8_C(3)

    /**\name    FIFO length definitions */
#define BMI088_FIFO_A_LENGTH                      UINT8_C(6)
#define BMI088_FIFO_G_ALL_DATA_LENGTH             UINT8_C(8)
#define BMI088_FIFO_G_X_LENGTH                    UINT8_C(2)
#define BMI088_FIFO_G_Y_LENGTH                    UINT8_C(2)
#define BMI088_FIFO_G_Z_LENGTH                    UINT8_C(2)

    /**\name    Self test result indicators */
#define BMI088_SELFTEST_PASS                      UINT8_C(0)
#define BMI088_SELFTEST_FAIL                      INT8_C(-1)

    /**\name    Sensor bit resolution   */
#define BMI088_16_BIT_RESOLUTION                  UINT8_C(16)

    /**\name    Absolute value */
#ifndef ABS
#define ABS(a)      ((a) > 0 ? (a) : -(a))
#endif

    /**\name    Utility Macros  */
#define BMI088_SET_LOW_BYTE                       UINT16_C(0x00FF)
#define BMI088_SET_HIGH_BYTE                      UINT16_C(0xFF00)
#define BMI088_SET_LOW_NIBBLE                     UINT8_C(0x0F)

    /**\name     Macro definition for register bit slicing, read access */
#define BMI088_GET_BITSLICE(regvar, bitname)\
                        (regvar & bitname##_MASK) >> bitname##_POS

    /**\name     Macro definition for register bit slicing, write access */
#define BMI088_SET_BITSLICE(regvar, bitname, val)\
                  (regvar & ~bitname##_MASK) | ((val<<bitname##_POS)&bitname##_MASK)

    /**\name     Macro definition for difference between 2 values */
#define BMI088_GET_DIFF(x, y) ((x) - (y))

    /**\name     Macro definition to get LSB of 16 bit variable */
#define BMI088_GET_LSB(var)   (uint8_t)(var & BMI088_SET_LOW_BYTE)

    /**\name     Macro definition to get MSB of 16 bit variable */
#define BMI088_GET_MSB(var)   (uint8_t)((var & BMI088_SET_HIGH_BYTE) >> 8)

    /*************************** Data structures *****************************/

    /**\name    Typedef definitions */
    /*!
     * @brief Bus communication function pointer which should be mapped to
     * the platform specific read and write functions of the user
     *
     *  @note : dev_addr is used for I2C read/write operations only.
     *  		For SPI read/write operations this is dummy variable.
     */
    typedef int8_t (*bmi088_com_fptr_t)(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);

    /**\name    delay function pointer */
    typedef void (*bmi088_delay_fptr_t)(uint32_t period);

    /**\name    Structure Definitions */

    /*!
     *  @brief Sensor XYZ data structure
     */
    struct bmi088_sensor_data
    {
        /*! X-axis sensor data */
        int16_t x;
        /*! Y-axis sensor data */
        int16_t y;
        /*! Z-axis sensor data */
        int16_t z;
    };

    /*!
     *  @brief Sensor configuration structure
     */
    struct bmi088_cfg
    {
        /*! power mode */
        uint8_t power;
        /*! range */
        uint8_t range;
        /*! bandwidth */
        uint8_t bw;
        /*! output data rate */
        uint8_t odr;
    };

    /*!
     *  @brief Error Status structure
     */
    struct bmi088_err_reg
    {
        /*! Indicates fatal error */
        uint8_t fatal_err;
        /*! Indicates command error */
        uint8_t cmd_err;
        /*! Indicates error code */
        uint8_t err_code;
    };

    /*!
     *  @brief Enum to select accelerometer Interrupt pins
     */
    enum bmi088_accel_int_channel
    {
        BMI088_INT_CHANNEL_1, /* interrupt Channel 1 for accel sensor*/
        BMI088_INT_CHANNEL_2 /* interrupt Channel 2 */
    };

    /*!
     *  @brief Enum to select gyroscope Interrupt pins
     */
    enum bmi088_gyro_int_channel
    {
        BMI088_INT_CHANNEL_3, /* interrupt Channel 3 for gyro sensor*/
        BMI088_INT_CHANNEL_4, /* interrupt Channel 4 */
        BMI088_INT_CHANNEL_BOTH /* Both interrupt Channel 3 and 4 */
    };

    /*!
     *  @brief Enum to select accelerometer interrupts
     */
    enum bmi088_accel_int_types
    {
        BMI088_ACCEL_DATA_RDY_INT, /* Accel data ready interrupt */
    };

    /*!
     *  @brief Enum to select gyroscope interrupts
     */
    enum bmi088_gyro_int_types
    {
        BMI088_GYRO_DATA_RDY_INT, /* Gyro data ready interrupt */
    };

    /*!
     *  @brief Interrupt pin configuration structure
     */
    struct bmi088_int_pin_cfg
    {
#ifdef LITTLE_ENDIAN
        /*! Enable interrupt pin -> 0 - disable, 1 - enable */
        uint8_t enable_int_pin :1;
        /*! 0 - push-pull, 1 - open drain */
        uint8_t output_mode :1;
        /*! 0 - active low, 1 - active high level */
        uint8_t lvl :1;
#else
        /*! 0 - active low, 1 - active high level */
        uint8_t lvl : 1;
        /*! 0 - push-pull, 1 - open drain */
        uint8_t output_mode : 1;
        /*! Enable interrupt pin -> 0 - disable, 1 - enable */
        uint8_t enable_int_pin : 1;
#endif
    };

    /*!
     *  @brief Interrupt Configuration structure
     */
    struct bmi088_int_cfg
    {
        /*! Accel Interrupt channel */
        enum bmi088_accel_int_channel accel_int_channel;
        /*! Gyro Interrupt channel */
        enum bmi088_gyro_int_channel gyro_int_channel;
        /*! Select Accel Interrupt */
        enum bmi088_accel_int_types accel_int_type;
        /*! Select Gyro Interrupt */
        enum bmi088_gyro_int_types gyro_int_type;
        /*! Structure to configure accel interrupt pins */
        struct bmi088_int_pin_cfg accel_int_pin_cfg;
        /*! Structure to configure gyro interrupt pin 3 */
        struct bmi088_int_pin_cfg gyro_int_pin_3_cfg;
        /*! Structure to configure gyro interrupt pin 4 */
        struct bmi088_int_pin_cfg gyro_int_pin_4_cfg;
    };

    /*!
     *  @brief This structure holds the information for usage of
     *  FIFO by the user.
     */
    struct bmi088_fifo_frame
    {
        /*! Data buffer of user defined length is to be mapped here */
        uint8_t *data;
        /*! Number of bytes of FIFO to be read as specified by the user */
        uint16_t length;
        /*! Length of each frame stored in fifo */
        uint16_t frame_length;
        /*! Enabling the FIFO header to stream in header mode */
        uint8_t fifo_header_enable;
        /*! Sensor data enabled status */
        uint8_t fifo_data_enable;
        /*! Will be equal to length when no more frames are there to parse */
        uint16_t byte_start_idx;
        /*! Value of FIFO sensor time */
        uint32_t sensor_time;
        /*! Value of Skipped frame counts */
        uint8_t skipped_frame_count;
        /*! Value of dropped frame count */
        uint8_t dropped_frame_count;
    };

    /*!
     *  @brief
     *  This structure holds all relevant information about BMI088
     */
    struct bmi088_dev
    {
        /*! Accel chip Id */
        uint8_t accel_chip_id;
        /*! Gyro chip Id */
        uint8_t gyro_chip_id;
        /*! Accel device Id */
        uint8_t accel_id;
        /*! Gyro device Id */
        uint8_t gyro_id;
        /*! 0 - I2C , 1 - SPI Interface */
        uint8_t interface;
        /*! Decide SPI or I2C read mechanism */
        uint8_t dummy_byte;
        /*! Structure to configure accel sensor  */
        struct bmi088_cfg accel_cfg;
        /*! Structure to configure gyro sensor  */
        struct bmi088_cfg gyro_cfg;
        /*! Accel FIFO related configurations */
        struct bmi088_fifo_frame *accel_fifo;
        /*! Gyro FIFO related configurations */
        struct bmi088_fifo_frame *gyro_fifo;
        /*! Config stream data buffer address will be assigned*/
        const uint8_t *config_file_ptr;
        /*! Max read/write length (maximum supported length is 32).
         To be set by the user */
        uint8_t read_write_len;
        /*! Read function pointer */
        bmi088_com_fptr_t read;
        /*! Write function pointer */
        bmi088_com_fptr_t write;
        /*! Delay function pointer */
        bmi088_delay_fptr_t delay_ms;
    };

/*************************** C++ guard macro *****************************/
#ifdef __cplusplus
}
#endif

#endif /* BMI088_DEFS_H_ */
