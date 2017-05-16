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
 * @file	bmi055_defs.h
 * @date	19 Apr, 2017
 * @version	1.1.0
 * @brief	Sensor driver for BMI055 sensor *
 */

#ifndef BMI055_DEFS_H_
#define BMI055_DEFS_H_

/******************************************************************************/
/*!			Header includes	*/
/******************************************************************************/
#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#include <stddef.h>
#endif

/******************************************************************************/
/*!			C++ guard macro	*/
/******************************************************************************/
#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************************************/
/*!			Common macros	*/
/******************************************************************************/
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

/******************************************************************************/
/*!			Sensor macros	*/
/******************************************************************************/
/* Test for an endian machine */
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define LITTLE_ENDIAN   1
#elif __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
#define BIG_ENDIAN   1
#else
#error "Code does not support Endian format of the processor"
#endif

/******************************************************************************/
/*!         Utility macros   */
/******************************************************************************/
#define BMI055_SET_BITS(reg_data, bitname, data)	((reg_data & ~(bitname##_MASK)) | \
							((data << bitname##_POS) & bitname##_MASK))

#define BMI055_GET_BITS(reg_data, bitname)		((reg_data & (bitname##_MASK)) >> \
							(bitname##_POS))

#define BMI055_SET_BIT_POS0(reg_data, bitname, data)	((reg_data & ~(bitname##_MASK)) | \
							(data & bitname##_MASK))

#define BMI055_GET_BIT_POS0(reg_data, bitname)		(reg_data & (bitname##_MASK))


/******************************************************************************/
/*!         Macros for defining version number   */
/******************************************************************************/
#define BMI055_VER_MAJOR         UINT8_C(1)
#define BMI055_VER_MINOR         UINT8_C(1)
#define BMI055_VER_PATCH         UINT8_C(0)

/******************************************************************************/
/*!         Macros for parameter settings   */
/******************************************************************************/
/*! Bandwidth settings */
#define BANDWIDTH_CONFIG	UINT8_C(0x01)
/*! Range settings */
#define RANGE_CONFIG		UINT8_C(0x02)
/*! Range plus bandwidth settings */
#define CONFIG_ALL		UINT8_C(BANDWIDTH_CONFIG | RANGE_CONFIG)

/******************************************************************************/
/*!         Macros for defining enable or disable */
/******************************************************************************/
#define BMI055_ENABLE                   UINT8_C(0x01)
#define BMI055_DISABLE                  UINT8_C(0x00)
#define	CODE_UNDER_MODIFICATION		UINT8_C(0x00)
#define	BMI055_FILTERED			UINT8_C(0x00)
#define	BMI055_UNFILTERED	        UINT8_C(0x01)

/******************************************************************************/
/*!			BMI055 Accelerometer Macros	*/
/******************************************************************************/
/*! Macros to define the types of interrupts */
/* Slope/ Any-motion interrupt */
#define BMI055_ACC_SLOPE_INT			UINT8_C(0x00)
/* double tap interrupt */
#define BMI055_ACC_DOUBLE_TAP_INT		UINT8_C(0x01)
/* single tap interrupt */
#define BMI055_ACC_SINGLE_TAP_INT		UINT8_C(0x02)
/* orientation interrupt */
#define BMI055_ACC_ORIENT_INT			UINT8_C(0x03)
/* flat interrupt */
#define BMI055_ACC_FLAT_INT			UINT8_C(0x04)
/* low-g interrupt */
#define BMI055_ACC_LOW_G_INT			UINT8_C(0x05)
/* high-g interrupt */
#define BMI055_ACC_HIGH_G_INT			UINT8_C(0x06)
/* slow/ no-motion interrupt */
#define BMI055_ACC_SLOW_NO_MOTION_INT		UINT8_C(0x07)
/* New data interrupt */
#define BMI055_ACC_NEW_DATA_INT			UINT8_C(0x08)
/* All accelerometer interrupts */
#define BMI055_ALL_ACCEL_INT			UINT8_C(0x09)

/*! Macros to define the channel number of interrupts */
/* Interrupt Channel 1 for accelerometer sensor */
#define BMI055_INT_CHANNEL_1                       UINT8_C(0x00)
/* Interrupt Channel 2 for accelerometer sensor */
#define BMI055_INT_CHANNEL_2                       UINT8_C(0x01)

/*! Register map of accelerometer registers */
#define BMI055_ACC_CHIP_ID_ADDR                    UINT8_C(0x00)
#define BMI055_ACC_X_L_ADDR                        UINT8_C(0x02)
#define BMI055_ACC_X_H_ADDR                        UINT8_C(0x03)
#define BMI055_ACC_Y_L_ADDR                        UINT8_C(0x04)
#define BMI055_ACC_Y_H_ADDR                        UINT8_C(0x05)
#define BMI055_ACC_Z_L_ADDR                        UINT8_C(0x06)
#define BMI055_ACC_Z_H_ADDR                        UINT8_C(0x07)
#define BMI055_ACC_TEMP_ADDR                       UINT8_C(0x08)
#define BMI055_ACC_INT_STATUS_0_ADDR               UINT8_C(0x09)
#define BMI055_ACC_INT_STATUS_1_ADDR               UINT8_C(0x0A)
#define BMI055_ACC_INT_STATUS_2_ADDR               UINT8_C(0x0B)
#define BMI055_ACC_INT_STATUS_3_ADDR               UINT8_C(0x0C)
#define BMI055_ACC_FIFO_STATUS_ADDR                UINT8_C(0x0E)
#define BMI055_ACC_RANGE_ADDR                      UINT8_C(0x0F)
#define BMI055_ACC_BW_ADDR                         UINT8_C(0x10)
#define BMI055_ACC_PMU_LPW_ADDR                    UINT8_C(0x11)
#define BMI055_ACC_PMU_LOW_POWER_ADDR              UINT8_C(0x12)
#define BMI055_ACC_DATA_CTRL_ADDR                  UINT8_C(0x13)
#define BMI055_ACC_SOFTRESET_ADDR                  UINT8_C(0x14)
#define BMI055_ACC_INT_EN_0_ADDR                   UINT8_C(0x16)
#define BMI055_ACC_INT_EN_1_ADDR                   UINT8_C(0x17)
#define BMI055_ACC_INT_EN_2_ADDR                   UINT8_C(0x18)
#define BMI055_ACC_INT_MAP_0_ADDR                  UINT8_C(0x19)
#define BMI055_ACC_INT_MAP_1_ADDR                  UINT8_C(0x1A)
#define BMI055_ACC_INT_MAP_2_ADDR                  UINT8_C(0x1B)
#define BMI055_ACC_INT_SRC_ADDR                    UINT8_C(0x1E)
#define BMI055_ACC_INT_OUT_CTRL_ADDR               UINT8_C(0x20)
#define BMI055_ACC_INT_LATCH_ADDR                  UINT8_C(0x21)
#define BMI055_ACC_INT_LH_0_ADDR                   UINT8_C(0x22)
#define BMI055_ACC_INT_LH_1_ADDR                   UINT8_C(0x23)
#define BMI055_ACC_INT_LH_2_ADDR                   UINT8_C(0x24)
#define BMI055_ACC_INT_LH_3_ADDR                   UINT8_C(0x25)
#define BMI055_ACC_INT_LH_4_ADDR                   UINT8_C(0x26)
#define BMI055_ACC_INT_MOT_0_ADDR                  UINT8_C(0x27)
#define BMI055_ACC_INT_MOT_1_ADDR                  UINT8_C(0x28)
#define BMI055_ACC_INT_MOT_2_ADDR                  UINT8_C(0x29)
#define BMI055_ACC_INT_TAP_0_ADDR                  UINT8_C(0x2A)
#define BMI055_ACC_INT_TAP_1_ADDR                  UINT8_C(0x2B)
#define BMI055_ACC_INT_ORIENT_0_ADDR               UINT8_C(0x2C)
#define BMI055_ACC_INT_ORIENT_1_ADDR               UINT8_C(0x2D)
#define BMI055_ACC_INT_FLAT_0_ADDR                 UINT8_C(0x2E)
#define BMI055_ACC_INT_FLAT_1_ADDR                 UINT8_C(0x2F)
#define BMI055_ACC_FIFO_CONFIG_0_ADDR              UINT8_C(0x30)
#define BMI055_ACC_SELFTEST_ADDR                   UINT8_C(0x32)
#define BMI055_ACC_EEPROM_CTRL_ADDR                UINT8_C(0x33)
#define BMI055_ACC_SERIAL_CTRL_ADDR                UINT8_C(0x34)
#define BMI055_ACC_OFFSET_CTRL_ADDR                UINT8_C(0x36)
#define BMI055_ACC_OFC_SETTING_ADDR                UINT8_C(0x37)
#define BMI055_ACC_OFFSET_X_ADDR                   UINT8_C(0x38)
#define BMI055_ACC_OFFSET_Y_ADDR                   UINT8_C(0x39)
#define BMI055_ACC_OFFSET_Z_ADDR                   UINT8_C(0x3A)
#define BMI055_ACC_TRIM_GPO_ADDR                   UINT8_C(0x3B)
#define BMI055_ACC_TRIM_GP1_ADDR                   UINT8_C(0x3C)
#define BMI055_ACC_FIFO_CONFIG_1_ADDR              UINT8_C(0x3E)
#define BMI055_ACC_FIFO_DATA_ADDR                  UINT8_C(0x3F)

/*! Mask definitions for range, bandwidth and power */
#define BMI055_ACC_RANGE_MASK                       UINT8_C(0x0F)
#define BMI055_ACC_BW_MASK                          UINT8_C(0x1F)
#define BMI055_ACC_POWER_MASK                       UINT8_C(0xE0)

/*! Bit positions for range, bandwidth and power */
#define BMI055_ACC_POWER_POS                       UINT8_C(0x05)

/*! Mask definitions for INT_EN 0 registers */
#define BMI055_ACC_ANY_MOTION_X_INT_EN_0_MASK      UINT8_C(0x01)
#define BMI055_ACC_ANY_MOTION_Y_INT_EN_0_MASK      UINT8_C(0x02)
#define BMI055_ACC_ANY_MOTION_Z_INT_EN_0_MASK      UINT8_C(0x04)
#define BMI055_ACC_DOUBLE_TAP_INT_EN_0_MASK        UINT8_C(0x10)
#define BMI055_ACC_SINGLE_TAP_INT_EN_0_MASK        UINT8_C(0x20)
#define BMI055_ACC_ORIENT_INT_EN_0_MASK            UINT8_C(0x40)
#define BMI055_ACC_FLAT_INT_EN_0_MASK              UINT8_C(0x80)

/*! Bit positions for INT_EN 0 registers */
#define BMI055_ACC_ANY_MOTION_Y_INT_EN_0_POS      UINT8_C(0x01)
#define BMI055_ACC_ANY_MOTION_Z_INT_EN_0_POS      UINT8_C(0x02)
#define BMI055_ACC_ORIENT_INT_EN_0_POS            UINT8_C(0x06)

/*! Mask definitions for INT_EN 1 registers */
#define BMI055_ACC_HIGH_G_X_INT_EN_1_MASK          UINT8_C(0x01)
#define BMI055_ACC_HIGH_G_Y_INT_EN_1_MASK          UINT8_C(0x02)
#define BMI055_ACC_HIGH_G_Z_INT_EN_1_MASK          UINT8_C(0x04)
#define BMI055_ACC_LOW_G_INT_EN_1_MASK             UINT8_C(0x08)
#define BMI055_ACC_NEW_DATA_INT_EN_1_MASK          UINT8_C(0x10)
#define BMI055_ACC_FIFO_FULL_INT_EN_1_MASK         UINT8_C(0x20)
#define BMI055_ACC_FIFO_WM_INT_EN_1_MASK           UINT8_C(0x40)

/*! Bit positions for INT_EN 1 registers */
#define BMI055_ACC_NEW_DATA_INT_EN_1_POS          UINT8_C(0x04)
#define BMI055_ACC_LOW_G_INT_EN_1_POS             UINT8_C(0x03)
#define BMI055_ACC_HIGH_G_Y_INT_EN_1_POS          UINT8_C(0x01)
#define BMI055_ACC_HIGH_G_Z_INT_EN_1_POS          UINT8_C(0x02)

/*! Mask definitions for INT_EN 2 registers */
#define BMI055_ACC_SLOW_NO_MOT_X_INT_EN_2_MASK     UINT8_C(0x01)
#define BMI055_ACC_SLOW_NO_MOT_Y_INT_EN_2_MASK     UINT8_C(0x02)
#define BMI055_ACC_SLOW_NO_MOT_Z_INT_EN_2_MASK     UINT8_C(0x04)
#define BMI055_ACC_SLOW_NO_MOT_SEL_INT_EN_2_MASK   UINT8_C(0x08)

/*! Bit positions for INT_EN 2 registers */
#define BMI055_ACC_SLOW_NO_MOT_X_INT_EN_2_POS0
#define BMI055_ACC_SLOW_NO_MOT_Y_INT_EN_2_POS     UINT8_C(0x01)
#define BMI055_ACC_SLOW_NO_MOT_Z_INT_EN_2_POS     UINT8_C(0x02)
#define BMI055_ACC_SLOW_NO_MOT_SEL_INT_EN_2_POS   UINT8_C(0x03)

/*! Mask definitions for INT_MAP 0 registers */
#define BMI055_ACC_INT1_MAP_0_LOW_G_MASK           UINT8_C(0x01)
#define BMI055_ACC_INT1_MAP_0_HIGH_G_MASK          UINT8_C(0x02)
#define BMI055_ACC_INT1_MAP_0_SLOPE_MASK           UINT8_C(0x04)
#define BMI055_ACC_INT1_MAP_0_SLOW_NO_MOT_MASK     UINT8_C(0x08)
#define BMI055_ACC_INT1_MAP_0_DOUBLE_TAP_MASK      UINT8_C(0x10)
#define BMI055_ACC_INT1_MAP_0_SINGLE_TAP_MASK      UINT8_C(0x20)
#define BMI055_ACC_INT1_MAP_0_ORIENT_MASK          UINT8_C(0x40)
#define BMI055_ACC_INT1_MAP_0_FLAT_MASK            UINT8_C(0x80)

/*! Bit positions for INT_MAP 0 registers */
#define BMI055_ACC_INT1_MAP_0_HIGH_G_POS          UINT8_C(0x01)
#define BMI055_ACC_INT1_MAP_0_SLOW_NO_MOT_POS     UINT8_C(0x03)
#define BMI055_ACC_INT1_MAP_0_SLOPE_POS           UINT8_C(0x02)
#define BMI055_ACC_INT1_MAP_0_ORIENT_POS          UINT8_C(0x06)

/*! Mask definitions for INT_MAP 1 registers */
#define BMI055_ACC_INT1_MAP_1_NEW_DATA_MASK        UINT8_C(0x01)
#define BMI055_ACC_INT1_MAP_1_FIFO_WM_MASK         UINT8_C(0x02)
#define BMI055_ACC_INT1_MAP_1_FIFO_FULL_MASK       UINT8_C(0x04)
#define BMI055_ACC_INT2_MAP_1_FIFO_FULL_MASK       UINT8_C(0x20)
#define BMI055_ACC_INT2_MAP_1_FIFO_WM_MASK         UINT8_C(0x40)
#define BMI055_ACC_INT2_MAP_1_NEW_DATA_MASK        UINT8_C(0x80)

/*! Bit positions for INT_MAP 1 registers */
#define BMI055_ACC_INT2_MAP_1_NEW_DATA_POS       UINT8_C(0x07)

/*! Mask definitions for INT_MAP 2 registers */
#define BMI055_ACC_INT2_MAP_2_LOW_G_MASK           UINT8_C(0x01)
#define BMI055_ACC_INT2_MAP_2_HIGH_G_MASK          UINT8_C(0x02)
#define BMI055_ACC_INT2_MAP_2_SLOPE_MASK           UINT8_C(0x04)
#define BMI055_ACC_INT2_MAP_2_SLOW_NO_MOT_MASK     UINT8_C(0x08)
#define BMI055_ACC_INT2_MAP_2_DOUBLE_TAP_MASK      UINT8_C(0x10)
#define BMI055_ACC_INT2_MAP_2_SINGLE_TAP_MASK      UINT8_C(0x20)
#define BMI055_ACC_INT2_MAP_2_ORIENT_MASK          UINT8_C(0x40)
#define BMI055_ACC_INT2_MAP_2_FLAT_MASK            UINT8_C(0x80)

/*! Bit positions for INT_MAP 2 registers */
#define BMI055_ACC_INT2_MAP_2_HIGH_G_POS          UINT8_C(0x01)
#define BMI055_ACC_INT2_MAP_2_SLOW_NO_MOT_POS     UINT8_C(0x03)
#define BMI055_ACC_INT2_MAP_2_SLOPE_POS           UINT8_C(0x02)
#define BMI055_ACC_INT2_MAP_2_ORIENT_POS          UINT8_C(0x06)

/*! Mask definitions for INT_OUT_CTRL register */
#define BMI055_ACC_INT1_LVL_MASK                   UINT8_C(0x01)
#define BMI055_ACC_INT1_OD_MASK                    UINT8_C(0x02)
#define BMI055_ACC_INT2_LVL_MASK                   UINT8_C(0x04)
#define BMI055_ACC_INT2_OD_MASK                    UINT8_C(0x08)

/*! Bit position for INT_OUT_CTRL register */
#define BMI055_ACC_INT1_OD_POS                     UINT8_C(0x01)
#define BMI055_ACC_INT2_LVL_POS                    UINT8_C(0x02)
#define BMI055_ACC_INT2_OD_POS                     UINT8_C(0x03)

/*! Mask definitions for INT_RST_LATCH register */
#define BMI055_ACC_LATCH_DUR_MASK                  UINT8_C(0x0F)
#define BMI055_ACC_RESET_INT_MASK                  UINT8_C(0x80)

/*! Bit position for INT_RST_LATCH register */
#define BMI055_ACC_RESET_INT_POS                  UINT8_C(0x07)

/*! Mask definitions for INT_SRC register */
#define BMI055_ACC_SRC_LOW_G_MASK                  UINT8_C(0x01)
#define BMI055_ACC_SRC_HIGH_G_MASK                 UINT8_C(0x02)
#define BMI055_ACC_SRC_SLOPE_MASK                  UINT8_C(0x04)
#define BMI055_ACC_SRC_SLOW_NO_MOT_MASK            UINT8_C(0x08)
#define BMI055_ACC_SRC_TAP_MASK                    UINT8_C(0x10)
#define BMI055_ACC_SRC_NEW_DATA_MASK               UINT8_C(0x20)

/*! Bit positions for INT_SRC register */
#define BMI055_ACC_SRC_NEW_DATA_POS               UINT8_C(0x05)
#define BMI055_ACC_SRC_HIGH_G_POS                 UINT8_C(0x01)
#define BMI055_ACC_SRC_SLOW_NO_MOT_POS            UINT8_C(0x03)
#define BMI055_ACC_SRC_SLOPE_POS                  UINT8_C(0x02)

/*! Mask definitions for slope/ Any-motion Interrupt */
#define BMI055_ACC_SLOPE_DUR_MASK                  UINT8_C(0x03)

/*! Mask definitions for single/ double tap Interrupts */
#define BMI055_ACC_TAP_DUR_MASK                    UINT8_C(0x07)
#define BMI055_ACC_TAP_SHOCK_MASK                  UINT8_C(0x40)
#define BMI055_ACC_TAP_QUIET_MASK                  UINT8_C(0x80)
#define BMI055_ACC_TAP_THRES_MASK                  UINT8_C(0x1F)
#define BMI055_ACC_TAP_SAMP_MASK                   UINT8_C(0xC0)

/*! Mask definitions for Orientation Interrupt */
#define BMI055_ACC_ORIENT_MODE_MASK                UINT8_C(0x03)
#define BMI055_ACC_ORIENT_BLOCKING_MASK            UINT8_C(0x0C)
#define BMI055_ACC_ORIENT_HYST_MASK                UINT8_C(0x70)
#define BMI055_ACC_ORIENT_THETA_MASK               UINT8_C(0x3F)
#define BMI055_ACC_ORIENT_UD_EN_MASK               UINT8_C(0x40)

/*! Bit position for Orientation Interrupt */
#define BMI055_ACC_ORIENT_BLOCKING_POS             UINT8_C(0x02)
#define BMI055_ACC_ORIENT_HYST_POS                 UINT8_C(0x04)
#define BMI055_ACC_ORIENT_UD_EN_POS                UINT8_C(0x06)

/*! Mask definitions for Flat Interrupt */
#define BMI055_ACC_FLAT_THETA_MASK                 UINT8_C(0x3F)
#define BMI055_ACC_FLAT_HYST_MASK                  UINT8_C(0x07)
#define BMI055_ACC_FLAT_HOLD_TIME_MASK             UINT8_C(0x30)

/*! Mask definitions for Low-g Interrupt */
#define BMI055_ACC_LOW_MODE_MASK                   UINT8_C(0x04)
#define BMI055_ACC_LOW_HYST_MASK                   UINT8_C(0x03)

/*! Bit position for Low-g Interrupt */
#define BMI055_ACC_LOW_MODE_POS                   UINT8_C(0x02)

/*! Mask definitions for High-g Interrupt */
#define BMI055_ACC_HIGH_HYST_MASK                  UINT8_C(0xC0)

/*! Bit position for High-g Interrupt */
#define BMI055_ACC_HIGH_HYST_POS                   UINT8_C(0x06)

/*! Mask definitions for Slow/ No motion Interrupt */
/* Defines no of consecutive slope data points */
#define BMI055_ACC_SLOW_DUR_MASK                   UINT8_C(0x0C)
/* Defines delay before interrupt is triggered */
#define BMI055_ACC_NO_MOTION_DUR_MASK              UINT8_C(0xFC)

/*! Bit position for Slow/ No motion Interrupt */
#define BMI055_ACC_SLOW_DUR_POS                   UINT8_C(0x02)
#define BMI055_ACC_NO_MOTION_DUR_POS              UINT8_C(0x02)

/*! Mask definitions for FIFO water-mark Interrupt */
#define BMI055_ACC_FIFO_WM_LEVEL_TRIG_MASK         UINT8_C(0x3F)

/*! Mask definitions for Interrupt status bits */
#define BMI055_ACC_LOW_G_INT_STATUS_MASK           UINT32_C(1)
#define BMI055_ACC_HIGH_G_INT_STATUS_MASK          UINT32_C(1 << 1)
#define BMI055_ACC_SLOPE_INT_STATUS_MASK           UINT32_C(1 << 2)
#define BMI055_ACC_SLOW_NO_MOT_INT_STATUS_MASK     UINT32_C(1 << 3)
#define BMI055_ACC_DOUBLE_TAP_INT_STATUS_MASK      UINT32_C(1 << 4)
#define BMI055_ACC_SINGLE_TAP_INT_STATUS_MASK      UINT32_C(1 << 5)
#define BMI055_ACC_ORIENT_INT_STATUS_MASK          UINT32_C(1 << 6)
#define BMI055_ACC_TAP_INT_STATUS_MASK             UINT32_C(1 << 7)
#define BMI055_ACC_FIFO_FULL_INT_STATUS_MASK       UINT32_C(1 << 13)
#define BMI055_ACC_FIFO_WM_INT_STATUS_MASK         UINT32_C(1 << 14)
#define BMI055_ACC_NEW_DATA_INT_STATUS_MASK        UINT32_C(1 << 15)
#define BMI055_ACC_SLOPE_X_INT_STATUS_MASK         UINT32_C(1 << 16)
#define BMI055_ACC_SLOPE_Y_INT_STATUS_MASK         UINT32_C(1 << 17)
#define BMI055_ACC_SLOPE_Z_INT_STATUS_MASK         UINT32_C(1 << 18)
#define BMI055_ACC_SLOPE_SIGN_INT_STATUS_MASK      UINT32_C(1 << 19)
#define BMI055_ACC_TAP_X_INT_STATUS_MASK           UINT32_C(1 << 20)
#define BMI055_ACC_TAP_Y_INT_STATUS_MASK           UINT32_C(1 << 21)
#define BMI055_ACC_TAP_Z_INT_STATUS_MASK           UINT32_C(1 << 22)
#define BMI055_ACC_TAP_SIGN_INT_STATUS_MASK        UINT32_C(1 << 23)
#define BMI055_ACC_HIGH_G_X_INT_STATUS_MASK        UINT32_C(1 << 24)
#define BMI055_ACC_HIGH_G_Y_INT_STATUS_MASK        UINT32_C(1 << 25)
#define BMI055_ACC_HIGH_G_Z_INT_STATUS_MASK        UINT32_C(1 << 26)
#define BMI055_ACC_HIGH_G_SIGN_INT_STATUS_MASK     UINT32_C(1 << 27)
#define BMI055_ACC_ORIENT_BIT_MASK                 UINT32_C(7 << 28)
#define BMI055_ACC_ORIENT_PORTRAIT_UPRIGHT_MASK    UINT32_C(0 << 28)
#define BMI055_ACC_ORIENT_PORTRAIT_DOWN_MASK       UINT32_C(1 << 28)
#define BMI055_ACC_ORIENT_LANDSACPE_LEFT_MASK      UINT32_C(2 << 28)
#define BMI055_ACC_ORIENT_LANDSACPE_RIGHT_MASK     UINT32_C(3 << 28)
#define BMI055_ACC_ORIENT_Z_AXIS_DIR_MASK          UINT32_C(1 << 30)
#define BMI055_ACC_FLAT_INT_STATUS_MASK            UINT32_C(1 << 31)

/*! BMI055 Accelerometer unique chip identifier */
#define BMI055_ACCEL_CHIP_ID                       UINT8_C(0xFA)

/*! BMI055 Accelerometer Bandwidth */
#define BMI055_ACCEL_BW_7_81_HZ                    UINT8_C(0x08)
#define BMI055_ACCEL_BW_15_63_HZ                   UINT8_C(0x09)
#define BMI055_ACCEL_BW_31_25_HZ                   UINT8_C(0x0A)
#define BMI055_ACCEL_BW_62_5_HZ                    UINT8_C(0x0B)
#define BMI055_ACCEL_BW_125_HZ                     UINT8_C(0x0C)
#define BMI055_ACCEL_BW_250_HZ                     UINT8_C(0x0D)
#define BMI055_ACCEL_BW_500_HZ                     UINT8_C(0x0E)
#define BMI055_ACCEL_BW_1000_HZ                    UINT8_C(0x0F)

/*! BMI055 Accelerometer Range */
#define BMI055_ACCEL_RANGE_2G                      UINT8_C(0x03)
#define BMI055_ACCEL_RANGE_4G                      UINT8_C(0x05)
#define BMI055_ACCEL_RANGE_8G                      UINT8_C(0x08)
#define BMI055_ACCEL_RANGE_16G                     UINT8_C(0x0C)

/*! BMI055 Accelerometer Power */
#define BMI055_ACCEL_PM_NORMAL                     UINT8_C(0x00)
#define BMI055_ACCEL_PM_DEEP_SUSPEND               UINT8_C(0x01)
#define BMI055_ACCEL_PM_LOW_POWER                  UINT8_C(0x02)
#define BMI055_ACCEL_INVALID_POWER                  UINT8_C(0x03)
#define BMI055_ACCEL_PM_SUSPEND                    UINT8_C(0x04)

/*! Maximum limits definition */
#define BMI055_ACC_BW_MAX                          UINT8_C(0x1F)
#define BMI055_ACC_POWER_MAX                       UINT8_C(0x05)

/******************************************************************************/
/*!			BMI055 Gyroscope Macros	*/
/******************************************************************************/
/*! Interrupt types of gyroscope */
/* Slope/ Any-motion interrupt */
#define BMI055_GYRO_ANY_MOTION_INT                 UINT8_C(0x00)
/* High-rate Interrupt */
#define BMI055_GYRO_HIGH_RATE_INT                  UINT8_C(0x01)
/* data-ready interrupt */
#define BMI055_GYRO_DATA_RDY_INT                   UINT8_C(0x02)
/* Auto-offset interrupt */
#define BMI055_GYRO_AUTO_OFFSET_INT                UINT8_C(0x03)
/* Slow-offset interrupt */
#define BMI055_GYRO_SLOW_OFFSET_INT                UINT8_C(0x04)
/* Fast-offset interrupt */
#define BMI055_GYRO_FAST_OFFSET_INT                UINT8_C(0x05)

/*! Interrupt channels of gyroscope */
 /* Interrupt Channel 3 for gyroscope sensor*/
#define BMI055_INT_CHANNEL_3                       UINT8_C(0x00)
 /* Interrupt Channel 4 for gyroscope sensor*/
#define BMI055_INT_CHANNEL_4                       UINT8_C(0x01)

/*! Register map of Gyroscope registers */
#define BMI055_GYR_CHIP_ID_ADDR                    UINT8_C(0x00)
#define BMI055_GYR_X_L_ADDR                        UINT8_C(0x02)
#define BMI055_GYR_X_H_ADDR                        UINT8_C(0x03)
#define BMI055_GYR_Y_L_ADDR                        UINT8_C(0x04)
#define BMI055_GYR_Y_H_ADDR                        UINT8_C(0x05)
#define BMI055_GYR_Z_L_ADDR                        UINT8_C(0x06)
#define BMI055_GYR_Z_H_ADDR                        UINT8_C(0x07)
#define BMI055_GYR_INT_STATUS_0_ADDR               UINT8_C(0x09)
#define BMI055_GYR_INT_STATUS_1_ADDR               UINT8_C(0x0A)
#define BMI055_GYR_INT_STATUS_2_ADDR               UINT8_C(0x0B)
#define BMI055_GYR_INT_STATUS_3_ADDR               UINT8_C(0x0C)
#define BMI055_GYR_FIFO_STATUS_ADDR                UINT8_C(0x0E)
#define BMI055_GYR_RANGE_ADDR                      UINT8_C(0x0F)
#define BMI055_GYR_BW_ADDR                         UINT8_C(0x10)
#define BMI055_GYR_LPM1_ADDR                       UINT8_C(0x11)
#define BMI055_GYR_LPM2_ADDR                       UINT8_C(0x12)
#define BMI055_GYR_RATE_HBW_ADDR                   UINT8_C(0x13)
#define BMI055_GYR_SOFTRESET_ADDR                  UINT8_C(0x14)
#define BMI055_GYR_INT_EN_0_ADDR                   UINT8_C(0x15)
#define BMI055_GYR_INT_EN_1_ADDR                   UINT8_C(0x16)
#define BMI055_GYR_INT_MAP_0_ADDR                  UINT8_C(0x17)
#define BMI055_GYR_INT_MAP_1_ADDR                  UINT8_C(0x18)
#define BMI055_GYR_INT_MAP_2_ADDR                  UINT8_C(0x19)
#define BMI055_GYRO_0_REG                          UINT8_C(0x1A)
#define BMI055_GYRO_1_REG                          UINT8_C(0x1B)
#define BMI055_GYRO_2_REG                          UINT8_C(0x1C)
#define BMI055_GYRO_3_REG                          UINT8_C(0x1E)
#define BMI055_GYR_INT_LATCH_ADDR                  UINT8_C(0x21)
#define BMI055_GYR_INT_HR_0_ADDR                   UINT8_C(0x22)
#define BMI055_GYR_INT_HR_1_ADDR                   UINT8_C(0x23)
#define BMI055_GYR_INT_HR_2_ADDR                   UINT8_C(0x24)
#define BMI055_GYR_INT_HR_3_ADDR                   UINT8_C(0x25)
#define BMI055_GYR_INT_HR_4_ADDR                   UINT8_C(0x26)
#define BMI055_GYR_INT_HR_5_ADDR                   UINT8_C(0x27)
#define BMI055_GYR_SOC_ADDR                        UINT8_C(0x31)
#define BMI055_GYR_A_FOC_ADDR                      UINT8_C(0x32)
#define BMI055_GYR_TRIM_NVM_CTRL_ADDR              UINT8_C(0x33)
#define BMI055_BGW_SPI3_WDT_ADDR                   UINT8_C(0x34)
#define BMI055_GYR_OFFSET_COMP_ADDR                UINT8_C(0x36)
#define BMI055_GYR_OFFSET_COMP_X_ADDR              UINT8_C(0x37)
#define BMI055_GYR_OFFSET_COMP_Y_ADDR              UINT8_C(0x38)
#define BMI055_GYR_OFFSET_COMP_Z_ADDR              UINT8_C(0x39)
#define BMI055_GYR_TRIM_GPO_ADDR                   UINT8_C(0x3A)
#define BMI055_GYR_TRIM_GP1_ADDR                   UINT8_C(0x3B)
#define BMI055_GYR_SELFTEST_ADDR                   UINT8_C(0x3C)
#define BMI055_GYR_FIFO_CONFIG_0_ADDR              UINT8_C(0x3D)
#define BMI055_GYR_FIFO_CONFIG_1_ADDR              UINT8_C(0x3E)
#define BMI055_GYR_FIFO_DATA_ADDR                  UINT8_C(0x3F)

/*! Mask definitions for range, bandwidth and power */
#define BMI055_GYRO_RANGE_MASK                      UINT8_C(0x07)
#define BMI055_GYRO_BW_MASK                         UINT8_C(0x0F)
#define BMI055_GYRO_POWER_MASK                      UINT8_C(0xA0)

/*! Mask definitions for INT_EN 0 register */
#define BMI055_GYR_AUTO_OFF_INT_EN_0_MASK          UINT8_C(0x04)
#define BMI055_GYR_FIFO_INT_EN_0_MASK              UINT8_C(0x40)
#define BMI055_GYR_DATA_INT_EN_0_MASK              UINT8_C(0x80)

/*! Mask definitions for INT_EN 1 register */
#define BMI055_GYR_INT1_LVL_INT_EN_1_MASK          UINT8_C(0x01)
#define BMI055_GYR_INT1_OD_INT_EN_1_MASK           UINT8_C(0x02)
#define BMI055_GYR_INT2_LVL_INT_EN_1_MASK          UINT8_C(0x04)
#define BMI055_GYR_INT2_OD_INT_EN_1_MASK           UINT8_C(0x08)


/*! Mask definitions for INT_MAP 0 register */
#define BMI055_GYR_INT1_MAP_0_ANY_MOT_MASK         UINT8_C(0x02)
#define BMI055_GYR_INT1_MAP_0_HIGHRATE_MASK        UINT8_C(0x08)

/*! Mask definitions for INT_MAP 1 register */
#define BMI055_GYR_INT1_MAP_1_DATA_MASK            UINT8_C(0x01)
#define BMI055_GYR_INT1_MAP_1_FAST_OFF_MASK        UINT8_C(0x02)
#define BMI055_GYR_INT1_MAP_1_FIFO_MASK            UINT8_C(0x04)
#define BMI055_GYR_INT1_MAP_1_AUTO_OFF_MASK        UINT8_C(0x08)
#define BMI055_GYR_INT2_MAP_1_AUTO_OFF_MASK        UINT8_C(0x10)
#define BMI055_GYR_INT2_MAP_1_FIFO_MASK            UINT8_C(0x20)
#define BMI055_GYR_INT2_MAP_1_FAST_OFF_MASK        UINT8_C(0x40)
#define BMI055_GYR_INT2_MAP_1_DATA_MASK            UINT8_C(0x80)

/*! Mask definitions for INT_MAP 2 register */
#define BMI055_GYR_INT2_MAP_2_ANY_MOT_MASK         UINT8_C(0x02)
#define BMI055_GYR_INT2_MAP_2_HIGHRATE_MASK        UINT8_C(0x08)

/*! Mask definitions for Data Source 0 Register */
#define BMI055_GYR_SRC_ANY_MOT_MASK                UINT8_C(0x02)
#define BMI055_GYR_SRC_HIGHRATE_MASK               UINT8_C(0x08)
#define BMI055_GYR_SRC_SLOW_OFFSET_MASK            UINT8_C(0x20)

/*! Mask definitions for Data Source 1 Register */
#define BMI055_GYR_SRC_FAST_OFFSET_MASK            UINT8_C(0x80)

/*! Mask definitions for INT_RST_LATCH */
#define BMI055_GYR_LATCH_DUR_MASK                  UINT8_C(0x0F)
#define BMI055_GYR_OFFSET_RESET_MASK               UINT8_C(0x40)
#define BMI055_GYR_RESET_INT_MASK                  UINT8_C(0x80)

/*! Mask definitions for GYRO_3_REG */
#define BMI055_GYR_FIFO_WM_EN_MASK                 UINT8_C(0x80)

/*! Mask definitions for Any-motion Interrupt */
#define BMI055_GYR_ANY_THRES_MASK                  UINT8_C(0x7F)
#define BMI055_GYR_ANY_EN_X_MASK                   UINT8_C(0x01)
#define BMI055_GYR_ANY_EN_Y_MASK                   UINT8_C(0x02)
#define BMI055_GYR_ANY_EN_Z_MASK                   UINT8_C(0x04)
#define BMI055_GYR_ANY_DURSAMPLE_MASK              UINT8_C(0x30)
#define BMI055_GYR_AWAKE_DUR_MASK                  UINT8_C(0xC0)

/*! Mask definitions for High-Rate Interrupt */
#define BMI055_GYR_HIGH_EN_X_MASK                  UINT8_C(0x01)
#define BMI055_GYR_HIGH_TH_X_MASK                  UINT8_C(0x3E)
#define BMI055_GYR_HIGH_HY_X_MASK                  UINT8_C(0xC0)
#define BMI055_GYR_HIGH_EN_Y_MASK                  UINT8_C(0x01)
#define BMI055_GYR_HIGH_TH_Y_MASK                  UINT8_C(0x3E)
#define BMI055_GYR_HIGH_HY_Y_MASK                  UINT8_C(0xC0)
#define BMI055_GYR_HIGH_EN_Z_MASK                  UINT8_C(0x01)
#define BMI055_GYR_HIGH_TH_Z_MASK                  UINT8_C(0x3E)
#define BMI055_GYR_HIGH_HY_Z_MASK                  UINT8_C(0xC0)

/*! Mask definitions for Auto-offset Interrupt */
#define BMI055_GYR_AUTO_OFF_WORD_LEN_MASK          UINT8_C(0xC0)

/*! Mask definitions for Slow-offset Interrupt */
#define BMI055_GYR_SLOW_OFF_EN_X_MASK              UINT8_C(0x01)
#define BMI055_GYR_SLOW_OFF_EN_Y_MASK              UINT8_C(0x02)
#define BMI055_GYR_SLOW_OFF_EN_Z_MASK              UINT8_C(0x04)
#define BMI055_GYR_SLOW_OFF_DUR_MASK               UINT8_C(0x38)
#define BMI055_GYR_SLOW_OFF_THR_MASK               UINT8_C(0xC0)


/*! Mask definitions for Fast-offset Interrupt */
#define BMI055_GYR_FAST_OFF_EN_X_MASK              UINT8_C(0x01)
#define BMI055_GYR_FAST_OFF_EN_Y_MASK              UINT8_C(0x02)
#define BMI055_GYR_FAST_OFF_EN_Z_MASK              UINT8_C(0x04)
#define BMI055_GYR_FAST_OFF_EN_MASK                UINT8_C(0x08)
#define BMI055_GYR_FAST_OFF_WORD_LEN_MASK          UINT8_C(0x30)

/*! Mask definitions for FIFO Water-mark Interrupt */
#define BMI055_FIFO_WATERMARK_LEVEL_MASK           UINT8_C(0x7F)

/*! BMI055 Gyroscope unique chip identifier */
#define BMI055_GYRO_CHIP_ID                        UINT8_C(0x0F)

/*! BMI055 Gyroscope Bandwidth */
#define BMI055_GYRO_BW_32_HZ                       UINT8_C(0x07)
#define BMI055_GYRO_BW_64_HZ                       UINT8_C(0x06)
#define BMI055_GYRO_BW_12_HZ                       UINT8_C(0x05)
#define BMI055_GYRO_BW_23_HZ                       UINT8_C(0x04)
#define BMI055_GYRO_BW_47_HZ                       UINT8_C(0x03)
#define BMI055_GYRO_BW_116_HZ                      UINT8_C(0x02)
#define BMI055_GYRO_BW_230_HZ                      UINT8_C(0x01)
#define BMI055_GYRO_BW_523_HZ                      UINT8_C(0x00)

/*! BMI055 Gyroscope range */
#define BMI055_GYRO_RANGE_2000_DPS                 UINT8_C(0x00)
#define BMI055_GYRO_RANGE_1000_DPS                 UINT8_C(0x01)
#define BMI055_GYRO_RANGE_500_DPS                  UINT8_C(0x02)
#define BMI055_GYRO_RANGE_250_DPS                  UINT8_C(0x03)
#define BMI055_GYRO_RANGE_125_DPS                  UINT8_C(0x04)

/*! BMI055 Gyroscope power */
#define BMI055_GYRO_PM_NORMAL                      UINT8_C(0x00)
#define BMI055_GYRO_PM_DEEP_SUSPEND                UINT8_C(0x01)
#define BMI055_GYRO_PM_SUSPEND                     UINT8_C(0x02)

/******************************************************************************/
/*!			Common macros for both accelerometer and gyroscope  */
/******************************************************************************/
/*! Delay in ms settings */
#define BMI055_ONE_MS_DELAY                        UINT8_C(1)

/*! Interface settings */
#define BMI055_SPI_INTF                            UINT8_C(1)
#define BMI055_I2C_INTF                            UINT8_C(0)

/*! SPI read/write mask to configure address */
#define BMI055_SPI_RD_MASK                         UINT8_C(0x80)
#define BMI055_SPI_WR_MASK                         UINT8_C(0x7F)

/*! BMI055 I2C addresses */
#define BMI055_ACCEL_I2C_ADDR                      UINT8_C(0x18)
#define BMI055_GYRO_I2C_ADDR                       UINT8_C(0x68)

/*! Error Codes */
#define BMI055_OK                                  INT8_C(0)
#define BMI055_E_NULL_PTR                          INT8_C(-1)
#define BMI055_E_COM_FAIL                          INT8_C(-2)
#define BMI055_E_DEV_NOT_FOUND                     INT8_C(-3)
#define BMI055_E_OUT_OF_RANGE                      INT8_C(-4)
#define BMI055_E_INVALID_INPUT                     INT8_C(-5)
#define BMI055_E_CONFIG_FAIL	                   INT8_C(-6)
#define BMI055_E_INVALID_CHANNEL	           INT8_C(-6)

/*! Soft reset Value */
#define BMI055_SOFT_RESET_VAL                      UINT8_C(0xB6)

/*! Interrupt pin configuration macros */
/*! Open drain */
#define BMI055_INT_PIN_OPEN_DRAIN                  UINT8_C(0x01)
#define BMI055_INT_PIN_PUSH_PULL                   UINT8_C(0x00)

/*! Level */
#define BMI055_INT_PIN_ACTIVE_HIGH                 UINT8_C(0x01)
#define BMI055_INT_PIN_ACTIVE_LOW                  UINT8_C(0x00)

/*! Latch */
#define BMI055_INT_PIN_LATCH                       UINT8_C(0x01)
#define BMI055_INT_PIN_NON_LATCH                   UINT8_C(0x00)

/*! Reset */
#define BMI055_INT_LATCH_CLEAR                       UINT8_C(0x01)
#define BMI055_INT_LATCH_ACTIVE                       UINT8_C(0x00)

/*! Interrupt Latch duration */
#define BMI055_LATCH_DUR_NONE_1                    UINT8_C(0x00)
#define BMI055_LATCH_DUR_250_MS                    UINT8_C(0x01)
#define BMI055_LATCH_DUR_500_MS                    UINT8_C(0x02)
#define BMI055_LATCH_DUR_1_S                       UINT8_C(0x03)
#define BMI055_LATCH_DUR_2_S                       UINT8_C(0x04)
#define BMI055_LATCH_DUR_4_S                       UINT8_C(0x05)
#define BMI055_LATCH_DUR_8_S                       UINT8_C(0x06)
#define BMI055_LATCHED_1                           UINT8_C(0x07)
#define BMI055_LATCH_DUR_NONE_2                    UINT8_C(0x08)
#define BMI055_LATCH_DUR_250_US                    UINT8_C(0x09)
#define BMI055_LATCH_DUR_500_US                    UINT8_C(0x0A)
#define BMI055_LATCH_DUR_1_MS                      UINT8_C(0x0B)
#define BMI055_LATCH_DUR_12_5_MS                   UINT8_C(0x0C)
#define BMI055_LATCH_DUR_25_MS                     UINT8_C(0x0D)
#define BMI055_LATCH_DUR_50_MS                     UINT8_C(0x0E)
#define BMI055_LATCHED_2                           UINT8_C(0x0F)

/******************************************************************************/
/*!				Function Pointers*/
/******************************************************************************/
 /* For read and write communication through i2c or spi */
typedef int8_t (*bmi055_com_fptr_t)(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
 /* For delay */
typedef void (*bmi055_delay_fptr_t)(uint32_t period);

/******************************************************************************/
/*!				Data Structures */
/******************************************************************************/
/*! Structure to define x, y and z axis of the sensor */
struct bmi055_sensor_data {
	/*! X-axis sensor data */
	int16_t x;
	/*! Y-axis sensor data */
	int16_t y;
	/*! Z-axis sensor data */
	int16_t z;
};

/*! Structure to define bmi055 configuration details */
struct bmi055_cfg {
	/*! power mode */
	uint8_t power;
	/*! range */
	uint8_t range;
	/*! bandwidth */
	uint8_t bw;
};

/*! Structure to define bmi055 interrupt pin settings, for both
 * little and big endian.
 */
struct bmi055_int_pin_sett {
#ifdef LITTLE_ENDIAN
	/*! 0 - push-pull, 1 - open drain */
	uint8_t output_mode : 1;
	/*! 0 - active low, 1 - active high level  */
	uint8_t output_type : 1;
	/*! 1 - clears any latched interrupts, 0 - keeps latched
	interrupts active */
	uint8_t reset_int : 1;
	/*! 1 - resets offset value calculated with FastOffset,
	SlowOffset & AutoOffset */
	uint8_t offset_reset : 1;
	/*! latch duration */
	uint8_t latch_dur : 4;
#else
	/*! latch duration */
	uint8_t latch_dur : 4;
	/*! 1 - resets offset value calculated with FastOffset,
	SlowOffset & AutoOffset */
	uint8_t offset_reset : 1;
	/*! 1 - clears any latched interrupts, 0 - keeps latched
	interrupts active */
	uint8_t reset_int : 1;
	/*! 0 - active low, 1 - active high level */
	uint8_t output_type : 1;
	/*! 0 - push-pull, 1 - open drain */
	uint8_t output_mode : 1;
#endif
};

/*! Structure to define bmi055 accelerometer slope interrupt configurations,
 * for both little and big endian.
 */
struct bmi055_acc_slop_int_cfg {
#ifdef LITTLE_ENDIAN
	/*! slope interrupt, x-axis */
	uint8_t slope_en_x : 1;
	/*! slope interrupt, y-axis */
	uint8_t slope_en_y : 1;
	/*! slope interrupt, z-axis */
	uint8_t slope_en_z : 1;
	/*! slope duration */
	uint8_t slope_dur : 2;
	/*! slope threshold */
	uint8_t slope_thr;
#else
	/*! slope threshold */
	uint8_t slope_thr;
	/*! slope duration */
	uint8_t slope_dur : 2;
	/*! slope interrupt, z-axis */
	uint8_t slope_en_z : 1;
	/*! slope interrupt, y-axis */
	uint8_t slope_en_y : 1;
	/*! slope interrupt, x-axis */
	uint8_t slope_en_x : 1;
#endif
};

/*! Structure to define bmi055 accelerometer tap interrupt configurations,
 * for both little and big endian.
 */
struct bmi055_acc_tap_int_cfg {
#ifdef LITTLE_ENDIAN
	/*! tap duration */
	uint16_t tap_dur : 3;
	/*! tap shock */
	uint16_t tap_shock : 1;
	/*! tap quiet */
	uint16_t tap_quiet : 1;
	/*! tap threshold */
	uint16_t tap_thr : 5;
	/*! tap sample */
	uint16_t tap_sample : 2;
#else
	/*! tap sample */
	uint16_t tap_sample : 2;
	/*! tap threshold */
	uint16_t tap_thr : 5;
	/*! tap quiet */
	uint16_t tap_quiet : 1;
	/*! tap shock */
	uint16_t tap_shock : 1;
	/*! tap duration */
	uint16_t tap_dur : 3;
#endif
};

/*! Structure to define bmi055 accelerometer orient interrupt configurations,
 * for both little and big endian.
 */
struct bmi055_acc_orient_int_cfg {
#ifdef LITTLE_ENDIAN
	/*! Enable orientation interrupt */
	uint16_t orient_en   : 1;
	/*! thresholds for switching between the different orientations */
	uint16_t orient_mode : 2;
	/*! blocking_mode */
	uint16_t orient_blocking : 2;
	/*! Orientation interrupt hysteresis */
	uint16_t orient_hyst : 3;
	/*! Orientation interrupt theta */
	uint16_t orient_theta : 6;
	/*! 1 / 0 - Enable/ disable Orientation interrupt */
	uint16_t orient_ud_en : 1;
#else
	/*! 1 / 0 - Enable/ disable Orientation interrupt */
	uint16_t orient_ud_en : 1;
	/*! Orientation interrupt theta */
	uint16_t orient_theta : 6;
	/*! Orientation interrupt hysteresis */
	uint16_t orient_hyst : 3;
	/*! blocking_mode */
	uint16_t orient_blocking : 2;
	/*! thresholds for switching between the different orientations */
	uint16_t orient_mode : 2;
	/*! Enable orientation interrupt */
	uint16_t orient_en   : 1;
#endif
};

/*! Structure to define bmi055 accelerometer flat detect interrupt
 * configurations, for both little and big endian.
 */
struct bmi055_acc_flat_detect_int_cfg {
#ifdef LITTLE_ENDIAN
	/*! flat threshold */
	uint16_t flat_theta : 6;
	/*! flat interrupt hysteresis */
	uint16_t flat_hy : 3;
	/*! delay time for which the flat value must remain stable
	for the flat interrupt to be generated */
	uint16_t flat_hold_time : 2;
#else
	/*! delay time for which the flat value must remain stable
	for the flat interrupt to be generated */
	uint16_t flat_hold_time : 2;
	/*! flat interrupt hysteresis */
	uint16_t flat_hy : 3;
	/*! flat threshold */
	uint16_t flat_theta : 6;
#endif
};

/*! Structure to define bmi055 accelerometer low-g interrupt configurations,
 * for both little and big endian.
 */
struct bmi055_acc_low_g_int_cfg {
#ifdef LITTLE_ENDIAN
	/*! low-g interrupt enable */
	uint8_t low_g_en : 1;
	/*! low-g interrupt trigger delay */
	uint8_t low_dur;
	/*! low-g interrupt trigger threshold */
	uint8_t low_thres;
	/*! hysteresis of low-g interrupt */
	uint8_t low_hyst : 2;
	/*! 0 - single-axis mode, 1 - axis-summing mode */
	uint8_t low_mode : 1;
#else
	/*! 0 - single-axis mode, 1 - axis-summing mode */
	uint8_t low_mode : 1;
	/*! hysteresis of low-g interrupt */
	uint8_t low_hyst : 2;
	/*! low-g interrupt trigger threshold - ranges from 0 to 1992 mg*/
	uint16_t low_thres;
	/*! low-g interrupt trigger delay ranges from 2 to 512 msec */
	uint16_t low_dur;
	/*! low-g interrupt enable */
	uint8_t low_g_en : 1;
#endif
};

/*! Structure to define bmi055 accelerometer high-g interrupt configurations,
 * for both little and big endian.
 */
struct bmi055_acc_high_g_int_cfg {
#ifdef LITTLE_ENDIAN
	/*! High-g interrupt x  */
	uint8_t high_g_en_x : 1;
	/*! High-g interrupt y  */
	uint8_t high_g_en_y : 1;
	/*! High-g interrupt z  */
	uint8_t high_g_en_z : 1;
	/*! High-g hysteresis  */
	uint8_t high_hy : 2;
	/*! High-g threshold */
	uint8_t high_thres;
	/*! High-g duration */
	uint8_t high_dur;
#else
	/*! High-g duration */
	uint8_t high_dur;
	/*! High-g threshold */
	uint8_t high_thres;
	/*! High-g hysteresis  */
	uint8_t high_hy : 2;
	/*! High-g interrupt z  */
	uint8_t high_g_en_z : 1;
	/*! High-g interrupt y  */
	uint8_t high_g_en_y : 1;
	/*! High-g interrupt x  */
	uint8_t high_g_en_x : 1;
#endif
};

/*! Structure to define bmi055 accelerometer slow/no motion interrupt
 * configurations, for both little and big endian.
 */
struct bmi055_acc_slow_no_mot_int_cfg {
#ifdef LITTLE_ENDIAN
	/*! slow/ no motion interrupt, x-axis */
	uint16_t slow_no_mot_en_x :1;
	/*! slow/ no motion interrupt, y-axis */
	uint16_t slow_no_mot_en_y :1;
	/*! slow/ no motion interrupt, z-axis */
	uint16_t slow_no_mot_en_z :1;
	/*! 1 - enable no-motion, 0 - enable slow-motion */
	uint16_t slow_no_mot_sel : 1;
	/*! Slow motion duration */
	uint16_t slow_mot_dur : 2;
	/*! No motion duration */
	uint16_t no_mot_dur : 6;
	/*! slow/ no motion  threshold */
	uint8_t slow_no_mot_thres;
#else
	/*! slow/ no motion  threshold */
	uint8_t slow_no_mot_thres;
	/*! No motion duration */
	uint16_t no_mot_dur : 6;
	/*! Slow motion duration */
	uint16_t slow_mot_dur : 2;
	/*! 1 - enable no-motion, 0 - enable slow-motion */
	uint16_t slow_no_mot_sel : 1;
	/*! slow/no motion interrupt, z-axis */
	uint16_t slow_no_mot_en_z :1;
	/*! slow/no motion interrupt, y-axis */
	uint16_t slow_no_mot_en_y :1;
	/*! slow/no motion interrupt, x-axis */
	uint16_t slow_no_mot_en_x :1;
#endif
};

/*! Structure to define bmi055 gyroscope any motion interrupt configurations,
 * for both little and big endian.
 */
struct bmi055_gyr_any_mot_int_cfg {
#ifdef LITTLE_ENDIAN
	/*! any motion interrupt, x-axis */
	uint8_t any_en_x : 1;
	/*! any motion interrupt, y-axis */
	uint8_t any_en_y : 1;
	/*! any motion interrupt, z-axis */
	uint8_t any_en_z : 1;
	/*! any motion sample duration */
	uint8_t any_dursample : 2;
	/*! awake duration */
	uint8_t awake_dur : 2;
	/*! any motion threshold */
	uint8_t any_thr;
#else
	/*! any motion threshold */
	uint8_t any_thr;
	/*! awake duration */
	uint8_t awake_dur : 2;
	/*! any motion sample duration */
	uint8_t any_dursample : 2;
	/*! any motion interrupt, z-axis */
	uint8_t any_en_z : 1;
	/*! any motion interrupt, y-axis */
	uint8_t any_en_y : 1;
	/*! any motion interrupt, x-axis */
	uint8_t any_en_x : 1;
#endif

};

/*! Structure to define bmi055 gyroscope high rate interrupt configurations,
 * for both little and big endian.
 */
struct bmi055_gyr_high_rate_int_cfg {
#ifdef LITTLE_ENDIAN
	/*! 1 / 0 - enables/disables high-rate interrupt for x-axis  */
	uint8_t high_en_x : 1;
	/*! high rate threshold for x-axis */
	uint8_t high_thr_x : 5;
	/*! high rate hysteresis for x-axis*/
	uint8_t high_hy_x : 2;
	/*! High rate duration for x-axis*/
	uint8_t high_dur_x;
	/*! 1 / 0 - enables/disables high-rate interrupt for y-axis  */
	uint8_t high_en_y : 1;
	/*! high rate threshold for y-axis */
	uint8_t high_thr_y : 5;
	/*! high rate hysteresis for y-axis*/
	uint8_t high_hy_y : 2;
	/*! High rate duration for y-axis*/
	uint8_t high_dur_y;
	/*! 1 / 0 - enables/disables high-rate interrupt for z-axis  */
	uint8_t high_en_z : 1;
	/*! high rate threshold for z-axis */
	uint8_t high_thr_z : 5;
	/*! high rate hysteresis for z-axis*/
	uint8_t high_hy_z : 2;
	/*! High rate duration for z-axis*/
	uint8_t high_dur_z;
#else
	/*! High rate duration for z-axis*/
	uint8_t high_dur_z;
	/*! high rate hysteresis for z-axis*/
	uint8_t high_hy_z : 2;
	/*! high rate threshold for z-axis */
	uint8_t high_thr_z : 5;
	/*! 1 / 0 - enables/disables high-rate interrupt for z-axis  */
	uint8_t high_en_z : 1;
	/*! High rate duration for y-axis*/
	uint8_t high_dur_y;
	/*! high rate hysteresis for y-axis*/
	uint8_t high_hy_y : 2;
	/*! high rate threshold for y-axis */
	uint8_t high_thr_y : 5;
	/*! 1 / 0 - enables/disables high-rate interrupt for y-axis  */
	uint8_t high_en_y : 1;
	/*! High rate duration for x-axis*/
	uint8_t high_dur_x;
	/*! high rate hysteresis for x-axis*/
	uint8_t high_hy_x : 2;
	/*! high rate threshold for x-axis */
	uint8_t high_thr_x : 5;
	/*! 1 / 0 - enables/disables high-rate interrupt for x-axis  */
	uint8_t high_en_x : 1;
#endif

};

/*! Structure to define bmi055 gyroscope auto offset interrupt configurations,
 * for both little and big endian.
 */
struct bmi055_gyr_auto_offset_int_cfg {
	/*! Auto-offset word length */
	uint8_t auto_offset_wordlen;
};

struct bmi055_gyr_slow_offset_int_cfg {
#ifdef LITTLE_ENDIAN
	/*! 1 / 0 - enables/disables slow offset compensation for x-axis */
	uint8_t slow_offset_en_x : 1;
	/*! 1 / 0 - enables/disables slow offset compensation for y-axis */
	uint8_t slow_offset_en_y : 1;
	/*! 1 / 0 - enables/disables slow offset compensation for z-axis */
	uint8_t slow_offset_en_z : 1;
	/*! Slow offset duration */
	uint8_t slow_offset_dur : 3;
	/*! Slow_offset threshold */
	uint8_t slow_offset_thres : 2;
#else
	/*! Slow_offset threshold */
	uint8_t slow_offset_thres : 2;
	/*! Slow offset duration */
	uint8_t slow_offset_dur : 3;
	/*! 1 / 0 - enables/disables slow offset compensation for z-axis */
	uint8_t slow_offset_en_z : 1;
	/*! 1 / 0 - enables/disables slow offset compensation for y-axis */
	uint8_t slow_offset_en_y : 1;
	/*! 1 / 0 - enables/disables slow offset compensation for x-axis */
	uint8_t slow_offset_en_x : 1;
#endif

};

/*! Structure to define bmi055 gyroscope fast offset interrupt configurations,
 * for both little and big endian.
 */
struct bmi055_gyr_fast_offset_int_cfg {
	#ifdef LITTLE_ENDIAN
	/*! 1 / 0 - enables/disables fast offset compensation for x-axis */
	uint8_t fast_offset_en_x : 1;
	/*! 1 / 0 - enables/disables fast offset compensation for y-axis */
	uint8_t fast_offset_en_y : 1;
	/*! 1 / 0 - enables/disables fast offset compensation for z-axis */
	uint8_t fast_offset_en_z : 1;
	/*! triggers fast offset compensation for the enabled axes */
	uint8_t fast_offset_en : 1;
	/*! fast_offset wordlength */
	uint8_t fast_offset_wordlen : 2;
	#else
	/*! fast_offset wordlength */
	uint8_t fast_offset_wordlen : 2;
	/*! triggers fast offset compensation for the enabled axes */
	uint8_t fast_offset_en : 1;
	/*! 1 / 0 - enables/disables fast offset compensation for z-axis */
	uint8_t fast_offset_en_z : 1;
	/*! 1 / 0 - enables/disables fast offset compensation for y-axis */
	uint8_t fast_offset_en_y : 1;
	/*! 1 / 0 - enables/disables fast offset compensation for x-axis */
	uint8_t fast_offset_en_x : 1;
#endif
};

/*!
 * Union that defines bmi055 structures for accelerometer interrupt
 * configurations.
 */
union bmi055_accel_int_type_cfg {
	/*! New data enable */
	uint8_t accel_new_data_en;
	/*! Tap interrupt structure */
	struct bmi055_acc_tap_int_cfg   acc_tap_int;
	/*! Slope/ Any motion interrupt structure */
	struct bmi055_acc_slop_int_cfg   acc_slope_int;
	/*! No-motion/ Slow motion interrupt structure */
	struct bmi055_acc_slow_no_mot_int_cfg   acc_slow_no_mot_int;
	/*! Orientation interrupt structure */
	struct bmi055_acc_orient_int_cfg    acc_orient_int;
	/*! Flat interrupt structure */
	struct bmi055_acc_flat_detect_int_cfg  acc_flat_int;
	/*! Low-g interrupt structure */
	struct bmi055_acc_low_g_int_cfg   acc_low_g_int;
	/*! High-g interrupt structure */
	struct bmi055_acc_high_g_int_cfg   acc_high_g_int;
};

/*!
 * Union that defines bmi055 structures for gyroscope interrupt configurations.
 */
union bmi055_gyro_int_type_cfg {
	/*! New data enable */
	uint8_t gyro_new_data_en;
	/*! Any-motion interrupt structure */
	struct bmi055_gyr_any_mot_int_cfg   gyr_any_mot_int;
	/*! High-Rate interrupt structure */
	struct bmi055_gyr_high_rate_int_cfg   gyr_high_rate_int;
	/*! Auto-Offset interrupt structure */
	struct bmi055_gyr_auto_offset_int_cfg   gyr_auto_offset_int;
	/*! Slow-Offset interrupt structure */
	struct bmi055_gyr_slow_offset_int_cfg   gyr_slow_offset_int;
	/*! Fast-Offset interrupt structure */
	struct bmi055_gyr_fast_offset_int_cfg   gyr_fast_offset_int;
};

/*!
 * Structure to define bmi055 accelerometer interrupt settings
*/
struct bmi055_accel_int_sett {
	/*! Select Interrupt channel */
	uint8_t acc_int_channel;
	/*! Select Interrupt type */
	uint8_t acc_int_types;
	/*! Select interrupt source type */
	uint8_t int_data_src_type;
	/*! Structure configuring Interrupt pins */
	struct bmi055_int_pin_sett   int_pin_sett;
	/*! Union configures required interrupt */
	union bmi055_accel_int_type_cfg   acc_int_type_cfg;
};

/*!
 * Structure to define bmi055 gyroscope interrupt settings
*/
struct bmi055_gyro_int_sett {
	/*! Select Interrupt channel */
	uint8_t gyr_int_channel;
	/*! Select Interrupt type */
	uint8_t gyr_int_types;
	/*! Select interrupt source type */
	uint8_t int_data_src_type;
	/*! Structure configuring Interrupt pins */
	struct bmi055_int_pin_sett   int_pin_sett;
	/*! Union configures required interrupt */
	union bmi055_gyro_int_type_cfg   gyro_int_type_cfg;
};

/*!
 * Structure to define bmi055 sensor configurations
 */
struct bmi055_dev {
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
	/*! Structure to configure accelerometer sensor  */
	struct bmi055_cfg accel_cfg;
	/*! Structure to configure gyroscope sensor  */
	struct bmi055_cfg gyro_cfg;
	/*! Read function pointer */
	bmi055_com_fptr_t read;
	/*! Write function pointer */
	bmi055_com_fptr_t write;
	/*!  Delay function pointer */
	bmi055_delay_fptr_t delay_ms;
};

/******************************************************************************/
/*!			C++ Guard Macros	*/
/******************************************************************************/
#ifdef __cplusplus
}
#endif

#endif /* BMI055_DEFS_H_ */
