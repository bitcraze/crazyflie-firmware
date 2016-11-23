/** \mainpage
*
****************************************************************************
* Copyright (C) 2015 - 2016 Bosch Sensortec GmbH
*
* File : bma2x2.h
*
* Date : 2016/03/11
*
* Revision : 2.0.4 $
*
* Usage: Sensor Driver file for BMA2x2 sensor
*
****************************************************************************
* \section License
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*   Redistributions of source code must retain the above copyright
*   notice, this list of conditions and the following disclaimer.
*
*   Redistributions in binary form must reproduce the above copyright
*   notice, this list of conditions and the following disclaimer in the
*   documentation and/or other materials provided with the distribution.
*
*   Neither the name of the copyright holder nor the names of the
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
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
**************************************************************************/

/*! \file bma2x2.h
    \brief BMA2x2 Sensor Driver Support Header File */
#ifndef __BMA2x2_H__
#define __BMA2x2_H__
/****************************************************************/
/**\name	DATA TYPES INCLUDES		*/
/************************************************************/
#include <stdint.h>

#include "bstdr_comm_support.h"
#include "bstdr_types.h"


/**************************************************************/
/**\name	I2C ADDRESS DEFINITIONS    */
/**************************************************************/
/**< The following definition of I2C address is used for the following sensors
* BMA255
* BMA355
* BMA280
* BMA282
* BMA223
* BMA254
* BMA284
* BMA250E
* BMA222E
*/
#define BMA2x2_I2C_ADDR1                (0x18)
#define BMA2x2_I2C_ADDR2                (0x19)

/**< The following definition of I2C address is used for the following sensors
* BMC150
* BMC056
* BMC156
*/
#define BMA2x2_I2C_ADDR3                (0x10)
#define BMA2x2_I2C_ADDR4                (0x11)

/**************************************************************/
/**\name	CONSTANTS DEFINITION    */
/**************************************************************/
#define         BMA2x2_INIT_VALUE                       ((uint8_t)0)
#define         BMA2x2_GEN_READ_WRITE_LENGTH            ((uint8_t)1)
#define         BMA2x2_LSB_MSB_READ_LENGTH		((uint8_t)2)
	/**	BIT SHIFT DEFINITIONS    */
#define         BMA2x2_SHIFT_TWO_BITS                   ((uint8_t)2)
#define         BMA2x2_SHIFT_FOUR_BITS                  ((uint8_t)4)
#define         BMA2x2_SHIFT_FIVE_BITS                  ((uint8_t)5)
#define         BMA2x2_SHIFT_SIX_BITS                   ((uint8_t)6)
#define         BMA2x2_SHIFT_EIGHT_BITS                 ((uint8_t)8)
	/**	FIFO DEFINITIONS    */
#define		BMA2x2_FIFO_MODE_STATUS_RANGE		((uint8_t)2)
#define		BMA2x2_FIFO_DATA_SELECT_RANGE		((uint8_t)4)
#define		BMA2x2_FIFO_MODE_RANGE			((uint8_t)4)
#define         BMA2x2_FIFO_WML_RANGE                   ((uint8_t)32)
	/**	MODE RANGES    */
#define         BMA2x2_ACCEL_BW_MIN_RANGE               ((uint8_t)7)
#define         BMA2x2_ACCEL_BW_1000HZ_RANGE            ((uint8_t)15)
#define         BMA2x2_ACCEL_BW_MAX_RANGE               ((uint8_t)16)
#define		BMA2x2_SLEEP_DURN_MIN_RANGE		((uint8_t)4)
#define		BMA2x2_SLEEP_TIMER_MODE_RANGE		((uint8_t)2)
#define		BMA2x2_SLEEP_DURN_MAX_RANGE		((uint8_t)16)
#define		BMA2x2_POWER_MODE_RANGE			((uint8_t)6)
#define		BMA2x2_SELF_TEST_AXIS_RANGE		((uint8_t)4)
#define		BMA2x2_SELF_TEST_SIGN_RANGE		((uint8_t)2)

/**************************************************************/
/**\name	ERROR CODE DEFINITIONS    */
/**************************************************************/
#define E_OUT_OF_RANGE          ((int8_t)-2)
#define E_BMA2x2_NULL_PTR       ((int8_t)-127)
#define BMA2x2_NULL             (0)
#define ERROR					((int8_t)-1)
#define	SUCCESS					((uint8_t)0)
/**************************************************************/
/**\name	RETURN TYPE DEFINITION    */
/**************************************************************/
#define	bstdr_ret_t        int8_t
/**< This refers BMA2x2 return type as char */

/**************************************************************/
/**\name	REGISTER ADDRESS DEFINITIONS    */
/**************************************************************/
#define BMA2x2_EEP_OFFSET                       (0x16)
#define BMA2x2_IMAGE_BASE                       (0x38)
#define BMA2x2_IMAGE_LEN                        (22)
#define BMA2x2_CHIP_ID_ADDR			(0x00)
/** DATA ADDRESS DEFINITIONS */
#define BMA2x2_X_AXIS_LSB_ADDR                  (0x02)
#define BMA2x2_X_AXIS_MSB_ADDR                  (0x03)
#define BMA2x2_Y_AXIS_LSB_ADDR                  (0x04)
#define BMA2x2_Y_AXIS_MSB_ADDR                  (0x05)
#define BMA2x2_Z_AXIS_LSB_ADDR                  (0x06)
#define BMA2x2_Z_AXIS_MSB_ADDR                  (0x07)
#define BMA2x2_TEMP_ADDR			(0x08)
/**STATUS ADDRESS DEFINITIONS */
#define BMA2x2_STAT1_ADDR			(0x09)
#define BMA2x2_STAT2_ADDR			(0x0A)
#define BMA2x2_STAT_TAP_SLOPE_ADDR		(0x0B)
#define BMA2x2_STAT_ORIENT_HIGH_ADDR		(0x0C)
#define BMA2x2_STAT_FIFO_ADDR			(0x0E)
/**STATUS ADDRESS DEFINITIONS */
#define BMA2x2_RANGE_SELECT_ADDR		(0x0F)
#define BMA2x2_BW_SELECT_ADDR                   (0x10)
#define BMA2x2_MODE_CTRL_ADDR                   (0x11)
#define BMA2x2_LOW_NOISE_CTRL_ADDR              (0x12)
#define BMA2x2_DATA_CTRL_ADDR                   (0x13)
#define BMA2x2_RST_ADDR                         (0x14)
/**INTERUPT ADDRESS DEFINITIONS */
#define BMA2x2_INTR_ENABLE1_ADDR                (0x16)
#define BMA2x2_INTR_ENABLE2_ADDR                (0x17)
#define BMA2x2_INTR_SLOW_NO_MOTION_ADDR         (0x18)
#define BMA2x2_INTR1_PAD_SELECT_ADDR            (0x19)
#define BMA2x2_INTR_DATA_SELECT_ADDR            (0x1A)
#define BMA2x2_INTR2_PAD_SELECT_ADDR             (0x1B)
#define BMA2x2_INTR_SOURCE_ADDR                  (0x1E)
#define BMA2x2_INTR_SET_ADDR                     (0x20)
#define BMA2x2_INTR_CTRL_ADDR                    (0x21)
/** FEATURE ADDRESS DEFINITIONS */
#define BMA2x2_LOW_DURN_ADDR                     (0x22)
#define BMA2x2_LOW_THRES_ADDR                    (0x23)
#define BMA2x2_LOW_HIGH_HYST_ADDR                (0x24)
#define BMA2x2_HIGH_DURN_ADDR                    (0x25)
#define BMA2x2_HIGH_THRES_ADDR                   (0x26)
#define BMA2x2_SLOPE_DURN_ADDR                   (0x27)
#define BMA2x2_SLOPE_THRES_ADDR                  (0x28)
#define BMA2x2_SLOW_NO_MOTION_THRES_ADDR         (0x29)
#define BMA2x2_TAP_PARAM_ADDR                    (0x2A)
#define BMA2x2_TAP_THRES_ADDR                    (0x2B)
#define BMA2x2_ORIENT_PARAM_ADDR                 (0x2C)
#define BMA2x2_THETA_BLOCK_ADDR                  (0x2D)
#define BMA2x2_THETA_FLAT_ADDR                   (0x2E)
#define BMA2x2_FLAT_HOLD_TIME_ADDR               (0x2F)
#define BMA2x2_SELFTEST_ADDR                     (0x32)
#define BMA2x2_EEPROM_CTRL_ADDR                  (0x33)
#define BMA2x2_SERIAL_CTRL_ADDR                  (0x34)
/**OFFSET ADDRESS DEFINITIONS */
#define BMA2x2_OFFSET_CTRL_ADDR                  (0x36)
#define BMA2x2_OFFSET_PARAMS_ADDR                (0x37)
#define BMA2x2_OFFSET_X_AXIS_ADDR                (0x38)
#define BMA2x2_OFFSET_Y_AXIS_ADDR                (0x39)
#define BMA2x2_OFFSET_Z_AXIS_ADDR                (0x3A)
/**GP ADDRESS DEFINITIONS */
#define BMA2x2_GP0_ADDR                          (0x3B)
#define BMA2x2_GP1_ADDR                          (0x3C)
/**FIFO ADDRESS DEFINITIONS */
#define BMA2x2_FIFO_MODE_ADDR                    (0x3E)
#define BMA2x2_FIFO_DATA_OUTPUT_ADDR             (0x3F)
#define BMA2x2_FIFO_WML_TRIG                     (0x30)

/**************************************************************/
/**\name	ACCEL RESOLUTION DEFINITION   */
/**************************************************************/
#define BMA2x2_12_RESOLUTION                    (0)
#define BMA2x2_10_RESOLUTION                    (1)
#define BMA2x2_14_RESOLUTION                    (2)

/**************************************************************/
/**\name	ACCEL DELAY DEFINITION   */
/**************************************************************/
/* register write and read delays */
#define BMA2x2_MDELAY_DATA_TYPE                 uint32_t
#define BMA2x2_EE_W_DELAY                       (28)

/**************************************************************/
/**\name	STRUCTURE DEFINITIONS    */
/**************************************************************/
/*!
 *	@brief read accel xyz data for 10,14 and 12 bit resolution
 */
typedef struct {
	int16_t x; /**< accel x data 10,14 and 12 resolution*/
	int16_t y; /**< accel y data 10,14 and 12 resolution*/
	int16_t z; /**< accel z data 10,14 and 12 resolution*/
}bma2x2_xyz_t;

/*!
 *	@brief bma2x2 initialization struct
 *	struct bma2x2_t is used for assigning the following parameters.
 *
 *	Bus write function pointer: BMA2x2_WR_FUNC_PTR
 *	Bus read function pointer: BMA2x2_RD_FUNC_PTR
 *	Burst read function pointer: BMA2x2_BRD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *
 *	I2C address: dev_addr
 *	Chip id of the sensor: chip_id
 */
typedef struct{
	uint8_t power_mode_u8;/**< save current bma2x2 operation mode */
	uint8_t chip_id;/**< chip_id of bma2x2 */
	uint8_t ctrl_mode_reg;/**< the value of power mode register 0x11*/
	uint8_t low_mode_reg;/**< the value of power mode register 0x12*/
	uint8_t dev_addr;/**< initializes bma2x2's I2C device address*/
	uint8_t fifo_config;/**< store the fifo configuration register*/
	sensor_read bus_read;/**< function pointer to the SPI/I2C write function */
	sensor_write bus_write;/**< function pointer to the SPI/I2C read function */
	delay_msec delay;
}bma2x2_t;

/*********************************************************************/
/**\name REGISTER BIT MASK, BIT LENGTH, BIT POSITION DEFINITIONS  */
/********************************************************************/
/******************************/
/**\name CHIP ID  */
/******************************/
#define BMA2x2_CHIP_ID_POS             (0)
#define BMA2x2_CHIP_ID_MSK             (0xFF)
#define BMA2x2_CHIP_ID_LEN             (8)
#define BMA2x2_CHIP_ID_REG             BMA2x2_CHIP_ID_ADDR

/******************************/
/**\name DATA REGISTER-X  */
/******************************/
#define BMA2x2_NEW_DATA_X_POS          (0)
#define BMA2x2_NEW_DATA_X_LEN          (1)
#define BMA2x2_NEW_DATA_X_MSK          (0x01)
#define BMA2x2_NEW_DATA_X_REG          BMA2x2_X_AXIS_LSB_ADDR

#define BMA2x2_ACCEL_X14_LSB_POS          (2)
#define BMA2x2_ACCEL_X14_LSB_LEN          (6)
#define BMA2x2_ACCEL_X14_LSB_MSK          (0xFC)
#define BMA2x2_ACCEL_X14_LSB_REG           BMA2x2_X_AXIS_LSB_ADDR

#define BMA2x2_ACCEL_X12_LSB_POS           (4)
#define BMA2x2_ACCEL_X12_LSB_LEN           (4)
#define BMA2x2_ACCEL_X12_LSB_MSK           (0xF0)
#define BMA2x2_ACCEL_X12_LSB_REG           BMA2x2_X_AXIS_LSB_ADDR

#define BMA2x2_ACCEL_X10_LSB_POS           (6)
#define BMA2x2_ACCEL_X10_LSB_LEN           (2)
#define BMA2x2_ACCEL_X10_LSB_MSK           (0xC0)
#define BMA2x2_ACCEL_X10_LSB_REG           BMA2x2_X_AXIS_LSB_ADDR

#define BMA2x2_ACCEL_X8_LSB_POS           (0)
#define BMA2x2_ACCEL_X8_LSB_LEN           (0)
#define BMA2x2_ACCEL_X8_LSB_MSK           (0x00)
#define BMA2x2_ACCEL_X8_LSB_REG           BMA2x2_X_AXIS_LSB_ADDR

#define BMA2x2_ACCEL_X_MSB_POS           (0)
#define BMA2x2_ACCEL_X_MSB_LEN           (8)
#define BMA2x2_ACCEL_X_MSB_MSK           (0xFF)
#define BMA2x2_ACCEL_X_MSB_REG           BMA2x2_X_AXIS_MSB_ADDR
/******************************/
/**\name DATA REGISTER-Y  */
/******************************/
#define BMA2x2_NEW_DATA_Y_POS          (0)
#define BMA2x2_NEW_DATA_Y_LEN          (1)
#define BMA2x2_NEW_DATA_Y_MSK          (0x01)
#define BMA2x2_NEW_DATA_Y_REG          BMA2x2_Y_AXIS_LSB_ADDR

#define BMA2x2_ACCEL_Y14_LSB_POS           (2)
#define BMA2x2_ACCEL_Y14_LSB_LEN           (6)
#define BMA2x2_ACCEL_Y14_LSB_MSK           (0xFC)
#define BMA2x2_ACCEL_Y14_LSB_REG           BMA2x2_Y_AXIS_LSB_ADDR

#define BMA2x2_ACCEL_Y12_LSB_POS           (4)
#define BMA2x2_ACCEL_Y12_LSB_LEN           (4)
#define BMA2x2_ACCEL_Y12_LSB_MSK           (0xF0)
#define BMA2x2_ACCEL_Y12_LSB_REG           BMA2x2_Y_AXIS_LSB_ADDR

#define BMA2x2_ACCEL_Y10_LSB_POS           (6)
#define BMA2x2_ACCEL_Y10_LSB_LEN           (2)
#define BMA2x2_ACCEL_Y10_LSB_MSK           (0xC0)
#define BMA2x2_ACCEL_Y10_LSB_REG           BMA2x2_Y_AXIS_LSB_ADDR

#define BMA2x2_ACCEL_Y8_LSB_POS           (0)
#define BMA2x2_ACCEL_Y8_LSB_LEN           (0)
#define BMA2x2_ACCEL_Y8_LSB_MSK           (0x00)
#define BMA2x2_ACCEL_Y8_LSB_REG           BMA2x2_Y_AXIS_LSB_ADDR

#define BMA2x2_ACCEL_Y_MSB_POS           (0)
#define BMA2x2_ACCEL_Y_MSB_LEN           (8)
#define BMA2x2_ACCEL_Y_MSB_MSK           (0xFF)
#define BMA2x2_ACCEL_Y_MSB_REG           BMA2x2_Y_AXIS_MSB_ADDR
/******************************/
/**\name DATA REGISTER-Z  */
/******************************/
#define BMA2x2_NEW_DATA_Z_POS          (0)
#define BMA2x2_NEW_DATA_Z_LEN          (1)
#define BMA2x2_NEW_DATA_Z_MSK          (0x01)
#define BMA2x2_NEW_DATA_Z_REG          BMA2x2_Z_AXIS_LSB_ADDR

#define BMA2x2_ACCEL_Z14_LSB_POS           (2)
#define BMA2x2_ACCEL_Z14_LSB_LEN           (6)
#define BMA2x2_ACCEL_Z14_LSB_MSK           (0xFC)
#define BMA2x2_ACCEL_Z14_LSB_REG           BMA2x2_Z_AXIS_LSB_ADDR

#define BMA2x2_ACCEL_Z12_LSB_POS           (4)
#define BMA2x2_ACCEL_Z12_LSB_LEN           (4)
#define BMA2x2_ACCEL_Z12_LSB_MSK           (0xF0)
#define BMA2x2_ACCEL_Z12_LSB_REG           BMA2x2_Z_AXIS_LSB_ADDR

#define BMA2x2_ACCEL_Z10_LSB_POS           (6)
#define BMA2x2_ACCEL_Z10_LSB_LEN           (2)
#define BMA2x2_ACCEL_Z10_LSB_MSK           (0xC0)
#define BMA2x2_ACCEL_Z10_LSB_REG           BMA2x2_Z_AXIS_LSB_ADDR

#define BMA2x2_ACCEL_Z8_LSB_POS           (0)
#define BMA2x2_ACCEL_Z8_LSB_LEN           (0)
#define BMA2x2_ACCEL_Z8_LSB_MSK           (0x00)
#define BMA2x2_ACCEL_Z8_LSB_REG           BMA2x2_Z_AXIS_LSB_ADDR

#define BMA2x2_ACCEL_Z_MSB_POS           (0)
#define BMA2x2_ACCEL_Z_MSB_LEN           (8)
#define BMA2x2_ACCEL_Z_MSB_MSK           (0xFF)
#define BMA2x2_ACCEL_Z_MSB_REG           BMA2x2_Z_AXIS_MSB_ADDR

/******************************/
/**\name TEMPERATURE */
/******************************/
#define BMA2x2_ACCEL_TEMP_MSB_POS           (0)
#define BMA2x2_ACCEL_TEMP_MSB_LEN           (8)
#define BMA2x2_ACCEL_TEMP_MSB_MSK           (0xFF)
#define BMA2x2_ACCEL_TEMP_MSB_REG           BMA2x2_TEMPERATURE_REG

/***************************************/
/**\name INTERRUPT STATUS OF LOW-G */
/**************************************/
#define BMA2x2_LOW_G_INTR_STAT_POS          (0)
#define BMA2x2_LOW_G_INTR_STAT_LEN          (1)
#define BMA2x2_LOW_G_INTR_STAT_MSK          (0x01)
#define BMA2x2_LOW_G_INTR_STAT_REG          BMA2x2_STAT1_ADDR
/***************************************/
/**\name INTERRUPT STATUS OF HIGH-G */
/**************************************/
#define BMA2x2_HIGH_G_INTR_STAT_POS          (1)
#define BMA2x2_HIGH_G_INTR_STAT_LEN          (1)
#define BMA2x2_HIGH_G_INTR_STAT_MSK          (0x02)
#define BMA2x2_HIGH_G_INTR_STAT_REG          BMA2x2_STAT1_ADDR
/***************************************/
/**\name INTERRUPT STATUS OF SLOPE */
/**************************************/
#define BMA2x2_SLOPE_INTR_STAT_POS          (2)
#define BMA2x2_SLOPE_INTR_STAT_LEN          (1)
#define BMA2x2_SLOPE_INTR_STAT_MSK          (0x04)
#define BMA2x2_SLOPE_INTR_STAT_REG          BMA2x2_STAT1_ADDR
/*******************************************/
/**\name INTERRUPT STATUS OF SLOW NO MOTION*/
/*******************************************/
#define BMA2x2_SLOW_NO_MOTION_INTR_STAT_POS          (3)
#define BMA2x2_SLOW_NO_MOTION_INTR_STAT_LEN          (1)
#define BMA2x2_SLOW_NO_MOTION_INTR_STAT_MSK          (0x08)
#define BMA2x2_SLOW_NO_MOTION_INTR_STAT_REG          BMA2x2_STAT1_ADDR
/***************************************/
/**\name INTERRUPT STATUS OF DOUBLE TAP */
/**************************************/
#define BMA2x2_DOUBLE_TAP_INTR_STAT_POS     (4)
#define BMA2x2_DOUBLE_TAP_INTR_STAT_LEN     (1)
#define BMA2x2_DOUBLE_TAP_INTR_STAT_MSK     (0x10)
#define BMA2x2_DOUBLE_TAP_INTR_STAT_REG     BMA2x2_STAT1_ADDR
/***************************************/
/**\name INTERRUPT STATUS OF SINGLE TAP */
/**************************************/
#define BMA2x2_SINGLE_TAP_INTR_STAT_POS     (5)
#define BMA2x2_SINGLE_TAP_INTR_STAT_LEN     (1)
#define BMA2x2_SINGLE_TAP_INTR_STAT_MSK     (0x20)
#define BMA2x2_SINGLE_TAP_INTR_STAT_REG     BMA2x2_STAT1_ADDR
/***************************************/
/**\name INTERRUPT STATUS OF ORIENT*/
/**************************************/
#define BMA2x2_ORIENT_INTR_STAT_POS         (6)
#define BMA2x2_ORIENT_INTR_STAT_LEN         (1)
#define BMA2x2_ORIENT_INTR_STAT_MSK         (0x40)
#define BMA2x2_ORIENT_INTR_STAT_REG         BMA2x2_STAT1_ADDR
/***************************************/
/**\name INTERRUPT STATUS OF FLAT */
/**************************************/
#define BMA2x2_FLAT_INTR_STAT_POS           (7)
#define BMA2x2_FLAT_INTR_STAT_LEN           (1)
#define BMA2x2_FLAT_INTR_STAT_MSK           (0x80)
#define BMA2x2_FLAT_INTR_STAT_REG           BMA2x2_STAT1_ADDR
/***************************************/
/**\name INTERRUPT STATUS OF FIFO FULL */
/**************************************/
#define BMA2x2_FIFO_FULL_INTR_STAT_POS           (5)
#define BMA2x2_FIFO_FULL_INTR_STAT_LEN           (1)
#define BMA2x2_FIFO_FULL_INTR_STAT_MSK           (0x20)
#define BMA2x2_FIFO_FULL_INTR_STAT_REG           BMA2x2_STAT2_ADDR
/*******************************************/
/**\name INTERRUPT STATUS OF FIFO WATERMARK*/
/******************************************/
#define BMA2x2_FIFO_WM_INTR_STAT_POS           (6)
#define BMA2x2_FIFO_WM_INTR_STAT_LEN           (1)
#define BMA2x2_FIFO_WM_INTR_STAT_MSK           (0x40)
#define BMA2x2_FIFO_WM_INTR_STAT_REG           BMA2x2_STAT2_ADDR
/***************************************/
/**\name INTERRUPT STATUS OF DATA */
/**************************************/
#define BMA2x2_DATA_INTR_STAT_POS           (7)
#define BMA2x2_DATA_INTR_STAT_LEN           (1)
#define BMA2x2_DATA_INTR_STAT_MSK           (0x80)
#define BMA2x2_DATA_INTR_STAT_REG           BMA2x2_STAT2_ADDR
/*********************************************/
/**\name INTERRUPT STATUS SLOPE XYZ AND SIGN */
/*********************************************/
#define BMA2x2_SLOPE_FIRST_X_POS        (0)
#define BMA2x2_SLOPE_FIRST_X_LEN        (1)
#define BMA2x2_SLOPE_FIRST_X_MSK        (0x01)
#define BMA2x2_SLOPE_FIRST_X_REG        BMA2x2_STAT_TAP_SLOPE_ADDR

#define BMA2x2_SLOPE_FIRST_Y_POS        (1)
#define BMA2x2_SLOPE_FIRST_Y_LEN        (1)
#define BMA2x2_SLOPE_FIRST_Y_MSK        (0x02)
#define BMA2x2_SLOPE_FIRST_Y_REG        BMA2x2_STAT_TAP_SLOPE_ADDR

#define BMA2x2_SLOPE_FIRST_Z_POS        (2)
#define BMA2x2_SLOPE_FIRST_Z_LEN        (1)
#define BMA2x2_SLOPE_FIRST_Z_MSK        (0x04)
#define BMA2x2_SLOPE_FIRST_Z_REG        BMA2x2_STAT_TAP_SLOPE_ADDR

#define BMA2x2_SLOPE_SIGN_STAT_POS         (3)
#define BMA2x2_SLOPE_SIGN_STAT_LEN         (1)
#define BMA2x2_SLOPE_SIGN_STAT_MSK         (0x08)
#define BMA2x2_SLOPE_SIGN_STAT_REG         BMA2x2_STAT_TAP_SLOPE_ADDR
/*********************************************/
/**\name INTERRUPT STATUS TAP XYZ AND SIGN */
/*********************************************/
#define BMA2x2_TAP_FIRST_X_POS        (4)
#define BMA2x2_TAP_FIRST_X_LEN        (1)
#define BMA2x2_TAP_FIRST_X_MSK        (0x10)
#define BMA2x2_TAP_FIRST_X_REG        BMA2x2_STAT_TAP_SLOPE_ADDR

#define BMA2x2_TAP_FIRST_Y_POS        (5)
#define BMA2x2_TAP_FIRST_Y_LEN        (1)
#define BMA2x2_TAP_FIRST_Y_MSK        (0x20)
#define BMA2x2_TAP_FIRST_Y_REG        BMA2x2_STAT_TAP_SLOPE_ADDR

#define BMA2x2_TAP_FIRST_Z_POS        (6)
#define BMA2x2_TAP_FIRST_Z_LEN        (1)
#define BMA2x2_TAP_FIRST_Z_MSK        (0x40)
#define BMA2x2_TAP_FIRST_Z_REG        BMA2x2_STAT_TAP_SLOPE_ADDR

#define BMA2x2_TAP_SIGN_STAT_POS         (7)
#define BMA2x2_TAP_SIGN_STAT_LEN         (1)
#define BMA2x2_TAP_SIGN_STAT_MSK         (0x80)
#define BMA2x2_TAP_SIGN_STAT_REG         BMA2x2_STAT_TAP_SLOPE_ADDR
/*********************************************/
/**\name INTERRUPT STATUS HIGH_G XYZ AND SIGN */
/*********************************************/
#define BMA2x2_HIGH_G_FIRST_X_POS        (0)
#define BMA2x2_HIGH_G_FIRST_X_LEN        (1)
#define BMA2x2_HIGH_G_FIRST_X_MSK        (0x01)
#define BMA2x2_HIGH_G_FIRST_X_REG        BMA2x2_STAT_ORIENT_HIGH_ADDR

#define BMA2x2_HIGH_G_FIRST_Y_POS        (1)
#define BMA2x2_HIGH_G_FIRST_Y_LEN        (1)
#define BMA2x2_HIGH_G_FIRST_Y_MSK        (0x02)
#define BMA2x2_HIGH_G_FIRST_Y_REG        BMA2x2_STAT_ORIENT_HIGH_ADDR

#define BMA2x2_HIGH_G_FIRST_Z_POS        (2)
#define BMA2x2_HIGH_G_FIRST_Z_LEN        (1)
#define BMA2x2_HIGH_G_FIRST_Z_MSK        (0x04)
#define BMA2x2_HIGH_G_FIRST_Z_REG        BMA2x2_STAT_ORIENT_HIGH_ADDR

#define BMA2x2_HIGH_G_SIGN_STAT_POS         (3)
#define BMA2x2_HIGH_G_SIGN_STAT_LEN         (1)
#define BMA2x2_HIGH_G_SIGN_STAT_MSK         (0x08)
#define BMA2x2_HIGH_G_SIGN_STAT_REG         BMA2x2_STAT_ORIENT_HIGH_ADDR
/*********************************************/
/**\name INTERRUPT STATUS ORIENT */
/*********************************************/
#define BMA2x2_ORIENT_STAT_POS             (4)
#define BMA2x2_ORIENT_STAT_LEN             (3)
#define BMA2x2_ORIENT_STAT_MSK             (0x70)
#define BMA2x2_ORIENT_STAT_REG             BMA2x2_STAT_ORIENT_HIGH_ADDR
/*********************************************/
/**\name INTERRUPT STATUS FLAT */
/*********************************************/
#define BMA2x2_FLAT_STAT_POS               (7)
#define BMA2x2_FLAT_STAT_LEN               (1)
#define BMA2x2_FLAT_STAT_MSK               (0x80)
#define BMA2x2_FLAT_STAT_REG               BMA2x2_STAT_ORIENT_HIGH_ADDR

/*********************************************/
/**\name INTERRUPT STATUS OF FIFO FRAME COUNT */
/*********************************************/
#define BMA2x2_FIFO_FRAME_COUNT_STAT_POS             (0)
#define BMA2x2_FIFO_FRAME_COUNT_STAT_LEN             (7)
#define BMA2x2_FIFO_FRAME_COUNT_STAT_MSK             (0x7F)
#define BMA2x2_FIFO_FRAME_COUNT_STAT_REG             BMA2x2_STAT_FIFO_ADDR
/*********************************************/
/**\name INTERRUPT STATUS OF FIFO OVERRUN */
/*********************************************/
#define BMA2x2_FIFO_OVERRUN_STAT_POS             (7)
#define BMA2x2_FIFO_OVERRUN_STAT_LEN             (1)
#define BMA2x2_FIFO_OVERRUN_STAT_MSK             (0x80)
#define BMA2x2_FIFO_OVERRUN_STAT_REG             BMA2x2_STAT_FIFO_ADDR
/****************************/
/**\name RANGE */
/****************************/
#define BMA2x2_RANGE_SELECT_POS             (0)
#define BMA2x2_RANGE_SELECT_LEN             (4)
#define BMA2x2_RANGE_SELECT_MSK             (0x0F)
#define BMA2x2_RANGE_SELECT_REG             BMA2x2_RANGE_SELECT_ADDR
/****************************/
/**\name BANDWIDTH */
/****************************/
#define BMA2x2_BW_POS             (0)
#define BMA2x2_BW_LEN             (5)
#define BMA2x2_BW_MSK             (0x1F)
#define BMA2x2_BW_REG             BMA2x2_BW_SELECT_ADDR
/****************************/
/**\name SLEEP DURATION */
/****************************/
#define BMA2x2_SLEEP_DURN_POS             (1)
#define BMA2x2_SLEEP_DURN_LEN             (4)
#define BMA2x2_SLEEP_DURN_MSK             (0x1E)
#define BMA2x2_SLEEP_DURN_REG             BMA2x2_MODE_CTRL_ADDR
/****************************/
/**\name POWER MODEPOWER MODE */
/****************************/
#define BMA2x2_MODE_CTRL_POS             (5)
#define BMA2x2_MODE_CTRL_LEN             (3)
#define BMA2x2_MODE_CTRL_MSK             (0xE0)
#define BMA2x2_MODE_CTRL_REG             BMA2x2_MODE_CTRL_ADDR
/****************************/
/**\name SLEEP TIMER */
/****************************/
#define BMA2x2_SLEEP_TIMER_POS          (5)
#define BMA2x2_SLEEP_TIMER_LEN          (1)
#define BMA2x2_SLEEP_TIMER_MSK          (0x20)
#define BMA2x2_SLEEP_TIMER_REG          BMA2x2_LOW_NOISE_CTRL_ADDR
/****************************/
/**\name LOWPOWER MODE */
/****************************/
#define BMA2x2_LOW_POWER_MODE_POS          (6)
#define BMA2x2_LOW_POWER_MODE_LEN          (1)
#define BMA2x2_LOW_POWER_MODE_MSK          (0x40)
#define BMA2x2_LOW_POWER_MODE_REG          BMA2x2_LOW_NOISE_CTRL_ADDR
/*******************************************/
/**\name DISABLE MSB SHADOWING PROCEDURE  */
/*******************************************/
#define BMA2x2_DIS_SHADOW_PROC_POS       (6)
#define BMA2x2_DIS_SHADOW_PROC_LEN       (1)
#define BMA2x2_DIS_SHADOW_PROC_MSK       (0x40)
#define BMA2x2_DIS_SHADOW_PROC_REG       BMA2x2_DATA_CTRL_ADDR
/***************************************************/
/**\name FILTERED OR UNFILTERED ACCELERATION DATA   */
/***************************************************/
#define BMA2x2_ENABLE_DATA_HIGH_BW_POS         (7)
#define BMA2x2_ENABLE_DATA_HIGH_BW_LEN         (1)
#define BMA2x2_ENABLE_DATA_HIGH_BW_MSK         (0x80)
#define BMA2x2_ENABLE_DATA_HIGH_BW_REG         BMA2x2_DATA_CTRL_ADDR
/***************************************************/
/**\name SOFT RESET VALUE   */
/***************************************************/
#define BMA2x2_ENABLE_SOFT_RESET_VALUE        (0xB6)
/**********************************************/
/**\name INTERRUPT ENABLE OF SLOPE-XYZ   */
/**********************************************/
#define BMA2x2_ENABLE_SLOPE_X_INTR_POS         (0)
#define BMA2x2_ENABLE_SLOPE_X_INTR_LEN         (1)
#define BMA2x2_ENABLE_SLOPE_X_INTR_MSK         (0x01)
#define BMA2x2_ENABLE_SLOPE_X_INTR_REG         BMA2x2_INTR_ENABLE1_ADDR

#define BMA2x2_ENABLE_SLOPE_Y_INTR_POS         (1)
#define BMA2x2_ENABLE_SLOPE_Y_INTR_LEN         (1)
#define BMA2x2_ENABLE_SLOPE_Y_INTR_MSK         (0x02)
#define BMA2x2_ENABLE_SLOPE_Y_INTR_REG         BMA2x2_INTR_ENABLE1_ADDR

#define BMA2x2_ENABLE_SLOPE_Z_INTR_POS         (2)
#define BMA2x2_ENABLE_SLOPE_Z_INTR_LEN         (1)
#define BMA2x2_ENABLE_SLOPE_Z_INTR_MSK         (0x04)
#define BMA2x2_ENABLE_SLOPE_Z_INTR_REG         BMA2x2_INTR_ENABLE1_ADDR
/**********************************************/
/**\name INTERRUPT ENABLE OF DOUBLE TAP   */
/**********************************************/
#define BMA2x2_ENABLE_DOUBLE_TAP_INTR_POS      (4)
#define BMA2x2_ENABLE_DOUBLE_TAP_INTR_LEN      (1)
#define BMA2x2_ENABLE_DOUBLE_TAP_INTR_MSK      (0x10)
#define BMA2x2_ENABLE_DOUBLE_TAP_INTR_REG      BMA2x2_INTR_ENABLE1_ADDR
/**********************************************/
/**\name INTERRUPT ENABLE OF SINGLE TAP   */
/**********************************************/
#define BMA2x2_ENABLE_SINGLE_TAP_INTR_POS      (5)
#define BMA2x2_ENABLE_SINGLE_TAP_INTR_LEN      (1)
#define BMA2x2_ENABLE_SINGLE_TAP_INTR_MSK      (0x20)
#define BMA2x2_ENABLE_SINGLE_TAP_INTR_REG      BMA2x2_INTR_ENABLE1_ADDR
/**********************************************/
/**\name INTERRUPT ENABLE OF ORIENT  */
/**********************************************/
#define BMA2x2_ENABLE_ORIENT_INTR_POS          (6)
#define BMA2x2_ENABLE_ORIENT_INTR_LEN          (1)
#define BMA2x2_ENABLE_ORIENT_INTR_MSK          (0x40)
#define BMA2x2_ENABLE_ORIENT_INTR_REG          BMA2x2_INTR_ENABLE1_ADDR
/**********************************************/
/**\name INTERRUPT ENABLE OF FLAT  */
/**********************************************/
#define BMA2x2_ENABLE_FLAT_INTR_POS            (7)
#define BMA2x2_ENABLE_FLAT_INTR_LEN            (1)
#define BMA2x2_ENABLE_FLAT_INTR_MSK            (0x80)
#define BMA2x2_ENABLE_FLAT_INTR_REG            BMA2x2_INTR_ENABLE1_ADDR
/**********************************************/
/**\name INTERRUPT ENABLE OF HIGH_G-XYZ   */
/**********************************************/
#define BMA2x2_ENABLE_HIGH_G_X_INTR_POS         (0)
#define BMA2x2_ENABLE_HIGH_G_X_INTR_LEN         (1)
#define BMA2x2_ENABLE_HIGH_G_X_INTR_MSK         (0x01)
#define BMA2x2_ENABLE_HIGH_G_X_INTR_REG         BMA2x2_INTR_ENABLE2_ADDR

#define BMA2x2_ENABLE_HIGH_G_Y_INTR_POS         (1)
#define BMA2x2_ENABLE_HIGH_G_Y_INTR_LEN         (1)
#define BMA2x2_ENABLE_HIGH_G_Y_INTR_MSK         (0x02)
#define BMA2x2_ENABLE_HIGH_G_Y_INTR_REG         BMA2x2_INTR_ENABLE2_ADDR

#define BMA2x2_ENABLE_HIGH_G_Z_INTR_POS         (2)
#define BMA2x2_ENABLE_HIGH_G_Z_INTR_LEN         (1)
#define BMA2x2_ENABLE_HIGH_G_Z_INTR_MSK         (0x04)
#define BMA2x2_ENABLE_HIGH_G_Z_INTR_REG         BMA2x2_INTR_ENABLE2_ADDR
/**********************************************/
/**\name INTERRUPT ENABLE OF LOW_G  */
/**********************************************/
#define BMA2x2_ENABLE_LOW_G_INTR_POS            (3)
#define BMA2x2_ENABLE_LOW_G_INTR_LEN            (1)
#define BMA2x2_ENABLE_LOW_G_INTR_MSK            (0x08)
#define BMA2x2_ENABLE_LOW_G_INTR_REG            BMA2x2_INTR_ENABLE2_ADDR
/**********************************************/
/**\name INTERRUPT ENABLE OF DATA   */
/**********************************************/
#define BMA2x2_ENABLE_NEW_DATA_INTR_POS        (4)
#define BMA2x2_ENABLE_NEW_DATA_INTR_LEN        (1)
#define BMA2x2_ENABLE_NEW_DATA_INTR_MSK        (0x10)
#define BMA2x2_ENABLE_NEW_DATA_INTR_REG        BMA2x2_INTR_ENABLE2_ADDR
/**********************************************/
/**\name INTERRUPT ENABLE OF FIFO FULL   */
/**********************************************/
#define BMA2x2_INTR_FIFO_FULL_ENABLE_INTR_POS        (5)
#define BMA2x2_INTR_FIFO_FULL_ENABLE_INTR_LEN        (1)
#define BMA2x2_INTR_FIFO_FULL_ENABLE_INTR_MSK        (0x20)
#define BMA2x2_INTR_FIFO_FULL_ENABLE_INTR_REG        BMA2x2_INTR_ENABLE2_ADDR
/**********************************************/
/**\name INTERRUPT ENABLE OF FIFO WATER MARK   */
/**********************************************/
#define BMA2x2_INTR_FIFO_WM_ENABLE_INTR_POS        (6)
#define BMA2x2_INTR_FIFO_WM_ENABLE_INTR_LEN        (1)
#define BMA2x2_INTR_FIFO_WM_ENABLE_INTR_MSK        (0x40)
#define BMA2x2_INTR_FIFO_WM_ENABLE_INTR_REG        BMA2x2_INTR_ENABLE2_ADDR
/************************************************/
/**\name INTERRUPT ENABLE OF SLOW NO MOTION-XYZ */
/*************************************************/
#define BMA2x2_INTR_SLOW_NO_MOTION_ENABLE_X_INTR_POS        (0)
#define BMA2x2_INTR_SLOW_NO_MOTION_ENABLE_X_INTR_LEN        (1)
#define BMA2x2_INTR_SLOW_NO_MOTION_ENABLE_X_INTR_MSK        (0x01)
#define BMA2x2_INTR_SLOW_NO_MOTION_ENABLE_X_INTR_REG        \
BMA2x2_INTR_SLOW_NO_MOTION_ADDR

#define BMA2x2_INTR_SLOW_NO_MOTION_ENABLE_Y_INTR_POS        (1)
#define BMA2x2_INTR_SLOW_NO_MOTION_ENABLE_Y_INTR_LEN        (1)
#define BMA2x2_INTR_SLOW_NO_MOTION_ENABLE_Y_INTR_MSK        (0x02)
#define BMA2x2_INTR_SLOW_NO_MOTION_ENABLE_Y_INTR_REG        \
BMA2x2_INTR_SLOW_NO_MOTION_ADDR

#define BMA2x2_INTR_SLOW_NO_MOTION_ENABLE_Z_INTR_POS        (2)
#define BMA2x2_INTR_SLOW_NO_MOTION_ENABLE_Z_INTR_LEN        (1)
#define BMA2x2_INTR_SLOW_NO_MOTION_ENABLE_Z_INTR_MSK        (0x04)
#define BMA2x2_INTR_SLOW_NO_MOTION_ENABLE_Z_INTR_REG        \
BMA2x2_INTR_SLOW_NO_MOTION_ADDR
/**********************************************/
/**\name INTERRUPT ENABLE OF SLOW NO MOTION SELECT */
/**********************************************/
#define BMA2x2_INTR_SLOW_NO_MOTION_ENABLE_SELECT_INTR_POS        (3)
#define BMA2x2_INTR_SLOW_NO_MOTION_ENABLE_SELECT_INTR_LEN        (1)
#define BMA2x2_INTR_SLOW_NO_MOTION_ENABLE_SELECT_INTR_MSK        (0x08)
#define BMA2x2_INTR_SLOW_NO_MOTION_ENABLE_SELECT_INTR_REG        \
BMA2x2_INTR_SLOW_NO_MOTION_ADDR
/**********************************************/
/**\name INTERRUPT1 ENABLE OF PAD LOW_G */
/**********************************************/
#define BMA2x2_ENABLE_INTR1_PAD_LOW_G_POS        (0)
#define BMA2x2_ENABLE_INTR1_PAD_LOW_G_LEN        (1)
#define BMA2x2_ENABLE_INTR1_PAD_LOW_G_MSK        (0x01)
#define BMA2x2_ENABLE_INTR1_PAD_LOW_G_REG        BMA2x2_INTR1_PAD_SELECT_ADDR
/**********************************************/
/**\name INTERRUPT1 ENABLE OF PAD HIGH_G */
/**********************************************/
#define BMA2x2_ENABLE_INTR1_PAD_HIGH_G_POS       (1)
#define BMA2x2_ENABLE_INTR1_PAD_HIGH_G_LEN       (1)
#define BMA2x2_ENABLE_INTR1_PAD_HIGH_G_MSK       (0x02)
#define BMA2x2_ENABLE_INTR1_PAD_HIGH_G_REG       BMA2x2_INTR1_PAD_SELECT_ADDR
/**********************************************/
/**\name INTERRUPT1 ENABLE OF PAD SLOPE */
/**********************************************/
#define BMA2x2_ENABLE_INTR1_PAD_SLOPE_POS       (2)
#define BMA2x2_ENABLE_INTR1_PAD_SLOPE_LEN       (1)
#define BMA2x2_ENABLE_INTR1_PAD_SLOPE_MSK       (0x04)
#define BMA2x2_ENABLE_INTR1_PAD_SLOPE_REG       BMA2x2_INTR1_PAD_SELECT_ADDR
/**********************************************/
/**\name INTERRUPT1 ENABLE OF SLOW NO MOTION  */
/**********************************************/
#define BMA2x2_ENABLE_INTR1_PAD_SLOW_NO_MOTION_POS        (3)
#define BMA2x2_ENABLE_INTR1_PAD_SLOW_NO_MOTION_LEN        (1)
#define BMA2x2_ENABLE_INTR1_PAD_SLOW_NO_MOTION_MSK        (0x08)
#define BMA2x2_ENABLE_INTR1_PAD_SLOW_NO_MOTION_REG        \
BMA2x2_INTR1_PAD_SELECT_ADDR
/**********************************************/
/**\name INTERRUPT1 ENABLE OF PAD DOUBLE_TAP */
/**********************************************/
#define BMA2x2_ENABLE_INTR1_PAD_DOUBLE_TAP_POS      (4)
#define BMA2x2_ENABLE_INTR1_PAD_DOUBLE_TAP_LEN      (1)
#define BMA2x2_ENABLE_INTR1_PAD_DOUBLE_TAP_MSK      (0x10)
#define BMA2x2_ENABLE_INTR1_PAD_DOUBLE_TAP_REG      BMA2x2_INTR1_PAD_SELECT_ADDR
/**********************************************/
/**\name INTERRUPT1 ENABLE OF PAD SINGLE_TAP */
/**********************************************/
#define BMA2x2_ENABLE_INTR1_PAD_SINGLE_TAP_POS     (5)
#define BMA2x2_ENABLE_INTR1_PAD_SINGLE_TAP_LEN     (1)
#define BMA2x2_ENABLE_INTR1_PAD_SINGLE_TAP_MSK     (0x20)
#define BMA2x2_ENABLE_INTR1_PAD_SINGLE_TAP_REG     BMA2x2_INTR1_PAD_SELECT_ADDR
/**********************************************/
/**\name INTERRUPT1 ENABLE OF PAD ORIENT*/
/**********************************************/
#define BMA2x2_ENABLE_INTR1_PAD_ORIENT_POS      (6)
#define BMA2x2_ENABLE_INTR1_PAD_ORIENT_LEN      (1)
#define BMA2x2_ENABLE_INTR1_PAD_ORIENT_MSK      (0x40)
#define BMA2x2_ENABLE_INTR1_PAD_ORIENT_REG      BMA2x2_INTR1_PAD_SELECT_ADDR
/**********************************************/
/**\name INTERRUPT1 ENABLE OF PAD FLAT */
/**********************************************/
#define BMA2x2_ENABLE_INTR1_PAD_FLAT_POS        (7)
#define BMA2x2_ENABLE_INTR1_PAD_FLAT_LEN        (1)
#define BMA2x2_ENABLE_INTR1_PAD_FLAT_MSK        (0x80)
#define BMA2x2_ENABLE_INTR1_PAD_FLAT_REG        BMA2x2_INTR1_PAD_SELECT_ADDR
/**********************************************/
/**\name INTERRUPT2 ENABLE OF PAD LOW_G */
/**********************************************/
#define BMA2x2_ENABLE_INTR2_PAD_LOW_G_POS        (0)
#define BMA2x2_ENABLE_INTR2_PAD_LOW_G_LEN        (1)
#define BMA2x2_ENABLE_INTR2_PAD_LOW_G_MSK        (0x01)
#define BMA2x2_ENABLE_INTR2_PAD_LOW_G_REG        BMA2x2_INTR2_PAD_SELECT_ADDR
/**********************************************/
/**\name INTERRUPT2 ENABLE OF PAD HIGH_G */
/**********************************************/
#define BMA2x2_ENABLE_INTR2_PAD_HIGH_G_POS       (1)
#define BMA2x2_ENABLE_INTR2_PAD_HIGH_G_LEN       (1)
#define BMA2x2_ENABLE_INTR2_PAD_HIGH_G_MSK       (0x02)
#define BMA2x2_ENABLE_INTR2_PAD_HIGH_G_REG       BMA2x2_INTR2_PAD_SELECT_ADDR
/**********************************************/
/**\name INTERRUPT2 ENABLE OF PAD SLOPE */
/**********************************************/
#define BMA2x2_ENABLE_INTR2_PAD_SLOPE_POS       (2)
#define BMA2x2_ENABLE_INTR2_PAD_SLOPE_LEN       (1)
#define BMA2x2_ENABLE_INTR2_PAD_SLOPE_MSK       (0x04)
#define BMA2x2_ENABLE_INTR2_PAD_SLOPE_REG       BMA2x2_INTR2_PAD_SELECT_ADDR
/**********************************************/
/**\name INTERRUPT2 ENABLE OF PAD SLOW NO MOTION */
/**********************************************/
#define BMA2x2_ENABLE_INTR2_PAD_SLOW_NO_MOTION_POS        (3)
#define BMA2x2_ENABLE_INTR2_PAD_SLOW_NO_MOTION_LEN        (1)
#define BMA2x2_ENABLE_INTR2_PAD_SLOW_NO_MOTION_MSK        (0x08)
#define BMA2x2_ENABLE_INTR2_PAD_SLOW_NO_MOTION_REG        \
BMA2x2_INTR2_PAD_SELECT_ADDR
/**********************************************/
/**\name INTERRUPT2 ENABLE OF PAD DOUBLE_TAP */
/**********************************************/
#define BMA2x2_ENABLE_INTR2_PAD_DOUBLE_TAP_POS      (4)
#define BMA2x2_ENABLE_INTR2_PAD_DOUBLE_TAP_LEN      (1)
#define BMA2x2_ENABLE_INTR2_PAD_DOUBLE_TAP_MSK      (0x10)
#define BMA2x2_ENABLE_INTR2_PAD_DOUBLE_TAP_REG      BMA2x2_INTR2_PAD_SELECT_ADDR
/**********************************************/
/**\name INTERRUPT2 ENABLE OF PAD SINGLE_TAP */
/**********************************************/
#define BMA2x2_ENABLE_INTR2_PAD_SINGLE_TAP_POS     (5)
#define BMA2x2_ENABLE_INTR2_PAD_SINGLE_TAP_LEN     (1)
#define BMA2x2_ENABLE_INTR2_PAD_SINGLE_TAP_MSK     (0x20)
#define BMA2x2_ENABLE_INTR2_PAD_SINGLE_TAP_REG     BMA2x2_INTR2_PAD_SELECT_ADDR
/**********************************************/
/**\name INTERRUPT2 ENABLE OF PAD ORIENT */
/**********************************************/
#define BMA2x2_ENABLE_INTR2_PAD_ORIENT_POS      (6)
#define BMA2x2_ENABLE_INTR2_PAD_ORIENT_LEN      (1)
#define BMA2x2_ENABLE_INTR2_PAD_ORIENT_MSK      (0x40)
#define BMA2x2_ENABLE_INTR2_PAD_ORIENT_REG      BMA2x2_INTR2_PAD_SELECT_ADDR
/**********************************************/
/**\name INTERRUPT2 ENABLE OF PAD FLAT */
/**********************************************/
#define BMA2x2_ENABLE_INTR2_PAD_FLAT_POS        (7)
#define BMA2x2_ENABLE_INTR2_PAD_FLAT_LEN        (1)
#define BMA2x2_ENABLE_INTR2_PAD_FLAT_MSK        (0x80)
#define BMA2x2_ENABLE_INTR2_PAD_FLAT_REG        BMA2x2_INTR2_PAD_SELECT_ADDR
/**********************************************/
/**\name INTERRUPT2 ENABLE OF PAD DATA */
/**********************************************/
#define BMA2x2_ENABLE_INTR1_PAD_NEWDATA_POS     (0)
#define BMA2x2_ENABLE_INTR1_PAD_NEWDATA_LEN     (1)
#define BMA2x2_ENABLE_INTR1_PAD_NEWDATA_MSK     (0x01)
#define BMA2x2_ENABLE_INTR1_PAD_NEWDATA_REG     BMA2x2_INTR_DATA_SELECT_ADDR
/**********************************************/
/**\name INTERRUPT1 ENABLE OF FIFO WATER MARK */
/**********************************************/
#define BMA2x2_ENABLE_INTR1_PAD_FIFO_WM_POS     (1)
#define BMA2x2_ENABLE_INTR1_PAD_FIFO_WM_LEN     (1)
#define BMA2x2_ENABLE_INTR1_PAD_FIFO_WM_MSK     (0x02)
#define BMA2x2_ENABLE_INTR1_PAD_FIFO_WM_REG     BMA2x2_INTR_DATA_SELECT_ADDR
/**********************************************/
/**\name INTERRUPT1 ENABLE OF FIFO FULL*/
/**********************************************/
#define BMA2x2_ENABLE_INTR1_PAD_FIFO_FULL_POS     (2)
#define BMA2x2_ENABLE_INTR1_PAD_FIFO_FULL_LEN     (1)
#define BMA2x2_ENABLE_INTR1_PAD_FIFO_FULL_MSK     (0x04)
#define BMA2x2_ENABLE_INTR1_PAD_FIFO_FULL_REG     BMA2x2_INTR_DATA_SELECT_ADDR
/**********************************************/
/**\name INTERRUPT2 ENABLE OF PAD FIFO FULL */
/**********************************************/
#define BMA2x2_ENABLE_INTR2_PAD_FIFO_FULL_POS     (5)
#define BMA2x2_ENABLE_INTR2_PAD_FIFO_FULL_LEN     (1)
#define BMA2x2_ENABLE_INTR2_PAD_FIFO_FULL_MSK     (0x20)
#define BMA2x2_ENABLE_INTR2_PAD_FIFO_FULL_REG     BMA2x2_INTR_DATA_SELECT_ADDR
/**********************************************/
/**\name INTERRUPT2 ENABLE OF PAD FIFO WATERMARK*/
/**********************************************/
#define BMA2x2_ENABLE_INTR2_PAD_FIFO_WM_POS     (6)
#define BMA2x2_ENABLE_INTR2_PAD_FIFO_WM_LEN     (1)
#define BMA2x2_ENABLE_INTR2_PAD_FIFO_WM_MSK     (0x40)
#define BMA2x2_ENABLE_INTR2_PAD_FIFO_WM_REG     BMA2x2_INTR_DATA_SELECT_ADDR
/**********************************************/
/**\name INTERRUPT2 ENABLE OF PAD DATA */
/**********************************************/
#define BMA2x2_ENABLE_INTR2_PAD_NEWDATA_POS     (7)
#define BMA2x2_ENABLE_INTR2_PAD_NEWDATA_LEN     (1)
#define BMA2x2_ENABLE_INTR2_PAD_NEWDATA_MSK     (0x80)
#define BMA2x2_ENABLE_INTR2_PAD_NEWDATA_REG     BMA2x2_INTR_DATA_SELECT_ADDR
/**********************************************/
/**\name  INTERRUPT SOURCE SELECTION OF LOW_G*/
/**********************************************/
#define BMA2x2_UNFILT_INTR_SOURCE_LOW_G_POS        (0)
#define BMA2x2_UNFILT_INTR_SOURCE_LOW_G_LEN        (1)
#define BMA2x2_UNFILT_INTR_SOURCE_LOW_G_MSK        (0x01)
#define BMA2x2_UNFILT_INTR_SOURCE_LOW_G_REG        BMA2x2_INTR_SOURCE_ADDR
/**********************************************/
/**\name  INTERRUPT SOURCE SELECTION OF HIGH_G*/
/**********************************************/
#define BMA2x2_UNFILT_INTR_SOURCE_HIGH_G_POS       (1)
#define BMA2x2_UNFILT_INTR_SOURCE_HIGH_G_LEN       (1)
#define BMA2x2_UNFILT_INTR_SOURCE_HIGH_G_MSK       (0x02)
#define BMA2x2_UNFILT_INTR_SOURCE_HIGH_G_REG       BMA2x2_INTR_SOURCE_ADDR
/**********************************************/
/**\name  INTERRUPT SOURCE SELECTION OF SLOPE*/
/**********************************************/
#define BMA2x2_UNFILT_INTR_SOURCE_SLOPE_POS       (2)
#define BMA2x2_UNFILT_INTR_SOURCE_SLOPE_LEN       (1)
#define BMA2x2_UNFILT_INTR_SOURCE_SLOPE_MSK       (0x04)
#define BMA2x2_UNFILT_INTR_SOURCE_SLOPE_REG       BMA2x2_INTR_SOURCE_ADDR
/**********************************************/
/**\name  INTERRUPT SOURCE SELECTION OF SLOW NO MOTION*/
/**********************************************/
#define BMA2x2_UNFILT_INTR_SOURCE_SLOW_NO_MOTION_POS        (3)
#define BMA2x2_UNFILT_INTR_SOURCE_SLOW_NO_MOTION_LEN        (1)
#define BMA2x2_UNFILT_INTR_SOURCE_SLOW_NO_MOTION_MSK        (0x08)
#define BMA2x2_UNFILT_INTR_SOURCE_SLOW_NO_MOTION_REG        \
BMA2x2_INTR_SOURCE_ADDR
/**********************************************/
/**\name  INTERRUPT SOURCE SELECTION OF TAP*/
/**********************************************/
#define BMA2x2_UNFILT_INTR_SOURCE_TAP_POS         (4)
#define BMA2x2_UNFILT_INTR_SOURCE_TAP_LEN         (1)
#define BMA2x2_UNFILT_INTR_SOURCE_TAP_MSK         (0x10)
#define BMA2x2_UNFILT_INTR_SOURCE_TAP_REG         BMA2x2_INTR_SOURCE_ADDR
/**********************************************/
/**\name  INTERRUPT SOURCE SELECTION OF DATA*/
/**********************************************/
#define BMA2x2_UNFILT_INTR_SOURCE_DATA_POS        (5)
#define BMA2x2_UNFILT_INTR_SOURCE_DATA_LEN        (1)
#define BMA2x2_UNFILT_INTR_SOURCE_DATA_MSK        (0x20)
#define BMA2x2_UNFILT_INTR_SOURCE_DATA_REG        BMA2x2_INTR_SOURCE_ADDR
/****************************************************/
/**\name  INTERRUPT PAD ACTIVE LEVEL AND OUTPUT TYPE*/
/****************************************************/
#define BMA2x2_INTR1_PAD_ACTIVE_LEVEL_POS       (0)
#define BMA2x2_INTR1_PAD_ACTIVE_LEVEL_LEN       (1)
#define BMA2x2_INTR1_PAD_ACTIVE_LEVEL_MSK       (0x01)
#define BMA2x2_INTR1_PAD_ACTIVE_LEVEL_REG       BMA2x2_INTR_SET_ADDR

#define BMA2x2_INTR2_PAD_ACTIVE_LEVEL_POS       (2)
#define BMA2x2_INTR2_PAD_ACTIVE_LEVEL_LEN       (1)
#define BMA2x2_INTR2_PAD_ACTIVE_LEVEL_MSK       (0x04)
#define BMA2x2_INTR2_PAD_ACTIVE_LEVEL_REG       BMA2x2_INTR_SET_ADDR

#define BMA2x2_INTR1_PAD_OUTPUT_TYPE_POS        (1)
#define BMA2x2_INTR1_PAD_OUTPUT_TYPE_LEN        (1)
#define BMA2x2_INTR1_PAD_OUTPUT_TYPE_MSK        (0x02)
#define BMA2x2_INTR1_PAD_OUTPUT_TYPE_REG        BMA2x2_INTR_SET_ADDR

#define BMA2x2_INTR2_PAD_OUTPUT_TYPE_POS        (3)
#define BMA2x2_INTR2_PAD_OUTPUT_TYPE_LEN        (1)
#define BMA2x2_INTR2_PAD_OUTPUT_TYPE_MSK        (0x08)
#define BMA2x2_INTR2_PAD_OUTPUT_TYPE_REG        BMA2x2_INTR_SET_ADDR
/****************************************************/
/**\name   LATCH INTERRUPT */
/****************************************************/
#define BMA2x2_LATCH_INTR_POS                (0)
#define BMA2x2_LATCH_INTR_LEN                (4)
#define BMA2x2_LATCH_INTR_MSK                (0x0F)
#define BMA2x2_LATCH_INTR_REG                BMA2x2_INTR_CTRL_ADDR
/****************************************************/
/**\name   RESET LATCH INTERRUPT */
/****************************************************/
#define BMA2x2_RESET_INTR_POS           (7)
#define BMA2x2_RESET_INTR_LEN           (1)
#define BMA2x2_RESET_INTR_MSK           (0x80)
#define BMA2x2_RESET_INTR_REG           BMA2x2_INTR_CTRL_ADDR
/****************************************************/
/**\name   LOW_G HYSTERESIS */
/****************************************************/
#define BMA2x2_LOW_G_HYST_POS                   (0)
#define BMA2x2_LOW_G_HYST_LEN                   (2)
#define BMA2x2_LOW_G_HYST_MSK                   (0x03)
#define BMA2x2_LOW_G_HYST_REG                   BMA2x2_LOW_HIGH_HYST_ADDR
/****************************************************/
/**\name   LOW_G MODE */
/****************************************************/
#define BMA2x2_LOW_G_INTR_MODE_POS               (2)
#define BMA2x2_LOW_G_INTR_MODE_LEN               (1)
#define BMA2x2_LOW_G_INTR_MODE_MSK               (0x04)
#define BMA2x2_LOW_G_INTR_MODE_REG               BMA2x2_LOW_HIGH_HYST_ADDR

/****************************************************/
/**\name   HIGH_G HYSTERESIS */
/****************************************************/
#define BMA2x2_HIGH_G_HYST_POS                  (6)
#define BMA2x2_HIGH_G_HYST_LEN                  (2)
#define BMA2x2_HIGH_G_HYST_MSK                  (0xC0)
#define BMA2x2_HIGH_G_HYST_REG                  BMA2x2_LOW_HIGH_HYST_ADDR
/****************************************************/
/**\name   SLOPE DURATION */
/****************************************************/
#define BMA2x2_SLOPE_DURN_POS                    (0)
#define BMA2x2_SLOPE_DURN_LEN                    (2)
#define BMA2x2_SLOPE_DURN_MSK                    (0x03)
#define BMA2x2_SLOPE_DURN_REG                    BMA2x2_SLOPE_DURN_ADDR
/****************************************************/
/**\name   SLOW NO MOTION DURATION */
/****************************************************/
#define BMA2x2_SLOW_NO_MOTION_DURN_POS                    (2)
#define BMA2x2_SLOW_NO_MOTION_DURN_LEN                    (6)
#define BMA2x2_SLOW_NO_MOTION_DURN_MSK                    (0xFC)
#define BMA2x2_SLOW_NO_MOTION_DURN_REG                    BMA2x2_SLOPE_DURN_ADDR

/****************************************************/
/**\name   TAP DURATION */
/****************************************************/
#define BMA2x2_TAP_DURN_POS                    (0)
#define BMA2x2_TAP_DURN_LEN                    (3)
#define BMA2x2_TAP_DURN_MSK                    (0x07)
#define BMA2x2_TAP_DURN_REG                    BMA2x2_TAP_PARAM_ADDR

/****************************************************/
/**\name   TAP SHOCK DURATION */
/****************************************************/
#define BMA2x2_TAP_SHOCK_DURN_POS             (6)
#define BMA2x2_TAP_SHOCK_DURN_LEN             (1)
#define BMA2x2_TAP_SHOCK_DURN_MSK             (0x40)
#define BMA2x2_TAP_SHOCK_DURN_REG             BMA2x2_TAP_PARAM_ADDR

/* This advance tap interrupt only uses for the chip id 0xFB */
#define BMA2x2_ADV_TAP_INTR_POS                (5)
#define BMA2x2_ADV_TAP_INTR_LEN                (1)
#define BMA2x2_ADV_TAP_INTR_MSK                (0x20)
#define BMA2x2_ADV_TAP_INTR_REG                BMA2x2_TAP_PARAM_ADDR
/****************************************************/
/**\name   TAP QUIET DURATION */
/****************************************************/
#define BMA2x2_TAP_QUIET_DURN_POS             (7)
#define BMA2x2_TAP_QUIET_DURN_LEN             (1)
#define BMA2x2_TAP_QUIET_DURN_MSK             (0x80)
#define BMA2x2_TAP_QUIET_DURN_REG             BMA2x2_TAP_PARAM_ADDR
/****************************************************/
/**\name   TAP THRESHOLD */
/****************************************************/
#define BMA2x2_TAP_THRES_POS                  (0)
#define BMA2x2_TAP_THRES_LEN                  (5)
#define BMA2x2_TAP_THRES_MSK                  (0x1F)
#define BMA2x2_TAP_THRES_REG                  BMA2x2_TAP_THRES_ADDR
/****************************************************/
/**\name   TAP SAMPLES */
/****************************************************/
#define BMA2x2_TAP_SAMPLES_POS                (6)
#define BMA2x2_TAP_SAMPLES_LEN                (2)
#define BMA2x2_TAP_SAMPLES_MSK                (0xC0)
#define BMA2x2_TAP_SAMPLES_REG                BMA2x2_TAP_THRES_ADDR
/****************************************************/
/**\name  ORIENT MODE */
/****************************************************/
#define BMA2x2_ORIENT_MODE_POS                  (0)
#define BMA2x2_ORIENT_MODE_LEN                  (2)
#define BMA2x2_ORIENT_MODE_MSK                  (0x03)
#define BMA2x2_ORIENT_MODE_REG                  BMA2x2_ORIENT_PARAM_ADDR
/****************************************************/
/**\name   ORIENT BLOCKING */
/****************************************************/
#define BMA2x2_ORIENT_BLOCK_POS                 (2)
#define BMA2x2_ORIENT_BLOCK_LEN                 (2)
#define BMA2x2_ORIENT_BLOCK_MSK                 (0x0C)
#define BMA2x2_ORIENT_BLOCK_REG                 BMA2x2_ORIENT_PARAM_ADDR
/****************************************************/
/**\name   ORIENT HYSTERESIS */
/****************************************************/
#define BMA2x2_ORIENT_HYST_POS                  (4)
#define BMA2x2_ORIENT_HYST_LEN                  (3)
#define BMA2x2_ORIENT_HYST_MSK                  (0x70)
#define BMA2x2_ORIENT_HYST_REG                  BMA2x2_ORIENT_PARAM_ADDR
/****************************************************/
/**\name   ORIENT AXIS  */
/****************************************************/
#define BMA2x2_ORIENT_UD_ENABLE_POS                  (6)
#define BMA2x2_ORIENT_UD_ENABLE_LEN                  (1)
#define BMA2x2_ORIENT_UD_ENABLE_MSK                  (0x40)
#define BMA2x2_ORIENT_UD_ENABLE_REG                  BMA2x2_THETA_BLOCK_ADDR

/****************************************************/
/**\name   THETA BLOCKING */
/****************************************************/
#define BMA2x2_THETA_BLOCK_POS                  (0)
#define BMA2x2_THETA_BLOCK_LEN                  (6)
#define BMA2x2_THETA_BLOCK_MSK                  (0x3F)
#define BMA2x2_THETA_BLOCK_REG                  BMA2x2_THETA_BLOCK_ADDR
/****************************************************/
/**\name   THETA FLAT */
/****************************************************/
#define BMA2x2_THETA_FLAT_POS                  (0)
#define BMA2x2_THETA_FLAT_LEN                  (6)
#define BMA2x2_THETA_FLAT_MSK                  (0x3F)
#define BMA2x2_THETA_FLAT_REG                  BMA2x2_THETA_FLAT_ADDR
/****************************************************/
/**\name   THETA HOLD TIME */
/****************************************************/
#define BMA2x2_FLAT_HOLD_TIME_POS              (4)
#define BMA2x2_FLAT_HOLD_TIME_LEN              (2)
#define BMA2x2_FLAT_HOLD_TIME_MSK              (0x30)
#define BMA2x2_FLAT_HOLD_TIME_REG              BMA2x2_FLAT_HOLD_TIME_ADDR
/****************************************************/
/**\name   FLAT HYSTERESIS */
/****************************************************/
#define BMA2x2_FLAT_HYST_POS                   (0)
#define BMA2x2_FLAT_HYST_LEN                   (3)
#define BMA2x2_FLAT_HYST_MSK                   (0x07)
#define BMA2x2_FLAT_HYST_REG                   BMA2x2_FLAT_HOLD_TIME_ADDR
/****************************************************/
/**\name   FIFO WATER MARK LEVEL TRIGGER RETAIN  */
/****************************************************/
#define BMA2x2_FIFO_WML_TRIG_RETAIN_POS                   (0)
#define BMA2x2_FIFO_WML_TRIG_RETAIN_LEN                   (6)
#define BMA2x2_FIFO_WML_TRIG_RETAIN_MSK                   (0x3F)
#define BMA2x2_FIFO_WML_TRIG_RETAIN_REG                   BMA2x2_FIFO_WML_TRIG
/****************************************************/
/**\name   ACTIVATE SELF TEST  */
/****************************************************/
#define BMA2x2_ENABLE_SELFTEST_POS                (0)
#define BMA2x2_ENABLE_SELFTEST_LEN                (2)
#define BMA2x2_ENABLE_SELFTEST_MSK                (0x03)
#define BMA2x2_ENABLE_SELFTEST_REG                BMA2x2_SELFTEST_ADDR
/****************************************************/
/**\name   SELF TEST -- NEGATIVE   */
/****************************************************/
#define BMA2x2_NEG_SELFTEST_POS               (2)
#define BMA2x2_NEG_SELFTEST_LEN               (1)
#define BMA2x2_NEG_SELFTEST_MSK               (0x04)
#define BMA2x2_NEG_SELFTEST_REG               BMA2x2_SELFTEST_ADDR
/****************************************************/
/**\name   EEPROM CONTROL   */
/****************************************************/
#define BMA2x2_UNLOCK_EE_PROG_MODE_POS     (0)
#define BMA2x2_UNLOCK_EE_PROG_MODE_LEN     (1)
#define BMA2x2_UNLOCK_EE_PROG_MODE_MSK     (0x01)
#define BMA2x2_UNLOCK_EE_PROG_MODE_REG     BMA2x2_EEPROM_CTRL_ADDR
/**********************************************************************/
/**\name  SETTING THIS BIT STARTS WRITING SETTING REGISTERS TO EEPROM */
/*********************************************************************/
#define BMA2x2_START_EE_PROG_TRIG_POS      (1)
#define BMA2x2_START_EE_PROG_TRIG_LEN      (1)
#define BMA2x2_START_EE_PROG_TRIG_MSK      (0x02)
#define BMA2x2_START_EE_PROG_TRIG_REG      BMA2x2_EEPROM_CTRL_ADDR
/****************************************************/
/**\name   STATUS OF WRITING TO EEPROM   */
/****************************************************/
#define BMA2x2_EE_PROG_READY_POS          (2)
#define BMA2x2_EE_PROG_READY_LEN          (1)
#define BMA2x2_EE_PROG_READY_MSK          (0x04)
#define BMA2x2_EE_PROG_READY_REG          BMA2x2_EEPROM_CTRL_ADDR
/****************************************************/
/**\name   UPDATE IMAGE REGISTERS WRITING TO EEPROM   */
/****************************************************/
#define BMA2x2_UPDATE_IMAGE_POS                (3)
#define BMA2x2_UPDATE_IMAGE_LEN                (1)
#define BMA2x2_UPDATE_IMAGE_MSK                (0x08)
#define BMA2x2_UPDATE_IMAGE_REG                BMA2x2_EEPROM_CTRL_ADDR

#define BMA2x2_EE_REMAIN_POS                (4)
#define BMA2x2_EE_REMAIN_LEN                (4)
#define BMA2x2_EE_REMAIN_MSK                (0xF0)
#define BMA2x2_EE_REMAIN_REG                BMA2x2_EEPROM_CTRL_ADDR
/****************************************************/
/**\name   SPI INTERFACE MODE SELECTION   */
/***************************************************/
#define BMA2x2_ENABLE_SPI_MODE_3_POS              (0)
#define BMA2x2_ENABLE_SPI_MODE_3_LEN              (1)
#define BMA2x2_ENABLE_SPI_MODE_3_MSK              (0x01)
#define BMA2x2_ENABLE_SPI_MODE_3_REG              BMA2x2_SERIAL_CTRL_ADDR
/****************************************************/
/**\name   I2C WATCHDOG PERIOD SELECTION   */
/***************************************************/
#define BMA2x2_I2C_WDT_PERIOD_POS        (1)
#define BMA2x2_I2C_WDT_PERIOD_LEN        (1)
#define BMA2x2_I2C_WDT_PERIOD_MSK        (0x02)
#define BMA2x2_I2C_WDT_PERIOD_REG        BMA2x2_SERIAL_CTRL_ADDR
/****************************************************/
/**\name   I2C WATCHDOG ENABLE   */
/***************************************************/
#define BMA2x2_ENABLE_I2C_WDT_POS            (2)
#define BMA2x2_ENABLE_I2C_WDT_LEN            (1)
#define BMA2x2_ENABLE_I2C_WDT_MSK            (0x04)
#define BMA2x2_ENABLE_I2C_WDT_REG            BMA2x2_SERIAL_CTRL_ADDR
/****************************************************/
/**\name   SPI INTERFACE MODE SELECTIONE            */
/***************************************************/
#define BMA2x2_UNLOCK_EE_WRITE_TRIM_POS        (4)
#define BMA2x2_UNLOCK_EE_WRITE_TRIM_LEN        (4)
#define BMA2x2_UNLOCK_EE_WRITE_TRIM_MSK        (0xF0)
#define BMA2x2_UNLOCK_EE_WRITE_TRIM_REG        BMA2x2_CTRL_UNLOCK_REG
/******************************************************************/
/**\name   OFFSET  COMPENSATION/SLOW COMPENSATION FOR X,Y,Z AXIS */
/*****************************************************************/
#define BMA2x2_ENABLE_SLOW_COMP_X_POS              (0)
#define BMA2x2_ENABLE_SLOW_COMP_X_LEN              (1)
#define BMA2x2_ENABLE_SLOW_COMP_X_MSK              (0x01)
#define BMA2x2_ENABLE_SLOW_COMP_X_REG              BMA2x2_OFFSET_CTRL_ADDR

#define BMA2x2_ENABLE_SLOW_COMP_Y_POS              (1)
#define BMA2x2_ENABLE_SLOW_COMP_Y_LEN              (1)
#define BMA2x2_ENABLE_SLOW_COMP_Y_MSK              (0x02)
#define BMA2x2_ENABLE_SLOW_COMP_Y_REG              BMA2x2_OFFSET_CTRL_ADDR

#define BMA2x2_ENABLE_SLOW_COMP_Z_POS              (2)
#define BMA2x2_ENABLE_SLOW_COMP_Z_LEN              (1)
#define BMA2x2_ENABLE_SLOW_COMP_Z_MSK              (0x04)
#define BMA2x2_ENABLE_SLOW_COMP_Z_REG              BMA2x2_OFFSET_CTRL_ADDR
/****************************************************/
/**\name   FAST COMPENSATION READY FLAG            */
/***************************************************/
#define BMA2x2_FAST_CAL_RDY_STAT_POS             (4)
#define BMA2x2_FAST_CAL_RDY_STAT_LEN             (1)
#define BMA2x2_FAST_CAL_RDY_STAT_MSK             (0x10)
#define BMA2x2_FAST_CAL_RDY_STAT_REG             BMA2x2_OFFSET_CTRL_ADDR
/****************************************************/
/**\name   FAST COMPENSATION FOR X,Y,Z AXIS         */
/***************************************************/
#define BMA2x2_CAL_TRIGGER_POS                (5)
#define BMA2x2_CAL_TRIGGER_LEN                (2)
#define BMA2x2_CAL_TRIGGER_MSK                (0x60)
#define BMA2x2_CAL_TRIGGER_REG                BMA2x2_OFFSET_CTRL_ADDR
/****************************************************/
/**\name   RESET OFFSET REGISTERS         */
/***************************************************/
#define BMA2x2_RST_OFFSET_POS           (7)
#define BMA2x2_RST_OFFSET_LEN           (1)
#define BMA2x2_RST_OFFSET_MSK           (0x80)
#define BMA2x2_RST_OFFSET_REG           BMA2x2_OFFSET_CTRL_ADDR
/****************************************************/
/**\name   SLOW COMPENSATION  CUTOFF        */
/***************************************************/
#define BMA2x2_COMP_CUTOFF_POS                 (0)
#define BMA2x2_COMP_CUTOFF_LEN                 (1)
#define BMA2x2_COMP_CUTOFF_MSK                 (0x01)
#define BMA2x2_COMP_CUTOFF_REG                 BMA2x2_OFFSET_PARAMS_ADDR
/****************************************************/
/**\name    COMPENSATION TARGET       */
/***************************************************/
#define BMA2x2_COMP_TARGET_OFFSET_X_POS        (1)
#define BMA2x2_COMP_TARGET_OFFSET_X_LEN        (2)
#define BMA2x2_COMP_TARGET_OFFSET_X_MSK        (0x06)
#define BMA2x2_COMP_TARGET_OFFSET_X_REG        BMA2x2_OFFSET_PARAMS_ADDR

#define BMA2x2_COMP_TARGET_OFFSET_Y_POS        (3)
#define BMA2x2_COMP_TARGET_OFFSET_Y_LEN        (2)
#define BMA2x2_COMP_TARGET_OFFSET_Y_MSK        (0x18)
#define BMA2x2_COMP_TARGET_OFFSET_Y_REG        BMA2x2_OFFSET_PARAMS_ADDR

#define BMA2x2_COMP_TARGET_OFFSET_Z_POS        (5)
#define BMA2x2_COMP_TARGET_OFFSET_Z_LEN        (2)
#define BMA2x2_COMP_TARGET_OFFSET_Z_MSK        (0x60)
#define BMA2x2_COMP_TARGET_OFFSET_Z_REG        BMA2x2_OFFSET_PARAMS_ADDR
/****************************************************/
/**\name    FIFO DATA SELECT       */
/***************************************************/
#define BMA2x2_FIFO_DATA_SELECT_POS                 (0)
#define BMA2x2_FIFO_DATA_SELECT_LEN                 (2)
#define BMA2x2_FIFO_DATA_SELECT_MSK                 (0x03)
#define BMA2x2_FIFO_DATA_SELECT_REG                 BMA2x2_FIFO_MODE_ADDR
/****************************************************/
/**\name   FIFO MODE      */
/***************************************************/
#define BMA2x2_FIFO_MODE_POS                 (6)
#define BMA2x2_FIFO_MODE_LEN                 (2)
#define BMA2x2_FIFO_MODE_MSK                 (0xC0)
#define BMA2x2_FIFO_MODE_REG                 BMA2x2_FIFO_MODE_ADDR

/****************************************************/
/**\name  BITSLICE FUNCTIONS      */
/***************************************************/
#define BMA2x2_GET_BITSLICE(regvar, bitname)\
((regvar & bitname##_MSK) >> bitname##_POS)


#define BMA2x2_SET_BITSLICE(regvar, bitname, val)\
((regvar & ~bitname##_MSK) | ((val<<bitname##_POS)&bitname##_MSK))

/****************************************************/
/**\name   CONSTANTS      */
/***************************************************/
/****************************************************/
/**\name  RESOLUTION SELECTION      */
/***************************************************/
/* Definitions used for accel resolution bit shifting*/
#define BMA2x2_14_BIT_SHIFT		(0xFC)
/**< It refers 14bit accel resolution*/
#define BMA2x2_10_BIT_SHIFT		(0xC0)
/**< It refers 10bit accel resolution*/
#define BMA2x2_12_BIT_SHIFT		(0xF0)
/**< It refers 12bit accel resolution*/
#define BANDWIDTH_DEFINE		(0xFB)
/**< Chip id set for accel bandwidth define*/

/****************************************************/
/**\name  ENABLE DISABLE SELECTION     */
/***************************************************/
#define INTR_ENABLE	(0X01)
/**< Enable selection for bit */
#define INTR_DISABLE	(0x00)
/**< Disable selection for bit */

/****************************************************/
/**\name  OUTPUT TYPE SELECT     */
/***************************************************/
#define OPEN_DRAIN	(0x01)
/**< It refers open drain selection*/
#define PUSS_PULL	(0x01)
/**< It refers push pull selection*/

/****************************************************/
/**\name  LEVEL SELECT     */
/***************************************************/
#define	ACTIVE_LOW	(0x00)
/**< It refers active low selection*/
#define	ACTIVE_HIGH	(0x01)
/**< It refers active high selection*/

/****************************************************/
/**\name  STATUS SELECT     */
/***************************************************/
#define BMA2x2_STAT1                             (0)
/**< It refers Status interrupt1 */
#define BMA2x2_STAT2                             (1)
/**< It refers Status interrupt2 */
#define BMA2x2_STAT3                             (2)
/**< It refers Status interrupt3  */
#define BMA2x2_STAT4                             (3)
/**< It refers Status interrupt4  */
#define BMA2x2_STAT5                             (4)
/**< It refers Status interrupt5  */

/****************************************************/
/**\name  RANGE AND BANDWIDTH SELECT     */
/***************************************************/
#define BMA2x2_RANGE_2G                 (3)
/**< sets range to +/- 2G mode */
#define BMA2x2_RANGE_4G                 (5)
/**< sets range to +/- 4G mode */
#define BMA2x2_RANGE_8G                 (8)
/**< sets range to +/- 8G mode */
#define BMA2x2_RANGE_16G                (12)
/**< sets range to +/- 16G mode */


#define BMA2x2_BW_7_81HZ        (0x08)
 /**< sets bandwidth to LowPass 7.81HZ  */
#define BMA2x2_BW_15_63HZ       (0x09)
/**< sets bandwidth to LowPass 15.63HZ  */
#define BMA2x2_BW_31_25HZ       (0x0A)
/**< sets bandwidth to LowPass 31.25HZ  */
#define BMA2x2_BW_62_50HZ       (0x0B)
 /**< sets bandwidth to LowPass 62.50HZ  */
#define BMA2x2_BW_125HZ         (0x0C)
 /**< sets bandwidth to LowPass 125HZ  */
#define BMA2x2_BW_250HZ         (0x0D)
/**< sets bandwidth to LowPass 250HZ  */
#define BMA2x2_BW_500HZ         (0x0E)
/**< sets bandwidth to LowPass 500HZ  */
#define BMA2x2_BW_1000HZ        (0x0F)
 /**< sets bandwidth to LowPass 1000HZ  */

/******************************************/
/**\name  SLEEP DURATION SELECT     */
/******************************************/
#define BMA2x2_SLEEP_DURN_0_5MS        (0x05)
/* sets sleep duration to 0.5 ms  */
#define BMA2x2_SLEEP_DURN_1MS          (0x06)
/* sets sleep duration to 1 ms */
#define BMA2x2_SLEEP_DURN_2MS          (0x07)
/* sets sleep duration to 2 ms */
#define BMA2x2_SLEEP_DURN_4MS          (0x08)
/* sets sleep duration to 4 ms */
#define BMA2x2_SLEEP_DURN_6MS          (0x09)
/* sets sleep duration to 6 ms*/
#define BMA2x2_SLEEP_DURN_10MS         (0x0A)
/* sets sleep duration to 10 ms */
#define BMA2x2_SLEEP_DURN_25MS         (0x0B)
/* sets sleep duration to 25 ms */
#define BMA2x2_SLEEP_DURN_50MS         (0x0C)
/* sets sleep duration to 50 ms */
#define BMA2x2_SLEEP_DURN_100MS        (0x0D)
/* sets sleep duration to 100 ms */
#define BMA2x2_SLEEP_DURN_500MS        (0x0E)
/* sets sleep duration to 500 ms */
#define BMA2x2_SLEEP_DURN_1S           (0x0F)
/* sets sleep duration to 1 s */

/******************************************/
/**\name  LATCH DURATION     */
/******************************************/
#define BMA2x2_LATCH_DURN_NON_LATCH    (0x00)
/* sets LATCH duration to NON LATCH  */
#define BMA2x2_LATCH_DURN_250MS        (0x01)
/* sets LATCH duration to 250 ms */
#define BMA2x2_LATCH_DURN_500MS        (0x02)
/* sets LATCH duration to 500 ms */
#define BMA2x2_LATCH_DURN_1S           (0x03)
 /* sets LATCH duration to 1 s */
#define BMA2x2_LATCH_DURN_2S           (0x04)
 /* sets LATCH duration to 2 s*/
#define BMA2x2_LATCH_DURN_4S           (0x05)
 /* sets LATCH duration to 4 s */
#define BMA2x2_LATCH_DURN_8S           (0x06)
 /* sets LATCH duration to 8 s */
#define BMA2x2_LATCH_DURN_LATCH        (0x07)
 /* sets LATCH duration to LATCH */
#define BMA2x2_LATCH_DURN_NON_LATCH1   (0x08)
 /* sets LATCH duration to NON LATCH1 */
#define BMA2x2_LATCH_DURN_250US        (0x09)
 /* sets LATCH duration to 250 Us */
#define BMA2x2_LATCH_DURN_500US        (0x0A)
 /* sets LATCH duration to 500 Us */
#define BMA2x2_LATCH_DURN_1MS          (0x0B)
 /* sets LATCH duration to 1 Ms */
#define BMA2x2_LATCH_DURN_12_5MS       (0x0C)
/* sets LATCH duration to 12.5 Ms */
#define BMA2x2_LATCH_DURN_25MS         (0x0D)
/* sets LATCH duration to 25 Ms */
#define BMA2x2_LATCH_DURN_50MS         (0x0E)
 /* sets LATCH duration to 50 Ms */
#define BMA2x2_LATCH_DURN_LATCH1       (0x0F)
/* sets LATCH duration to LATCH*/

/******************************************/
/**\name  MODE SETTINGS     */
/******************************************/
#define BMA2x2_MODE_NORMAL             (0)
#define BMA2x2_MODE_LOWPOWER1          (1)
#define BMA2x2_MODE_SUSPEND            (2)
#define BMA2x2_MODE_DEEP_SUSPEND       (3)
#define BMA2x2_MODE_LOWPOWER2          (4)
#define BMA2x2_MODE_STANDBY            (5)

/******************************************/
/**\name  AXIS SELECTION     */
/******************************************/
#define BMA2x2_X_AXIS           (0)
/**< It refers BMA2x2 X-axis */
#define BMA2x2_Y_AXIS           (1)
/**< It refers BMA2x2 Y-axis */
#define BMA2x2_Z_AXIS           (2)
/**< It refers BMA2x2 Z-axis */

/******************************************/
/**\name  INTERRUPT TYPE SELECTION     */
/******************************************/
#define BMA2x2_LOW_G_INTR       (0)
/**< enable/disable low-g interrupt*/
#define BMA2x2_HIGH_G_X_INTR    (1)
/**< enable/disable high_g X interrupt*/
#define BMA2x2_HIGH_G_Y_INTR    (2)
/**< enable/disable high_g Y interrupt*/
#define BMA2x2_HIGH_G_Z_INTR    (3)
/**< enable/disable high_g Z interrupt*/
#define BMA2x2_DATA_ENABLE      (4)
/**< enable/disable data interrupt*/
#define BMA2x2_SLOPE_X_INTR     (5)
/**< enable/disable slope X interrupt*/
#define BMA2x2_SLOPE_Y_INTR     (6)
/**< enable/disable slope X interrupt*/
#define BMA2x2_SLOPE_Z_INTR     (7)
/**< enable/disable slope X interrupt*/
#define BMA2x2_SINGLE_TAP_INTR  (8)
/**< enable/disable single tap interrupt*/
#define BMA2x2_DOUBLE_TAP_INTR  (9)
/**< enable/disable double tap interrupt*/
#define BMA2x2_ORIENT_INTR      (10)
/**< enable/disable orient interrupt*/
#define BMA2x2_FLAT_INTR        (11)
/**< enable/disable flat interrupt*/
#define BMA2x2_FIFO_FULL_INTR   (12)
/**< enable/disable fifo full interrupt*/
#define BMA2x2_FIFO_WM_INTR     (13)
/**< enable/disable fifo water mark interrupt*/

/******************************************/
/**\name  INTERRUPTS PADS     */
/******************************************/
#define BMA2x2_INTR1_LOW_G             (0)
/**< disable low-g interrupt*/
#define BMA2x2_INTR2_LOW_G             (1)
/**< enable low-g interrupt*/
#define BMA2x2_INTR1_HIGH_G            (0)
/**< disable high-g interrupt*/
#define BMA2x2_INTR2_HIGH_G            (1)
/**< enable high-g interrupt*/
#define BMA2x2_INTR1_SLOPE             (0)
/**< disable slope interrupt*/
#define BMA2x2_INTR2_SLOPE             (1)
/**< enable slope interrupt*/
#define BMA2x2_INTR1_SLOW_NO_MOTION    (0)
/**< disable slow no motion interrupt*/
#define BMA2x2_INTR2_SLOW_NO_MOTION    (1)
/**< enable slow no motion  interrupt*/
#define BMA2x2_INTR1_DOUBLE_TAP        (0)
/**< disable double tap  interrupt*/
#define BMA2x2_INTR2_DOUBLE_TAP        (1)
/**< enable double tap  interrupt*/
#define BMA2x2_INTR1_SINGLE_TAP        (0)
/**< disable single tap  interrupt*/
#define BMA2x2_INTR2_SINGLE_TAP        (1)
/**< enable single tap  interrupt*/
#define BMA2x2_INTR1_ORIENT            (0)
/**< disable orient  interrupt*/
#define BMA2x2_INTR2_ORIENT            (1)
/**< enable orient  interrupt*/
#define BMA2x2_INTR1_FLAT              (0)
/**< disable flat  interrupt*/
#define BMA2x2_INTR2_FLAT              (1)
/**< enable flat  interrupt*/
#define BMA2x2_INTR1_NEWDATA           (0)
/**< disable data  interrupt*/
#define BMA2x2_INTR2_NEWDATA           (1)
/**< enable data interrupt*/
#define BMA2x2_INTR1_FIFO_WM           (0)
/**< disable fifo watermark  interrupt*/
#define BMA2x2_INTR2_FIFO_WM           (1)
/**< enable fifo watermark  interrupt*/
#define BMA2x2_INTR1_FIFO_FULL         (0)
/**< disable fifo full  interrupt*/
#define BMA2x2_INTR2_FIFO_FULL         (1)
/**< enable fifo full  interrupt*/

/******************************************/
/**\name  SOURCE REGISTER     */
/******************************************/
#define BMA2x2_SOURCE_LOW_G            (0)
#define BMA2x2_SOURCE_HIGH_G           (1)
#define BMA2x2_SOURCE_SLOPE            (2)
#define BMA2x2_SOURCE_SLOW_NO_MOTION   (3)
#define BMA2x2_SOURCE_TAP              (4)
#define BMA2x2_SOURCE_DATA             (5)

#define BMA2x2_INTR1_OUTPUT      (0)
#define BMA2x2_INTR2_OUTPUT      (1)
#define BMA2x2_INTR1_LEVEL       (0)
#define BMA2x2_INTR2_LEVEL       (1)

/******************************************/
/**\name  DURATION     */
/******************************************/
#define BMA2x2_LOW_DURN                (0)
#define BMA2x2_HIGH_DURN               (1)
#define BMA2x2_SLOPE_DURN              (2)
#define BMA2x2_SLOW_NO_MOTION_DURN     (3)

/******************************************/
/**\name  THRESHOLD     */
/******************************************/
#define BMA2x2_LOW_THRES                (0)
#define BMA2x2_HIGH_THRES               (1)
#define BMA2x2_SLOPE_THRES              (2)
#define BMA2x2_SLOW_NO_MOTION_THRES     (3)


#define BMA2x2_LOW_G_HYST                (0)
#define BMA2x2_HIGH_G_HYST               (1)

#define BMA2x2_ORIENT_THETA             (0)
#define BMA2x2_FLAT_THETA               (1)

#define BMA2x2_I2C_SELECT               (0)
#define BMA2x2_I2C_ENABLE               (1)
/******************************************/
/**\name  COMPENSATION     */
/******************************************/
#define BMA2x2_SLOW_COMP_X              (0)
#define BMA2x2_SLOW_COMP_Y              (1)
#define BMA2x2_SLOW_COMP_Z              (2)
/******************************************/
/**\name  OFFSET TRIGGER     */
/******************************************/
#define BMA2x2_CUT_OFF                  (0)
#define BMA2x2_OFFSET_TRIGGER_X         (1)
#define BMA2x2_OFFSET_TRIGGER_Y         (2)
#define BMA2x2_OFFSET_TRIGGER_Z         (3)
/******************************************/
/**\name  GP REGISTERS     */
/******************************************/
#define BMA2x2_GP0                      (0)
#define BMA2x2_GP1                      (1)
/******************************************/
/**\name  SLO NO MOTION REGISTER      */
/******************************************/
#define BMA2x2_SLOW_NO_MOTION_ENABLE_X          (0)
#define BMA2x2_SLOW_NO_MOTION_ENABLE_Y          (1)
#define BMA2x2_SLOW_NO_MOTION_ENABLE_Z          (2)
#define BMA2x2_SLOW_NO_MOTION_ENABLE_SELECT     (3)
/******************************************/
/**\name  WAKE UP      */
/******************************************/
#define BMA2x2_WAKE_UP_DURN_20MS         (0)
#define BMA2x2_WAKE_UP_DURN_80MS         (1)
#define BMA2x2_WAKE_UP_DURN_320MS        (2)
#define BMA2x2_WAKE_UP_DURN_2560MS       (3)


/* LG/HG thresholds are in LSB and depend on RANGE setting */
/* no range check on threshold calculation */

#define BMA2x2_SELFTEST0_ON            (1)
#define BMA2x2_SELFTEST1_ON            (2)

#define BMA2x2_EE_W_OFF                 (0)
#define BMA2x2_EE_W_ON                  (1)
/******************************************/
/**\name  RESOLUTION SETTINGS      */
/******************************************/
#define BMA2x2_RESOLUTION_12_BIT        (0)
#define BMA2x2_RESOLUTION_10_BIT        (1)
#define BMA2x2_RESOLUTION_14_BIT        (3)

/******************************************/
/**\name  CHIP ID SELECTION      */
/******************************************/
#define BMA2x2           (0x16)
#define BMA280           (0x17)
#define BMA222E          (0x18)
#define BMA250E          (0x19)
/******************************************/
/**\name  LOW-G MODE SELECTION    */
/******************************************/
#define LOW_G_SINGLE_AXIS_MODE	(0x00)
#define LOW_G_SUMMING_MODE		(0x01)
/******************************************/
/**\name TAP DURATION DEFINITION    */
/******************************************/
#define TAP_DURN_50_MS			(0x00)
#define TAP_DURN_100_MS			(0x01)
#define TAP_DURN_150_MS			(0x02)
#define TAP_DURN_200_MS			(0x03)
#define TAP_DURN_250_MS			(0x04)
#define TAP_DURN_375_MS			(0x05)
#define TAP_DURN_500_MS			(0x06)
#define TAP_DURN_700_MS			(0x07)
/******************************************/
/**\name TAP SHOCK DEFINITION    */
/******************************************/
#define TAP_SHOCK_50_MS		(0x00)
#define TAP_SHOCK_75_MS		(0x01)
/******************************************/
/**\name TAP QUIET DEFINITION    */
/******************************************/
#define	TAP_QUIET_30_MS		(0x00)
#define	TAP_QUIET_20_MS		(0x01)
/****************************************************/
/**\name	ARRAY SIZE DEFINITIONS      */
/***************************************************/
#define BMA2x2_ACCEL_DATA_SIZE				(2)
#define BMA2x2_ACCEL_XYZ_DATA_SIZE			(6)
#define BMA2x2_ACCEL_XYZ_TEMP_DATA_SIZE		(7)
/****************************************************/
/**\name	ARRAY PARAMETERS      */
/***************************************************/

#define BMA2x2_SENSOR_DATA_ACCEL_LSB	(0)
#define BMA2x2_SENSOR_DATA_ACCEL_MSB	(1)

#define BMA2x2_SENSOR_DATA_XYZ_X_LSB				(0)
#define BMA2x2_SENSOR_DATA_XYZ_X_MSB				(1)
#define BMA2x2_SENSOR_DATA_XYZ_Y_LSB				(2)
#define BMA2x2_SENSOR_DATA_XYZ_Y_MSB				(3)
#define BMA2x2_SENSOR_DATA_XYZ_Z_LSB				(4)
#define BMA2x2_SENSOR_DATA_XYZ_Z_MSB				(5)
#define BMA2x2_SENSOR_DATA_TEMP						(6)

#define BMA2x2_RESOLUTION_12_MASK		(0xF0)
#define BMA2x2_RESOLUTION_10_MASK		(0xC0)
#define BMA2x2_RESOLUTION_14_MASK		(0xFC)

#define	BMA2x2_POWER_MODE_HEX_E_ZERO_MASK			(0xE0)
#define	BMA2x2_POWER_MODE_HEX_4_ZERO_MASK			(0x40)
#define	BMA2x2_POWER_MODE_HEX_ZERO_ZERO_MASK		(0x00)
#define	BMA2x2_POWER_MODE_HEX_ZERO_ONE_MASK			(0x01)
#define	BMA2x2_POWER_MODE_HEX_ZERO_TWO_MASK			(0x02)
#define	BMA2x2_POWER_MODE_HEX_ZERO_FOUR_MASK		(0x04)
#define	BMA2x2_POWER_MODE_HEX_ZERO_SIX_MASK			(0x06)

/** Macro to convert floating point
low-g-thresholds in G to 8-bit register values.<br>
  * Example: BMA2x2_LOW_TH_IN_G( 0.3, 2.0) generates
  * the register value for 0.3G threshold in 2G mode.
  * \brief convert g-values to 8-bit value
 */
#define BMA2x2_LOW_THRES_IN_G(gthres, range)  ((256 * gthres) / range)

/** Macro to convert floating point high-g-thresholds
    in G to 8-bit register values.<br>
  * Example: BMA2x2_HIGH_TH_IN_G( 1.4, 2.0)
  * generates the register value for 1.4G threshold in 2G mode.
  * \brief convert g-values to 8-bit value
 */
#define BMA2x2_HIGH_THRES_IN_G(gthres, range)   ((256 * gthres) / range)

/** Macro to convert floating point low-g-hysteresis
in G to 8-bit register values.<br>
  * Example: BMA2x2_LOW_HY_IN_G( 0.2, 2.0)
  *generates the register value for 0.2G threshold in 2G mode.
  * \brief convert g-values to 8-bit value
 */
#define BMA2x2_LOW_HYST_IN_G(ghyst, range)   ((32 * ghyst) / range)

/** Macro to convert floating point high-g-hysteresis
   in G to 8-bit register values.<br>
  * Example: BMA2x2_HIGH_HY_IN_G( 0.2, 2.0) generates
  *the register value for 0.2G threshold in 2G mode.
  * \brief convert g-values to 8-bit value
 */
#define BMA2x2_HIGH_HYST_IN_G(ghyst, range)    ((32 * ghyst) / range)


/** Macro to convert floating point G-thresholds
    to 8-bit register values<br>
  * Example: BMA2x2_SLOPE_TH_IN_G( 1.2, 2.0)
  * generates the register value for 1.2G threshold in 2G mode.
  * \brief convert g-values to 8-bit value
 */

#define BMA2x2_SLOPE_THRES_IN_G(gthres, range)    ((128 * gthres) / range)
/******************************************/
/**\name FUNCTION DECLARATION  */
/******************************************/

bstdr_ret_t bma2x2_init(bma2x2_t *bma2x2);

bstdr_ret_t bma2x2_check_connection(void);

bstdr_ret_t bma2x2_read_accel_xyz(bma2x2_xyz_t *accel);
bstdr_ret_t bma2x2_read_temp(int8_t *temp_int8_t);

bstdr_ret_t bma2x2_set_range(uint8_t range_u8);
bstdr_ret_t bma2x2_set_bw(uint8_t bw_u8);
bstdr_ret_t bma2x2_set_power_mode(uint8_t power_mode_u8);
bstdr_ret_t bma2x2_soft_rst(void);

bstdr_ret_t bma2x2_get_selftest_axis(uint8_t *selftest_axis_u8);
bstdr_ret_t bma2x2_set_selftest_axis(uint8_t selftest_axis_u8);
bstdr_ret_t bma2x2_get_selftest_sign(uint8_t *selftest_sign_u8);
bstdr_ret_t bma2x2_set_selftest_sign(uint8_t selftest_sign_u8);

bstdr_ret_t bma2x2_get_fifo_stat(uint8_t *stat_fifo_u8);
bstdr_ret_t bma2x2_get_fifo_frame_count(uint8_t *frame_count_u8);
bstdr_ret_t bma2x2_get_fifo_overrun(uint8_t *fifo_overrun_u8);
bstdr_ret_t bma2x2_get_fifo_mode(uint8_t *fifo_mode_u8);
bstdr_ret_t bma2x2_set_fifo_mode(uint8_t fifo_mode_u8);
bstdr_ret_t bma2x2_get_fifo_data_select(uint8_t *fifo_data_select_u8);
bstdr_ret_t bma2x2_set_fifo_data_select(uint8_t fifo_data_select_u8);
bstdr_ret_t bma2x2_get_fifo_data_output_reg(uint8_t *output_reg_u8);

bstdr_ret_t bma2x2_write_reg(uint8_t adr_u8,uint8_t *data_u8, uint8_t len_u8);
bstdr_ret_t bma2x2_read_reg(uint8_t adr_u8,uint8_t *data_u8, uint8_t len_u8);

#endif

