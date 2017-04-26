/** \mainpage
*
****************************************************************************
* Copyright (C) 2012 - 2015 Bosch Sensortec GmbH
*
* File : bmp285.h
*
* Date : 2015/03/27
*
* Revision : 2.0.4(Pressure and Temperature compensation code revision is 1.1)
*
* Usage: Sensor Driver for BMP285 sensor
*
****************************************************************************
*
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
/*! \file bmp285.h
    \brief BMP285 Sensor Driver Support Header File */
#ifndef __BMP285_H__
#define __BMP285_H__

#include "bstdr_comm_support.h"
#include "bstdr_types.h"

#define BMP285_ENABLE_FLOAT

/***************************************************************/
/**\name	GET AND SET BITSLICE FUNCTIONS       */
/***************************************************************/
/* never change this line */
#define BMP285_GET_BITSLICE(regvar, bitname)\
	((regvar & bitname##__MSK) >> bitname##__POS)

#define BMP285_SET_BITSLICE(regvar, bitname, val)\
	((regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK))

/***************************************************************/
/**\name	COMMON USED CONSTANTS       */
/***************************************************************/
/* Constants */
#define BMP285_NULL                          (0)
#define BMP285_RETURN_FUNCTION_TYPE          bstdr_ret_t
/* shift definitions*/
#define BMP285_SHIFT_BIT_POSITION_BY_01_BIT				 (1)
#define BMP285_SHIFT_BIT_POSITION_BY_02_BITS			(2)
#define BMP285_SHIFT_BIT_POSITION_BY_03_BITS			(3)
#define BMP285_SHIFT_BIT_POSITION_BY_04_BITS			(4)
#define BMP285_SHIFT_BIT_POSITION_BY_05_BITS			(5)
#define BMP285_SHIFT_BIT_POSITION_BY_08_BITS			(8)
#define BMP285_SHIFT_BIT_POSITION_BY_10_BITS			(10)
#define BMP285_SHIFT_BIT_POSITION_BY_11_BITS			(11)
#define BMP285_SHIFT_BIT_POSITION_BY_12_BITS			(12)
#define BMP285_SHIFT_BIT_POSITION_BY_13_BITS			(13)
#define BMP285_SHIFT_BIT_POSITION_BY_14_BITS			(14)
#define BMP285_SHIFT_BIT_POSITION_BY_15_BITS			(15)
#define BMP285_SHIFT_BIT_POSITION_BY_16_BITS			(16)
#define BMP285_SHIFT_BIT_POSITION_BY_17_BITS			(17)
#define BMP285_SHIFT_BIT_POSITION_BY_18_BITS			(18)
#define BMP285_SHIFT_BIT_POSITION_BY_19_BITS			(19)
#define BMP285_SHIFT_BIT_POSITION_BY_25_BITS			(25)
#define BMP285_SHIFT_BIT_POSITION_BY_29_BITS			(29)
#define BMP285_SHIFT_BIT_POSITION_BY_30_BITS			(30)
#define BMP285_SHIFT_BIT_POSITION_BY_31_BITS			(31)
#define BMP285_SHIFT_BIT_POSITION_BY_33_BITS			(33)
#define BMP285_SHIFT_BIT_POSITION_BY_35_BITS			(35)
#define BMP285_SHIFT_BIT_POSITION_BY_47_BITS			(47)

/* numeric definitions */
#define	BMP285_PRESSURE_TEMPERATURE_CALIB_DATA_LENGTH	    (25)
#define	BMP285_GEN_READ_WRITE_DATA_LENGTH			(1)
#define	BMP285_TEMPERATURE_DATA_LENGTH				(3)
#define	BMP285_PRESSURE_DATA_LENGTH				(3)
#define	BMP285_ALL_DATA_FRAME_LENGTH				(6)
#define	BMP285_INIT_VALUE					(0)
#define	BMP285_INVALID_DATA					(0)

/************************************************/
/**\name	ERROR CODES      */
/************************************************/
#define	SUCCESS			((uint8_t)0)
#define E_BMP285_NULL_PTR         ((int8_t)-127)
#define E_BMP285_COMM_RES         ((int8_t)-1)
#define E_BMP285_OUT_OF_RANGE     ((int8_t)-2)
#define ERROR                     ((int8_t)-1)
/************************************************/
/**\name	CHIP ID DEFINITION       */
/***********************************************/
#define BMP285_CHIP_ID1		(0x56)
#define BMP285_CHIP_ID2		(0x57)
#define BMP285_CHIP_ID3		(0x58)
/************************************************/
/**\name	I2C ADDRESS DEFINITION       */
/***********************************************/
#define BMP285_I2C_ADDRESS1                  (0x76)
#define BMP285_I2C_ADDRESS2                  (0x77)
/************************************************/
/**\name	POWER MODE DEFINITION       */
/***********************************************/
/* Sensor Specific constants */
#define BMP285_SLEEP_MODE                    (0x00)
#define BMP285_FORCED_MODE                   (0x01)
#define BMP285_NORMAL_MODE                   (0x03)
#define BMP285_SOFT_RESET_CODE               (0xB6)
/************************************************/
/**\name	STANDBY TIME DEFINITION       */
/***********************************************/
#define BMP285_STANDBY_TIME_1_MS              (0x00)
#define BMP285_STANDBY_TIME_63_MS             (0x01)
#define BMP285_STANDBY_TIME_125_MS            (0x02)
#define BMP285_STANDBY_TIME_250_MS            (0x03)
#define BMP285_STANDBY_TIME_500_MS            (0x04)
#define BMP285_STANDBY_TIME_1000_MS           (0x05)
#define BMP285_STANDBY_TIME_2000_MS           (0x06)
#define BMP285_STANDBY_TIME_4000_MS           (0x07)
/************************************************/
/**\name	OVERSAMPLING DEFINITION       */
/***********************************************/
#define BMP285_OVERSAMP_SKIPPED          (0x00)
#define BMP285_OVERSAMP_1X               (0x01)
#define BMP285_OVERSAMP_2X               (0x02)
#define BMP285_OVERSAMP_4X               (0x03)
#define BMP285_OVERSAMP_8X               (0x04)
#define BMP285_OVERSAMP_16X              (0x05)
/************************************************/
/**\name	WORKING MODE DEFINITION       */
/***********************************************/
#define BMP285_ULTRA_LOW_POWER_MODE          (0x00)
#define BMP285_LOW_POWER_MODE	             (0x01)
#define BMP285_STANDARD_RESOLUTION_MODE      (0x02)
#define BMP285_HIGH_RESOLUTION_MODE          (0x03)
#define BMP285_ULTRA_HIGH_RESOLUTION_MODE    (0x04)

#define BMP285_ULTRALOWPOWER_OVERSAMP_PRESSURE          BMP285_OVERSAMP_1X
#define BMP285_ULTRALOWPOWER_OVERSAMP_TEMPERATURE       BMP285_OVERSAMP_1X

#define BMP285_LOWPOWER_OVERSAMP_PRESSURE	         BMP285_OVERSAMP_2X
#define BMP285_LOWPOWER_OVERSAMP_TEMPERATURE	         BMP285_OVERSAMP_1X

#define BMP285_STANDARDRESOLUTION_OVERSAMP_PRESSURE     BMP285_OVERSAMP_4X
#define BMP285_STANDARDRESOLUTION_OVERSAMP_TEMPERATURE  BMP285_OVERSAMP_1X

#define BMP285_HIGHRESOLUTION_OVERSAMP_PRESSURE         BMP285_OVERSAMP_8X
#define BMP285_HIGHRESOLUTION_OVERSAMP_TEMPERATURE      BMP285_OVERSAMP_1X

#define BMP285_ULTRAHIGHRESOLUTION_OVERSAMP_PRESSURE       BMP285_OVERSAMP_16X
#define BMP285_ULTRAHIGHRESOLUTION_OVERSAMP_TEMPERATURE    BMP285_OVERSAMP_2X
/************************************************/
/**\name	FILTER DEFINITION       */
/***********************************************/
#define BMP285_FILTER_COEFF_OFF               (0x00)
#define BMP285_FILTER_COEFF_2                 (0x01)
#define BMP285_FILTER_COEFF_4                 (0x02)
#define BMP285_FILTER_COEFF_8                 (0x03)
#define BMP285_FILTER_COEFF_16                (0x04)
/************************************************/
/**\name	DELAY TIME DEFINITION       */
/***********************************************/
#define T_INIT_MAX					(20)
/* 20/16 = 1.25 ms */
#define T_MEASURE_PER_OSRS_MAX				(37)
/* 37/16 = 2.3125 ms*/
#define T_SETUP_PRESSURE_MAX				(10)
/* 10/16 = 0.625 ms */
/************************************************/
/**\name	CALIBRATION PARAMETERS DEFINITION       */
/***********************************************/
/*calibration parameters */
#define BMP285_TEMPERATURE_CALIB_DIG_T1_LSB_REG             (0x88)
#define BMP285_TEMPERATURE_CALIB_DIG_T1_MSB_REG             (0x89)
#define BMP285_TEMPERATURE_CALIB_DIG_T2_LSB_REG             (0x8A)
#define BMP285_TEMPERATURE_CALIB_DIG_T2_MSB_REG             (0x8B)
#define BMP285_TEMPERATURE_CALIB_DIG_T3_LSB_REG             (0x8C)
#define BMP285_TEMPERATURE_CALIB_DIG_T3_MSB_REG             (0x8D)
#define BMP285_PRESSURE_CALIB_DIG_P1_LSB_REG                (0x8E)
#define BMP285_PRESSURE_CALIB_DIG_P1_MSB_REG                (0x8F)
#define BMP285_PRESSURE_CALIB_DIG_P2_LSB_REG                (0x90)
#define BMP285_PRESSURE_CALIB_DIG_P2_MSB_REG                (0x91)
#define BMP285_PRESSURE_CALIB_DIG_P3_LSB_REG                (0x92)
#define BMP285_PRESSURE_CALIB_DIG_P3_MSB_REG                (0x93)
#define BMP285_PRESSURE_CALIB_DIG_P4_LSB_REG                (0x94)
#define BMP285_PRESSURE_CALIB_DIG_P4_MSB_REG                (0x95)
#define BMP285_PRESSURE_CALIB_DIG_P5_LSB_REG                (0x96)
#define BMP285_PRESSURE_CALIB_DIG_P5_MSB_REG                (0x97)
#define BMP285_PRESSURE_CALIB_DIG_P6_LSB_REG                (0x98)
#define BMP285_PRESSURE_CALIB_DIG_P6_MSB_REG                (0x99)
#define BMP285_PRESSURE_CALIB_DIG_P7_LSB_REG                (0x9A)
#define BMP285_PRESSURE_CALIB_DIG_P7_MSB_REG                (0x9B)
#define BMP285_PRESSURE_CALIB_DIG_P8_LSB_REG                (0x9C)
#define BMP285_PRESSURE_CALIB_DIG_P8_MSB_REG                (0x9D)
#define BMP285_PRESSURE_CALIB_DIG_P9_LSB_REG                (0x9E)
#define BMP285_PRESSURE_CALIB_DIG_P9_MSB_REG                (0x9F)
/************************************************/
/**\name	REGISTER ADDRESS DEFINITION       */
/***********************************************/
#define BMP285_CHIP_ID_REG                   (0xD0)  /*Chip ID Register */
#define BMP285_RST_REG                       (0xE0) /*Softreset Register */
#define BMP285_STAT_REG                      (0xF3)  /*Status Register */
#define BMP285_CTRL_MEAS_REG                 (0xF4)  /*Ctrl Measure Register */
#define BMP285_CONFIG_REG                    (0xF5)  /*Configuration Register */
#define BMP285_PRESSURE_MSB_REG              (0xF7)  /*Pressure MSB Register */
#define BMP285_PRESSURE_LSB_REG              (0xF8)  /*Pressure LSB Register */
#define BMP285_PRESSURE_XLSB_REG             (0xF9)  /*Pressure XLSB Register */
#define BMP285_TEMPERATURE_MSB_REG           (0xFA)  /*Temperature MSB Reg */
#define BMP285_TEMPERATURE_LSB_REG           (0xFB)  /*Temperature LSB Reg */
#define BMP285_TEMPERATURE_XLSB_REG          (0xFC)  /*Temperature XLSB Reg */
/************************************************/
/**\name	BIT LENGTH,POSITION AND MASK DEFINITION      */
/***********************************************/
/* Status Register */
#define BMP285_STATUS_REG_MEASURING__POS           (3)
#define BMP285_STATUS_REG_MEASURING__MSK           (0x08)
#define BMP285_STATUS_REG_MEASURING__LEN           (1)
#define BMP285_STATUS_REG_MEASURING__REG           (BMP285_STAT_REG)

#define BMP285_STATUS_REG_IM_UPDATE__POS            (0)
#define BMP285_STATUS_REG_IM_UPDATE__MSK            (0x01)
#define BMP285_STATUS_REG_IM_UPDATE__LEN            (1)
#define BMP285_STATUS_REG_IM_UPDATE__REG           (BMP285_STAT_REG)
/************************************************/
/**\name	BIT LENGTH,POSITION AND MASK DEFINITION
FOR TEMPERATURE OVERSAMPLING */
/***********************************************/
/* Control Measurement Register */
#define BMP285_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__POS             (5)
#define BMP285_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__MSK             (0xE0)
#define BMP285_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__LEN             (3)
#define BMP285_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__REG             \
(BMP285_CTRL_MEAS_REG)
/************************************************/
/**\name	BIT LENGTH,POSITION AND MASK DEFINITION
FOR PRESSURE OVERSAMPLING */
/***********************************************/
#define BMP285_CTRL_MEAS_REG_OVERSAMP_PRESSURE__POS             (2)
#define BMP285_CTRL_MEAS_REG_OVERSAMP_PRESSURE__MSK             (0x1C)
#define BMP285_CTRL_MEAS_REG_OVERSAMP_PRESSURE__LEN             (3)
#define BMP285_CTRL_MEAS_REG_OVERSAMP_PRESSURE__REG             \
(BMP285_CTRL_MEAS_REG)
/************************************************/
/**\name	BIT LENGTH,POSITION AND MASK DEFINITION
FOR POWER MODE */
/***********************************************/
#define BMP285_CTRL_MEAS_REG_POWER_MODE__POS              (0)
#define BMP285_CTRL_MEAS_REG_POWER_MODE__MSK              (0x03)
#define BMP285_CTRL_MEAS_REG_POWER_MODE__LEN              (2)
#define BMP285_CTRL_MEAS_REG_POWER_MODE__REG             (BMP285_CTRL_MEAS_REG)
/************************************************/
/**\name	BIT LENGTH,POSITION AND MASK DEFINITION
FOR STANDBY DURATION */
/***********************************************/
/* Configuration Register */
#define BMP285_CONFIG_REG_STANDBY_DURN__POS                 (5)
#define BMP285_CONFIG_REG_STANDBY_DURN__MSK                 (0xE0)
#define BMP285_CONFIG_REG_STANDBY_DURN__LEN                 (3)
#define BMP285_CONFIG_REG_STANDBY_DURN__REG                 (BMP285_CONFIG_REG)
/************************************************/
/**\name	BIT LENGTH,POSITION AND MASK DEFINITION
FOR IIR FILTER */
/***********************************************/
#define BMP285_CONFIG_REG_FILTER__POS              (2)
#define BMP285_CONFIG_REG_FILTER__MSK              (0x1C)
#define BMP285_CONFIG_REG_FILTER__LEN              (3)
#define BMP285_CONFIG_REG_FILTER__REG              (BMP285_CONFIG_REG)
/************************************************/
/**\name	BIT LENGTH,POSITION AND MASK DEFINITION
FOR SPI ENABLE*/
/***********************************************/
#define BMP285_CONFIG_REG_SPI3_ENABLE__POS             (0)
#define BMP285_CONFIG_REG_SPI3_ENABLE__MSK             (0x01)
#define BMP285_CONFIG_REG_SPI3_ENABLE__LEN             (1)
#define BMP285_CONFIG_REG_SPI3_ENABLE__REG             (BMP285_CONFIG_REG)
/************************************************/
/**\name	BIT LENGTH,POSITION AND MASK DEFINITION
FOR PRESSURE AND TEMPERATURE DATA REGISTERS */
/***********************************************/
/* Data Register */
#define BMP285_PRESSURE_XLSB_REG_DATA__POS         (4)
#define BMP285_PRESSURE_XLSB_REG_DATA__MSK         (0xF0)
#define BMP285_PRESSURE_XLSB_REG_DATA__LEN         (4)
#define BMP285_PRESSURE_XLSB_REG_DATA__REG         (BMP285_PRESSURE_XLSB_REG)

#define BMP285_TEMPERATURE_XLSB_REG_DATA__POS      (4)
#define BMP285_TEMPERATURE_XLSB_REG_DATA__MSK      (0xF0)
#define BMP285_TEMPERATURE_XLSB_REG_DATA__LEN      (4)
#define BMP285_TEMPERATURE_XLSB_REG_DATA__REG      (BMP285_TEMPERATURE_XLSB_REG)


#define BMP285_MDELAY_DATA_TYPE uint32_t
/****************************************************/
/**\name	DEFINITIONS FOR ARRAY SIZE OF DATA   */
/***************************************************/
#define	BMP285_TEMPERATURE_DATA_SIZE		(3)
#define	BMP285_PRESSURE_DATA_SIZE		(3)
#define	BMP285_DATA_FRAME_SIZE			(6)
#define	BMP285_CALIB_DATA_SIZE			(25)

#define	BMP285_TEMPERATURE_MSB_DATA		(0)
#define	BMP285_TEMPERATURE_LSB_DATA		(1)
#define	BMP285_TEMPERATURE_XLSB_DATA		(2)

#define	BMP285_PRESSURE_MSB_DATA		(0)
#define	BMP285_PRESSURE_LSB_DATA		(1)
#define	BMP285_PRESSURE_XLSB_DATA		(2)

#define	BMP285_DATA_FRAME_PRESSURE_MSB_BYTE	(0)
#define	BMP285_DATA_FRAME_PRESSURE_LSB_BYTE	(1)
#define	BMP285_DATA_FRAME_PRESSURE_XLSB_BYTE	(2)
#define	BMP285_DATA_FRAME_TEMPERATURE_MSB_BYTE	(3)
#define	BMP285_DATA_FRAME_TEMPERATURE_LSB_BYTE	(4)
#define	BMP285_DATA_FRAME_TEMPERATURE_XLSB_BYTE	(5)

/****************************************************/
/**\name	ARRAY PARAMETER FOR CALIBRATION     */
/***************************************************/
#define	BMP285_TEMPERATURE_CALIB_DIG_T1_LSB		(0)
#define	BMP285_TEMPERATURE_CALIB_DIG_T1_MSB		(1)
#define	BMP285_TEMPERATURE_CALIB_DIG_T2_LSB		(2)
#define	BMP285_TEMPERATURE_CALIB_DIG_T2_MSB		(3)
#define	BMP285_TEMPERATURE_CALIB_DIG_T3_LSB		(4)
#define	BMP285_TEMPERATURE_CALIB_DIG_T3_MSB		(5)
#define	BMP285_PRESSURE_CALIB_DIG_P1_LSB       (6)
#define	BMP285_PRESSURE_CALIB_DIG_P1_MSB       (7)
#define	BMP285_PRESSURE_CALIB_DIG_P2_LSB       (8)
#define	BMP285_PRESSURE_CALIB_DIG_P2_MSB       (9)
#define	BMP285_PRESSURE_CALIB_DIG_P3_LSB       (10)
#define	BMP285_PRESSURE_CALIB_DIG_P3_MSB       (11)
#define	BMP285_PRESSURE_CALIB_DIG_P4_LSB       (12)
#define	BMP285_PRESSURE_CALIB_DIG_P4_MSB       (13)
#define	BMP285_PRESSURE_CALIB_DIG_P5_LSB       (14)
#define	BMP285_PRESSURE_CALIB_DIG_P5_MSB       (15)
#define	BMP285_PRESSURE_CALIB_DIG_P6_LSB       (16)
#define	BMP285_PRESSURE_CALIB_DIG_P6_MSB       (17)
#define	BMP285_PRESSURE_CALIB_DIG_P7_LSB       (18)
#define	BMP285_PRESSURE_CALIB_DIG_P7_MSB       (19)
#define	BMP285_PRESSURE_CALIB_DIG_P8_LSB       (20)
#define	BMP285_PRESSURE_CALIB_DIG_P8_MSB       (21)
#define	BMP285_PRESSURE_CALIB_DIG_P9_LSB       (22)
#define	BMP285_PRESSURE_CALIB_DIG_P9_MSB       (23)
#define	BMP285_PRESSURE_CALIB_DIG_P10	       (24)
/**************************************************************/
/**\name	STRUCTURE DEFINITIONS                         */
/**************************************************************/
/*!
 * @brief This structure holds all device specific calibration parameters
 */
struct bmp285_calib_param_t {
	uint16_t dig_T1;/**<calibration T1 data*/
	int16_t dig_T2;/**<calibration T2 data*/
	int16_t dig_T3;/**<calibration T3 data*/
	uint16_t dig_P1;/**<calibration P1 data*/
	int16_t dig_P2;/**<calibration P2 data*/
	int16_t dig_P3;/**<calibration P3 data*/
	int16_t dig_P4;/**<calibration P4 data*/
	int16_t dig_P5;/**<calibration P5 data*/
	int16_t dig_P6;/**<calibration P6 data*/
	int16_t dig_P7;/**<calibration P7 data*/
	int16_t dig_P8;/**<calibration P8 data*/
	int16_t dig_P9;/**<calibration P9 data*/
	int8_t dig_P10;/**<calibration P10 data*/
	
	int32_t t_fine;/**<calibration t_fine data*/
};

/*!
 * @brief This structure holds BMP285 initialization parameters
 */
struct bmp285_t{
	struct bmp285_calib_param_t calib_param;/**<calibration data*/

	uint8_t chip_id;/**< chip id of the sensor*/
	uint8_t dev_addr;/**< device address of the sensor*/

	uint8_t oversamp_temperature;/**< temperature over sampling*/
	uint8_t oversamp_pressure;/**< pressure over sampling*/

	sensor_read bus_read;/**< bus write function pointer*/
	sensor_write bus_write;/**< bus read function pointer*/
	delay_msec delay_ms;/**< delay function pointer*/
};

/**************************************************************/
/**\name	FUNCTION DECLARATIONS                         */
/**************************************************************/
/**************************************************************/
/**\name	FUNCTION FOR  INTIALIZATION                       */
/**************************************************************/
/*!
 *	@brief This function is used for initialize
 *	the bus read and bus write functions
 *  and assign the chip id and I2C address of the BMP285 sensor
 *	chip id is read in the register 0xD0 bit from 0 to 7
 *
 *	@param *bmp285 structure pointer.
 *
 *	@note While changing the parameter of the p_bmp285
 *	@note consider the following point:
 *	Changing the reference value of the parameter
 *	will changes the local copy or local reference
 *	make sure your changes will not
 *	affect the reference value of the parameter
 *	(Better case don't change the reference value of the parameter)
 *
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMP285_RETURN_FUNCTION_TYPE bmp285_init(struct bmp285_t *bmp285);
/**************************************************************/
/**\name	FUNCTION FOR READ UNCOMPENSATED TEMPERATURE     */
/**************************************************************/
/*!
 *	@brief This API is used to read uncompensated temperature
 *	in the registers 0xFA, 0xFB and 0xFC
 *	@note 0xFA -> MSB -> bit from 0 to 7
 *	@note 0xFB -> LSB -> bit from 0 to 7
 *	@note 0xFC -> LSB -> bit from 4 to 7
 *
 *	@param v_uncomp_temperature_s32 : The uncompensated temperature.
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMP285_RETURN_FUNCTION_TYPE bmp285_read_uncomp_temperature(int32_t *v_uncomp_temperature_s32);
/**************************************************************/
/**\name	FUNCTION FOR READ TRUE TEMPERATURE S32 OUTPUT    */
/**************************************************************/
/*!
 *	@brief Reads actual temperature
 *	from uncompensated temperature
 *	@note Returns the value in 0.01 degree Centigrade
 *	@note Output value of "5123" equals 51.23 DegC.
 *
 *
 *
 *  @param v_uncomp_temperature_s32 : value of uncompensated temperature
 *
 *
 *
 *  @return Actual temperature output as int32_t
 *
*/
int32_t bmp285_compensate_temperature_int32(int32_t v_uncomp_temperature_s32);
/**************************************************************/
/**\name	FUNCTION FOR READ UNCOMPENSATED PRESSURE     */
/**************************************************************/
/*!
 *	@brief This API is used to read uncompensated pressure.
 *	in the registers 0xF7, 0xF8 and 0xF9
 *	@note 0xF7 -> MSB -> bit from 0 to 7
 *	@note 0xF8 -> LSB -> bit from 0 to 7
 *	@note 0xF9 -> LSB -> bit from 4 to 7
 *
 *
 *
 *	@param v_uncomp_pressure_s32 : The value of uncompensated pressure
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMP285_RETURN_FUNCTION_TYPE bmp285_read_uncomp_pressure(int32_t *v_uncomp_pressure_s32);
/**************************************************************/
/**\name	FUNCTION FOR READ TRUE PRESSURE S32 OUTPUT    */
/**************************************************************/
/*!
 *	@brief Reads actual pressure from uncompensated pressure
 *	and returns the value in Pascal(Pa)
 *	@note Output value of "96386" equals 96386 Pa =
 *	963.86 hPa = 963.86 millibar
 *
 *  @param  v_uncomp_pressure_s32: value of uncompensated pressure
 *
 *  @return Returns the Actual pressure out put as int32_t
 *
*/
uint32_t bmp285_compensate_pressure_int32(int32_t v_uncomp_pressure_s32);
/**************************************************************/
/**\name	FUNCTION FOR READ UNCOMPENSATED TEMPERATURE AND PRESSURE  */
/**************************************************************/
/*!
 * @brief reads uncompensated pressure and temperature
 *
 *
 * @param  v_uncomp_pressure_s32: The value of uncompensated pressure.
 * @param  v_uncomp_temperature_s32: The value of uncompensated temperature.
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMP285_RETURN_FUNCTION_TYPE bmp285_read_uncomp_pressure_temperature(
		int32_t *v_uncomp_pressure_s32, int32_t *v_uncomp_temperature_s32);
/**************************************************************/
/**\name	FUNCTION FOR READ TRUE TEMPERATURE AND PRESSURE    */
/**************************************************************/
/*!
 * @brief This API reads the true pressure and temperature
 *
 *
 *  @param  v_pressure_u32 : The value of compensated pressure.
 *  @param  v_temperature_s32 : The value of compensated temperature.
 *
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMP285_RETURN_FUNCTION_TYPE bmp285_read_pressure_temperature(
		uint32_t *v_pressure_u32, int32_t *v_pressure_s32);
/**************************************************************/
/**\name	FUNCTION FOR READ CALIBRATION DATA    */
/**************************************************************/
/*!
 *	@brief This API is used to
 *	calibration parameters used for calculation in the registers
 *
 *  parameter | Register address |   bit
 *------------|------------------|----------------
 *	dig_T1    |  0x88 and 0x89   | from 0 : 7 to 8: 15
 *	dig_T2    |  0x8A and 0x8B   | from 0 : 7 to 8: 15
 *	dig_T3    |  0x8C and 0x8D   | from 0 : 7 to 8: 15
 *	dig_P1    |  0x8E and 0x8F   | from 0 : 7 to 8: 15
 *	dig_P2    |  0x90 and 0x91   | from 0 : 7 to 8: 15
 *	dig_P3    |  0x92 and 0x93   | from 0 : 7 to 8: 15
 *	dig_P4    |  0x94 and 0x95   | from 0 : 7 to 8: 15
 *	dig_P5    |  0x96 and 0x97   | from 0 : 7 to 8: 15
 *	dig_P6    |  0x98 and 0x99   | from 0 : 7 to 8: 15
 *	dig_P7    |  0x9A and 0x9B   | from 0 : 7 to 8: 15
 *	dig_P8    |  0x9C and 0x9D   | from 0 : 7 to 8: 15
 *	dig_P9    |  0x9E and 0x9F   | from 0 : 7 to 8: 15
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMP285_RETURN_FUNCTION_TYPE bmp285_get_calib_param(void);
/**************************************************************/
/**\name	FUNCTION FOR OVERSAMPLING TEMPERATURE AND PRESSURE    */
/**************************************************************/
/*!
 *	@brief This API is used to get
 *	the temperature oversampling setting in the register 0xF4
 *	bits from 5 to 7
 *
 *        value             | Temperature oversampling
 *  ------------------------|------------------------------
 *       0x00               |  BMP285_OVERSAMP_SKIPPED
 *       0x01               |  BMP285_OVERSAMP_1X
 *       0x02               |  BMP285_OVERSAMP_2X
 *       0x03               |  BMP285_OVERSAMP_4X
 *       0x04               |  BMP285_OVERSAMP_8X
 *       0x05,0x06 and 0x07 |  BMP285_OVERSAMP_16X
 *
 *
 *  @param v_value_u8 :The value of temperature over sampling
 *
 *
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMP285_RETURN_FUNCTION_TYPE bmp285_get_oversamp_temperature(uint8_t *v_value_u8);
/*!
 *	@brief This API is used to set
 *	the temperature oversampling setting in the register 0xF4
 *	bits from 5 to 7
 *
 *        value             | Temperature oversampling
 *  ------------------------|------------------------------
 *       0x00               |  BMP285_OVERSAMP_SKIPPED
 *       0x01               |  BMP285_OVERSAMP_1X
 *       0x02               |  BMP285_OVERSAMP_2X
 *       0x03               |  BMP285_OVERSAMP_4X
 *       0x04               |  BMP285_OVERSAMP_8X
 *       0x05,0x06 and 0x07 |  BMP285_OVERSAMP_16X
 *
 *
 *  @param v_value_u8 :The value of temperature over sampling
 *
 *
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMP285_RETURN_FUNCTION_TYPE bmp285_set_oversamp_temperature(uint8_t v_value_u8);
/*!
 *	@brief This API is used to get
 *	the pressure oversampling setting in the register 0xF4
 *	bits from 2 to 4
 *
 *        value             | Pressure oversampling
 *  ------------------------|------------------------------
 *       0x00               |  BMP285_OVERSAMP_SKIPPED
 *       0x01               |  BMP285_OVERSAMP_1X
 *       0x02               |  BMP285_OVERSAMP_2X
 *       0x03               |  BMP285_OVERSAMP_4X
 *       0x04               |  BMP285_OVERSAMP_8X
 *       0x05,0x06 and 0x07 |  BMP285_OVERSAMP_16X
 *
 *
 *  @param  v_value_u8 : The value of pressure over sampling
 *
 *
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMP285_RETURN_FUNCTION_TYPE bmp285_get_oversamp_pressure(uint8_t *v_value_u8);
/*!
 *	@brief This API is used to set
 *	the pressure oversampling setting in the register 0xF4
 *	bits from 2 to 4
 *
 *        value             | Pressure oversampling
 *  ------------------------|------------------------------
 *       0x00               |  BMP285_OVERSAMP_SKIPPED
 *       0x01               |  BMP285_OVERSAMP_1X
 *       0x02               |  BMP285_OVERSAMP_2X
 *       0x03               |  BMP285_OVERSAMP_4X
 *       0x04               |  BMP285_OVERSAMP_8X
 *       0x05,0x06 and 0x07 |  BMP285_OVERSAMP_16X
 *
 *
 *  @param  v_value_u8 : The value of pressure over sampling
 *
 *
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMP285_RETURN_FUNCTION_TYPE bmp285_set_oversamp_pressure(uint8_t v_value_u8);
/**************************************************************/
/**\name	FUNCTION FOR POWER MODE    */
/**************************************************************/
/*!
 *	@brief This API used to get the
 *	Operational Mode from the sensor in the register 0xF4 bit 0 and 1
 *
 *
 *
 *	@param v_power_mode_u8 : The value of power mode value
 *  value            |   Power mode
 * ------------------|------------------
 *	0x00             | BMP285_SLEEP_MODE
 *	0x01 and 0x02    | BMP285_FORCED_MODE
 *	0x03             | BMP285_NORMAL_MODE
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMP285_RETURN_FUNCTION_TYPE bmp285_get_power_mode(uint8_t *v_power_mode_u8);
/*!
 *	@brief This API used to set the
 *	Operational Mode from the sensor in the register 0xF4 bit 0 and 1
 *
 *
 *
 *	@param v_power_mode_u8 : The value of power mode value
 *  value            |   Power mode
 * ------------------|------------------
 *	0x00             | BMP285_SLEEP_MODE
 *	0x01 and 0x02    | BMP285_FORCED_MODE
 *	0x03             | BMP285_NORMAL_MODE
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMP285_RETURN_FUNCTION_TYPE bmp285_set_power_mode(uint8_t v_power_mode_u8);
/**************************************************************/
/**\name	FUNCTION FOR SOFT RESET   */
/**************************************************************/
/*!
 * @brief Used to reset the sensor
 * The value 0xB6 is written to the
 * 0xE0 register the device is reset using the
 * complete power-on-reset procedure.
 * Softreset can be easily set using bmp285_set_softreset().
 *
 * @note Usage Hint : bmp285_set_softreset()
 *
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMP285_RETURN_FUNCTION_TYPE bmp285_set_soft_rst(void);
/**************************************************************/
/**\name	FUNCTION FOR SPI ENABLE    */
/**************************************************************/
/*!
 *	@brief This API used to get the sensor
 *	SPI mode(communication type) in the register 0xF5 bit 0
 *
 *
 *
 *	@param v_enable_disable_u8 : The spi3 enable or disable state
 *    value    | Description
 *  -----------|---------------
 *     0       | Disable
 *     1       | Enable
 *
 *
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMP285_RETURN_FUNCTION_TYPE bmp285_get_spi3(uint8_t *v_enable_disable_u8);
/*!
 *	@brief This API used to set the sensor
 *	SPI mode(communication type) in the register 0xF5 bit 0
 *
 *
 *
 *	@param v_enable_disable_u8 : The spi3 enable or disable state
 *    value    | Description
 *  -----------|---------------
 *     0       | Disable
 *     1       | Enable
 *
 *
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMP285_RETURN_FUNCTION_TYPE bmp285_set_spi3(uint8_t v_enable_disable_u8);
/**************************************************************/
/**\name	FUNCTION FOR IIR FILTER SETTING   */
/**************************************************************/
/*!
 *	@brief This API is used to reads filter setting
 *	in the register 0xF5 bit 3 and 4
 *
 *
 *
 *	@param v_value_u8 : The value of filter coefficient
 *	value	    |	Filter coefficient
 * -------------|-------------------------
 *	0x00        | BMP285_FILTER_COEFF_OFF
 *	0x01        | BMP285_FILTER_COEFF_2
 *	0x02        | BMP285_FILTER_COEFF_4
 *	0x03        | BMP285_FILTER_COEFF_8
 *	0x04        | BMP285_FILTER_COEFF_16
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMP285_RETURN_FUNCTION_TYPE bmp285_get_filter(uint8_t *v_value_u8);
/*!
 *	@brief This API is used to write filter setting
 *	in the register 0xF5 bit 3 and 4
 *
 *
 *
 *	@param v_value_u8 : The value of filter coefficient
 *	value	    |	Filter coefficient
 * -------------|-------------------------
 *	0x00        | BMP285_FILTER_COEFF_OFF
 *	0x01        | BMP285_FILTER_COEFF_2
 *	0x02        | BMP285_FILTER_COEFF_4
 *	0x03        | BMP285_FILTER_COEFF_8
 *	0x04        | BMP285_FILTER_COEFF_16
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMP285_RETURN_FUNCTION_TYPE bmp285_set_filter(uint8_t v_value_u8);
/**************************************************************/
/**\name	FUNCTION FOR STANDBY DURATION   */
/**************************************************************/
/*!
 *	@brief This API used to Read the
 *	standby duration time from the sensor in the register 0xF5 bit 5 to 7
 *
 *	@param v_standby_durn_u8 : The standby duration time value.
 *  value     |  standby duration
 * -----------|--------------------
 *    0x00    | BMP285_STANDBYTIME_1_MS
 *    0x01    | BMP285_STANDBYTIME_63_MS
 *    0x02    | BMP285_STANDBYTIME_125_MS
 *    0x03    | BMP285_STANDBYTIME_250_MS
 *    0x04    | BMP285_STANDBYTIME_500_MS
 *    0x05    | BMP285_STANDBYTIME_1000_MS
 *    0x06    | BMP285_STANDBYTIME_2000_MS
 *    0x07    | BMP285_STANDBYTIME_4000_MS
 *
 *
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMP285_RETURN_FUNCTION_TYPE bmp285_get_standby_durn(uint8_t *v_standby_durn_u8);
/*!
 *	@brief This API used to Read the
 *	standby duration time from the sensor in the register 0xF5 bit 5 to 7
 *	@note Normal mode comprises an automated perpetual cycling between an (active)
 *	Measurement period and an (inactive) standby period.
 *	@note The standby time is determined by the contents of the register t_sb.
 *	Standby time can be set using BMP285_STANDBYTIME_125_MS.
 *
 *	@note bmp285_set_standby_durn(BMP285_STANDBYTIME_125_MS)
 *
 *
 *
 *	@param v_standby_durn_u8 : The standby duration time value.
 *  value     |  standby duration
 * -----------|--------------------
 *    0x00    | BMP285_STANDBYTIME_1_MS
 *    0x01    | BMP285_STANDBYTIME_63_MS
 *    0x02    | BMP285_STANDBYTIME_125_MS
 *    0x03    | BMP285_STANDBYTIME_250_MS
 *    0x04    | BMP285_STANDBYTIME_500_MS
 *    0x05    | BMP285_STANDBYTIME_1000_MS
 *    0x06    | BMP285_STANDBYTIME_2000_MS
 *    0x07    | BMP285_STANDBYTIME_4000_MS
 *
 *
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMP285_RETURN_FUNCTION_TYPE bmp285_set_standby_durn(uint8_t v_standby_durn_u8);
/**************************************************************/
/**\name	FUNCTION FOR WORK MODE   */
/**************************************************************/
/*!
 *	@brief This API is used to write
 *	 the working mode of the sensor
 *
 *
 *  @param v_work_mode_u8 : The value of work mode
 *   value      |  mode
 * -------------|-------------
 *    0         | BMP285_ULTRA_LOW_POWER_MODE
 *    1         | BMP285_LOW_POWER_MODE
 *    2         | BMP285_STANDARD_RESOLUTION_MODE
 *    3         | BMP285_HIGH_RESOLUTION_MODE
 *    4         | BMP285_ULTRA_HIGH_RESOLUTION_MODE
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMP285_RETURN_FUNCTION_TYPE bmp285_set_work_mode(uint8_t v_work_mode_u8);
/**************************************************************/
/**\name	FUNCTION FOR FORCE MODE READING    */
/**************************************************************/
/*!
 *	@brief This API used to read both
 *	uncompensated pressure and temperature in forced mode
 *
 *
 *  @param  v_uncomp_pressure_s32: The value of uncompensated pressure.
 *  @param  v_uncomp_temperature_s32: The value of uncompensated temperature
 *
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMP285_RETURN_FUNCTION_TYPE bmp285_get_forced_uncomp_pressure_temperature(
		int32_t *v_uncomp_pressure_s32, int32_t *v_uncomp_temperature_s32);
/**************************************************************/
/**\name	FUNCTION FOR COMMON READ AND WRITE    */
/**************************************************************/
/*!
 * @brief
 *	This API write the data to
 *	the given register
 *
 *
 *	@param v_addr_u8 -> Address of the register
 *	@param v_data_u8 -> The data from the register
 *	@param v_len_u8 -> no of bytes to read
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
BMP285_RETURN_FUNCTION_TYPE bmp285_write_register(uint8_t v_addr_u8,
		uint8_t *v_data_u8, uint8_t v_len_u8);
/*!
 * @brief
 *	This API reads the data from
 *	the given register
 *
 *
 *	@param v_addr_u8 -> Address of the register
 *	@param v_data_u8 -> The data from the register
 *	@param v_len_u8 -> no of bytes to read
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
BMP285_RETURN_FUNCTION_TYPE bmp285_read_register(uint8_t v_addr_u8,
		uint8_t *v_data_u8, uint8_t v_len_u8);
/**************************************************************/
/**\name	FUNCTION FOR TRUE TEMPERATURE CALCULATION   */
/**************************************************************/
#ifdef BMP285_ENABLE_FLOAT
/*!
 * @brief This API used to read
 * actual temperature from uncompensated temperature
 * @note Returns the value in Degree centigrade
 * @note Output value of "51.23" equals 51.23 DegC.
 *
 *
 *
 *  @param v_uncomp_temperature_s32 : value of uncompensated temperature
 *
 *
 *
 *  @return
 *	Actual temperature in floating point
 *
*/
float bmp285_compensate_temperature_float(int32_t v_uncomp_temperature_s32);
/**************************************************************/
/**\name	FUNCTION FOR TRUE PRESSURE CALCULATION   */
/**************************************************************/
/*!
 *	@brief Reads actual pressure from uncompensated pressure
 *	and returns pressure in Pa as float.
 *	@note Output value of "96386.2"
 *	equals 96386.2 Pa = 963.862 hPa.
 *
 *
 *
 *  @param v_uncomp_pressure_s32 : value of uncompensated pressure
 *
 *
 *
 *  @return
 *	Actual pressure in floating point
 *
*/
float bmp285_compensate_pressure_float(int32_t v_uncomp_pressure_s32);
#endif
#if defined(BMP285_ENABLE_INT64) && defined(BMP285_64BITSUPPORT_PRESENT)
/*!
 * @brief This API used to read actual pressure from uncompensated pressure
 * @note returns the value in Pa as unsigned 32 bit
 * integer in Q24.8 format (24 integer bits and
 * 8 fractional bits). Output value of "24674867"
 * represents 24674867 / 256 = 96386.2 Pa = 963.862 hPa
 *
 *
 *
 *  @param v_uncomp_pressure_s32 : value of uncompensated pressure
 *
 *
 *
 *  @return actual pressure as 64bit output
 *
*/
uint32_t bmp285_compensate_pressure_int64(int32_t v_uncomp_pressure_s32);
#endif
/**************************************************************/
/**\name	FUNCTION FOR DELAY CALCULATION DURING FORCEMODE  */
/**************************************************************/
/*!
 * @brief Computing waiting time for sensor data read
 *
 *
 *
 *
 *  @param v_delaytime_u8r: The value of delay time
 *
 *
 *  @return 0
 *
 *
 */
BMP285_RETURN_FUNCTION_TYPE bmp285_compute_wait_time(uint8_t
		*v_delaytime_u8r);

#endif
