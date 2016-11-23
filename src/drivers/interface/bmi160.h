/** \mainpage
*
****************************************************************************
* Copyright (C) 2014 Bosch Sensortec GmbH
*
* File : bmi160.h
*
* Date : 2014/11/19
*
* Revision : 2.0.4 $
*
* Usage: Sensor Driver for BMI160 sensor
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
/*! \file bmi160.h
    \brief BMI160 Sensor Driver Support Header File */
/* user defined code to be added here ... */
#ifndef __BMI160_H__
#define __BMI160_H__

#include "bstdr_types.h"
/**************************************************************/
/**\name	STRUCTURE DEFINITIONS                             */
/**************************************************************/
/*!
 * @brief Structure containing gyro xyz data
 */
typedef struct {
	int16_t x;	/**<gyro X  data*/
	int16_t y;	/**<gyro Y  data*/
	int16_t z;	/**<gyro Z  data*/
}bmi160_xyz_t;

/*!
*	@brief bmi160 structure
*	This structure holds all relevant information about bmi160
*/
typedef struct{
	uint8_t chip_id;		/**< chip id of BMI160 */
	uint8_t dev_addr;		/**< device identifier of BMI160 */

	bmi160_xyz_t gyro_data;	/**< gyro data after data read */
	bmi160_xyz_t acc_data;	/**< accel data after data read */
	bmi160_xyz_t mag_data;	/**< mag data after data read */

	sensor_write bus_write; /**< generic write function pointer */
	sensor_read bus_read; 	/**< generic read function pointer */
	delay_msec delay; /**< generic read function pointer */
}bmi160_t;


typedef enum
{
	BMI160_OK = 0,
	BMI160_E_GEN_ERROR = -1,
	BMI160_E_OUT_OF_RANGE = -2,
	BMI160_E_BUSY = -3,
	BMI160_E_CON_ERROR= -4,
	BMI160_E_CHIPID_ERROR= -5,
	BMI160_E_NULL_PTR = -127
}bmi160_ret_t;

/*******************************************/
/**\name	I2C Addresses    				*/
/******************************************/
#define BMI160_I2C_ADDR1	0x68 /**< I2C Address needs to be changed */
#define BMI160_I2C_ADDR2    0x69 /**< I2C Address needs to be changed */

/*******************************************/
/**\name	CONSTANTS        */
/******************************************/
#define	BMI160_MAXIMUM_TIMEOUT            ((uint8_t)10)

#define BMI160_CHIP_ID				0xD1

/* Constants */
#define BMI160_DELAY_SETTLING_TIME		5

/****************************************************/
/**\name	REGISTER DEFINITIONS       */
/***************************************************/
/*******************/
/**\name CHIP ID */
/*******************/
#define BMI160_USER_CHIP_ID_ADDR				0x00
/*******************/
/**\name ERROR STATUS */
/*******************/
#define BMI160_USER_ERROR_ADDR					0X02
/*******************/
/**\name POWER MODE STATUS */
/*******************/
#define BMI160_USER_PMU_STAT_ADDR				0X03
/*******************/
/**\name MAG DATA REGISTERS */
/*******************/
#define BMI160_USER_DATA_0_ADDR					0X04
#define BMI160_USER_DATA_1_ADDR					0X05
#define BMI160_USER_DATA_2_ADDR					0X06
#define BMI160_USER_DATA_3_ADDR					0X07
#define BMI160_USER_DATA_4_ADDR					0X08
#define BMI160_USER_DATA_5_ADDR					0X09
#define BMI160_USER_DATA_6_ADDR					0X0A
#define BMI160_USER_DATA_7_ADDR					0X0B
/*******************/
/**\name GYRO DATA REGISTERS */
/*******************/
#define BMI160_USER_DATA_8_ADDR					0X0C
#define BMI160_USER_DATA_9_ADDR					0X0D
#define BMI160_USER_DATA_10_ADDR				0X0E
#define BMI160_USER_DATA_11_ADDR				0X0F
#define BMI160_USER_DATA_12_ADDR				0X10
#define BMI160_USER_DATA_13_ADDR				0X11
#define BMI160_USER_DATA_14_ADDR				0X12
#define BMI160_USER_DATA_15_ADDR				0X13
/*******************/
/**\name ACCEL DATA REGISTERS */
/*******************/
#define BMI160_USER_DATA_16_ADDR				0X14
#define BMI160_USER_DATA_17_ADDR				0X15
#define BMI160_USER_DATA_18_ADDR				0X16
#define BMI160_USER_DATA_19_ADDR				0X17
/*******************/
/**\name SENSOR TIME REGISTERS */
/*******************/
#define BMI160_USER_SENSORTIME_0_ADDR			0X18
#define BMI160_USER_SENSORTIME_1_ADDR			0X19
#define BMI160_USER_SENSORTIME_2_ADDR			0X1A
/*******************/
/**\name STATUS REGISTER FOR SENSOR STATUS FLAG */
/*******************/
#define BMI160_USER_STAT_ADDR					0X1B
/*******************/
/**\name INTERRUPY STATUS REGISTERS */
/*******************/
#define BMI160_USER_INTR_STAT_0_ADDR			0X1C
#define BMI160_USER_INTR_STAT_1_ADDR			0X1D
#define BMI160_USER_INTR_STAT_2_ADDR			0X1E
#define BMI160_USER_INTR_STAT_3_ADDR			0X1F
/*******************/
/**\name TEMPERATURE REGISTERS */
/*******************/
#define BMI160_USER_TEMPERATURE_0_ADDR			0X20
#define BMI160_USER_TEMPERATURE_1_ADDR			0X21
/*******************/
/**\name FIFO REGISTERS */
/*******************/
#define BMI160_USER_FIFO_LENGTH_0_ADDR			0X22
#define BMI160_USER_FIFO_LENGTH_1_ADDR			0X23
#define BMI160_USER_FIFO_DATA_ADDR				0X24
/***************************************************/
/**\name ACCEL CONFIG REGISTERS  FOR ODR, BANDWIDTH AND UNDERSAMPLING*/
/******************************************************/
#define BMI160_USER_ACCEL_CONFIG_ADDR			0X40
/*******************/
/**\name ACCEL RANGE */
/*******************/
#define BMI160_USER_ACCEL_RANGE_ADDR            0X41
/***************************************************/
/**\name GYRO CONFIG REGISTERS  FOR ODR AND BANDWIDTH */
/******************************************************/
#define BMI160_USER_GYRO_CONFIG_ADDR            0X42
/*******************/
/**\name GYRO RANGE */
/*******************/
#define BMI160_USER_GYRO_RANGE_ADDR             0X43
/***************************************************/
/**\name MAG CONFIG REGISTERS  FOR ODR*/
/******************************************************/
#define BMI160_USER_MAG_CONFIG_ADDR				0X44
/***************************************************/
/**\name REGISTER FOR GYRO AND ACCEL DOWNSAMPLING RATES FOR FIFO*/
/******************************************************/
#define BMI160_USER_FIFO_DOWN_ADDR              0X45
/***************************************************/
/**\name FIFO CONFIG REGISTERS*/
/******************************************************/
#define BMI160_USER_FIFO_CONFIG_0_ADDR          0X46
#define BMI160_USER_FIFO_CONFIG_1_ADDR          0X47
/***************************************************/
/**\name MAG INTERFACE REGISTERS*/
/******************************************************/
#define BMI160_USER_MAG_IF_0_ADDR				0X4B
#define BMI160_USER_MAG_IF_1_ADDR				0X4C
#define BMI160_USER_MAG_IF_2_ADDR				0X4D
#define BMI160_USER_MAG_IF_3_ADDR				0X4E
#define BMI160_USER_MAG_IF_4_ADDR				0X4F
/***************************************************/
/**\name INTERRUPT ENABLE REGISTERS*/
/******************************************************/
#define BMI160_USER_INTR_ENABLE_0_ADDR			0X50
#define BMI160_USER_INTR_ENABLE_1_ADDR			0X51
#define BMI160_USER_INTR_ENABLE_2_ADDR			0X52
#define BMI160_USER_INTR_OUT_CTRL_ADDR			0X53
/***************************************************/
/**\name LATCH DURATION REGISTERS*/
/******************************************************/
#define BMI160_USER_INTR_LATCH_ADDR				0X54
/***************************************************/
/**\name MAP INTERRUPT 1 and 2 REGISTERS*/
/******************************************************/
#define BMI160_USER_INTR_MAP_0_ADDR				0X55
#define BMI160_USER_INTR_MAP_1_ADDR				0X56
#define BMI160_USER_INTR_MAP_2_ADDR				0X57
/***************************************************/
/**\name DATA SOURCE REGISTERS*/
/******************************************************/
#define BMI160_USER_INTR_DATA_0_ADDR			0X58
#define BMI160_USER_INTR_DATA_1_ADDR			0X59
/***************************************************/
/**\name 
INTERRUPT THRESHOLD, HYSTERESIS, DURATION, MODE CONFIGURATION REGISTERS*/
/******************************************************/
#define BMI160_USER_INTR_LOWHIGH_0_ADDR			0X5A
#define BMI160_USER_INTR_LOWHIGH_1_ADDR			0X5B
#define BMI160_USER_INTR_LOWHIGH_2_ADDR			0X5C
#define BMI160_USER_INTR_LOWHIGH_3_ADDR			0X5D
#define BMI160_USER_INTR_LOWHIGH_4_ADDR			0X5E
#define BMI160_USER_INTR_MOTION_0_ADDR			0X5F
#define BMI160_USER_INTR_MOTION_1_ADDR			0X60
#define BMI160_USER_INTR_MOTION_2_ADDR			0X61
#define BMI160_USER_INTR_MOTION_3_ADDR			0X62
#define BMI160_USER_INTR_TAP_0_ADDR				0X63
#define BMI160_USER_INTR_TAP_1_ADDR				0X64
#define BMI160_USER_INTR_ORIENT_0_ADDR			0X65
#define BMI160_USER_INTR_ORIENT_1_ADDR			0X66
#define BMI160_USER_INTR_FLAT_0_ADDR			0X67
#define BMI160_USER_INTR_FLAT_1_ADDR			0X68
/***************************************************/
/**\name FAST OFFSET CONFIGURATION REGISTER*/
/******************************************************/
#define BMI160_USER_FOC_CONFIG_ADDR				0X69
/***************************************************/
/**\name MISCELLANEOUS CONFIGURATION REGISTER*/
/******************************************************/
#define BMI160_USER_CONFIG_ADDR					0X6A
/***************************************************/
/**\name SERIAL INTERFACE SETTINGS REGISTER*/
/******************************************************/
#define BMI160_USER_IF_CONFIG_ADDR				0X6B
/***************************************************/
/**\name GYRO POWER MODE TRIGGER REGISTER */
/******************************************************/
#define BMI160_USER_PMU_TRIGGER_ADDR			0X6C
/***************************************************/
/**\name SELF_TEST REGISTER*/
/******************************************************/
#define BMI160_USER_SELF_TEST_ADDR				0X6D
/***************************************************/
/**\name SPI,I2C SELECTION REGISTER*/
/******************************************************/
#define BMI160_USER_NV_CONFIG_ADDR				0x70
/***************************************************/
/**\name ACCEL AND GYRO OFFSET REGISTERS*/
/******************************************************/
#define BMI160_USER_OFFSET_0_ADDR				0X71
#define BMI160_USER_OFFSET_1_ADDR				0X72
#define BMI160_USER_OFFSET_2_ADDR				0X73
#define BMI160_USER_OFFSET_3_ADDR				0X74
#define BMI160_USER_OFFSET_4_ADDR				0X75
#define BMI160_USER_OFFSET_5_ADDR				0X76
#define BMI160_USER_OFFSET_6_ADDR				0X77
/***************************************************/
/**\name STEP COUNTER INTERRUPT REGISTERS*/
/******************************************************/
#define BMI160_USER_STEP_COUNT_0_ADDR			0X78
#define BMI160_USER_STEP_COUNT_1_ADDR			0X79
/***************************************************/
/**\name STEP COUNTER CONFIGURATION REGISTERS*/
/******************************************************/
#define BMI160_USER_STEP_CONFIG_0_ADDR			0X7A
#define BMI160_USER_STEP_CONFIG_1_ADDR			0X7B
/***************************************************/
/**\name COMMAND REGISTER*/
/******************************************************/
#define BMI160_CMD_COMMANDS_ADDR				0X7E

/* USER REGISTERS DEFINITION END */
/**************************************************************************/

/* PARAMETER DEFINITIONS */
/**************************************************/
/**\name	FIFO FRAME COUNT DEFINITION           */
/*************************************************/
#define FIFO_FRAME				1024
#define FIFO_CONFIG_CHECK1		0x00
#define FIFO_CONFIG_CHECK2		0x80

/**************************************************/
/**\name	ACCEL RANGE          */
/*************************************************/
#define BMI160_ACCEL_RANGE_2G           0X03
#define BMI160_ACCEL_RANGE_4G           0X05
#define BMI160_ACCEL_RANGE_8G           0X08
#define BMI160_ACCEL_RANGE_16G          0X0C
/**************************************************/
/**\name	ACCEL ODR          */
/*************************************************/
#define BMI160_ACCEL_OUTPUT_DATA_RATE_RESERVED       0x00
#define BMI160_ACCEL_OUTPUT_DATA_RATE_0_78HZ         0x01
#define BMI160_ACCEL_OUTPUT_DATA_RATE_1_56HZ         0x02
#define BMI160_ACCEL_OUTPUT_DATA_RATE_3_12HZ         0x03
#define BMI160_ACCEL_OUTPUT_DATA_RATE_6_25HZ         0x04
#define BMI160_ACCEL_OUTPUT_DATA_RATE_12_5HZ         0x05
#define BMI160_ACCEL_OUTPUT_DATA_RATE_25HZ           0x06
#define BMI160_ACCEL_OUTPUT_DATA_RATE_50HZ           0x07
#define BMI160_ACCEL_OUTPUT_DATA_RATE_100HZ          0x08
#define BMI160_ACCEL_OUTPUT_DATA_RATE_200HZ          0x09
#define BMI160_ACCEL_OUTPUT_DATA_RATE_400HZ          0x0A
#define BMI160_ACCEL_OUTPUT_DATA_RATE_800HZ          0x0B
#define BMI160_ACCEL_OUTPUT_DATA_RATE_1600HZ         0x0C
#define BMI160_ACCEL_OUTPUT_DATA_RATE_RESERVED0      0x0D
#define BMI160_ACCEL_OUTPUT_DATA_RATE_RESERVED1      0x0E
#define BMI160_ACCEL_OUTPUT_DATA_RATE_RESERVED2      0x0F

/**************************************************/
/**\name	ACCEL BANDWIDTH PARAMETER         */
/*************************************************/
#define BMI160_ACCEL_OSR4_AVG1			0x00
#define BMI160_ACCEL_OSR2_AVG2			0x01
#define BMI160_ACCEL_NORMAL_AVG4		0x02
#define BMI160_ACCEL_CIC_AVG8			0x03
#define BMI160_ACCEL_RES_AVG16			0x04
#define BMI160_ACCEL_RES_AVG32			0x05
#define BMI160_ACCEL_RES_AVG64			0x06
#define BMI160_ACCEL_RES_AVG128			0x07
/**************************************************/
/**\name	GYRO ODR         */
/*************************************************/
#define BMI160_GYRO_OUTPUT_DATA_RATE_RESERVED		0x00
#define BMI160_GYRO_OUTPUT_DATA_RATE_25HZ			0x06
#define BMI160_GYRO_OUTPUT_DATA_RATE_50HZ			0x07
#define BMI160_GYRO_OUTPUT_DATA_RATE_100HZ			0x08
#define BMI160_GYRO_OUTPUT_DATA_RATE_200HZ			0x09
#define BMI160_GYRO_OUTPUT_DATA_RATE_400HZ			0x0A
#define BMI160_GYRO_OUTPUT_DATA_RATE_800HZ			0x0B
#define BMI160_GYRO_OUTPUT_DATA_RATE_1600HZ			0x0C
#define BMI160_GYRO_OUTPUT_DATA_RATE_3200HZ			0x0D
/**************************************************/
/**\name	GYRO BANDWIDTH PARAMETER         */
/*************************************************/
#define BMI160_GYRO_OSR4_MODE		0x00
#define BMI160_GYRO_OSR2_MODE		0x01
#define BMI160_GYRO_NORMAL_MODE		0x02
#define BMI160_GYRO_CIC_MODE		0x03
/**************************************************/
/**\name	GYROSCOPE RANGE PARAMETER         */
/*************************************************/
#define BMI160_GYRO_RANGE_2000_DEG_SEC	0x00
#define BMI160_GYRO_RANGE_1000_DEG_SEC	0x01
#define BMI160_GYRO_RANGE_500_DEG_SEC	0x02
#define BMI160_GYRO_RANGE_250_DEG_SEC	0x03
#define BMI160_GYRO_RANGE_125_DEG_SEC	0x04

/**************************************************/
/**\name	FIFO CONFIGURATIONS    */
/*************************************************/
#define FIFO_HEADER_ENABLE			0X01
#define FIFO_MAG_ENABLE				0X01
#define FIFO_ACCEL_ENABLE			0X01
#define FIFO_GYRO_ENABLE			0X01
#define FIFO_TIME_ENABLE			0X01
#define FIFO_STOPONFULL_ENABLE		0X01
#define FIFO_WM_INTERRUPT_ENABLE	0X01

/* FIFO definitions*/
#define FIFO_HEAD_A        0x84
#define FIFO_HEAD_G        0x88
#define FIFO_HEAD_M        0x90

#define FIFO_HEAD_G_A	0x8C
#define FIFO_HEAD_M_A   0x94
#define FIFO_HEAD_M_G   0x98

#define FIFO_HEAD_M_G_A		0x9C

#define FIFO_HEAD_SENSOR_TIME			0x44
#define FIFO_HEAD_SKIP_FRAME			0x40
#define FIFO_HEAD_OVER_READ_LSB			0x80
#define FIFO_HEAD_OVER_READ_MSB			0x00


/* FIFO 1024 byte, max fifo frame count not over 150 */
#define FIFO_FRAME_CNT		146

#define	FIFO_OVER_READ_RETURN		((int8_t)-10)
#define	FIFO_SENSORTIME_RETURN		((int8_t)-9)
#define	FIFO_SKIP_OVER_LEN			((int8_t)-8)
#define	FIFO_M_G_A_OVER_LEN			((int8_t)-7)
#define	FIFO_M_G_OVER_LEN			((int8_t)-6)
#define	FIFO_M_A_OVER_LEN			((int8_t)-5)
#define	FIFO_G_A_OVER_LEN			((int8_t)-4)
#define	FIFO_M_OVER_LEN				((int8_t)-3)
#define	FIFO_G_OVER_LEN				((int8_t)-2)
#define	FIFO_A_OVER_LEN				((int8_t)-1)
/**************************************************/
/**\name	ACCEL POWER MODE    */
/*************************************************/
#define ACCEL_MODE_NORMAL	0x11
#define	ACCEL_LOWPOWER		0X12
#define	ACCEL_SUSPEND		0X10
/**************************************************/
/**\name	GYRO POWER MODE    */
/*************************************************/
#define GYRO_MODE_SUSPEND		0x14
#define GYRO_MODE_NORMAL		0x15
#define GYRO_MODE_FASTSTARTUP	0x17

/**************************************************/
/**\name	ENABLE/DISABLE BIT VALUES    */
/*************************************************/
#define BMI160_ENABLE	0x01
#define BMI160_DISABLE	0x00
/**************************************************/
/**\name	INTERRUPT EDGE TRIGGER ENABLE    */
/*************************************************/
#define BMI160_EDGE		0x01
#define BMI160_LEVEL	0x00
/**************************************************/
/**\name	INTERRUPT LEVEL ENABLE    */
/*************************************************/
#define BMI160_LEVEL_LOW		0x00
#define BMI160_LEVEL_HIGH		0x01
/**************************************************/
/**\name	INTERRUPT OUTPUT ENABLE    */
/*************************************************/
#define BMI160_OPEN_DRAIN	0x01
#define BMI160_PUSH_PULL	0x00

/* interrupt output enable*/
#define BMI160_INPUT	0x01
#define BMI160_OUTPUT	0x00


/**************************************************/
/**\name	GYRO OFFSET MASK DEFINITION   */
/*************************************************/
#define BMI160_GYRO_MANUAL_OFFSET_0_7	0x00FF
#define BMI160_GYRO_MANUAL_OFFSET_8_9	0x0300

/**************************************************/
/**\name	MAG INIT DEFINITION  */
/*************************************************/
#define BMI160_COMMAND_REG_ONE		0x37
#define BMI160_COMMAND_REG_TWO		0x9A
#define BMI160_COMMAND_REG_THREE	0xC0
#define	RESET_STEP_COUNTER			0xB2


/**************************************************************/
/**\name	USER DATA REGISTERS DEFINITION START                         */
/**************************************************************/
/* Chip ID Description - Reg Addr --> 0x00, Bit --> 0...7 */
#define BMI160_USER_CHIP_ID__POS             0
#define BMI160_USER_CHIP_ID__MSK            0xFF
#define BMI160_USER_CHIP_ID__LEN             8
#define BMI160_USER_CHIP_ID__REG             BMI160_USER_CHIP_ID_ADDR

#define FIFTY_EIGHT                         0X3A

/* Error Description - Reg Addr --> 0x02, Bit --> 0 */
#define BMI160_USER_ERR_STAT__POS               0
#define BMI160_USER_ERR_STAT__LEN               8
#define BMI160_USER_ERR_STAT__MSK               0xFF
#define BMI160_USER_ERR_STAT__REG               BMI160_USER_ERROR_ADDR

#define BMI160_USER_FATAL_ERR__POS               0
#define BMI160_USER_FATAL_ERR__LEN               1
#define BMI160_USER_FATAL_ERR__MSK               0x01
#define BMI160_USER_FATAL_ERR__REG               BMI160_USER_ERROR_ADDR

/* Error Description - Reg Addr --> 0x02, Bit --> 1...4 */
#define BMI160_USER_ERR_CODE__POS               1
#define BMI160_USER_ERR_CODE__LEN               4
#define BMI160_USER_ERR_CODE__MSK               0x1E
#define BMI160_USER_ERR_CODE__REG               BMI160_USER_ERROR_ADDR

/* Error Description - Reg Addr --> 0x02, Bit --> 5 */
#define BMI160_USER_I2C_FAIL_ERR__POS               5
#define BMI160_USER_I2C_FAIL_ERR__LEN               1
#define BMI160_USER_I2C_FAIL_ERR__MSK               0x20
#define BMI160_USER_I2C_FAIL_ERR__REG               BMI160_USER_ERROR_ADDR

/* Error Description - Reg Addr --> 0x02, Bit --> 6 */
#define BMI160_USER_DROP_CMD_ERR__POS              6
#define BMI160_USER_DROP_CMD_ERR__LEN              1
#define BMI160_USER_DROP_CMD_ERR__MSK              0x40
#define BMI160_USER_DROP_CMD_ERR__REG              BMI160_USER_ERROR_ADDR

/* Error Description - Reg Addr --> 0x02, Bit --> 7 */
#define BMI160_USER_MAG_DADA_RDY_ERR__POS               7
#define BMI160_USER_MAG_DADA_RDY_ERR__LEN               1
#define BMI160_USER_MAG_DADA_RDY_ERR__MSK               0x80
#define BMI160_USER_MAG_DADA_RDY_ERR__REG               BMI160_USER_ERROR_ADDR

/* PMU_Status Description of MAG - Reg Addr --> 0x03, Bit --> 1..0 */
#define BMI160_USER_MAG_POWER_MODE_STAT__POS		0
#define BMI160_USER_MAG_POWER_MODE_STAT__LEN		2
#define BMI160_USER_MAG_POWER_MODE_STAT__MSK		0x03
#define BMI160_USER_MAG_POWER_MODE_STAT__REG		\
BMI160_USER_PMU_STAT_ADDR

/* PMU_Status Description of GYRO - Reg Addr --> 0x03, Bit --> 3...2 */
#define BMI160_USER_GYRO_POWER_MODE_STAT__POS               2
#define BMI160_USER_GYRO_POWER_MODE_STAT__LEN               2
#define BMI160_USER_GYRO_POWER_MODE_STAT__MSK               0x0C
#define BMI160_USER_GYRO_POWER_MODE_STAT__REG		      \
BMI160_USER_PMU_STAT_ADDR

/* PMU_Status Description of ACCEL - Reg Addr --> 0x03, Bit --> 5...4 */
#define BMI160_USER_ACCEL_POWER_MODE_STAT__POS               4
#define BMI160_USER_ACCEL_POWER_MODE_STAT__LEN               2
#define BMI160_USER_ACCEL_POWER_MODE_STAT__MSK               0x30
#define BMI160_USER_ACCEL_POWER_MODE_STAT__REG		    \
BMI160_USER_PMU_STAT_ADDR

/* Mag_X(LSB) Description - Reg Addr --> 0x04, Bit --> 0...7 */
#define BMI160_USER_DATA_0_MAG_X_LSB__POS           0
#define BMI160_USER_DATA_0_MAG_X_LSB__LEN           8
#define BMI160_USER_DATA_0_MAG_X_LSB__MSK          0xFF
#define BMI160_USER_DATA_0_MAG_X_LSB__REG           BMI160_USER_DATA_0_ADDR

/* Mag_X(LSB) Description - Reg Addr --> 0x04, Bit --> 3...7 */
#define BMI160_USER_DATA_MAG_X_LSB__POS           3
#define BMI160_USER_DATA_MAG_X_LSB__LEN           5
#define BMI160_USER_DATA_MAG_X_LSB__MSK          0xF8
#define BMI160_USER_DATA_MAG_X_LSB__REG          BMI160_USER_DATA_0_ADDR

/* Mag_X(MSB) Description - Reg Addr --> 0x05, Bit --> 0...7 */
#define BMI160_USER_DATA_1_MAG_X_MSB__POS           0
#define BMI160_USER_DATA_1_MAG_X_MSB__LEN           8
#define BMI160_USER_DATA_1_MAG_X_MSB__MSK          0xFF
#define BMI160_USER_DATA_1_MAG_X_MSB__REG          BMI160_USER_DATA_1_ADDR

/* Mag_Y(LSB) Description - Reg Addr --> 0x06, Bit --> 0...7 */
#define BMI160_USER_DATA_2_MAG_Y_LSB__POS           0
#define BMI160_USER_DATA_2_MAG_Y_LSB__LEN           8
#define BMI160_USER_DATA_2_MAG_Y_LSB__MSK          0xFF
#define BMI160_USER_DATA_2_MAG_Y_LSB__REG          BMI160_USER_DATA_2_ADDR

/* Mag_Y(LSB) Description - Reg Addr --> 0x06, Bit --> 3...7 */
#define BMI160_USER_DATA_MAG_Y_LSB__POS           3
#define BMI160_USER_DATA_MAG_Y_LSB__LEN           5
#define BMI160_USER_DATA_MAG_Y_LSB__MSK          0xF8
#define BMI160_USER_DATA_MAG_Y_LSB__REG          BMI160_USER_DATA_2_ADDR

/* Mag_Y(MSB) Description - Reg Addr --> 0x07, Bit --> 0...7 */
#define BMI160_USER_DATA_3_MAG_Y_MSB__POS           0
#define BMI160_USER_DATA_3_MAG_Y_MSB__LEN           8
#define BMI160_USER_DATA_3_MAG_Y_MSB__MSK          0xFF
#define BMI160_USER_DATA_3_MAG_Y_MSB__REG          BMI160_USER_DATA_3_ADDR

/* Mag_Z(LSB) Description - Reg Addr --> 0x08, Bit --> 0...7 */
#define BMI160_USER_DATA_4_MAG_Z_LSB__POS           0
#define BMI160_USER_DATA_4_MAG_Z_LSB__LEN           8
#define BMI160_USER_DATA_4_MAG_Z_LSB__MSK          0xFF
#define BMI160_USER_DATA_4_MAG_Z_LSB__REG          BMI160_USER_DATA_4_ADDR

/* Mag_X(LSB) Description - Reg Addr --> 0x08, Bit --> 3...7 */
#define BMI160_USER_DATA_MAG_Z_LSB__POS           1
#define BMI160_USER_DATA_MAG_Z_LSB__LEN           7
#define BMI160_USER_DATA_MAG_Z_LSB__MSK          0xFE
#define BMI160_USER_DATA_MAG_Z_LSB__REG          BMI160_USER_DATA_4_ADDR

/* Mag_Z(MSB) Description - Reg Addr --> 0x09, Bit --> 0...7 */
#define BMI160_USER_DATA_5_MAG_Z_MSB__POS           0
#define BMI160_USER_DATA_5_MAG_Z_MSB__LEN           8
#define BMI160_USER_DATA_5_MAG_Z_MSB__MSK          0xFF
#define BMI160_USER_DATA_5_MAG_Z_MSB__REG          BMI160_USER_DATA_5_ADDR

/* RHALL(LSB) Description - Reg Addr --> 0x0A, Bit --> 0...7 */
#define BMI160_USER_DATA_6_RHALL_LSB__POS           0
#define BMI160_USER_DATA_6_RHALL_LSB__LEN           8
#define BMI160_USER_DATA_6_RHALL_LSB__MSK          0xFF
#define BMI160_USER_DATA_6_RHALL_LSB__REG          BMI160_USER_DATA_6_ADDR

/* Mag_R(LSB) Description - Reg Addr --> 0x0A, Bit --> 3...7 */
#define BMI160_USER_DATA_MAG_R_LSB__POS           2
#define BMI160_USER_DATA_MAG_R_LSB__LEN           6
#define BMI160_USER_DATA_MAG_R_LSB__MSK          0xFC
#define BMI160_USER_DATA_MAG_R_LSB__REG          BMI160_USER_DATA_6_ADDR

/* RHALL(MSB) Description - Reg Addr --> 0x0B, Bit --> 0...7 */
#define BMI160_USER_DATA_7_RHALL_MSB__POS           0
#define BMI160_USER_DATA_7_RHALL_MSB__LEN           8
#define BMI160_USER_DATA_7_RHALL_MSB__MSK          0xFF
#define BMI160_USER_DATA_7_RHALL_MSB__REG          BMI160_USER_DATA_7_ADDR

/* GYR_X (LSB) Description - Reg Addr --> 0x0C, Bit --> 0...7 */
#define BMI160_USER_DATA_8_GYRO_X_LSB__POS           0
#define BMI160_USER_DATA_8_GYRO_X_LSB__LEN           8
#define BMI160_USER_DATA_8_GYRO_X_LSB__MSK          0xFF
#define BMI160_USER_DATA_8_GYRO_X_LSB__REG          BMI160_USER_DATA_8_ADDR

/* GYR_X (MSB) Description - Reg Addr --> 0x0D, Bit --> 0...7 */
#define BMI160_USER_DATA_9_GYRO_X_MSB__POS           0
#define BMI160_USER_DATA_9_GYRO_X_MSB__LEN           8
#define BMI160_USER_DATA_9_GYRO_X_MSB__MSK          0xFF
#define BMI160_USER_DATA_9_GYRO_X_MSB__REG          BMI160_USER_DATA_9_ADDR

/* GYR_Y (LSB) Description - Reg Addr --> 0x0E, Bit --> 0...7 */
#define BMI160_USER_DATA_10_GYRO_Y_LSB__POS           0
#define BMI160_USER_DATA_10_GYRO_Y_LSB__LEN           8
#define BMI160_USER_DATA_10_GYRO_Y_LSB__MSK          0xFF
#define BMI160_USER_DATA_10_GYRO_Y_LSB__REG          BMI160_USER_DATA_10_ADDR

/* GYR_Y (MSB) Description - Reg Addr --> 0x0F, Bit --> 0...7 */
#define BMI160_USER_DATA_11_GYRO_Y_MSB__POS           0
#define BMI160_USER_DATA_11_GYRO_Y_MSB__LEN           8
#define BMI160_USER_DATA_11_GYRO_Y_MSB__MSK          0xFF
#define BMI160_USER_DATA_11_GYRO_Y_MSB__REG          BMI160_USER_DATA_11_ADDR

/* GYR_Z (LSB) Description - Reg Addr --> 0x10, Bit --> 0...7 */
#define BMI160_USER_DATA_12_GYRO_Z_LSB__POS           0
#define BMI160_USER_DATA_12_GYRO_Z_LSB__LEN           8
#define BMI160_USER_DATA_12_GYRO_Z_LSB__MSK          0xFF
#define BMI160_USER_DATA_12_GYRO_Z_LSB__REG          BMI160_USER_DATA_12_ADDR

/* GYR_Z (MSB) Description - Reg Addr --> 0x11, Bit --> 0...7 */
#define BMI160_USER_DATA_13_GYRO_Z_MSB__POS           0
#define BMI160_USER_DATA_13_GYRO_Z_MSB__LEN           8
#define BMI160_USER_DATA_13_GYRO_Z_MSB__MSK          0xFF
#define BMI160_USER_DATA_13_GYRO_Z_MSB__REG          BMI160_USER_DATA_13_ADDR

/* ACC_X (LSB) Description - Reg Addr --> 0x12, Bit --> 0...7 */
#define BMI160_USER_DATA_14_ACCEL_X_LSB__POS           0
#define BMI160_USER_DATA_14_ACCEL_X_LSB__LEN           8
#define BMI160_USER_DATA_14_ACCEL_X_LSB__MSK          0xFF
#define BMI160_USER_DATA_14_ACCEL_X_LSB__REG          BMI160_USER_DATA_14_ADDR

/* ACC_X (MSB) Description - Reg Addr --> 0x13, Bit --> 0...7 */
#define BMI160_USER_DATA_15_ACCEL_X_MSB__POS           0
#define BMI160_USER_DATA_15_ACCEL_X_MSB__LEN           8
#define BMI160_USER_DATA_15_ACCEL_X_MSB__MSK          0xFF
#define BMI160_USER_DATA_15_ACCEL_X_MSB__REG          BMI160_USER_DATA_15_ADDR

/* ACC_Y (LSB) Description - Reg Addr --> 0x14, Bit --> 0...7 */
#define BMI160_USER_DATA_16_ACCEL_Y_LSB__POS           0
#define BMI160_USER_DATA_16_ACCEL_Y_LSB__LEN           8
#define BMI160_USER_DATA_16_ACCEL_Y_LSB__MSK          0xFF
#define BMI160_USER_DATA_16_ACCEL_Y_LSB__REG          BMI160_USER_DATA_16_ADDR

/* ACC_Y (MSB) Description - Reg Addr --> 0x15, Bit --> 0...7 */
#define BMI160_USER_DATA_17_ACCEL_Y_MSB__POS           0
#define BMI160_USER_DATA_17_ACCEL_Y_MSB__LEN           8
#define BMI160_USER_DATA_17_ACCEL_Y_MSB__MSK          0xFF
#define BMI160_USER_DATA_17_ACCEL_Y_MSB__REG          BMI160_USER_DATA_17_ADDR

/* ACC_Z (LSB) Description - Reg Addr --> 0x16, Bit --> 0...7 */
#define BMI160_USER_DATA_18_ACCEL_Z_LSB__POS           0
#define BMI160_USER_DATA_18_ACCEL_Z_LSB__LEN           8
#define BMI160_USER_DATA_18_ACCEL_Z_LSB__MSK          0xFF
#define BMI160_USER_DATA_18_ACCEL_Z_LSB__REG          BMI160_USER_DATA_18_ADDR

/* ACC_Z (MSB) Description - Reg Addr --> 0x17, Bit --> 0...7 */
#define BMI160_USER_DATA_19_ACCEL_Z_MSB__POS           0
#define BMI160_USER_DATA_19_ACCEL_Z_MSB__LEN           8
#define BMI160_USER_DATA_19_ACCEL_Z_MSB__MSK          0xFF
#define BMI160_USER_DATA_19_ACCEL_Z_MSB__REG          BMI160_USER_DATA_19_ADDR

/* SENSORTIME_0 (LSB) Description - Reg Addr --> 0x18, Bit --> 0...7 */
#define BMI160_USER_SENSORTIME_0_SENSOR_TIME_LSB__POS           0
#define BMI160_USER_SENSORTIME_0_SENSOR_TIME_LSB__LEN           8
#define BMI160_USER_SENSORTIME_0_SENSOR_TIME_LSB__MSK          0xFF
#define BMI160_USER_SENSORTIME_0_SENSOR_TIME_LSB__REG          \
		BMI160_USER_SENSORTIME_0_ADDR

/* SENSORTIME_1 (MSB) Description - Reg Addr --> 0x19, Bit --> 0...7 */
#define BMI160_USER_SENSORTIME_1_SENSOR_TIME_MSB__POS           0
#define BMI160_USER_SENSORTIME_1_SENSOR_TIME_MSB__LEN           8
#define BMI160_USER_SENSORTIME_1_SENSOR_TIME_MSB__MSK          0xFF
#define BMI160_USER_SENSORTIME_1_SENSOR_TIME_MSB__REG          \
		BMI160_USER_SENSORTIME_1_ADDR

/* SENSORTIME_2 (MSB) Description - Reg Addr --> 0x1A, Bit --> 0...7 */
#define BMI160_USER_SENSORTIME_2_SENSOR_TIME_MSB__POS           0
#define BMI160_USER_SENSORTIME_2_SENSOR_TIME_MSB__LEN           8
#define BMI160_USER_SENSORTIME_2_SENSOR_TIME_MSB__MSK          0xFF
#define BMI160_USER_SENSORTIME_2_SENSOR_TIME_MSB__REG          \
		BMI160_USER_SENSORTIME_2_ADDR

/* Status Description - Reg Addr --> 0x1B, Bit --> 1 */
#define BMI160_USER_STAT_GYRO_SELFTEST_OK__POS          1
#define BMI160_USER_STAT_GYRO_SELFTEST_OK__LEN          1
#define BMI160_USER_STAT_GYRO_SELFTEST_OK__MSK          0x02
#define BMI160_USER_STAT_GYRO_SELFTEST_OK__REG         \
		BMI160_USER_STAT_ADDR

/* Status Description - Reg Addr --> 0x1B, Bit --> 2 */
#define BMI160_USER_STAT_MAG_MANUAL_OPERATION__POS          2
#define BMI160_USER_STAT_MAG_MANUAL_OPERATION__LEN          1
#define BMI160_USER_STAT_MAG_MANUAL_OPERATION__MSK          0x04
#define BMI160_USER_STAT_MAG_MANUAL_OPERATION__REG          \
		BMI160_USER_STAT_ADDR

/* Status Description - Reg Addr --> 0x1B, Bit --> 3 */
#define BMI160_USER_STAT_FOC_RDY__POS          3
#define BMI160_USER_STAT_FOC_RDY__LEN          1
#define BMI160_USER_STAT_FOC_RDY__MSK          0x08
#define BMI160_USER_STAT_FOC_RDY__REG          BMI160_USER_STAT_ADDR

/* Status Description - Reg Addr --> 0x1B, Bit --> 4 */
#define BMI160_USER_STAT_NVM_RDY__POS           4
#define BMI160_USER_STAT_NVM_RDY__LEN           1
#define BMI160_USER_STAT_NVM_RDY__MSK           0x10
#define BMI160_USER_STAT_NVM_RDY__REG           BMI160_USER_STAT_ADDR

/* Status Description - Reg Addr --> 0x1B, Bit --> 5 */
#define BMI160_USER_STAT_DATA_RDY_MAG__POS           5
#define BMI160_USER_STAT_DATA_RDY_MAG__LEN           1
#define BMI160_USER_STAT_DATA_RDY_MAG__MSK           0x20
#define BMI160_USER_STAT_DATA_RDY_MAG__REG           BMI160_USER_STAT_ADDR

/* Status Description - Reg Addr --> 0x1B, Bit --> 6 */
#define BMI160_USER_STAT_DATA_RDY_GYRO__POS           6
#define BMI160_USER_STAT_DATA_RDY_GYRO__LEN           1
#define BMI160_USER_STAT_DATA_RDY_GYRO__MSK           0x40
#define BMI160_USER_STAT_DATA_RDY_GYRO__REG           BMI160_USER_STAT_ADDR

/* Status Description - Reg Addr --> 0x1B, Bit --> 7 */
#define BMI160_USER_STAT_DATA_RDY_ACCEL__POS           7
#define BMI160_USER_STAT_DATA_RDY_ACCEL__LEN           1
#define BMI160_USER_STAT_DATA_RDY_ACCEL__MSK           0x80
#define BMI160_USER_STAT_DATA_RDY_ACCEL__REG           BMI160_USER_STAT_ADDR

/* Int_Status_0 Description - Reg Addr --> 0x1C, Bit --> 0 */
#define BMI160_USER_INTR_STAT_0_STEP_INTR__POS           0
#define BMI160_USER_INTR_STAT_0_STEP_INTR__LEN           1
#define BMI160_USER_INTR_STAT_0_STEP_INTR__MSK          0x01
#define BMI160_USER_INTR_STAT_0_STEP_INTR__REG          \
		BMI160_USER_INTR_STAT_0_ADDR

/* Int_Status_0 Description - Reg Addr --> 0x1C, Bit --> 1 */
#define BMI160_USER_INTR_STAT_0_SIGNIFICANT_INTR__POS		1
#define BMI160_USER_INTR_STAT_0_SIGNIFICANT_INTR__LEN		1
#define BMI160_USER_INTR_STAT_0_SIGNIFICANT_INTR__MSK		0x02
#define BMI160_USER_INTR_STAT_0_SIGNIFICANT_INTR__REG       \
		BMI160_USER_INTR_STAT_0_ADDR

/* Int_Status_0 Description - Reg Addr --> 0x1C, Bit --> 2 */
#define BMI160_USER_INTR_STAT_0_ANY_MOTION__POS           2
#define BMI160_USER_INTR_STAT_0_ANY_MOTION__LEN           1
#define BMI160_USER_INTR_STAT_0_ANY_MOTION__MSK          0x04
#define BMI160_USER_INTR_STAT_0_ANY_MOTION__REG          \
		BMI160_USER_INTR_STAT_0_ADDR

/* Int_Status_0 Description - Reg Addr --> 0x1C, Bit --> 3 */
#define BMI160_USER_INTR_STAT_0_PMU_TRIGGER__POS           3
#define BMI160_USER_INTR_STAT_0_PMU_TRIGGER__LEN           1
#define BMI160_USER_INTR_STAT_0_PMU_TRIGGER__MSK          0x08
#define BMI160_USER_INTR_STAT_0_PMU_TRIGGER__REG          \
		BMI160_USER_INTR_STAT_0_ADDR

/* Int_Status_0 Description - Reg Addr --> 0x1C, Bit --> 4 */
#define BMI160_USER_INTR_STAT_0_DOUBLE_TAP_INTR__POS           4
#define BMI160_USER_INTR_STAT_0_DOUBLE_TAP_INTR__LEN           1
#define BMI160_USER_INTR_STAT_0_DOUBLE_TAP_INTR__MSK          0x10
#define BMI160_USER_INTR_STAT_0_DOUBLE_TAP_INTR__REG          \
		BMI160_USER_INTR_STAT_0_ADDR

/* Int_Status_0 Description - Reg Addr --> 0x1C, Bit --> 5 */
#define BMI160_USER_INTR_STAT_0_SINGLE_TAP_INTR__POS           5
#define BMI160_USER_INTR_STAT_0_SINGLE_TAP_INTR__LEN           1
#define BMI160_USER_INTR_STAT_0_SINGLE_TAP_INTR__MSK          0x20
#define BMI160_USER_INTR_STAT_0_SINGLE_TAP_INTR__REG          \
		BMI160_USER_INTR_STAT_0_ADDR

/* Int_Status_0 Description - Reg Addr --> 0x1C, Bit --> 6 */
#define BMI160_USER_INTR_STAT_0_ORIENT__POS           6
#define BMI160_USER_INTR_STAT_0_ORIENT__LEN           1
#define BMI160_USER_INTR_STAT_0_ORIENT__MSK          0x40
#define BMI160_USER_INTR_STAT_0_ORIENT__REG          \
		BMI160_USER_INTR_STAT_0_ADDR

/* Int_Status_0 Description - Reg Addr --> 0x1C, Bit --> 7 */
#define BMI160_USER_INTR_STAT_0_FLAT__POS           7
#define BMI160_USER_INTR_STAT_0_FLAT__LEN           1
#define BMI160_USER_INTR_STAT_0_FLAT__MSK          0x80
#define BMI160_USER_INTR_STAT_0_FLAT__REG          \
		BMI160_USER_INTR_STAT_0_ADDR

/* Int_Status_1 Description - Reg Addr --> 0x1D, Bit --> 2 */
#define BMI160_USER_INTR_STAT_1_HIGH_G_INTR__POS               2
#define BMI160_USER_INTR_STAT_1_HIGH_G_INTR__LEN               1
#define BMI160_USER_INTR_STAT_1_HIGH_G_INTR__MSK              0x04
#define BMI160_USER_INTR_STAT_1_HIGH_G_INTR__REG              \
		BMI160_USER_INTR_STAT_1_ADDR

/* Int_Status_1 Description - Reg Addr --> 0x1D, Bit --> 3 */
#define BMI160_USER_INTR_STAT_1_LOW_G_INTR__POS               3
#define BMI160_USER_INTR_STAT_1_LOW_G_INTR__LEN               1
#define BMI160_USER_INTR_STAT_1_LOW_G_INTR__MSK              0x08
#define BMI160_USER_INTR_STAT_1_LOW_G_INTR__REG              \
		BMI160_USER_INTR_STAT_1_ADDR

/* Int_Status_1 Description - Reg Addr --> 0x1D, Bit --> 4 */
#define BMI160_USER_INTR_STAT_1_DATA_RDY_INTR__POS               4
#define BMI160_USER_INTR_STAT_1_DATA_RDY_INTR__LEN               1
#define BMI160_USER_INTR_STAT_1_DATA_RDY_INTR__MSK               0x10
#define BMI160_USER_INTR_STAT_1_DATA_RDY_INTR__REG               \
		BMI160_USER_INTR_STAT_1_ADDR

/* Int_Status_1 Description - Reg Addr --> 0x1D, Bit --> 5 */
#define BMI160_USER_INTR_STAT_1_FIFO_FULL_INTR__POS               5
#define BMI160_USER_INTR_STAT_1_FIFO_FULL_INTR__LEN               1
#define BMI160_USER_INTR_STAT_1_FIFO_FULL_INTR__MSK               0x20
#define BMI160_USER_INTR_STAT_1_FIFO_FULL_INTR__REG               \
		BMI160_USER_INTR_STAT_1_ADDR

/* Int_Status_1 Description - Reg Addr --> 0x1D, Bit --> 6 */
#define BMI160_USER_INTR_STAT_1_FIFO_WM_INTR__POS               6
#define BMI160_USER_INTR_STAT_1_FIFO_WM_INTR__LEN               1
#define BMI160_USER_INTR_STAT_1_FIFO_WM_INTR__MSK               0x40
#define BMI160_USER_INTR_STAT_1_FIFO_WM_INTR__REG               \
		BMI160_USER_INTR_STAT_1_ADDR

/* Int_Status_1 Description - Reg Addr --> 0x1D, Bit --> 7 */
#define BMI160_USER_INTR_STAT_1_NOMOTION_INTR__POS               7
#define BMI160_USER_INTR_STAT_1_NOMOTION_INTR__LEN               1
#define BMI160_USER_INTR_STAT_1_NOMOTION_INTR__MSK               0x80
#define BMI160_USER_INTR_STAT_1_NOMOTION_INTR__REG               \
		BMI160_USER_INTR_STAT_1_ADDR

/* Int_Status_2 Description - Reg Addr --> 0x1E, Bit --> 0 */
#define BMI160_USER_INTR_STAT_2_ANY_MOTION_FIRST_X__POS               0
#define BMI160_USER_INTR_STAT_2_ANY_MOTION_FIRST_X__LEN               1
#define BMI160_USER_INTR_STAT_2_ANY_MOTION_FIRST_X__MSK               0x01
#define BMI160_USER_INTR_STAT_2_ANY_MOTION_FIRST_X__REG               \
		BMI160_USER_INTR_STAT_2_ADDR

/* Int_Status_2 Description - Reg Addr --> 0x1E, Bit --> 1 */
#define BMI160_USER_INTR_STAT_2_ANY_MOTION_FIRST_Y__POS               1
#define BMI160_USER_INTR_STAT_2_ANY_MOTION_FIRST_Y__LEN               1
#define BMI160_USER_INTR_STAT_2_ANY_MOTION_FIRST_Y__MSK               0x02
#define BMI160_USER_INTR_STAT_2_ANY_MOTION_FIRST_Y__REG               \
		BMI160_USER_INTR_STAT_2_ADDR

/* Int_Status_2 Description - Reg Addr --> 0x1E, Bit --> 2 */
#define BMI160_USER_INTR_STAT_2_ANY_MOTION_FIRST_Z__POS               2
#define BMI160_USER_INTR_STAT_2_ANY_MOTION_FIRST_Z__LEN               1
#define BMI160_USER_INTR_STAT_2_ANY_MOTION_FIRST_Z__MSK               0x04
#define BMI160_USER_INTR_STAT_2_ANY_MOTION_FIRST_Z__REG               \
		BMI160_USER_INTR_STAT_2_ADDR

/* Int_Status_2 Description - Reg Addr --> 0x1E, Bit --> 3 */
#define BMI160_USER_INTR_STAT_2_ANY_MOTION_SIGN__POS               3
#define BMI160_USER_INTR_STAT_2_ANY_MOTION_SIGN__LEN               1
#define BMI160_USER_INTR_STAT_2_ANY_MOTION_SIGN__MSK               0x08
#define BMI160_USER_INTR_STAT_2_ANY_MOTION_SIGN__REG               \
		BMI160_USER_INTR_STAT_2_ADDR

/* Int_Status_2 Description - Reg Addr --> 0x1E, Bit --> 4 */
#define BMI160_USER_INTR_STAT_2_TAP_FIRST_X__POS               4
#define BMI160_USER_INTR_STAT_2_TAP_FIRST_X__LEN               1
#define BMI160_USER_INTR_STAT_2_TAP_FIRST_X__MSK               0x10
#define BMI160_USER_INTR_STAT_2_TAP_FIRST_X__REG               \
		BMI160_USER_INTR_STAT_2_ADDR

/* Int_Status_2 Description - Reg Addr --> 0x1E, Bit --> 5 */
#define BMI160_USER_INTR_STAT_2_TAP_FIRST_Y__POS               5
#define BMI160_USER_INTR_STAT_2_TAP_FIRST_Y__LEN               1
#define BMI160_USER_INTR_STAT_2_TAP_FIRST_Y__MSK               0x20
#define BMI160_USER_INTR_STAT_2_TAP_FIRST_Y__REG               \
		BMI160_USER_INTR_STAT_2_ADDR

/* Int_Status_2 Description - Reg Addr --> 0x1E, Bit --> 6 */
#define BMI160_USER_INTR_STAT_2_TAP_FIRST_Z__POS               6
#define BMI160_USER_INTR_STAT_2_TAP_FIRST_Z__LEN               1
#define BMI160_USER_INTR_STAT_2_TAP_FIRST_Z__MSK               0x40
#define BMI160_USER_INTR_STAT_2_TAP_FIRST_Z__REG               \
		BMI160_USER_INTR_STAT_2_ADDR

/* Int_Status_2 Description - Reg Addr --> 0x1E, Bit --> 7 */
#define BMI160_USER_INTR_STAT_2_TAP_SIGN__POS               7
#define BMI160_USER_INTR_STAT_2_TAP_SIGN__LEN               1
#define BMI160_USER_INTR_STAT_2_TAP_SIGN__MSK               0x80
#define BMI160_USER_INTR_STAT_2_TAP_SIGN__REG               \
		BMI160_USER_INTR_STAT_2_ADDR

/* Int_Status_2 Description - Reg Addr --> 0x1E, Bit --> 0...7 */
#define BMI160_USER_INTR_STAT_2__POS               0
#define BMI160_USER_INTR_STAT_2__LEN               8
#define BMI160_USER_INTR_STAT_2__MSK               0xFF
#define BMI160_USER_INTR_STAT_2__REG               \
		BMI160_USER_INTR_STAT_2_ADDR

/* Int_Status_3 Description - Reg Addr --> 0x1F, Bit --> 0 */
#define BMI160_USER_INTR_STAT_3_HIGH_G_FIRST_X__POS               0
#define BMI160_USER_INTR_STAT_3_HIGH_G_FIRST_X__LEN               1
#define BMI160_USER_INTR_STAT_3_HIGH_G_FIRST_X__MSK               0x01
#define BMI160_USER_INTR_STAT_3_HIGH_G_FIRST_X__REG               \
		BMI160_USER_INTR_STAT_3_ADDR

/* Int_Status_3 Description - Reg Addr --> 0x1E, Bit --> 1 */
#define BMI160_USER_INTR_STAT_3_HIGH_G_FIRST_Y__POS               1
#define BMI160_USER_INTR_STAT_3_HIGH_G_FIRST_Y__LEN               1
#define BMI160_USER_INTR_STAT_3_HIGH_G_FIRST_Y__MSK               0x02
#define BMI160_USER_INTR_STAT_3_HIGH_G_FIRST_Y__REG               \
		BMI160_USER_INTR_STAT_3_ADDR

/* Int_Status_3 Description - Reg Addr --> 0x1F, Bit --> 2 */
#define BMI160_USER_INTR_STAT_3_HIGH_G_FIRST_Z__POS               2
#define BMI160_USER_INTR_STAT_3_HIGH_G_FIRST_Z__LEN               1
#define BMI160_USER_INTR_STAT_3_HIGH_G_FIRST_Z__MSK               0x04
#define BMI160_USER_INTR_STAT_3_HIGH_G_FIRST_Z__REG               \
		BMI160_USER_INTR_STAT_3_ADDR

/* Int_Status_3 Description - Reg Addr --> 0x1F, Bit --> 3 */
#define BMI160_USER_INTR_STAT_3_HIGH_G_SIGN__POS               3
#define BMI160_USER_INTR_STAT_3_HIGH_G_SIGN__LEN               1
#define BMI160_USER_INTR_STAT_3_HIGH_G_SIGN__MSK               0x08
#define BMI160_USER_INTR_STAT_3_HIGH_G_SIGN__REG               \
		BMI160_USER_INTR_STAT_3_ADDR

/* Int_Status_3 Description - Reg Addr --> 0x1F, Bit --> 4...5 */
#define BMI160_USER_INTR_STAT_3_ORIENT_XY__POS               4
#define BMI160_USER_INTR_STAT_3_ORIENT_XY__LEN               2
#define BMI160_USER_INTR_STAT_3_ORIENT_XY__MSK               0x30
#define BMI160_USER_INTR_STAT_3_ORIENT_XY__REG               \
		BMI160_USER_INTR_STAT_3_ADDR

/* Int_Status_3 Description - Reg Addr --> 0x1F, Bit --> 6 */
#define BMI160_USER_INTR_STAT_3_ORIENT_Z__POS               6
#define BMI160_USER_INTR_STAT_3_ORIENT_Z__LEN               1
#define BMI160_USER_INTR_STAT_3_ORIENT_Z__MSK               0x40
#define BMI160_USER_INTR_STAT_3_ORIENT_Z__REG               \
		BMI160_USER_INTR_STAT_3_ADDR

/* Int_Status_3 Description - Reg Addr --> 0x1F, Bit --> 7 */
#define BMI160_USER_INTR_STAT_3_FLAT__POS               7
#define BMI160_USER_INTR_STAT_3_FLAT__LEN               1
#define BMI160_USER_INTR_STAT_3_FLAT__MSK               0x80
#define BMI160_USER_INTR_STAT_3_FLAT__REG               \
		BMI160_USER_INTR_STAT_3_ADDR

/* Int_Status_3 Description - Reg Addr --> 0x1F, Bit --> 0...7 */
#define BMI160_USER_INTR_STAT_3__POS               0
#define BMI160_USER_INTR_STAT_3__LEN               8
#define BMI160_USER_INTR_STAT_3__MSK               0xFF
#define BMI160_USER_INTR_STAT_3__REG               \
		BMI160_USER_INTR_STAT_3_ADDR

/* Temperature Description - LSB Reg Addr --> 0x20, Bit --> 0...7 */
#define BMI160_USER_TEMP_LSB_VALUE__POS               0
#define BMI160_USER_TEMP_LSB_VALUE__LEN               8
#define BMI160_USER_TEMP_LSB_VALUE__MSK               0xFF
#define BMI160_USER_TEMP_LSB_VALUE__REG               \
		BMI160_USER_TEMPERATURE_0_ADDR

/* Temperature Description - LSB Reg Addr --> 0x21, Bit --> 0...7 */
#define BMI160_USER_TEMP_MSB_VALUE__POS               0
#define BMI160_USER_TEMP_MSB_VALUE__LEN               8
#define BMI160_USER_TEMP_MSB_VALUE__MSK               0xFF
#define BMI160_USER_TEMP_MSB_VALUE__REG               \
		BMI160_USER_TEMPERATURE_1_ADDR

/* Fifo_Length0 Description - Reg Addr --> 0x22, Bit --> 0...7 */
#define BMI160_USER_FIFO_BYTE_COUNTER_LSB__POS           0
#define BMI160_USER_FIFO_BYTE_COUNTER_LSB__LEN           8
#define BMI160_USER_FIFO_BYTE_COUNTER_LSB__MSK          0xFF
#define BMI160_USER_FIFO_BYTE_COUNTER_LSB__REG          \
		BMI160_USER_FIFO_LENGTH_0_ADDR

/*Fifo_Length1 Description - Reg Addr --> 0x23, Bit --> 0...2 */
#define BMI160_USER_FIFO_BYTE_COUNTER_MSB__POS           0
#define BMI160_USER_FIFO_BYTE_COUNTER_MSB__LEN           3
#define BMI160_USER_FIFO_BYTE_COUNTER_MSB__MSK          0x07
#define BMI160_USER_FIFO_BYTE_COUNTER_MSB__REG          \
		BMI160_USER_FIFO_LENGTH_1_ADDR


/* Fifo_Data Description - Reg Addr --> 0x24, Bit --> 0...7 */
#define BMI160_USER_FIFO_DATA__POS           0
#define BMI160_USER_FIFO_DATA__LEN           8
#define BMI160_USER_FIFO_DATA__MSK          0xFF
#define BMI160_USER_FIFO_DATA__REG          BMI160_USER_FIFO_DATA_ADDR


/* Acc_Conf Description - Reg Addr --> 0x40, Bit --> 0...3 */
#define BMI160_USER_ACCEL_CONFIG_OUTPUT_DATA_RATE__POS               0
#define BMI160_USER_ACCEL_CONFIG_OUTPUT_DATA_RATE__LEN               4
#define BMI160_USER_ACCEL_CONFIG_OUTPUT_DATA_RATE__MSK               0x0F
#define BMI160_USER_ACCEL_CONFIG_OUTPUT_DATA_RATE__REG		       \
BMI160_USER_ACCEL_CONFIG_ADDR

/* Acc_Conf Description - Reg Addr --> 0x40, Bit --> 4...6 */
#define BMI160_USER_ACCEL_CONFIG_ACCEL_BW__POS               4
#define BMI160_USER_ACCEL_CONFIG_ACCEL_BW__LEN               3
#define BMI160_USER_ACCEL_CONFIG_ACCEL_BW__MSK               0x70
#define BMI160_USER_ACCEL_CONFIG_ACCEL_BW__REG	BMI160_USER_ACCEL_CONFIG_ADDR

/* Acc_Conf Description - Reg Addr --> 0x40, Bit --> 7 */
#define BMI160_USER_ACCEL_CONFIG_ACCEL_UNDER_SAMPLING__POS           7
#define BMI160_USER_ACCEL_CONFIG_ACCEL_UNDER_SAMPLING__LEN           1
#define BMI160_USER_ACCEL_CONFIG_ACCEL_UNDER_SAMPLING__MSK           0x80
#define BMI160_USER_ACCEL_CONFIG_ACCEL_UNDER_SAMPLING__REG	\
BMI160_USER_ACCEL_CONFIG_ADDR

/* Acc_Range Description - Reg Addr --> 0x41, Bit --> 0...3 */
#define BMI160_USER_ACCEL_RANGE__POS               0
#define BMI160_USER_ACCEL_RANGE__LEN               4
#define BMI160_USER_ACCEL_RANGE__MSK               0x0F
#define BMI160_USER_ACCEL_RANGE__REG               BMI160_USER_ACCEL_RANGE_ADDR

/* Gyro_Conf Description - Reg Addr --> 0x42, Bit --> 0...3 */
#define BMI160_USER_GYRO_CONFIG_OUTPUT_DATA_RATE__POS               0
#define BMI160_USER_GYRO_CONFIG_OUTPUT_DATA_RATE__LEN               4
#define BMI160_USER_GYRO_CONFIG_OUTPUT_DATA_RATE__MSK               0x0F
#define BMI160_USER_GYRO_CONFIG_OUTPUT_DATA_RATE__REG               \
BMI160_USER_GYRO_CONFIG_ADDR

/* Gyro_Conf Description - Reg Addr --> 0x42, Bit --> 4...5 */
#define BMI160_USER_GYRO_CONFIG_BW__POS               4
#define BMI160_USER_GYRO_CONFIG_BW__LEN               2
#define BMI160_USER_GYRO_CONFIG_BW__MSK               0x30
#define BMI160_USER_GYRO_CONFIG_BW__REG               \
BMI160_USER_GYRO_CONFIG_ADDR

/* Gyr_Range Description - Reg Addr --> 0x43, Bit --> 0...2 */
#define BMI160_USER_GYRO_RANGE__POS               0
#define BMI160_USER_GYRO_RANGE__LEN               3
#define BMI160_USER_GYRO_RANGE__MSK               0x07
#define BMI160_USER_GYRO_RANGE__REG               BMI160_USER_GYRO_RANGE_ADDR

/* Mag_Conf Description - Reg Addr --> 0x44, Bit --> 0...3 */
#define BMI160_USER_MAG_CONFIG_OUTPUT_DATA_RATE__POS               0
#define BMI160_USER_MAG_CONFIG_OUTPUT_DATA_RATE__LEN               4
#define BMI160_USER_MAG_CONFIG_OUTPUT_DATA_RATE__MSK               0x0F
#define BMI160_USER_MAG_CONFIG_OUTPUT_DATA_RATE__REG               \
BMI160_USER_MAG_CONFIG_ADDR


/* Fifo_Downs Description - Reg Addr --> 0x45, Bit --> 0...2 */
#define BMI160_USER_FIFO_DOWN_GYRO__POS               0
#define BMI160_USER_FIFO_DOWN_GYRO__LEN               3
#define BMI160_USER_FIFO_DOWN_GYRO__MSK               0x07
#define BMI160_USER_FIFO_DOWN_GYRO__REG	BMI160_USER_FIFO_DOWN_ADDR

/* Fifo_filt Description - Reg Addr --> 0x45, Bit --> 3 */
#define BMI160_USER_FIFO_FILTER_GYRO__POS               3
#define BMI160_USER_FIFO_FILTER_GYRO__LEN               1
#define BMI160_USER_FIFO_FILTER_GYRO__MSK               0x08
#define BMI160_USER_FIFO_FILTER_GYRO__REG	  BMI160_USER_FIFO_DOWN_ADDR

/* Fifo_Downs Description - Reg Addr --> 0x45, Bit --> 4...6 */
#define BMI160_USER_FIFO_DOWN_ACCEL__POS               4
#define BMI160_USER_FIFO_DOWN_ACCEL__LEN               3
#define BMI160_USER_FIFO_DOWN_ACCEL__MSK               0x70
#define BMI160_USER_FIFO_DOWN_ACCEL__REG	BMI160_USER_FIFO_DOWN_ADDR

/* Fifo_FILT Description - Reg Addr --> 0x45, Bit --> 7 */
#define BMI160_USER_FIFO_FILTER_ACCEL__POS               7
#define BMI160_USER_FIFO_FILTER_ACCEL__LEN               1
#define BMI160_USER_FIFO_FILTER_ACCEL__MSK               0x80
#define BMI160_USER_FIFO_FILTER_ACCEL__REG	BMI160_USER_FIFO_DOWN_ADDR

/* Fifo_Config_0 Description - Reg Addr --> 0x46, Bit --> 0...7 */
#define BMI160_USER_FIFO_WM__POS               0
#define BMI160_USER_FIFO_WM__LEN               8
#define BMI160_USER_FIFO_WM__MSK               0xFF
#define BMI160_USER_FIFO_WM__REG	BMI160_USER_FIFO_CONFIG_0_ADDR

/* Fifo_Config_1 Description - Reg Addr --> 0x47, Bit --> 0 */
#define BMI160_USER_FIFO_STOP_ON_FULL__POS		0
#define BMI160_USER_FIFO_STOP_ON_FULL__LEN		1
#define BMI160_USER_FIFO_STOP_ON_FULL__MSK		0x01
#define BMI160_USER_FIFO_STOP_ON_FULL__REG	BMI160_USER_FIFO_CONFIG_1_ADDR

/* Fifo_Config_1 Description - Reg Addr --> 0x47, Bit --> 1 */
#define BMI160_USER_FIFO_TIME_ENABLE__POS               1
#define BMI160_USER_FIFO_TIME_ENABLE__LEN               1
#define BMI160_USER_FIFO_TIME_ENABLE__MSK               0x02
#define BMI160_USER_FIFO_TIME_ENABLE__REG	BMI160_USER_FIFO_CONFIG_1_ADDR

/* Fifo_Config_1 Description - Reg Addr --> 0x47, Bit --> 2 */
#define BMI160_USER_FIFO_TAG_INTR2_ENABLE__POS               2
#define BMI160_USER_FIFO_TAG_INTR2_ENABLE__LEN               1
#define BMI160_USER_FIFO_TAG_INTR2_ENABLE__MSK               0x04
#define BMI160_USER_FIFO_TAG_INTR2_ENABLE__REG	BMI160_USER_FIFO_CONFIG_1_ADDR

/* Fifo_Config_1 Description - Reg Addr --> 0x47, Bit --> 3 */
#define BMI160_USER_FIFO_TAG_INTR1_ENABLE__POS               3
#define BMI160_USER_FIFO_TAG_INTR1_ENABLE__LEN               1
#define BMI160_USER_FIFO_TAG_INTR1_ENABLE__MSK               0x08
#define BMI160_USER_FIFO_TAG_INTR1_ENABLE__REG	BMI160_USER_FIFO_CONFIG_1_ADDR

/* Fifo_Config_1 Description - Reg Addr --> 0x47, Bit --> 4 */
#define BMI160_USER_FIFO_HEADER_ENABLE__POS               4
#define BMI160_USER_FIFO_HEADER_ENABLE__LEN               1
#define BMI160_USER_FIFO_HEADER_ENABLE__MSK               0x10
#define BMI160_USER_FIFO_HEADER_ENABLE__REG		         \
BMI160_USER_FIFO_CONFIG_1_ADDR

/* Fifo_Config_1 Description - Reg Addr --> 0x47, Bit --> 5 */
#define BMI160_USER_FIFO_MAG_ENABLE__POS               5
#define BMI160_USER_FIFO_MAG_ENABLE__LEN               1
#define BMI160_USER_FIFO_MAG_ENABLE__MSK               0x20
#define BMI160_USER_FIFO_MAG_ENABLE__REG		     \
BMI160_USER_FIFO_CONFIG_1_ADDR

/* Fifo_Config_1 Description - Reg Addr --> 0x47, Bit --> 6 */
#define BMI160_USER_FIFO_ACCEL_ENABLE__POS               6
#define BMI160_USER_FIFO_ACCEL_ENABLE__LEN               1
#define BMI160_USER_FIFO_ACCEL_ENABLE__MSK               0x40
#define BMI160_USER_FIFO_ACCEL_ENABLE__REG		        \
BMI160_USER_FIFO_CONFIG_1_ADDR

/* Fifo_Config_1 Description - Reg Addr --> 0x47, Bit --> 7 */
#define BMI160_USER_FIFO_GYRO_ENABLE__POS               7
#define BMI160_USER_FIFO_GYRO_ENABLE__LEN               1
#define BMI160_USER_FIFO_GYRO_ENABLE__MSK               0x80
#define BMI160_USER_FIFO_GYRO_ENABLE__REG		       \
BMI160_USER_FIFO_CONFIG_1_ADDR



/* Mag_IF_0 Description - Reg Addr --> 0x4b, Bit --> 1...7 */
#define BMI160_USER_I2C_DEVICE_ADDR__POS               1
#define BMI160_USER_I2C_DEVICE_ADDR__LEN               7
#define BMI160_USER_I2C_DEVICE_ADDR__MSK               0xFE
#define BMI160_USER_I2C_DEVICE_ADDR__REG	BMI160_USER_MAG_IF_0_ADDR

/* Mag_IF_1 Description - Reg Addr --> 0x4c, Bit --> 0...1 */
#define BMI160_USER_MAG_BURST__POS               0
#define BMI160_USER_MAG_BURST__LEN               2
#define BMI160_USER_MAG_BURST__MSK               0x03
#define BMI160_USER_MAG_BURST__REG               BMI160_USER_MAG_IF_1_ADDR

/* Mag_IF_1 Description - Reg Addr --> 0x4c, Bit --> 2...5 */
#define BMI160_USER_MAG_OFFSET__POS               2
#define BMI160_USER_MAG_OFFSET__LEN               4
#define BMI160_USER_MAG_OFFSET__MSK               0x3C
#define BMI160_USER_MAG_OFFSET__REG               BMI160_USER_MAG_IF_1_ADDR

/* Mag_IF_1 Description - Reg Addr --> 0x4c, Bit --> 7 */
#define BMI160_USER_MAG_MANUAL_ENABLE__POS               7
#define BMI160_USER_MAG_MANUAL_ENABLE__LEN               1
#define BMI160_USER_MAG_MANUAL_ENABLE__MSK               0x80
#define BMI160_USER_MAG_MANUAL_ENABLE__REG               \
BMI160_USER_MAG_IF_1_ADDR

/* Mag_IF_2 Description - Reg Addr --> 0x4d, Bit -->0... 7 */
#define BMI160_USER_READ_ADDR__POS               0
#define BMI160_USER_READ_ADDR__LEN               8
#define BMI160_USER_READ_ADDR__MSK               0xFF
#define BMI160_USER_READ_ADDR__REG               BMI160_USER_MAG_IF_2_ADDR

/* Mag_IF_3 Description - Reg Addr --> 0x4e, Bit -->0... 7 */
#define BMI160_USER_WRITE_ADDR__POS               0
#define BMI160_USER_WRITE_ADDR__LEN               8
#define BMI160_USER_WRITE_ADDR__MSK               0xFF
#define BMI160_USER_WRITE_ADDR__REG               BMI160_USER_MAG_IF_3_ADDR

/* Mag_IF_4 Description - Reg Addr --> 0x4f, Bit -->0... 7 */
#define BMI160_USER_WRITE_DATA__POS               0
#define BMI160_USER_WRITE_DATA__LEN               8
#define BMI160_USER_WRITE_DATA__MSK               0xFF
#define BMI160_USER_WRITE_DATA__REG               BMI160_USER_MAG_IF_4_ADDR

/* Int_En_0 Description - Reg Addr --> 0x50, Bit -->0 */
#define BMI160_USER_INTR_ENABLE_0_ANY_MOTION_X_ENABLE__POS               0
#define BMI160_USER_INTR_ENABLE_0_ANY_MOTION_X_ENABLE__LEN               1
#define BMI160_USER_INTR_ENABLE_0_ANY_MOTION_X_ENABLE__MSK               0x01
#define BMI160_USER_INTR_ENABLE_0_ANY_MOTION_X_ENABLE__REG	              \
BMI160_USER_INTR_ENABLE_0_ADDR

/* Int_En_0 Description - Reg Addr --> 0x50, Bit -->1 */
#define BMI160_USER_INTR_ENABLE_0_ANY_MOTION_Y_ENABLE__POS               1
#define BMI160_USER_INTR_ENABLE_0_ANY_MOTION_Y_ENABLE__LEN               1
#define BMI160_USER_INTR_ENABLE_0_ANY_MOTION_Y_ENABLE__MSK               0x02
#define BMI160_USER_INTR_ENABLE_0_ANY_MOTION_Y_ENABLE__REG	          \
BMI160_USER_INTR_ENABLE_0_ADDR

/* Int_En_0 Description - Reg Addr --> 0x50, Bit -->2 */
#define BMI160_USER_INTR_ENABLE_0_ANY_MOTION_Z_ENABLE__POS               2
#define BMI160_USER_INTR_ENABLE_0_ANY_MOTION_Z_ENABLE__LEN               1
#define BMI160_USER_INTR_ENABLE_0_ANY_MOTION_Z_ENABLE__MSK               0x04
#define BMI160_USER_INTR_ENABLE_0_ANY_MOTION_Z_ENABLE__REG	            \
BMI160_USER_INTR_ENABLE_0_ADDR

/* Int_En_0 Description - Reg Addr --> 0x50, Bit -->4 */
#define BMI160_USER_INTR_ENABLE_0_DOUBLE_TAP_ENABLE__POS               4
#define BMI160_USER_INTR_ENABLE_0_DOUBLE_TAP_ENABLE__LEN               1
#define BMI160_USER_INTR_ENABLE_0_DOUBLE_TAP_ENABLE__MSK               0x10
#define BMI160_USER_INTR_ENABLE_0_DOUBLE_TAP_ENABLE__REG	        \
BMI160_USER_INTR_ENABLE_0_ADDR

/* Int_En_0 Description - Reg Addr --> 0x50, Bit -->5 */
#define BMI160_USER_INTR_ENABLE_0_SINGLE_TAP_ENABLE__POS               5
#define BMI160_USER_INTR_ENABLE_0_SINGLE_TAP_ENABLE__LEN               1
#define BMI160_USER_INTR_ENABLE_0_SINGLE_TAP_ENABLE__MSK               0x20
#define BMI160_USER_INTR_ENABLE_0_SINGLE_TAP_ENABLE__REG	       \
BMI160_USER_INTR_ENABLE_0_ADDR

/* Int_En_0 Description - Reg Addr --> 0x50, Bit -->6 */
#define BMI160_USER_INTR_ENABLE_0_ORIENT_ENABLE__POS               6
#define BMI160_USER_INTR_ENABLE_0_ORIENT_ENABLE__LEN               1
#define BMI160_USER_INTR_ENABLE_0_ORIENT_ENABLE__MSK               0x40
#define BMI160_USER_INTR_ENABLE_0_ORIENT_ENABLE__REG	           \
BMI160_USER_INTR_ENABLE_0_ADDR

/* Int_En_0 Description - Reg Addr --> 0x50, Bit -->7 */
#define BMI160_USER_INTR_ENABLE_0_FLAT_ENABLE__POS               7
#define BMI160_USER_INTR_ENABLE_0_FLAT_ENABLE__LEN               1
#define BMI160_USER_INTR_ENABLE_0_FLAT_ENABLE__MSK               0x80
#define BMI160_USER_INTR_ENABLE_0_FLAT_ENABLE__REG	           \
BMI160_USER_INTR_ENABLE_0_ADDR

/* Int_En_1 Description - Reg Addr --> 0x51, Bit -->0 */
#define BMI160_USER_INTR_ENABLE_1_HIGH_G_X_ENABLE__POS               0
#define BMI160_USER_INTR_ENABLE_1_HIGH_G_X_ENABLE__LEN               1
#define BMI160_USER_INTR_ENABLE_1_HIGH_G_X_ENABLE__MSK               0x01
#define BMI160_USER_INTR_ENABLE_1_HIGH_G_X_ENABLE__REG	           \
BMI160_USER_INTR_ENABLE_1_ADDR

/* Int_En_1 Description - Reg Addr --> 0x51, Bit -->1 */
#define BMI160_USER_INTR_ENABLE_1_HIGH_G_Y_ENABLE__POS               1
#define BMI160_USER_INTR_ENABLE_1_HIGH_G_Y_ENABLE__LEN               1
#define BMI160_USER_INTR_ENABLE_1_HIGH_G_Y_ENABLE__MSK               0x02
#define BMI160_USER_INTR_ENABLE_1_HIGH_G_Y_ENABLE__REG	           \
BMI160_USER_INTR_ENABLE_1_ADDR

/* Int_En_1 Description - Reg Addr --> 0x51, Bit -->2 */
#define BMI160_USER_INTR_ENABLE_1_HIGH_G_Z_ENABLE__POS               2
#define BMI160_USER_INTR_ENABLE_1_HIGH_G_Z_ENABLE__LEN               1
#define BMI160_USER_INTR_ENABLE_1_HIGH_G_Z_ENABLE__MSK               0x04
#define BMI160_USER_INTR_ENABLE_1_HIGH_G_Z_ENABLE__REG	           \
BMI160_USER_INTR_ENABLE_1_ADDR

/* Int_En_1 Description - Reg Addr --> 0x51, Bit -->3 */
#define BMI160_USER_INTR_ENABLE_1_LOW_G_ENABLE__POS               3
#define BMI160_USER_INTR_ENABLE_1_LOW_G_ENABLE__LEN               1
#define BMI160_USER_INTR_ENABLE_1_LOW_G_ENABLE__MSK               0x08
#define BMI160_USER_INTR_ENABLE_1_LOW_G_ENABLE__REG	          \
BMI160_USER_INTR_ENABLE_1_ADDR

/* Int_En_1 Description - Reg Addr --> 0x51, Bit -->4 */
#define BMI160_USER_INTR_ENABLE_1_DATA_RDY_ENABLE__POS               4
#define BMI160_USER_INTR_ENABLE_1_DATA_RDY_ENABLE__LEN               1
#define BMI160_USER_INTR_ENABLE_1_DATA_RDY_ENABLE__MSK               0x10
#define BMI160_USER_INTR_ENABLE_1_DATA_RDY_ENABLE__REG	            \
BMI160_USER_INTR_ENABLE_1_ADDR

/* Int_En_1 Description - Reg Addr --> 0x51, Bit -->5 */
#define BMI160_USER_INTR_ENABLE_1_FIFO_FULL_ENABLE__POS               5
#define BMI160_USER_INTR_ENABLE_1_FIFO_FULL_ENABLE__LEN               1
#define BMI160_USER_INTR_ENABLE_1_FIFO_FULL_ENABLE__MSK               0x20
#define BMI160_USER_INTR_ENABLE_1_FIFO_FULL_ENABLE__REG	              \
BMI160_USER_INTR_ENABLE_1_ADDR

/* Int_En_1 Description - Reg Addr --> 0x51, Bit -->6 */
#define BMI160_USER_INTR_ENABLE_1_FIFO_WM_ENABLE__POS               6
#define BMI160_USER_INTR_ENABLE_1_FIFO_WM_ENABLE__LEN               1
#define BMI160_USER_INTR_ENABLE_1_FIFO_WM_ENABLE__MSK               0x40
#define BMI160_USER_INTR_ENABLE_1_FIFO_WM_ENABLE__REG	           \
BMI160_USER_INTR_ENABLE_1_ADDR

/* Int_En_2 Description - Reg Addr --> 0x52, Bit -->0 */
#define BMI160_USER_INTR_ENABLE_2_NOMOTION_X_ENABLE__POS               0
#define BMI160_USER_INTR_ENABLE_2_NOMOTION_X_ENABLE__LEN               1
#define BMI160_USER_INTR_ENABLE_2_NOMOTION_X_ENABLE__MSK               0x01
#define BMI160_USER_INTR_ENABLE_2_NOMOTION_X_ENABLE__REG	  \
BMI160_USER_INTR_ENABLE_2_ADDR

/* Int_En_2 Description - Reg Addr --> 0x52, Bit -->1 */
#define BMI160_USER_INTR_ENABLE_2_NOMOTION_Y_ENABLE__POS               1
#define BMI160_USER_INTR_ENABLE_2_NOMOTION_Y_ENABLE__LEN               1
#define BMI160_USER_INTR_ENABLE_2_NOMOTION_Y_ENABLE__MSK               0x02
#define BMI160_USER_INTR_ENABLE_2_NOMOTION_Y_ENABLE__REG	  \
BMI160_USER_INTR_ENABLE_2_ADDR

/* Int_En_2 Description - Reg Addr --> 0x52, Bit -->2 */
#define BMI160_USER_INTR_ENABLE_2_NOMOTION_Z_ENABLE__POS               2
#define BMI160_USER_INTR_ENABLE_2_NOMOTION_Z_ENABLE__LEN               1
#define BMI160_USER_INTR_ENABLE_2_NOMOTION_Z_ENABLE__MSK               0x04
#define BMI160_USER_INTR_ENABLE_2_NOMOTION_Z_ENABLE__REG	  \
BMI160_USER_INTR_ENABLE_2_ADDR

/* Int_En_2 Description - Reg Addr --> 0x52, Bit -->3 */
#define BMI160_USER_INTR_ENABLE_2_STEP_DETECTOR_ENABLE__POS               3
#define BMI160_USER_INTR_ENABLE_2_STEP_DETECTOR_ENABLE__LEN               1
#define BMI160_USER_INTR_ENABLE_2_STEP_DETECTOR_ENABLE__MSK               0x08
#define BMI160_USER_INTR_ENABLE_2_STEP_DETECTOR_ENABLE__REG	  \
BMI160_USER_INTR_ENABLE_2_ADDR

/* Int_Out_Ctrl Description - Reg Addr --> 0x53, Bit -->0 */
#define BMI160_USER_INTR1_EDGE_CTRL__POS               0
#define BMI160_USER_INTR1_EDGE_CTRL__LEN               1
#define BMI160_USER_INTR1_EDGE_CTRL__MSK               0x01
#define BMI160_USER_INTR1_EDGE_CTRL__REG		\
BMI160_USER_INTR_OUT_CTRL_ADDR

/* Int_Out_Ctrl Description - Reg Addr --> 0x53, Bit -->1 */
#define BMI160_USER_INTR1_LEVEL__POS               1
#define BMI160_USER_INTR1_LEVEL__LEN               1
#define BMI160_USER_INTR1_LEVEL__MSK               0x02
#define BMI160_USER_INTR1_LEVEL__REG               \
BMI160_USER_INTR_OUT_CTRL_ADDR

/* Int_Out_Ctrl Description - Reg Addr --> 0x53, Bit -->2 */
#define BMI160_USER_INTR1_OUTPUT_TYPE__POS               2
#define BMI160_USER_INTR1_OUTPUT_TYPE__LEN               1
#define BMI160_USER_INTR1_OUTPUT_TYPE__MSK               0x04
#define BMI160_USER_INTR1_OUTPUT_TYPE__REG               \
BMI160_USER_INTR_OUT_CTRL_ADDR

/* Int_Out_Ctrl Description - Reg Addr --> 0x53, Bit -->3 */
#define BMI160_USER_INTR1_OUTPUT_ENABLE__POS               3
#define BMI160_USER_INTR1_OUTPUT_ENABLE__LEN               1
#define BMI160_USER_INTR1_OUTPUT_ENABLE__MSK               0x08
#define BMI160_USER_INTR1_OUTPUT_ENABLE__REG		\
BMI160_USER_INTR_OUT_CTRL_ADDR

/* Int_Out_Ctrl Description - Reg Addr --> 0x53, Bit -->4 */
#define BMI160_USER_INTR2_EDGE_CTRL__POS               4
#define BMI160_USER_INTR2_EDGE_CTRL__LEN               1
#define BMI160_USER_INTR2_EDGE_CTRL__MSK               0x10
#define BMI160_USER_INTR2_EDGE_CTRL__REG		\
BMI160_USER_INTR_OUT_CTRL_ADDR

/* Int_Out_Ctrl Description - Reg Addr --> 0x53, Bit -->5 */
#define BMI160_USER_INTR2_LEVEL__POS               5
#define BMI160_USER_INTR2_LEVEL__LEN               1
#define BMI160_USER_INTR2_LEVEL__MSK               0x20
#define BMI160_USER_INTR2_LEVEL__REG               \
BMI160_USER_INTR_OUT_CTRL_ADDR

/* Int_Out_Ctrl Description - Reg Addr --> 0x53, Bit -->6 */
#define BMI160_USER_INTR2_OUTPUT_TYPE__POS               6
#define BMI160_USER_INTR2_OUTPUT_TYPE__LEN               1
#define BMI160_USER_INTR2_OUTPUT_TYPE__MSK               0x40
#define BMI160_USER_INTR2_OUTPUT_TYPE__REG               \
BMI160_USER_INTR_OUT_CTRL_ADDR

/* Int_Out_Ctrl Description - Reg Addr --> 0x53, Bit -->7 */
#define BMI160_USER_INTR2_OUTPUT_EN__POS               7
#define BMI160_USER_INTR2_OUTPUT_EN__LEN               1
#define BMI160_USER_INTR2_OUTPUT_EN__MSK               0x80
#define BMI160_USER_INTR2_OUTPUT_EN__REG		\
BMI160_USER_INTR_OUT_CTRL_ADDR

/* Int_Latch Description - Reg Addr --> 0x54, Bit -->0...3 */
#define BMI160_USER_INTR_LATCH__POS               0
#define BMI160_USER_INTR_LATCH__LEN               4
#define BMI160_USER_INTR_LATCH__MSK               0x0F
#define BMI160_USER_INTR_LATCH__REG               BMI160_USER_INTR_LATCH_ADDR

/* Int_Latch Description - Reg Addr --> 0x54, Bit -->4 */
#define BMI160_USER_INTR1_INPUT_ENABLE__POS               4
#define BMI160_USER_INTR1_INPUT_ENABLE__LEN               1
#define BMI160_USER_INTR1_INPUT_ENABLE__MSK               0x10
#define BMI160_USER_INTR1_INPUT_ENABLE__REG               \
BMI160_USER_INTR_LATCH_ADDR

/* Int_Latch Description - Reg Addr --> 0x54, Bit -->5*/
#define BMI160_USER_INTR2_INPUT_ENABLE__POS               5
#define BMI160_USER_INTR2_INPUT_ENABLE__LEN               1
#define BMI160_USER_INTR2_INPUT_ENABLE__MSK               0x20
#define BMI160_USER_INTR2_INPUT_ENABLE__REG              \
BMI160_USER_INTR_LATCH_ADDR

/* Int_Map_0 Description - Reg Addr --> 0x55, Bit -->0 */
#define BMI160_USER_INTR_MAP_0_INTR1_LOW_G__POS               0
#define BMI160_USER_INTR_MAP_0_INTR1_LOW_G__LEN               1
#define BMI160_USER_INTR_MAP_0_INTR1_LOW_G__MSK               0x01
#define BMI160_USER_INTR_MAP_0_INTR1_LOW_G__REG	BMI160_USER_INTR_MAP_0_ADDR

/* Int_Map_0 Description - Reg Addr --> 0x55, Bit -->1 */
#define BMI160_USER_INTR_MAP_0_INTR1_HIGH_G__POS               1
#define BMI160_USER_INTR_MAP_0_INTR1_HIGH_G__LEN               1
#define BMI160_USER_INTR_MAP_0_INTR1_HIGH_G__MSK               0x02
#define BMI160_USER_INTR_MAP_0_INTR1_HIGH_G__REG	\
BMI160_USER_INTR_MAP_0_ADDR

/* Int_Map_0 Description - Reg Addr --> 0x55, Bit -->2 */
#define BMI160_USER_INTR_MAP_0_INTR1_ANY_MOTION__POS               2
#define BMI160_USER_INTR_MAP_0_INTR1_ANY_MOTION__LEN               1
#define BMI160_USER_INTR_MAP_0_INTR1_ANY_MOTION__MSK               0x04
#define BMI160_USER_INTR_MAP_0_INTR1_ANY_MOTION__REG BMI160_USER_INTR_MAP_0_ADDR

/* Int_Map_0 Description - Reg Addr --> 0x55, Bit -->3 */
#define BMI160_USER_INTR_MAP_0_INTR1_NOMOTION__POS               3
#define BMI160_USER_INTR_MAP_0_INTR1_NOMOTION__LEN               1
#define BMI160_USER_INTR_MAP_0_INTR1_NOMOTION__MSK               0x08
#define BMI160_USER_INTR_MAP_0_INTR1_NOMOTION__REG BMI160_USER_INTR_MAP_0_ADDR

/* Int_Map_0 Description - Reg Addr --> 0x55, Bit -->4 */
#define BMI160_USER_INTR_MAP_0_INTR1_DOUBLE_TAP__POS               4
#define BMI160_USER_INTR_MAP_0_INTR1_DOUBLE_TAP__LEN               1
#define BMI160_USER_INTR_MAP_0_INTR1_DOUBLE_TAP__MSK               0x10
#define BMI160_USER_INTR_MAP_0_INTR1_DOUBLE_TAP__REG	\
BMI160_USER_INTR_MAP_0_ADDR

/* Int_Map_0 Description - Reg Addr --> 0x55, Bit -->5 */
#define BMI160_USER_INTR_MAP_0_INTR1_SINGLE_TAP__POS               5
#define BMI160_USER_INTR_MAP_0_INTR1_SINGLE_TAP__LEN               1
#define BMI160_USER_INTR_MAP_0_INTR1_SINGLE_TAP__MSK               0x20
#define BMI160_USER_INTR_MAP_0_INTR1_SINGLE_TAP__REG	      \
BMI160_USER_INTR_MAP_0_ADDR

/* Int_Map_0 Description - Reg Addr --> 0x55, Bit -->6 */
#define BMI160_USER_INTR_MAP_0_INTR1_ORIENT__POS               6
#define BMI160_USER_INTR_MAP_0_INTR1_ORIENT__LEN               1
#define BMI160_USER_INTR_MAP_0_INTR1_ORIENT__MSK               0x40
#define BMI160_USER_INTR_MAP_0_INTR1_ORIENT__REG	          \
BMI160_USER_INTR_MAP_0_ADDR

/* Int_Map_0 Description - Reg Addr --> 0x56, Bit -->7 */
#define BMI160_USER_INTR_MAP_0_INTR1_FLAT__POS               7
#define BMI160_USER_INTR_MAP_0_INTR1_FLAT__LEN               1
#define BMI160_USER_INTR_MAP_0_INTR1_FLAT__MSK               0x80
#define BMI160_USER_INTR_MAP_0_INTR1_FLAT__REG	BMI160_USER_INTR_MAP_0_ADDR

/* Int_Map_1 Description - Reg Addr --> 0x56, Bit -->0 */
#define BMI160_USER_INTR_MAP_1_INTR2_PMU_TRIG__POS               0
#define BMI160_USER_INTR_MAP_1_INTR2_PMU_TRIG__LEN               1
#define BMI160_USER_INTR_MAP_1_INTR2_PMU_TRIG__MSK               0x01
#define BMI160_USER_INTR_MAP_1_INTR2_PMU_TRIG__REG BMI160_USER_INTR_MAP_1_ADDR

/* Int_Map_1 Description - Reg Addr --> 0x56, Bit -->1 */
#define BMI160_USER_INTR_MAP_1_INTR2_FIFO_FULL__POS               1
#define BMI160_USER_INTR_MAP_1_INTR2_FIFO_FULL__LEN               1
#define BMI160_USER_INTR_MAP_1_INTR2_FIFO_FULL__MSK               0x02
#define BMI160_USER_INTR_MAP_1_INTR2_FIFO_FULL__REG	         \
BMI160_USER_INTR_MAP_1_ADDR

/* Int_Map_1 Description - Reg Addr --> 0x56, Bit -->2 */
#define BMI160_USER_INTR_MAP_1_INTR2_FIFO_WM__POS               2
#define BMI160_USER_INTR_MAP_1_INTR2_FIFO_WM__LEN               1
#define BMI160_USER_INTR_MAP_1_INTR2_FIFO_WM__MSK               0x04
#define BMI160_USER_INTR_MAP_1_INTR2_FIFO_WM__REG	         \
BMI160_USER_INTR_MAP_1_ADDR

/* Int_Map_1 Description - Reg Addr --> 0x56, Bit -->3 */
#define BMI160_USER_INTR_MAP_1_INTR2_DATA_RDY__POS               3
#define BMI160_USER_INTR_MAP_1_INTR2_DATA_RDY__LEN               1
#define BMI160_USER_INTR_MAP_1_INTR2_DATA_RDY__MSK               0x08
#define BMI160_USER_INTR_MAP_1_INTR2_DATA_RDY__REG	      \
BMI160_USER_INTR_MAP_1_ADDR

/* Int_Map_1 Description - Reg Addr --> 0x56, Bit -->4 */
#define BMI160_USER_INTR_MAP_1_INTR1_PMU_TRIG__POS               4
#define BMI160_USER_INTR_MAP_1_INTR1_PMU_TRIG__LEN               1
#define BMI160_USER_INTR_MAP_1_INTR1_PMU_TRIG__MSK               0x10
#define BMI160_USER_INTR_MAP_1_INTR1_PMU_TRIG__REG BMI160_USER_INTR_MAP_1_ADDR

/* Int_Map_1 Description - Reg Addr --> 0x56, Bit -->5 */
#define BMI160_USER_INTR_MAP_1_INTR1_FIFO_FULL__POS               5
#define BMI160_USER_INTR_MAP_1_INTR1_FIFO_FULL__LEN               1
#define BMI160_USER_INTR_MAP_1_INTR1_FIFO_FULL__MSK               0x20
#define BMI160_USER_INTR_MAP_1_INTR1_FIFO_FULL__REG	       \
BMI160_USER_INTR_MAP_1_ADDR

/* Int_Map_1 Description - Reg Addr --> 0x56, Bit -->6 */
#define BMI160_USER_INTR_MAP_1_INTR1_FIFO_WM__POS               6
#define BMI160_USER_INTR_MAP_1_INTR1_FIFO_WM__LEN               1
#define BMI160_USER_INTR_MAP_1_INTR1_FIFO_WM__MSK               0x40
#define BMI160_USER_INTR_MAP_1_INTR1_FIFO_WM__REG	\
BMI160_USER_INTR_MAP_1_ADDR

/* Int_Map_1 Description - Reg Addr --> 0x56, Bit -->7 */
#define BMI160_USER_INTR_MAP_1_INTR1_DATA_RDY__POS               7
#define BMI160_USER_INTR_MAP_1_INTR1_DATA_RDY__LEN               1
#define BMI160_USER_INTR_MAP_1_INTR1_DATA_RDY__MSK               0x80
#define BMI160_USER_INTR_MAP_1_INTR1_DATA_RDY__REG	\
BMI160_USER_INTR_MAP_1_ADDR

/* Int_Map_2 Description - Reg Addr --> 0x57, Bit -->0 */
#define BMI160_USER_INTR_MAP_2_INTR2_LOW_G__POS               0
#define BMI160_USER_INTR_MAP_2_INTR2_LOW_G__LEN               1
#define BMI160_USER_INTR_MAP_2_INTR2_LOW_G__MSK               0x01
#define BMI160_USER_INTR_MAP_2_INTR2_LOW_G__REG	BMI160_USER_INTR_MAP_2_ADDR

/* Int_Map_2 Description - Reg Addr --> 0x57, Bit -->1 */
#define BMI160_USER_INTR_MAP_2_INTR2_HIGH_G__POS               1
#define BMI160_USER_INTR_MAP_2_INTR2_HIGH_G__LEN               1
#define BMI160_USER_INTR_MAP_2_INTR2_HIGH_G__MSK               0x02
#define BMI160_USER_INTR_MAP_2_INTR2_HIGH_G__REG	\
BMI160_USER_INTR_MAP_2_ADDR

/* Int_Map_2 Description - Reg Addr --> 0x57, Bit -->2 */
#define BMI160_USER_INTR_MAP_2_INTR2_ANY_MOTION__POS               2
#define BMI160_USER_INTR_MAP_2_INTR2_ANY_MOTION__LEN               1
#define BMI160_USER_INTR_MAP_2_INTR2_ANY_MOTION__MSK               0x04
#define BMI160_USER_INTR_MAP_2_INTR2_ANY_MOTION__REG BMI160_USER_INTR_MAP_2_ADDR

/* Int_Map_2 Description - Reg Addr --> 0x57, Bit -->3 */
#define BMI160_USER_INTR_MAP_2_INTR2_NOMOTION__POS               3
#define BMI160_USER_INTR_MAP_2_INTR2_NOMOTION__LEN               1
#define BMI160_USER_INTR_MAP_2_INTR2_NOMOTION__MSK               0x08
#define BMI160_USER_INTR_MAP_2_INTR2_NOMOTION__REG BMI160_USER_INTR_MAP_2_ADDR

/* Int_Map_2 Description - Reg Addr --> 0x57, Bit -->4 */
#define BMI160_USER_INTR_MAP_2_INTR2_DOUBLE_TAP__POS               4
#define BMI160_USER_INTR_MAP_2_INTR2_DOUBLE_TAP__LEN               1
#define BMI160_USER_INTR_MAP_2_INTR2_DOUBLE_TAP__MSK               0x10
#define BMI160_USER_INTR_MAP_2_INTR2_DOUBLE_TAP__REG	\
BMI160_USER_INTR_MAP_2_ADDR

/* Int_Map_2 Description - Reg Addr --> 0x57, Bit -->5 */
#define BMI160_USER_INTR_MAP_2_INTR2_SINGLE_TAP__POS               5
#define BMI160_USER_INTR_MAP_2_INTR2_SINGLE_TAP__LEN               1
#define BMI160_USER_INTR_MAP_2_INTR2_SINGLE_TAP__MSK               0x20
#define BMI160_USER_INTR_MAP_2_INTR2_SINGLE_TAP__REG	\
BMI160_USER_INTR_MAP_2_ADDR

/* Int_Map_2 Description - Reg Addr --> 0x57, Bit -->6 */
#define BMI160_USER_INTR_MAP_2_INTR2_ORIENT__POS               6
#define BMI160_USER_INTR_MAP_2_INTR2_ORIENT__LEN               1
#define BMI160_USER_INTR_MAP_2_INTR2_ORIENT__MSK               0x40
#define BMI160_USER_INTR_MAP_2_INTR2_ORIENT__REG	\
BMI160_USER_INTR_MAP_2_ADDR

/* Int_Map_2 Description - Reg Addr --> 0x57, Bit -->7 */

#define BMI160_USER_INTR_MAP_2_INTR2_FLAT__POS               7
#define BMI160_USER_INTR_MAP_2_INTR2_FLAT__LEN               1
#define BMI160_USER_INTR_MAP_2_INTR2_FLAT__MSK               0x80
#define BMI160_USER_INTR_MAP_2_INTR2_FLAT__REG	BMI160_USER_INTR_MAP_2_ADDR


/* Int_Data_0 Description - Reg Addr --> 0x58, Bit --> 3 */
#define BMI160_USER_INTR_DATA_0_INTR_TAP_SOURCE__POS               3
#define BMI160_USER_INTR_DATA_0_INTR_TAP_SOURCE__LEN               1
#define BMI160_USER_INTR_DATA_0_INTR_TAP_SOURCE__MSK               0x08
#define BMI160_USER_INTR_DATA_0_INTR_TAP_SOURCE__REG	           \
BMI160_USER_INTR_DATA_0_ADDR


/* Int_Data_0 Description - Reg Addr --> 0x58, Bit --> 7 */
#define BMI160_USER_INTR_DATA_0_INTR_LOW_HIGH_SOURCE__POS           7
#define BMI160_USER_INTR_DATA_0_INTR_LOW_HIGH_SOURCE__LEN           1
#define BMI160_USER_INTR_DATA_0_INTR_LOW_HIGH_SOURCE__MSK           0x80
#define BMI160_USER_INTR_DATA_0_INTR_LOW_HIGH_SOURCE__REG            \
BMI160_USER_INTR_DATA_0_ADDR


/* Int_Data_1 Description - Reg Addr --> 0x59, Bit --> 7 */
#define BMI160_USER_INTR_DATA_1_INTR_MOTION_SOURCE__POS               7
#define BMI160_USER_INTR_DATA_1_INTR_MOTION_SOURCE__LEN               1
#define BMI160_USER_INTR_DATA_1_INTR_MOTION_SOURCE__MSK               0x80
#define BMI160_USER_INTR_DATA_1_INTR_MOTION_SOURCE__REG               \
		BMI160_USER_INTR_DATA_1_ADDR

/* Int_LowHigh_0 Description - Reg Addr --> 0x5a, Bit --> 0...7 */
#define BMI160_USER_INTR_LOWHIGH_0_INTR_LOW_DURN__POS               0
#define BMI160_USER_INTR_LOWHIGH_0_INTR_LOW_DURN__LEN               8
#define BMI160_USER_INTR_LOWHIGH_0_INTR_LOW_DURN__MSK               0xFF
#define BMI160_USER_INTR_LOWHIGH_0_INTR_LOW_DURN__REG               \
		BMI160_USER_INTR_LOWHIGH_0_ADDR

/* Int_LowHigh_1 Description - Reg Addr --> 0x5b, Bit --> 0...7 */
#define BMI160_USER_INTR_LOWHIGH_1_INTR_LOW_THRES__POS               0
#define BMI160_USER_INTR_LOWHIGH_1_INTR_LOW_THRES__LEN               8
#define BMI160_USER_INTR_LOWHIGH_1_INTR_LOW_THRES__MSK               0xFF
#define BMI160_USER_INTR_LOWHIGH_1_INTR_LOW_THRES__REG               \
		BMI160_USER_INTR_LOWHIGH_1_ADDR

/* Int_LowHigh_2 Description - Reg Addr --> 0x5c, Bit --> 0...1 */
#define BMI160_USER_INTR_LOWHIGH_2_INTR_LOW_G_HYST__POS               0
#define BMI160_USER_INTR_LOWHIGH_2_INTR_LOW_G_HYST__LEN               2
#define BMI160_USER_INTR_LOWHIGH_2_INTR_LOW_G_HYST__MSK               0x03
#define BMI160_USER_INTR_LOWHIGH_2_INTR_LOW_G_HYST__REG               \
		BMI160_USER_INTR_LOWHIGH_2_ADDR

/* Int_LowHigh_2 Description - Reg Addr --> 0x5c, Bit --> 2 */
#define BMI160_USER_INTR_LOWHIGH_2_INTR_LOW_G_MODE__POS               2
#define BMI160_USER_INTR_LOWHIGH_2_INTR_LOW_G_MODE__LEN               1
#define BMI160_USER_INTR_LOWHIGH_2_INTR_LOW_G_MODE__MSK               0x04
#define BMI160_USER_INTR_LOWHIGH_2_INTR_LOW_G_MODE__REG               \
		BMI160_USER_INTR_LOWHIGH_2_ADDR

/* Int_LowHigh_2 Description - Reg Addr --> 0x5c, Bit --> 6...7 */
#define BMI160_USER_INTR_LOWHIGH_2_INTR_HIGH_G_HYST__POS               6
#define BMI160_USER_INTR_LOWHIGH_2_INTR_HIGH_G_HYST__LEN               2
#define BMI160_USER_INTR_LOWHIGH_2_INTR_HIGH_G_HYST__MSK               0xC0
#define BMI160_USER_INTR_LOWHIGH_2_INTR_HIGH_G_HYST__REG               \
		BMI160_USER_INTR_LOWHIGH_2_ADDR

/* Int_LowHigh_3 Description - Reg Addr --> 0x5d, Bit --> 0...7 */
#define BMI160_USER_INTR_LOWHIGH_3_INTR_HIGH_G_DURN__POS               0
#define BMI160_USER_INTR_LOWHIGH_3_INTR_HIGH_G_DURN__LEN               8
#define BMI160_USER_INTR_LOWHIGH_3_INTR_HIGH_G_DURN__MSK               0xFF
#define BMI160_USER_INTR_LOWHIGH_3_INTR_HIGH_G_DURN__REG               \
		BMI160_USER_INTR_LOWHIGH_3_ADDR

/* Int_LowHigh_4 Description - Reg Addr --> 0x5e, Bit --> 0...7 */
#define BMI160_USER_INTR_LOWHIGH_4_INTR_HIGH_THRES__POS               0
#define BMI160_USER_INTR_LOWHIGH_4_INTR_HIGH_THRES__LEN               8
#define BMI160_USER_INTR_LOWHIGH_4_INTR_HIGH_THRES__MSK               0xFF
#define BMI160_USER_INTR_LOWHIGH_4_INTR_HIGH_THRES__REG               \
		BMI160_USER_INTR_LOWHIGH_4_ADDR

/* Int_Motion_0 Description - Reg Addr --> 0x5f, Bit --> 0...1 */
#define BMI160_USER_INTR_MOTION_0_INTR_ANY_MOTION_DURN__POS               0
#define BMI160_USER_INTR_MOTION_0_INTR_ANY_MOTION_DURN__LEN               2
#define BMI160_USER_INTR_MOTION_0_INTR_ANY_MOTION_DURN__MSK               0x03
#define BMI160_USER_INTR_MOTION_0_INTR_ANY_MOTION_DURN__REG               \
		BMI160_USER_INTR_MOTION_0_ADDR

	/* Int_Motion_0 Description - Reg Addr --> 0x5f, Bit --> 2...7 */
#define BMI160_USER_INTR_MOTION_0_INTR_SLOW_NO_MOTION_DURN__POS      2
#define BMI160_USER_INTR_MOTION_0_INTR_SLOW_NO_MOTION_DURN__LEN      6
#define BMI160_USER_INTR_MOTION_0_INTR_SLOW_NO_MOTION_DURN__MSK      0xFC
#define BMI160_USER_INTR_MOTION_0_INTR_SLOW_NO_MOTION_DURN__REG       \
		BMI160_USER_INTR_MOTION_0_ADDR

/* Int_Motion_1 Description - Reg Addr --> 0x60, Bit --> 0...7 */
#define BMI160_USER_INTR_MOTION_1_INTR_ANY_MOTION_THRES__POS               0
#define BMI160_USER_INTR_MOTION_1_INTR_ANY_MOTION_THRES__LEN               8
#define BMI160_USER_INTR_MOTION_1_INTR_ANY_MOTION_THRES__MSK               0xFF
#define BMI160_USER_INTR_MOTION_1_INTR_ANY_MOTION_THRES__REG               \
		BMI160_USER_INTR_MOTION_1_ADDR

/* Int_Motion_2 Description - Reg Addr --> 0x61, Bit --> 0...7 */
#define BMI160_USER_INTR_MOTION_2_INTR_SLOW_NO_MOTION_THRES__POS       0
#define BMI160_USER_INTR_MOTION_2_INTR_SLOW_NO_MOTION_THRES__LEN       8
#define BMI160_USER_INTR_MOTION_2_INTR_SLOW_NO_MOTION_THRES__MSK       0xFF
#define BMI160_USER_INTR_MOTION_2_INTR_SLOW_NO_MOTION_THRES__REG       \
		BMI160_USER_INTR_MOTION_2_ADDR

/* Int_Motion_3 Description - Reg Addr --> 0x62, Bit --> 0 */
#define BMI160_USER_INTR_MOTION_3_INTR_SLOW_NO_MOTION_SELECT__POS	0
#define BMI160_USER_INTR_MOTION_3_INTR_SLOW_NO_MOTION_SELECT__LEN	1
#define BMI160_USER_INTR_MOTION_3_INTR_SLOW_NO_MOTION_SELECT__MSK	0x01
#define BMI160_USER_INTR_MOTION_3_INTR_SLOW_NO_MOTION_SELECT__REG   \
BMI160_USER_INTR_MOTION_3_ADDR

/* Int_Motion_3 Description - Reg Addr --> 0x62, Bit --> 1 */
#define BMI160_USER_INTR_SIGNIFICATION_MOTION_SELECT__POS	1
#define BMI160_USER_INTR_SIGNIFICATION_MOTION_SELECT__LEN		1
#define BMI160_USER_INTR_SIGNIFICATION_MOTION_SELECT__MSK		0x02
#define BMI160_USER_INTR_SIGNIFICATION_MOTION_SELECT__REG		\
		BMI160_USER_INTR_MOTION_3_ADDR

/* Int_Motion_3 Description - Reg Addr --> 0x62, Bit --> 3..2 */
#define BMI160_USER_INTR_SIGNIFICANT_MOTION_SKIP__POS		2
#define BMI160_USER_INTR_SIGNIFICANT_MOTION_SKIP__LEN		2
#define BMI160_USER_INTR_SIGNIFICANT_MOTION_SKIP__MSK		0x0C
#define BMI160_USER_INTR_SIGNIFICANT_MOTION_SKIP__REG		\
		BMI160_USER_INTR_MOTION_3_ADDR

/* Int_Motion_3 Description - Reg Addr --> 0x62, Bit --> 5..4 */
#define BMI160_USER_INTR_SIGNIFICANT_MOTION_PROOF__POS		4
#define BMI160_USER_INTR_SIGNIFICANT_MOTION_PROOF__LEN		2
#define BMI160_USER_INTR_SIGNIFICANT_MOTION_PROOF__MSK		0x30
#define BMI160_USER_INTR_SIGNIFICANT_MOTION_PROOF__REG		\
		BMI160_USER_INTR_MOTION_3_ADDR

/* INT_TAP_0 Description - Reg Addr --> 0x63, Bit --> 0..2*/
#define BMI160_USER_INTR_TAP_0_INTR_TAP_DURN__POS               0
#define BMI160_USER_INTR_TAP_0_INTR_TAP_DURN__LEN               3
#define BMI160_USER_INTR_TAP_0_INTR_TAP_DURN__MSK               0x07
#define BMI160_USER_INTR_TAP_0_INTR_TAP_DURN__REG	\
BMI160_USER_INTR_TAP_0_ADDR

/* Int_Tap_0 Description - Reg Addr --> 0x63, Bit --> 6 */
#define BMI160_USER_INTR_TAP_0_INTR_TAP_SHOCK__POS               6
#define BMI160_USER_INTR_TAP_0_INTR_TAP_SHOCK__LEN               1
#define BMI160_USER_INTR_TAP_0_INTR_TAP_SHOCK__MSK               0x40
#define BMI160_USER_INTR_TAP_0_INTR_TAP_SHOCK__REG BMI160_USER_INTR_TAP_0_ADDR

/* Int_Tap_0 Description - Reg Addr --> 0x63, Bit --> 7 */
#define BMI160_USER_INTR_TAP_0_INTR_TAP_QUIET__POS               7
#define BMI160_USER_INTR_TAP_0_INTR_TAP_QUIET__LEN               1
#define BMI160_USER_INTR_TAP_0_INTR_TAP_QUIET__MSK               0x80
#define BMI160_USER_INTR_TAP_0_INTR_TAP_QUIET__REG BMI160_USER_INTR_TAP_0_ADDR

/* Int_Tap_1 Description - Reg Addr --> 0x64, Bit --> 0...4 */
#define BMI160_USER_INTR_TAP_1_INTR_TAP_THRES__POS               0
#define BMI160_USER_INTR_TAP_1_INTR_TAP_THRES__LEN               5
#define BMI160_USER_INTR_TAP_1_INTR_TAP_THRES__MSK               0x1F
#define BMI160_USER_INTR_TAP_1_INTR_TAP_THRES__REG BMI160_USER_INTR_TAP_1_ADDR

/* Int_Orient_0 Description - Reg Addr --> 0x65, Bit --> 0...1 */
#define BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_MODE__POS               0
#define BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_MODE__LEN               2
#define BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_MODE__MSK               0x03
#define BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_MODE__REG               \
		BMI160_USER_INTR_ORIENT_0_ADDR

/* Int_Orient_0 Description - Reg Addr --> 0x65, Bit --> 2...3 */
#define BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_BLOCKING__POS               2
#define BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_BLOCKING__LEN               2
#define BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_BLOCKING__MSK               0x0C
#define BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_BLOCKING__REG               \
		BMI160_USER_INTR_ORIENT_0_ADDR

/* Int_Orient_0 Description - Reg Addr --> 0x65, Bit --> 4...7 */
#define BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_HYST__POS               4
#define BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_HYST__LEN               4
#define BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_HYST__MSK               0xF0
#define BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_HYST__REG               \
		BMI160_USER_INTR_ORIENT_0_ADDR

/* Int_Orient_1 Description - Reg Addr --> 0x66, Bit --> 0...5 */
#define BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_THETA__POS               0
#define BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_THETA__LEN               6
#define BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_THETA__MSK               0x3F
#define BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_THETA__REG               \
		BMI160_USER_INTR_ORIENT_1_ADDR

/* Int_Orient_1 Description - Reg Addr --> 0x66, Bit --> 6 */
#define BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_UD_ENABLE__POS               6
#define BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_UD_ENABLE__LEN               1
#define BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_UD_ENABLE__MSK               0x40
#define BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_UD_ENABLE__REG               \
		BMI160_USER_INTR_ORIENT_1_ADDR

/* Int_Orient_1 Description - Reg Addr --> 0x66, Bit --> 7 */
#define BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_AXES_EX__POS               7
#define BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_AXES_EX__LEN               1
#define BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_AXES_EX__MSK               0x80
#define BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_AXES_EX__REG               \
		BMI160_USER_INTR_ORIENT_1_ADDR

/* Int_Flat_0 Description - Reg Addr --> 0x67, Bit --> 0...5 */
#define BMI160_USER_INTR_FLAT_0_INTR_FLAT_THETA__POS               0
#define BMI160_USER_INTR_FLAT_0_INTR_FLAT_THETA__LEN               6
#define BMI160_USER_INTR_FLAT_0_INTR_FLAT_THETA__MSK               0x3F
#define BMI160_USER_INTR_FLAT_0_INTR_FLAT_THETA__REG  \
		BMI160_USER_INTR_FLAT_0_ADDR

/* Int_Flat_1 Description - Reg Addr --> 0x68, Bit --> 0...3 */
#define BMI160_USER_INTR_FLAT_1_INTR_FLAT_HYST__POS		0
#define BMI160_USER_INTR_FLAT_1_INTR_FLAT_HYST__LEN		4
#define BMI160_USER_INTR_FLAT_1_INTR_FLAT_HYST__MSK		0x0F
#define BMI160_USER_INTR_FLAT_1_INTR_FLAT_HYST__REG	 \
BMI160_USER_INTR_FLAT_1_ADDR

/* Int_Flat_1 Description - Reg Addr --> 0x68, Bit --> 4...5 */
#define BMI160_USER_INTR_FLAT_1_INTR_FLAT_HOLD__POS                4
#define BMI160_USER_INTR_FLAT_1_INTR_FLAT_HOLD__LEN                2
#define BMI160_USER_INTR_FLAT_1_INTR_FLAT_HOLD__MSK                0x30
#define BMI160_USER_INTR_FLAT_1_INTR_FLAT_HOLD__REG  \
		BMI160_USER_INTR_FLAT_1_ADDR

/* Foc_Conf Description - Reg Addr --> 0x69, Bit --> 0...1 */
#define BMI160_USER_FOC_ACCEL_Z__POS               0
#define BMI160_USER_FOC_ACCEL_Z__LEN               2
#define BMI160_USER_FOC_ACCEL_Z__MSK               0x03
#define BMI160_USER_FOC_ACCEL_Z__REG               BMI160_USER_FOC_CONFIG_ADDR

/* Foc_Conf Description - Reg Addr --> 0x69, Bit --> 2...3 */
#define BMI160_USER_FOC_ACCEL_Y__POS               2
#define BMI160_USER_FOC_ACCEL_Y__LEN               2
#define BMI160_USER_FOC_ACCEL_Y__MSK               0x0C
#define BMI160_USER_FOC_ACCEL_Y__REG               BMI160_USER_FOC_CONFIG_ADDR

/* Foc_Conf Description - Reg Addr --> 0x69, Bit --> 4...5 */
#define BMI160_USER_FOC_ACCEL_X__POS               4
#define BMI160_USER_FOC_ACCEL_X__LEN               2
#define BMI160_USER_FOC_ACCEL_X__MSK               0x30
#define BMI160_USER_FOC_ACCEL_X__REG               BMI160_USER_FOC_CONFIG_ADDR

/* Foc_Conf Description - Reg Addr --> 0x69, Bit --> 6 */
#define BMI160_USER_FOC_GYRO_ENABLE__POS               6
#define BMI160_USER_FOC_GYRO_ENABLE__LEN               1
#define BMI160_USER_FOC_GYRO_ENABLE__MSK               0x40
#define BMI160_USER_FOC_GYRO_ENABLE__REG               \
BMI160_USER_FOC_CONFIG_ADDR

/* CONF Description - Reg Addr --> 0x6A, Bit --> 1 */
#define BMI160_USER_CONFIG_NVM_PROG_ENABLE__POS               1
#define BMI160_USER_CONFIG_NVM_PROG_ENABLE__LEN               1
#define BMI160_USER_CONFIG_NVM_PROG_ENABLE__MSK               0x02
#define BMI160_USER_CONFIG_NVM_PROG_ENABLE__REG               \
BMI160_USER_CONFIG_ADDR

/*IF_CONF Description - Reg Addr --> 0x6B, Bit --> 0 */

#define BMI160_USER_IF_CONFIG_SPI3__POS               0
#define BMI160_USER_IF_CONFIG_SPI3__LEN               1
#define BMI160_USER_IF_CONFIG_SPI3__MSK               0x01
#define BMI160_USER_IF_CONFIG_SPI3__REG               BMI160_USER_IF_CONFIG_ADDR

/*IF_CONF Description - Reg Addr --> 0x6B, Bit --> 5..4 */
#define BMI160_USER_IF_CONFIG_IF_MODE__POS               4
#define BMI160_USER_IF_CONFIG_IF_MODE__LEN               2
#define BMI160_USER_IF_CONFIG_IF_MODE__MSK               0x30
#define BMI160_USER_IF_CONFIG_IF_MODE__REG		\
BMI160_USER_IF_CONFIG_ADDR

/* Pmu_Trigger Description - Reg Addr --> 0x6c, Bit --> 0...2 */
#define BMI160_USER_GYRO_SLEEP_TRIGGER__POS               0
#define BMI160_USER_GYRO_SLEEP_TRIGGER__LEN               3
#define BMI160_USER_GYRO_SLEEP_TRIGGER__MSK               0x07
#define BMI160_USER_GYRO_SLEEP_TRIGGER__REG	BMI160_USER_PMU_TRIGGER_ADDR

/* Pmu_Trigger Description - Reg Addr --> 0x6c, Bit --> 3...4 */
#define BMI160_USER_GYRO_WAKEUP_TRIGGER__POS               3
#define BMI160_USER_GYRO_WAKEUP_TRIGGER__LEN               2
#define BMI160_USER_GYRO_WAKEUP_TRIGGER__MSK               0x18
#define BMI160_USER_GYRO_WAKEUP_TRIGGER__REG	BMI160_USER_PMU_TRIGGER_ADDR

/* Pmu_Trigger Description - Reg Addr --> 0x6c, Bit --> 5 */
#define BMI160_USER_GYRO_SLEEP_STATE__POS               5
#define BMI160_USER_GYRO_SLEEP_STATE__LEN               1
#define BMI160_USER_GYRO_SLEEP_STATE__MSK               0x20
#define BMI160_USER_GYRO_SLEEP_STATE__REG	BMI160_USER_PMU_TRIGGER_ADDR

/* Pmu_Trigger Description - Reg Addr --> 0x6c, Bit --> 6 */
#define BMI160_USER_GYRO_WAKEUP_INTR__POS               6
#define BMI160_USER_GYRO_WAKEUP_INTR__LEN               1
#define BMI160_USER_GYRO_WAKEUP_INTR__MSK               0x40
#define BMI160_USER_GYRO_WAKEUP_INTR__REG	BMI160_USER_PMU_TRIGGER_ADDR

/* Self_Test Description - Reg Addr --> 0x6d, Bit --> 0...1 */
#define BMI160_USER_ACCEL_SELFTEST_AXIS__POS               0
#define BMI160_USER_ACCEL_SELFTEST_AXIS__LEN               2
#define BMI160_USER_ACCEL_SELFTEST_AXIS__MSK               0x03
#define BMI160_USER_ACCEL_SELFTEST_AXIS__REG	BMI160_USER_SELF_TEST_ADDR

/* Self_Test Description - Reg Addr --> 0x6d, Bit --> 2 */
#define BMI160_USER_ACCEL_SELFTEST_SIGN__POS               2
#define BMI160_USER_ACCEL_SELFTEST_SIGN__LEN               1
#define BMI160_USER_ACCEL_SELFTEST_SIGN__MSK               0x04
#define BMI160_USER_ACCEL_SELFTEST_SIGN__REG	BMI160_USER_SELF_TEST_ADDR

/* Self_Test Description - Reg Addr --> 0x6d, Bit --> 3 */
#define BMI160_USER_SELFTEST_AMP__POS               3
#define BMI160_USER_SELFTEST_AMP__LEN               1
#define BMI160_USER_SELFTEST_AMP__MSK               0x08
#define BMI160_USER_SELFTEST_AMP__REG		BMI160_USER_SELF_TEST_ADDR

/* Self_Test Description - Reg Addr --> 0x6d, Bit --> 4 */
#define BMI160_USER_GYRO_SELFTEST_START__POS               4
#define BMI160_USER_GYRO_SELFTEST_START__LEN               1
#define BMI160_USER_GYRO_SELFTEST_START__MSK               0x10
#define BMI160_USER_GYRO_SELFTEST_START__REG		    \
BMI160_USER_SELF_TEST_ADDR

/* NV_CONF Description - Reg Addr --> 0x70, Bit --> 0 */
#define BMI160_USER_NV_CONFIG_SPI_ENABLE__POS               0
#define BMI160_USER_NV_CONFIG_SPI_ENABLE__LEN               1
#define BMI160_USER_NV_CONFIG_SPI_ENABLE__MSK               0x01
#define BMI160_USER_NV_CONFIG_SPI_ENABLE__REG	 BMI160_USER_NV_CONFIG_ADDR

/*IF_CONF Description - Reg Addr --> 0x70, Bit --> 1 */
#define BMI160_USER_IF_CONFIG_I2C_WDT_SELECT__POS               1
#define BMI160_USER_IF_CONFIG_I2C_WDT_SELECT__LEN               1
#define BMI160_USER_IF_CONFIG_I2C_WDT_SELECT__MSK               0x02
#define BMI160_USER_IF_CONFIG_I2C_WDT_SELECT__REG		\
BMI160_USER_NV_CONFIG_ADDR

/*IF_CONF Description - Reg Addr --> 0x70, Bit --> 2 */
#define BMI160_USER_IF_CONFIG_I2C_WDT_ENABLE__POS               2
#define BMI160_USER_IF_CONFIG_I2C_WDT_ENABLE__LEN               1
#define BMI160_USER_IF_CONFIG_I2C_WDT_ENABLE__MSK               0x04
#define BMI160_USER_IF_CONFIG_I2C_WDT_ENABLE__REG		\
BMI160_USER_NV_CONFIG_ADDR

/* NV_CONF Description - Reg Addr --> 0x70, Bit --> 3 */
#define BMI160_USER_NV_CONFIG_SPARE0__POS               3
#define BMI160_USER_NV_CONFIG_SPARE0__LEN               1
#define BMI160_USER_NV_CONFIG_SPARE0__MSK               0x08
#define BMI160_USER_NV_CONFIG_SPARE0__REG	BMI160_USER_NV_CONFIG_ADDR

/* NV_CONF Description - Reg Addr --> 0x70, Bit --> 4...7 */
#define BMI160_USER_NV_CONFIG_NVM_COUNTER__POS               4
#define BMI160_USER_NV_CONFIG_NVM_COUNTER__LEN               4
#define BMI160_USER_NV_CONFIG_NVM_COUNTER__MSK               0xF0
#define BMI160_USER_NV_CONFIG_NVM_COUNTER__REG	BMI160_USER_NV_CONFIG_ADDR

/* Offset_0 Description - Reg Addr --> 0x71, Bit --> 0...7 */
#define BMI160_USER_OFFSET_0_ACCEL_OFF_X__POS               0
#define BMI160_USER_OFFSET_0_ACCEL_OFF_X__LEN               8
#define BMI160_USER_OFFSET_0_ACCEL_OFF_X__MSK               0xFF
#define BMI160_USER_OFFSET_0_ACCEL_OFF_X__REG	BMI160_USER_OFFSET_0_ADDR

/* Offset_1 Description - Reg Addr --> 0x72, Bit --> 0...7 */
#define BMI160_USER_OFFSET_1_ACCEL_OFF_Y__POS               0
#define BMI160_USER_OFFSET_1_ACCEL_OFF_Y__LEN               8
#define BMI160_USER_OFFSET_1_ACCEL_OFF_Y__MSK               0xFF
#define BMI160_USER_OFFSET_1_ACCEL_OFF_Y__REG	BMI160_USER_OFFSET_1_ADDR

/* Offset_2 Description - Reg Addr --> 0x73, Bit --> 0...7 */
#define BMI160_USER_OFFSET_2_ACCEL_OFF_Z__POS               0
#define BMI160_USER_OFFSET_2_ACCEL_OFF_Z__LEN               8
#define BMI160_USER_OFFSET_2_ACCEL_OFF_Z__MSK               0xFF
#define BMI160_USER_OFFSET_2_ACCEL_OFF_Z__REG	BMI160_USER_OFFSET_2_ADDR

/* Offset_3 Description - Reg Addr --> 0x74, Bit --> 0...7 */
#define BMI160_USER_OFFSET_3_GYRO_OFF_X__POS               0
#define BMI160_USER_OFFSET_3_GYRO_OFF_X__LEN               8
#define BMI160_USER_OFFSET_3_GYRO_OFF_X__MSK               0xFF
#define BMI160_USER_OFFSET_3_GYRO_OFF_X__REG	BMI160_USER_OFFSET_3_ADDR

/* Offset_4 Description - Reg Addr --> 0x75, Bit --> 0...7 */
#define BMI160_USER_OFFSET_4_GYRO_OFF_Y__POS               0
#define BMI160_USER_OFFSET_4_GYRO_OFF_Y__LEN               8
#define BMI160_USER_OFFSET_4_GYRO_OFF_Y__MSK               0xFF
#define BMI160_USER_OFFSET_4_GYRO_OFF_Y__REG	BMI160_USER_OFFSET_4_ADDR

/* Offset_5 Description - Reg Addr --> 0x76, Bit --> 0...7 */
#define BMI160_USER_OFFSET_5_GYRO_OFF_Z__POS               0
#define BMI160_USER_OFFSET_5_GYRO_OFF_Z__LEN               8
#define BMI160_USER_OFFSET_5_GYRO_OFF_Z__MSK               0xFF
#define BMI160_USER_OFFSET_5_GYRO_OFF_Z__REG	BMI160_USER_OFFSET_5_ADDR


/* Offset_6 Description - Reg Addr --> 0x77, Bit --> 0..1 */
#define BMI160_USER_OFFSET_6_GYRO_OFF_X__POS               0
#define BMI160_USER_OFFSET_6_GYRO_OFF_X__LEN               2
#define BMI160_USER_OFFSET_6_GYRO_OFF_X__MSK               0x03
#define BMI160_USER_OFFSET_6_GYRO_OFF_X__REG	BMI160_USER_OFFSET_6_ADDR

/* Offset_6 Description - Reg Addr --> 0x77, Bit --> 2...3 */
#define BMI160_USER_OFFSET_6_GYRO_OFF_Y__POS               2
#define BMI160_USER_OFFSET_6_GYRO_OFF_Y__LEN               2
#define BMI160_USER_OFFSET_6_GYRO_OFF_Y__MSK               0x0C
#define BMI160_USER_OFFSET_6_GYRO_OFF_Y__REG	BMI160_USER_OFFSET_6_ADDR

/* Offset_6 Description - Reg Addr --> 0x77, Bit --> 4...5 */
#define BMI160_USER_OFFSET_6_GYRO_OFF_Z__POS               4
#define BMI160_USER_OFFSET_6_GYRO_OFF_Z__LEN               2
#define BMI160_USER_OFFSET_6_GYRO_OFF_Z__MSK               0x30
#define BMI160_USER_OFFSET_6_GYRO_OFF_Z__REG	 BMI160_USER_OFFSET_6_ADDR

/* Offset_6 Description - Reg Addr --> 0x77, Bit --> 6 */
#define BMI160_USER_OFFSET_6_ACCEL_OFF_ENABLE__POS               6
#define BMI160_USER_OFFSET_6_ACCEL_OFF_ENABLE__LEN               1
#define BMI160_USER_OFFSET_6_ACCEL_OFF_ENABLE__MSK               0x40
#define BMI160_USER_OFFSET_6_ACCEL_OFF_ENABLE__REG	 \
BMI160_USER_OFFSET_6_ADDR

/* Offset_6 Description - Reg Addr --> 0x77, Bit -->  7 */
#define BMI160_USER_OFFSET_6_GYRO_OFF_EN__POS               7
#define BMI160_USER_OFFSET_6_GYRO_OFF_EN__LEN               1
#define BMI160_USER_OFFSET_6_GYRO_OFF_EN__MSK               0x80
#define BMI160_USER_OFFSET_6_GYRO_OFF_EN__REG	 BMI160_USER_OFFSET_6_ADDR

/* STEP_CNT_0  Description - Reg Addr --> 0x78, Bit -->  0 to 7 */
#define BMI160_USER_STEP_COUNT_LSB__POS               0
#define BMI160_USER_STEP_COUNT_LSB__LEN               7
#define BMI160_USER_STEP_COUNT_LSB__MSK               0xFF
#define BMI160_USER_STEP_COUNT_LSB__REG	 BMI160_USER_STEP_COUNT_0_ADDR

/* STEP_CNT_1  Description - Reg Addr --> 0x79, Bit -->  0 to 7 */
#define BMI160_USER_STEP_COUNT_MSB__POS               0
#define BMI160_USER_STEP_COUNT_MSB__LEN               7
#define BMI160_USER_STEP_COUNT_MSB__MSK               0xFF
#define BMI160_USER_STEP_COUNT_MSB__REG	 BMI160_USER_STEP_COUNT_1_ADDR

/* STEP_CONFIG_0  Description - Reg Addr --> 0x7A, Bit -->  0 to 7 */
#define BMI160_USER_STEP_CONFIG_ZERO__POS               0
#define BMI160_USER_STEP_CONFIG_ZERO__LEN               7
#define BMI160_USER_STEP_CONFIG_ZERO__MSK               0xFF
#define BMI160_USER_STEP_CONFIG_ZERO__REG	 BMI160_USER_STEP_CONFIG_0_ADDR


/* STEP_CONFIG_1  Description - Reg Addr --> 0x7B, Bit -->  0 to 2 and
4 to 7 */
#define BMI160_USER_STEP_CONFIG_ONE_CNF1__POS               0
#define BMI160_USER_STEP_CONFIG_ONE_CNF1__LEN               3
#define BMI160_USER_STEP_CONFIG_ONE_CNF1__MSK               0x07
#define BMI160_USER_STEP_CONFIG_ONE_CNF1__REG	 BMI160_USER_STEP_CONFIG_1_ADDR

#define BMI160_USER_STEP_CONFIG_ONE_CNF2__POS               4
#define BMI160_USER_STEP_CONFIG_ONE_CNF2__LEN               4
#define BMI160_USER_STEP_CONFIG_ONE_CNF2__MSK               0xF0
#define BMI160_USER_STEP_CONFIG_ONE_CNF2__REG	 BMI160_USER_STEP_CONFIG_1_ADDR

/* STEP_CONFIG_1  Description - Reg Addr --> 0x7B, Bit -->  0 to 2 */
#define BMI160_USER_STEP_CONFIG_1_STEP_COUNT_ENABLE__POS		3
#define BMI160_USER_STEP_CONFIG_1_STEP_COUNT_ENABLE__LEN		1
#define BMI160_USER_STEP_CONFIG_1_STEP_COUNT_ENABLE__MSK		0x08
#define BMI160_USER_STEP_CONFIG_1_STEP_COUNT_ENABLE__REG	\
BMI160_USER_STEP_CONFIG_1_ADDR

/* USER REGISTERS DEFINITION END */

/* MACROS FOR SET / GET BITS */
#define BMI160_BITS_MSK(bitspos,bitslen) \
		(((0xff>>(8-bitslen))<<bitspos))
#define BMI160_GET_BITSLICE(regvar, bitspos, bitslen) \
			((regvar & BMI160_BITS_MSK(bitspos,bitslen)) >> bitspos)
#define BMI160_SET_BITSLICE(regvar, bitspos,bitslen, val)\
		((regvar & ~BMI160_BITS_MSK(bitspos,bitslen)) | ((val<<bitspos)&BMI160_BITS_MSK(bitspos,bitslen)))

/**************************************************/
/**\name	 FUNCTION DECLARATIONS  */
/*************************************************/
bmi160_ret_t bmi160_init(bmi160_t *bmi160_ptr);
bmi160_ret_t bmi160_check_connection();

bmi160_ret_t bmi160_read_gyro_xyz(bmi160_xyz_t *gyro);
bmi160_ret_t bmi160_read_acc_xyz(bmi160_xyz_t *acc);

bmi160_ret_t bmi160_read_imu_data(bmi160_xyz_t *acc,bmi160_xyz_t *gyro);

bmi160_ret_t bmi160_set_accel_output_datarate(uint8_t odr);
bmi160_ret_t bmi160_set_accel_bw(uint8_t bw);
bmi160_ret_t bmi160_set_accel_range(uint8_t range);

bmi160_ret_t bmi160_set_gyro_output_datarate(uint8_t odr);
bmi160_ret_t bmi160_set_gyro_bw(uint8_t bw);
bmi160_ret_t bmi160_set_gyro_range(uint8_t range);

bmi160_ret_t bmi160_set_command_register(uint8_t cmd);

bmi160_ret_t bmi160_read_reg(uint8_t reg_addr, uint8_t *reg);
bmi160_ret_t bmi160_write_reg(uint8_t reg_addr, uint8_t reg);


#endif

