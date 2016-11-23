/*!
****************************************************************************
* Copyright (C) 2015 - 2016 Bosch Sensortec GmbH
*
* bma2x2.c
* Date: 2016/03/11
* Revision: 2.0.4 $
*
* Usage: Sensor Driver for BMA2x2 sensor
*
****************************************************************************
* License:
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
/*! file <BMA2x2 >
    brief <Sensor driver for BMA2x2> */
#include "bma2x2.h"
/*! user defined code to be added here ... */
static bma2x2_t *p_bma2x2;
/*! Based on Bit resolution value_u8 should be modified */
uint8_t V_BMA2x2RESOLUTION_U8 = BMA2x2_12_RESOLUTION;

/*!
 *	@brief
 *	This function is used for initialize
 *	bus read and bus write functions
 *	assign the chip id and device address
 *	chip id is read in the register 0x00 bit from 0 to 7
 *
 *	@param bma2x2 : structure pointer
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *	@note
 *	While changing the parameter of the bma2x2_t
 *	consider the following point:
 *	Changing the reference value of the parameter
 *	will changes the local copy or local reference
 *	make sure your changes will not
 *	affect the reference value of the parameter
 *	(Better case don't change the reference value of the parameter)
 *
*/
bstdr_ret_t bma2x2_init(bma2x2_t *bma2x2)
{
	/*  Variable used to return value of
	communication routine*/
	bstdr_ret_t com_rslt = BSTDR_E_GEN_ERROR;
	uint8_t data_u8 = 0;
	uint8_t config_data_u8 = 0;
	/* assign bma2x2 ptr */
	p_bma2x2 = bma2x2;
	/* read Chip Id */
	com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr,BMA2x2_CHIP_ID_REG, &data_u8, 1);
	p_bma2x2->chip_id = data_u8;    /* get bit slice */
	/* read the fifo config register and update	the value to the fifo_config*/
	com_rslt += bma2x2_read_reg(BMA2x2_FIFO_MODE_REG,&config_data_u8, 1);
	p_bma2x2->fifo_config = config_data_u8;
	return com_rslt;
}

/*!
 *	@brief checks connection (chip id read out)
*/
bstdr_ret_t bma2x2_check_connection(void)
{
	/*  Variable used to return value of communication routine*/
	bstdr_ret_t com_rslt = BSTDR_E_GEN_ERROR;
	uint8_t data_u8 = 0;
	/* read Chip Id */
	com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr,BMA2x2_CHIP_ID_REG, &data_u8, 1);
	if(data_u8 == 0xFA){
		com_rslt = BSTDR_OK;
	}
	return com_rslt;
}
/*!
 *	@brief This API reads acceleration data X,Y,Z values
 *	from location 02h to 07h
 *
 *  @param accel : pointer holding the data of accel
 *		       value       |   resolution
 *       ----------------- | --------------
 *              0          | BMA2x2_12_RESOLUTION
 *              1          | BMA2x2_10_RESOLUTION
 *              2          | BMA2x2_14_RESOLUTION
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
bstdr_ret_t bma2x2_read_accel_xyz(bma2x2_xyz_t *accel)
{
	/*  Variable used to return value of
	communication routine*/
	bstdr_ret_t com_rslt = BSTDR_E_GEN_ERROR;

	uint8_t data_u8[6] = {0};

	if (p_bma2x2 == BMA2x2_NULL) {
		/* Check the struct p_bma2x2 is empty */
		return BSTDR_E_NULL_PTR;
		} else {
		switch (V_BMA2x2RESOLUTION_U8) {
		/* This case used for the resolution bit 12*/
		case BMA2x2_12_RESOLUTION:
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, BMA2x2_ACCEL_X12_LSB_REG,	data_u8, 6);
			/* read the x data_u8*/
			accel->x = (int16_t)((((int32_t)((int8_t)data_u8[BMA2x2_SENSOR_DATA_XYZ_X_MSB]))<< BMA2x2_SHIFT_EIGHT_BITS) |
			(data_u8[BMA2x2_SENSOR_DATA_XYZ_X_LSB] & BMA2x2_12_BIT_SHIFT));
			accel->x = accel->x >> BMA2x2_SHIFT_FOUR_BITS;

			/* read the y data_u8*/
			accel->y = (int16_t)((((int32_t)((int8_t)
			data_u8[BMA2x2_SENSOR_DATA_XYZ_Y_MSB]))
			<< BMA2x2_SHIFT_EIGHT_BITS) |
			(data_u8[BMA2x2_SENSOR_DATA_XYZ_Y_LSB] &
			BMA2x2_12_BIT_SHIFT));
			accel->y = accel->y >> BMA2x2_SHIFT_FOUR_BITS;

			/* read the z data_u8*/
			accel->z = (int16_t)((((int32_t)((int8_t)
			data_u8[BMA2x2_SENSOR_DATA_XYZ_Z_MSB]))
			<< BMA2x2_SHIFT_EIGHT_BITS) |
			(data_u8[BMA2x2_SENSOR_DATA_XYZ_Z_LSB] &
			BMA2x2_12_BIT_SHIFT));
			accel->z = accel->z >> BMA2x2_SHIFT_FOUR_BITS;

		break;
		case BMA2x2_10_RESOLUTION:
		/* This case used for the resolution bit 10*/
			com_rslt = p_bma2x2->bus_read
			(p_bma2x2->dev_addr, BMA2x2_ACCEL_X10_LSB_REG,
			data_u8, BMA2x2_SHIFT_SIX_BITS);
			/* read the x data_u8*/
			accel->x = (int16_t)((((int32_t)((int8_t)
			data_u8[BMA2x2_SENSOR_DATA_XYZ_X_MSB]))
			<< BMA2x2_SHIFT_EIGHT_BITS) |
			(data_u8[BMA2x2_SENSOR_DATA_XYZ_X_LSB] &
			BMA2x2_10_BIT_SHIFT));
			accel->x = accel->x >> BMA2x2_SHIFT_SIX_BITS;

			/* read the y data_u8*/
			accel->y = (int16_t)((((int32_t)((int8_t)
			data_u8[BMA2x2_SENSOR_DATA_XYZ_Y_MSB]))
			<< BMA2x2_SHIFT_EIGHT_BITS) |
			(data_u8[BMA2x2_SENSOR_DATA_XYZ_Y_LSB] &
			BMA2x2_10_BIT_SHIFT));
			accel->y = accel->y >> BMA2x2_SHIFT_SIX_BITS;

			/* read the z data_u8*/
			accel->z = (int16_t)((((int32_t)((int8_t)
			data_u8[BMA2x2_SENSOR_DATA_XYZ_Z_MSB]))
			<< BMA2x2_SHIFT_EIGHT_BITS) |
			(data_u8[BMA2x2_SENSOR_DATA_XYZ_Z_LSB]
			& BMA2x2_10_BIT_SHIFT));
			accel->z = accel->z >> BMA2x2_SHIFT_SIX_BITS;
		break;
		/* This case used for the resolution bit 14*/
		case BMA2x2_14_RESOLUTION:
			com_rslt = p_bma2x2->bus_read
			(p_bma2x2->dev_addr, BMA2x2_ACCEL_X14_LSB_REG,
			data_u8, BMA2x2_SHIFT_SIX_BITS);

			/* read the x data_u8*/
			accel->x = (int16_t)((((int32_t)((int8_t)
			data_u8[BMA2x2_SENSOR_DATA_XYZ_X_MSB]))<<
			BMA2x2_SHIFT_EIGHT_BITS) |
			(data_u8[BMA2x2_SENSOR_DATA_XYZ_X_LSB]
			& BMA2x2_14_BIT_SHIFT));
			accel->x = accel->x >> BMA2x2_SHIFT_TWO_BITS;

			/* read the y data_u8*/
			accel->y = (int16_t)((((int32_t)((int8_t)
			data_u8[BMA2x2_SENSOR_DATA_XYZ_Y_MSB]))<<
			BMA2x2_SHIFT_EIGHT_BITS) |
			(data_u8[BMA2x2_SENSOR_DATA_XYZ_Y_LSB]
			& BMA2x2_14_BIT_SHIFT));
			accel->y = accel->y >> BMA2x2_SHIFT_TWO_BITS;

			/* read the z data_u8*/
			accel->z = (int16_t)((((int32_t)((int8_t)
			data_u8[BMA2x2_SENSOR_DATA_XYZ_Z_MSB]))<<
			BMA2x2_SHIFT_EIGHT_BITS) |
			(data_u8[BMA2x2_SENSOR_DATA_XYZ_Z_LSB]
			& BMA2x2_14_BIT_SHIFT));
			accel->z = accel->z >> BMA2x2_SHIFT_TWO_BITS;
		break;
		default:
		break;
		}
	}
	return com_rslt;
}

/*!
 * @brief This API is used to get the ranges(g values) of the sensor
 *	in the register 0x0F bit from 0 to 3
 *
 *
 *	@param range_u8 : The value of range
 *		  range_u8       |   result
 *       ----------------- | --------------
 *              0x03       | BMA2x2_RANGE_2G
 *              0x05       | BMA2x2_RANGE_4G
 *              0x08       | BMA2x2_RANGE_8G
 *              0x0C       | BMA2x2_RANGE_16G
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
bstdr_ret_t bma2x2_get_range(uint8_t *range_u8)
{
	/*  Variable used to return value of
	communication routine*/
	bstdr_ret_t com_rslt = BSTDR_E_GEN_ERROR;
	uint8_t data_u8 = 0;

	if (p_bma2x2 == BMA2x2_NULL) {
		/* Check the struct p_bma2x2 is empty */
		return BSTDR_E_NULL_PTR;
		} else {
		/* Read the range register 0x0F*/
		com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr,
		BMA2x2_RANGE_SELECT_REG, &data_u8,
		1);
		data_u8 = BMA2x2_GET_BITSLICE(data_u8, BMA2x2_RANGE_SELECT);
		*range_u8 = data_u8;
	}
	return com_rslt;
}
/*!
 * @brief This API is used to set the ranges(g values) of the sensor
 *	in the register 0x0F bit from 0 to 3
 *
 *
 *	@param range_u8 : The value of range
 *		  range_u8 |   result
 *       ----------------- | --------------
 *              0x03       | BMA2x2_RANGE_2G
 *              0x05       | BMA2x2_RANGE_4G
 *              0x08       | BMA2x2_RANGE_8G
 *              0x0C       | BMA2x2_RANGE_16G
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
bstdr_ret_t bma2x2_set_range(uint8_t range_u8)
{
	/*  Variable used to return value of
	communication routine*/
	bstdr_ret_t com_rslt = BSTDR_E_GEN_ERROR;
	uint8_t data_u8 = 0;

	if (p_bma2x2 == BMA2x2_NULL) {
		/* Check the struct p_bma2x2 is empty */
		return BSTDR_E_NULL_PTR;
		} else {
		if ((range_u8 == BMA2x2_RANGE_2G) ||
		(range_u8 == BMA2x2_RANGE_4G) ||
		(range_u8 == BMA2x2_RANGE_8G) ||
		(range_u8 == BMA2x2_RANGE_16G)) {
			com_rslt = p_bma2x2->bus_read
			(p_bma2x2->dev_addr,
			BMA2x2_RANGE_SELECT_REG, &data_u8,
			1);
			switch (range_u8) {
			case BMA2x2_RANGE_2G:
				data_u8  = BMA2x2_SET_BITSLICE(data_u8,
				BMA2x2_RANGE_SELECT,
				BMA2x2_RANGE_2G);
			break;
			case BMA2x2_RANGE_4G:
				data_u8  = BMA2x2_SET_BITSLICE(data_u8,
				BMA2x2_RANGE_SELECT,
				BMA2x2_RANGE_4G);
			break;
			case BMA2x2_RANGE_8G:
				data_u8  = BMA2x2_SET_BITSLICE(data_u8,
				BMA2x2_RANGE_SELECT,
				BMA2x2_RANGE_8G);
			break;
			case BMA2x2_RANGE_16G:
				data_u8  = BMA2x2_SET_BITSLICE(data_u8,
				BMA2x2_RANGE_SELECT,
				BMA2x2_RANGE_16G);
			break;
			default:
			break;
			}
			/* Write the range register 0x0F*/
			com_rslt += p_bma2x2->bus_write
			(p_bma2x2->dev_addr,
			BMA2x2_RANGE_SELECT_REG, &data_u8,
			1);
		} else {
		com_rslt = BSTDR_E_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to get the bandwidth of the sensor in the register
 *	0x10 bit from 0 to 4
 *
 *
 *  @param bw_u8 : The value of bandwidth
 *		  bw_u8          |   result
 *       ----------------- | --------------
 *              0x08       | BMA2x2_BW_7_81HZ
 *              0x09       | BMA2x2_BW_15_63HZ
 *              0x0A       | BMA2x2_BW_31_25HZ
 *              0x0B       | BMA2x2_BW_62_50HZ
 *              0x0C       | BMA2x2_BW_125HZ
 *              0x0D       | BMA2x2_BW_250HZ
 *              0x0E       | BMA2x2_BW_500HZ
 *              0x0F       | BMA2x2_BW_1000HZ
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
bstdr_ret_t bma2x2_get_bw(uint8_t *bw_u8)
{
	/*  Variable used to return value of
	communication routine*/
	bstdr_ret_t com_rslt = BSTDR_E_GEN_ERROR;
	uint8_t data_u8 = 0;

	if (p_bma2x2 == BMA2x2_NULL) {
		/* Check the struct p_bma2x2 is empty */
		return BSTDR_E_NULL_PTR;
		} else {
			/* Read the bandwidth register 0x10*/
			com_rslt = p_bma2x2->bus_read
			(p_bma2x2->dev_addr,
			BMA2x2_BW_REG, &data_u8,
			1);
			data_u8 = BMA2x2_GET_BITSLICE(data_u8, BMA2x2_BW);
			*bw_u8 = data_u8;
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to set the bandwidth of the sensor
 *      in the register
 *	0x10 bit from 0 to 4
 *
 *
 *  @param bw_u8 : The value of bandwidth
 *		  bw_u8          |   result
 *       ----------------- | --------------
 *              0x08       | BMA2x2_BW_7_81HZ
 *              0x09       | BMA2x2_BW_15_63HZ
 *              0x0A       | BMA2x2_BW_31_25HZ
 *              0x0B       | BMA2x2_BW_62_50HZ
 *              0x0C       | BMA2x2_BW_125HZ
 *              0x0D       | BMA2x2_BW_250HZ
 *              0x0E       | BMA2x2_BW_500HZ
 *              0x0F       | BMA2x2_BW_1000HZ
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
bstdr_ret_t bma2x2_set_bw(uint8_t bw_u8)
{
/*  Variable used to return value of
communication routine*/
bstdr_ret_t com_rslt = BSTDR_E_GEN_ERROR;
uint8_t data_u8 = 0;
uint8_t data_bw_u8 = 0;
if (p_bma2x2 == BMA2x2_NULL) {
		/* Check the struct p_bma2x2 is empty */
		com_rslt = BSTDR_E_NULL_PTR;
	} else {
	/* Check the chip id 0xFB, it support upto 500Hz*/
	if (p_bma2x2->chip_id == BANDWIDTH_DEFINE) {
		if (bw_u8 > BMA2x2_ACCEL_BW_MIN_RANGE &&
		bw_u8 < BMA2x2_ACCEL_BW_1000HZ_RANGE) {
			switch (bw_u8) {
			case BMA2x2_BW_7_81HZ:
				data_bw_u8 = BMA2x2_BW_7_81HZ;

				/*  7.81 Hz      64000 uS   */
			break;
			case BMA2x2_BW_15_63HZ:
				data_bw_u8 = BMA2x2_BW_15_63HZ;

			/*  15.63 Hz     32000 uS   */
			break;
			case BMA2x2_BW_31_25HZ:
				data_bw_u8 = BMA2x2_BW_31_25HZ;

			/*  31.25 Hz     16000 uS   */
			break;
			case BMA2x2_BW_62_50HZ:
				data_bw_u8 = BMA2x2_BW_62_50HZ;

			/*  62.50 Hz     8000 uS   */
			break;
			case BMA2x2_BW_125HZ:
				data_bw_u8 = BMA2x2_BW_125HZ;

			/*  125 Hz       4000 uS   */
			break;
			case BMA2x2_BW_250HZ:
				data_bw_u8 = BMA2x2_BW_250HZ;

			/*  250 Hz       2000 uS   */
			break;
			case BMA2x2_BW_500HZ:
				data_bw_u8 = BMA2x2_BW_500HZ;

			/*  500 Hz       1000 uS   */
			break;
			default:
			break;
			}
			/* Write the bandwidth register */
			com_rslt = p_bma2x2->bus_read
			(p_bma2x2->dev_addr,
			BMA2x2_BW_REG, &data_u8,
			1);
			data_u8 = BMA2x2_SET_BITSLICE(data_u8,
			BMA2x2_BW, data_bw_u8);
			com_rslt += p_bma2x2->bus_write
			(p_bma2x2->dev_addr,
			BMA2x2_BW_REG, &data_u8,
			1);
			} else {
			com_rslt = BSTDR_E_OUT_OF_RANGE;
			}
		} else {
		if (bw_u8 > BMA2x2_ACCEL_BW_MIN_RANGE &&
		bw_u8 < BMA2x2_ACCEL_BW_MAX_RANGE) {
			switch (bw_u8) {
			case BMA2x2_BW_7_81HZ:
				data_bw_u8 = BMA2x2_BW_7_81HZ;

			/*  7.81 Hz      64000 uS   */
			break;
			case BMA2x2_BW_15_63HZ:
				data_bw_u8 = BMA2x2_BW_15_63HZ;

			/*  15.63 Hz     32000 uS   */
			break;
			case BMA2x2_BW_31_25HZ:
				data_bw_u8 = BMA2x2_BW_31_25HZ;

			/*  31.25 Hz     16000 uS   */
			break;
			case BMA2x2_BW_62_50HZ:
				data_bw_u8 = BMA2x2_BW_62_50HZ;

			/*  62.50 Hz     8000 uS   */
			break;
			case BMA2x2_BW_125HZ:
				data_bw_u8 = BMA2x2_BW_125HZ;

			/*  125 Hz       4000 uS   */
			break;
			case BMA2x2_BW_250HZ:
				data_bw_u8 = BMA2x2_BW_250HZ;

			/*  250 Hz       2000 uS   */
			break;
			case BMA2x2_BW_500HZ:
				data_bw_u8 = BMA2x2_BW_500HZ;

			/*!  500 Hz       1000 uS   */
			break;
			case BMA2x2_BW_1000HZ:
				data_bw_u8 = BMA2x2_BW_1000HZ;

			/*  1000 Hz      500 uS   */
			break;
			default:
			break;
			}
			/* Write the bandwidth register */
			com_rslt = p_bma2x2->bus_read
			(p_bma2x2->dev_addr,
			BMA2x2_BW_REG, &data_u8,
			1);
			data_u8 = BMA2x2_SET_BITSLICE
			(data_u8, BMA2x2_BW, data_bw_u8);
			com_rslt += p_bma2x2->bus_write
			(p_bma2x2->dev_addr,
			BMA2x2_BW_REG, &data_u8,
			1);
			} else {
			com_rslt = BSTDR_E_OUT_OF_RANGE;
			}
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to get the operating
 *	modes of the sensor in the register 0x11 and 0x12
 *	@note Register 0x11 - bit from 5 to 7
 *	@note Register 0x12 - bit from 5 and 6
 *
 *
 *  @param power_mode_u8 : The value of power mode
 *	power_mode_u8           |value  |   0x11  |   0x12
 *  ------------------------- |-------| --------|--------
 *  BMA2x2_MODE_NORMAL        |  0    |  0x00   |  0x00
 *  BMA2x2_MODE_LOWPOWER1     |  1    |  0x02   |  0x00
 *  BMA2x2_MODE_SUSPEND       |  2    |  0x06   |  0x00
 *  BMA2x2_MODE_DEEP_SUSPEND  |  3    |  0x01   |  0x00
 *  BMA2x2_MODE_LOWPOWER2     |  4    |  0x02   |  0x01
 *  BMA2x2_MODE_STANDBY       |  5    |  0x04   |  0x00
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
bstdr_ret_t bma2x2_get_power_mode(
uint8_t *power_mode_u8)
{
	/*  Variable used to return value of
	communication routine*/
bstdr_ret_t com_rslt = BSTDR_E_GEN_ERROR;
uint8_t data_u8 = 0;
uint8_t data2_u8 = 0;
if (p_bma2x2 == BMA2x2_NULL) {
	/* Check the struct p_bma2x2 is empty */
		com_rslt = BSTDR_E_NULL_PTR;
	} else {
		com_rslt = p_bma2x2->bus_read
		(p_bma2x2->dev_addr, BMA2x2_MODE_CTRL_REG,
		&data_u8, 1);
		com_rslt += p_bma2x2->bus_read
		(p_bma2x2->dev_addr, BMA2x2_LOW_NOISE_CTRL_ADDR,
		&data2_u8, 1);

		data_u8  = (data_u8 &
		BMA2x2_POWER_MODE_HEX_E_ZERO_MASK) >>
		BMA2x2_SHIFT_FIVE_BITS;
		data2_u8  = (data2_u8 &
		BMA2x2_POWER_MODE_HEX_4_ZERO_MASK) >>
		BMA2x2_SHIFT_SIX_BITS;

	if ((data_u8 ==
	BMA2x2_POWER_MODE_HEX_ZERO_ZERO_MASK) &&
	(data2_u8 ==
	BMA2x2_POWER_MODE_HEX_ZERO_ZERO_MASK)) {
		*power_mode_u8  = BMA2x2_MODE_NORMAL;
		} else {
		if ((data_u8 ==
		BMA2x2_POWER_MODE_HEX_ZERO_TWO_MASK) &&
		(data2_u8 ==
		BMA2x2_POWER_MODE_HEX_ZERO_ZERO_MASK)) {
			*power_mode_u8  =
			BMA2x2_MODE_LOWPOWER1;
			} else {
			if ((data_u8 ==
			BMA2x2_POWER_MODE_HEX_ZERO_FOUR_MASK
			|| data_u8 ==
			BMA2x2_POWER_MODE_HEX_ZERO_SIX_MASK) &&
			(data2_u8 ==
			BMA2x2_POWER_MODE_HEX_ZERO_ZERO_MASK)) {
				*power_mode_u8  =
				BMA2x2_MODE_SUSPEND;
				} else {
				if (((data_u8 &
				BMA2x2_POWER_MODE_HEX_ZERO_ONE_MASK)
				== BMA2x2_POWER_MODE_HEX_ZERO_ONE_MASK)) {
					*power_mode_u8  =
					BMA2x2_MODE_DEEP_SUSPEND;
					} else {
					if ((data_u8 ==
					BMA2x2_POWER_MODE_HEX_ZERO_TWO_MASK)
					&& (data2_u8 ==
					BMA2x2_POWER_MODE_HEX_ZERO_ONE_MASK)) {
						*power_mode_u8  =
						BMA2x2_MODE_LOWPOWER2;
					} else {
					if ((data_u8 ==
					BMA2x2_POWER_MODE_HEX_ZERO_FOUR_MASK) &&
					(data2_u8 ==
					BMA2x2_POWER_MODE_HEX_ZERO_ONE_MASK))
						*power_mode_u8  =
							BMA2x2_MODE_STANDBY;
					else
						*power_mode_u8 =
						BMA2x2_MODE_DEEP_SUSPEND;
						}
					}
				}
			}
		}
	}
	p_bma2x2->power_mode_u8 = *power_mode_u8;
return com_rslt;
}
/*!
 *	@brief This API is used to set the operating
 *	modes of the sensor in the register 0x11 and 0x12
 *	@note Register 0x11 - bit from 5 to 7
 *	@note Register 0x12 - bit from 5 and 6
 *
 *
 *  @param power_mode_u8 : The value of power mode
 *	power_mode_u8           |value  |   0x11  |   0x12
 *  ------------------------- |-------| --------|--------
 *  BMA2x2_MODE_NORMAL        |  0    |  0x00   |  0x00
 *  BMA2x2_MODE_LOWPOWER1     |  1    |  0x02   |  0x00
 *  BMA2x2_MODE_SUSPEND       |  2    |  0x06   |  0x00
 *  BMA2x2_MODE_DEEP_SUSPEND  |  3    |  0x01   |  0x00
 *  BMA2x2_MODE_LOWPOWER2     |  4    |  0x02   |  0x01
 *  BMA2x2_MODE_STANDBY       |  5    |  0x04   |  0x00
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
bstdr_ret_t bma2x2_set_power_mode(uint8_t power_mode_u8)
{
		/*  Variable used to return value of
	communication routine*/
	bstdr_ret_t com_rslt = BSTDR_E_GEN_ERROR;
	uint8_t mode_ctr_eleven_reg = 0;
	uint8_t mode_ctr_twel_reg = 0;
	uint8_t data_u8 = 0;
	uint8_t data2_u8 = 0;
	uint8_t pre_fifo_config_data = 0;

	if (p_bma2x2 == BMA2x2_NULL) {
		/* Check the struct p_bma2x2 is empty */
		com_rslt = BSTDR_E_NULL_PTR;
	} else {
		com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr,BMA2x2_MODE_CTRL_REG,&data_u8, 1);
		com_rslt += p_bma2x2->bus_read(p_bma2x2->dev_addr,BMA2x2_LOW_POWER_MODE_REG,&data2_u8, 1);

		if (power_mode_u8 < BMA2x2_POWER_MODE_RANGE) {
				switch (power_mode_u8)	{
				case BMA2x2_MODE_NORMAL:
					p_bma2x2->ctrl_mode_reg =
					BMA2x2_POWER_MODE_HEX_ZERO_ZERO_MASK;
					p_bma2x2->low_mode_reg =
					BMA2x2_POWER_MODE_HEX_ZERO_ZERO_MASK;
				break;
				case BMA2x2_MODE_LOWPOWER1:
					p_bma2x2->ctrl_mode_reg =
					BMA2x2_POWER_MODE_HEX_ZERO_TWO_MASK;
					p_bma2x2->low_mode_reg =
					BMA2x2_POWER_MODE_HEX_ZERO_ZERO_MASK;
				break;
				case BMA2x2_MODE_LOWPOWER2:
					p_bma2x2->ctrl_mode_reg =
					BMA2x2_POWER_MODE_HEX_ZERO_TWO_MASK;
					p_bma2x2->low_mode_reg =
					BMA2x2_POWER_MODE_HEX_ZERO_ONE_MASK;
				break;
				case BMA2x2_MODE_SUSPEND:
					p_bma2x2->ctrl_mode_reg =
					BMA2x2_POWER_MODE_HEX_ZERO_FOUR_MASK;
					p_bma2x2->low_mode_reg =
					BMA2x2_POWER_MODE_HEX_ZERO_ZERO_MASK;
				break;
				case BMA2x2_MODE_STANDBY:
					p_bma2x2->ctrl_mode_reg =
					BMA2x2_POWER_MODE_HEX_ZERO_FOUR_MASK;
					p_bma2x2->low_mode_reg =
					BMA2x2_POWER_MODE_HEX_ZERO_ONE_MASK;
				break;
				case BMA2x2_MODE_DEEP_SUSPEND:
					p_bma2x2->ctrl_mode_reg =
					BMA2x2_POWER_MODE_HEX_ZERO_ONE_MASK;
				break;
				}
		}
		mode_ctr_eleven_reg = p_bma2x2->ctrl_mode_reg;
		mode_ctr_twel_reg =  p_bma2x2->low_mode_reg;
		/* write the power mode to
		register 0x12*/
		data2_u8  = BMA2x2_SET_BITSLICE
		(data2_u8, BMA2x2_LOW_POWER_MODE,
		mode_ctr_twel_reg);
		com_rslt += p_bma2x2->bus_write
		(p_bma2x2->dev_addr, BMA2x2_LOW_POWER_MODE_REG,
		&data2_u8, 1);
		/*A minimum delay of
		atleast 450us is required for
		the low power modes,
		as per the data sheet.*/
		p_bma2x2->delay(1);
		/* Enter the power mode to suspend*/
		data_u8  = BMA2x2_SET_BITSLICE
		(data_u8, BMA2x2_MODE_CTRL,
		BMA2x2_SHIFT_FOUR_BITS);
		/* write the power mode to suspend*/
		com_rslt += p_bma2x2->bus_write
		(p_bma2x2->dev_addr, BMA2x2_MODE_CTRL_REG,
		&data_u8, 1);
		/*A minimum delay of
		atleast 450us is required for
		the low power modes,
		as per the data sheet.*/
		p_bma2x2->delay(1);
		/* write the previous FIFO mode and data select*/
		pre_fifo_config_data = p_bma2x2->fifo_config;
		com_rslt += bma2x2_write_reg(BMA2x2_FIFO_MODE_REG,
		&pre_fifo_config_data, 1);
		/*A minimum delay of
		atleast 450us is required for
		the low power modes,
		as per the data sheet.*/
		p_bma2x2->delay(1);
		com_rslt += p_bma2x2->bus_read
		(p_bma2x2->dev_addr,
		BMA2x2_MODE_CTRL_REG,
		&data_u8, 1);
		/* write the power mode to 11th register*/
		data_u8  = BMA2x2_SET_BITSLICE
		(data_u8, BMA2x2_MODE_CTRL,
		mode_ctr_eleven_reg);
		com_rslt += p_bma2x2->bus_write
		(p_bma2x2->dev_addr, BMA2x2_MODE_CTRL_REG,
		&data_u8, 1);
		/*A minimum delay of
		atleast 450us is required for
		the low power modes,
		as per the data sheet.*/
		p_bma2x2->delay(1);
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to assign the power mode values
 *	modes of the sensor in the register 0x11 and 0x12
 *	@note Register 0x11 - bit from 5 to 7
 *	@note Register 0x12 - bit from 5 and 6
 *
 *
 *  @param power_mode_u8 : The value of power mode
 *	power_mode_u8           |value  |   0x11  |   0x12
 *  ------------------------- |-------| --------|--------
 *  BMA2x2_MODE_NORMAL        |  0    |  0x00   |  0x00
 *  BMA2x2_MODE_LOWPOWER1     |  1    |  0x02   |  0x00
 *  BMA2x2_MODE_SUSPEND       |  2    |  0x06   |  0x00
 *  BMA2x2_MODE_DEEP_SUSPEND  |  3    |  0x01   |  0x00
 *  BMA2x2_MODE_LOWPOWER2     |  4    |  0x02   |  0x01
 *  BMA2x2_MODE_STANDBY       |  5    |  0x04   |  0x00
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
bstdr_ret_t bma2x2_set_mode_value(uint8_t power_mode_u8)
{
	bstdr_ret_t com_rslt = BSTDR_OK;

	if (p_bma2x2 == BMA2x2_NULL) {
		/* Check the struct p_bma2x2 is empty */
		com_rslt = BSTDR_E_NULL_PTR;
	} else {
	if (power_mode_u8 < BMA2x2_POWER_MODE_RANGE) {
		switch (power_mode_u8)	{
		case BMA2x2_MODE_NORMAL:
			p_bma2x2->ctrl_mode_reg =
			BMA2x2_POWER_MODE_HEX_ZERO_ZERO_MASK;
			p_bma2x2->low_mode_reg =
			BMA2x2_POWER_MODE_HEX_ZERO_ZERO_MASK;
		break;
		case BMA2x2_MODE_LOWPOWER1:
			p_bma2x2->ctrl_mode_reg =
			BMA2x2_POWER_MODE_HEX_ZERO_TWO_MASK;
			p_bma2x2->low_mode_reg =
			BMA2x2_POWER_MODE_HEX_ZERO_ZERO_MASK;
		break;
		case BMA2x2_MODE_LOWPOWER2:
			p_bma2x2->ctrl_mode_reg =
			BMA2x2_POWER_MODE_HEX_ZERO_TWO_MASK;
			p_bma2x2->low_mode_reg =
			BMA2x2_POWER_MODE_HEX_ZERO_ONE_MASK;
		break;
		case BMA2x2_MODE_SUSPEND:
			p_bma2x2->ctrl_mode_reg =
			BMA2x2_POWER_MODE_HEX_ZERO_FOUR_MASK;
			p_bma2x2->low_mode_reg =
			BMA2x2_POWER_MODE_HEX_ZERO_ZERO_MASK;
		break;
		case BMA2x2_MODE_STANDBY:
			p_bma2x2->ctrl_mode_reg =
			BMA2x2_POWER_MODE_HEX_ZERO_FOUR_MASK;
			p_bma2x2->low_mode_reg =
			BMA2x2_POWER_MODE_HEX_ZERO_ONE_MASK;
		break;
		case BMA2x2_MODE_DEEP_SUSPEND:
			p_bma2x2->ctrl_mode_reg =
			BMA2x2_POWER_MODE_HEX_ZERO_ONE_MASK;
		break;
		}
		} else {
			com_rslt = BSTDR_E_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This function is used for the soft reset
 *	The soft reset register will be written
 *	with 0xB6 in the register 0x14.
 *
 *
 *
 *  \param : None
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
bstdr_ret_t bma2x2_soft_rst(void){
	/*  Variable used to return value of
	communication routine*/
	bstdr_ret_t com_rslt = BSTDR_E_GEN_ERROR;
	uint8_t data_u8 = BMA2x2_ENABLE_SOFT_RESET_VALUE;

	if (p_bma2x2 == BMA2x2_NULL) {
		/* Check the struct p_bma2x2 is empty */
		return BSTDR_E_NULL_PTR;
		}  else {
			/*! To reset the sensor
			0xB6 value_u8 will be written */
			com_rslt = p_bma2x2->bus_write
			(p_bma2x2->dev_addr, BMA2x2_RST_ADDR,
			&data_u8, 1);
		}
	return com_rslt;
}

/*!
 *	@brief This API is for to get
 *	the self test axis(self_test_axis) in the register ox32 bit 0 to 2
 *
 *
 *
 *  @param selftest_axis_u8 : The value of selftest axis
 *     selftest_axis_u8     |    result
 *  ------------------------- |------------------
 *     0x00                   | self test disable
 *     0x01                   | x-axis
 *     0x02                   | y-axis
 *     0x03                   | z-axis
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
bstdr_ret_t bma2x2_get_selftest_axis(uint8_t *selftest_axis_u8)
{
	uint8_t data_u8 = 0;
	/*  Variable used to return value of
	communication routine*/
	bstdr_ret_t com_rslt = BSTDR_E_GEN_ERROR;

	if (p_bma2x2 == BMA2x2_NULL) {
		/* Check the struct p_bma2x2 is empty */
		return BSTDR_E_NULL_PTR;
		} else {
			/* read the self test axis*/
			com_rslt = p_bma2x2->bus_read
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_SELFTEST_REG,
			&data_u8, 1);
			*selftest_axis_u8 = BMA2x2_GET_BITSLICE
			(data_u8, BMA2x2_ENABLE_SELFTEST);
		}
	return com_rslt;
}
/*!
 *	@brief This API is for to set
 *	the self test axis(self_test_axis) in the register ox32 bit 0 to 2
 *
 *
 *
 *  @param selftest_axis_u8 : The value of selftest axis
 *     selftest_axis_u8     |    result
 *  ------------------------- |------------------
 *     0x00                   | self test disable
 *     0x01                   | x-axis
 *     0x02                   | y-axis
 *     0x03                   | z-axis
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
bstdr_ret_t bma2x2_set_selftest_axis(uint8_t selftest_axis_u8)
{
	uint8_t data_u8 = 0;
	/*  Variable used to return value of
	communication routine*/
	bstdr_ret_t com_rslt = BSTDR_E_GEN_ERROR;

	if (p_bma2x2 == BMA2x2_NULL) {
		/* Check the struct p_bma2x2 is empty */
		return BSTDR_E_NULL_PTR;
		} else {
		if (selftest_axis_u8 < BMA2x2_SELF_TEST_AXIS_RANGE) {
			/* write the self test axis*/
			com_rslt = p_bma2x2->bus_read
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_SELFTEST_REG,
			&data_u8, 1);
			data_u8 = BMA2x2_SET_BITSLICE
			(data_u8, BMA2x2_ENABLE_SELFTEST, selftest_axis_u8);
			com_rslt += p_bma2x2->bus_write
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_SELFTEST_REG,
			&data_u8, 1);
		 } else {
		com_rslt = BSTDR_E_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API is for to get
 *	the Self Test sign(selftest_sign) in the register 0x32 bit 2
 *
 *
 *
 *  @param selftest_sign_u8 : The value of self test sign
 *     selftest_sign_u8     |    result
 *  ------------------------- |------------------
 *     0x00                   | negative sign
 *     0x01                   | positive sign
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
bstdr_ret_t bma2x2_get_selftest_sign(uint8_t *selftest_sign_u8){
	uint8_t data_u8 = 0;
	/*  Variable used to return value of
	communication routine*/
	bstdr_ret_t com_rslt = BSTDR_E_GEN_ERROR;

	if (p_bma2x2 == BMA2x2_NULL) {
		/* Check the struct p_bma2x2 is empty */
		return BSTDR_E_NULL_PTR;
		} else {
			/* read self test sign */
			com_rslt = p_bma2x2->bus_read
			(p_bma2x2->dev_addr,
			BMA2x2_NEG_SELFTEST_REG,
			&data_u8, 1);
			*selftest_sign_u8 = BMA2x2_GET_BITSLICE
			(data_u8, BMA2x2_NEG_SELFTEST);
		}
	return com_rslt;
}
/*!
 *	@brief This API is for to set
 *	the Self Test sign(selftest_sign) in the register 0x32 bit 2
 *
 *
 *
 *  @param selftest_sign_u8 : The value of self test sign
 *     selftest_sign_u8     |    result
 *  ------------------------- |------------------
 *     0x00                   | negative sign
 *     0x01                   | positive sign
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
bstdr_ret_t bma2x2_set_selftest_sign(uint8_t selftest_sign_u8)
{
	uint8_t data_u8 = 0;
	/*  Variable used to return value of
	communication routine*/
	bstdr_ret_t com_rslt = BSTDR_E_GEN_ERROR;

	if (p_bma2x2 == BMA2x2_NULL) {
		/* Check the struct p_bma2x2 is empty */
		return BSTDR_E_NULL_PTR;
		} else {
		if (selftest_sign_u8 <
		BMA2x2_SELF_TEST_SIGN_RANGE) {
			/* write self test sign */
			com_rslt = p_bma2x2->bus_read
			(p_bma2x2->dev_addr,
			BMA2x2_NEG_SELFTEST_REG,
			&data_u8, 1);
			data_u8 = BMA2x2_SET_BITSLICE
			(data_u8, BMA2x2_NEG_SELFTEST, selftest_sign_u8);
			com_rslt += p_bma2x2->bus_write
			(p_bma2x2->dev_addr,
			BMA2x2_NEG_SELFTEST_REG,
			&data_u8, 1);
		} else {
		com_rslt = BSTDR_E_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}

/*!
 *	@brief This API reads fifo overrun and fifo frame counter
 *	status register byte  from location 0Eh
 *
 *  @param stat_fifo_u8 : The status of fifo overrun and frame counter
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
bstdr_ret_t bma2x2_get_fifo_stat(uint8_t *stat_fifo_u8)
{
	/*  Variable used to return value of
	communication routine*/
	bstdr_ret_t com_rslt = BSTDR_E_GEN_ERROR;

	if (p_bma2x2 == BMA2x2_NULL) {
		/* Check the struct p_bma2x2 is empty */
		return BSTDR_E_NULL_PTR;
		} else {
			/* Read the interrupt status register 0x0E*/
			com_rslt = p_bma2x2->bus_read(
			p_bma2x2->dev_addr,
			BMA2x2_STAT_FIFO_ADDR,
			stat_fifo_u8, 1);
		}
	return com_rslt;
}
/*!
 *	@brief This API read fifo frame count
 *	from location 0Eh bit position 0 to 6
 *
 *
 * @param frame_count_u8 : The status of fifo frame count
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
bstdr_ret_t bma2x2_get_fifo_frame_count(
uint8_t *frame_count_u8)
{
	/*  Variable used to return value of
	communication routine*/
	bstdr_ret_t com_rslt = BSTDR_E_GEN_ERROR;
	uint8_t data_u8 = 0;

	if (p_bma2x2 == BMA2x2_NULL) {
		/* Check the struct p_bma2x2 is empty */
		return BSTDR_E_NULL_PTR;
		} else {
			/* Read the FIFO frame count*/
			com_rslt = p_bma2x2->bus_read(
			p_bma2x2->dev_addr,
			BMA2x2_FIFO_FRAME_COUNT_STAT_REG,
			&data_u8, 1);
			*frame_count_u8 = BMA2x2_GET_BITSLICE(data_u8,
			BMA2x2_FIFO_FRAME_COUNT_STAT);
		}
	return com_rslt;
}
/*!
 *	@brief This API read fifo overrun
 *	from location 0Eh bit position 7
 *
 *
 * @param fifo_overrun_u8 : The status of fifo overrun
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
bstdr_ret_t bma2x2_get_fifo_overrun(
uint8_t *fifo_overrun_u8)
{
		/*  Variable used to return value of
	communication routine*/
	bstdr_ret_t com_rslt = BSTDR_E_GEN_ERROR;
	uint8_t data_u8 = 0;

	if (p_bma2x2 == BMA2x2_NULL) {
		/* Check the struct p_bma2x2 is empty */
		return BSTDR_E_NULL_PTR;
		} else {
			/* Read the status of fifo over run*/
			com_rslt = p_bma2x2->bus_read(
			p_bma2x2->dev_addr,
			BMA2x2_FIFO_OVERRUN_STAT_REG,
			&data_u8, 1);
			*fifo_overrun_u8 = BMA2x2_GET_BITSLICE(data_u8,
			BMA2x2_FIFO_OVERRUN_STAT);
		}
	return com_rslt;
}

/*!
 *	@brief This API is used to get
 *	the status of fifo (fifo_mode) in the register 0x3E bit 6 and 7
 *
 *
 *  @param fifo_mode_u8 : The value of fifo mode
 *     fifo_mode_u8         |    result
 *  ------------------------- |------------------
 *        0x00                |   BYPASS
 *        0x01                |   FIFO
 *        0x02                |   STREAM
 *        0x03                |   RESERVED
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
bstdr_ret_t bma2x2_get_fifo_mode(uint8_t *fifo_mode_u8)
{
	/*  Variable used to return value of
	communication routine*/
	bstdr_ret_t com_rslt = BSTDR_E_GEN_ERROR;
	uint8_t data_u8 = 0;

	if (p_bma2x2 == BMA2x2_NULL) {
		/* Check the struct p_bma2x2 is empty */
		return BSTDR_E_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->bus_read(
			p_bma2x2->dev_addr,
			BMA2x2_FIFO_MODE_REG, &data_u8,
			1);
			*fifo_mode_u8 = BMA2x2_GET_BITSLICE(data_u8,
			BMA2x2_FIFO_MODE);
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to set
 *	the status of fifo (fifo_mode) in the register 0x3E bit 6 and 7
 *
 *
 *  @param fifo_mode_u8 : The value of fifo mode
 *     fifo_mode_u8         |    result
 *  ------------------------- |------------------
 *        0x00                |   BYPASS
 *        0x01                |   FIFO
 *        0x02                |   STREAM
 *        0x03                |   RESERVED
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
bstdr_ret_t bma2x2_set_fifo_mode(uint8_t fifo_mode_u8)
{
	uint8_t data_u8 = 0;
		/*  Variable used to return value of
	communication routine*/
	bstdr_ret_t com_rslt = BSTDR_E_GEN_ERROR;
	uint8_t config_data_u8 = 0;

	if (p_bma2x2 == BMA2x2_NULL) {
		/* Check the struct p_bma2x2 is empty */
		return BSTDR_E_NULL_PTR;
		} else {
		if (fifo_mode_u8 < BMA2x2_FIFO_MODE_RANGE) {
			com_rslt = p_bma2x2->bus_read
			(p_bma2x2->dev_addr,
			BMA2x2_FIFO_MODE_REG, &data_u8,
			1);
			data_u8 = BMA2x2_SET_BITSLICE(data_u8,
			BMA2x2_FIFO_MODE, fifo_mode_u8);
			com_rslt += p_bma2x2->bus_write
			(p_bma2x2->dev_addr,
			BMA2x2_FIFO_MODE_REG,
			&data_u8, 1);
			if (com_rslt == BSTDR_OK) {
				com_rslt += bma2x2_read_reg(
				BMA2x2_FIFO_MODE_REG,
				&config_data_u8,
				1);
				p_bma2x2->fifo_config = config_data_u8;
			}
		} else {
		com_rslt = BSTDR_E_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/*!
 * @brief This API is used to get
 * the axis enable of fifo data select in the register 0x3E bit 0 and 1
 *
 *
 *  @param fifo_data_select_u8 : The value of FIFO axis data select
 *   fifo_data_select_u8    |    result
 *  ------------------------- |------------------
 *        0x00                |   XYZ
 *        0x01                |   Y
 *        0x02                |   X
 *        0x03                |   Z
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
bstdr_ret_t bma2x2_get_fifo_data_select(uint8_t *fifo_data_select_u8)
{
		/*  Variable used to return value of
	communication routine*/
	bstdr_ret_t com_rslt = BSTDR_E_GEN_ERROR;
	uint8_t data_u8 = 0;

	if (p_bma2x2 == BMA2x2_NULL) {
		/* Check the struct p_bma2x2 is empty */
		return BSTDR_E_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->bus_read(
			p_bma2x2->dev_addr,
			BMA2x2_FIFO_DATA_SELECT_REG,
			&data_u8, 1);
			*fifo_data_select_u8 = BMA2x2_GET_BITSLICE(data_u8,	BMA2x2_FIFO_DATA_SELECT);
		}
	return com_rslt;
}
/*!
 * @brief This API is used to set
 * the axis enable of fifo data select in the register 0x3E bit 0 and 1
 *
 *
 *  @param fifo_data_select_u8 : The value of FIFO axis data select
 *   fifo_data_select_u8    |    result
 *  ------------------------- |------------------
 *        0x00                |   XYZ
 *        0x01                |   Y
 *        0x02                |   X
 *        0x03                |   Z
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
bstdr_ret_t bma2x2_set_fifo_data_select(uint8_t fifo_data_select_u8)
{
	uint8_t data_u8 = 0;
	/*  Variable used to return value of
	communication routine*/
	bstdr_ret_t com_rslt = BSTDR_E_GEN_ERROR;
	uint8_t config_data_u8 = 0;

	if (p_bma2x2 == BMA2x2_NULL) {
		/* Check the struct p_bma2x2 is empty */
		return BSTDR_E_NULL_PTR;
		} else {
		if (fifo_data_select_u8 < BMA2x2_FIFO_DATA_SELECT_RANGE) {
			com_rslt = p_bma2x2->bus_read
			(p_bma2x2->dev_addr,
			BMA2x2_FIFO_DATA_SELECT_REG, &data_u8,
			1);
			data_u8 = BMA2x2_SET_BITSLICE
			(data_u8,
			BMA2x2_FIFO_DATA_SELECT, fifo_data_select_u8);
			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr,	BMA2x2_FIFO_DATA_SELECT_REG,&data_u8, 1);
			if (com_rslt == BSTDR_OK) {
				com_rslt += bma2x2_read_reg(
				BMA2x2_FIFO_MODE_REG,
				 &config_data_u8,
				 1);
				p_bma2x2->fifo_config = config_data_u8;
			}
		} else {
		com_rslt = BSTDR_E_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to get
 *	the fifo data in the register 0x3F bit 0 to 7
 *
 *
 *  @param  output_reg_u8 : The value of fifo data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
bstdr_ret_t bma2x2_get_fifo_data_output_reg(uint8_t *output_reg_u8)
{
	uint8_t data_u8 = 0;
	/*  Variable used to return value of
	communication routine*/
	bstdr_ret_t com_rslt = BSTDR_E_GEN_ERROR;

	if (p_bma2x2 == BMA2x2_NULL) {
		/* Check the struct p_bma2x2 is empty */
		return BSTDR_E_NULL_PTR;
		} else {
			/*GET FIFO DATA OUTPUT REGISTER*/
			com_rslt = p_bma2x2->bus_read(
			p_bma2x2->dev_addr,
			BMA2x2_FIFO_DATA_OUTPUT_ADDR,
			&data_u8, 1);
			*output_reg_u8 = data_u8;
		}
	return com_rslt;
}
/*!
 * @brief This API is used to read the temp
 * from register 0x08
 *
 *
 *
 *  @param  temp_s8: The value of temperature
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
bstdr_ret_t bma2x2_read_temp(int8_t *temp_s8)
{
	uint8_t data_u8 = 0;
		/*  Variable used to return value of
	communication routine*/
	bstdr_ret_t com_rslt = BSTDR_E_GEN_ERROR;

	if (p_bma2x2 == BMA2x2_NULL) {
		/* Check the struct p_bma2x2 is empty */
		return BSTDR_E_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr,BMA2x2_TEMP_ADDR,&data_u8, 1);
			*temp_s8 = (int8_t)data_u8;
		}
	return com_rslt;
}

/*!
 * @brief
 *	This API gives data to the given register and
 *	the data is written in the corresponding register address
 *
 *
 *	@param adr_u8  -> Address of the register
 *	@param data_u8 -> The data to the register
 *	@param len_u8 -> no of bytes to read
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
bstdr_ret_t bma2x2_write_reg(uint8_t adr_u8,uint8_t *data_u8, uint8_t len_u8)
{
	/*  Variable used to return value of
	communication routine*/
	bstdr_ret_t com_rslt = BSTDR_E_GEN_ERROR;

	if (p_bma2x2 == BMA2x2_NULL) {
		/* Check the struct p_bma2x2 is empty */
		return BSTDR_E_NULL_PTR;
		} else {
		/* Write the data to the register*/
		com_rslt = p_bma2x2->bus_write
		(p_bma2x2->dev_addr, adr_u8, data_u8, len_u8);
	}
	return com_rslt;
}
/*!
 * @brief This API reads the data from
 *           the given register address
 *
 *
 *	@param adr_u8 -> Address of the register
 *	@param data_u8 -> The data from the register
 *	@param len_u8 -> no of bytes to read
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
bstdr_ret_t bma2x2_read_reg(uint8_t adr_u8,uint8_t *data_u8, uint8_t len_u8)
{
	/*  Variable used to return value of
	communication routine*/
	bstdr_ret_t com_rslt = BSTDR_E_GEN_ERROR;

	if (p_bma2x2 == BMA2x2_NULL) {
		/* Check the struct p_bma2x2 is empty */
		return BSTDR_E_NULL_PTR;
		} else {
			/*Read the data from the register*/
			com_rslt = p_bma2x2->bus_read
			(p_bma2x2->dev_addr, adr_u8, data_u8, len_u8);
		}
	return com_rslt;
}
