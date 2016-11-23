/*
****************************************************************************
* Copyright (C) 2010 - 2015 Bosch Sensortec GmbH
*
* bmg160.c
* Date: 2015/04/29
* Revision: 2.0.4 $
*
* Usage: Sensor Driver for BMG160 sensor
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
/*! file bmg160.c
    brief Driver for BMG160 */
#include "bmg160.h"
static bmg160_t *p_bmg160;


/*!
 *	@brief This function is used for initialize
 *	the bus read and bus write functions
 *  and assign the chip id and I2C address of the gyro
 *	chip id is read in the register 0x00 bit from 0 to 7
 *
 *	@param bmg160 structure pointer.
 *
 *	@note While changing the parameter of the bmg160_t
 *	consider the following point:
 *	@note Changing the reference value of the parameter
 *	will changes the local copy or local reference
 *	make sure your changes will not
 *	affect the reference value of the parameter
 *	(Better case don't change the reference value of the parameter)
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
bstdr_ret_t bmg160_init(bmg160_t *bmg160)
{
	/* variable used to return the bus communication status*/
	bstdr_ret_t comres = BSTDR_E_GEN_ERROR;
	uint8_t v_data_u8  = 0;
	/* Assign the initial function pointers*/
	p_bmg160 = bmg160;
	/*Read CHIP_ID */
	comres = p_bmg160->bus_read(p_bmg160->dev_addr,
	 BMG160_CHIP_ID_ADDR, &v_data_u8, 1);
	p_bmg160->chip_id = v_data_u8;
	return comres;
}
/** @brief checks the bmg160 connection (chip id read out)
 * */
bstdr_ret_t bmg160_check_connection(void){
	/* variable used to return the bus communication status*/
	bstdr_ret_t comres = BSTDR_E_GEN_ERROR;
	uint8_t v_data_u8  = 0;
	if(p_bmg160!=0){
		/*Read CHIP_ID */
		p_bmg160->bus_read(p_bmg160->dev_addr,	BMG160_CHIP_ID_ADDR, &v_data_u8, 1);
		if(v_data_u8==0x0F){
			comres = BSTDR_OK;
		}
	} else{
		comres = BSTDR_E_NULL_PTR;
	}
	return comres;
}

/*!
 * @brief Reads data X,Y and Z from register location 0x02 to 0x07
 *
 *
 *
 *
 *  @param data: The value of gyro xyz axis data
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
bstdr_ret_t bmg160_get_data_XYZ(bmg160_xyz_t *data)
{
	/* variable used to return the bus communication status*/
	bstdr_ret_t comres = BSTDR_E_GEN_ERROR;
	uint8_t v_data_u8[BMG160_XYZ_DATA_SIZE] = {0};

	/* check the p_bmg160 struct pointer is NULL*/
	if (p_bmg160 == BMG160_NULL) {
		return  BSTDR_E_NULL_PTR;
		} else {
		comres = p_bmg160->bus_read(p_bmg160->dev_addr, BMG160_RATE_X_LSB_BIT__REG, v_data_u8, BMG160_ALL_DATA_FRAME_LENGTH);
		/* Data X */
		v_data_u8[BMG160_DATA_FRAME_X_LSB_BYTE] =
		BMG160_GET_BITSLICE(v_data_u8[BMG160_DATA_FRAME_X_LSB_BYTE],
		BMG160_RATE_X_LSB_BIT);
		data->x = (int16_t)
		((((int32_t)((int8_t)v_data_u8[BMG160_DATA_FRAME_X_MSB_BYTE]))
		<< BMG160_SHIFT_BIT_POSITION_BY_08_BITS) |
		(v_data_u8[BMG160_DATA_FRAME_X_LSB_BYTE]));
		/* Data Y */
		v_data_u8[BMG160_DATA_FRAME_Y_LSB_BYTE] =
		BMG160_GET_BITSLICE(v_data_u8[BMG160_DATA_FRAME_Y_LSB_BYTE],
		BMG160_RATE_Y_LSB_BIT);
		data->y = (int16_t)
		((((int32_t)((int8_t)v_data_u8[BMG160_DATA_FRAME_Y_MSB_BYTE]))
		<< BMG160_SHIFT_BIT_POSITION_BY_08_BITS) |
		(v_data_u8[BMG160_DATA_FRAME_Y_LSB_BYTE]));
		/* Data Z */
		v_data_u8[BMG160_DATA_FRAME_Z_LSB_BYTE] =
		BMG160_GET_BITSLICE(v_data_u8[BMG160_DATA_FRAME_Z_LSB_BYTE],
		BMG160_RATE_Z_LSB_BIT);
		data->z = (int16_t)
		((((int32_t)((int8_t)v_data_u8[BMG160_DATA_FRAME_Z_MSB_BYTE]))
		<< BMG160_SHIFT_BIT_POSITION_BY_08_BITS) |
		(v_data_u8[BMG160_DATA_FRAME_Z_LSB_BYTE]));
	}
	return comres;
}
/*!
 * @brief Reads Temperature from register location 0x08
 *
 *
 *
 *
 *  @param v_temp_s8: The value of temperature
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
bstdr_ret_t bmg160_get_temp(int8_t *v_temp_s8)
{
	/* variable used to return the bus communication status*/
	bstdr_ret_t comres = BSTDR_E_GEN_ERROR;
	uint8_t v_data_u8  = 0;
	/* check the p_bmg160 struct pointer is NULL*/
	if (p_bmg160 == BMG160_NULL) {
		return  BSTDR_E_NULL_PTR;
		} else {
		/* read temperature data*/
		comres = p_bmg160->bus_read(p_bmg160->dev_addr,
		BMG160_TEMP_ADDR, &v_data_u8,
		1);
		*v_temp_s8 = v_data_u8;
	}
	return comres;
}
/*!
 *	@brief This API is used to get
 *	the range in the register 0x0F bits from 0 to 2
 *
 *	@param v_range_u8 : The value of gyro range
 *	value    |   range
 * ----------|-----------
 *    0x00   | BMG160_RANGE_2000
 *    0x01   | BMG160_RANGE_1000
 *    0x02   | BMG160_RANGE_500
 *    0x03   | BMG160_RANGE_250
 *    0x04   | BMG160_RANGE_125
 *
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
bstdr_ret_t bmg160_get_range(uint8_t *v_range_u8)
{
	/* variable used to return the bus communication status*/
	bstdr_ret_t comres = BSTDR_E_GEN_ERROR;
	uint8_t v_data_u8  = 0;
	/* check the p_bmg160 struct pointer is NULL*/
	if (p_bmg160 == BMG160_NULL) {
		return  BSTDR_E_NULL_PTR;
		} else {
		/* read the gyro range */
		comres = p_bmg160->bus_read
		(p_bmg160->dev_addr,
		BMG160_RANGE_ADDR_RANGE__REG, &v_data_u8,
		1);
		*v_range_u8 =
		BMG160_GET_BITSLICE(v_data_u8, BMG160_RANGE_ADDR_RANGE);
	}
	return comres;
}
/*!
 *	@brief This API is used to set
 *	the range in the register 0x0F bits from 0 to 2
 *
 *	@param v_range_u8 : The value of gyro range
 *	value    |   range
 * ----------|-----------
 *    0x00   | BMG160_RANGE_2000
 *    0x01   | BMG160_RANGE_1000
 *    0x02   | BMG160_RANGE_500
 *    0x03   | BMG160_RANGE_250
 *    0x04   | BMG160_RANGE_125
 *
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
bstdr_ret_t bmg160_set_range(uint8_t v_range_u8)
{
	/* variable used to return the bus communication status*/
	bstdr_ret_t comres = BSTDR_E_GEN_ERROR;
	uint8_t v_data_u8  = 0;
	/* check the p_bmg160 struct pointer is NULL*/
	if (p_bmg160 == BMG160_NULL) {
		return  BSTDR_E_NULL_PTR;
		} else {
		if (v_range_u8 < BMG160_BIT_LENGTH_RANGE) {
			/* write the range*/
			comres = p_bmg160->bus_read
			(p_bmg160->dev_addr,
			BMG160_RANGE_ADDR_RANGE__REG, &v_data_u8,
			1);
			v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
			BMG160_RANGE_ADDR_RANGE,
			v_range_u8);
			comres += p_bmg160->bus_write
			(p_bmg160->dev_addr,
			BMG160_RANGE_ADDR_RANGE__REG, &v_data_u8,
			1);
		} else {
			comres = BSTDR_E_OUT_OF_RANGE;
		}
	}
	return comres;
}
/*!
 *	@brief This API is used to get the gyro bandwidth
 *	in the register 0x10 bits from 0 to 3
 *
 *
 *
 *
 *
 *	@param  v_bw_u8: The value of gyro bandwidth
 *  value   |  bandwidth
 * ---------|---------------
 *   0x00   |  BMG160_BW_500_HZ
 *   0x01   |  BMG160_BW_230_HZ
 *   0x02   |  BMG160_BW_116_HZ
 *   0x03   |  BMG160_BW_47_HZ
 *   0x04   |  BMG160_BW_23_HZ
 *   0x05   |  BMG160_BW_12_HZ
 *   0x06   |  BMG160_BW_64_HZ
 *   0x07   |  BMG160_BW_32_HZ
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
bstdr_ret_t bmg160_get_bw(uint8_t *v_bw_u8)
{
	/* variable used to return the bus communication status*/
	bstdr_ret_t comres = BSTDR_E_GEN_ERROR;
	uint8_t v_data_u8  = 0;
	/* check the p_bmg160 struct pointer is NULL*/
	if (p_bmg160 == BMG160_NULL) {
		return  BSTDR_E_NULL_PTR;
		} else {
		/* read gyro bandwidth*/
		comres = p_bmg160->bus_read
		(p_bmg160->dev_addr, BMG160_BW_ADDR__REG,
		&v_data_u8, 1);
		*v_bw_u8 = BMG160_GET_BITSLICE(v_data_u8,
			BMG160_BW_ADDR);
	}
	return comres;
}
/*!
 *	@brief This API is used to set the gyro bandwidth
 *	in the register 0x10 bits from 0 to 3
 *
 *
 *
 *
 *
 *	@param  v_bw_u8: The value of gyro bandwidth
 *  value   |  bandwidth
 * ---------|---------------
 *   0x00   |  BMG160_BW_500_HZ
 *   0x01   |  BMG160_BW_230_HZ
 *   0x02   |  BMG160_BW_116_HZ
 *   0x03   |  BMG160_BW_47_HZ
 *   0x04   |  BMG160_BW_23_HZ
 *   0x05   |  BMG160_BW_12_HZ
 *   0x06   |  BMG160_BW_64_HZ
 *   0x07   |  BMG160_BW_32_HZ
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
bstdr_ret_t bmg160_set_bw(uint8_t v_bw_u8)
{
	/* variable used to return the bus communication status*/
	bstdr_ret_t comres = BSTDR_E_GEN_ERROR;
	uint8_t v_data_u8  = 0;
	uint8_t v_mode_u8r  = 0;
	uint8_t v_auto_sleep_dur = 0;
	/* check the p_bmg160 struct pointer is NULL*/
	if (p_bmg160 == BMG160_NULL) {
		return  BSTDR_E_NULL_PTR;
		} else {
		if (v_bw_u8 < BMG160_BIT_LENGTH_BW) {
			comres = bmg160_get_power_mode(&v_mode_u8r);
			if (v_mode_u8r == BMG160_MODE_ADVANCEDPOWERSAVING) {
				/* check the advance power save mode */
				comres = bmg160_get_auto_sleep_durn(&v_auto_sleep_dur);
				comres = bmg160_set_auto_sleep_durn(v_auto_sleep_dur,v_bw_u8);
				}
				/* read gyro bandwidth*/
				comres = p_bmg160->bus_read
				(p_bmg160->dev_addr,
				BMG160_BW_ADDR__REG,
				&v_data_u8, 1);
				v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
				BMG160_BW_ADDR, v_bw_u8);
				comres += p_bmg160->bus_write
				(p_bmg160->dev_addr,
				BMG160_BW_ADDR__REG,
				&v_data_u8, 1);
		} else {
			comres = BSTDR_E_OUT_OF_RANGE;
		}
	}
	return comres;
}
/*!
 *	This function is used for the soft reset
 *	The soft reset register will be written with 0xB6 in the register 0x14.
 *
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
bstdr_ret_t bmg160_set_soft_rst(void)
{
	/* variable used to return the bus communication status*/
	bstdr_ret_t comres = BSTDR_E_GEN_ERROR;
	uint8_t v_soft_rst_u8  = 0;

	v_soft_rst_u8 = BMG160_SOFT_RESET;
	/* check the p_bmg160 struct pointer is NULL*/
	if (p_bmg160 == BMG160_NULL) {
		return  BSTDR_E_NULL_PTR;
		} else {
		/* write soft reset*/
		comres = p_bmg160->bus_write(p_bmg160->dev_addr,
		BMG160_BGW_SOFT_RST_ADDR, &v_soft_rst_u8,
		1);
	}
	return comres;
}
/*!
 *	@brief This API is used to get the fifo(fifo_enable)
 *	interrupt enable bits of the sensor in the registers 0x15 bit 6
 *
 *
 *
 *
 *	@param v_fifo_enable_u8 : The value of  fifo enable
 *   value    |  Description
 * -----------|---------------
 *    1       |  BMG160_ENABLE
 *    0       |  BMG160_DISABLE
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
bstdr_ret_t bmg160_get_fifo_enable(uint8_t *v_fifo_enable_u8)
{
	/* variable used to return the bus communication status*/
	bstdr_ret_t comres = BSTDR_E_GEN_ERROR;
	uint8_t v_data_u8  = 0;
	/* check the p_bmg160 struct pointer is NULL*/
	if (p_bmg160 == BMG160_NULL) {
		return  BSTDR_E_NULL_PTR;
		} else {
		/* read the fifo enable */
		comres = p_bmg160->bus_read(p_bmg160->dev_addr,
		BMG160_INTR_ENABLE0_FIFO__REG,
		&v_data_u8, 1);
		*v_fifo_enable_u8 = BMG160_GET_BITSLICE(v_data_u8,
		BMG160_INTR_ENABLE0_FIFO);
	}
	return comres;
}
/*!
 *	@brief This API is used to set the fifo(fifo_enable)
 *	interrupt enable bits of the sensor in the registers 0x15 bit 6
 *
 *
 *
 *
 *	@param v_fifo_enable_u8 : The value of  fifo enable
 *   value    |  Description
 * -----------|---------------
 *    1       |  BMG160_ENABLE
 *    0       |  BMG160_DISABLE
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
bstdr_ret_t bmg160_set_fifo_enable(uint8_t v_fifo_enable_u8)
{
	/* variable used to return the bus communication status*/
	bstdr_ret_t comres = BSTDR_E_GEN_ERROR;
	uint8_t v_data_u8  = 0;
	/* check the p_bmg160 struct pointer is NULL*/
	if (p_bmg160 == BMG160_NULL) {
		return  BSTDR_E_NULL_PTR;
		} else {
		if (v_fifo_enable_u8 < BMG160_BIT_LENGTH_FIFO) {
			/* write the fifo enable */
			comres = p_bmg160->bus_read
			(p_bmg160->dev_addr,
			BMG160_INTR_ENABLE0_FIFO__REG,
			&v_data_u8, 1);
			v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
			BMG160_INTR_ENABLE0_FIFO, v_fifo_enable_u8);
			comres += p_bmg160->bus_write
			(p_bmg160->dev_addr,
			BMG160_INTR_ENABLE0_FIFO__REG,
			&v_data_u8, 1);
		} else {
			comres = BSTDR_E_OUT_OF_RANGE;
		}
	}
	return comres;
}
/*!
 *	@brief This API is used to get
 *	the status of fifo water mark in the register 0x1E bit 7
 *
 *
 *
 *	@param v_fifo_wm_enable_u8 : The value of fifo water mark enable
 *   value    |  Description
 * -----------|---------------
 *    1       |  BMG160_ENABLE
 *    0       |  BMG160_DISABLE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
bstdr_ret_t bmg160_get_fifo_wm_enable(
uint8_t *v_fifo_wm_enable_u8)
{
	/* variable used to return the bus communication status*/
	bstdr_ret_t comres = BSTDR_E_GEN_ERROR;
	uint8_t v_data_u8  = 0;
	/* check the p_bmg160 struct pointer is NULL*/
	if (p_bmg160 == BMG160_NULL) {
		return  BSTDR_E_NULL_PTR;
		} else {
		/* read fifo water mark enable */
		comres = p_bmg160->bus_read(p_bmg160->dev_addr,
		BMG160_INTR_4_FIFO_WM_ENABLE__REG,
		&v_data_u8, 1);
		*v_fifo_wm_enable_u8 = BMG160_GET_BITSLICE(v_data_u8,
		BMG160_INTR_4_FIFO_WM_ENABLE);
	}
	return comres;
}
/*!
 *	@brief This API is used to set
 *	the status of fifo water mark in the register 0x1E bit 7
 *
 *
 *
 *	@param v_fifo_wm_enable_u8 : The value of fifo water mark enable
 *   value    |  Description
 * -----------|---------------
 *    1       |  BMG160_ENABLE
 *    0       |  BMG160_DISABLE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
bstdr_ret_t bmg160_set_fifo_wm_enable(
uint8_t v_fifo_wm_enable_u8)
{
	/* variable used to return the bus communication status*/
	bstdr_ret_t comres = BSTDR_E_GEN_ERROR;
	uint8_t v_data_u8  = 0;
	/* check the p_bmg160 struct pointer is NULL*/
	if (p_bmg160 == BMG160_NULL) {
		return  BSTDR_E_NULL_PTR;
		} else {
		/* write fifo water mark enable*/
		if (v_fifo_wm_enable_u8 < BMG160_BIT_LENGTH_FIFO_WM) {
			comres = p_bmg160->bus_read
			(p_bmg160->dev_addr,
			BMG160_INTR_4_FIFO_WM_ENABLE__REG,
			&v_data_u8, 1);
			v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
			BMG160_INTR_4_FIFO_WM_ENABLE, v_fifo_wm_enable_u8);
			comres += p_bmg160->bus_write
			(p_bmg160->dev_addr,
			BMG160_INTR_4_FIFO_WM_ENABLE__REG,
			&v_data_u8, 1);
		} else {
			comres = BSTDR_E_OUT_OF_RANGE;
		}
	}
	return comres;
}
/*!
 *	@brief  This API is used to get the status of FIFO tag
 *	in the register 0x3D bit 7
 *
 *
 *
 *	@param  v_fifo_tag_u8 : The value of fifo tag enable
 *   value    |  Description
 * -----------|---------------
 *    1       |  BMG160_ENABLE
 *    0       |  BMG160_DISABLE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
bstdr_ret_t bmg160_get_fifo_tag(uint8_t *v_fifo_tag_u8)
{
	/* variable used to return the bus communication status*/
	bstdr_ret_t comres = BSTDR_E_GEN_ERROR;
	uint8_t v_data_u8 = 0;
	/* check the p_bmg160 struct pointer is NULL*/
	if (p_bmg160 == BMG160_NULL) {
		return  BSTDR_E_NULL_PTR;
		} else {
		/* read fifo tag*/
		comres = p_bmg160->bus_read(p_bmg160->dev_addr,
		BMG160_FIFO_CGF1_ADDR_TAG__REG,
		&v_data_u8, 1);
		*v_fifo_tag_u8 = BMG160_GET_BITSLICE(v_data_u8,
		BMG160_FIFO_CGF1_ADDR_TAG);
	}
	return comres;
}
/*!
 *	@brief  This API is used to set the status of FIFO tag
 *	in the register 0x3D bit 7
 *
 *
 *
 *	@param  v_fifo_tag_u8 : The value of fifo tag enable
 *   value    |  Description
 * -----------|---------------
 *    1       |  BMG160_ENABLE
 *    0       |  BMG160_DISABLE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
bstdr_ret_t bmg160_set_fifo_tag(uint8_t v_fifo_tag_u8)
{
	/* variable used to return the bus communication status*/
	bstdr_ret_t comres = BSTDR_E_GEN_ERROR;
	uint8_t v_data_u8 = 0;
	/* check the p_bmg160 struct pointer is NULL*/
	if (p_bmg160 == BMG160_NULL) {
		return  BSTDR_E_NULL_PTR;
		} else {
		if (v_fifo_tag_u8 < BMG160_BIT_LENGTH_FIFO_TAG) {
			/* write fifo tag */
			comres = p_bmg160->bus_read
			(p_bmg160->dev_addr,
			BMG160_FIFO_CGF1_ADDR_TAG__REG,
			&v_data_u8, 1);
			v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
			BMG160_FIFO_CGF1_ADDR_TAG, v_fifo_tag_u8);
			comres += p_bmg160->bus_write
			(p_bmg160->dev_addr,
			BMG160_FIFO_CGF1_ADDR_TAG__REG,
			&v_data_u8, 1);
		} else {
			comres = BSTDR_E_OUT_OF_RANGE;
		}
	}
	return comres;
}
/*!
 *	@brief This API is used to get Water Mark Level
 *	in the register 0x3D bit from 0 to 6
 *
 *
 *
 *	@param v_fifo_wm_level_u8 : The value of fifo water mark level
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
bstdr_ret_t bmg160_get_fifo_wm_level(
uint8_t *v_fifo_wm_level_u8)
{
	/* variable used to return the bus communication status*/
	bstdr_ret_t comres = BSTDR_E_GEN_ERROR;
	uint8_t v_data_u8 = 0;
	/* check the p_bmg160 struct pointer is NULL*/
	if (p_bmg160 == BMG160_NULL) {
		return  BSTDR_E_NULL_PTR;
		} else {
		comres = p_bmg160->bus_read(p_bmg160->dev_addr,
		BMG160_FIFO_CGF1_ADDR_WML__REG,
		&v_data_u8, 1);
		*v_fifo_wm_level_u8 = BMG160_GET_BITSLICE(v_data_u8,
		BMG160_FIFO_CGF1_ADDR_WML);
	}
	return comres;
}
/*!
 *	@brief This API is used to set Water Mark Level
 *	in the register 0x3D bit from 0 to 6
 *
 *
 *
 *	@param v_fifo_wm_level_u8 : The value of fifo water mark level
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
bstdr_ret_t bmg160_set_fifo_wm_level(
uint8_t v_fifo_wm_level_u8)
{
	/* variable used to return the bus communication status*/
	bstdr_ret_t comres = BSTDR_E_GEN_ERROR;
	uint8_t v_data_u8 = 0;
	/* check the p_bmg160 struct pointer is NULL*/
	if (p_bmg160 == BMG160_NULL) {
		return  BSTDR_E_NULL_PTR;
		} else {
		if (v_fifo_wm_level_u8 < BMG160_FIFO_WM_LENGTH) {
			comres = p_bmg160->bus_read
			(p_bmg160->dev_addr,
			BMG160_FIFO_CGF1_ADDR_WML__REG,
			&v_data_u8, 1);
			v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
			BMG160_FIFO_CGF1_ADDR_WML, v_fifo_wm_level_u8);
			comres += p_bmg160->bus_write
			(p_bmg160->dev_addr,
			BMG160_FIFO_CGF1_ADDR_WML__REG,
			&v_data_u8, 1);
		} else {
			comres = BSTDR_E_OUT_OF_RANGE;
		}
	}
	return comres;
}

/*!
 * @brief Reads FIFO data from location 0x3F
 *
 *
 *
 *
 *  @param v_fifo_data_u8 : The data of fifo
 *
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error result of communication routines
 *
 *
*/
bstdr_ret_t bmg160_get_FIFO_data_reg(uint8_t *v_fifo_data_u8)
{
	/* variable used to return the bus communication status*/
	bstdr_ret_t comres = BSTDR_E_GEN_ERROR;
	uint8_t v_data_u8 = 0;
	/* check the p_bmg160 struct pointer is NULL*/
	if (p_bmg160 == BMG160_NULL) {
		return  BSTDR_E_NULL_PTR;
		} else {
		/* read the fifo data */
		comres = p_bmg160->bus_read(p_bmg160->dev_addr,
		BMG160_FIFO_DATA_ADDR, &v_data_u8,
		1);
		*v_fifo_data_u8 = v_data_u8;
	}
	return comres;
}
/*!
 *	@brief this api is used to read the fifo status
 *	of frame_counter and overrun in the register 0x0E
 *	@note frame_counter > bit from 0 to 6
 *	@note overrun -> bit 7
 *
 *
 *
 *	@param v_fifo_stat_u8 : The value of fifo overrun and fifo counter
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
bstdr_ret_t bmg160_get_fifo_stat_reg(
uint8_t *v_fifo_stat_u8)
{
	/* variable used to return the bus communication status*/
	bstdr_ret_t comres = BSTDR_E_GEN_ERROR;
	uint8_t v_data_u8 = 0;
	/* check the p_bmg160 struct pointer is NULL*/
	if (p_bmg160 == BMG160_NULL) {
		return  BSTDR_E_NULL_PTR;
		} else {
		/* read fifo over run and frame counter */
		comres = p_bmg160->bus_read(p_bmg160->dev_addr,
		BMG160_FIFO_STAT_ADDR, &v_data_u8,
		1);
		*v_fifo_stat_u8 = v_data_u8;
	}
	return comres;
}
/*!
 *	@brief this API is used to get the fifo frame counter
 *	in the register 0x0E bit 0 to 6
 *
 *
 *
 *	@param v_fifo_frame_count_u8: The value of fifo frame counter
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
bstdr_ret_t bmg160_get_fifo_frame_count(
uint8_t *v_fifo_frame_count_u8)
{
	/* variable used to return the bus communication status*/
	bstdr_ret_t comres = BSTDR_E_GEN_ERROR;
	uint8_t v_data_u8  = 0;
	/* check the p_bmg160 struct pointer is NULL*/
	if (p_bmg160 == BMG160_NULL) {
		return  BSTDR_E_NULL_PTR;
		} else {
		/* read fifo frame counter */
		comres = p_bmg160->bus_read(p_bmg160->dev_addr,
		BMG160_FIFO_STAT_FRAME_COUNTER__REG,
		&v_data_u8, 1);
		*v_fifo_frame_count_u8 = BMG160_GET_BITSLICE(v_data_u8,
		BMG160_FIFO_STAT_FRAME_COUNTER);
	}
	return comres;
}
/*!
 *	@brief this API is used to get the fifo over run
 *	in the register 0x0E bit 7
 *
 *
 *
 *	@param v_fifo_overrun_u8: The value of fifo over run
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
bstdr_ret_t bmg160_get_fifo_overrun(
uint8_t *v_fifo_overrun_u8)
{
	/* variable used to return the bus communication status*/
	bstdr_ret_t comres = BSTDR_E_GEN_ERROR;
	uint8_t v_data_u8 = 0;
	/* check the p_bmg160 struct pointer is NULL*/
	if (p_bmg160 == BMG160_NULL) {
		return  BSTDR_E_NULL_PTR;
		} else {
		/* read fifo over run*/
		comres = p_bmg160->bus_read(p_bmg160->dev_addr,
		BMG160_FIFO_STAT_OVERRUN__REG,
		&v_data_u8, 1);
		*v_fifo_overrun_u8 = BMG160_GET_BITSLICE(v_data_u8,
		BMG160_FIFO_STAT_OVERRUN);
	}
	return comres;
}
/*!
 *	@brief This API is used to get the status of fifo mode
 *	in the register 0x3E bit 6 and 7
 *
 *
 *
 *	@param v_fifo_mode_u8 : The value of fifo mode
 *        mode      |    value
 *  ----------------|--------------
 *      BYPASS      |  0
 *      FIFO        |  1
 *      STREAM      |  2
 *      RESERVED    |  3
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
bstdr_ret_t bmg160_get_fifo_mode(uint8_t *v_fifo_mode_u8)
{
	/* variable used to return the bus communication status*/
	bstdr_ret_t comres = BSTDR_E_GEN_ERROR;
	uint8_t v_data_u8 = 0;
	/* check the p_bmg160 struct pointer is NULL*/
	if (p_bmg160 == BMG160_NULL) {
		return  BSTDR_E_NULL_PTR;
		} else {
		/* read fifo mode*/
		comres = p_bmg160->bus_read(p_bmg160->dev_addr,
		BMG160_FIFO_CGF0_ADDR_MODE__REG, &v_data_u8,
		1);
		*v_fifo_mode_u8 = BMG160_GET_BITSLICE(v_data_u8,
		BMG160_FIFO_CGF0_ADDR_MODE);
	}
	return comres;
}
/*!
 *	@brief This API is used to set the status of fifo mode
 *	in the register 0x3E bit 6 and 7
 *
 *
 *
 *	@param v_fifo_mode_u8 : The value of fifo mode
 *        mode      |    value
 *  ----------------|--------------
 *      BYPASS      |  0
 *      FIFO        |  1
 *      STREAM      |  2
 *      RESERVED    |  3
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
bstdr_ret_t bmg160_set_fifo_mode(uint8_t v_fifo_mode_u8)
{
	/* variable used to return the bus communication status*/
	bstdr_ret_t comres = BSTDR_E_GEN_ERROR;
	uint8_t v_data_u8 = 0;
	/* check the p_bmg160 struct pointer is NULL*/
	if (p_bmg160 == BMG160_NULL) {
		return  BSTDR_E_NULL_PTR;
		} else {
		if (v_fifo_mode_u8 < BMG160_BIT_LENGTH_FIFO_MODE) {
			/* write fifo mode*/
			comres = p_bmg160->bus_read
			(p_bmg160->dev_addr,
			BMG160_FIFO_CGF0_ADDR_MODE__REG,
			&v_data_u8, 1);
			v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
			BMG160_FIFO_CGF0_ADDR_MODE, v_fifo_mode_u8);
			comres += p_bmg160->bus_write
			(p_bmg160->dev_addr,
			BMG160_FIFO_CGF0_ADDR_MODE__REG,
			&v_data_u8, 1);
		} else {
			comres = BSTDR_E_OUT_OF_RANGE;
		}
	}
	return comres;
}
/*!
 *	@brief This API is used to get the status of fifo
 *	data select in the register 0x3E bit 0 and 1
 *
 *
 *	@param v_fifo_data_select_u8 : The value of fifo data selection
 *      data selection         |    value
 *  ---------------------------|--------------
 *      X,Y and Z (DEFAULT)    |  0
 *      X only                 |  1
 *      Y only                 |  2
 *      Z only                 |  3
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
bstdr_ret_t bmg160_get_fifo_data_select(
uint8_t *v_fifo_data_select_u8)
{
	/* variable used to return the bus communication status*/
	bstdr_ret_t comres = BSTDR_E_GEN_ERROR;
	uint8_t v_data_u8 = 0;
	/* check the p_bmg160 struct pointer is NULL*/
	if (p_bmg160 == BMG160_NULL) {
		return  BSTDR_E_NULL_PTR;
		} else {
		/* read fifo data select*/
		comres = p_bmg160->bus_read(p_bmg160->dev_addr,
		BMG160_FIFO_CGF0_ADDR_DATA_SELECT__REG,
		&v_data_u8, 1);
		*v_fifo_data_select_u8 = BMG160_GET_BITSLICE(v_data_u8,
		BMG160_FIFO_CGF0_ADDR_DATA_SELECT);
	}
	return comres;
}
/*!
 *	@brief This API is used to set the status of fifo
 *	data select in the register 0x3E bit 0 and 1
 *
 *
 *	@param v_fifo_data_select_u8 : The value of fifo data selection
 *      data selection         |    value
 *  ---------------------------|--------------
 *      X,Y and Z (DEFAULT)    |  0
 *      X only                 |  1
 *      Y only                 |  2
 *      Z only                 |  3
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
bstdr_ret_t bmg160_set_fifo_data_select(
uint8_t v_fifo_data_select_u8)
{
	/* variable used to return the bus communication status*/
	bstdr_ret_t comres = BSTDR_E_GEN_ERROR;
	uint8_t v_data_u8 = 0;
	/* check the p_bmg160 struct pointer is NULL*/
	if (p_bmg160 == BMG160_NULL) {
		return  BSTDR_E_NULL_PTR;
		} else {
		if (v_fifo_data_select_u8 <
		BMG160_BIT_LENGTH_FIFO_DATA_SELECT) {
			/* write fifo data select*/
			comres = p_bmg160->bus_read
			(p_bmg160->dev_addr,
			BMG160_FIFO_CGF0_ADDR_DATA_SELECT__REG,
			&v_data_u8, 1);
			v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
			BMG160_FIFO_CGF0_ADDR_DATA_SELECT,
			v_fifo_data_select_u8);
			comres += p_bmg160->bus_write
			(p_bmg160->dev_addr,
			BMG160_FIFO_CGF0_ADDR_DATA_SELECT__REG,
			&v_data_u8, 1);
		} else {
			comres = BSTDR_E_OUT_OF_RANGE;
		}
	}
	return comres;
}
/*!
 *	@brief This API is used to get the operating modes of the
 *	sensor in the registers 0x11 and 0x12
 *
 *
 *
 *  @param  v_power_mode_u8 :The value of power mode
 *  value     |   power mode
 * -----------|----------------
 *     0      | BMG160_MODE_NORMAL
 *     1      | BMG160_MODE_SUSPEND
 *     2      | BMG160_MODE_DEEPSUSPEND
 *     3      | BMG160_MODE_FASTPOWERUP
 *     4      | BMG160_MODE_ADVANCEDPOWERSAVING
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
bstdr_ret_t bmg160_get_power_mode(uint8_t *v_power_mode_u8)
{
	/* variable used to return the bus communication status*/
	bstdr_ret_t comres = BSTDR_E_GEN_ERROR;
	uint8_t data1 = 0;
	uint8_t data2 = 0;
	uint8_t data3 = 0;

	if (p_bmg160 == 0) {
		return  BSTDR_E_NULL_PTR;
		} else {
		/* read the power mode*/
		comres = p_bmg160->bus_read(p_bmg160->dev_addr,
		BMG160_MODE_LPM1_ADDR, &data1,
		1);
		comres += p_bmg160->bus_read(p_bmg160->dev_addr,
		BMG160_MODE_LPM2_ADDR, &data2,
		1);
		data1  = (data1 & 0xA0) >> 5;
		data3  = (data2 & 0x40) >> 6;
		data2  = (data2 & 0x80) >> 7;
		if (data3 == 0x01) {
			*v_power_mode_u8  = BMG160_MODE_ADVANCEDPOWERSAVING;
		} else {
			if ((data1 == 0x00) && (data2 == 0x00)) {
				*v_power_mode_u8  = BMG160_MODE_NORMAL;
				} else {
				if ((data1 == 0x01) || (data1 == 0x05)) {
					*v_power_mode_u8  =
					BMG160_MODE_DEEPSUSPEND;
					} else {
					if ((data1 == 0x04) &&
					(data2 == 0x00)) {
						*v_power_mode_u8  =
						BMG160_MODE_SUSPEND;
					} else {
					if ((data1 == 0x04) &&
						(data2 == 0x01))
							*v_power_mode_u8  =
							BMG160_MODE_FASTPOWERUP;
						}
					}
				}
			}
		}
	return comres;
}
/*!
 *	@brief This API is used to set the operating modes of the
 *	sensor in the registers 0x11 and 0x12
 *
 *
 *
 *  @param  v_power_mode_u8 :The value of power mode
 *  value     |   power mode
 * -----------|----------------
 *     0      | BMG160_MODE_NORMAL
 *     1      | BMG160_MODE_SUSPEND
 *     2      | BMG160_MODE_DEEPSUSPEND
 *     3      | BMG160_MODE_FASTPOWERUP
 *     4      | BMG160_MODE_ADVANCEDPOWERSAVING
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
bstdr_ret_t bmg160_set_power_mode(uint8_t v_power_mode_u8)
{
	/* variable used to return the bus communication status*/
	bstdr_ret_t comres = BSTDR_E_GEN_ERROR;
	uint8_t data1 = 0;
	uint8_t data2 = 0;
	uint8_t data3 = 0;
	uint8_t v_autosleepduration = 0;
	uint8_t v_bw_u8r = 0;

	if (p_bmg160 == 0) {
		return  BSTDR_E_NULL_PTR;
		} else {
		if (v_power_mode_u8 < BMG160_BIT_LENGTH_POWER_MODE) {
			/* write the power mode*/
			comres = p_bmg160->bus_read
				(p_bmg160->dev_addr,
			BMG160_MODE_LPM1_ADDR, &data1,
			1);
			comres += p_bmg160->bus_read
				(p_bmg160->dev_addr,
			BMG160_MODE_LPM2_ADDR, &data2,
			1);
			switch (v_power_mode_u8) {
			case BMG160_MODE_NORMAL:
				data1  = BMG160_SET_BITSLICE(data1,
				BMG160_MODE_LPM1, 0);
				data2  = BMG160_SET_BITSLICE(data2,
				BMG160_MODE_LPM2_ADDR_FAST_POWERUP,
				0);
				data3  = BMG160_SET_BITSLICE(data2,
				BMG160_MODE_LPM2_ADDR_ADV_POWERSAVING,
				0);
				comres += p_bmg160->bus_write
				(p_bmg160->dev_addr,
			BMG160_MODE_LPM1_ADDR, &data1,
			1);
			p_bmg160->delay(BMG160_POWER_MODE_DELAY);
			/*A minimum delay of at least
			450us is required for Multiple write.*/
			comres += p_bmg160->bus_write
				(p_bmg160->dev_addr,
			BMG160_MODE_LPM2_ADDR, &data3,
			1);
				break;
			case BMG160_MODE_DEEPSUSPEND:
				data1  = BMG160_SET_BITSLICE(data1,
				BMG160_MODE_LPM1,
				1);
				data2  = BMG160_SET_BITSLICE(data2,
				BMG160_MODE_LPM2_ADDR_FAST_POWERUP,
				0);
				data3  = BMG160_SET_BITSLICE(data2,
				BMG160_MODE_LPM2_ADDR_ADV_POWERSAVING,
				0);
				comres += p_bmg160->bus_write
				(p_bmg160->dev_addr,
			BMG160_MODE_LPM1_ADDR, &data1,
			1);
			p_bmg160->delay(BMG160_POWER_MODE_DELAY);
			/*A minimum delay of at least
			450us is required for Multiple write.*/
			comres += p_bmg160->bus_write
				(p_bmg160->dev_addr,
			BMG160_MODE_LPM2_ADDR, &data3,
			1);
				break;
			case BMG160_MODE_SUSPEND:
				data1  = BMG160_SET_BITSLICE(data1,
				BMG160_MODE_LPM1, BMG160_BIT_MASK_MODE_LPM1);
				data2  = BMG160_SET_BITSLICE(data2,
				BMG160_MODE_LPM2_ADDR_FAST_POWERUP,
				0);
				data3  = BMG160_SET_BITSLICE(data2,
				BMG160_MODE_LPM2_ADDR_ADV_POWERSAVING,
				0);
				comres += p_bmg160->bus_write
				(p_bmg160->dev_addr,
			BMG160_MODE_LPM1_ADDR, &data1,
			1);
			p_bmg160->delay(BMG160_POWER_MODE_DELAY);
			/*A minimum delay of at least
			450us is required for Multiple write.*/
			comres += p_bmg160->bus_write
				(p_bmg160->dev_addr,
			BMG160_MODE_LPM2_ADDR, &data3,
			1);
				break;
			case BMG160_MODE_FASTPOWERUP:
				data1  = BMG160_SET_BITSLICE(data1,
				BMG160_MODE_LPM1, BMG160_BIT_MASK_MODE_LPM1);
				data2  = BMG160_SET_BITSLICE(data2,
				BMG160_MODE_LPM2_ADDR_FAST_POWERUP,
				1);
				data3  = BMG160_SET_BITSLICE(data2,
				BMG160_MODE_LPM2_ADDR_ADV_POWERSAVING,
				0);
				comres += p_bmg160->bus_write
				(p_bmg160->dev_addr,
			BMG160_MODE_LPM1_ADDR, &data1,
			1);
			p_bmg160->delay(BMG160_POWER_MODE_DELAY);
			/*A minimum delay of at least
			450us is required for Multiple write.*/
			comres += p_bmg160->bus_write
			(p_bmg160->dev_addr,
			BMG160_MODE_LPM2_ADDR, &data3,
			1);
				break;
			case BMG160_MODE_ADVANCEDPOWERSAVING:
				/* Configuring the proper settings for auto
				sleep duration */
				bmg160_get_bw(&v_bw_u8r);
				bmg160_get_auto_sleep_durn(
					&v_autosleepduration);
				bmg160_set_auto_sleep_durn(v_autosleepduration,
				v_bw_u8r);
				comres += p_bmg160->bus_read
					(p_bmg160->dev_addr,
				BMG160_MODE_LPM2_ADDR, &data2,
				1);
				/* Configuring the advanced power saving mode*/
				data1  = BMG160_SET_BITSLICE(data1,
				BMG160_MODE_LPM1, 0);
				data2  = BMG160_SET_BITSLICE(data2,
				BMG160_MODE_LPM2_ADDR_FAST_POWERUP,
				0);
				data3  = BMG160_SET_BITSLICE(data2,
				BMG160_MODE_LPM2_ADDR_ADV_POWERSAVING,
				1);
				comres += p_bmg160->bus_write
				(p_bmg160->dev_addr,
			BMG160_MODE_LPM1_ADDR, &data1,
			1);
			p_bmg160->delay(BMG160_POWER_MODE_DELAY);
			/*A minimum delay of at least
			450us is required for Multiple write.*/
			comres += p_bmg160->bus_write
				(p_bmg160->dev_addr,
			BMG160_MODE_LPM2_ADDR, &data3,
			1);
				break;
				}
		} else {
		comres = BSTDR_E_OUT_OF_RANGE;
		}
	}
	return comres;
}
/*!
 *	@brief This API is used to to do selftest to sensor
 *	sensor in the register 0x3C
 *
 *
 *
 *
 *  @param v_result_u8: The value of self test
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
bstdr_ret_t bmg160_selftest(uint8_t *v_result_u8)
	{
	/* variable used to return the bus communication status*/
	bstdr_ret_t comres = BSTDR_E_GEN_ERROR;
	uint8_t v_data1_u8 = 0;
	uint8_t v_data2_u8 = 0;

	comres = p_bmg160->bus_read(p_bmg160->dev_addr,
	BMG160_SELFTEST_ADDR, &v_data1_u8,
	1);
	v_data2_u8  = BMG160_GET_BITSLICE(v_data1_u8,
	BMG160_SELFTEST_ADDR_RATEOK);
	v_data1_u8  = BMG160_SET_BITSLICE(v_data1_u8,
	BMG160_SELFTEST_ADDR_TRIGBIST,
	1);
	comres += p_bmg160->bus_write(p_bmg160->dev_addr,
	BMG160_SELFTEST_ADDR_TRIGBIST__REG, &v_data1_u8,
	1);

	/* Waiting time to complete the selftest process */
	p_bmg160->delay(BMG160_SELFTEST_DELAY);

	/* Reading Selftest v_result_u8 bir bist_failure */
	comres += p_bmg160->bus_read(p_bmg160->dev_addr,
	BMG160_SELFTEST_ADDR_BISTFAIL__REG, &v_data1_u8,
	1);
	v_data1_u8  = BMG160_GET_BITSLICE(v_data1_u8,
	BMG160_SELFTEST_ADDR_BISTFAIL);
	if ((v_data1_u8 == BMG160_SELFTEST_BISTFAIL) &&
	(v_data2_u8 == BMG160_SELFTEST_RATEOK))
		*v_result_u8 = C_BMG160_SUCCESS;
	else
		*v_result_u8 = C_BMG160_FAILURE;
	return comres;
}
/*!
 *	@brief  This API is used to get the auto sleep duration
 *	in the register 0x12 bit 0 to 2
 *
 *
 *
 *  @param v_durn_u8 : The value of gyro auto sleep duration
 *           sleep duration     |   value
 *  ----------------------------|----------
 *	             not allowed    |   0
 *	             4ms            |   1
 *	             5ms            |   2
 *	             8ms            |   3
 *	             10ms           |   4
 *	             15ms           |   5
 *	             20ms           |   6
 *	             40ms           |   7
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
bstdr_ret_t bmg160_get_auto_sleep_durn(uint8_t *v_durn_u8)
{
	/* variable used to return the bus communication status*/
	bstdr_ret_t comres = BSTDR_E_GEN_ERROR;
	uint8_t v_data_u8 = 0;
	/* check the p_bmg160 struct pointer is NULL*/
	if (p_bmg160 == BMG160_NULL) {
		return  BSTDR_E_NULL_PTR;
		} else {
		/* read auto sleep duration*/
		comres = p_bmg160->bus_read(p_bmg160->dev_addr,
		 BMG160_MODE_LPM2_ADDR_AUTO_SLEEP_DURN__REG,
		 &v_data_u8, 1);
		*v_durn_u8 = BMG160_GET_BITSLICE(v_data_u8,
		BMG160_MODE_LPM2_ADDR_AUTO_SLEEP_DURN);
	}
	return comres;
}
/*!
 *	@brief  This API is used to set the auto sleep duration
 *	in the register 0x12 bit 0 to 2
 *
 *
 *
 *  @param v_durn_u8 : The value of gyro auto sleep duration
 *           sleep duration     |   value
 *  ----------------------------|----------
 *	             not allowed    |   0
 *	             4ms            |   1
 *	             5ms            |   2
 *	             8ms            |   3
 *	             10ms           |   4
 *	             15ms           |   5
 *	             20ms           |   6
 *	             40ms           |   7
 *
 *	@param v_bw_u8 : The value of selected bandwidth
 *        v_bw_u8               |   value
 *  ----------------------------|----------
 *	C_BMG160_NO_FILTER_U8X      |   0
 *	C_BMG160_BW_230HZ_U8X       |   1
 *	C_BMG160_BW_116HZ_u8X       |   2
 *	C_BMG160_BW_47HZ_u8X        |   3
 *	C_BMG160_BW_23HZ_u8X        |   4
 *	C_BMG160_BW_12HZ_u8X        |   5
 *	C_BMG160_BW_64HZ_u8X        |   6
 *	C_BMG160_BW_32HZ_u8X        |   7
 *
 *	@note: sleep duration depends on selected power mode and bandwidth
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
bstdr_ret_t bmg160_set_auto_sleep_durn(uint8_t v_durn_u8,uint8_t v_bw_u8)
{
/* variable used to return the bus communication status*/
	bstdr_ret_t comres = BSTDR_E_GEN_ERROR;
uint8_t v_data_u8 = 0;
uint8_t v_auto_sleep_durn_u8r = 0;
/* check the p_bmg160 struct pointer is NULL*/
if (p_bmg160 == BMG160_NULL) {
	return  BSTDR_E_NULL_PTR;
} else {
	/* write auto sleep duration*/
	comres = p_bmg160->bus_read
		(p_bmg160->dev_addr,
		BMG160_MODE_LPM2_ADDR_AUTO_SLEEP_DURN__REG,
		&v_data_u8, 1);
		if (v_durn_u8 < BMG160_BIT_LENGTH_DURN) {
			switch (v_bw_u8) {
			case C_BMG160_NO_FILTER_U8X:
				if (v_durn_u8 >
				C_BMG160_4MS_AUTO_SLEEP_DURN_U8X)
					v_auto_sleep_durn_u8r =
					v_durn_u8;
				else
					v_auto_sleep_durn_u8r =
					C_BMG160_4MS_AUTO_SLEEP_DURN_U8X;
				break;
			case C_BMG160_BW_230HZ_U8X:
				if (v_durn_u8 >
				C_BMG160_4MS_AUTO_SLEEP_DURN_U8X)
					v_auto_sleep_durn_u8r =
					v_durn_u8;
				else
					v_auto_sleep_durn_u8r =
					C_BMG160_4MS_AUTO_SLEEP_DURN_U8X;
				break;
			case C_BMG160_BW_116HZ_U8X:
				if (v_durn_u8 >
				C_BMG160_4MS_AUTO_SLEEP_DURN_U8X)
					v_auto_sleep_durn_u8r =
					v_durn_u8;
				else
					v_auto_sleep_durn_u8r =
					C_BMG160_4MS_AUTO_SLEEP_DURN_U8X;
				break;
			case C_BMG160_BW_47HZ_U8X:
				if (v_durn_u8 >
				C_BMG160_5MS_AUTO_SLEEP_DURN_U8X)
					v_auto_sleep_durn_u8r =
					v_durn_u8;
				else
					v_auto_sleep_durn_u8r =
					C_BMG160_5MS_AUTO_SLEEP_DURN_U8X;
				break;
			case C_BMG160_BW_23HZ_U8X:
				if (v_durn_u8 >
				C_BMG160_10MS_AUTO_SLEEP_DURN_U8X)
					v_auto_sleep_durn_u8r =
					v_durn_u8;
				else
					v_auto_sleep_durn_u8r =
					C_BMG160_10MS_AUTO_SLEEP_DURN_U8X;
				break;
			case C_BMG160_BW_12HZ_U8X:
				if (v_durn_u8 >
				C_BMG160_20MS_AUTO_SLEEP_DURN_U8X)
					v_auto_sleep_durn_u8r =
					v_durn_u8;
				else
				v_auto_sleep_durn_u8r =
				C_BMG160_20MS_AUTO_SLEEP_DURN_U8X;
				break;
			case C_BMG160_BW_64HZ_U8X:
				if (v_durn_u8 >
				C_BMG160_10MS_AUTO_SLEEP_DURN_U8X)
					v_auto_sleep_durn_u8r =
					v_durn_u8;
				else
					v_auto_sleep_durn_u8r =
					C_BMG160_10MS_AUTO_SLEEP_DURN_U8X;
				break;
			case C_BMG160_BW_32HZ_U8X:
				if (v_durn_u8 >
				C_BMG160_20MS_AUTO_SLEEP_DURN_U8X)
					v_auto_sleep_durn_u8r =
					v_durn_u8;
				else
					v_auto_sleep_durn_u8r =
					C_BMG160_20MS_AUTO_SLEEP_DURN_U8X;
				break;
			default:
			if (v_durn_u8 >
				C_BMG160_4MS_AUTO_SLEEP_DURN_U8X)
				v_auto_sleep_durn_u8r =
					v_durn_u8;
				else
				v_auto_sleep_durn_u8r =
				C_BMG160_4MS_AUTO_SLEEP_DURN_U8X;
				break;
			}
		v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
		BMG160_MODE_LPM2_ADDR_AUTO_SLEEP_DURN,
		v_auto_sleep_durn_u8r);
		comres += p_bmg160->bus_write
			(p_bmg160->dev_addr,
		BMG160_MODE_LPM2_ADDR_AUTO_SLEEP_DURN__REG,
		&v_data_u8, 1);
	} else {
		comres = BSTDR_E_OUT_OF_RANGE;
	}
}
return comres;
}
/*!
 *	@brief  This API is used to get the sleep duration
 *	in the register 0x11 bit 1 to 3
 *
 *
 *
 *	@param v_durn_u8 : The value of sleep duration
 *           sleep duration     |   value
 *  ----------------------------|----------
 *	             2ms            |   0
 *	             4ms            |   1
 *	             5ms            |   2
 *	             8ms            |   3
 *	             10ms           |   4
 *	             15ms           |   5
 *	             18ms           |   6
 *	             20ms           |   7
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
bstdr_ret_t bmg160_get_sleep_durn(uint8_t *v_durn_u8)
{
	/* variable used to return the bus communication status*/
	bstdr_ret_t comres = BSTDR_E_GEN_ERROR;
	uint8_t v_data_u8 = 0;
	/* check the p_bmg160 struct pointer is NULL*/
	if (p_bmg160 == BMG160_NULL) {
		return  BSTDR_E_NULL_PTR;
		} else {
		/* read sleep duration */
		comres = p_bmg160->bus_read(p_bmg160->dev_addr,
		 BMG160_MODELPM1_ADDR_SLEEP_DURN__REG,
		 &v_data_u8, 1);
		*v_durn_u8 = BMG160_GET_BITSLICE(v_data_u8,
		BMG160_MODELPM1_ADDR_SLEEP_DURN);
	}
	return comres;
}
/*!
 *	@brief  This API is used to set the sleep duration
 *	in the register 0x11 bit 1 to 3
 *
 *
 *
 *	@param v_durn_u8 : The value of sleep duration
 *           sleep duration     |   value
 *  ----------------------------|----------
 *	             2ms            |   0
 *	             4ms            |   1
 *	             5ms            |   2
 *	             8ms            |   3
 *	             10ms           |   4
 *	             15ms           |   5
 *	             18ms           |   6
 *	             20ms           |   7
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
bstdr_ret_t bmg160_set_sleep_durn(uint8_t v_durn_u8){
	/* variable used to return the bus communication status*/
	bstdr_ret_t comres = BSTDR_E_GEN_ERROR;
	uint8_t v_data_u8 = 0;
	/* check the p_bmg160 struct pointer is NULL*/
	if (p_bmg160 == BMG160_NULL) {
		return  BSTDR_E_NULL_PTR;
		} else {
		if (v_durn_u8 < BMG160_BIT_LENGTH_DURN) {
			/* write sleep duration*/
			comres = p_bmg160->bus_read
			(p_bmg160->dev_addr,
			BMG160_MODELPM1_ADDR_SLEEP_DURN__REG,
			&v_data_u8, 1);
			v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
			BMG160_MODELPM1_ADDR_SLEEP_DURN, v_durn_u8);
			comres += p_bmg160->bus_write(p_bmg160->dev_addr,BMG160_MODELPM1_ADDR_SLEEP_DURN__REG,&v_data_u8, 1);
		} else {
			comres = BSTDR_E_OUT_OF_RANGE;
		}
	}
	return comres;
}

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
bstdr_ret_t bmg160_read_register(uint8_t v_addr_u8,
uint8_t *v_data_u8, uint8_t v_len_u8)
{
	/* variable used to return the bus communication status*/
	bstdr_ret_t comres = BSTDR_E_GEN_ERROR;
	/* check the p_bmg160 struct pointer is NULL*/
	if (p_bmg160 == BMG160_NULL) {
		return  BSTDR_E_NULL_PTR;
		} else {
		comres = p_bmg160->bus_read
		(p_bmg160->dev_addr, v_addr_u8, v_data_u8, v_len_u8);
	}
	return comres;
}
/*!
 * @brief
 *	This API reads the data from
 *	the given register
 *
 *
 *	@param v_addr_u8 -> Address of the register
 *	@param v_data_u8 -> The data from the register
 *	@param v_len_u32 -> no of bytes to read
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
bstdr_ret_t bmg160_burst_read(uint8_t v_addr_u8,
uint8_t *v_data_u8, uint32_t v_len_u32)
{
	/* variable used to return the bus communication status*/
	bstdr_ret_t comres = BSTDR_E_GEN_ERROR;
	/* check the p_bmg160 struct pointer is NULL*/
	if (p_bmg160 == BMG160_NULL) {
		return  BSTDR_E_NULL_PTR;
		} else {
		comres = p_bmg160->bus_read(p_bmg160->dev_addr,	v_addr_u8, v_data_u8, v_len_u32);
	}
	return comres;
}
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
bstdr_ret_t bmg160_write_register(uint8_t v_addr_u8,
uint8_t *v_data_u8, uint8_t v_len_u8)
{
	/* variable used to return the bus communication status*/
	bstdr_ret_t comres = BSTDR_E_GEN_ERROR;
	/* check the p_bmg160 struct pointer is NULL*/
	if (p_bmg160 == BMG160_NULL) {
		return  BSTDR_E_NULL_PTR;
		} else {
		comres = p_bmg160->bus_write
		(p_bmg160->dev_addr, v_addr_u8, v_data_u8, v_len_u8);
	}
	return comres;
}

