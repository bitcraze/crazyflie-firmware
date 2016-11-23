/*
****************************************************************************
* Copyright (C) 2014 Bosch Sensortec GmbH
*
* bmi160.c
* Date: 2014/11/13
* Revision: 2.0.4 $
*
* Usage: Sensor Driver for BMI160 sensor
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
/*! file <BMI160 >
    brief <Sensor driver for BMI160> */
#include "bmi160.h"

bmi160_t *p_bmi160;

/**************************************************/
/**\name	 FUNCTION DECLARATIONS  */
/*************************************************/


/**
 * Initialization Function
 * */
bmi160_ret_t bmi160_init(bmi160_t *bmi160_ptr){
	// initialization of the bmi160
	bmi160_ret_t com_rslt = BMI160_E_GEN_ERROR;
	uint8_t data[2] = {0};

	// check for NULL pointer
	if(bmi160_ptr !=0){
		// asign pointer
		p_bmi160 = bmi160_ptr;

		// read the chip id and check for plausibility
		com_rslt = p_bmi160->bus_read(p_bmi160->dev_addr,0x00, data, 1);

		if (BMI160_CHIP_ID == data[0]){
			p_bmi160->chip_id = data[0];
			com_rslt = BMI160_OK;
		} else com_rslt = BMI160_E_CHIPID_ERROR;
	} else
		com_rslt = BMI160_E_NULL_PTR;

	return com_rslt;
}

/** @brief check the connection (chip id register read out via bus)
 * */
bmi160_ret_t bmi160_check_connection(){
	bmi160_ret_t com_rslt = BMI160_E_GEN_ERROR;
	uint8_t data;
	if (p_bmi160 != 0) {

		// read the gyro xyz data
		com_rslt = p_bmi160->bus_read(p_bmi160->dev_addr, BMI160_USER_DATA_8_GYRO_X_LSB__REG,&data, 1);

		// check chip id
		if(BMI160_CHIP_ID == data){
			com_rslt = BMI160_OK;
		}
	}else
		com_rslt = BMI160_E_NULL_PTR;
	return com_rslt;
}

/** read gyro xyz data
 * */
bmi160_ret_t bmi160_read_gyro_xyz(bmi160_xyz_t *gyro){
	bmi160_ret_t com_rslt = BMI160_E_GEN_ERROR;
	uint8_t data[6] = {0};
	if (p_bmi160 != 0) {
		/* read the gyro xyz data*/
		com_rslt = p_bmi160->bus_read(p_bmi160->dev_addr, BMI160_USER_DATA_8_GYRO_X_LSB__REG, data, 6);
		gyro->x = (((int16_t)((int8_t)data[1]) << 8) | (data[0]));
		gyro->y = (((int16_t)((int8_t)data[3]) << 8) | (data[2]));
		gyro->z = (((int16_t)((int8_t)data[5]) << 8) | (data[4]));

		com_rslt = BMI160_OK;

	}else
		com_rslt = BMI160_E_NULL_PTR;
	return com_rslt;
}

/** read accel xyz
 * */
bmi160_ret_t bmi160_read_acc_xyz(bmi160_xyz_t *acc){
	bmi160_ret_t com_rslt = BMI160_E_GEN_ERROR;
	uint8_t data[6] = {0};
	if (p_bmi160 != 0) {
		/* read the gyro xyz data*/
		com_rslt = p_bmi160->bus_read(p_bmi160->dev_addr, BMI160_USER_DATA_14_ACCEL_X_LSB__REG,data, 6);
		acc->x = (((int16_t)((int8_t)data[1]) << 8) | (data[0]));
		acc->y = (((int16_t)((int8_t)data[3]) << 8) | (data[2]));
		acc->z = (((int16_t)((int8_t)data[5]) << 8) | (data[4]));
		com_rslt = BMI160_OK;
	}else com_rslt = BMI160_E_NULL_PTR;
	return com_rslt;
}

bmi160_ret_t bmi160_read_imu_data(bmi160_xyz_t *acc,bmi160_xyz_t *gyro){
	bmi160_ret_t com_rslt = BMI160_E_GEN_ERROR,tmp1,tmp2;
	tmp1 = bmi160_read_gyro_xyz(gyro);
	tmp2 = bmi160_read_acc_xyz(acc);
	if (tmp1 == BMI160_OK && tmp2==BMI160_OK){
		com_rslt = BMI160_OK;
	}
	return com_rslt;
}

// data rates
bmi160_ret_t bmi160_set_accel_output_datarate(uint8_t odr){
	bmi160_ret_t com_rslt = BMI160_E_GEN_ERROR;

	uint8_t tmp_data;
	uint8_t datarate;

	if (p_bmi160 != 0) {
		if (odr <= 0x0f) {
			p_bmi160->bus_read(p_bmi160->dev_addr, BMI160_USER_ACCEL_CONFIG_ADDR, &tmp_data, 1);
			datarate = BMI160_SET_BITSLICE(tmp_data, BMI160_USER_ACCEL_CONFIG_OUTPUT_DATA_RATE__POS,BMI160_USER_ACCEL_CONFIG_OUTPUT_DATA_RATE__LEN, odr);	// BIT_POS = 4, LEN = 4

			p_bmi160->bus_write(p_bmi160->dev_addr,	BMI160_USER_ACCEL_CONFIG_ADDR, &datarate, 1);
			com_rslt = BMI160_OK;
		} else com_rslt = BMI160_E_OUT_OF_RANGE;
	} else com_rslt = BMI160_E_NULL_PTR;
	return com_rslt;
}


bmi160_ret_t bmi160_set_accel_bw(uint8_t bw){
	bmi160_ret_t com_rslt = BMI160_OK;
	uint8_t data_bw, tmp_data;

	if (p_bmi160 != 0) {
		if (bw <= 0x07) {
			p_bmi160->bus_read(p_bmi160->dev_addr, BMI160_USER_ACCEL_CONFIG_ADDR, &tmp_data,1);
			data_bw = BMI160_SET_BITSLICE(tmp_data, BMI160_USER_ACCEL_CONFIG_ACCEL_BW__POS,BMI160_USER_ACCEL_CONFIG_ACCEL_BW__LEN, bw);	// BIT_POS = 0, LEN = 4

			p_bmi160->bus_write(p_bmi160->dev_addr, BMI160_USER_ACCEL_CONFIG_ADDR,&data_bw, 1);
			com_rslt = BMI160_OK;
		} else com_rslt = BMI160_E_OUT_OF_RANGE;
	} else com_rslt = BMI160_E_NULL_PTR;
	return com_rslt;
}

/** accelerometer range configuration
 * */
bmi160_ret_t bmi160_set_accel_range(uint8_t range){
	bmi160_ret_t com_rslt = BMI160_OK;
	uint8_t data_range, tmp_data;

	if (p_bmi160 != 0) {
		if (range <= 0x0f) {
			p_bmi160->bus_read(p_bmi160->dev_addr, BMI160_USER_ACCEL_RANGE_ADDR, &tmp_data, 1);
			data_range = BMI160_SET_BITSLICE(tmp_data, BMI160_USER_ACCEL_RANGE__POS,BMI160_USER_ACCEL_RANGE__LEN, range);	// BIT_POS = 0, LEN = 4

			p_bmi160->bus_write(p_bmi160->dev_addr, BMI160_USER_ACCEL_RANGE_ADDR,&data_range, 1);
			com_rslt = BMI160_OK;
		} else com_rslt = BMI160_E_OUT_OF_RANGE;
	} else com_rslt = BMI160_E_NULL_PTR;
	return com_rslt;
}


/** set gyroscope output data rate
 * */
bmi160_ret_t bmi160_set_gyro_output_datarate(uint8_t odr){
	bmi160_ret_t com_rslt = BMI160_E_GEN_ERROR;

	uint8_t tmp_data;
	uint8_t datarate;

	if (p_bmi160 != 0) {
		if (odr <= 0x0f) {
			com_rslt = p_bmi160->bus_read(p_bmi160->dev_addr, BMI160_USER_GYRO_CONFIG_ADDR, &tmp_data, 1);
			datarate = BMI160_SET_BITSLICE(tmp_data, BMI160_USER_GYRO_CONFIG_OUTPUT_DATA_RATE__POS,BMI160_USER_GYRO_CONFIG_OUTPUT_DATA_RATE__LEN, odr);	// BIT_POS = 4, LEN = 4
			com_rslt = p_bmi160->bus_write(p_bmi160->dev_addr,	BMI160_USER_GYRO_CONFIG_ADDR, &datarate, 1);
		} else
			com_rslt = BMI160_E_OUT_OF_RANGE;
	} else
		com_rslt = BMI160_E_NULL_PTR;
	return com_rslt;
}

/** set gyro bandwidth
 * */
bmi160_ret_t bmi160_set_gyro_bw(uint8_t bw){
	bmi160_ret_t com_rslt = BMI160_E_GEN_ERROR;
	uint8_t data_bw, tmp_data;

	if (p_bmi160 != 0) {
		if (bw <= 0x03) {
			// read configuration
			p_bmi160->bus_read(p_bmi160->dev_addr, BMI160_USER_GYRO_CONFIG_ADDR, &tmp_data,1);
			data_bw = BMI160_SET_BITSLICE(tmp_data, BMI160_USER_GYRO_CONFIG_BW__POS,BMI160_USER_GYRO_CONFIG_BW__LEN, bw);	// BIT_POS = 0, LEN = 4

			// write updated configuration
			p_bmi160->bus_write(p_bmi160->dev_addr, BMI160_USER_GYRO_CONFIG_ADDR,&data_bw, 1);
			com_rslt = BMI160_OK;
		} else
			com_rslt = BMI160_E_OUT_OF_RANGE;
	} else
		com_rslt = BMI160_E_NULL_PTR;
	return com_rslt;
}

bmi160_ret_t bmi160_set_gyro_range(uint8_t range){
	bmi160_ret_t com_rslt = BMI160_E_GEN_ERROR;
	uint8_t data_range, tmp_data;

	if (p_bmi160 != 0) {
		if (range <= 0x05) {
			p_bmi160->bus_read(p_bmi160->dev_addr, BMI160_USER_GYRO_RANGE_ADDR, &tmp_data, 1);
			data_range = BMI160_SET_BITSLICE(tmp_data, BMI160_USER_GYRO_RANGE__POS,BMI160_USER_GYRO_RANGE__LEN, range);	// BIT_POS = 0, LEN = 3
			com_rslt = p_bmi160->bus_write(p_bmi160->dev_addr, BMI160_USER_GYRO_RANGE_ADDR,&data_range, 1);
		} else
			com_rslt = BMI160_E_OUT_OF_RANGE;
	} else
		com_rslt = BMI160_E_NULL_PTR;
	return com_rslt;
}

/** @brief sets command register of BMI160
 * */
bmi160_ret_t bmi160_set_command_register(uint8_t cmd){

	bmi160_ret_t com_rslt = BMI160_E_GEN_ERROR;
	if (p_bmi160 != 0) {
		com_rslt = p_bmi160->bus_write(p_bmi160->dev_addr, BMI160_CMD_COMMANDS_ADDR,&cmd, 1);
	} else
		com_rslt = BMI160_E_NULL_PTR;
	return com_rslt;
}

/** @brief read from register
 * */
bmi160_ret_t bmi160_read_reg(uint8_t reg_addr, uint8_t *reg){
	bmi160_ret_t com_rslt = BMI160_E_GEN_ERROR;
	if (p_bmi160 != 0) {
		com_rslt = p_bmi160->bus_read(p_bmi160->dev_addr, reg_addr, reg, 1);
	} else
		com_rslt = BMI160_E_NULL_PTR;
	return com_rslt;
}

/** @brief write to register
 * */
bmi160_ret_t bmi160_write_reg(uint8_t reg_addr, uint8_t reg){
	bmi160_ret_t com_rslt = BMI160_E_GEN_ERROR;
	if (p_bmi160 != 0) {
		com_rslt = p_bmi160->bus_write(p_bmi160->dev_addr, reg_addr, &reg, 1);
	} else
		com_rslt = BMI160_E_NULL_PTR;
	return com_rslt;
}



