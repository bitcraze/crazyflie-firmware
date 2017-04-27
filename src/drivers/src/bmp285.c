/*
 ****************************************************************************
 * Copyright (C) 2012 - 2015 Bosch Sensortec GmbH
 *
 * File : bmp285.c
 *
 * Date : 2016/06/30
 *
 * Revision : 2.0.9(Pressure and Temperature compensation code revision is 1.1)
 *
 * Usage: Sensor Driver for BMP285 sensor
 *
 ****************************************************************************
 *
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
 *
 **************************************************************************/
#include "bmp285.h"

static struct bmp285_t *p_bmp285; /**< pointer to BMP285 */

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
BMP285_RETURN_FUNCTION_TYPE bmp285_init(struct bmp285_t *bmp285)
{
	/* variable used to return communication result*/
	BMP285_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	uint8_t v_data_u8 = BMP285_INIT_VALUE;

	p_bmp285 = bmp285;/* assign BMP285 ptr */
	/* read chip id */
	com_rslt = p_bmp285->bus_read(p_bmp285->dev_addr,
	BMP285_CHIP_ID_REG, &v_data_u8,
	BMP285_GEN_READ_WRITE_DATA_LENGTH);/* read Chip Id */
	p_bmp285->chip_id = v_data_u8;
	/* readout bmp285 calibparam structure */
	com_rslt += bmp285_get_calib_param();
	return com_rslt;
}
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
BMP285_RETURN_FUNCTION_TYPE bmp285_read_uncomp_temperature(
		int32_t *v_uncomp_temperature_s32)
{
	/* variable used to return communication result*/
	BMP285_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* Array holding the MSB and LSb value
	 a_data_u8r[0] - Temperature MSB
	 a_data_u8r[1] - Temperature LSB
	 a_data_u8r[2] - Temperature LSB
	 */
	uint8_t a_data_u8r[BMP285_TEMPERATURE_DATA_SIZE] = {BMP285_INIT_VALUE,
			BMP285_INIT_VALUE, BMP285_INIT_VALUE};
	/* check the p_bmp285 structure pointer as NULL*/
	if (p_bmp285 == BMP285_NULL) {
		return  E_BMP285_NULL_PTR;
	} else {
		/* read temperature data */
		com_rslt = p_bmp285->bus_read(p_bmp285->dev_addr,
				BMP285_TEMPERATURE_MSB_REG, a_data_u8r,
				BMP285_TEMPERATURE_DATA_LENGTH);
		*v_uncomp_temperature_s32 = (int32_t)((((uint32_t)(
				a_data_u8r[BMP285_TEMPERATURE_MSB_DATA]))
				<< BMP285_SHIFT_BIT_POSITION_BY_12_BITS)
				| (((uint32_t)(
				a_data_u8r[BMP285_TEMPERATURE_LSB_DATA]))
				<< BMP285_SHIFT_BIT_POSITION_BY_04_BITS)
				| ((uint32_t)a_data_u8r[BMP285_TEMPERATURE_XLSB_DATA]
				>> BMP285_SHIFT_BIT_POSITION_BY_04_BITS));
	}
	return com_rslt;
}
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
int32_t bmp285_compensate_temperature_s32(int32_t v_uncomp_temperature_s32)
{
	int32_t v_x1_u32r = BMP285_INIT_VALUE;
	int32_t v_x2_u32r = BMP285_INIT_VALUE;
	int32_t temperature = BMP285_INIT_VALUE;
	/* calculate true temperature*/
	/*calculate x1*/
	v_x1_u32r = ((((v_uncomp_temperature_s32
			>> BMP285_SHIFT_BIT_POSITION_BY_03_BITS)
			- ((int32_t)p_bmp285->calib_param.dig_T1
			<< BMP285_SHIFT_BIT_POSITION_BY_01_BIT)))
			* ((int32_t)p_bmp285->calib_param.dig_T2))
			>> BMP285_SHIFT_BIT_POSITION_BY_11_BITS;
	/*calculate x2*/
	v_x2_u32r = (((((v_uncomp_temperature_s32
			>> BMP285_SHIFT_BIT_POSITION_BY_04_BITS)
			- ((int32_t)p_bmp285->calib_param.dig_T1))
			* ((v_uncomp_temperature_s32
			>> BMP285_SHIFT_BIT_POSITION_BY_04_BITS)
			- ((int32_t)p_bmp285->calib_param.dig_T1)))
			>> BMP285_SHIFT_BIT_POSITION_BY_12_BITS)
			* ((int32_t)p_bmp285->calib_param.dig_T3))
			>> BMP285_SHIFT_BIT_POSITION_BY_14_BITS;
	/*calculate t_fine*/
	p_bmp285->calib_param.t_fine = v_x1_u32r + v_x2_u32r;
	/*calculate temperature*/
	temperature = (p_bmp285->calib_param.t_fine * 5 + 128)
			>> BMP285_SHIFT_BIT_POSITION_BY_08_BITS;

	return temperature;
}
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
BMP285_RETURN_FUNCTION_TYPE bmp285_read_uncomp_pressure(
		int32_t *v_uncomp_pressure_s32)
{
	/* variable used to return communication result*/
	BMP285_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* Array holding the MSB and LSb value
	 a_data_u8[0] - Pressure MSB
	 a_data_u8[1] - Pressure LSB
	 a_data_u8[2] - Pressure LSB
	 */
	uint8_t a_data_u8[BMP285_PRESSURE_DATA_SIZE] = {BMP285_INIT_VALUE,
			BMP285_INIT_VALUE, BMP285_INIT_VALUE};
	/* check the p_bmp285 structure pointer as NULL*/
	if (p_bmp285 == BMP285_NULL) {
		return  E_BMP285_NULL_PTR;
	} else {
		com_rslt = p_bmp285->bus_read(p_bmp285->dev_addr,
				BMP285_PRESSURE_MSB_REG, a_data_u8,
				BMP285_PRESSURE_DATA_LENGTH);
		*v_uncomp_pressure_s32 = (int32_t)((((uint32_t)(
				a_data_u8[BMP285_PRESSURE_MSB_DATA]))
				<< BMP285_SHIFT_BIT_POSITION_BY_12_BITS)
				| (((uint32_t)(a_data_u8[BMP285_PRESSURE_LSB_DATA]))
				<< BMP285_SHIFT_BIT_POSITION_BY_04_BITS)
				| ((uint32_t)a_data_u8[BMP285_PRESSURE_XLSB_DATA]
				>> BMP285_SHIFT_BIT_POSITION_BY_04_BITS));
	}
	return com_rslt;
}
/*!
 *	@brief Reads actual pressure from uncompensated pressure
 *	and returns the value in Pascal(Pa)
 *	@note Output value of "96386" equals 96386 Pa =
 *	963.86 hPa = 963.86 millibar
 *
 *
 *
 *
 *  @param  v_uncomp_pressure_s32: value of uncompensated pressure
 *
 *
 *
 *  @return Returns the Actual pressure out put as int32_t
 *
 */
uint32_t bmp285_compensate_pressure_s32(int32_t v_uncomp_pressure_s32)
{
	int32_t v_x1_u32r = BMP285_INIT_VALUE;
	int32_t v_x2_u32r = BMP285_INIT_VALUE;
	int32_t v_x3_u32r = BMP285_INIT_VALUE;
	uint32_t v_pressure_u32 = BMP285_INIT_VALUE;
	/* calculate x1*/
	v_x1_u32r = (((int32_t)p_bmp285->calib_param.t_fine)
			>> BMP285_SHIFT_BIT_POSITION_BY_01_BIT) - (int32_t)64000;
	/* calculate x2*/
	v_x2_u32r = (((v_x1_u32r >> BMP285_SHIFT_BIT_POSITION_BY_02_BITS)
			* (v_x1_u32r >> BMP285_SHIFT_BIT_POSITION_BY_02_BITS))
			>> BMP285_SHIFT_BIT_POSITION_BY_11_BITS)
			* ((int32_t)p_bmp285->calib_param.dig_P6);
	v_x2_u32r = v_x2_u32r + ((v_x1_u32r *
			((int32_t)p_bmp285->calib_param.dig_P5))
			<< BMP285_SHIFT_BIT_POSITION_BY_01_BIT);
	v_x2_u32r = (v_x2_u32r >> BMP285_SHIFT_BIT_POSITION_BY_02_BITS)
			+ (((int32_t)p_bmp285->calib_param.dig_P4)
			<< BMP285_SHIFT_BIT_POSITION_BY_16_BITS);
	/* calculate x1*/
	v_x1_u32r = (((p_bmp285->calib_param.dig_P3
			* (((v_x1_u32r
			>> BMP285_SHIFT_BIT_POSITION_BY_02_BITS) * (v_x1_u32r
			>> BMP285_SHIFT_BIT_POSITION_BY_02_BITS))
			>> BMP285_SHIFT_BIT_POSITION_BY_13_BITS))
			>> BMP285_SHIFT_BIT_POSITION_BY_03_BITS)
			+ ((((int32_t)p_bmp285->calib_param.dig_P2)
			* v_x1_u32r)
			>> BMP285_SHIFT_BIT_POSITION_BY_01_BIT))
			>> BMP285_SHIFT_BIT_POSITION_BY_18_BITS;
	v_x1_u32r = ((((32768 + v_x1_u32r))
			* ((int32_t)p_bmp285->calib_param.dig_P1))
			>> BMP285_SHIFT_BIT_POSITION_BY_15_BITS);
	/* calculate pressure*/
	v_pressure_u32 = (((uint32_t)(((int32_t)1048576) - v_uncomp_pressure_s32)
			- (v_x2_u32r >> BMP285_SHIFT_BIT_POSITION_BY_12_BITS)))
			* 3125;
	/* check overflow*/
	if (v_pressure_u32 < 0x80000000)
		/* Avoid exception caused by division by zero */
		if (v_x1_u32r != BMP285_INIT_VALUE)
			v_pressure_u32 = (v_pressure_u32
					<< BMP285_SHIFT_BIT_POSITION_BY_01_BIT)
					/ ((uint32_t)v_x1_u32r);
		else
			return BMP285_INVALID_DATA;
	else
	/* Avoid exception caused by division by zero */
	if (v_x1_u32r != BMP285_INIT_VALUE)
		v_pressure_u32 = (v_pressure_u32 / (uint32_t)v_x1_u32r) * 2;
	else
		return BMP285_INVALID_DATA;
	/* calculate x1*/
	v_x1_u32r = (((int32_t)p_bmp285->calib_param.dig_P9) * ((int32_t)(
			((v_pressure_u32
			>> BMP285_SHIFT_BIT_POSITION_BY_03_BITS)
			* (v_pressure_u32
			>> BMP285_SHIFT_BIT_POSITION_BY_03_BITS))
			>> BMP285_SHIFT_BIT_POSITION_BY_13_BITS)))
			>> BMP285_SHIFT_BIT_POSITION_BY_12_BITS;
	/* calculate x2*/
	v_x2_u32r = (((int32_t)(v_pressure_u32 >>
			BMP285_SHIFT_BIT_POSITION_BY_02_BITS))
			* ((int32_t)p_bmp285->calib_param.dig_P8))
			>> BMP285_SHIFT_BIT_POSITION_BY_13_BITS;
	/* calculate x3*/
	v_x3_u32r = (int32_t)(((int32_t)(((v_pressure_u32
			>> BMP285_SHIFT_BIT_POSITION_BY_03_BITS) *
			(v_pressure_u32 >> BMP285_SHIFT_BIT_POSITION_BY_03_BITS))
			>> BMP285_SHIFT_BIT_POSITION_BY_08_BITS) *
			((int32_t)((((int32_t)p_bmp285->calib_param.dig_P10)
			* (int32_t)v_pressure_u32)
			>> BMP285_SHIFT_BIT_POSITION_BY_14_BITS)))
			>> BMP285_SHIFT_BIT_POSITION_BY_13_BITS);
	/* calculate true pressure*/
	v_pressure_u32 = (uint32_t)((int32_t)v_pressure_u32 +
			((v_x1_u32r + v_x2_u32r + v_x3_u32r
			+ p_bmp285->calib_param.dig_P7)
			>> BMP285_SHIFT_BIT_POSITION_BY_04_BITS));

	return v_pressure_u32;
}
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
		int32_t *v_uncomp_pressure_s32, int32_t *v_uncomp_temperature_s32)
{
	/* variable used to return communication result*/
	BMP285_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* Array holding the temperature and pressure data
	 a_data_u8[0] - Pressure MSB
	 a_data_u8[1] - Pressure LSB
	 a_data_u8[2] - Pressure LSB
	 a_data_u8[3] - Temperature MSB
	 a_data_u8[4] - Temperature LSB
	 a_data_u8[5] - Temperature LSB
	 */
	uint8_t a_data_u8[BMP285_ALL_DATA_FRAME_LENGTH] = {BMP285_INIT_VALUE,
			BMP285_INIT_VALUE, BMP285_INIT_VALUE, BMP285_INIT_VALUE,
			BMP285_INIT_VALUE, BMP285_INIT_VALUE};
	/* check the p_bmp285 structure pointer as NULL*/
	if (p_bmp285 == BMP285_NULL) {
		return  E_BMP285_NULL_PTR;
	} else {
		com_rslt = p_bmp285->bus_read(p_bmp285->dev_addr,
				BMP285_PRESSURE_MSB_REG, a_data_u8,
				BMP285_DATA_FRAME_SIZE);
		/*Pressure*/
		*v_uncomp_pressure_s32 = (int32_t)((((uint32_t)(
				a_data_u8[BMP285_DATA_FRAME_PRESSURE_MSB_BYTE]))
				<< BMP285_SHIFT_BIT_POSITION_BY_12_BITS)
				| (((uint32_t)(
				a_data_u8[BMP285_DATA_FRAME_PRESSURE_LSB_BYTE]))
				<< BMP285_SHIFT_BIT_POSITION_BY_04_BITS)
				| ((uint32_t)a_data_u8[
				BMP285_DATA_FRAME_PRESSURE_XLSB_BYTE]
				>> BMP285_SHIFT_BIT_POSITION_BY_04_BITS));

		/* Temperature */
		*v_uncomp_temperature_s32 = (int32_t)((((uint32_t)(a_data_u8[
				BMP285_DATA_FRAME_TEMPERATURE_MSB_BYTE]))
				<< BMP285_SHIFT_BIT_POSITION_BY_12_BITS)
				| (((uint32_t)(a_data_u8[
				BMP285_DATA_FRAME_TEMPERATURE_LSB_BYTE]))
				<< BMP285_SHIFT_BIT_POSITION_BY_04_BITS)
				| ((uint32_t)a_data_u8[
				BMP285_DATA_FRAME_TEMPERATURE_XLSB_BYTE]
				>> BMP285_SHIFT_BIT_POSITION_BY_04_BITS));
	}
	return com_rslt;
}
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
		uint32_t *v_pressure_u32,int32_t *v_temperature_s32)
{
	/* variable used to return communication result*/
	BMP285_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	int32_t v_uncomp_pressure_s32 = BMP285_INIT_VALUE;
	int32_t v_uncomp_temperature_s32 = BMP285_INIT_VALUE;
	/* check the p_bmp285 structure pointer as NULL*/
	if (p_bmp285 == BMP285_NULL) {
		return  E_BMP285_NULL_PTR;
	} else {
		/* read uncompensated pressure and temperature*/
		com_rslt = bmp285_read_uncomp_pressure_temperature(
				&v_uncomp_pressure_s32,
				&v_uncomp_temperature_s32);
		/* read true pressure and temperature*/
		*v_temperature_s32 = bmp285_compensate_temperature_s32(
				v_uncomp_temperature_s32);
		*v_pressure_u32 = bmp285_compensate_pressure_s32(
				v_uncomp_pressure_s32);
	}
	return com_rslt;
}
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
BMP285_RETURN_FUNCTION_TYPE bmp285_get_calib_param(void)
{
	/* variable used to return communication result*/
	BMP285_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	uint8_t a_data_u8[BMP285_CALIB_DATA_SIZE] = {
	BMP285_INIT_VALUE,
	BMP285_INIT_VALUE, BMP285_INIT_VALUE, BMP285_INIT_VALUE,
	BMP285_INIT_VALUE, BMP285_INIT_VALUE, BMP285_INIT_VALUE,
	BMP285_INIT_VALUE, BMP285_INIT_VALUE, BMP285_INIT_VALUE,
	BMP285_INIT_VALUE, BMP285_INIT_VALUE, BMP285_INIT_VALUE,
	BMP285_INIT_VALUE, BMP285_INIT_VALUE, BMP285_INIT_VALUE,
	BMP285_INIT_VALUE, BMP285_INIT_VALUE, BMP285_INIT_VALUE,
	BMP285_INIT_VALUE, BMP285_INIT_VALUE, BMP285_INIT_VALUE,
	BMP285_INIT_VALUE, BMP285_INIT_VALUE, BMP285_INIT_VALUE};
	/* check the p_bmp285 structure pointer as NULL*/
	if (p_bmp285 == BMP285_NULL) {
		return  E_BMP285_NULL_PTR;
	} else {
		com_rslt = p_bmp285->bus_read(p_bmp285->dev_addr,
				BMP285_TEMPERATURE_CALIB_DIG_T1_LSB_REG,
				a_data_u8,
				BMP285_PRESSURE_TEMPERATURE_CALIB_DATA_LENGTH);
		/* read calibration values*/
		p_bmp285->calib_param.dig_T1 = (uint16_t)((((uint16_t)((uint8_t)a_data_u8[
					BMP285_TEMPERATURE_CALIB_DIG_T1_MSB]))
					<< BMP285_SHIFT_BIT_POSITION_BY_08_BITS)
					| a_data_u8[
					BMP285_TEMPERATURE_CALIB_DIG_T1_LSB]);
		p_bmp285->calib_param.dig_T2 = (int16_t)((((int16_t)((int8_t)a_data_u8[
					BMP285_TEMPERATURE_CALIB_DIG_T2_MSB]))
					<< BMP285_SHIFT_BIT_POSITION_BY_08_BITS)
					| a_data_u8[
					BMP285_TEMPERATURE_CALIB_DIG_T2_LSB]);
		p_bmp285->calib_param.dig_T3 = (int16_t)((((int16_t)((int8_t)a_data_u8[
					BMP285_TEMPERATURE_CALIB_DIG_T3_MSB]))
					<< BMP285_SHIFT_BIT_POSITION_BY_08_BITS)
					| a_data_u8[
					BMP285_TEMPERATURE_CALIB_DIG_T3_LSB]);
		p_bmp285->calib_param.dig_P1 = (uint16_t)((((uint16_t)((uint8_t)a_data_u8[
					BMP285_PRESSURE_CALIB_DIG_P1_MSB]))
					<< BMP285_SHIFT_BIT_POSITION_BY_08_BITS)
					| a_data_u8[
					BMP285_PRESSURE_CALIB_DIG_P1_LSB]);
		p_bmp285->calib_param.dig_P2 = (int16_t)((((int16_t)((int8_t)a_data_u8[
					BMP285_PRESSURE_CALIB_DIG_P2_MSB]))
					<< BMP285_SHIFT_BIT_POSITION_BY_08_BITS)
					| a_data_u8[
					BMP285_PRESSURE_CALIB_DIG_P2_LSB]);
		p_bmp285->calib_param.dig_P3 = (int16_t)((((int16_t)((int8_t)a_data_u8[
					BMP285_PRESSURE_CALIB_DIG_P3_MSB]))
					<< BMP285_SHIFT_BIT_POSITION_BY_08_BITS)
					| a_data_u8[
					BMP285_PRESSURE_CALIB_DIG_P3_LSB]);
		p_bmp285->calib_param.dig_P4 = (int16_t)((((int16_t)((int8_t)a_data_u8[
					BMP285_PRESSURE_CALIB_DIG_P4_MSB]))
					<< BMP285_SHIFT_BIT_POSITION_BY_08_BITS)
					| a_data_u8[
					BMP285_PRESSURE_CALIB_DIG_P4_LSB]);
		p_bmp285->calib_param.dig_P5 = (int16_t)((((int16_t)((int8_t)a_data_u8[
					BMP285_PRESSURE_CALIB_DIG_P5_MSB]))
					<< BMP285_SHIFT_BIT_POSITION_BY_08_BITS)
					| a_data_u8[
					BMP285_PRESSURE_CALIB_DIG_P5_LSB]);
		p_bmp285->calib_param.dig_P6 = (int16_t)((((int16_t)((int8_t)a_data_u8[
					BMP285_PRESSURE_CALIB_DIG_P6_MSB]))
					<< BMP285_SHIFT_BIT_POSITION_BY_08_BITS)
					| a_data_u8[
					BMP285_PRESSURE_CALIB_DIG_P6_LSB]);
		p_bmp285->calib_param.dig_P7 = (int16_t)((((int16_t)((int8_t)a_data_u8[
					BMP285_PRESSURE_CALIB_DIG_P7_MSB]))
					<< BMP285_SHIFT_BIT_POSITION_BY_08_BITS)
					| a_data_u8[
					BMP285_PRESSURE_CALIB_DIG_P7_LSB]);
		p_bmp285->calib_param.dig_P8 = (int16_t)((((int16_t)((int8_t)a_data_u8[
					BMP285_PRESSURE_CALIB_DIG_P8_MSB]))
					<< BMP285_SHIFT_BIT_POSITION_BY_08_BITS)
					| a_data_u8[
					BMP285_PRESSURE_CALIB_DIG_P8_LSB]);
		p_bmp285->calib_param.dig_P9 = (int16_t)((((int16_t)((int8_t)a_data_u8[
					BMP285_PRESSURE_CALIB_DIG_P9_MSB]))
					<< BMP285_SHIFT_BIT_POSITION_BY_08_BITS)
					| a_data_u8[
					BMP285_PRESSURE_CALIB_DIG_P9_LSB]);
		p_bmp285->calib_param.dig_P10 =
				(int8_t)(a_data_u8[BMP285_PRESSURE_CALIB_DIG_P10]);
	}
	return com_rslt;
}
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
BMP285_RETURN_FUNCTION_TYPE bmp285_get_oversamp_temperature(uint8_t *v_value_u8)
{
	/* variable used to return communication result*/
	BMP285_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	uint8_t v_data_u8 = BMP285_INIT_VALUE;
	/* check the p_bmp285 structure pointer as NULL*/
	if (p_bmp285 == BMP285_NULL) {
		return  E_BMP285_NULL_PTR;
	} else {
		/* read temperature over sampling*/
		com_rslt = p_bmp285->bus_read(p_bmp285->dev_addr,
				BMP285_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__REG,
				&v_data_u8, BMP285_GEN_READ_WRITE_DATA_LENGTH);
		*v_value_u8 = BMP285_GET_BITSLICE(v_data_u8,
				BMP285_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE);
		/* assign temperature oversampling*/
		p_bmp285->oversamp_temperature = *v_value_u8;
	}
	return com_rslt;
}
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
BMP285_RETURN_FUNCTION_TYPE bmp285_set_oversamp_temperature(uint8_t v_value_u8)
{
	/* variable used to return communication result*/
	BMP285_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	uint8_t v_data_u8 = BMP285_INIT_VALUE;
	/* check the p_bmp285 structure pointer as NULL*/
	if (p_bmp285 == BMP285_NULL) {
		return  E_BMP285_NULL_PTR;
	} else {
		com_rslt = p_bmp285->bus_read(p_bmp285->dev_addr,
				BMP285_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__REG,
				&v_data_u8, BMP285_GEN_READ_WRITE_DATA_LENGTH);
		if (com_rslt == SUCCESS) {
			/* write over sampling*/
			v_data_u8 = BMP285_SET_BITSLICE(v_data_u8,
				BMP285_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE,
				v_value_u8);
			com_rslt += p_bmp285->bus_write(
				p_bmp285->dev_addr,
				BMP285_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__REG,
				&v_data_u8,
				BMP285_GEN_READ_WRITE_DATA_LENGTH);
			p_bmp285->oversamp_temperature = v_value_u8;
		}
	}
	return com_rslt;
}
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
BMP285_RETURN_FUNCTION_TYPE bmp285_get_oversamp_pressure(uint8_t *v_value_u8)
{
	/* variable used to return communication result*/
	BMP285_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	uint8_t v_data_u8 = BMP285_INIT_VALUE;
	/* check the p_bmp285 structure pointer as NULL*/
	if (p_bmp285 == BMP285_NULL) {
		return  E_BMP285_NULL_PTR;
	} else {
		/* read pressure over sampling */
		com_rslt = p_bmp285->bus_read(p_bmp285->dev_addr,
				BMP285_CTRL_MEAS_REG_OVERSAMP_PRESSURE__REG,
				&v_data_u8, BMP285_GEN_READ_WRITE_DATA_LENGTH);
		*v_value_u8 = BMP285_GET_BITSLICE(v_data_u8,
				BMP285_CTRL_MEAS_REG_OVERSAMP_PRESSURE);

		p_bmp285->oversamp_pressure = *v_value_u8;
	}
	return com_rslt;
}
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
BMP285_RETURN_FUNCTION_TYPE bmp285_set_oversamp_pressure(uint8_t v_value_u8)
{
	/* variable used to return communication result*/
	BMP285_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	uint8_t v_data_u8 = BMP285_INIT_VALUE;
	/* check the p_bmp285 structure pointer as NULL*/
	if (p_bmp285 == BMP285_NULL) {
		return  E_BMP285_NULL_PTR;
	} else {
		com_rslt = p_bmp285->bus_read(p_bmp285->dev_addr,
				BMP285_CTRL_MEAS_REG_OVERSAMP_PRESSURE__REG,
				&v_data_u8, BMP285_GEN_READ_WRITE_DATA_LENGTH);
		if (com_rslt == SUCCESS) {
			/* write pressure over sampling */
			v_data_u8 = BMP285_SET_BITSLICE(v_data_u8,
					BMP285_CTRL_MEAS_REG_OVERSAMP_PRESSURE,
					v_value_u8);
			com_rslt += p_bmp285->bus_write(
				p_bmp285->dev_addr,
				BMP285_CTRL_MEAS_REG_OVERSAMP_PRESSURE__REG,
				&v_data_u8,
				BMP285_GEN_READ_WRITE_DATA_LENGTH);

			p_bmp285->oversamp_pressure = v_value_u8;
		}
	}
	return com_rslt;
}
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
BMP285_RETURN_FUNCTION_TYPE bmp285_get_power_mode(uint8_t *v_power_mode_u8)
{
	/* variable used to return communication result*/
	BMP285_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	uint8_t v_mode_u8 = BMP285_INIT_VALUE;
	/* check the p_bmp285 structure pointer as NULL*/
	if (p_bmp285 == BMP285_NULL) {
		return  E_BMP285_NULL_PTR;
	} else {
		/* read the power mode*/
		com_rslt = p_bmp285->bus_read(p_bmp285->dev_addr,
				BMP285_CTRL_MEAS_REG_POWER_MODE__REG,
				&v_mode_u8, BMP285_GEN_READ_WRITE_DATA_LENGTH);
		*v_power_mode_u8 = BMP285_GET_BITSLICE(v_mode_u8,
				BMP285_CTRL_MEAS_REG_POWER_MODE);
	}
	return com_rslt;
}
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
BMP285_RETURN_FUNCTION_TYPE bmp285_set_power_mode(uint8_t v_power_mode_u8)
{
	/* variable used to return communication result*/
	BMP285_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	uint8_t v_mode_u8 = BMP285_INIT_VALUE;
	/* check the p_bmp285 structure pointer as NULL*/
	if (p_bmp285 == BMP285_NULL) {
		return  E_BMP285_NULL_PTR;
	} else {
		if (v_power_mode_u8 <= BMP285_NORMAL_MODE) {
			/* write the power mode*/
			v_mode_u8 = (p_bmp285->oversamp_temperature
					<< BMP285_SHIFT_BIT_POSITION_BY_05_BITS)
					+ (p_bmp285->oversamp_pressure
					<< BMP285_SHIFT_BIT_POSITION_BY_02_BITS)
					+ v_power_mode_u8;
			com_rslt = p_bmp285->bus_write(
					p_bmp285->dev_addr,
					BMP285_CTRL_MEAS_REG_POWER_MODE__REG,
					&v_mode_u8,
					BMP285_GEN_READ_WRITE_DATA_LENGTH);
		} else {
			com_rslt = E_BMP285_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/*!
 * @brief Used to reset the sensor
 * The value 0xB6 is written to the 0xE0 register
 * the device is reset using the
 * complete power-on-reset procedure.
 * Soft reset can be easily set using bmp285_set_softreset().
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
BMP285_RETURN_FUNCTION_TYPE bmp285_set_soft_rst(void)
{
	/* variable used to return communication result*/
	BMP285_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	uint8_t v_data_u8 = BMP285_SOFT_RESET_CODE;
	/* check the p_bmp285 structure pointer as NULL*/
	if (p_bmp285 == BMP285_NULL) {
		return  E_BMP285_NULL_PTR;
	} else {
		/* write soft reset */
		com_rslt = p_bmp285->bus_write(p_bmp285->dev_addr,
				BMP285_RST_REG, &v_data_u8,
				BMP285_GEN_READ_WRITE_DATA_LENGTH);
	}
	return com_rslt;
}
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
BMP285_RETURN_FUNCTION_TYPE bmp285_get_spi3(uint8_t *v_enable_disable_u8)
{
	/* variable used to return communication result*/
	BMP285_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	uint8_t v_data_u8 = BMP285_INIT_VALUE;
	/* check the p_bmp285 structure pointer as NULL*/
	if (p_bmp285 == BMP285_NULL) {
		return  E_BMP285_NULL_PTR;
	} else {
		com_rslt = p_bmp285->bus_read(p_bmp285->dev_addr,
				BMP285_CONFIG_REG_SPI3_ENABLE__REG, &v_data_u8,
				BMP285_GEN_READ_WRITE_DATA_LENGTH);
		*v_enable_disable_u8 = BMP285_GET_BITSLICE(v_data_u8,
				BMP285_CONFIG_REG_SPI3_ENABLE);
	}
	return com_rslt;
}
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
BMP285_RETURN_FUNCTION_TYPE bmp285_set_spi3(uint8_t v_enable_disable_u8)
{
	/* variable used to return communication result*/
	BMP285_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	uint8_t v_data_u8 = BMP285_INIT_VALUE;
	/* check the p_bmp285 structure pointer as NULL*/
	if (p_bmp285 == BMP285_NULL) {
		return  E_BMP285_NULL_PTR;
	} else {
		com_rslt = p_bmp285->bus_read(p_bmp285->dev_addr,
				BMP285_CONFIG_REG_SPI3_ENABLE__REG, &v_data_u8,
				BMP285_GEN_READ_WRITE_DATA_LENGTH);
		if (com_rslt == SUCCESS) {
			v_data_u8 = BMP285_SET_BITSLICE(v_data_u8,
					BMP285_CONFIG_REG_SPI3_ENABLE,
					v_enable_disable_u8);
			com_rslt += p_bmp285->bus_write(
					p_bmp285->dev_addr,
					BMP285_CONFIG_REG_SPI3_ENABLE__REG,
					&v_data_u8,
					BMP285_GEN_READ_WRITE_DATA_LENGTH);
		}
	}
	return com_rslt;
}
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
BMP285_RETURN_FUNCTION_TYPE bmp285_get_filter(uint8_t *v_value_u8)
{
	/* variable used to return communication result*/
	BMP285_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	uint8_t v_data_u8 = BMP285_INIT_VALUE;
	/* check the p_bmp285 structure pointer as NULL*/
	if (p_bmp285 == BMP285_NULL) {
		return  E_BMP285_NULL_PTR;
	} else {
		/* read filter*/
		com_rslt = p_bmp285->bus_read(p_bmp285->dev_addr,
				BMP285_CONFIG_REG_FILTER__REG, &v_data_u8,
				BMP285_GEN_READ_WRITE_DATA_LENGTH);
		*v_value_u8 = BMP285_GET_BITSLICE(v_data_u8,
				BMP285_CONFIG_REG_FILTER);
	}
	return com_rslt;
}
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
BMP285_RETURN_FUNCTION_TYPE bmp285_set_filter(uint8_t v_value_u8)
{
	BMP285_RETURN_FUNCTION_TYPE com_rslt = SUCCESS;
	uint8_t v_data_u8 = BMP285_INIT_VALUE;
	/* check the p_bmp285 structure pointer as NULL*/
	if (p_bmp285 == BMP285_NULL) {
		return  E_BMP285_NULL_PTR;
	} else {
		/* write filter*/
		com_rslt = p_bmp285->bus_read(p_bmp285->dev_addr,
				BMP285_CONFIG_REG_FILTER__REG, &v_data_u8,
				BMP285_GEN_READ_WRITE_DATA_LENGTH);
		if (com_rslt == SUCCESS) {
			v_data_u8 = BMP285_SET_BITSLICE(v_data_u8,
					BMP285_CONFIG_REG_FILTER, v_value_u8);
			com_rslt += p_bmp285->bus_write(
					p_bmp285->dev_addr,
					BMP285_CONFIG_REG_FILTER__REG,
					&v_data_u8,
					BMP285_GEN_READ_WRITE_DATA_LENGTH);
		}
	}
	return com_rslt;
}
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
BMP285_RETURN_FUNCTION_TYPE bmp285_get_standby_durn(uint8_t *v_standby_durn_u8)
{
	/* variable used to return communication result*/
	BMP285_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	uint8_t v_data_u8 = BMP285_INIT_VALUE;
	/* check the p_bmp285 structure pointer as NULL*/
	if (p_bmp285 == BMP285_NULL) {
		return  E_BMP285_NULL_PTR;
	} else {
		/* read the standby duration*/
		com_rslt = p_bmp285->bus_read(p_bmp285->dev_addr,
				BMP285_CONFIG_REG_STANDBY_DURN__REG, &v_data_u8,
				BMP285_GEN_READ_WRITE_DATA_LENGTH);
		*v_standby_durn_u8 = BMP285_GET_BITSLICE(v_data_u8,
				BMP285_CONFIG_REG_STANDBY_DURN);
	}
	return com_rslt;
}
/*!
 *	@brief This API used to Read the
 *	standby duration time from the sensor
 *	in the register 0xF5 bit 5 to 7
 *	@note Normal mode comprises an
 *	automated perpetual cycling between an (active)
 *	Measurement period and an (inactive) standby period.
 *	@note The standby time is determined
 *	by the contents of the register t_sb.
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
BMP285_RETURN_FUNCTION_TYPE bmp285_set_standby_durn(uint8_t v_standby_durn_u8)
{
	/* variable used to return communication result*/
	BMP285_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	uint8_t v_data_u8 = BMP285_INIT_VALUE;
	/* check the p_bmp285 structure pointer as NULL*/
	if (p_bmp285 == BMP285_NULL) {
		return  E_BMP285_NULL_PTR;
	} else {
		/* write the standby duration*/
		com_rslt = p_bmp285->bus_read(p_bmp285->dev_addr,
				BMP285_CONFIG_REG_STANDBY_DURN__REG, &v_data_u8,
				BMP285_GEN_READ_WRITE_DATA_LENGTH);
		if (com_rslt == SUCCESS) {
			v_data_u8 = BMP285_SET_BITSLICE(v_data_u8,
					BMP285_CONFIG_REG_STANDBY_DURN,
					v_standby_durn_u8);
			com_rslt += p_bmp285->bus_write(
					p_bmp285->dev_addr,
					BMP285_CONFIG_REG_STANDBY_DURN__REG,
					&v_data_u8,
					BMP285_GEN_READ_WRITE_DATA_LENGTH);
		}
	}
	return com_rslt;
}
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
BMP285_RETURN_FUNCTION_TYPE bmp285_set_work_mode(uint8_t v_work_mode_u8)
{
	/* variable used to return communication result*/
	BMP285_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	uint8_t v_data_u8 = BMP285_INIT_VALUE;
	/* check the p_bmp285 structure pointer as NULL*/
	if (p_bmp285 == BMP285_NULL) {
		return  E_BMP285_NULL_PTR;
	} else {
	if (v_work_mode_u8 <= BMP285_ULTRA_HIGH_RESOLUTION_MODE) {
		com_rslt = p_bmp285->bus_read(
				p_bmp285->dev_addr,
				BMP285_CTRL_MEAS_REG, &v_data_u8,
				BMP285_GEN_READ_WRITE_DATA_LENGTH);
		if (com_rslt == SUCCESS) {
			switch (v_work_mode_u8) {
			/* write work mode*/
			case BMP285_ULTRA_LOW_POWER_MODE:
				p_bmp285->oversamp_temperature =
				BMP285_ULTRALOWPOWER_OVERSAMP_TEMPERATURE;
				p_bmp285->oversamp_pressure =
					BMP285_ULTRALOWPOWER_OVERSAMP_PRESSURE;
				break;
			case BMP285_LOW_POWER_MODE:
				p_bmp285->oversamp_temperature =
					BMP285_LOWPOWER_OVERSAMP_TEMPERATURE;
				p_bmp285->oversamp_pressure =
					BMP285_LOWPOWER_OVERSAMP_PRESSURE;
				break;
			case BMP285_STANDARD_RESOLUTION_MODE:
				p_bmp285->oversamp_temperature =
				BMP285_STANDARDRESOLUTION_OVERSAMP_TEMPERATURE;
				p_bmp285->oversamp_pressure =
				BMP285_STANDARDRESOLUTION_OVERSAMP_PRESSURE;
				break;
			case BMP285_HIGH_RESOLUTION_MODE:
				p_bmp285->oversamp_temperature =
				BMP285_HIGHRESOLUTION_OVERSAMP_TEMPERATURE;
				p_bmp285->oversamp_pressure =
				BMP285_HIGHRESOLUTION_OVERSAMP_PRESSURE;
				break;
			case BMP285_ULTRA_HIGH_RESOLUTION_MODE:
				p_bmp285->oversamp_temperature =
				BMP285_ULTRAHIGHRESOLUTION_OVERSAMP_TEMPERATURE;
				p_bmp285->oversamp_pressure =
				BMP285_ULTRAHIGHRESOLUTION_OVERSAMP_PRESSURE;
				break;
			}
			v_data_u8 = BMP285_SET_BITSLICE(v_data_u8,
				BMP285_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE,
				p_bmp285->oversamp_temperature);
			v_data_u8 = BMP285_SET_BITSLICE(v_data_u8,
				BMP285_CTRL_MEAS_REG_OVERSAMP_PRESSURE,
				p_bmp285->oversamp_pressure);
			com_rslt += p_bmp285->bus_write(
				p_bmp285->dev_addr, BMP285_CTRL_MEAS_REG,
				&v_data_u8, BMP285_GEN_READ_WRITE_DATA_LENGTH);
		}
	} else {
		com_rslt = E_BMP285_OUT_OF_RANGE;
	}
	}
	return com_rslt;
}
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
		int32_t *v_uncomp_pressure_s32, int32_t *v_uncomp_temperature_s32)
{
	/* variable used to return communication result*/
	BMP285_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	uint8_t v_data_u8 = BMP285_INIT_VALUE;
	uint8_t v_waittime_u8 = BMP285_INIT_VALUE;
	/* check the p_bmp285 structure pointer as NULL*/
	if (p_bmp285 == BMP285_NULL) {
		return  E_BMP285_NULL_PTR;
	} else {
		/* read pressure and temperature*/
		v_data_u8 = (p_bmp285->oversamp_temperature
				<< BMP285_SHIFT_BIT_POSITION_BY_05_BITS)
				+ (p_bmp285->oversamp_pressure
				<< BMP285_SHIFT_BIT_POSITION_BY_02_BITS)
				+ BMP285_FORCED_MODE;
		com_rslt = p_bmp285->bus_write(p_bmp285->dev_addr,
				BMP285_CTRL_MEAS_REG, &v_data_u8,
				BMP285_GEN_READ_WRITE_DATA_LENGTH);
		bmp285_compute_wait_time(&v_waittime_u8);
		p_bmp285->delay_ms(v_waittime_u8);
		com_rslt += bmp285_read_uncomp_pressure_temperature(
				v_uncomp_pressure_s32,
				v_uncomp_temperature_s32);
	}
	return com_rslt;
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
BMP285_RETURN_FUNCTION_TYPE bmp285_write_register(uint8_t v_addr_u8, uint8_t *v_data_u8,
		uint8_t v_len_u8)
{
	/* variable used to return communication result*/
	BMP285_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* check the p_bmp285 structure pointer as NULL*/
	if (p_bmp285 == BMP285_NULL) {
		return  E_BMP285_NULL_PTR;
	} else {
		com_rslt = p_bmp285->bus_write(p_bmp285->dev_addr,
				v_addr_u8, v_data_u8, v_len_u8);
	}
	return com_rslt;
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
BMP285_RETURN_FUNCTION_TYPE bmp285_read_register(uint8_t v_addr_u8, uint8_t *v_data_u8,
		uint8_t v_len_u8)
{
	/* variable used to return communication result*/
	BMP285_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* check the p_bmp285 structure pointer as NULL*/
	if (p_bmp285 == BMP285_NULL) {
		return  E_BMP285_NULL_PTR;
	} else {
		com_rslt = p_bmp285->bus_read(p_bmp285->dev_addr,
				v_addr_u8, v_data_u8, v_len_u8);
	}
	return com_rslt;
}
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
float bmp285_compensate_temperature_float(int32_t v_uncomp_temperature_s32)
{
	float v_x1_u32r = BMP285_INIT_VALUE;
	float v_x2_u32r = BMP285_INIT_VALUE;
	float temperature = BMP285_INIT_VALUE;
	/* calculate x1*/
	v_x1_u32r = (((float)v_uncomp_temperature_s32) / 16384.0f -
			((float)p_bmp285->calib_param.dig_T1) / 1024.0f) *
	((float)p_bmp285->calib_param.dig_T2);
	/* calculate x2*/
	v_x2_u32r = ((((float)v_uncomp_temperature_s32) / 131072.0f -
			((float)p_bmp285->calib_param.dig_T1) / 8192.0f) *
			(((float)v_uncomp_temperature_s32) / 131072.0f -
			((float)p_bmp285->calib_param.dig_T1) / 8192.0f)) *
	((float)p_bmp285->calib_param.dig_T3);
	/* calculate t_fine*/
	p_bmp285->calib_param.t_fine = (int32_t)(v_x1_u32r + v_x2_u32r);
	/* calculate true pressure*/
	temperature = (v_x1_u32r + v_x2_u32r) / 5120.0f;

	return temperature;
}
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
float bmp285_compensate_pressure_float(int32_t v_uncomp_pressure_s32)
{
	float v_x1_u32r = BMP285_INIT_VALUE;
	float v_x2_u32r = BMP285_INIT_VALUE;
	float pressure = BMP285_INIT_VALUE;
	float v_pressure_diff_dbl = BMP285_INIT_VALUE;
	float v_x3_u32r = BMP285_INIT_VALUE;

	v_x1_u32r = ((float)p_bmp285->calib_param.t_fine/2.0f) - 64000.0f;
	v_x2_u32r = v_x1_u32r * v_x1_u32r *
	((float)p_bmp285->calib_param.dig_P6) / 32768.0f;
	v_x2_u32r = v_x2_u32r + v_x1_u32r *
	((float)p_bmp285->calib_param.dig_P5) * 2.0f;
	v_x2_u32r = (v_x2_u32r / 4.0f) +
	(((float)p_bmp285->calib_param.dig_P4) * 65536.0f);
	v_x1_u32r = (((float)p_bmp285->calib_param.dig_P3) *
		v_x1_u32r * v_x1_u32r / 524288.0f +
		((float)p_bmp285->calib_param.dig_P2) * v_x1_u32r) / 524288.0f;
	v_x1_u32r = (1.0f + v_x1_u32r / 32768.0f) *
	((float)p_bmp285->calib_param.dig_P1);
	pressure = 1048576.0f - (float)v_uncomp_pressure_s32;
	/* Avoid exception caused by division by zero */
	if ((v_x1_u32r > 0) || (v_x1_u32r < 0))
		pressure = (pressure - (v_x2_u32r / 4096.0f)) *
					6250.0f / v_x1_u32r;
	else
		return BMP285_INVALID_DATA;
		
	v_x1_u32r = ((float)p_bmp285->calib_param.dig_P9) *
	pressure * pressure / 2147483648.0f;
	v_x2_u32r = pressure * ((float)p_bmp285->calib_param.dig_P8) / 32768.0f;

	/* calculate different pressure*/
	v_pressure_diff_dbl = pressure / 256.0f;
	/* calculate x3*/
	v_x3_u32r = ((v_pressure_diff_dbl *
	v_pressure_diff_dbl * v_pressure_diff_dbl
	* (float)p_bmp285->calib_param.dig_P10)
	/ 131072.0f);
	/* calculate true pressure*/
	pressure = pressure + (v_x1_u32r + v_x2_u32r + v_x3_u32r +
	((float)p_bmp285->calib_param.dig_P7))
	/ 16.0f;

	return pressure;
}
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
uint32_t bmp285_compensate_pressure_int64(int32_t v_uncomp_pressure_s32)
{
	int64_t v_x1_s64r = BMP285_INIT_VALUE;
	int64_t v_x2_s64r = BMP285_INIT_VALUE;
	int64_t v_x3_s64r = BMP285_INIT_VALUE;
	int64_t pressure = BMP285_INIT_VALUE;
	/* calculate x1*/
	v_x1_s64r = ((int64_t)p_bmp285->calib_param.t_fine) - 128000;
	v_x2_s64r = v_x1_s64r * v_x1_s64r *
	(int64_t)p_bmp285->calib_param.dig_P6;
	/* calculate x2*/
	v_x2_s64r = v_x2_s64r + ((v_x1_s64r *
	(int64_t)p_bmp285->calib_param.dig_P5)
	<< BMP285_SHIFT_BIT_POSITION_BY_17_BITS);
	v_x2_s64r = v_x2_s64r +
	(((int64_t)p_bmp285->calib_param.dig_P4)
	<< BMP285_SHIFT_BIT_POSITION_BY_35_BITS);
	v_x1_s64r = ((v_x1_s64r * v_x1_s64r *
	(int64_t)p_bmp285->calib_param.dig_P3)
	>> BMP285_SHIFT_BIT_POSITION_BY_08_BITS) +
	((v_x1_s64r * (int64_t)p_bmp285->calib_param.dig_P2)
	<< BMP285_SHIFT_BIT_POSITION_BY_12_BITS);
	v_x1_s64r = (((((int64_t)1)
	<< BMP285_SHIFT_BIT_POSITION_BY_47_BITS) + v_x1_s64r)) *
	((int64_t)p_bmp285->calib_param.dig_P1)
	>> BMP285_SHIFT_BIT_POSITION_BY_33_BITS;
	pressure = 1048576 - v_uncomp_pressure_s32;
	if (v_x1_s64r != BMP285_INIT_VALUE)
		#if defined __KERNEL__
			pressure = div64_s64((((pressure
			<< BMP285_SHIFT_BIT_POSITION_BY_31_BITS) - v_x2_s64r)
			* 3125), v_x1_s64r);
		#else
			pressure = (((pressure
			<< BMP285_SHIFT_BIT_POSITION_BY_31_BITS) - v_x2_s64r)
			* 3125) / v_x1_s64r;
		#endif
	else
		return BMP285_INVALID_DATA;
	v_x1_s64r = (((int64_t)p_bmp285->calib_param.dig_P9) *
	(pressure >> BMP285_SHIFT_BIT_POSITION_BY_13_BITS) * (pressure
	>> BMP285_SHIFT_BIT_POSITION_BY_13_BITS))
	>> BMP285_SHIFT_BIT_POSITION_BY_25_BITS;
	v_x2_s64r = (((int64_t)p_bmp285->calib_param.dig_P8) *
	pressure) >> BMP285_SHIFT_BIT_POSITION_BY_19_BITS;
	/* calculate x3*/
	v_x3_s64r = (int64_t)(((int64_t)(((pressure
	>> BMP285_SHIFT_BIT_POSITION_BY_04_BITS)
	* (pressure >> BMP285_SHIFT_BIT_POSITION_BY_04_BITS))
	>> BMP285_SHIFT_BIT_POSITION_BY_30_BITS) * (int64_t)((pressure
	* (int64_t)p_bmp285->calib_param.dig_P10)
	>> BMP285_SHIFT_BIT_POSITION_BY_10_BITS))
	>> BMP285_SHIFT_BIT_POSITION_BY_29_BITS);
	/* calculate true pressure*/
	pressure = ((pressure + v_x1_s64r + v_x2_s64r + v_x3_s64r)
	>> BMP285_SHIFT_BIT_POSITION_BY_08_BITS) +
	(((int64_t)p_bmp285->calib_param.dig_P7)
	<< BMP285_SHIFT_BIT_POSITION_BY_04_BITS);

	return (uint32_t)pressure;
}
#endif
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
BMP285_RETURN_FUNCTION_TYPE bmp285_compute_wait_time(uint8_t *v_delaytime_u8r)
{
	/* variable used to return communication result*/
	BMP285_RETURN_FUNCTION_TYPE com_rslt = SUCCESS;

	*v_delaytime_u8r = (T_INIT_MAX + T_MEASURE_PER_OSRS_MAX * (((1
			<< p_bmp285->oversamp_temperature)
			>> BMP285_SHIFT_BIT_POSITION_BY_01_BIT)
			+ ((1 << p_bmp285->oversamp_pressure)
			>> BMP285_SHIFT_BIT_POSITION_BY_01_BIT))
			+ ((p_bmp285->oversamp_pressure > 0) ?
			T_SETUP_PRESSURE_MAX : 0) + 15) / 16;
	return com_rslt;
}
