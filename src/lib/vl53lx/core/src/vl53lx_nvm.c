
// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53LX and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */





#ifdef _MSC_VER
#define snprintf _snprintf
#endif



#include "vl53lx_ll_def.h"
#include "vl53lx_platform.h"
#include <vl53lx_platform_log.h>
#include "vl53lx_register_map.h"
#include "vl53lx_core.h"
#include "vl53lx_nvm_structs.h"
#include "vl53lx_nvm_map.h"
#include "vl53lx_nvm.h"



#define LOG_FUNCTION_START(fmt, ...) 
#define LOG_FUNCTION_END(status, ...) 
#define LOG_FUNCTION_END_FMT(status, fmt, ...) 

#define trace_print(level, ...) 

VL53LX_Error VL53LX_nvm_enable(
	VL53LX_DEV      Dev,
	uint16_t        nvm_ctrl_pulse_width,
	int32_t         nvm_power_up_delay_us)
{


	VL53LX_Error status = VL53LX_ERROR_NONE;

	LOG_FUNCTION_START("");




	if (status == VL53LX_ERROR_NONE)
		status = VL53LX_disable_firmware(Dev);




	if (status == VL53LX_ERROR_NONE)
		status = VL53LX_enable_powerforce(Dev);



	if (status == VL53LX_ERROR_NONE)
		status = VL53LX_WaitUs(
			Dev,
			VL53LX_ENABLE_POWERFORCE_SETTLING_TIME_US);



	if (status == VL53LX_ERROR_NONE)
		status = VL53LX_WrByte(
					Dev,
					VL53LX_RANGING_CORE__NVM_CTRL__PDN,
					0x01);



	if (status == VL53LX_ERROR_NONE)
		status = VL53LX_WrByte(
					Dev,
					VL53LX_RANGING_CORE__CLK_CTRL1,
					0x05);



	if (status == VL53LX_ERROR_NONE)
		status = VL53LX_WaitUs(
					Dev,
					nvm_power_up_delay_us);



	if (status == VL53LX_ERROR_NONE)
		status = VL53LX_WrByte(
					Dev,
					VL53LX_RANGING_CORE__NVM_CTRL__MODE,
					0x01);

	if (status == VL53LX_ERROR_NONE)
		status = VL53LX_WrWord(
			Dev,
			VL53LX_RANGING_CORE__NVM_CTRL__PULSE_WIDTH_MSB,
			nvm_ctrl_pulse_width);

	LOG_FUNCTION_END(status);

	return status;

}


VL53LX_Error VL53LX_nvm_read(
	VL53LX_DEV    Dev,
	uint8_t       start_address,
	uint8_t       count,
	uint8_t      *pdata)
{


	VL53LX_Error status   = VL53LX_ERROR_NONE;
	uint8_t      nvm_addr = 0;

	LOG_FUNCTION_START("");

	trace_print(
		   VL53LX_TRACE_LEVEL_INFO,
		   "%-12s = 0x%02X (%3u)\n",
		   "nvm_addr", nvm_addr, nvm_addr);

	trace_print(
		   VL53LX_TRACE_LEVEL_INFO,
		   "%-12s = 0x%02X (%3u)\n",
		   "count", count, count);

	for (nvm_addr = start_address;
		nvm_addr < (start_address+count) ; nvm_addr++) {



		if (status == VL53LX_ERROR_NONE)
			status = VL53LX_WrByte(
				Dev,
				VL53LX_RANGING_CORE__NVM_CTRL__ADDR,
				nvm_addr);



		if (status == VL53LX_ERROR_NONE)
			status = VL53LX_WrByte(
				Dev,
				VL53LX_RANGING_CORE__NVM_CTRL__READN,
				0x00);



		if (status == VL53LX_ERROR_NONE)
			status = VL53LX_WaitUs(
				Dev,
				VL53LX_NVM_READ_TRIGGER_DELAY_US);

		if (status == VL53LX_ERROR_NONE)
			status = VL53LX_WrByte(
				Dev,
				VL53LX_RANGING_CORE__NVM_CTRL__READN,
				0x01);


		if (status == VL53LX_ERROR_NONE)
			status = VL53LX_ReadMulti(
				Dev,
				VL53LX_RANGING_CORE__NVM_CTRL__DATAOUT_MMM,
				pdata,
				4);

		trace_print(
			VL53LX_TRACE_LEVEL_INFO,
			"NVM address : 0x%02X = 0x%02X%02X%02X%02X\n",
			nvm_addr, *pdata, *(pdata+1), *(pdata+2), *(pdata+3));



		pdata = pdata + 4;


	}

	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error VL53LX_nvm_disable(
	VL53LX_DEV    Dev)
{


	VL53LX_Error status = VL53LX_ERROR_NONE;

	LOG_FUNCTION_START("");

	if (status == VL53LX_ERROR_NONE)
		status = VL53LX_WrByte(
					Dev,
					VL53LX_RANGING_CORE__NVM_CTRL__READN,
					0x01);



	if (status == VL53LX_ERROR_NONE)
		status = VL53LX_WrByte(
					Dev,
					VL53LX_RANGING_CORE__NVM_CTRL__PDN,
					0x00);



	if (status == VL53LX_ERROR_NONE)
		status = VL53LX_disable_powerforce(Dev);



	if (status == VL53LX_ERROR_NONE)
		status = VL53LX_enable_firmware(Dev);

	LOG_FUNCTION_END(status);

	return status;

}


VL53LX_Error VL53LX_nvm_format_decode(
	uint16_t                   buf_size,
	uint8_t                   *pbuffer,
	VL53LX_decoded_nvm_data_t *pdata)
{



	VL53LX_Error status = VL53LX_ERROR_NONE;

	uint8_t    i        = 0;
	uint8_t   *ptmp     = NULL;
	int        pptmp[VL53LX_NVM_MAX_FMT_RANGE_DATA];

	LOG_FUNCTION_START("");

	if (buf_size < VL53LX_NVM_SIZE_IN_BYTES)
		return VL53LX_ERROR_BUFFER_TOO_SMALL;

	pdata->nvm__identification_model_id =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__IDENTIFICATION__MODEL_ID,
			0x000000FF,
			0,
			0);
	pdata->nvm__identification_module_type =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__IDENTIFICATION__MODULE_TYPE,
			0x000000FF,
			0,
			0);
	pdata->nvm__identification_revision_id =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__IDENTIFICATION__REVISION_ID,
			0x0000000F,
			0,
			0);
	pdata->nvm__identification_module_id =
		(uint16_t)VL53LX_i2c_decode_with_mask(
			2,
			pbuffer + VL53LX_NVM__IDENTIFICATION__MODULE_ID,
			0x0000FFFF,
			0,
			0);
	pdata->nvm__i2c_valid =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__I2C_VALID,
			0x000000FF,
			0,
			0);
	pdata->nvm__i2c_device_address_ews =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__I2C_SLAVE__DEVICE_ADDRESS,
			0x000000FF,
			0,
			0);
	pdata->nvm__ews__fast_osc_frequency =
		(uint16_t)VL53LX_i2c_decode_with_mask(
			2,
			pbuffer +
			VL53LX_NVM__EWS__OSC_MEASURED__FAST_OSC_FREQUENCY,
			0x0000FFFF,
			0,
			0);
	pdata->nvm__ews__fast_osc_trim_max =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__EWS__FAST_OSC_TRIM_MAX,
			0x0000007F,
			0,
			0);
	pdata->nvm__ews__fast_osc_freq_set =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__EWS__FAST_OSC_FREQ_SET,
			0x00000007,
			0,
			0);
	pdata->nvm__ews__slow_osc_calibration =
		(uint16_t)VL53LX_i2c_decode_with_mask(
			2,
			pbuffer + VL53LX_NVM__EWS__SLOW_OSC_CALIBRATION,
			0x000003FF,
			0,
			0);
	pdata->nvm__fmt__fast_osc_frequency =
		(uint16_t)VL53LX_i2c_decode_with_mask(
			2,
			pbuffer +
			VL53LX_NVM__FMT__OSC_MEASURED__FAST_OSC_FREQUENCY,
			0x0000FFFF,
			0,
			0);
	pdata->nvm__fmt__fast_osc_trim_max =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__FMT__FAST_OSC_TRIM_MAX,
			0x0000007F,
			0,
			0);
	pdata->nvm__fmt__fast_osc_freq_set =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__FMT__FAST_OSC_FREQ_SET,
			0x00000007,
			0,
			0);
	pdata->nvm__fmt__slow_osc_calibration =
		(uint16_t)VL53LX_i2c_decode_with_mask(
			2,
			pbuffer + VL53LX_NVM__FMT__SLOW_OSC_CALIBRATION,
			0x000003FF,
			0,
			0);
	pdata->nvm__vhv_config_unlock =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__VHV_CONFIG_UNLOCK,
			0x000000FF,
			0,
			0);
	pdata->nvm__ref_selvddpix =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__REF_SELVDDPIX,
			0x0000000F,
			0,
			0);
	pdata->nvm__ref_selvquench =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__REF_SELVQUENCH,
			0x00000078,
			3,
			0);
	pdata->nvm__regavdd1v2_sel =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__REGAVDD1V2_SEL_REGDVDD1V2_SEL,
			0x0000000C,
			2,
			0);
	pdata->nvm__regdvdd1v2_sel =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__REGAVDD1V2_SEL_REGDVDD1V2_SEL,
			0x00000003,
			0,
			0);
	pdata->nvm__vhv_timeout__macrop =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer +
			VL53LX_NVM__VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND,
			0x00000003,
			0,
			0);
	pdata->nvm__vhv_loop_bound =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer +
			VL53LX_NVM__VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND,
			0x000000FC,
			2,
			0);
	pdata->nvm__vhv_count_threshold =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__VHV_CONFIG__COUNT_THRESH,
			0x000000FF,
			0,
			0);
	pdata->nvm__vhv_offset =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__VHV_CONFIG__OFFSET,
			0x0000003F,
			0,
			0);
	pdata->nvm__vhv_init_enable =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__VHV_CONFIG__INIT,
			0x00000080,
			7,
			0);
	pdata->nvm__vhv_init_value =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__VHV_CONFIG__INIT,
			0x0000003F,
			0,
			0);
	pdata->nvm__laser_safety_vcsel_trim_ll =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__LASER_SAFETY__VCSEL_TRIM_LL,
			0x00000007,
			0,
			0);
	pdata->nvm__laser_safety_vcsel_selion_ll =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__LASER_SAFETY__VCSEL_SELION_LL,
			0x0000003F,
			0,
			0);
	pdata->nvm__laser_safety_vcsel_selion_max_ll =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__LASER_SAFETY__VCSEL_SELION_MAX_LL,
			0x0000003F,
			0,
			0);
	pdata->nvm__laser_safety_mult_ll =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__LASER_SAFETY__MULT_LL,
			0x0000003F,
			0,
			0);
	pdata->nvm__laser_safety_clip_ll =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__LASER_SAFETY__CLIP_LL,
			0x0000003F,
			0,
			0);
	pdata->nvm__laser_safety_vcsel_trim_ld =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__LASER_SAFETY__VCSEL_TRIM_LD,
			0x00000007,
			0,
			0);
	pdata->nvm__laser_safety_vcsel_selion_ld =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__LASER_SAFETY__VCSEL_SELION_LD,
			0x0000003F,
			0,
			0);
	pdata->nvm__laser_safety_vcsel_selion_max_ld =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__LASER_SAFETY__VCSEL_SELION_MAX_LD,
			0x0000003F,
			0,
			0);
	pdata->nvm__laser_safety_mult_ld =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__LASER_SAFETY__MULT_LD,
			0x0000003F,
			0,
			0);
	pdata->nvm__laser_safety_clip_ld =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__LASER_SAFETY__CLIP_LD,
			0x0000003F,
			0,
			0);
	pdata->nvm__laser_safety_lock_byte =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__LASER_SAFETY_LOCK_BYTE,
			0x000000FF,
			0,
			0);
	pdata->nvm__laser_safety_unlock_byte =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__LASER_SAFETY_UNLOCK_BYTE,
			0x000000FF,
			0,
			0);



	ptmp = pbuffer + VL53LX_NVM__EWS__SPAD_ENABLES_RTN_0_;
	for (i = 0 ; i < VL53LX_RTN_SPAD_BUFFER_SIZE ; i++)
		pdata->nvm__ews__spad_enables_rtn[i] = *ptmp++;

	ptmp = pbuffer + VL53LX_NVM__EWS__SPAD_ENABLES_REF__LOC1_0_;
	for (i = 0 ; i < VL53LX_REF_SPAD_BUFFER_SIZE ; i++)
		pdata->nvm__ews__spad_enables_ref__loc1[i] = *ptmp++;

	ptmp = pbuffer + VL53LX_NVM__EWS__SPAD_ENABLES_REF__LOC2_0_;
	for (i = 0 ; i < VL53LX_REF_SPAD_BUFFER_SIZE ; i++)
		pdata->nvm__ews__spad_enables_ref__loc2[i] = *ptmp++;

	ptmp = pbuffer + VL53LX_NVM__EWS__SPAD_ENABLES_REF__LOC3_0_;
	for (i = 0 ; i < VL53LX_REF_SPAD_BUFFER_SIZE ; i++)
		pdata->nvm__ews__spad_enables_ref__loc3[i] = *ptmp++;



	ptmp = pbuffer + VL53LX_NVM__FMT__SPAD_ENABLES_RTN_0_;
	for (i = 0 ; i < VL53LX_RTN_SPAD_BUFFER_SIZE ; i++)
		pdata->nvm__fmt__spad_enables_rtn[i] = *ptmp++;

	ptmp = pbuffer + VL53LX_NVM__FMT__SPAD_ENABLES_REF__LOC1_0_;
	for (i = 0 ; i < VL53LX_REF_SPAD_BUFFER_SIZE ; i++)
		pdata->nvm__fmt__spad_enables_ref__loc1[i] = *ptmp++;

	ptmp = pbuffer + VL53LX_NVM__FMT__SPAD_ENABLES_REF__LOC2_0_;
	for (i = 0 ; i < VL53LX_REF_SPAD_BUFFER_SIZE ; i++)
		pdata->nvm__fmt__spad_enables_ref__loc2[i] = *ptmp++;

	ptmp = pbuffer + VL53LX_NVM__FMT__SPAD_ENABLES_REF__LOC3_0_;
	for (i = 0 ; i < VL53LX_REF_SPAD_BUFFER_SIZE ; i++)
		pdata->nvm__fmt__spad_enables_ref__loc3[i] = *ptmp++;


	pdata->nvm__fmt__roi_config__mode_roi_centre_spad =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer +
			VL53LX_NVM__FMT__ROI_CONFIG__MODE_ROI_CENTRE_SPAD,
			0x000000FF,
			0,
			0);
	pdata->nvm__fmt__roi_config__mode_roi_x_size =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer +
			VL53LX_NVM__FMT__ROI_CONFIG__MODE_ROI_XY_SIZE,
			0x000000F0,
			4,
			0);
	pdata->nvm__fmt__roi_config__mode_roi_y_size =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__FMT__ROI_CONFIG__MODE_ROI_XY_SIZE,
			0x0000000F,
			0,
			0);
	pdata->nvm__fmt__ref_spad_apply__num_requested_ref_spad =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer +
			VL53LX_NVM__FMT__REF_SPAD_APPLY__NUM_REQUESTED_REF_SPAD,
			0x000000FF,
			0,
			0);
	pdata->nvm__fmt__ref_spad_man__ref_location =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__FMT__REF_SPAD_MAN__REF_LOCATION,
			0x00000003,
			0,
			0);
	pdata->nvm__fmt__mm_config__inner_offset_mm =
		(uint16_t)VL53LX_i2c_decode_with_mask(
			2,
			pbuffer + VL53LX_NVM__FMT__MM_CONFIG__INNER_OFFSET_MM,
			0x0000FFFF,
			0,
			0);
	pdata->nvm__fmt__mm_config__outer_offset_mm =
		(uint16_t)VL53LX_i2c_decode_with_mask(
			2,
			pbuffer + VL53LX_NVM__FMT__MM_CONFIG__OUTER_OFFSET_MM,
			0x0000FFFF,
			0,
			0);
	pdata->nvm__fmt__algo_part_to_part_range_offset_mm =
		(uint16_t)VL53LX_i2c_decode_with_mask(
			2,
			pbuffer +
			VL53LX_NVM__FMT__ALGO__PART_TO_PART_RANGE_OFFSET_MM,
			0x00000FFF,
			0,
			0);
	pdata->nvm__fmt__algo__crosstalk_compensation_plane_offset_kcps =
		(uint16_t)VL53LX_i2c_decode_with_mask(
		2,
		pbuffer +
		VL53LX_NVM__FMT__ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS,
		0x0000FFFF,
		0,
		0);
	pdata->nvm__fmt__algo__crosstalk_compensation_x_plane_gradient_kcps =
	(uint16_t)VL53LX_i2c_decode_with_mask(
	2,
	pbuffer +
	VL53LX_NVM__FMT__ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS,
	0x0000FFFF,
	0,
	0);
	pdata->nvm__fmt__algo__crosstalk_compensation_y_plane_gradient_kcps =
	(uint16_t)VL53LX_i2c_decode_with_mask(
	2,
	pbuffer +
	VL53LX_NVM__FMT__ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS,
	0x0000FFFF,
	0,
	0);
	pdata->nvm__fmt__spare__host_config__nvm_config_spare_0 =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer +
			VL53LX_NVM__FMT__SPARE_HOST_CONFIG__NVM_CONFIG_SPARE_0,
			0x000000FF,
			0,
			0);
	pdata->nvm__fmt__spare__host_config__nvm_config_spare_1 =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer +
			VL53LX_NVM__FMT__SPARE_HOST_CONFIG__NVM_CONFIG_SPARE_1,
			0x000000FF,
			0,
			0);
	pdata->nvm__customer_space_programmed =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__CUSTOMER_NVM_SPACE_PROGRAMMED,
			0x000000FF,
			0,
			0);
	pdata->nvm__cust__i2c_device_address =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__CUST__I2C_SLAVE__DEVICE_ADDRESS,
			0x000000FF,
			0,
			0);
	pdata->nvm__cust__ref_spad_apply__num_requested_ref_spad =
		(uint8_t)VL53LX_i2c_decode_with_mask(
		1,
		pbuffer +
		VL53LX_NVM__CUST__REF_SPAD_APPLY__NUM_REQUESTED_REF_SPAD,
		0x000000FF,
		0,
		0);
	pdata->nvm__cust__ref_spad_man__ref_location =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__CUST__REF_SPAD_MAN__REF_LOCATION,
			0x00000003,
			0,
			0);
	pdata->nvm__cust__mm_config__inner_offset_mm =
		(uint16_t)VL53LX_i2c_decode_with_mask(
			2,
			pbuffer + VL53LX_NVM__CUST__MM_CONFIG__INNER_OFFSET_MM,
			0x0000FFFF,
			0,
			0);
	pdata->nvm__cust__mm_config__outer_offset_mm =
		(uint16_t)VL53LX_i2c_decode_with_mask(
			2,
			pbuffer + VL53LX_NVM__CUST__MM_CONFIG__OUTER_OFFSET_MM,
			0x0000FFFF,
			0,
			0);
	pdata->nvm__cust__algo_part_to_part_range_offset_mm =
		(uint16_t)VL53LX_i2c_decode_with_mask(
			2,
			pbuffer +
			VL53LX_NVM__CUST__ALGO__PART_TO_PART_RANGE_OFFSET_MM,
			0x00000FFF,
			0,
			0);
	pdata->nvm__cust__algo__crosstalk_compensation_plane_offset_kcps =
	(uint16_t)VL53LX_i2c_decode_with_mask(
	2,
	pbuffer +
	VL53LX_NVM__CUST__ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS,
	0x0000FFFF,
	0,
	0);
	pdata->nvm__cust__algo__crosstalk_compensation_x_plane_gradient_kcps =
	(uint16_t)VL53LX_i2c_decode_with_mask(
	2,
	pbuffer +
	VL53LX_NVM__CUST__ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS,
	0x0000FFFF,
	0,
	0);
	pdata->nvm__cust__algo__crosstalk_compensation_y_plane_gradient_kcps =
	(uint16_t)VL53LX_i2c_decode_with_mask(
	2,
	pbuffer +
	VL53LX_NVM__CUST__ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS,
	0x0000FFFF,
	0,
	0);
	pdata->nvm__cust__spare__host_config__nvm_config_spare_0 =
	(uint8_t)VL53LX_i2c_decode_with_mask(
	1,
	pbuffer + VL53LX_NVM__CUST__SPARE_HOST_CONFIG__NVM_CONFIG_SPARE_0,
	0x000000FF,
	0,
	0);
	pdata->nvm__cust__spare__host_config__nvm_config_spare_1 =
		(uint8_t)VL53LX_i2c_decode_with_mask(
		1,
		pbuffer +
		VL53LX_NVM__CUST__SPARE_HOST_CONFIG__NVM_CONFIG_SPARE_1,
		0x000000FF,
		0,
		0);



	if (status == VL53LX_ERROR_NONE)
		status =
		VL53LX_nvm_decode_optical_centre(
			buf_size,
			pbuffer + VL53LX_NVM__FMT__OPTICAL_CENTRE_DATA_INDEX,
			&(pdata->fmt_optical_centre));



	if (status == VL53LX_ERROR_NONE)
		status =
		VL53LX_nvm_decode_cal_peak_rate_map(
			buf_size,
			pbuffer + VL53LX_NVM__FMT__CAL_PEAK_RATE_MAP_DATA_INDEX,
			&(pdata->fmt_peak_rate_map));



	if (status == VL53LX_ERROR_NONE)
		status =
		VL53LX_nvm_decode_additional_offset_cal_data(
			buf_size,
			pbuffer +
			VL53LX_NVM__FMT__ADDITIONAL_OFFSET_CAL_DATA_INDEX,
			&(pdata->fmt_add_offset_data));



	pptmp[0] = VL53LX_NVM__FMT__RANGE_RESULTS__140MM_MM_PRE_RANGE;
	pptmp[1] = VL53LX_NVM__FMT__RANGE_RESULTS__140MM_DARK;
	pptmp[2] = VL53LX_NVM__FMT__RANGE_RESULTS__400MM_DARK;
	pptmp[3] = VL53LX_NVM__FMT__RANGE_RESULTS__400MM_AMBIENT;

	for (i = 0 ; i < VL53LX_NVM_MAX_FMT_RANGE_DATA ; i++) {
		if (status == VL53LX_ERROR_NONE)
			status =
				VL53LX_nvm_decode_fmt_range_results_data(
					buf_size,
					pbuffer + pptmp[i],
					&(pdata->fmt_range_data[i]));
	}


	if (status == VL53LX_ERROR_NONE)
		status =
			VL53LX_nvm_decode_fmt_info(
				buf_size,
				pbuffer,
				&(pdata->fmt_info));

	if (status == VL53LX_ERROR_NONE)
		status =
			VL53LX_nvm_decode_ews_info(
				buf_size,
				pbuffer,
				&(pdata->ews_info));

	LOG_FUNCTION_END(status);

	return status;

}


VL53LX_Error VL53LX_nvm_decode_optical_centre(
	uint16_t                    buf_size,
	uint8_t                    *pbuffer,
	VL53LX_optical_centre_t    *pdata)
{

	VL53LX_Error status   = VL53LX_ERROR_NONE;

	uint16_t  tmp = 0;

	if (buf_size < VL53LX_NVM__FMT__OPTICAL_CENTRE_DATA_SIZE)
		return VL53LX_ERROR_BUFFER_TOO_SMALL;


	tmp  = 0x0100;
	tmp -= (uint16_t)*(pbuffer + 2);
	if (tmp > 0x0FF)
		tmp = 0;

	pdata->x_centre = (uint8_t)tmp;
	pdata->y_centre = *(pbuffer + 3);

	return status;
}


VL53LX_Error VL53LX_nvm_decode_cal_peak_rate_map(
	uint16_t                    buf_size,
	uint8_t                    *pbuffer,
	VL53LX_cal_peak_rate_map_t *pdata)
{

	VL53LX_Error status   = VL53LX_ERROR_NONE;

	uint8_t   *ptmp = NULL;
	uint8_t       i = 0;

	if (buf_size < VL53LX_NVM__FMT__CAL_PEAK_RATE_MAP_DATA_SIZE)
		return VL53LX_ERROR_BUFFER_TOO_SMALL;

	pdata->cal_distance_mm =
		(uint16_t)VL53LX_i2c_decode_uint16_t(2, pbuffer);

	pdata->cal_reflectance_pc =
		(uint16_t)VL53LX_i2c_decode_uint16_t(2, pbuffer + 2);
	pdata->cal_reflectance_pc =
		pdata->cal_reflectance_pc >> 6;

	pdata->max_samples = VL53LX_NVM_PEAK_RATE_MAP_SAMPLES;
	pdata->width       = VL53LX_NVM_PEAK_RATE_MAP_WIDTH;
	pdata->height      = VL53LX_NVM_PEAK_RATE_MAP_HEIGHT;

	ptmp = pbuffer + 4;
	for (i = 0 ; i < VL53LX_NVM_PEAK_RATE_MAP_SAMPLES ; i++) {
		pdata->peak_rate_mcps[i] =
			(uint16_t)VL53LX_i2c_decode_uint16_t(2, ptmp);
		ptmp += 2;
	}

	return status;
}


VL53LX_Error VL53LX_nvm_decode_additional_offset_cal_data(
	uint16_t                             buf_size,
	uint8_t                             *pbuffer,
	VL53LX_additional_offset_cal_data_t *pdata)
{

	VL53LX_Error status   = VL53LX_ERROR_NONE;

	if (buf_size < VL53LX_NVM__FMT__ADDITIONAL_OFFSET_CAL_DATA_SIZE)
		return VL53LX_ERROR_BUFFER_TOO_SMALL;

	pdata->result__mm_inner_actual_effective_spads =
		(uint16_t)VL53LX_i2c_decode_uint16_t(2, pbuffer);

	pdata->result__mm_outer_actual_effective_spads =
		(uint16_t)VL53LX_i2c_decode_uint16_t(2, pbuffer + 2);

	pdata->result__mm_inner_peak_signal_count_rtn_mcps =
		(uint16_t)VL53LX_i2c_decode_uint16_t(2, pbuffer + 4);

	pdata->result__mm_outer_peak_signal_count_rtn_mcps =
		(uint16_t)VL53LX_i2c_decode_uint16_t(2, pbuffer + 6);

	return status;
}


VL53LX_Error VL53LX_nvm_decode_fmt_range_results_data(
	uint16_t                             buf_size,
	uint8_t                             *pbuffer,
	VL53LX_decoded_nvm_fmt_range_data_t *pdata)
{

	VL53LX_Error status   = VL53LX_ERROR_NONE;

	if (buf_size < VL53LX_NVM__FMT__RANGE_RESULTS__SIZE_BYTES)
		return VL53LX_ERROR_BUFFER_TOO_SMALL;

	pdata->result__actual_effective_rtn_spads =
		(uint16_t)VL53LX_i2c_decode_uint16_t(2, pbuffer);

	pdata->ref_spad_array__num_requested_ref_spads =
		*(pbuffer+2);

	pdata->ref_spad_array__ref_location =
		*(pbuffer+3);

	pdata->result__peak_signal_count_rate_rtn_mcps =
		(uint16_t)VL53LX_i2c_decode_uint16_t(2, pbuffer + 4);

	pdata->result__ambient_count_rate_rtn_mcps =
		(uint16_t)VL53LX_i2c_decode_uint16_t(2, pbuffer + 6);

	pdata->result__peak_signal_count_rate_ref_mcps =
		(uint16_t)VL53LX_i2c_decode_uint16_t(2, pbuffer + 8);

	pdata->result__ambient_count_rate_ref_mcps =
		(uint16_t)VL53LX_i2c_decode_uint16_t(2, pbuffer + 10);

	pdata->measured_distance_mm =
		(uint16_t)VL53LX_i2c_decode_uint16_t(2, pbuffer + 12);

	pdata->measured_distance_stdev_mm =
		(uint16_t)VL53LX_i2c_decode_uint16_t(2, pbuffer + 14);

	return status;
}


VL53LX_Error VL53LX_nvm_decode_fmt_info(
	uint16_t                       buf_size,
	uint8_t                       *pbuffer,
	VL53LX_decoded_nvm_fmt_info_t *pdata)
{

	VL53LX_Error status   = VL53LX_ERROR_NONE;

	if (buf_size < VL53LX_NVM_SIZE_IN_BYTES)
		return VL53LX_ERROR_BUFFER_TOO_SMALL;

	pdata->nvm__fmt__fgc[0] =
		(char)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__FMT__FGC__BYTE_0,
			0x000000FE,
			1,
			0);
	pdata->nvm__fmt__fgc[1] =
		(char)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__FMT__FGC__BYTE_1,
			0x000001FC,
			2,
			0);
	pdata->nvm__fmt__fgc[2] =
		(char)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__FMT__FGC__BYTE_2 - 1,
			0x000003F8,
			3,
			0);
	pdata->nvm__fmt__fgc[3] =
		(char)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__FMT__FGC__BYTE_3 - 1,
			0x000007F0,
			4,
			0);
	pdata->nvm__fmt__fgc[4] =
		(char)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__FMT__FGC__BYTE_4 - 1,
			0x00000FE0,
			5,
			0);
	pdata->nvm__fmt__fgc[5] =
		(char)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__FMT__FGC__BYTE_5 - 1,
			0x00001FC0,
			6,
			0);
	pdata->nvm__fmt__fgc[6] =
		(char)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__FMT__FGC__BYTE_6 - 1,
			0x00003F80,
			7,
			0);
	pdata->nvm__fmt__fgc[7] =
		(char)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__FMT__FGC__BYTE_6,
			0x0000007F,
			0,
			0);
	pdata->nvm__fmt__fgc[8] =
		(char)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__FMT__FGC__BYTE_7,
			0x000000FE,
			1,
			0);
	pdata->nvm__fmt__fgc[9] =
		(char)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__FMT__FGC__BYTE_8,
			0x000001FC,
			2,
			0);
	pdata->nvm__fmt__fgc[10] =
		(char)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__FMT__FGC__BYTE_9 - 1,
			0x000003F8,
			3,
			0);
	pdata->nvm__fmt__fgc[11] =
		(char)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__FMT__FGC__BYTE_10 - 1,
			0x000007F0,
			4,
			0);
	pdata->nvm__fmt__fgc[12] =
		(char)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__FMT__FGC__BYTE_11 - 1,
			0x00000FE0,
			5,
			0);
	pdata->nvm__fmt__fgc[13] =
		(char)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__FMT__FGC__BYTE_12 - 1,
			0x00001FC0,
			6,
			0);
	pdata->nvm__fmt__fgc[14] =
		(char)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__FMT__FGC__BYTE_13 - 1,
			0x00003F80,
			7,
			0);
	pdata->nvm__fmt__fgc[15] =
		(char)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__FMT__FGC__BYTE_13,
			0x0000007F,
			0,
			0);
	pdata->nvm__fmt__fgc[16] =
		(char)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__FMT__FGC__BYTE_14,
			0x000000FE,
			1,
			0);
	pdata->nvm__fmt__fgc[17] =
		(char)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__FMT__FGC__BYTE_15,
			0x000001FC,
			2,
			0);
	pdata->nvm__fmt__fgc[18] = 0x00;

			pdata->nvm__fmt__test_program_major =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__FMT__TEST_PROGRAM_MAJOR_MINOR,
			0x000000E0,
			5,
			0);
	pdata->nvm__fmt__test_program_minor =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__FMT__TEST_PROGRAM_MAJOR_MINOR,
			0x0000001F,
			0,
			0);
	pdata->nvm__fmt__map_major =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__FMT__MAP_MAJOR_MINOR,
			0x000000E0,
			5,
			0);
	pdata->nvm__fmt__map_minor =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__FMT__MAP_MAJOR_MINOR,
			0x0000001F,
			0,
			0);
	pdata->nvm__fmt__year =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__FMT__YEAR_MONTH,
			0x000000F0,
			4,
			0);
	pdata->nvm__fmt__month =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__FMT__YEAR_MONTH,
			0x0000000F,
			0,
			0);
	pdata->nvm__fmt__day =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__FMT__DAY_MODULE_DATE_PHASE,
			0x000000F8,
			3,
			0);
	pdata->nvm__fmt__module_date_phase =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__FMT__DAY_MODULE_DATE_PHASE,
			0x00000007,
			0,
			0);
	pdata->nvm__fmt__time =
		(uint16_t)VL53LX_i2c_decode_with_mask(
			2,
			pbuffer + VL53LX_NVM__FMT__TIME,
			0x0000FFFF,
			0,
			0);
	pdata->nvm__fmt__tester_id =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__FMT__TESTER_ID,
			0x000000FF,
			0,
			0);
	pdata->nvm__fmt__site_id =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__FMT__SITE_ID,
			0x000000FF,
			0,
			0);

	return status;
}


VL53LX_Error VL53LX_nvm_decode_ews_info(
	uint16_t                       buf_size,
	uint8_t                       *pbuffer,
	VL53LX_decoded_nvm_ews_info_t *pdata)
{

	VL53LX_Error status   = VL53LX_ERROR_NONE;

	if (buf_size < VL53LX_NVM_SIZE_IN_BYTES)
		return VL53LX_ERROR_BUFFER_TOO_SMALL;

	pdata->nvm__ews__test_program_major =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__EWS__TEST_PROGRAM_MAJOR_MINOR,
			0x000000E0,
			5,
			0);
	pdata->nvm__ews__test_program_minor =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__EWS__TEST_PROGRAM_MAJOR_MINOR,
			0x0000001F,
			0,
			0);
	pdata->nvm__ews__probe_card_major =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__EWS__PROBE_CARD_MAJOR_MINOR,
			0x000000F0,
			4,
			0);
	pdata->nvm__ews__probe_card_minor =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__EWS__PROBE_CARD_MAJOR_MINOR,
			0x0000000F,
			0,
			0);
	pdata->nvm__ews__tester_id =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__EWS__TESTER_ID,
			0x000000FF,
			0,
			0);
	pdata->nvm__ews__lot[0] =
		(char)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__EWS__LOT__BYTE_0,
			0x000000FC,
			2,
			32);
	pdata->nvm__ews__lot[1] =
		(char)VL53LX_i2c_decode_with_mask(
			2,
			pbuffer + VL53LX_NVM__EWS__LOT__BYTE_1 - 1,
			0x000003F0,
			4,
			32);
	pdata->nvm__ews__lot[2] =
		(char)VL53LX_i2c_decode_with_mask(
			2,
			pbuffer + VL53LX_NVM__EWS__LOT__BYTE_2 - 1,
			0x00000FC0,
			6,
			32);
	pdata->nvm__ews__lot[3] =
		(char)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__EWS__LOT__BYTE_2,
			0x0000003F,
			0,
			32);
	pdata->nvm__ews__lot[4] =
		(char)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__EWS__LOT__BYTE_3,
			0x000000FC,
			2,
			32);
	pdata->nvm__ews__lot[5] =
		(char)VL53LX_i2c_decode_with_mask(
			2,
			pbuffer + VL53LX_NVM__EWS__LOT__BYTE_4 - 1,
			0x000003F0,
			4,
			32);
	pdata->nvm__ews__lot[6] =
		(char)VL53LX_i2c_decode_with_mask(
			2,
			pbuffer + VL53LX_NVM__EWS__LOT__BYTE_5 - 1,
			0x00000FC0,
			6,
			32);

	pdata->nvm__ews__lot[7] = 0x00;

	pdata->nvm__ews__wafer =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__EWS__WAFER,
			0x0000001F,
			0,
			0);
	pdata->nvm__ews__xcoord =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__EWS__XCOORD,
			0x000000FF,
			0,
			0);
	pdata->nvm__ews__ycoord =
		(uint8_t)VL53LX_i2c_decode_with_mask(
			1,
			pbuffer + VL53LX_NVM__EWS__YCOORD,
			0x000000FF,
			0,
			0);

	return status;

}


void VL53LX_nvm_format_encode(
	VL53LX_decoded_nvm_data_t *pnvm_info,
	uint8_t                   *pnvm_data)
{
	SUPPRESS_UNUSED_WARNING(pnvm_info);
	SUPPRESS_UNUSED_WARNING(pnvm_data);
}


VL53LX_Error VL53LX_read_nvm_raw_data(
	VL53LX_DEV     Dev,
	uint8_t        start_address,
	uint8_t        count,
	uint8_t       *pnvm_raw_data)
{



	VL53LX_Error status = VL53LX_ERROR_NONE;

	LOG_FUNCTION_START("");



	if (status == VL53LX_ERROR_NONE)
		status = VL53LX_nvm_enable(
					Dev,
					0x0004,
					VL53LX_NVM_POWER_UP_DELAY_US);



	if (status == VL53LX_ERROR_NONE)
		status = VL53LX_nvm_read(
			Dev,
			start_address,
			count,
			pnvm_raw_data);



	if (status == VL53LX_ERROR_NONE)
		status = VL53LX_nvm_disable(Dev);

	LOG_FUNCTION_END(status);

	return status;

}


VL53LX_Error VL53LX_read_nvm(
	VL53LX_DEV                 Dev,
	uint8_t                    nvm_format,
	VL53LX_decoded_nvm_data_t *pnvm_info)
{



	VL53LX_Error status = VL53LX_ERROR_NONE;

	uint8_t nvm_data[VL53LX_NVM_SIZE_IN_BYTES];

	LOG_FUNCTION_START("");

	SUPPRESS_UNUSED_WARNING(nvm_format);



	status = VL53LX_read_nvm_raw_data(
				Dev,
				0,
				VL53LX_NVM_SIZE_IN_BYTES >> 2,
				nvm_data);





	if (status == VL53LX_ERROR_NONE)
		status = VL53LX_nvm_format_decode(
			VL53LX_NVM_SIZE_IN_BYTES,
			nvm_data,
			pnvm_info);

	LOG_FUNCTION_END(status);

	return status;

}


VL53LX_Error VL53LX_read_nvm_optical_centre(
	VL53LX_DEV                        Dev,
	VL53LX_optical_centre_t          *pcentre)
{


	VL53LX_Error status = VL53LX_ERROR_NONE;

	uint8_t nvm_data[VL53LX_NVM__FMT__OPTICAL_CENTRE_DATA_SIZE];

	LOG_FUNCTION_START("");



	status =
		VL53LX_read_nvm_raw_data(
			Dev,
			(uint8_t)(VL53LX_NVM__FMT__OPTICAL_CENTRE_DATA_INDEX
					>> 2),
			(uint8_t)(VL53LX_NVM__FMT__OPTICAL_CENTRE_DATA_SIZE
					>> 2),
			nvm_data);



	if (status == VL53LX_ERROR_NONE)
		status =
			VL53LX_nvm_decode_optical_centre(
				VL53LX_NVM__FMT__OPTICAL_CENTRE_DATA_SIZE,
				nvm_data,
				pcentre);

	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error VL53LX_read_nvm_cal_peak_rate_map(
	VL53LX_DEV                           Dev,
	VL53LX_cal_peak_rate_map_t          *pcal_data)
{


	VL53LX_Error status = VL53LX_ERROR_NONE;

	uint8_t nvm_data[VL53LX_NVM__FMT__CAL_PEAK_RATE_MAP_DATA_SIZE];

	LOG_FUNCTION_START("");



	status =
		VL53LX_read_nvm_raw_data(
			Dev,
			(uint8_t)(VL53LX_NVM__FMT__CAL_PEAK_RATE_MAP_DATA_INDEX
					>> 2),
			(uint8_t)(VL53LX_NVM__FMT__CAL_PEAK_RATE_MAP_DATA_SIZE
					>> 2),
			nvm_data);



	if (status == VL53LX_ERROR_NONE)
		status =
			VL53LX_nvm_decode_cal_peak_rate_map(
				VL53LX_NVM__FMT__CAL_PEAK_RATE_MAP_DATA_SIZE,
				nvm_data,
				pcal_data);

	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error VL53LX_read_nvm_additional_offset_cal_data(
	VL53LX_DEV                           Dev,
	VL53LX_additional_offset_cal_data_t *pcal_data)
{



	VL53LX_Error status = VL53LX_ERROR_NONE;

	uint8_t nvm_data[VL53LX_NVM__FMT__ADDITIONAL_OFFSET_CAL_DATA_SIZE];

	LOG_FUNCTION_START("");



	status =
		VL53LX_read_nvm_raw_data(
			Dev,
			(uint8_t)(
			VL53LX_NVM__FMT__ADDITIONAL_OFFSET_CAL_DATA_INDEX >> 2),
			(uint8_t)(
			VL53LX_NVM__FMT__ADDITIONAL_OFFSET_CAL_DATA_SIZE >> 2),
			nvm_data);



	if (status == VL53LX_ERROR_NONE)
		status = VL53LX_nvm_decode_additional_offset_cal_data(
			VL53LX_NVM__FMT__ADDITIONAL_OFFSET_CAL_DATA_SIZE,
			nvm_data,
			pcal_data);

	LOG_FUNCTION_END(status);

	return status;

}


VL53LX_Error VL53LX_read_nvm_fmt_range_results_data(
	VL53LX_DEV                           Dev,
	uint16_t                             range_results_select,
	VL53LX_decoded_nvm_fmt_range_data_t *prange_data)
{



	VL53LX_Error status = VL53LX_ERROR_NONE;

	uint8_t nvm_data[VL53LX_NVM__FMT__RANGE_RESULTS__SIZE_BYTES];

	LOG_FUNCTION_START("");



	status = VL53LX_read_nvm_raw_data(
		Dev,
		(uint8_t)(range_results_select >> 2),
		(uint8_t)(VL53LX_NVM__FMT__RANGE_RESULTS__SIZE_BYTES >> 2),
		nvm_data);



	if (status == VL53LX_ERROR_NONE)
		status =
			VL53LX_nvm_decode_fmt_range_results_data(
				VL53LX_NVM__FMT__RANGE_RESULTS__SIZE_BYTES,
				nvm_data,
				prange_data);

	LOG_FUNCTION_END(status);

	return status;

}


