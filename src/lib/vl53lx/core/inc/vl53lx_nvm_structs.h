
/* SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause */
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53LX and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */







#ifndef _VL53LX_NVM_STRUCTS_H_
#define _VL53LX_NVM_STRUCTS_H_


#ifdef __cplusplus
extern "C"
{
#endif

#include "vl53lx_platform.h"
#include "vl53lx_ll_def.h"


typedef struct {

	uint16_t  result__actual_effective_rtn_spads;
	uint8_t   ref_spad_array__num_requested_ref_spads;
	uint8_t   ref_spad_array__ref_location;
	uint16_t  result__peak_signal_count_rate_rtn_mcps;
	uint16_t  result__ambient_count_rate_rtn_mcps;
	uint16_t  result__peak_signal_count_rate_ref_mcps;
	uint16_t  result__ambient_count_rate_ref_mcps;
	uint16_t  measured_distance_mm;
	uint16_t  measured_distance_stdev_mm;
} VL53LX_decoded_nvm_fmt_range_data_t;


typedef struct {

	char      nvm__fmt__fgc[19];
	uint8_t   nvm__fmt__test_program_major;
	uint8_t   nvm__fmt__test_program_minor;
	uint8_t   nvm__fmt__map_major;
	uint8_t   nvm__fmt__map_minor;
	uint8_t   nvm__fmt__year;
	uint8_t   nvm__fmt__month;
	uint8_t   nvm__fmt__day;
	uint8_t   nvm__fmt__module_date_phase;
	uint16_t  nvm__fmt__time;
	uint8_t   nvm__fmt__tester_id;
	uint8_t   nvm__fmt__site_id;
	uint8_t   nvm__ews__test_program_major;
	uint8_t   nvm__ews__test_program_minor;
	uint8_t   nvm__ews__probe_card_major;
	uint8_t   nvm__ews__probe_card_minor;
	uint8_t   nvm__ews__tester_id;
	char      nvm__ews__lot[8];
	uint8_t   nvm__ews__wafer;
	uint8_t   nvm__ews__xcoord;
	uint8_t   nvm__ews__ycoord;
} VL53LX_decoded_nvm_fmt_info_t;


typedef struct {

	uint8_t   nvm__ews__test_program_major;
	uint8_t   nvm__ews__test_program_minor;
	uint8_t   nvm__ews__probe_card_major;
	uint8_t   nvm__ews__probe_card_minor;
	uint8_t   nvm__ews__tester_id;
	char      nvm__ews__lot[8];
	uint8_t   nvm__ews__wafer;
	uint8_t   nvm__ews__xcoord;
	uint8_t   nvm__ews__ycoord;
} VL53LX_decoded_nvm_ews_info_t;


typedef struct {
	uint8_t   nvm__identification_model_id;
	uint8_t   nvm__identification_module_type;
	uint8_t   nvm__identification_revision_id;
	uint16_t  nvm__identification_module_id;
	uint8_t   nvm__i2c_valid;
	uint8_t   nvm__i2c_device_address_ews;
	uint16_t  nvm__ews__fast_osc_frequency;
	uint8_t   nvm__ews__fast_osc_trim_max;
	uint8_t   nvm__ews__fast_osc_freq_set;
	uint16_t  nvm__ews__slow_osc_calibration;
	uint16_t  nvm__fmt__fast_osc_frequency;
	uint8_t   nvm__fmt__fast_osc_trim_max;
	uint8_t   nvm__fmt__fast_osc_freq_set;
	uint16_t  nvm__fmt__slow_osc_calibration;
	uint8_t   nvm__vhv_config_unlock;
	uint8_t   nvm__ref_selvddpix;
	uint8_t   nvm__ref_selvquench;
	uint8_t   nvm__regavdd1v2_sel;
	uint8_t   nvm__regdvdd1v2_sel;
	uint8_t   nvm__vhv_timeout__macrop;
	uint8_t   nvm__vhv_loop_bound;
	uint8_t   nvm__vhv_count_threshold;
	uint8_t   nvm__vhv_offset;
	uint8_t   nvm__vhv_init_enable;
	uint8_t   nvm__vhv_init_value;
	uint8_t   nvm__laser_safety_vcsel_trim_ll;
	uint8_t   nvm__laser_safety_vcsel_selion_ll;
	uint8_t   nvm__laser_safety_vcsel_selion_max_ll;
	uint8_t   nvm__laser_safety_mult_ll;
	uint8_t   nvm__laser_safety_clip_ll;
	uint8_t   nvm__laser_safety_vcsel_trim_ld;
	uint8_t   nvm__laser_safety_vcsel_selion_ld;
	uint8_t   nvm__laser_safety_vcsel_selion_max_ld;
	uint8_t   nvm__laser_safety_mult_ld;
	uint8_t   nvm__laser_safety_clip_ld;
	uint8_t   nvm__laser_safety_lock_byte;
	uint8_t   nvm__laser_safety_unlock_byte;
	uint8_t   nvm__ews__spad_enables_rtn[VL53LX_RTN_SPAD_BUFFER_SIZE];
	uint8_t   nvm__ews__spad_enables_ref__loc1[VL53LX_REF_SPAD_BUFFER_SIZE];
	uint8_t   nvm__ews__spad_enables_ref__loc2[VL53LX_REF_SPAD_BUFFER_SIZE];
	uint8_t   nvm__ews__spad_enables_ref__loc3[VL53LX_REF_SPAD_BUFFER_SIZE];
	uint8_t   nvm__fmt__spad_enables_rtn[VL53LX_RTN_SPAD_BUFFER_SIZE];
	uint8_t   nvm__fmt__spad_enables_ref__loc1[VL53LX_REF_SPAD_BUFFER_SIZE];
	uint8_t   nvm__fmt__spad_enables_ref__loc2[VL53LX_REF_SPAD_BUFFER_SIZE];
	uint8_t   nvm__fmt__spad_enables_ref__loc3[VL53LX_REF_SPAD_BUFFER_SIZE];
	uint8_t   nvm__fmt__roi_config__mode_roi_centre_spad;
	uint8_t   nvm__fmt__roi_config__mode_roi_x_size;
	uint8_t   nvm__fmt__roi_config__mode_roi_y_size;
	uint8_t   nvm__fmt__ref_spad_apply__num_requested_ref_spad;
	uint8_t   nvm__fmt__ref_spad_man__ref_location;
	uint16_t  nvm__fmt__mm_config__inner_offset_mm;
	uint16_t  nvm__fmt__mm_config__outer_offset_mm;
	uint16_t  nvm__fmt__algo_part_to_part_range_offset_mm;
	uint16_t  nvm__fmt__algo__crosstalk_compensation_plane_offset_kcps;
	uint16_t  nvm__fmt__algo__crosstalk_compensation_x_plane_gradient_kcps;
	uint16_t  nvm__fmt__algo__crosstalk_compensation_y_plane_gradient_kcps;
	uint8_t   nvm__fmt__spare__host_config__nvm_config_spare_0;
	uint8_t   nvm__fmt__spare__host_config__nvm_config_spare_1;
	uint8_t   nvm__customer_space_programmed;
	uint8_t   nvm__cust__i2c_device_address;
	uint8_t   nvm__cust__ref_spad_apply__num_requested_ref_spad;
	uint8_t   nvm__cust__ref_spad_man__ref_location;
	uint16_t  nvm__cust__mm_config__inner_offset_mm;
	uint16_t  nvm__cust__mm_config__outer_offset_mm;
	uint16_t  nvm__cust__algo_part_to_part_range_offset_mm;
	uint16_t  nvm__cust__algo__crosstalk_compensation_plane_offset_kcps;
	uint16_t  nvm__cust__algo__crosstalk_compensation_x_plane_gradient_kcps;
	uint16_t  nvm__cust__algo__crosstalk_compensation_y_plane_gradient_kcps;
	uint8_t   nvm__cust__spare__host_config__nvm_config_spare_0;
	uint8_t   nvm__cust__spare__host_config__nvm_config_spare_1;
	VL53LX_optical_centre_t              fmt_optical_centre;
	VL53LX_cal_peak_rate_map_t           fmt_peak_rate_map;
	VL53LX_additional_offset_cal_data_t  fmt_add_offset_data;

	VL53LX_decoded_nvm_fmt_range_data_t
		fmt_range_data[VL53LX_NVM_MAX_FMT_RANGE_DATA];

	VL53LX_decoded_nvm_fmt_info_t        fmt_info;
	VL53LX_decoded_nvm_ews_info_t        ews_info;

} VL53LX_decoded_nvm_data_t;



#ifdef __cplusplus
}
#endif

#endif

