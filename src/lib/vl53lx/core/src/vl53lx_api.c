
// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53LX and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */






#include "vl53lx_api.h"
#include "vl53lx_register_settings.h"
#include "vl53lx_register_funcs.h"
#include "vl53lx_core.h"
#include "vl53lx_api_calibration.h"
#include "vl53lx_wait.h"
#include "vl53lx_preset_setup.h"
#include "vl53lx_api_debug.h"
#include "vl53lx_api_core.h"
#include "vl53lx_nvm.h"
#include "vl53lx_def.h"


#define ZONE_CHECK 5

#define LOG_FUNCTION_START(fmt, ...) 
#define LOG_FUNCTION_END(status, ...)
#define LOG_FUNCTION_END_FMT(status, fmt, ...)

#ifdef VL53LX_LOG_ENABLE
#define trace_print(level, ...) 
#endif

#ifndef MIN
#define MIN(v1, v2) ((v1) < (v2) ? (v1) : (v2))
#endif
#ifndef MAX
#define MAX(v1, v2) ((v1) < (v2) ? (v2) : (v1))
#endif

#define DMAX_REFLECTANCE_IDX 2



#define LOWPOWER_AUTO_VHV_LOOP_DURATION_US 245
#define LOWPOWER_AUTO_OVERHEAD_BEFORE_A_RANGING 1448
#define LOWPOWER_AUTO_OVERHEAD_BETWEEN_A_B_RANGING 2100

#define FDA_MAX_TIMING_BUDGET_US 550000
#define L4_FDA_MAX_TIMING_BUDGET_US 200000






static int32_t BDTable[VL53LX_TUNING_MAX_TUNABLE_KEY] = {
		TUNING_VERSION,
		TUNING_PROXY_MIN,
		TUNING_SINGLE_TARGET_XTALK_TARGET_DISTANCE_MM,
		TUNING_SINGLE_TARGET_XTALK_SAMPLE_NUMBER,
		TUNING_MIN_AMBIENT_DMAX_VALID,
		TUNING_MAX_SIMPLE_OFFSET_CALIBRATION_SAMPLE_NUMBER,
		TUNING_XTALK_FULL_ROI_TARGET_DISTANCE_MM,
		TUNING_SIMPLE_OFFSET_CALIBRATION_REPEAT,
		TUNING_XTALK_FULL_ROI_BIN_SUM_MARGIN,
		TUNING_XTALK_FULL_ROI_DEFAULT_OFFSET,
		TUNING_ZERO_DISTANCE_OFFSET_NON_LINEAR_FACTOR_DEFAULT
};

static VL53LX_Error ComputeDevicePresetMode(
		VL53LX_DistanceModes DistanceMode,
		VL53LX_DevicePresetModes *pDevicePresetMode);

static VL53LX_Error SetPresetModeL3CX(VL53LX_DEV Dev,
		VL53LX_DistanceModes DistanceMode,
		uint32_t inter_measurement_period_ms);

static int IsL4(VL53LX_DEV Dev);

static VL53LX_Error CheckValidRectRoi(VL53LX_UserRoi_t ROI);


VL53LX_Error VL53LX_GetVersion(VL53LX_Version_t *pVersion)
{
	VL53LX_Error Status = VL53LX_ERROR_NONE;

	LOG_FUNCTION_START("");

	pVersion->major = VL53LX_IMPLEMENTATION_VER_MAJOR;
	pVersion->minor = VL53LX_IMPLEMENTATION_VER_MINOR;
	pVersion->build = VL53LX_IMPLEMENTATION_VER_SUB;

	pVersion->revision = VL53LX_IMPLEMENTATION_VER_REVISION;

	LOG_FUNCTION_END(Status);
	return Status;
}

VL53LX_Error VL53LX_GetProductRevision(VL53LX_DEV Dev,
	uint8_t *pProductRevisionMajor, uint8_t *pProductRevisionMinor)
{
	VL53LX_Error Status = VL53LX_ERROR_NONE;
	uint8_t revision_id;
	VL53LX_LLDriverData_t   *pLLData;

	LOG_FUNCTION_START("");

	pLLData =  VL53LXDevStructGetLLDriverHandle(Dev);
	revision_id = pLLData->nvm_copy_data.identification__revision_id;
	*pProductRevisionMajor = 1;
	*pProductRevisionMinor = (revision_id & 0xF0) >> 4;

	LOG_FUNCTION_END(Status);
	return Status;

}

VL53LX_Error VL53LX_GetDeviceInfo(VL53LX_DEV Dev,
	VL53LX_DeviceInfo_t *pVL53LX_DeviceInfo)
{
	VL53LX_Error Status = VL53LX_ERROR_NONE;
	uint8_t revision_id;
	VL53LX_LLDriverData_t   *pLLData;

	LOG_FUNCTION_START("");

	pLLData =  VL53LXDevStructGetLLDriverHandle(Dev);

	pVL53LX_DeviceInfo->ProductType =
			pLLData->nvm_copy_data.identification__module_type;

	revision_id = pLLData->nvm_copy_data.identification__revision_id;
	pVL53LX_DeviceInfo->ProductRevisionMajor = 1;
	pVL53LX_DeviceInfo->ProductRevisionMinor = (revision_id & 0xF0) >> 4;

	LOG_FUNCTION_END(Status);
	return Status;
}

VL53LX_Error VL53LX_GetUID(VL53LX_DEV Dev, uint64_t *pUid)
{
	VL53LX_Error Status = VL53LX_ERROR_NONE;
	uint8_t fmtdata[8];

	LOG_FUNCTION_START("");

	Status = VL53LX_read_nvm_raw_data(Dev,
			(uint8_t)(0x1F8 >> 2),
			(uint8_t)(8 >> 2),
			fmtdata);
	memcpy(pUid, fmtdata, sizeof(uint64_t));

	LOG_FUNCTION_END(Status);
	return Status;
}



VL53LX_Error VL53LX_SetDeviceAddress(VL53LX_DEV Dev, uint8_t DeviceAddress)
{
	VL53LX_Error Status = VL53LX_ERROR_NONE;
	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);
	VL53LX_static_nvm_managed_t  *pdata = &(pdev->stat_nvm);

	LOG_FUNCTION_START("");

	Status = VL53LX_WrByte(Dev, VL53LX_I2C_SLAVE__DEVICE_ADDRESS,
			DeviceAddress);

	pdata->i2c_slave__device_address = (DeviceAddress) & 0x7F;

	LOG_FUNCTION_END(Status);
	return Status;
}


VL53LX_Error VL53LX_DataInit(VL53LX_DEV Dev)
{
	VL53LX_Error Status = VL53LX_ERROR_NONE;
	VL53LX_LLDriverData_t *pdev;
	uint8_t  measurement_mode;
	uint8_t i;

	LOG_FUNCTION_START("");


#ifdef USE_I2C_2V8
	Status = VL53LX_RdByte(Dev, VL53LX_PAD_I2C_HV__EXTSUP_CONFIG, &i);
	if (Status == VL53LX_ERROR_NONE) {
		i = (i & 0xfe) | 0x01;
		Status = VL53LX_WrByte(Dev, VL53LX_PAD_I2C_HV__EXTSUP_CONFIG,
				i);
	}
#endif

	if (Status == VL53LX_ERROR_NONE)
		Status = VL53LX_data_init(Dev, 1);

	if (Status == VL53LX_ERROR_NONE)
		Status = SetPresetModeL3CX(Dev,
			VL53LX_DISTANCEMODE_MEDIUM,
			1000);


	if (Status == VL53LX_ERROR_NONE)
		Status = VL53LX_SetMeasurementTimingBudgetMicroSeconds(Dev,
				33333);

	if (Status == VL53LX_ERROR_NONE) {
		pdev = VL53LXDevStructGetLLDriverHandle(Dev);
		memset(&pdev->per_vcsel_cal_data, 0,
				sizeof(pdev->per_vcsel_cal_data));
	}

	if (Status == VL53LX_ERROR_NONE) {
		Status = VL53LX_set_dmax_mode(Dev,
			VL53LX_DEVICEDMAXMODE__CUST_CAL_DATA);
	}


	if (Status == VL53LX_ERROR_NONE)
		Status = VL53LX_SmudgeCorrectionEnable(Dev,
			VL53LX_SMUDGE_CORRECTION_NONE);

	measurement_mode  = VL53LX_DEVICEMEASUREMENTMODE_BACKTOBACK;
	VL53LXDevDataSet(Dev, LLData.measurement_mode, measurement_mode);

	VL53LXDevDataSet(Dev, CurrentParameters.DistanceMode,
			VL53LX_DISTANCEMODE_MEDIUM);

	LOG_FUNCTION_END(Status);
	return Status;
}


VL53LX_Error VL53LX_WaitDeviceBooted(VL53LX_DEV Dev)
{
	VL53LX_Error Status = VL53LX_ERROR_NONE;

	LOG_FUNCTION_START("");

	Status = VL53LX_poll_for_boot_completion(Dev,
			VL53LX_BOOT_COMPLETION_POLLING_TIMEOUT_MS);

	LOG_FUNCTION_END(Status);
	return Status;
}




static VL53LX_Error ComputeDevicePresetMode(
		VL53LX_DistanceModes DistanceMode,
		VL53LX_DevicePresetModes *pDevicePresetMode)
{
	VL53LX_Error Status = VL53LX_ERROR_NONE;

	uint8_t DistIdx;
	VL53LX_DevicePresetModes RangingModes[3] = {
		VL53LX_DEVICEPRESETMODE_HISTOGRAM_SHORT_RANGE,
		VL53LX_DEVICEPRESETMODE_HISTOGRAM_MEDIUM_RANGE,
		VL53LX_DEVICEPRESETMODE_HISTOGRAM_LONG_RANGE};

	switch (DistanceMode) {
	case VL53LX_DISTANCEMODE_SHORT:
		DistIdx = 0;
		break;
	case VL53LX_DISTANCEMODE_MEDIUM:
		DistIdx = 1;
		break;
	default:
		DistIdx = 2;
	}

	*pDevicePresetMode = RangingModes[DistIdx];

	return Status;
}

static VL53LX_Error SetPresetModeL3CX(VL53LX_DEV Dev,
		VL53LX_DistanceModes DistanceMode,
		uint32_t inter_measurement_period_ms)
{
	VL53LX_Error Status = VL53LX_ERROR_NONE;
	VL53LX_DevicePresetModes   device_preset_mode;
	uint8_t measurement_mode;
	uint16_t dss_config__target_total_rate_mcps = 0;
	uint32_t phasecal_config_timeout_us = 0;
	uint32_t mm_config_timeout_us = 0;
	uint32_t lld_range_config_timeout_us = 0;

	LOG_FUNCTION_START("");

	measurement_mode  = VL53LX_DEVICEMEASUREMENTMODE_BACKTOBACK;

	Status = ComputeDevicePresetMode(DistanceMode,
			&device_preset_mode);

	if (Status == VL53LX_ERROR_NONE)
		Status =  VL53LX_get_preset_mode_timing_cfg(Dev,
				device_preset_mode,
				&dss_config__target_total_rate_mcps,
				&phasecal_config_timeout_us,
				&mm_config_timeout_us,
				&lld_range_config_timeout_us);

	if (Status == VL53LX_ERROR_NONE)
		Status = VL53LX_set_preset_mode(
				Dev,
				device_preset_mode,
				dss_config__target_total_rate_mcps,
				phasecal_config_timeout_us,
				mm_config_timeout_us,
				lld_range_config_timeout_us,
				inter_measurement_period_ms);

	if (Status == VL53LX_ERROR_NONE)
		VL53LXDevDataSet(Dev, LLData.measurement_mode,
				measurement_mode);

	LOG_FUNCTION_END(Status);
	return Status;
}

static int IsL4(VL53LX_DEV Dev)
{
	int devL4 = 0;
	VL53LX_LLDriverData_t *pDev;
	pDev = VL53LXDevStructGetLLDriverHandle(Dev);

	if ((pDev->nvm_copy_data.identification__module_type == 0xAA) &&
		(pDev->nvm_copy_data.identification__model_id == 0xEB))
		devL4 = 1;
	return devL4;
}

static VL53LX_Error CheckValidRectRoi(VL53LX_UserRoi_t ROI)
{
	VL53LX_Error Status = VL53LX_ERROR_NONE;

	LOG_FUNCTION_START("");


	if ((ROI.TopLeftX > 15) || (ROI.TopLeftY > 15) ||
		(ROI.BotRightX > 15) || (ROI.BotRightY > 15))
		Status = VL53LX_ERROR_INVALID_PARAMS;

	if ((ROI.TopLeftX > ROI.BotRightX) || (ROI.TopLeftY < ROI.BotRightY))
		Status = VL53LX_ERROR_INVALID_PARAMS;

	LOG_FUNCTION_END(Status);
	return Status;
}


VL53LX_Error VL53LX_SetDistanceMode(VL53LX_DEV Dev,
		VL53LX_DistanceModes DistanceMode)
{
	VL53LX_Error Status = VL53LX_ERROR_NONE;
	uint32_t inter_measurement_period_ms;
	uint32_t TimingBudget = 0;
	uint32_t MmTimeoutUs = 0;
	uint32_t PhaseCalTimeoutUs = 0;

	LOG_FUNCTION_START("%d", (int)DistanceMode);



	if ((DistanceMode != VL53LX_DISTANCEMODE_SHORT) &&
		(DistanceMode != VL53LX_DISTANCEMODE_MEDIUM) &&
		(DistanceMode != VL53LX_DISTANCEMODE_LONG))
		return VL53LX_ERROR_INVALID_PARAMS;

	if (IsL4(Dev) && (DistanceMode == VL53LX_DISTANCEMODE_SHORT))
		return VL53LX_ERROR_INVALID_PARAMS;

	inter_measurement_period_ms =  VL53LXDevDataGet(Dev,
				LLData.inter_measurement_period_ms);

	if (Status == VL53LX_ERROR_NONE)
		Status = VL53LX_get_timeouts_us(Dev, &PhaseCalTimeoutUs,
			&MmTimeoutUs, &TimingBudget);

	if (Status == VL53LX_ERROR_NONE)
		Status = SetPresetModeL3CX(Dev,
				DistanceMode,
				inter_measurement_period_ms);

	if (Status == VL53LX_ERROR_NONE) {
		VL53LXDevDataSet(Dev, CurrentParameters.DistanceMode,
				DistanceMode);
	}

	if (Status == VL53LX_ERROR_NONE) {
		Status = VL53LX_set_timeouts_us(Dev, PhaseCalTimeoutUs,
			MmTimeoutUs, TimingBudget);

		if (Status == VL53LX_ERROR_NONE)
			VL53LXDevDataSet(Dev, LLData.range_config_timeout_us,
				TimingBudget);
	}

	LOG_FUNCTION_END(Status);
	return Status;
}

VL53LX_Error VL53LX_GetDistanceMode(VL53LX_DEV Dev,
	VL53LX_DistanceModes *pDistanceMode)
{
	VL53LX_Error Status = VL53LX_ERROR_NONE;

	LOG_FUNCTION_START("");

	*pDistanceMode = VL53LXDevDataGet(Dev, CurrentParameters.DistanceMode);

	LOG_FUNCTION_END(Status);
	return Status;
}


VL53LX_Error VL53LX_SetMeasurementTimingBudgetMicroSeconds(VL53LX_DEV Dev,
	uint32_t MeasurementTimingBudgetMicroSeconds)
{
	VL53LX_Error Status = VL53LX_ERROR_NONE;
	uint32_t TimingGuard;
	uint32_t divisor;
	uint32_t TimingBudget = 0;
	uint32_t MmTimeoutUs = 0;
	uint32_t PhaseCalTimeoutUs = 0;
	uint32_t FDAMaxTimingBudgetUs = FDA_MAX_TIMING_BUDGET_US;

	LOG_FUNCTION_START("");


	if (MeasurementTimingBudgetMicroSeconds > 10000000)
		Status = VL53LX_ERROR_INVALID_PARAMS;

	if (Status == VL53LX_ERROR_NONE)
		Status = VL53LX_get_timeouts_us(Dev,
			&PhaseCalTimeoutUs,
			&MmTimeoutUs,
			&TimingBudget);

	TimingGuard = 1700;
	divisor = 6;

	if (IsL4(Dev))
		FDAMaxTimingBudgetUs = L4_FDA_MAX_TIMING_BUDGET_US;

	if (MeasurementTimingBudgetMicroSeconds <= TimingGuard)
		Status = VL53LX_ERROR_INVALID_PARAMS;
	else {
		TimingBudget = (MeasurementTimingBudgetMicroSeconds
				- TimingGuard);
	}

	if (Status == VL53LX_ERROR_NONE) {
		if (TimingBudget > FDAMaxTimingBudgetUs)
			Status = VL53LX_ERROR_INVALID_PARAMS;
		else {
			TimingBudget /= divisor;
			Status = VL53LX_set_timeouts_us(
				Dev,
				PhaseCalTimeoutUs,
				MmTimeoutUs,
				TimingBudget);
		}

		if (Status == VL53LX_ERROR_NONE)
			VL53LXDevDataSet(Dev,
				LLData.range_config_timeout_us,
				TimingBudget);
	}

	if (Status == VL53LX_ERROR_NONE) {
		VL53LXDevDataSet(Dev,
			CurrentParameters.MeasurementTimingBudgetMicroSeconds,
			MeasurementTimingBudgetMicroSeconds);
	}

	LOG_FUNCTION_END(Status);
	return Status;
}


VL53LX_Error VL53LX_GetMeasurementTimingBudgetMicroSeconds(VL53LX_DEV Dev,
	uint32_t *pMeasurementTimingBudgetMicroSeconds)
{
	VL53LX_Error Status = VL53LX_ERROR_NONE;
	uint32_t MmTimeoutUs = 0;
	uint32_t RangeTimeoutUs = 0;
	uint32_t PhaseCalTimeoutUs = 0;

	LOG_FUNCTION_START("");

	*pMeasurementTimingBudgetMicroSeconds = 0;

	if (Status == VL53LX_ERROR_NONE)
		Status = VL53LX_get_timeouts_us(Dev,
			&PhaseCalTimeoutUs,
			&MmTimeoutUs,
			&RangeTimeoutUs);

	if (Status == VL53LX_ERROR_NONE)
		*pMeasurementTimingBudgetMicroSeconds = (6 * RangeTimeoutUs) +
		1700;

	LOG_FUNCTION_END(Status);
	return Status;
}





VL53LX_Error VL53LX_SetUserROI(VL53LX_DEV Dev,
		VL53LX_UserRoi_t *pRoi)
{
	VL53LX_Error Status = VL53LX_ERROR_NONE;
	VL53LX_zone_config_t  zone_cfg;
	uint8_t x_centre, y_centre, width, height;

	Status = CheckValidRectRoi(*pRoi);
	if (Status != VL53LX_ERROR_NONE)
		return VL53LX_ERROR_INVALID_PARAMS;

	x_centre = (pRoi->BotRightX + pRoi->TopLeftX  + 1) / 2;
	y_centre = (pRoi->TopLeftY  + pRoi->BotRightY + 1) / 2;
	width =    (pRoi->BotRightX - pRoi->TopLeftX);
	height =   (pRoi->TopLeftY  - pRoi->BotRightY);
	zone_cfg.max_zones = 1;
	zone_cfg.active_zones = 0;
	zone_cfg.user_zones[0].x_centre = x_centre;
	zone_cfg.user_zones[0].y_centre = y_centre;
	zone_cfg.user_zones[0].width = width;
	zone_cfg.user_zones[0].height = height;
	if ((width < 3) || (height < 3))
		Status = VL53LX_ERROR_INVALID_PARAMS;
	else
		Status =  VL53LX_set_zone_config(Dev, &zone_cfg);

	LOG_FUNCTION_END(Status);
	return Status;
}

VL53LX_Error VL53LX_GetUserROI(VL53LX_DEV Dev,
		VL53LX_UserRoi_t *pRoi)
{
	VL53LX_Error Status = VL53LX_ERROR_NONE;
	VL53LX_zone_config_t zone_cfg;
	uint8_t  TopLeftX;
	uint8_t  TopLeftY;
	uint8_t  BotRightX;
	uint8_t  BotRightY;

	LOG_FUNCTION_START("");

	VL53LX_get_zone_config(Dev, &zone_cfg);

	TopLeftX = (2 * zone_cfg.user_zones[0].x_centre -
		zone_cfg.user_zones[0].width) >> 1;
	TopLeftY = (2 * zone_cfg.user_zones[0].y_centre +
		zone_cfg.user_zones[0].height) >> 1;
	BotRightX = (2 * zone_cfg.user_zones[0].x_centre +
		zone_cfg.user_zones[0].width) >> 1;
	BotRightY = (2 * zone_cfg.user_zones[0].y_centre -
		zone_cfg.user_zones[0].height) >> 1;
	pRoi->TopLeftX = TopLeftX;
	pRoi->TopLeftY = TopLeftY;
	pRoi->BotRightX = BotRightX;
	pRoi->BotRightY = BotRightY;

	LOG_FUNCTION_END(Status);
	return Status;
}





VL53LX_Error VL53LX_StartMeasurement(VL53LX_DEV Dev)
{
	VL53LX_Error Status = VL53LX_ERROR_NONE;
	uint8_t DeviceMeasurementMode;
	uint8_t i;
	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

	LOG_FUNCTION_START("");

	VL53LX_load_patch(Dev);
	for (i = 0; i < VL53LX_MAX_RANGE_RESULTS; i++) {
		pdev->PreviousRangeMilliMeter[i] = 0;
		pdev->PreviousRangeStatus[i] = 255;
		pdev->PreviousExtendedRange[i] = 0;
	}
	pdev->PreviousStreamCount = 0;
	pdev->PreviousRangeActiveResults = 0;

	DeviceMeasurementMode = VL53LXDevDataGet(Dev, LLData.measurement_mode);

	if (Status == VL53LX_ERROR_NONE)
		Status = VL53LX_init_and_start_range(
				Dev,
				DeviceMeasurementMode,
				VL53LX_DEVICECONFIGLEVEL_FULL);

	LOG_FUNCTION_END(Status);
	return Status;
}

VL53LX_Error VL53LX_StopMeasurement(VL53LX_DEV Dev)
{
	VL53LX_Error Status = VL53LX_ERROR_NONE;

	LOG_FUNCTION_START("");

	Status = VL53LX_stop_range(Dev);

	VL53LX_unload_patch(Dev);

	LOG_FUNCTION_END(Status);
	return Status;
}


VL53LX_Error VL53LX_ClearInterruptAndStartMeasurement(VL53LX_DEV Dev)
{
	VL53LX_Error Status = VL53LX_ERROR_NONE;
	uint8_t DeviceMeasurementMode;

	LOG_FUNCTION_START("");

	DeviceMeasurementMode = VL53LXDevDataGet(Dev, LLData.measurement_mode);

	Status = VL53LX_clear_interrupt_and_enable_next_range(Dev,
			DeviceMeasurementMode);

	LOG_FUNCTION_END(Status);
	return Status;
}


VL53LX_Error VL53LX_GetMeasurementDataReady(VL53LX_DEV Dev,
	uint8_t *pMeasurementDataReady)
{
	VL53LX_Error Status = VL53LX_ERROR_NONE;

	LOG_FUNCTION_START("");

	Status = VL53LX_is_new_data_ready(Dev, pMeasurementDataReady);

	LOG_FUNCTION_END(Status);
	return Status;
}

VL53LX_Error VL53LX_WaitMeasurementDataReady(VL53LX_DEV Dev)
{
	VL53LX_Error Status = VL53LX_ERROR_NONE;

	LOG_FUNCTION_START("");



	Status = VL53LX_poll_for_range_completion(Dev,
			VL53LX_RANGE_COMPLETION_POLLING_TIMEOUT_MS);

	LOG_FUNCTION_END(Status);
	return Status;
}

static uint8_t ConvertStatusHisto(uint8_t FilteredRangeStatus)
{
	uint8_t RangeStatus;

	switch (FilteredRangeStatus) {
	case VL53LX_DEVICEERROR_RANGEPHASECHECK:
		RangeStatus = VL53LX_RANGESTATUS_OUTOFBOUNDS_FAIL;
		break;
	case VL53LX_DEVICEERROR_SIGMATHRESHOLDCHECK:
		RangeStatus = VL53LX_RANGESTATUS_SIGMA_FAIL;
		break;
	case VL53LX_DEVICEERROR_RANGECOMPLETE_NO_WRAP_CHECK:
		RangeStatus =
			VL53LX_RANGESTATUS_RANGE_VALID_NO_WRAP_CHECK_FAIL;
		break;
	case VL53LX_DEVICEERROR_PHASECONSISTENCY:
		RangeStatus = VL53LX_RANGESTATUS_WRAP_TARGET_FAIL;
		break;
	case VL53LX_DEVICEERROR_PREV_RANGE_NO_TARGETS:
		RangeStatus = VL53LX_RANGESTATUS_TARGET_PRESENT_LACK_OF_SIGNAL;
		break;
	case VL53LX_DEVICEERROR_EVENTCONSISTENCY:
		RangeStatus = VL53LX_RANGESTATUS_WRAP_TARGET_FAIL;
		break;
	case VL53LX_DEVICEERROR_RANGECOMPLETE_MERGED_PULSE:
		RangeStatus = VL53LX_RANGESTATUS_RANGE_VALID_MERGED_PULSE;
		break;
	case VL53LX_DEVICEERROR_RANGECOMPLETE:
		RangeStatus = VL53LX_RANGESTATUS_RANGE_VALID;
		break;
	default:
		RangeStatus = VL53LX_RANGESTATUS_NONE;
	}

	return RangeStatus;
}

static VL53LX_Error SetTargetData(VL53LX_DEV Dev,
	uint8_t active_results, uint8_t streamcount, uint8_t iteration,
	uint8_t device_status, VL53LX_range_data_t *presults_data,
	VL53LX_TargetRangeData_t *pRangeData)
{
	VL53LX_Error Status = VL53LX_ERROR_NONE;
	VL53LX_LLDriverData_t *pdev =
			VL53LXDevStructGetLLDriverHandle(Dev);
	VL53LX_tuning_parm_storage_t *tp =
			&(pdev->VL53LX_LLDriverCommonData->tuning_parms);
	uint8_t sequency;
	uint8_t FilteredRangeStatus;
	FixPoint1616_t AmbientRate;
	FixPoint1616_t SignalRate;
	FixPoint1616_t TempFix1616;
	int16_t Range, RangeDiff, RangeMillimeterInit;
	int32_t ExtendedRangeEnabled = 0;
	uint8_t uwr_status;
	int16_t AddOffset;

	SUPPRESS_UNUSED_WARNING(Dev);

	FilteredRangeStatus = presults_data->range_status & 0x1F;

	SignalRate = VL53LX_FIXPOINT97TOFIXPOINT1616(
		presults_data->peak_signal_count_rate_mcps);
	pRangeData->SignalRateRtnMegaCps
		= SignalRate;

	AmbientRate = VL53LX_FIXPOINT97TOFIXPOINT1616(
		presults_data->ambient_count_rate_mcps);
	pRangeData->AmbientRateRtnMegaCps = AmbientRate;

	TempFix1616 = VL53LX_FIXPOINT97TOFIXPOINT1616(
			presults_data->VL53LX_p_002);

	pRangeData->SigmaMilliMeter = TempFix1616;

	pRangeData->RangeMilliMeter = presults_data->median_range_mm;
	pRangeData->RangeMaxMilliMeter = presults_data->max_range_mm;
	pRangeData->RangeMinMilliMeter = presults_data->min_range_mm;


	switch (device_status) {
	case VL53LX_DEVICEERROR_MULTCLIPFAIL:
	case VL53LX_DEVICEERROR_VCSELWATCHDOGTESTFAILURE:
	case VL53LX_DEVICEERROR_VCSELCONTINUITYTESTFAILURE:
	case VL53LX_DEVICEERROR_NOVHVVALUEFOUND:
		pRangeData->RangeStatus =  VL53LX_RANGESTATUS_HARDWARE_FAIL;
		break;
	case VL53LX_DEVICEERROR_USERROICLIP:
		pRangeData->RangeStatus =  VL53LX_RANGESTATUS_MIN_RANGE_FAIL;
		break;
	default:
		pRangeData->RangeStatus =  VL53LX_RANGESTATUS_RANGE_VALID;
	}


	if ((pRangeData->RangeStatus ==  VL53LX_RANGESTATUS_RANGE_VALID) &&
		(active_results == 0)) {
		pRangeData->RangeStatus =  VL53LX_RANGESTATUS_NONE;
		pRangeData->SignalRateRtnMegaCps = 0;
		pRangeData->SigmaMilliMeter = 0;
		pRangeData->RangeMilliMeter = 8191;
		pRangeData->RangeMaxMilliMeter = 8191;
		pRangeData->RangeMinMilliMeter = 8191;
	}


	if (pRangeData->RangeStatus ==  VL53LX_RANGESTATUS_RANGE_VALID)
		pRangeData->RangeStatus =
			ConvertStatusHisto(FilteredRangeStatus);



	VL53LX_get_tuning_parm(Dev, VL53LX_TUNINGPARM_UWR_ENABLE,
			&ExtendedRangeEnabled);

	sequency = streamcount % 2;
	uwr_status = 0;
	RangeMillimeterInit = pRangeData->RangeMilliMeter;
	AddOffset = 0;

	pRangeData->ExtendedRange = 0;

	if ((active_results != 1) ||
		(pdev->PreviousRangeActiveResults != 1))
		ExtendedRangeEnabled = 0;

	if (ExtendedRangeEnabled &&
		(pRangeData->RangeStatus ==
			VL53LX_RANGESTATUS_WRAP_TARGET_FAIL ||
			pRangeData->RangeStatus ==
			VL53LX_RANGESTATUS_OUTOFBOUNDS_FAIL)
		&& (pdev->PreviousRangeStatus[iteration] ==
			VL53LX_RANGESTATUS_WRAP_TARGET_FAIL ||
			pdev->PreviousRangeStatus[iteration] ==
			VL53LX_RANGESTATUS_OUTOFBOUNDS_FAIL ||
			(pdev->PreviousRangeStatus[iteration] ==
			VL53LX_RANGESTATUS_RANGE_VALID &&
			pdev->PreviousExtendedRange[iteration] == 1)))
	{
		// if (((pdev->PreviousStreamCount) ==
		// 	(pdev->VL53LX_LLDriverCommonData->hist_data.result__stream_count - 1 ))
		// || ((pdev->PreviousStreamCount) ==
		// 	(pdev->VL53LX_LLDriverCommonData->hist_data.result__stream_count + 127)))
		if(true)
		{
		RangeDiff = pRangeData->RangeMilliMeter -
			pdev->PreviousRangeMilliMeter[iteration];

		uwr_status = 1;
		switch (pdev->preset_mode) {
			case VL53LX_DEVICEPRESETMODE_HISTOGRAM_SHORT_RANGE:

				uwr_status = 0;
				break;

			case VL53LX_DEVICEPRESETMODE_HISTOGRAM_MEDIUM_RANGE:
				if (RangeDiff > tp->tp_uwr_med_z_1_min &&
					RangeDiff < tp->tp_uwr_med_z_1_max &&
					sequency == 1) {
					AddOffset =
					tp->tp_uwr_med_corr_z_1_rangeb;
				}
				else
				if (RangeDiff < -tp->tp_uwr_med_z_1_min &&
					RangeDiff > -tp->tp_uwr_med_z_1_max &&
					sequency == 0) {
					AddOffset =
					tp->tp_uwr_med_corr_z_1_rangea;
				}
				else
				if (RangeDiff > tp->tp_uwr_med_z_2_min &&
					RangeDiff < tp->tp_uwr_med_z_2_max &&
					sequency == 0) {
					AddOffset =
					tp->tp_uwr_med_corr_z_2_rangea;
				}
				else
				if (RangeDiff < -tp->tp_uwr_med_z_2_min &&
					RangeDiff > -tp->tp_uwr_med_z_2_max &&
					sequency == 1) {
					AddOffset =
					tp->tp_uwr_med_corr_z_2_rangeb;
				}
				else
				if (RangeDiff > tp->tp_uwr_med_z_3_min &&
					RangeDiff < tp->tp_uwr_med_z_3_max &&
					sequency == 1) {
					AddOffset =
					tp->tp_uwr_med_corr_z_3_rangeb;
				}
				else
				if (RangeDiff < -tp->tp_uwr_med_z_3_min &&
					RangeDiff > -tp->tp_uwr_med_z_3_max &&
					sequency == 0) {
					AddOffset =
					tp->tp_uwr_med_corr_z_3_rangea;
				}
				else
				if (RangeDiff > tp->tp_uwr_med_z_4_min &&
					RangeDiff < tp->tp_uwr_med_z_4_max &&
					sequency == 0) {
					AddOffset =
					tp->tp_uwr_med_corr_z_4_rangea;
				}
				else
				if (RangeDiff < -tp->tp_uwr_med_z_4_min &&
					RangeDiff > -tp->tp_uwr_med_z_4_max &&
					sequency == 1) {
					AddOffset =
					tp->tp_uwr_med_corr_z_4_rangeb;
				}
				else
				if (RangeDiff < tp->tp_uwr_med_z_5_max &&
					RangeDiff > tp->tp_uwr_med_z_5_min) {
					AddOffset =
					tp->tp_uwr_med_corr_z_5_rangea;
				} else
					uwr_status = 0;
				break;

			case VL53LX_DEVICEPRESETMODE_HISTOGRAM_LONG_RANGE:
				if (RangeDiff > tp->tp_uwr_lng_z_1_min &&
					RangeDiff < tp->tp_uwr_lng_z_1_max &&
					sequency == 0) {
					AddOffset =
					tp->tp_uwr_lng_corr_z_1_rangea;
				}
				else
				if (RangeDiff < -tp->tp_uwr_lng_z_1_min &&
					RangeDiff > -tp->tp_uwr_lng_z_1_max &&
					sequency == 1) {
					AddOffset =
					tp->tp_uwr_lng_corr_z_1_rangeb;
				}
				else
				if (RangeDiff > tp->tp_uwr_lng_z_2_min &&
					RangeDiff < tp->tp_uwr_lng_z_2_max &&
					sequency == 1) {
					AddOffset =
					tp->tp_uwr_lng_corr_z_2_rangeb;
				}
				else
				if (RangeDiff < -tp->tp_uwr_lng_z_2_min &&
					RangeDiff > -tp->tp_uwr_lng_z_2_max &&
					sequency == 0) {
					AddOffset =
					tp->tp_uwr_lng_corr_z_2_rangea;
				}
				else
				if (RangeDiff < tp->tp_uwr_lng_z_3_max &&
					RangeDiff > tp->tp_uwr_lng_z_3_min) {
					AddOffset =
					tp->tp_uwr_lng_corr_z_3_rangea;
				}
				else
					uwr_status = 0;
				break;

			default:
				uwr_status = 0;
				break;
			}
		}

		if (uwr_status) {
			pRangeData->RangeMilliMeter += AddOffset;
			pRangeData->RangeMinMilliMeter += AddOffset;
			pRangeData->RangeMaxMilliMeter += AddOffset;
			pRangeData->ExtendedRange = 1;
			pRangeData->RangeStatus = 0;
		}

	}

	pdev->PreviousRangeMilliMeter[iteration] = RangeMillimeterInit;
	pdev->PreviousRangeStatus[iteration] = pRangeData->RangeStatus;
	pdev->PreviousExtendedRange[iteration] = pRangeData->ExtendedRange;
	pdev->PreviousRangeActiveResults = active_results;

	Range = pRangeData->RangeMilliMeter;
	if ((pRangeData->RangeStatus ==  VL53LX_RANGESTATUS_RANGE_VALID) &&
		(Range < 0)) {
		if (Range < BDTable[VL53LX_TUNING_PROXY_MIN])
			pRangeData->RangeStatus =
					 VL53LX_RANGESTATUS_RANGE_INVALID;
		else
			pRangeData->RangeMilliMeter = 0;
	}

	return Status;
}


static VL53LX_Error SetMeasurementData(VL53LX_DEV Dev,
	VL53LX_range_results_t *presults,
	VL53LX_MultiRangingData_t *pMultiRangingData)
{
	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);
	uint8_t i;
	uint8_t iteration;
	VL53LX_TargetRangeData_t *pRangeData;
	VL53LX_range_data_t *presults_data;
	VL53LX_Error Status = VL53LX_ERROR_NONE;
	uint8_t ActiveResults;

	pMultiRangingData->NumberOfObjectsFound = presults->active_results;
	pMultiRangingData->HasXtalkValueChanged =
			presults->smudge_corrector_data.new_xtalk_applied_flag;


	pMultiRangingData->TimeStamp = 0;

	pMultiRangingData->StreamCount = presults->stream_count;

	ActiveResults = presults->active_results;
	if (ActiveResults < 1)

		iteration = 1;
	else
		iteration = ActiveResults;
	for (i = 0; i < iteration; i++) {
		pRangeData = &(pMultiRangingData->RangeData[i]);

		presults_data = &(presults->VL53LX_p_003[i]);
		if (Status == VL53LX_ERROR_NONE)
			Status = SetTargetData(Dev, ActiveResults,
					pMultiRangingData->StreamCount,
					i,
					presults->device_status,
					presults_data,
					pRangeData);

		pMultiRangingData->EffectiveSpadRtnCount =
				presults_data->VL53LX_p_004;

	}
	pdev->PreviousStreamCount = pdev->VL53LX_LLDriverCommonData->hist_data.result__stream_count;
	for (i = iteration; i < VL53LX_MAX_RANGE_RESULTS; i++) {
		pdev->PreviousRangeMilliMeter[i] = 0;
		pdev->PreviousRangeStatus[i] = 255;
		pdev->PreviousExtendedRange[i] = 0;
	}

	return Status;
}


VL53LX_Error VL53LX_GetMultiRangingData(VL53LX_DEV Dev,
		VL53LX_MultiRangingData_t *pMultiRangingData)
{
	VL53LX_Error Status = VL53LX_ERROR_NONE;
	VL53LX_LLDriverData_t *pdev =
			VL53LXDevStructGetLLDriverHandle(Dev);
	VL53LX_range_results_t *presults =
			(VL53LX_range_results_t *) pdev->VL53LX_LLDriverCommonData->wArea1;

	LOG_FUNCTION_START("");


	memset(pMultiRangingData, 0xFF,
		sizeof(VL53LX_MultiRangingData_t));


	Status = VL53LX_get_device_results(
				Dev,
				VL53LX_DEVICERESULTSLEVEL_FULL,
				presults);

	Status = SetMeasurementData(Dev,
					presults,
					pMultiRangingData);

	LOG_FUNCTION_END(Status);
	return Status;
}

VL53LX_Error VL53LX_GetAdditionalData(VL53LX_DEV Dev,
		VL53LX_AdditionalData_t *pAdditionalData)
{
	VL53LX_Error Status = VL53LX_ERROR_NONE;

	LOG_FUNCTION_START("");

	Status = VL53LX_get_additional_data(Dev, pAdditionalData);

	LOG_FUNCTION_END(Status);
	return Status;
}






VL53LX_Error VL53LX_SetTuningParameter(VL53LX_DEV Dev,
		uint16_t TuningParameterId, int32_t TuningParameterValue)
{
	VL53LX_Error Status = VL53LX_ERROR_NONE;

	LOG_FUNCTION_START("");

	if (TuningParameterId ==
		VL53LX_TUNINGPARM_DYNXTALK_NODETECT_XTALK_OFFSET_KCPS)
		return VL53LX_ERROR_INVALID_PARAMS;

	if (TuningParameterId >= 32768)
		Status = VL53LX_set_tuning_parm(Dev,
			TuningParameterId,
			TuningParameterValue);
	else {
		if (TuningParameterId < VL53LX_TUNING_MAX_TUNABLE_KEY)
			BDTable[TuningParameterId] = TuningParameterValue;
		else
			Status = VL53LX_ERROR_INVALID_PARAMS;
	}

	LOG_FUNCTION_END(Status);
	return Status;
}

VL53LX_Error VL53LX_GetTuningParameter(VL53LX_DEV Dev,
		uint16_t TuningParameterId, int32_t *pTuningParameterValue)
{
	VL53LX_Error Status = VL53LX_ERROR_NONE;

	LOG_FUNCTION_START("");

	if (TuningParameterId >= 32768)
		Status = VL53LX_get_tuning_parm(Dev,
			TuningParameterId,
			pTuningParameterValue);
	else {
		if (TuningParameterId < VL53LX_TUNING_MAX_TUNABLE_KEY)
			*pTuningParameterValue = BDTable[TuningParameterId];
		else
			Status = VL53LX_ERROR_INVALID_PARAMS;
	}

	LOG_FUNCTION_END(Status);
	return Status;
}


VL53LX_Error VL53LX_PerformRefSpadManagement(VL53LX_DEV Dev)
{
	VL53LX_Error Status = VL53LX_ERROR_NONE;
	VL53LX_Error RawStatus;
	uint8_t dcrbuffer[24];
	uint8_t *commbuf;
	uint8_t numloc[2] = {5, 3};
	VL53LX_LLDriverData_t *pdev;
	VL53LX_customer_nvm_managed_t *pc;
	VL53LX_DistanceModes DistanceMode;

	LOG_FUNCTION_START("");

	pdev = VL53LXDevStructGetLLDriverHandle(Dev);
	pc = &pdev->customer;

	if (Status == VL53LX_ERROR_NONE) {
		DistanceMode = VL53LXDevDataGet(Dev,
				CurrentParameters.DistanceMode);
		Status = VL53LX_run_ref_spad_char(Dev, &RawStatus);

		if (Status == VL53LX_ERROR_NONE)
			Status = VL53LX_SetDistanceMode(Dev, DistanceMode);
	}

	if (Status == VL53LX_WARNING_REF_SPAD_CHAR_RATE_TOO_HIGH) {

		Status = VL53LX_read_nvm_raw_data(Dev,
				(uint8_t)(0xA0 >> 2),
				(uint8_t)(24 >> 2),
				dcrbuffer);

		if (Status == VL53LX_ERROR_NONE)
			Status = VL53LX_WriteMulti(Dev,
				VL53LX_REF_SPAD_MAN__NUM_REQUESTED_REF_SPADS,
				numloc, 2);

		if (Status == VL53LX_ERROR_NONE) {
			pc->ref_spad_man__num_requested_ref_spads = numloc[0];
			pc->ref_spad_man__ref_location = numloc[1];
		}

		commbuf = &dcrbuffer[16];



		if (Status == VL53LX_ERROR_NONE)
			Status = VL53LX_WriteMulti(Dev,
				VL53LX_GLOBAL_CONFIG__SPAD_ENABLES_REF_0,
				commbuf, 6);

		if (Status == VL53LX_ERROR_NONE) {
			pc->global_config__spad_enables_ref_0 = commbuf[0];
			pc->global_config__spad_enables_ref_1 = commbuf[1];
			pc->global_config__spad_enables_ref_2 = commbuf[2];
			pc->global_config__spad_enables_ref_3 = commbuf[3];
			pc->global_config__spad_enables_ref_4 = commbuf[4];
			pc->global_config__spad_enables_ref_5 = commbuf[5];
		}

	}

	LOG_FUNCTION_END(Status);
	return Status;
}


VL53LX_Error VL53LX_SmudgeCorrectionEnable(VL53LX_DEV Dev,
		VL53LX_SmudgeCorrectionModes Mode)
{
	VL53LX_Error Status = VL53LX_ERROR_NONE;
	VL53LX_Error s1 = VL53LX_ERROR_NONE;
	VL53LX_Error s2 = VL53LX_ERROR_NONE;
	VL53LX_Error s3 = VL53LX_ERROR_NONE;

	LOG_FUNCTION_START("");

	switch (Mode) {
	case VL53LX_SMUDGE_CORRECTION_NONE:
		s1 = VL53LX_dynamic_xtalk_correction_disable(Dev);
		s2 = VL53LX_dynamic_xtalk_correction_apply_disable(Dev);
		s3 = VL53LX_dynamic_xtalk_correction_single_apply_disable(Dev);
		break;
	case VL53LX_SMUDGE_CORRECTION_CONTINUOUS:
		s1 = VL53LX_dynamic_xtalk_correction_enable(Dev);
		s2 = VL53LX_dynamic_xtalk_correction_apply_enable(Dev);
		s3 = VL53LX_dynamic_xtalk_correction_single_apply_disable(Dev);
		break;
	case VL53LX_SMUDGE_CORRECTION_SINGLE:
		s1 = VL53LX_dynamic_xtalk_correction_enable(Dev);
		s2 = VL53LX_dynamic_xtalk_correction_apply_enable(Dev);
		s3 = VL53LX_dynamic_xtalk_correction_single_apply_enable(Dev);
		break;
	case VL53LX_SMUDGE_CORRECTION_DEBUG:
		s1 = VL53LX_dynamic_xtalk_correction_enable(Dev);
		s2 = VL53LX_dynamic_xtalk_correction_apply_disable(Dev);
		s3 = VL53LX_dynamic_xtalk_correction_single_apply_disable(Dev);
		break;
	default:
		Status = VL53LX_ERROR_INVALID_PARAMS;
		break;
	}

	if (Status == VL53LX_ERROR_NONE) {
		Status = s1;
		if (Status == VL53LX_ERROR_NONE)
			Status = s2;
		if (Status == VL53LX_ERROR_NONE)
			Status = s3;
	}

	LOG_FUNCTION_END(Status);
	return Status;
}


VL53LX_Error VL53LX_SetXTalkCompensationEnable(VL53LX_DEV Dev,
	uint8_t XTalkCompensationEnable)
{
	VL53LX_Error Status = VL53LX_ERROR_NONE;

	LOG_FUNCTION_START("");

	if (XTalkCompensationEnable == 0)
		Status = VL53LX_disable_xtalk_compensation(Dev);
	else
		Status = VL53LX_enable_xtalk_compensation(Dev);

	LOG_FUNCTION_END(Status);
	return Status;
}


VL53LX_Error VL53LX_GetXTalkCompensationEnable(VL53LX_DEV Dev,
	uint8_t *pXTalkCompensationEnable)
{
	VL53LX_Error Status = VL53LX_ERROR_NONE;

	LOG_FUNCTION_START("");

	VL53LX_get_xtalk_compensation_enable(
		Dev,
		pXTalkCompensationEnable);

	LOG_FUNCTION_END(Status);
	return Status;
}

VL53LX_Error VL53LX_PerformXTalkCalibration(VL53LX_DEV Dev)
{
	VL53LX_Error Status = VL53LX_ERROR_NONE;
	VL53LX_Error UStatus;
	int16_t CalDistanceMm;
	VL53LX_xtalk_calibration_results_t xtalk;

	VL53LX_CalibrationData_t caldata;
	VL53LX_LLDriverData_t *pLLData;
	int i;
	uint32_t *pPlaneOffsetKcps;
	uint32_t Margin =
			BDTable[VL53LX_TUNING_XTALK_FULL_ROI_BIN_SUM_MARGIN];
	uint32_t DefaultOffset =
			BDTable[VL53LX_TUNING_XTALK_FULL_ROI_DEFAULT_OFFSET];
	uint32_t *pLLDataPlaneOffsetKcps;
	uint32_t sum = 0;
	uint8_t binok = 0;

	LOG_FUNCTION_START("");

	pPlaneOffsetKcps =
	&caldata.customer.algo__crosstalk_compensation_plane_offset_kcps;
	pLLData = VL53LXDevStructGetLLDriverHandle(Dev);
	pLLDataPlaneOffsetKcps =
	&pLLData->xtalk_cal.algo__crosstalk_compensation_plane_offset_kcps;

	CalDistanceMm = (int16_t)
	BDTable[VL53LX_TUNING_XTALK_FULL_ROI_TARGET_DISTANCE_MM];
	Status = VL53LX_run_hist_xtalk_extraction(Dev, CalDistanceMm,
			&UStatus);

	VL53LX_GetCalibrationData(Dev, &caldata);
	for (i = 0; i < VL53LX_XTALK_HISTO_BINS; i++) {
		sum += caldata.xtalkhisto.xtalk_shape.bin_data[i];
		if (caldata.xtalkhisto.xtalk_shape.bin_data[i] > 0)
			binok++;
	}
	if ((UStatus ==
		VL53LX_ERROR_XTALK_EXTRACTION_SIGMA_LIMIT_FAIL) ||
		(sum > (1024 + Margin)) || (sum < (1024 - Margin)) ||
		(binok < 3)) {
		*pPlaneOffsetKcps = DefaultOffset;
		*pLLDataPlaneOffsetKcps = DefaultOffset;
		caldata.xtalkhisto.xtalk_shape.bin_data[0] = 307;
		caldata.xtalkhisto.xtalk_shape.bin_data[1] = 410;
		caldata.xtalkhisto.xtalk_shape.bin_data[2] = 410;
		caldata.xtalkhisto.xtalk_shape.bin_data[3] = 307;
		for (i = 4; i < VL53LX_XTALK_HISTO_BINS; i++)
			caldata.xtalkhisto.xtalk_shape.bin_data[i] = 0;
		for (i = 0; i < VL53LX_BIN_REC_SIZE; i++)
			caldata.algo__xtalk_cpo_HistoMerge_kcps[i] =
				DefaultOffset + DefaultOffset * i;
		VL53LX_SetCalibrationData(Dev, &caldata);
	}

	if (Status == VL53LX_ERROR_NONE) {
		Status = VL53LX_get_current_xtalk_settings(Dev, &xtalk);
		Status = VL53LX_set_tuning_parm(Dev,
			VL53LX_TUNINGPARM_DYNXTALK_NODETECT_XTALK_OFFSET_KCPS,
			xtalk.algo__crosstalk_compensation_plane_offset_kcps);
	}

	LOG_FUNCTION_END(Status);
	return Status;
}


VL53LX_Error VL53LX_SetOffsetCorrectionMode(VL53LX_DEV Dev,
		VL53LX_OffsetCorrectionModes OffsetCorrectionMode)
{
	VL53LX_Error Status = VL53LX_ERROR_NONE;
	VL53LX_OffsetCorrectionMode offset_cor_mode;

	LOG_FUNCTION_START("");

	if (OffsetCorrectionMode ==
		VL53LX_OFFSETCORRECTIONMODE_PERVCSEL)
		offset_cor_mode =
				VL53LX_OFFSETCORRECTIONMODE__PER_VCSEL_OFFSETS;
	else {
		offset_cor_mode =
			VL53LX_OFFSETCORRECTIONMODE__MM1_MM2_OFFSETS;
		if (OffsetCorrectionMode !=
			VL53LX_OFFSETCORRECTIONMODE_STANDARD)
			Status = VL53LX_ERROR_INVALID_PARAMS;
	}

	if (Status == VL53LX_ERROR_NONE)
		Status =  VL53LX_set_offset_correction_mode(Dev,
				offset_cor_mode);

	LOG_FUNCTION_END(Status);
	return Status;
}


VL53LX_Error VL53LX_PerformOffsetSimpleCalibration(VL53LX_DEV Dev,
	int32_t CalDistanceMilliMeter)
{
	VL53LX_Error Status = VL53LX_ERROR_NONE;
	int32_t sum_ranging;
	uint8_t offset_meas;
	int16_t Max, UnderMax, OverMax, Repeat;
	int32_t total_count, inloopcount;
	int32_t IncRounding;
	int16_t meanDistance_mm;
	int16_t offset;
	VL53LX_MultiRangingData_t RangingMeasurementData;
	VL53LX_LLDriverData_t *pdev;
	uint8_t goodmeas;
	VL53LX_Error SmudgeStatus = VL53LX_ERROR_NONE;
	uint8_t smudge_corr_en;
	VL53LX_TargetRangeData_t *pRange;

	LOG_FUNCTION_START("");

	pdev = VL53LXDevStructGetLLDriverHandle(Dev);

	smudge_corr_en = pdev->VL53LX_LLDriverCommonData->smudge_correct_config.smudge_corr_enabled;
	SmudgeStatus = VL53LX_dynamic_xtalk_correction_disable(Dev);

	pdev->customer.algo__part_to_part_range_offset_mm = 0;
	pdev->customer.mm_config__inner_offset_mm = 0;
	pdev->customer.mm_config__outer_offset_mm = 0;
	memset(&pdev->per_vcsel_cal_data, 0, sizeof(pdev->per_vcsel_cal_data));
	Repeat = BDTable[VL53LX_TUNING_SIMPLE_OFFSET_CALIBRATION_REPEAT];
	Max = BDTable[
		VL53LX_TUNING_MAX_SIMPLE_OFFSET_CALIBRATION_SAMPLE_NUMBER];
	UnderMax = 1 + (Max / 2);
	OverMax = Max + (Max / 2);
	sum_ranging = 0;
	total_count = 0;

	while ((Repeat > 0) && (Status == VL53LX_ERROR_NONE)) {
		Status = VL53LX_StartMeasurement(Dev);

		if (Status == VL53LX_ERROR_NONE) {
			VL53LX_WaitMeasurementDataReady(Dev);
			VL53LX_GetMultiRangingData(Dev,
				&RangingMeasurementData);
			VL53LX_ClearInterruptAndStartMeasurement(Dev);
		}

		inloopcount = 0;
		offset_meas = 0;
		while ((Status == VL53LX_ERROR_NONE) && (inloopcount < Max) &&
				(offset_meas < OverMax)) {
			Status = VL53LX_WaitMeasurementDataReady(Dev);
			if (Status == VL53LX_ERROR_NONE)
				Status = VL53LX_GetMultiRangingData(Dev,
						&RangingMeasurementData);
			pRange = &(RangingMeasurementData.RangeData[0]);
			goodmeas = (pRange->RangeStatus ==
				VL53LX_RANGESTATUS_RANGE_VALID);
			if ((Status == VL53LX_ERROR_NONE) && goodmeas) {
				sum_ranging += pRange->RangeMilliMeter;
				inloopcount++;
			}
			Status = VL53LX_ClearInterruptAndStartMeasurement(Dev);
			offset_meas++;
		}
		total_count += inloopcount;


		if (inloopcount < UnderMax)
			Status = VL53LX_ERROR_OFFSET_CAL_NO_SAMPLE_FAIL;

		VL53LX_StopMeasurement(Dev);

		Repeat--;

	}

	if ((SmudgeStatus == VL53LX_ERROR_NONE) && (smudge_corr_en == 1))
		SmudgeStatus = VL53LX_dynamic_xtalk_correction_enable(Dev);

	if ((sum_ranging < 0) ||
		(sum_ranging > ((int32_t) total_count * 0xffff)))
		Status = VL53LX_WARNING_OFFSET_CAL_SIGMA_TOO_HIGH;

	if ((Status == VL53LX_ERROR_NONE) && (total_count > 0)) {
		IncRounding = total_count / 2;
		meanDistance_mm = (int16_t)((sum_ranging + IncRounding)
				/ total_count);
		offset = (int16_t)CalDistanceMilliMeter - meanDistance_mm;
		pdev->customer.algo__part_to_part_range_offset_mm = 0;
		pdev->customer.mm_config__inner_offset_mm = offset;
		pdev->customer.mm_config__outer_offset_mm = offset;

		Status = VL53LX_set_customer_nvm_managed(Dev,
				&(pdev->customer));
	}

	LOG_FUNCTION_END(Status);
	return Status;
}

VL53LX_Error VL53LX_PerformOffsetZeroDistanceCalibration(VL53LX_DEV Dev)
{
	#define START_OFFSET 50
	VL53LX_Error Status = VL53LX_ERROR_NONE;
	int32_t sum_ranging;
	uint8_t offset_meas;
	int16_t Max, UnderMax, OverMax, Repeat;
	int32_t total_count, inloopcount;
	int32_t IncRounding;
	int16_t meanDistance_mm;
	int16_t offset, ZeroDistanceOffset;
	VL53LX_MultiRangingData_t RangingMeasurementData;
	VL53LX_LLDriverData_t *pdev;
	uint8_t goodmeas;
	VL53LX_Error SmudgeStatus = VL53LX_ERROR_NONE;
	uint8_t smudge_corr_en;
	VL53LX_TargetRangeData_t *pRange;

	LOG_FUNCTION_START("");

	pdev = VL53LXDevStructGetLLDriverHandle(Dev);
	smudge_corr_en = pdev->VL53LX_LLDriverCommonData->smudge_correct_config.smudge_corr_enabled;
	SmudgeStatus = VL53LX_dynamic_xtalk_correction_disable(Dev);
	pdev->customer.algo__part_to_part_range_offset_mm = 0;
	pdev->customer.mm_config__inner_offset_mm = START_OFFSET;
	pdev->customer.mm_config__outer_offset_mm = START_OFFSET;
	memset(&pdev->per_vcsel_cal_data, 0, sizeof(pdev->per_vcsel_cal_data));
	ZeroDistanceOffset = BDTable[
		VL53LX_TUNING_ZERO_DISTANCE_OFFSET_NON_LINEAR_FACTOR];
	Repeat = BDTable[VL53LX_TUNING_SIMPLE_OFFSET_CALIBRATION_REPEAT];
	Max =
	BDTable[VL53LX_TUNING_MAX_SIMPLE_OFFSET_CALIBRATION_SAMPLE_NUMBER];
	UnderMax = 1 + (Max / 2);
	OverMax = Max + (Max / 2);
	sum_ranging = 0;
	total_count = 0;

	while ((Repeat > 0) && (Status == VL53LX_ERROR_NONE)) {
		Status = VL53LX_StartMeasurement(Dev);
		if (Status == VL53LX_ERROR_NONE) {
			VL53LX_WaitMeasurementDataReady(Dev);
			VL53LX_GetMultiRangingData(Dev,
				&RangingMeasurementData);
			VL53LX_ClearInterruptAndStartMeasurement(Dev);
		}
		inloopcount = 0;
		offset_meas = 0;
		while ((Status == VL53LX_ERROR_NONE) && (inloopcount < Max) &&
				(offset_meas < OverMax)) {
			Status = VL53LX_WaitMeasurementDataReady(Dev);
			if (Status == VL53LX_ERROR_NONE)
				Status = VL53LX_GetMultiRangingData(Dev,
						&RangingMeasurementData);
			pRange = &(RangingMeasurementData.RangeData[0]);
			goodmeas = (pRange->RangeStatus ==
				VL53LX_RANGESTATUS_RANGE_VALID);
			if ((Status == VL53LX_ERROR_NONE) && goodmeas) {
				sum_ranging = sum_ranging +
					pRange->RangeMilliMeter;
				inloopcount++;
			}
			Status = VL53LX_ClearInterruptAndStartMeasurement(Dev);
			offset_meas++;
		}
		total_count += inloopcount;
		if (inloopcount < UnderMax)
			Status = VL53LX_ERROR_OFFSET_CAL_NO_SAMPLE_FAIL;
		VL53LX_StopMeasurement(Dev);
		Repeat--;
	}
	if ((SmudgeStatus == VL53LX_ERROR_NONE) && (smudge_corr_en == 1))
		SmudgeStatus = VL53LX_dynamic_xtalk_correction_enable(Dev);
	if ((sum_ranging < 0) ||
		(sum_ranging > ((int32_t) total_count * 0xffff)))
		Status = VL53LX_WARNING_OFFSET_CAL_SIGMA_TOO_HIGH;

	if ((Status == VL53LX_ERROR_NONE) && (total_count > 0)) {
		IncRounding = total_count / 2;
		meanDistance_mm = (int16_t)
			((sum_ranging + IncRounding) / total_count);
		offset = START_OFFSET - meanDistance_mm + ZeroDistanceOffset;
		pdev->customer.algo__part_to_part_range_offset_mm = 0;
		pdev->customer.mm_config__inner_offset_mm = offset;
		pdev->customer.mm_config__outer_offset_mm = offset;
		Status = VL53LX_set_customer_nvm_managed(Dev,
			&(pdev->customer));
	}

	LOG_FUNCTION_END(Status);
	return Status;
}

VL53LX_Error VL53LX_SetCalibrationData(VL53LX_DEV Dev,
		VL53LX_CalibrationData_t *pCalibrationData)
{
	VL53LX_Error Status = VL53LX_ERROR_NONE;
	VL53LX_CustomerNvmManaged_t          *pC;
	VL53LX_calibration_data_t            cal_data;
	uint32_t x;
	VL53LX_xtalk_calibration_results_t xtalk;

	LOG_FUNCTION_START("");

	cal_data.struct_version = pCalibrationData->struct_version -
			VL53LX_ADDITIONAL_CALIBRATION_DATA_STRUCT_VERSION;


	memcpy(
		&(cal_data.add_off_cal_data),
		&(pCalibrationData->add_off_cal_data),
		sizeof(VL53LX_additional_offset_cal_data_t));


	memcpy(
		&(cal_data.optical_centre),
		&(pCalibrationData->optical_centre),
		sizeof(VL53LX_optical_centre_t));


	memcpy(
		&(cal_data.xtalkhisto),
		&(pCalibrationData->xtalkhisto),
		sizeof(VL53LX_xtalk_histogram_data_t));


	memcpy(
		&(cal_data.gain_cal),
		&(pCalibrationData->gain_cal),
		sizeof(VL53LX_gain_calibration_data_t));


	memcpy(
		&(cal_data.cal_peak_rate_map),
		&(pCalibrationData->cal_peak_rate_map),
		sizeof(VL53LX_cal_peak_rate_map_t));


	memcpy(
		&(cal_data.per_vcsel_cal_data),
		&(pCalibrationData->per_vcsel_cal_data),
		sizeof(VL53LX_per_vcsel_period_offset_cal_data_t));

	pC = &pCalibrationData->customer;
	x = pC->algo__crosstalk_compensation_plane_offset_kcps;
	cal_data.customer.algo__crosstalk_compensation_plane_offset_kcps =
		(uint16_t)(x&0x0000FFFF);

	cal_data.customer.global_config__spad_enables_ref_0 =
		pC->global_config__spad_enables_ref_0;
	cal_data.customer.global_config__spad_enables_ref_1 =
		pC->global_config__spad_enables_ref_1;
	cal_data.customer.global_config__spad_enables_ref_2 =
		pC->global_config__spad_enables_ref_2;
	cal_data.customer.global_config__spad_enables_ref_3 =
		pC->global_config__spad_enables_ref_3;
	cal_data.customer.global_config__spad_enables_ref_4 =
		pC->global_config__spad_enables_ref_4;
	cal_data.customer.global_config__spad_enables_ref_5 =
		pC->global_config__spad_enables_ref_5;
	cal_data.customer.global_config__ref_en_start_select =
		pC->global_config__ref_en_start_select;
	cal_data.customer.ref_spad_man__num_requested_ref_spads =
		pC->ref_spad_man__num_requested_ref_spads;
	cal_data.customer.ref_spad_man__ref_location =
		pC->ref_spad_man__ref_location;
	cal_data.customer.algo__crosstalk_compensation_x_plane_gradient_kcps =
		pC->algo__crosstalk_compensation_x_plane_gradient_kcps;
	cal_data.customer.algo__crosstalk_compensation_y_plane_gradient_kcps =
		pC->algo__crosstalk_compensation_y_plane_gradient_kcps;
	cal_data.customer.ref_spad_char__total_rate_target_mcps =
		pC->ref_spad_char__total_rate_target_mcps;
	cal_data.customer.algo__part_to_part_range_offset_mm =
		pC->algo__part_to_part_range_offset_mm;
	cal_data.customer.mm_config__inner_offset_mm =
		pC->mm_config__inner_offset_mm;
	cal_data.customer.mm_config__outer_offset_mm =
		pC->mm_config__outer_offset_mm;

	Status = VL53LX_set_part_to_part_data(Dev, &cal_data);

	if (Status != VL53LX_ERROR_NONE)
		goto ENDFUNC;

	Status = VL53LX_get_current_xtalk_settings(Dev, &xtalk);

	if (Status != VL53LX_ERROR_NONE)
		goto ENDFUNC;

	xtalk.algo__crosstalk_compensation_plane_offset_kcps = x;

	Status = VL53LX_set_tuning_parm(Dev,
			VL53LX_TUNINGPARM_DYNXTALK_NODETECT_XTALK_OFFSET_KCPS,
			x);


	memcpy(
		&(xtalk.algo__xtalk_cpo_HistoMerge_kcps[0]),
		&(pCalibrationData->algo__xtalk_cpo_HistoMerge_kcps[0]),
		sizeof(pCalibrationData->algo__xtalk_cpo_HistoMerge_kcps));

	Status = VL53LX_set_current_xtalk_settings(Dev, &xtalk);

ENDFUNC:
	LOG_FUNCTION_END(Status);
	return Status;

}

VL53LX_Error VL53LX_GetCalibrationData(VL53LX_DEV Dev,
		VL53LX_CalibrationData_t  *pCalibrationData)
{
	VL53LX_Error Status = VL53LX_ERROR_NONE;
	VL53LX_calibration_data_t      cal_data;
	VL53LX_CustomerNvmManaged_t         *pC;
	VL53LX_customer_nvm_managed_t       *pC2;
	VL53LX_xtalk_calibration_results_t xtalk;
	uint32_t                          tmp;

	LOG_FUNCTION_START("");


	Status = VL53LX_get_part_to_part_data(Dev, &cal_data);

	pCalibrationData->struct_version = cal_data.struct_version +
			VL53LX_ADDITIONAL_CALIBRATION_DATA_STRUCT_VERSION;


	memcpy(
		&(pCalibrationData->add_off_cal_data),
		&(cal_data.add_off_cal_data),
		sizeof(VL53LX_additional_offset_cal_data_t));


	memcpy(
		&(pCalibrationData->optical_centre),
		&(cal_data.optical_centre),
		sizeof(VL53LX_optical_centre_t));


	memcpy(
		&(pCalibrationData->xtalkhisto),
		&(cal_data.xtalkhisto),
		sizeof(VL53LX_xtalk_histogram_data_t));

	memcpy(
		&(pCalibrationData->gain_cal),
		&(cal_data.gain_cal),
		sizeof(VL53LX_gain_calibration_data_t));


	memcpy(
		&(pCalibrationData->cal_peak_rate_map),
		&(cal_data.cal_peak_rate_map),
		sizeof(VL53LX_cal_peak_rate_map_t));


	memcpy(
		&(pCalibrationData->per_vcsel_cal_data),
		&(cal_data.per_vcsel_cal_data),
		sizeof(VL53LX_per_vcsel_period_offset_cal_data_t));

	pC = &pCalibrationData->customer;
	pC2 = &cal_data.customer;
	pC->global_config__spad_enables_ref_0 =
		pC2->global_config__spad_enables_ref_0;
	pC->global_config__spad_enables_ref_1 =
		pC2->global_config__spad_enables_ref_1;
	pC->global_config__spad_enables_ref_2 =
		pC2->global_config__spad_enables_ref_2;
	pC->global_config__spad_enables_ref_3 =
		pC2->global_config__spad_enables_ref_3;
	pC->global_config__spad_enables_ref_4 =
		pC2->global_config__spad_enables_ref_4;
	pC->global_config__spad_enables_ref_5 =
		pC2->global_config__spad_enables_ref_5;
	pC->global_config__ref_en_start_select =
		pC2->global_config__ref_en_start_select;
	pC->ref_spad_man__num_requested_ref_spads =
		pC2->ref_spad_man__num_requested_ref_spads;
	pC->ref_spad_man__ref_location =
		pC2->ref_spad_man__ref_location;
	pC->algo__crosstalk_compensation_x_plane_gradient_kcps =
		pC2->algo__crosstalk_compensation_x_plane_gradient_kcps;
	pC->algo__crosstalk_compensation_y_plane_gradient_kcps =
		pC2->algo__crosstalk_compensation_y_plane_gradient_kcps;
	pC->ref_spad_char__total_rate_target_mcps =
		pC2->ref_spad_char__total_rate_target_mcps;
	pC->algo__part_to_part_range_offset_mm =
		pC2->algo__part_to_part_range_offset_mm;
	pC->mm_config__inner_offset_mm =
		pC2->mm_config__inner_offset_mm;
	pC->mm_config__outer_offset_mm =
		pC2->mm_config__outer_offset_mm;

	pC->algo__crosstalk_compensation_plane_offset_kcps =
		(uint32_t)(
			pC2->algo__crosstalk_compensation_plane_offset_kcps);

	Status = VL53LX_get_current_xtalk_settings(Dev, &xtalk);

	if (Status != VL53LX_ERROR_NONE)
		goto ENDFUNC;

	tmp = xtalk.algo__crosstalk_compensation_plane_offset_kcps;
	pC->algo__crosstalk_compensation_plane_offset_kcps = tmp;
	tmp = xtalk.algo__crosstalk_compensation_x_plane_gradient_kcps;
	pC->algo__crosstalk_compensation_x_plane_gradient_kcps = tmp;
	tmp = xtalk.algo__crosstalk_compensation_y_plane_gradient_kcps;
	pC->algo__crosstalk_compensation_y_plane_gradient_kcps = tmp;

	memcpy(&(pCalibrationData->algo__xtalk_cpo_HistoMerge_kcps[0]),
		&(xtalk.algo__xtalk_cpo_HistoMerge_kcps[0]),
		sizeof(pCalibrationData->algo__xtalk_cpo_HistoMerge_kcps));
ENDFUNC:
	LOG_FUNCTION_END(Status);
	return Status;
}



VL53LX_Error VL53LX_PerformOffsetPerVcselCalibration(VL53LX_DEV Dev,
	int32_t CalDistanceMilliMeter)
{
	VL53LX_Error Status = VL53LX_ERROR_NONE;
	int32_t sum_ranging_range_A, sum_ranging_range_B;
	uint8_t offset_meas_range_A, offset_meas_range_B;
	int16_t Max, UnderMax, OverMax, Repeat;
	int32_t inloopcount;
	int32_t IncRounding;
	int16_t meanDistance_mm;
	VL53LX_MultiRangingData_t RangingMeasurementData;
	VL53LX_LLDriverData_t *pdev;
	uint8_t goodmeas;
	VL53LX_DistanceModes currentDist;
	VL53LX_DistanceModes DistMode[3] = {VL53LX_DISTANCEMODE_SHORT,
			VL53LX_DISTANCEMODE_MEDIUM, VL53LX_DISTANCEMODE_LONG};
	int16_t offsetA[3] = {0, 0, 0};
	int16_t offsetB[3] = {0, 0, 0};

	VL53LX_Error SmudgeStatus = VL53LX_ERROR_NONE;
	uint8_t smudge_corr_en, ics;
	VL53LX_TargetRangeData_t *pRange;

	LOG_FUNCTION_START("");

	pdev = VL53LXDevStructGetLLDriverHandle(Dev);

	smudge_corr_en = pdev->VL53LX_LLDriverCommonData->smudge_correct_config.smudge_corr_enabled;
	SmudgeStatus = VL53LX_dynamic_xtalk_correction_disable(Dev);

	pdev->customer.algo__part_to_part_range_offset_mm = 0;
	pdev->customer.mm_config__inner_offset_mm = 0;
	pdev->customer.mm_config__outer_offset_mm = 0;
	pdev->customer.mm_config__outer_offset_mm = 0;
	memset(&pdev->per_vcsel_cal_data, 0, sizeof(pdev->per_vcsel_cal_data));

	Repeat = 0;
	if (IsL4(Dev))
		Repeat = 1;
	Max = 2 * BDTable[
		VL53LX_TUNING_MAX_SIMPLE_OFFSET_CALIBRATION_SAMPLE_NUMBER];
	UnderMax = 1 + (Max / 2);
	OverMax = Max + (Max / 2);

	Status = VL53LX_GetDistanceMode(Dev, &currentDist);

	while ((Repeat < 3) && (Status == VL53LX_ERROR_NONE)) {
		Status = VL53LX_SetDistanceMode(Dev, DistMode[Repeat]);
		Status = VL53LX_StartMeasurement(Dev);

		if (Status == VL53LX_ERROR_NONE) {
			VL53LX_WaitMeasurementDataReady(Dev);
			VL53LX_GetMultiRangingData(Dev,
				&RangingMeasurementData);
			VL53LX_ClearInterruptAndStartMeasurement(Dev);
		}

		inloopcount = 0;
		offset_meas_range_A = 0;
		sum_ranging_range_A = 0;
		offset_meas_range_B = 0;
		sum_ranging_range_B = 0;
		while ((Status == VL53LX_ERROR_NONE) && (inloopcount < Max) &&
				(inloopcount < OverMax)) {
			Status = VL53LX_WaitMeasurementDataReady(Dev);
			if (Status == VL53LX_ERROR_NONE)
				Status = VL53LX_GetMultiRangingData(Dev,
						&RangingMeasurementData);
			pRange = &(RangingMeasurementData.RangeData[0]);
			goodmeas = (pRange->RangeStatus ==
				VL53LX_RANGESTATUS_RANGE_VALID);
			ics = pdev->ll_state.cfg_internal_stream_count;
			if ((Status == VL53LX_ERROR_NONE) && goodmeas) {
				if (ics & 0x01) {
					sum_ranging_range_A +=
						pRange->RangeMilliMeter;
					offset_meas_range_A++;
				} else {
					sum_ranging_range_B +=
						pRange->RangeMilliMeter;
					offset_meas_range_B++;
				}
				inloopcount = offset_meas_range_A +
					offset_meas_range_B;
			}
			Status = VL53LX_ClearInterruptAndStartMeasurement(Dev);
		}


		if (inloopcount < UnderMax)
			Status = VL53LX_ERROR_OFFSET_CAL_NO_SAMPLE_FAIL;

		VL53LX_StopMeasurement(Dev);


		if ((sum_ranging_range_A < 0) ||
			(sum_ranging_range_B < 0) ||
			(sum_ranging_range_A >
			((int32_t) offset_meas_range_A * 0xffff)) ||
			(sum_ranging_range_B >
			((int32_t) offset_meas_range_B * 0xffff))) {
			Status = VL53LX_WARNING_OFFSET_CAL_SIGMA_TOO_HIGH;
		}

		if ((Status == VL53LX_ERROR_NONE) &&
			(offset_meas_range_A > 0)) {
			IncRounding = offset_meas_range_A / 2;
			meanDistance_mm = (int16_t)
				((sum_ranging_range_A + IncRounding)
				/ offset_meas_range_A);
			offsetA[Repeat] = (int16_t)
				CalDistanceMilliMeter - meanDistance_mm;
		}

		if ((Status == VL53LX_ERROR_NONE) &&
			(offset_meas_range_B > 0)) {
			IncRounding = offset_meas_range_B / 2;
			meanDistance_mm = (int16_t)
				((sum_ranging_range_B + IncRounding)
				/ offset_meas_range_B);
			offsetB[Repeat] = (int16_t)
				CalDistanceMilliMeter - meanDistance_mm;
		}
		Repeat++;
	}

	if ((SmudgeStatus == VL53LX_ERROR_NONE) && (smudge_corr_en == 1))
		SmudgeStatus = VL53LX_dynamic_xtalk_correction_enable(Dev);

	if (Status == VL53LX_ERROR_NONE) {
		pdev->per_vcsel_cal_data.short_a_offset_mm  = offsetA[0];
		pdev->per_vcsel_cal_data.short_b_offset_mm  = offsetB[0];
		pdev->per_vcsel_cal_data.medium_a_offset_mm = offsetA[1];
		pdev->per_vcsel_cal_data.medium_b_offset_mm = offsetB[1];
		pdev->per_vcsel_cal_data.long_a_offset_mm   = offsetA[2];
		pdev->per_vcsel_cal_data.long_b_offset_mm   = offsetB[2];
	}

	VL53LX_SetDistanceMode(Dev, currentDist);

	LOG_FUNCTION_END(Status);
	return Status;
}

VL53LX_Error VL53LX_GetOpticalCenter(VL53LX_DEV Dev,
		FixPoint1616_t *pOpticalCenterX,
		FixPoint1616_t *pOpticalCenterY)
{
	VL53LX_Error Status = VL53LX_ERROR_NONE;
	VL53LX_calibration_data_t  CalibrationData;

	LOG_FUNCTION_START("");

	*pOpticalCenterX = 0;
	*pOpticalCenterY = 0;
	Status = VL53LX_get_part_to_part_data(Dev, &CalibrationData);
	if (Status == VL53LX_ERROR_NONE) {
		*pOpticalCenterX = VL53LX_FIXPOINT44TOFIXPOINT1616(
				CalibrationData.optical_centre.x_centre);
		*pOpticalCenterY = VL53LX_FIXPOINT44TOFIXPOINT1616(
				CalibrationData.optical_centre.y_centre);
	}

	LOG_FUNCTION_END(Status);
	return Status;
}


