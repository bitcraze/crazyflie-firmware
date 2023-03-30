
/* SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause */
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53LX and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */

#ifndef _VL53LX_API_H_
#define _VL53LX_API_H_

#include "vl53lx_api_core.h"
#include "vl53lx_preset_setup.h"

#ifdef __cplusplus
extern "C"
{
#endif

/** @defgroup VL53LX_group VL53LX Function Definition
 *  @brief    VL53LX Function Definition
 *  @{
 */

/** @defgroup VL53LX_general_group VL53LX General Functions
 *  @brief    General functions and definitions
 *  @{
 */

/**
 * @brief Return the VL53LX driver Version
 *
 * @note This function doesn't access to the device
 *
 * @param   pVersion              Rer to current driver Version
 * @return  VL53LX_ERROR_NONE     Success
 * @return  "Other error code"    See ::VL53LX_Error
 */
VL53LX_Error VL53LX_GetVersion(VL53LX_Version_t *pVersion);

/**
 * @brief Reads the Product Revision for a for given Device
 * This function can be used to distinguish cut1.0 from cut1.1.
 *
 * @param   Dev                 Device Handle
 * @param   pProductRevisionMajor  Pointer to Product Revision Major
 * for a given Device
 * @param   pProductRevisionMinor  Pointer to Product Revision Minor
 * for a given Device
 * @return  VL53LX_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53LX_Error
 */
VL53LX_Error VL53LX_GetProductRevision(VL53LX_DEV Dev,
	uint8_t *pProductRevisionMajor, uint8_t *pProductRevisionMinor);

/**
 * @brief Reads the Device information for given Device
 *
 * @note This function Access to the device
 *
 * @param   Dev                 Device Handle
 * @param   pVL53LX_DeviceInfo  Pointer to current device info for a given
 *  Device
 * @return  VL53LX_ERROR_NONE   Success
 * @return  "Other error code"  See ::VL53LX_Error
 */
VL53LX_Error VL53LX_GetDeviceInfo(VL53LX_DEV Dev,
	VL53LX_DeviceInfo_t *pVL53LX_DeviceInfo);

/**
 * @brief Reads the Device unique identifier
 *
 * @note This function Access to the device
 *
 * @param   Dev                 Device Handle
 * @param   pUid                Pointer to current device unique ID
 * @return  VL53LX_ERROR_NONE   Success
 * @return  "Other error code"  See ::VL53LX_Error
 */
VL53LX_Error VL53LX_GetUID(VL53LX_DEV Dev, uint64_t *pUid);


/** @} VL53LX_general_group */

/** @defgroup VL53LX_init_group VL53LX Init Functions
 *  @brief    VL53LX Init Functions
 *  @{
 */

/**
 * @brief Set new device address
 *
 * After completion the device will answer to the new address programmed.
 * This function should be called when several devices are used in parallel
 * before start programming the sensor.
 * When a single device us used, there is no need to call this function.
 *
 * @note This function Access to the device
 *
 * @param   Dev                   Device Handle
 * @param   DeviceAddress         The new Device address
 * @return  VL53LX_ERROR_NONE     Success
 * @return  "Other error code"    See ::VL53LX_Error
 */
VL53LX_Error VL53LX_SetDeviceAddress(VL53LX_DEV Dev,
	uint8_t DeviceAddress);

/**
 *
 * @brief One time device initialization
 *
 * To be called after device has been powered on and booted
 * see @a VL53LX_WaitDeviceBooted()
 *
 * @par Function Description
 * When not used after a fresh device "power up", it may return
 * @a #VL53LX_ERROR_CALIBRATION_WARNING meaning wrong calibration data
 * may have been fetched from device that can result in ranging offset error\n
 * If VL53LX_DataInit is called several times then the application must restore
 * calibration calling @a VL53LX_SetOffsetCalibrationData()
 * It implies application has gathered calibration data thanks to
 * @a VL53LX_GetOffsetCalibrationData() after an initial calibration stage.
 *
 * @note This function Access to the device
 *
 * @param   Dev                   Device Handle
 * @return  VL53LX_ERROR_NONE     Success
 * @return  "Other error code"    See ::VL53LX_Error
 */
VL53LX_Error VL53LX_DataInit(VL53LX_DEV Dev);

/**
 * @brief Wait for device booted after chip enable (hardware standby)
 *
 * @param   Dev                   Device Handle
 * @return  VL53LX_ERROR_NONE     Success
 * @return  "Other error code"    See ::VL53LX_Error
 *
 */
VL53LX_Error VL53LX_WaitDeviceBooted(VL53LX_DEV Dev);


/** @} VL53LX_init_group */

/** @defgroup VL53LX_parameters_group VL53LX Parameters Functions
 *  @brief    Functions used to prepare and setup the device
 *  @{
 */

/**
 * @brief  Set the distance mode
 * @par Function Description
 * Set the distance mode to be used for the next ranging.<br>
 * The modes Short, Medium and Long are used to optimize the ranging accuracy
 * in a specific range of distance.<br> The user select one of these modes to
 * select the distance range.
 * @note This function doesn't Access to the device
 *
 * @warning This function should be called after @a VL53LX_DataInit().

 * @param   Dev                   Device Handle
 * @param   DistanceMode          Distance mode to apply, valid values are:
 * @li VL53LX_DISTANCEMODE_SHORT
 * @li VL53LX_DISTANCEMODE_MEDIUM
 * @li VL53LX_DISTANCEMODE_LONG
 * @return  VL53LX_ERROR_NONE               Success
 * @return  VL53LX_ERROR_MODE_NOT_SUPPORTED This error occurs when DistanceMode
 *                                          is not in the supported list
 * @return  "Other error code"              See ::VL53LX_Error
 */
VL53LX_Error VL53LX_SetDistanceMode(VL53LX_DEV Dev,
		VL53LX_DistanceModes DistanceMode);

/**
 * @brief  Get the distance mode
 * @par Function Description
 * Get the distance mode used for the next ranging.
 *
 * @param   Dev                   Device Handle
 * @param   *pDistanceMode        Pointer to Distance mode
 * @return  VL53LX_ERROR_NONE            Success
 * @return  "Other error code"           See ::VL53LX_Error
 */
VL53LX_Error VL53LX_GetDistanceMode(VL53LX_DEV Dev,
		VL53LX_DistanceModes *pDistanceMode);


/**
 * @brief Set Ranging Timing Budget in microseconds
 *
 * @par Function Description
 * Defines the maximum time allowed by the user to the device to run a
 * full ranging sequence for the current mode (ranging, histogram, ASL ...)
 *
 * @param   Dev                                Device Handle
 * @param MeasurementTimingBudgetMicroSeconds  Max measurement time in
 * microseconds.
 * @return  VL53LX_ERROR_NONE            Success
 * @return  VL53LX_ERROR_INVALID_PARAMS  Error timing parameter not
 *                                       supported.
 *                                       The maximum accepted value for the
 *                                       computed timing budget is 10 seconds
 *                                       the minimum value depends on the preset
 *                                       mode selected.
 * @return  "Other error code"           See ::VL53LX_Error
 */
VL53LX_Error VL53LX_SetMeasurementTimingBudgetMicroSeconds(
	VL53LX_DEV Dev, uint32_t MeasurementTimingBudgetMicroSeconds);

/**
 * @brief Get Ranging Timing Budget in microseconds
 *
 * @par Function Description
 * Returns the programmed the maximum time allowed by the user to the
 * device to run a full ranging sequence for the current mode
 * (ranging, histogram, ...)
 *
 * @param   Dev                                    Device Handle
 * @param   pMeasurementTimingBudgetMicroSeconds   Max measurement time in
 * microseconds.
 * @return  VL53LX_ERROR_NONE            Success
 * @return  "Other error code"           See ::VL53LX_Error
 */
VL53LX_Error VL53LX_GetMeasurementTimingBudgetMicroSeconds(
	VL53LX_DEV Dev, uint32_t *pMeasurementTimingBudgetMicroSeconds);
/** @} VL53LX_parameters_group */


/** @defgroup VL53LX_ROI_group VL53LX ROI Functions
 *  @brief    Functions used to select ROIs
 *  @{
 */

/**
 * @brief Set the ROI  to be used for ranging
 *
 * @par Function Description
 * The user defined ROI is a rectangle described as per the following system
 * from the Top Left corner to the Bottom Right corner.
 * <br>Minimal ROI size is 4x4 spads
 * @image html roi_coord.png
 *
 * @param   Dev                      Device Handle
 * @param   pUserROi                 Pointer to the Structure defining the ROI
 * @return  VL53LX_ERROR_NONE            Success
 * @return  "Other error code"           See ::VL53LX_Error
 */
VL53LX_Error VL53LX_SetUserROI(VL53LX_DEV Dev,
		VL53LX_UserRoi_t *pUserROi);

/**
 * @brief Get the ROI managed by the Device
 *
 * @par Function Description
 * Get the ROI managed by the Device
 *
 * @param   Dev                   Device Handle
 * @param   pUserROi                 Pointer to the Structure defining the ROI
 * @return  VL53LX_ERROR_NONE            Success
 * @return  "Other error code"           See ::VL53LX_Error
 */
VL53LX_Error VL53LX_GetUserROI(VL53LX_DEV Dev,
		VL53LX_UserRoi_t *pUserROi);

/** @} VL53LX_ROI_group */


/** @defgroup VL53LX_measurement_group VL53LX Measurement Functions
 *  @brief    Functions used for the measurements
 *  @{
 */

/**
 * @brief Start device measurement
 *
 * @details Started measurement will depend on distance parameter set through
 * @a VL53LX_SetDistanceMode()
 *
 * @note This function Access to the device
 *
 * @param   Dev                  Device Handle
 * @return  VL53LX_ERROR_NONE                  Success
 * @return  VL53LX_ERROR_TIME_OUT    Time out on start measurement
 * @return  VL53LX_ERROR_INVALID_PARAMS This error might occur in timed mode
 * when inter measurement period is smaller or too close to the timing budget.
 * In such case measurements are not started and user must correct the timings
 * passed to @a VL53LX_SetMeasurementTimingBudgetMicroSeconds() and
 * @a VL53LX_SetInterMeasurementPeriodMilliSeconds() functions.
 * @return  "Other error code"   See ::VL53LX_Error
 */
VL53LX_Error VL53LX_StartMeasurement(VL53LX_DEV Dev);

/**
 * @brief Stop device measurement
 *
 * @details Will set the device in standby mode at end of current measurement\n
 *          Not necessary in single mode as device shall return automatically
 *          in standby mode at end of measurement.
 *
 * @note This function Access to the device
 *
 * @param   Dev                  Device Handle
 * @return  VL53LX_ERROR_NONE    Success
 * @return  "Other error code"   See ::VL53LX_Error
 */
VL53LX_Error VL53LX_StopMeasurement(VL53LX_DEV Dev);

/**
 * @brief Clear the Interrupt flag and start new measurement
 * *
 * @note This function Access to the device
 *
 * @param   Dev                  Device Handle
 * @return  VL53LX_ERROR_NONE    Success
 * @return  "Other error code"   See ::VL53LX_Error
 */
VL53LX_Error VL53LX_ClearInterruptAndStartMeasurement(VL53LX_DEV Dev);

/**
 * @brief Return Measurement Data Ready
 *
 * @par Function Description
 * This function indicate that a measurement data is ready.
 * This function is used for non-blocking capture.
 *
 * @note This function Access to the device
 *
 * @param   Dev                    Device Handle
 * @param   pMeasurementDataReady  Pointer to Measurement Data Ready.
 * 0 = data not ready, 1 = data ready
 * @return  VL53LX_ERROR_NONE      Success
 * @return  "Other error code"     See ::VL53LX_Error
 */
VL53LX_Error VL53LX_GetMeasurementDataReady(VL53LX_DEV Dev,
	uint8_t *pMeasurementDataReady);

/**
 * @brief Wait for measurement data ready.
 * Blocking function.
 * Note that the timeout is given by:
 * VL53LX_RANGE_COMPLETION_POLLING_TIMEOUT_MS defined in def.h
 *
 *
 * @note This function Access to the device
 *
 * @param   Dev      Device Handle
 * @return  VL53LX_ERROR_NONE        Success
 * @return  VL53LX_ERROR_TIME_OUT In case of timeout
 */
VL53LX_Error VL53LX_WaitMeasurementDataReady(VL53LX_DEV Dev);


/**
 * @brief Retrieve all measurements from device with the current setup
 *
 * @par Function Description
 * Get data from last successful Ranging measurement
 *
 * @warning USER must call @a VL53LX_ClearInterruptAndStartMeasurement() prior
 * to call again this function
 *
 * @note This function Access to the device
 *
 * @note The first valid value returned by this function will have a range
 * status equal to VL53LX_RANGESTATUS_RANGE_VALID_NO_WRAP_CHECK which means that
 * the data is valid but no wrap around check have been done. User should take
 * care about that.
 *
 * @param   Dev                      Device Handle
 * @param   pMultiRangingData        Pointer to the data structure to fill up.
 * @return  VL53LX_ERROR_NONE        Success
 * @return  "Other error code"       See ::VL53LX_Error
 */
VL53LX_Error VL53LX_GetMultiRangingData(VL53LX_DEV Dev,
		VL53LX_MultiRangingData_t *pMultiRangingData);

/**
 * @brief Get Additional Data
 *
 * @par Function Description
 * This function is used to get lld debugging data on the last histogram
 * measurement. shall be called when a new measurement is ready (interrupt or
 * positive VL53LX_GetMeasurementDataReady() polling) and before a call to
 * VL53LX_ClearInterruptAndStartMeasurement().
 *
 * @param   Dev                      Device Handle
 * @param   pAdditionalData          Pointer to Additional data
 * @return  VL53LX_ERROR_NONE        Success
 * @return  "Other error code"       See ::VL53LX_Error
 */
VL53LX_Error VL53LX_GetAdditionalData(VL53LX_DEV Dev,
		VL53LX_AdditionalData_t *pAdditionalData);


/** @} VL53LX_measurement_group */

/** @defgroup VL53LX_Calibration_group VL53LX Calibration Functions
 *  @brief    Functions used for Calibration
 *  @{
 */


/**
 * @brief Set Tuning Parameter value for a given parameter ID
 *
 * @par Function Description
 * This function is used to improve the performance of the device. It permit to
 * change a particular value used for a timeout or a threshold or a constant
 * in an algorithm. The function will change the value of the parameter
 * identified by an unique ID.
 *
 * @note This function doesn't Access to the device
 *
 * @param   Dev                          Device Handle
 * @param   TuningParameterId            Tuning Parameter ID
 * @param   TuningParameterValue         Tuning Parameter Value
 * @return  VL53LX_ERROR_NONE        Success
 * @return  "Other error code"       See ::VL53LX_Error
 */
VL53LX_Error VL53LX_SetTuningParameter(VL53LX_DEV Dev,
		uint16_t TuningParameterId, int32_t TuningParameterValue);

/**
 * @brief Get Tuning Parameter value for a given parameter ID
 *
 * @par Function Description
 * This function is used to get the value of the parameter
 * identified by an unique ID.
 *
 * @note This function doesn't Access to the device
 *
 * @param   Dev                          Device Handle
 * @param   TuningParameterId            Tuning Parameter ID
 * @param   pTuningParameterValue        Pointer to Tuning Parameter Value
 * for a given TuningParameterId.
 * @return  VL53LX_ERROR_NONE        Success
 * @return  "Other error code"       See ::VL53LX_Error
 */
VL53LX_Error VL53LX_GetTuningParameter(VL53LX_DEV Dev,
		uint16_t TuningParameterId, int32_t *pTuningParameterValue);

/**
 * @brief Performs Reference Spad Management
 *
 * @par Function Description
 * The reference SPAD initialization procedure determines the minimum amount
 * of reference spads to be enables to achieve a target reference signal rate
 * and should be performed once during initialization.
 *
 * @note This function Access to the device
 *
 * @param   Dev                          Device Handle
 * @return  VL53LX_ERROR_NONE        Success
 * @return  "Other error code"       See ::VL53LX_Error
 */
VL53LX_Error VL53LX_PerformRefSpadManagement(VL53LX_DEV Dev);

/**
 * @brief Enable/Disable dynamic Xtalk compensation feature
 *
 * Enable/Disable dynamic Xtalk compensation (aka smudge correction).
 *
 * @param   Dev    Device Handle
 * @param   Mode   Set the smudge correction mode
 * See ::VL53LX_SmudgeCorrectionModes
 * @return  VL53LX_ERROR_NONE        Success
 * @return  "Other error code"       See ::VL53LX_Error
 */
VL53LX_Error VL53LX_SmudgeCorrectionEnable(VL53LX_DEV Dev,
		VL53LX_SmudgeCorrectionModes Mode);


/**
 * @brief Enable/Disable Cross talk compensation feature
 *
 * Enable/Disable Cross Talk correction.
 *
 * @param   Dev                       Device Handle
 * @param   XTalkCompensationEnable   Cross talk compensation
 *  to be set 0 = disabled or 1 = enabled.
 * @return  VL53LX_ERROR_NONE        Success
 * @return  "Other error code"       See ::VL53LX_Error
 */
VL53LX_Error VL53LX_SetXTalkCompensationEnable(VL53LX_DEV Dev,
uint8_t XTalkCompensationEnable);

/**
 * @brief Get Cross talk compensation rate enable
 *
 * Get if the Cross Talk is Enabled or Disabled.
 *
 * @note This function doesn't access to the device
 *
 * @param   Dev                        Device Handle
 * @param   pXTalkCompensationEnable   Pointer to the Cross talk compensation
 *  state 0=disabled or 1 = enabled
 * @return  VL53LX_ERROR_NONE        Success
 * @return  "Other error code"       See ::VL53LX_Error
 */
VL53LX_Error VL53LX_GetXTalkCompensationEnable(VL53LX_DEV Dev,
	uint8_t *pXTalkCompensationEnable);

/**
 * @brief Perform XTalk Calibration
 *
 * @details Perform a XTalk calibration of the Device.
 * This function will launch a  measurement, if interrupts
 * are enabled an interrupt will be done.
 * This function will clear the interrupt generated automatically.
 * This function will program a new value for the XTalk compensation
 * and it will enable the cross talk before exit.
 *
 * @warning This function is a blocking function
 *
 * @note This function Access to the device
 *
 * @param   Dev                  Device Handle
 * the calibration sets appropriate
 * distance mode and thus override existing one<br>
 * The calibration uses a target which should be located at least @60cm from the
 * device. The actual location of the target shall be passed
 * through the bare driver tuning parameters table
 *
 * @return  VL53LX_ERROR_NONE    Success
 * @return  "Other error code"   See ::VL53LX_Error
 */
VL53LX_Error VL53LX_PerformXTalkCalibration(VL53LX_DEV Dev);


/**
 * @brief Define the mode to be used for the offset correction
 *
 * Define the mode to be used for the offset correction.
 *
 * @param   Dev                       Device Handle
 * @param   OffsetCorrectionMode      Offset Correction Mode valid values are:
 * @li                                VL53LX_OFFSETCORRECTIONMODE_STANDARD
 * @li                                VL53LX_OFFSETCORRECTIONMODE_PERVCSEL
 *
 * @return  VL53LX_ERROR_NONE         Success
 * @return  "Other error code"        See ::VL53LX_Error
 */
VL53LX_Error VL53LX_SetOffsetCorrectionMode(VL53LX_DEV Dev,
		VL53LX_OffsetCorrectionModes OffsetCorrectionMode);


/**
 * @brief Perform Offset simple Calibration
 *
 * @details Perform a very simple offset calibration of the Device.
 * This function will launch few ranging measurements and computes offset
 * calibration. The preset mode and the distance mode MUST be set by the
 * application before to call this function.
 *
 * @warning This function is a blocking function
 *
 * @note This function Access to the device
 *
 * @param   Dev                  Device Handle
 * @param   CalDistanceMilliMeter     Calibration distance value used for the
 * offset compensation.
 *
 * @return  VL53LX_ERROR_NONE
 * @return  VL53LX_ERROR_OFFSET_CAL_NO_SAMPLE_FAIL the calibration failed by
 * lack of valid measurements
 * @return  VL53LX_WARNING_OFFSET_CAL_SIGMA_TOO_HIGH means that the target
 * distance combined to the number of loops performed in the calibration lead to
 * an internal overflow. Try to reduce the distance of the target (140 mm)
 * @return  "Other error code"   See ::VL53LX_Error
 */
VL53LX_Error VL53LX_PerformOffsetSimpleCalibration(VL53LX_DEV Dev,
		int32_t CalDistanceMilliMeter);

/**
 * @brief Perform Offset simple Calibration with a "zero distance" target
 *
 * @details Perform a simple offset calibration of the Device.
 * This function will launch few ranging measurements and computes offset
 * calibration. The preset mode and the distance mode MUST be set by the
 * application before to call this function.
 * A target must be place very close to the device.
 * Ideally the target shall be touching the coverglass.
 *
 * @warning This function is a blocking function
 *
 * @note This function Access to the device
 *
 * @param   Dev                  Device Handle
 *
 * @return  VL53LX_ERROR_NONE
 * @return  VL53LX_ERROR_OFFSET_CAL_NO_SAMPLE_FAIL the calibration failed by
 * lack of valid measurements
 * @return  VL53LX_WARNING_OFFSET_CAL_SIGMA_TOO_HIGH means that the target
 * distance is too large, try to put the target closer to the device
 * @return  "Other error code"   See ::VL53LX_Error
 */
VL53LX_Error VL53LX_PerformOffsetZeroDistanceCalibration(VL53LX_DEV Dev);


/**
 * @brief Perform Offset per Vcsel Calibration. i.e. per distance mode
 *
 * @details Perform offset calibration of the Device depending on the
 * three distance mode settings: short, medium and long.
 * This function will launch few ranging measurements and computes offset
 * calibration in each of the three distance modes.
 * The preset mode MUST be set by the application before to call this function.
 *
 * @warning This function is a blocking function
 *
 * @note This function Access to the device
 *
 * @param   Dev                  Device Handle
 * @param   CalDistanceMilliMeter     Distance of the target used for the
 * offset compensation calibration.
 *
 * @return  VL53LX_ERROR_NONE
 * @return  VL53LX_ERROR_OFFSET_CAL_NO_SAMPLE_FAIL the calibration failed by
 * lack of valid measurements
 * @return  VL53LX_WARNING_OFFSET_CAL_SIGMA_TOO_HIGH means that the target
 * distance combined to the number of loops performed in the calibration lead to
 * an internal overflow. Try to reduce the distance of the target (140 mm)
 * @return  "Other error code"   See ::VL53LX_Error
 */
VL53LX_Error VL53LX_PerformOffsetPerVcselCalibration(VL53LX_DEV Dev,
	int32_t CalDistanceMilliMeter);


/**
 * @brief Sets the Calibration Data.
 *
 * @par Function Description
 * This function set all the Calibration Data issued from the functions
 * @a VL53LX_PerformRefSpadManagement(), @a VL53LX_PerformXTalkCalibration,
 * @a VL53LX_PerformOffsetCalibration()
 *
 * @note This function doesn't Accesses the device
 *
 * @param   Dev                          Device Handle
 * @param   *pCalibrationData            Pointer to Calibration data to be set.
 * @return  VL53LX_ERROR_NONE            Success
 * @return  VL53LX_ERROR_INVALID_PARAMS  pCalibrationData points to an older
 * version of the inner structure. Need for support to convert its content.
 * @return  "Other error code"           See ::VL53LX_Error
 */
VL53LX_Error VL53LX_SetCalibrationData(VL53LX_DEV Dev,
		VL53LX_CalibrationData_t *pCalibrationData);

/**
 * @brief Gets the Calibration Data.
 *
 * @par Function Description
 * This function get all the Calibration Data issued from the functions
 * @a VL53LX_PerformRefSpadManagement(), @a VL53LX_PerformXTalkCalibration,
 * @a VL53LX_PerformOffsetCalibration()
 *
 * @note This function doesn't Accesses the device
 *
 * @param   Dev                          Device Handle
 * @param   *pCalibrationData            pointer where to store Calibration
 *  data.
 * @return  VL53LX_ERROR_NONE            Success
 * @return  "Other error code"           See ::VL53LX_Error
 */
VL53LX_Error VL53LX_GetCalibrationData(VL53LX_DEV Dev,
		VL53LX_CalibrationData_t  *pCalibrationData);


/**
 * @brief Gets the optical center.
 *
 * @par Function Description
 * This function get the optical center issued from the nvm set at FTM stage
 * expressed in the same coordinate system as the ROI are
 *
 * @note This function doesn't Accesses the device
 *
 * @param   Dev                          Device Handle
 * @param   pOpticalCenterX              pointer to the X position of center
 * in 16.16 fix point
 * @param   pOpticalCenterY              pointer to the Y position of center
 * in 16.16 fix point
 * @return  VL53L1_ERROR_NONE            Success
 * @return  "Other error code"           See ::VL53LX_Error
 */
VL53LX_Error VL53LX_GetOpticalCenter(VL53LX_DEV Dev,
		FixPoint1616_t *pOpticalCenterX,
		FixPoint1616_t *pOpticalCenterY);


/** @} VL53LX_Calibration_group */

/** @} VL53LX_group */

#ifdef __cplusplus
}
#endif

#endif /* _VL53LX_API_H_ */
