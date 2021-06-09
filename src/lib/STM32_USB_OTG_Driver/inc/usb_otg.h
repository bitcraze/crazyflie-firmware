/**
  ******************************************************************************
  * @file    usb_otg.h
  * @author  MCD Application Team
  * @version V2.2.1
  * @date    17-March-2018
  * @brief   OTG Core Header
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                      <http://www.st.com/SLA0044>
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_OTG__
#define __USB_OTG__


/** @addtogroup USB_OTG_DRIVER
  * @{
  */
  
/** @defgroup USB_OTG
  * @brief This file is the 
  * @{
  */ 


/** @defgroup USB_OTG_Exported_Defines
  * @{
  */ 


void USB_OTG_InitiateSRP(void);
void USB_OTG_InitiateHNP(uint8_t state , uint8_t mode);
void USB_OTG_Switchback (USB_OTG_CORE_DEVICE *pdev);
uint32_t  USB_OTG_GetCurrentState (USB_OTG_CORE_DEVICE *pdev);

/**
  * @}
  */ 


/** @defgroup USB_OTG_Exported_Types
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup USB_OTG_Exported_Macros
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup USB_OTG_Exported_Variables
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup USB_OTG_Exported_FunctionsPrototype
  * @{
  */ 
/**
  * @}
  */ 


#endif //__USB_OTG__


/**
  * @}
  */ 

/**
  * @}
  */ 
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

