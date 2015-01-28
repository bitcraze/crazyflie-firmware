/**
  ******************************************************************************
  * @file    STM32_CPAL_Driver/inc/cpal.h
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    21-December-2012
  * @brief   This file contains CPAL Structures definition.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CPAL_H
#define __CPAL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f40x_i2c_cpal_conf.h"
 
/* If STM32F10X family is used */
#if defined (STM32F10X_LD) || defined (STM32F10X_LD_VL) || defined (STM32F10X_MD) || defined (STM32F10X_MD_VL)\
 || defined (STM32F10X_HD) || defined (STM32F10X_HD_VL) || defined (STM32F10X_XL) || defined (STM32F10X_CL)
#include "stm32f10x.h"
#endif
   
/* If STM32L1XX family is used */
#if defined (STM32L1XX_MD) || defined (STM32L1XX_HD)
#include "stm32l1xx.h"
#endif 
  
/* If STM32F2XX family is used */   
#ifdef STM32F2XX
#include "stm32f2xx.h"
#endif 
 
/* If STM32F4XX family is used */   
#ifdef STM32F4XX
#include "stm32f4xx.h"
#endif
  
/* Exported types ------------------------------------------------------------*/

/*========= CPAL_Dev_TypeDef =========*/
/* CPAL devices enumeration contains the device instances */
  
typedef enum   
{
  CPAL_I2C1       =   0x00,	    /*!< Use I2C1 device */
 
  CPAL_I2C2       =   0x01,     /*!< Use I2C2 device */
 
  CPAL_I2C3       =   0x02      /*!< Use I2C3 device */

}CPAL_DevTypeDef;


/*========= CPAL_Direction_TypeDef =========*/
/* CPAL transfer directions enumeration is used to configure DMA channels 
   (TX and RX) if CPAL_PROGMODEL_DMA is selected or TX/RX interrupts 
   if CPAL_PROGMODEL_INTERRUPT is selected: 
       - If CPAL_DIRECTION_TX is selected only TX DMA Channel or TX Interrupt is configured
       - If CPAL_DIRECTION_RX is selected only RX DMA Channel or RX Interrupt is configured 
       - If CPAL_DIRECTION_TXRX is selected both TX and Rx DMA Channels or Interrupts are configured.*/  

typedef enum
{ 
  CPAL_DIRECTION_TX        = 0x01,         /*!<Transmitter only direction */

  CPAL_DIRECTION_RX        = 0x02,         /*!<Receiver only direction */

  CPAL_DIRECTION_TXRX      = 0x03,         /*!<Transmitter and Receiver direction */

}CPAL_DirectionTypeDef;


/*========= CPAL_Mode_TypeDef =========*/
/* CPAL device Mode enumeration is used to select in which mode the 
   device will proceed : in master mode or slave mode.
   When an I2C device is used in master mode, it will initiate communication 
   by sending start bit. When it is used in slave mode, it will wait till 
   receiving its own address to start communication.*/

typedef enum
{
  CPAL_MODE_MASTER   =   0x00,    /*!< Use device as master */
  
  CPAL_MODE_SLAVE    =   0x01     /*!< Use device as slave */
    
}CPAL_ModeTypeDef;


/*========= CPAL_ProgModel_TypeDef =========*/
/* CPAL Programming Models enumeration  is used to define the programming 
   model of device communication.
          - CPAL_PROGMODEL_DMA : device is programmed to communicate using DMA.
          - CPAL_PROGMODEL_INTERRUPT : device is programmed to communicate using 
            interruption ( TXE , RXNE ...) handlers .
          - CPAL_PROGMODEL_POLLING : device is programmed to communicate using 
            application polling routines (interrupts and DMA will not be used).*/

typedef enum
{
  CPAL_PROGMODEL_INTERRUPT = 0x01,         /*!<Interrupt transfer programming model */

  CPAL_PROGMODEL_DMA       = 0x02          /*!<DMA transfer programming model */

}CPAL_ProgModelTypeDef;


/*========= CPAL_Transfer_TypeDef =========*/
/* CPAL Transfer structure contains all transfer parameters which are used 
   in every Read or Write operation.*/

typedef struct  
{
  uint8_t*        pbBuffer;        /*!<The address of the buffer from/to which the transfer should start */ 

  __IO uint32_t   wNumData;        /*!<Number of data to be transferred for the current transaction */  
                   
  uint32_t        wAddr1;          /*!<Contains the Target device Address (optional)*/

  uint32_t        wAddr2;          /*!<Contains the Register/Physical Address into the device (optional) */

} CPAL_TransferTypeDef;


/*========= CPAL_State_TypeDef =========*/
/* CPAL global State enumeration contains the current state of CPAL communication. 
   Before starting each operation this state is tested. After each operation 
   CPAL_State is updated with the new value resulting from the relative operation.*/

typedef enum
{
  CPAL_STATE_DISABLED = 0x00,        /*!<The Disabled state indicates that the device 
                                         is not configured. */
    
  CPAL_STATE_READY    = 0x01,        /*!<The Ready state indicates that the device is configured
                                         correctly and is ready for read or write operation and/or 
                                         the last transaction has been successfully completed */  
                                           
  CPAL_STATE_READY_TX = 0x03,        /*!<The Ready_TX state indicates that the device is ready for 
                                         transmission operation */
                                          
  CPAL_STATE_READY_RX = 0x05,        /*!<The Ready_RX state indicates that the device is ready for 
                                         reception operation */
                                         
  CPAL_STATE_BUSY     = 0x02,        /*!<The Busy state indicates that a Write or Read 
                                         operation started */
  
  CPAL_STATE_BUSY_TX  = 0x06,        /*!<The Busy_TX state indicates that a transmission 
                                         operation is on going */
  
  CPAL_STATE_BUSY_RX  = 0x0A,        /*!<The Busy_RX state indicates that a reception 
                                         operation is on going */
  
  CPAL_STATE_ERROR    = 0x10,        /*!<The Error state indicates that the last operation failed. 
                                         To determine which error caused the failure, read the 
                                         device status structure. */
}CPAL_StateTypeDef;


/*========= CPAL_Dev_TypeDef =========*/
/* CPAL Device structure definition */

typedef struct 
{  
  CPAL_DevTypeDef         CPAL_Dev;          /*!<Instance of the device. This parameter can be one 
                                                 of the following values: CPAL_Dev_TypeDef */
  
  CPAL_DirectionTypeDef   CPAL_Direction;    /*!<Specifies the direction for the device transfers. 
                                                 It can be one of the following values: CPAL_Direction_TypeDef */
                                                
  CPAL_ModeTypeDef        CPAL_Mode;         /*!<Specifies the maser/slave mode of device. It can be one of the 
                                                 following values: CPAL_Mode_TypeDef */                                                

  CPAL_ProgModelTypeDef   CPAL_ProgModel;    /*!<Specifies the programming model for the device transfers. 
                                                 It can be one of the following values:  CPAL_ProgModel_Enum */

  CPAL_TransferTypeDef*   pCPAL_TransferTx;  /*!<Pointer on a structure specifying the parameters of the current 
                                                 transmission operations. The structure fields are specified as 
                                                 follows: CPAL_Transfer_TypeDef. Use pNULL value if this direction 
                                                 is not needed. */
 
  CPAL_TransferTypeDef*   pCPAL_TransferRx;  /*!<Pointer on a structure specifying the parameters of the current 
                                                 reception operations. The structure fields are specified as 
                                                 follows: CPAL_Transfer_TypeDef. Use pNULL value if this direction 
                                                 is not needed. */
 
  __IO CPAL_StateTypeDef  CPAL_State;        /*!<Holds the current State of the CPAL driver relative to the device 
                                                 instantiated by CPAL_Dev field. The state parameter can be one of 
                                                 the following values: CPAL_State_Enum */

  __IO uint32_t           wCPAL_DevError;    /*!<Specifies the error code for the current operation.The error codes 
                                                 are specified for each device type as follows: 
                                                 CPAL_I2CError_Enum for I2C devices */

  uint32_t                wCPAL_Options;     /*!<Bit-field value specifying additional options for the configuration 
                                                 of the device: The bit-field value can be any combination of following 
                                                 values: CPAL_Options_Enum. When a value is not selected the relative 
                                                 feature is disabled */    
  
  __IO uint32_t           wCPAL_Timeout;     /*!<This field is with timeout procedure. its used to detect timeout */    

  I2C_InitTypeDef*        pCPAL_I2C_Struct;  /*!<Pointer to a device Initialization structure as described 
                                                 in the standard device library driver. 
                                                 A NULL pointer can be provided for this field and, in this case, 
                                                 the default values will be used for all the device initialization. 
                                                 If only some fields need to be modified for the initialization, 
                                                 one can use the CPAL_PPP_StructInit() function just before setting 
                                                 the needed fields.
                                                   Example:
                                                     CPAL_InitTypeDef   I2C1_DevStructure;
                                                     CPAL_I2C_StructInit(&I2C1_DevStructure);
                                                     I2C1_DevStructure->pI2C_Struct->I2C_Speed = 100000; 
                                                     CPAL_I2C_Init(&I2C1_DevStructure); */  
}CPAL_InitTypeDef;


/*========= Table containing all I2C device structures =========*/
extern CPAL_InitTypeDef* I2C_DevStructures[];


/*========= CPAL_Global_Device_Structures =========*/
/* CPAL Global Device Structures are the Global default structures which 
   are used to handle devices configuration and status.*/  

#ifdef CPAL_USE_I2C1 
extern CPAL_InitTypeDef I2C1_DevStructure;
#endif /* CPAL_USE_I2C1 */

#ifdef CPAL_USE_I2C2 
extern CPAL_InitTypeDef I2C2_DevStructure;
#endif /* CPAL_USE_I2C2 */

#ifdef CPAL_USE_I2C3 
extern CPAL_InitTypeDef I2C3_DevStructure;
#endif /* CPAL_USE_I2C3 */

/* Exported constants --------------------------------------------------------*/
   
/*========= CPAL_Options_TypeDef =========*/
/* CPAL Options defines contains configuration options which can be affected 
   to wCPAL_Options which is a bit-field argument so any combination of these 
   parameters can be selected. If one option is not selected then the relative 
   feature is disabled.
   There are common options and device specific options.*/

#define CPAL_OPT_I2C_DUALADDR           ((uint32_t)0x00000001)  /*!<Use Dual Address Mode (available in Slave Mode only). 
                                                                    To use this option enable it by affecting this define 
                                                                    and own address2 to wCPAL_Options */

#define CPAL_OPT_DMATX_HTIT             ((uint32_t)0x00000200)  /*!<Enable the Transmitter DMA Half Transfer Complete interrupt */

#define CPAL_OPT_DMARX_HTIT             ((uint32_t)0x00001000)  /*!<Enable the Receiver DMA Half Transfer Complete interrupt */

#define CPAL_OPT_DMATX_CIRCULAR         ((uint32_t)0x00004000)  /*!<Enable the Circular Mode for DMA Transmitter */
  
#define CPAL_OPT_DMARX_CIRCULAR         ((uint32_t)0x00008000)  /*!<Enable the Circular Mode for DMA Receiver */
  
#define CPAL_OPT_NO_MEM_ADDR            ((uint32_t)0x00010000)  /*!<Enable No Memory addressing mode: only slave device address sent 
                                                                    No Register/Physical address to be sent after slave address */  
  
#define CPAL_OPT_16BIT_REG              ((uint32_t)0x00020000)  /*!<Enable 16-Bit Register/Physical addressing mode (two bytes, 
                                                                   MSB first). This option is supported only when CPAL_OPT_NO_MEM_ADDR 
                                                                   option is not set */  

#define CPAL_OPT_I2C_GENCALL            ((uint32_t)0x00100000)  /*!<Use General Call Address Mode (available in Slave Mode only) 
                                                                    (General Call Address = 0x00) */ 
  
#define CPAL_DMA_1BYTE_CASE             ((uint32_t)0x00200000)  /*!<This define is used internally in the library
                                                                    (not by user) and handle 1Byte transfer by IT 
                                                                    when DMA Programming Model is selected for reception */
  
#define CPAL_OPT_I2C_ERRIT_DISABLE      ((uint32_t)0x00400000)  /*!<Disable I2C Errors interrupt (Bus Error, Arbitration Loss, 
                                                                    Acknowledge Failure and Overrun/Underrun Errors).
                                                                    By default, errors interrupt is enabled to manage errors efficiently */
  
#define CPAL_OPT_I2C_NOSTOP             ((uint32_t)0x00800000)  /*!<Use communication mode with no STOP generation at the end 
                                                                    of data transfer (for multi-read/write operations) */

#define CPAL_OPT_I2C_NOSTOP_MODE        ((uint32_t)0x01000000)  /*!<Start communication in No STOP generation mode */

#define CPAL_OPT_I2C_NACK_ADD           ((uint32_t)0x40000000)  /*!<Initialize the I2C Slave device without enabling the acknowledgement of its 
                                                                    own address. This option must not be used with No Stop generation mode */


/*========= CPAL_Interne_Defines =========*/

#define CPAL_PASS             ((uint32_t)0x00000000) /*!<This value is returned if the last operation succeed */
    
#define CPAL_FAIL             ((uint32_t)0x00000001) /*!<This value is returned if the last operation failed */
  
#define pNULL                 (void*)0  /*!<This Value is used to initialise a null pointer */ 


/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/*<! Initialize NVIC Priority Group */ 
void CPAL_HAL_NVICInit(void); 

/*========= CPAL_TIMEOUT_Callback =========*/

#ifndef CPAL_TIMEOUT_UserCallback 
 uint32_t CPAL_TIMEOUT_UserCallback(CPAL_InitTypeDef* pDevInitStruct);   /*<!This function is called when a Timeout 
                                                                             occurs during communication with devices */ 
#endif



#ifdef __cplusplus
}
#endif

#endif /*__CPAL_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
