/**
  ******************************************************************************
  * @file    STM32_CPAL_Driver/inc/cpal_i2c.h
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    21-December-2012
  * @brief   This file contains all the functions prototypes for the I2C firmware 
  *          layer.
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

/* Includes ------------------------------------------------------------------*/
/* If STM32F10X family is used */
#if defined (STM32F10X_LD) || defined (STM32F10X_LD_VL) || defined (STM32F10X_MD) || defined (STM32F10X_MD_VL)\
 || defined (STM32F10X_HD) || defined (STM32F10X_HD_VL) || defined (STM32F10X_XL) || defined (STM32F10X_CL)
#include "cpal_i2c_hal_stm32f10x.h"
#endif
   
/* If STM32L1XX family is used */
#if defined (STM32L1XX_MD) || defined (STM32L1XX_HD)
#include "cpal_i2c_hal_stm32l1xx.h"
#endif

/* If STM32F2XX family is used */   
#ifdef STM32F2XX
#include "cpal_i2c_hal_stm32f2xx.h"
#endif

/* If STM32F4XX family is used */   
#ifdef STM32F4XX
#include "cpal_i2c_hal_stm32f4xx.h"
#endif

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CPAL_I2C_H
#define __CPAL_I2C_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
   
/*========= CPAL_I2CError_TypeDef =========*/ 
/* I2C Errors TypeDef */
   
typedef enum
{
  CPAL_I2C_ERR_NONE      = 0x0000, /*!<No Error: This is the default state for an Idle peripheral */

  CPAL_I2C_ERR_TIMEOUT   = 0x00FF, /*!<Timeout error: The specified timeout has been elapsed without 
                                         any response (expected flag or data didn't happen at expected time). */

  CPAL_I2C_ERR_BERR      = 0x0100, /*!<Bus error: This error occurs when I2C peripheral detects an external
                                       Stop or Start condition during address or data transfer. In this case:
                                          - The BERR bit is set and an interrupt is generated if the ITERREN bit is set.
                                       In Slave mode: 
                                         data are discarded and the lines are released by hardware:
                                          - In case of a misplaced Start, the slave considers it is a restart and 
                                            waits for an address, or a Stop condition.
                                          - In case of a misplaced Stop, the slave behaves like for a Stop condition and 
                                           the lines are released by hardware.
                                       In Master mode: 
                                         the lines are not released and the state of the current transmission is not 
                                         affected. It is up to the software to abort or not the current transmission.
                                       
                                       Software Clearing sequence for the BERR bit:      
                                         1. Writing '0' to this bit  */
                                            
                                                      
  CPAL_I2C_ERR_ARLO        = 0x0200, /*!<Arbitration Lost error: This error occurs when the I2C interface detects 
                                         an arbitration lost condition. In this case:
                                          - The ARLO bit is set by hardware (and an interrupt is generated if the 
                                            ITERREN bit is set).
                                         the I2C Interface goes automatically back to slave mode (the M/SL bit 
                                         is cleared). 
                                         When the I2C loses the arbitration, it is not able to acknowledge its slave
                                         address in the same transfer, but it can acknowledge it after a repeated 
                                         Start from the winning master.
                                         Lines are released by hardware.
                                              
                                         Software Clearing sequence for the BERR bit:      
                                          1. Writing '0' to this bit  */
                                                  
  CPAL_I2C_ERR_AF          = 0x0400, /*!<Acknowledge Failure : This error occurs when the interface detects 
                                         a non-acknowledge bit. In this case:
                                          - The AF bit is set and an interrupt is generated if the ITERREN bit 
                                            is set.
                                         A transmitter which receives a NACK must reset the communication:
                                          - If Slave: lines are released by hardware.
                                          - If Master: a Stop or repeated Start condition must be generated 
                                            by software.
                                                 
                                         Software Clearing sequence for the ARLO bit:
                                         1. Writing '0' to this bit */                                        
                                                      
  CPAL_I2C_ERR_OVR          = 0x0800, /*!<Overrun/Underrun error: An overrun error can occur in slave mode when clock 
                                          stretching is disabled and the I2C interface is receiving data. The interface has
                                          received a byte (RxNE=1) and the data in DR has not been read, before the next 
                                          byte is received by the interface. 
                                          In this case:
                                          The last received byte is lost.
                                           - In case of Overrun error, software should clear the RxNE bit and the transmitter 
                                             should retransmit the last received byte.
                                          
                                          Underrun error can occur in slave mode when clock stretching is disabled and the 
                                          I2C interface is transmitting data. The interface has not updated the DR with the 
                                          next byte(TxE=1), before the clock comes for the next byte. In this case:
                                           - The same byte in the DR register will be sent again.
                                           - The user should make sure that data received on the receiver side during an 
                                             underrun error are discarded and that the next bytes are written within the 
                                             clock low time specified in the I2C bus standard.
                                          For the first byte to be transmitted, the DR must be written after ADDR is 
                                          cleared and before the first SCL rising edge. If not possible, the receiver 
                                          must discard the first data.
                                      
                                       Software Clearing sequence for the ARLO bit:
                                        1. Writing '0' to this bit */
                                                  
 }CPAL_I2CErrorTypeDef;
 
 
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/


/*========= CPAL_I2C_Exported_Functions =========*/
/* These functions constitute the main CPAL API interface. All functions take as argument the 
   CPAL_InitTypeDef structure defined in @ref CPAL_Dev_TypeDef. */

uint32_t  CPAL_I2C_Init         (CPAL_InitTypeDef* pDevInitStruct); /*<!This function Initializes the selected I2C device 
                                                                        and all needed resources (GPIOs, clocks, DMA, 
                                                                        interrupts …) */
                                                       
uint32_t  CPAL_I2C_DeInit       (CPAL_InitTypeDef* pDevInitStruct); /*<!This function free the resources used by the I2C 
                                                                        device (GPIOs, clocks, DMA, interrupts …) and 
                                                                        deinitialize the device itself */
                                                       
uint32_t  CPAL_I2C_StructInit   (CPAL_InitTypeDef* pDevInitStruct); /*<!This function Initializes I2C device structure 
                                                                        by filling all fields with their default values.
                                                                        Warning: Pointer fields are filled with CPAL local variables
                                                                        pointer. To avoid all risks, it is recommended to declare
                                                                        application local variables and fill these fields with their
                                                                        pointers. */


#if defined (CPAL_I2C_MASTER_MODE) || ! defined (CPAL_I2C_LISTEN_MODE) 
uint32_t  CPAL_I2C_Write        (CPAL_InitTypeDef* pDevInitStruct); /*<!This function Writes data to the specified I2C device.
                                                                        All information relative to the write transfer parameters and
                                                                        current status are extracted from pCPAL_TransferTx field defined
                                                                        in @ref CPAL_Transfer_TypeDef */ 

uint32_t  CPAL_I2C_Read         (CPAL_InitTypeDef* pDevInitStruct); /*<!This function Read data from the specified I2C device 
                                                                        All information relative to the read transfer parameters and
                                                                        current status are extracted from pCPAL_TransferTx field defined
                                                                        in @ref CPAL_Transfer_TypeDef */ 

#endif /* CPAL_I2C_MASTER_MODE || ! CPAL_I2C_LISTEN_MODE */ 


#if defined (CPAL_I2C_LISTEN_MODE) && defined (CPAL_I2C_SLAVE_MODE)
uint32_t  CPAL_I2C_Listen       (CPAL_InitTypeDef* pDevInitStruct); /*<!This function allows the specified I2C device to enter listen mode 
                                                                        All information relative to the read or write transfer parameters and
                                                                        current status are extracted from fields defined in @ref CPAL_Transfer_TypeDef */
#endif /* CPAL_I2C_LISTEN_MODE && CPAL_I2C_SLAVE_MODE */

uint32_t  CPAL_I2C_IsDeviceReady(CPAL_InitTypeDef* pDevInitStruct); /*<!This function can be used to wait until target device is ready 
                                                                        for communication (ie. for memories after write operations) */


uint32_t CPAL_I2C_EV_IRQHandler(CPAL_InitTypeDef* pDevInitStruct); /*<!This function manages all I2C device events */ 

uint32_t CPAL_I2C_ER_IRQHandler(CPAL_InitTypeDef* pDevInitStruct); /*<!This function manages all I2C device errors  */ 

#ifdef CPAL_I2C_DMA_PROGMODEL
uint32_t CPAL_I2C_DMA_TX_IRQHandler(CPAL_InitTypeDef* pDevInitStruct); /*<!This function Handles DMA TX Interrupts */

uint32_t CPAL_I2C_DMA_RX_IRQHandler(CPAL_InitTypeDef* pDevInitStruct); /*<!This function Handles DMA RX Interrupts */
#endif /* CPAL_I2C_DMA_PROGMODEL */


/*========= Local DMA and IT Manager =========*/

uint32_t CPAL_I2C_Enable_DMA_IT (CPAL_InitTypeDef* pDevInitStruct, CPAL_DirectionTypeDef Direction); /* This function Configure I2C DMA 
                                                                                                    and Interrupts before starting 
                                                                                                    transfer phase */

/*========= CPAL_I2C_User_Callbacks =========*/
/* These functions prototypes only are declared here. User can (optionally) 
   implement the function body in his own application depending on the application needs.
   Each callback is called in a particular situation detailed in the callback description. */

#ifndef CPAL_I2C_TX_UserCallback
void CPAL_I2C_TX_UserCallback   (CPAL_InitTypeDef* pDevInitStruct); /*<!This function is called when (in Interrupt mode) 
                                                                        the peripheral is preparing to send data */ 
#endif

#ifndef CPAL_I2C_RX_UserCallback
void CPAL_I2C_RX_UserCallback   (CPAL_InitTypeDef* pDevInitStruct); /*<!This function is called when (in Interrupt mode) 
                                                                        the peripheral has received data */ 
#endif

#ifndef CPAL_I2C_TXTC_UserCallback
void CPAL_I2C_TXTC_UserCallback(CPAL_InitTypeDef* pDevInitStruct); /*<!This function is called when (in DMA or Interrupt mode)
                                                                      TX Transfer is complete (to use in DMA mode, Transfer complete 
                                                                      interrupt must be enabled) */ 
#endif

#ifndef CPAL_I2C_RXTC_UserCallback
void CPAL_I2C_RXTC_UserCallback(CPAL_InitTypeDef* pDevInitStruct); /*<!This function is called when (in DMA or Interrupt mode)
                                                                       RX Transfer is complete (to use in DMA mode, Transfer complete 
                                                                       interrupt must be enabled) */ 
#endif

#ifndef CPAL_I2C_DMATXTC_UserCallback
void CPAL_I2C_DMATXTC_UserCallback(CPAL_InitTypeDef* pDevInitStruct); /*<!This function is called (in DMA mode) when 
                                                                          DMA Transmission is finished (If Transfer Complete 
                                                                          interrupt is enabled) */
#endif

#ifndef CPAL_I2C_DMATXHT_UserCallback
void CPAL_I2C_DMATXHT_UserCallback(CPAL_InitTypeDef* pDevInitStruct); /*<!This function is called (in DMA mode) when the 
                                                                          DMA Transmission has reached the half of the 
                                                                          buffer (If Half Transfer interrupt is enabled) */
#endif

#ifndef CPAL_I2C_DMATXTE_UserCallback
void CPAL_I2C_DMATXTE_UserCallback(CPAL_InitTypeDef* pDevInitStruct); /*<!This function is called when (in DMA mode) a 
                                                                          DMA Transmission transfer error has occurred 
                                                                          (If Transfer Error interrupt is enabled ) */
#endif

#ifndef CPAL_I2C_DMARXTC_UserCallback
void CPAL_I2C_DMARXTC_UserCallback(CPAL_InitTypeDef* pDevInitStruct); /*<!This function is called when (in DMA mode) when 
                                                                          DMA Reception is finished (If Transfer Complete 
                                                                          interrupt is enabled) */
#endif

#ifndef CPAL_I2C_DMARXHT_UserCallback
void CPAL_I2C_DMARXHT_UserCallback(CPAL_InitTypeDef* pDevInitStruct); /*<!This function is called when (in DMA mode) the
                                                                          DMA Reception has reached the half of the 
                                                                          buffer (If Half Transfer interrupt is enabled) */
#endif

#ifndef CPAL_I2C_DMARXTE_UserCallback
void CPAL_I2C_DMARXTE_UserCallback(CPAL_InitTypeDef* pDevInitStruct); /*<!This function is called when (in DMA mode) a 
                                                                          DMA Reception transfer error has occurred 
                                                                          (If Transfer Error interrupt is enabled ) */
#endif

#ifndef CPAL_I2C_GENCALL_UserCallback
void CPAL_I2C_GENCALL_UserCallback(CPAL_InitTypeDef* pDevInitStruct); /*<!This function is called when General Call flag
                                                                          is set (used in General Call Mode only ) */
#endif

#ifndef CPAL_I2C_DUALF_UserCallback
void CPAL_I2C_DUALF_UserCallback(CPAL_InitTypeDef* pDevInitStruct); /*<!This function is called when Dual Address flag
                                                                        is set (used in Dual Address Mode only ) */
#endif

#ifndef CPAL_I2C_SLAVE_READ_UserCallback
void CPAL_I2C_SLAVE_READ_UserCallback(CPAL_InitTypeDef* pDevInitStruct); /*<!This function is called when a read operation is
                                                                             requested in Listen mode only */
#endif

#ifndef CPAL_I2C_SLAVE_WRITE_UserCallback
void CPAL_I2C_SLAVE_WRITE_UserCallback(CPAL_InitTypeDef* pDevInitStruct); /*<!This function is called when a write operation is
                                                                              requested in Listen mode only */
#endif

/*========= CPAL_User_ErrorCallback_Prototypes =========*/
/* User can use two types of Callback:
    - Single Error Callback : All error are handled by one Callback (CPAL_I2C_ERR_UserCallback()).
    - Multiple Error Callback : Each error has its own Callback ( CPAL_I2C_ERRTYPE_UserCallback () , 
        ERRTYPE : can be one of I2C Errors (BERR, ARLO, OVR and AF)).
   To select one of this type, user should comment or uncomment adequate defines in cpal_conf.h file. */  

#ifdef USE_SINGLE_ERROR_CALLBACK

  #ifndef CPAL_I2C_ERR_UserCallback
  void  CPAL_I2C_ERR_UserCallback(CPAL_DevTypeDef pDevInstance, uint32_t DeviceError); /*<!This callback is called when an error 
                                                                                           occurred on the peripheral while transferring
                                                                                           (If I2C Error interrupt is enabled). Device 
                                                                                           instance and error type (DeviceError) are 
                                                                                           passed as argument. Device_Error value can be
                                                                                           one of CPAL_I2CErrorTypeDef enumeration */
  #endif
#endif /* USE_SINGLE_ERROR_CALLBACK */
  
#ifdef USE_MULTIPLE_ERROR_CALLBACK

  #ifndef CPAL_I2C_BERR_UserCallback
   void  CPAL_I2C_BERR_UserCallback(CPAL_DevTypeDef pDevInstance); /*<!This callback is called when an Bus ERROR
                                                                       occurred on the peripheral while transferring
                                                                       (If I2C Error interrupt is enabled) */
  #endif  
 
  #ifndef CPAL_I2C_ARLO_UserCallback 
   void  CPAL_I2C_ARLO_UserCallback(CPAL_DevTypeDef pDevInstance); /*<!This callback is called when an Arbitration Lost 
                                                                       ERROR occurred on the peripheral while transferring 
                                                                       (If I2C Error interrupt is enabled) */
  #endif    
 
  #ifndef CPAL_I2C_OVR_UserCallback 
   void  CPAL_I2C_OVR_UserCallback(CPAL_DevTypeDef pDevInstance); /*<!This callback is called when an Overrun/Underrun 
                                                                      ERROR occurred on the peripheral while transferring 
                                                                      (If I2C Error interrupt is enabled) */
  #endif   
 
  #ifndef CPAL_I2C_AF_UserCallback
   void  CPAL_I2C_AF_UserCallback(CPAL_DevTypeDef pDevInstance); /*<!This callback is called when an Acknowledge 
                                                                     Failure occurred on the peripheral while transferring 
                                                                     (If I2C Error interrupt is enabled) */
  #endif 
 
#endif /* USE_SINGLE_ERROR_CALLBACK */
   

#ifdef __cplusplus
}
#endif

#endif /*__CPAL_I2C_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
