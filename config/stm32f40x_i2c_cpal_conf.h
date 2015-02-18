/**
  ******************************************************************************
  * @file    cpal_conf_template.h 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    21-December-2012
  * @brief   Library configuration file.
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
#ifndef __CPAL_CONF_H
#define __CPAL_CONF_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/*=======================================================================================================================================
                                                     User NOTES
=========================================================================================================================================
          
-------------------------------
1. How To use the CPAL Library:
-------------------------------

------- Refer to the user manual of the library and (eventually) the example to check if 
        this firmware is appropriate for your hardware (device and (eventually) evaluation board).
   
      - Section 1 : Select the Device instances to be used and the total number of devices.
             
      - Section 2 : Configure Transfer Options.
      
      - Section 3 : Select and configure transfer and error user Callbacks.

      - Section 4 : Configure Timeout mechanism and TimeoutCallback.
      
      - Section 5 : NVIC Priority Group Selection and Interrupt Priority Offset.
       
      - Section 6 : Configure CPAL_LOG Macro.

------ After configuring CPAL firmware functionality , You should proceed by configuring hardware used with CPAL
       (please refer to cpal_i2c_hal_device.h file (device = stm32f10x, stm32l1xx, ....) ).

------ After configuring CPAL Firmware Library, you should follow these steps to use the Firmware correctly :

      -1-  STRUCTURE INITIALIZATION 
      Start by initializing the Device. To perform this action, the global variable PPPx_DevStructure declared 
      in CPAL Firmware as CPAL_InitTypeDef (I2C1_DevStructure for I2C1, I2C2_DevStructure for I2C2 ...) must be used .
      There are two ways to proceed :
             
        ** Call the function CPAL_PPP_StructInit() using as parameter PPPx_DevStructure (where PPP = device type (ie. I2C...)
        and where x could be 1 for PPP1, 2 for PPP2 ...). This function sets the default values for all fields of this structure. 
              
        Default values for I2C devices are :
            I2Cx_DevStructure.CPAL_Direction                            = CPAL_DIRECTION_TXRX   
            I2Cx_DevStructure.CPAL_Mode                                 = CPAL_MODE_MASTER       
            I2Cx_DevStructure.CPAL_ProgModel                            = CPAL_PROGMODEL_DMA  
            I2Cx_DevStructure.pCPAL_TransferTx                          = pNULL
            I2Cx_DevStructure.pCPAL_TransferRx                          = pNULL
            I2Cx_DevStructure.CPAL_State                                = CPAL_STATE_DISABLED     
            I2Cx_DevStructure.wCPAL_DevError                            = CPAL_I2C_ERR_NONE       
            I2Cx_DevStructure.wCPAL_Options                             = 0        (all options disabled)
            I2Cx_DevStructure.wCPAL_Timeout                             = CPAL_TIMEOUT_DEFAULT
            I2Cx_DevStructure.pCPAL_I2C_Struct->I2C_ClockSpeed          = 100000
            I2Cx_DevStructure.pCPAL_I2C_Struct->I2C_Mode                = I2C_Mode_I2C
            I2Cx_DevStructure.pCPAL_I2C_Struct->I2C_DutyCycle           = I2C_DutyCycle_2
            I2Cx_DevStructure.pCPAL_I2C_Struct->I2C_OwnAddress1         = 0
            I2Cx_DevStructure.pCPAL_I2C_Struct->I2C_Ack                 = I2C_Ack_Enable
            I2Cx_DevStructure.pCPAL_I2C_Struct->I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit 
  
        
        pCPAL_TransferTx and pCPAL_TransferRx fields have to be updated in order to point to valid structures 
        (these structures should be local/global variables in the user application).
  
        ** Another way of configuration is without calling CPAL_PPP_StructInit() function. 
        Declare the following structures:
          - A PPP_InitTypeDef structure for the device configuration (ie. I2C_InitTypeDef structure)
          - One or two CPAL_TransferTypeDef variables (one for Tx and one for Rx).
          - Use the extern structure provided by the CPAL library: PPPx_InitStructure (ie. I2C1_DevStructure).  
        Fill in all the fields for these structures (one by one).
        Use the pointers to these structures to fill in the fields pCPAL_PPP_Struct and pCPAL_TransferTx and/or 
        pCPAL_TransferRx of the PPPx_DevStructure. 
        After that CPAL_State must be set to CPAL_STATE_DISABLED. 
        Finally, call the CPAL_PPP_Init() with pointer to the PPPx_DevStructure as argument.
  
          Example:
           // Declare local structures 
           I2C_InitTypeDef         I2C1_InitStructure;  
           CPAL_TransferTypeDef    TX_Transfer , RX_Transfer ;
           // Fill in all the fields of to these structures
           I2C1_InitStructure.I2C_ClockSpeed = 5000;
           I2C1_InitStructure.I2C_Mode = I2C_Mode_I2C;
           I2C1_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
           .....
           TX_Transfer.pbBuffer = 0;
           TX_Transfer.wNumData = 0;
           .....
           RX_Transfer.pbBuffer = 0;
           RX_Transfer.wNumData = 0;
           .....
           // Use these structures and fill all fields of I2C1_DevStructure.
           I2C1_DevStructure.CPAL_Dev = CPAL_I2C1;
           I2C1_DevStructure.CPAL_Direction = CPAL_DIRECTION_TXRX;    
           I2C1_DevStructure.CPAL_Mode = CPAL_MODE_SLAVE; 
           I2C1_DevStructure.wCPAL_Options = CPAL_OPT_DMATX_HTIT ;
           .....
           I2C1_DevStructure.pCPAL_TransferTx = &TX_Transfer ;
           I2C1_DevStructure.pCPAL_TransferRx = &RX_Transfer ;
           I2C1_DevStructure.pCPAL_I2C_Struct = &I2C1_InitStructure; 
           ...
           I2C1_DevStructure.wCPAL_State = CPAL_STATE_DISABLED;
           ....
           CPAL_I2C_Init(&I2C1_DevStructure);
  
      -2- DEVICE CONFIGURATION 
      Call the function CPAL_PPP_Init() to configure the selected device with the selected configuration by calling 
      CPAL_PPP_Init(). This function also enables device clock and initialize all related peripherals ( GPIO, DMA , IT and NVIC ).
      This function tests on CPAL_State, if it is equal to CPAL_STATE_BUSY it exit, otherwise device initialization is 
      performed and CPAL_State is set to CPAL_STATE_READY.
      This function returns CPAL_PASS state when the operation is correctly performed, or CPAL_FAIL when the current state of the
      device doesn't allow configuration (ie. state different from READY, DISABLED or ERROR).
      After calling this function, you may check on the new state of device, when it is equal to CPAL_STATE_READY, Transfer operations 
      can be started, otherwise you can call CPAL_PPP_DeInit() to deinitialize device and call CPAL_PPP_Init() once again.
  
      -3- 1- READ / WRITE OPERATIONS 
      Call the function CPAL_PPP_Write() or CPAL_PPP_Read() to perform transfer operations. 
      These functions handle communication events using device event interrupts (independently of programming model used: DMA, 
      Interrupt). These functions start preparing communication (send start condition, send salve address in case of 
      master mode ...) if connection is established between devices CPAL_State is set CPAL_STATE_BUSY_XX and data transfer starts. 
      By default, Error interrupts are enabled to manage device errors (Error interrupts can be disabled by affecting 
      CPAL_OPT_I2C_ERRIT_DISABLE to wCPAL_Options). When transfer is completed successfully, CPAL_State is set to CPAL_STATE_READY 
      and another operation can be started.
      These functions return CPAL_PASS if the current state of the device allows starting a new operation and the operation is correctly
      started (but not finished). It returns CPAL_FAIL when the state of the device doesn't allow starting a new communication (ie. 
      BUSY, DISABLED, ERROR) or when an error occurs during operation start.
      Once operation is started, user application may perform other tasks while CPAL is sending/receiving data on device through interrupt
      or DMA. 

      -3- 2- Listen Mode for Slave
      This mode allows slave device to start a communication without knowing in advance the nature of the operation (read or write).
      Slave enter in idle state and wait until it receive its own address, CPAL_State is set CPAL_STATE_BUSY. In accordance to the type 
      of received request from master device, the slave device state is changed to CPAL_STATE_BUSY_RX and CPAL_I2C_SLAVE_READ_UserCallback 
      is called for a read request or state is changed to CPAL_STATE_BUSY_TX and CPAL_I2C_SLAVE_WRITE_UserCallback for a write request.
      In these callbacks user must configure DMA, interrupts (Prepare DMA channel, DMA request and I2C interrupts) and transfer parameters.
      To configure DMA and Interrupts user can call "CPAL_I2C_Enable_DMA_IT" function which is implemented in Communication layer of CPAL Library. 
      Listen mode is enabled by uncommenting "CPAL_I2C_LISTEN_MODE" define in "cpal_conf.h" file. When this mode is enabled "CPAL_I2C_Read" and 
      "CPAL_I2C_Write" functions are replaced by one function "CPAL_I2C_Listen".
    
      -4- DEVICE DEINITIALIZATION 
       When transfer operations are finished, you may call CPAL_PPP_DeInit() to disable PPPx device and related resources 
      ( GPIO, DMA , IT and NVIC). CPAL_State is then set to CPAL_STATE_DISABLED by this function.



------ Callbacks are routines that let you insert your own code in different stages of communication and for handling
       device errors. Their prototypes are declared by the CPAL library (if the relative define in this cpal_conf.h is enabled)
       but their body is not implemented by CPAL library. It may be done by user when needed.
       There are three types of Callbacks: Transfer User Callbacks, Error User Callbacks and Timeout User Callbacks: 
       
        -a- Transfer User Callbacks : 

          ** CPAL_I2C_TX_UserCallback(CPAL_InitTypeDef* pDevInitStruct) 
               This function is called before sending data when Interrupt Programming Model is selected.

          ** CPAL_I2C_RX_UserCallback(CPAL_InitTypeDef* pDevInitStruct) 
               This function is called after receiving data when Interrupt Programming Model is selected.

          ** CPAL_I2C_TXTC_UserCallback(CPAL_InitTypeDef* pDevInitStruct) 
          ** CPAL_I2C_RXTC_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
               These functions are called when a transfer is complete when using Interrupt programming model or DMA 
               programming model.
                    
          ** CPAL_I2C_DMATXTC_UserCallback(CPAL_InitTypeDef* pDevInitStruct) 
          ** CPAL_I2C_DMATXTC_UserCallback(CPAL_InitTypeDef* pDevInitStruct) 
              These functions are called when Transfer complete Interrupt occurred in transmission/reception operation 
               if DMA Programming Model is selected

          ** CPAL_I2C_DMATXHT_UserCallback(CPAL_InitTypeDef* pDevInitStruct) 
          ** CPAL_I2C_DMARXHT_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
               These functions are called when Half transfer Interrupt occurred in transmission/reception operation 
               if DMA Programming Model is selected.

          ** CPAL_I2C_DMATXTE_UserCallback(CPAL_InitTypeDef* pDevInitStruct) 
          ** CPAL_I2C_DMARXTE_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
               These functions are called when a transfer error Interrupt occurred in transmission/reception operation 
               if DMA Programming Model is selected.

          ** CPAL_I2C_GENCALL_UserCallback(CPAL_InitTypeDef* pDevInitStruct) 
               This function is called when an Address Event interrupt occurred and General Call Address Flag is set 
               (available in Slave mode only and when the option CPAL_OPT_I2C_GENCALL is enabled).

          ** CPAL_I2C_DUALF_UserCallback(CPAL_InitTypeDef* pDevInitStruct) 
               This function is called when an Address Event interrupt occurred and Dual Address Flag is set 
              (available in Slave mode only and when the option CPAL_OPT_I2C_DUALADDR is enabled).

          ** CPAL_I2C_SLAVE_WRITE_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
               This function is called when a write operation is requested in Listen mode only.
                
          ** CPAL_I2C_SLAVE_READ_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
               This function is called when a read operation is requested in Listen mode only.
               
               
        -b- Error User Callbacks : 

          ** CPAL_I2C_ERR_UserCallback(CPAL_DevTypeDef pDevInstance, uint32_t Device_Error) 
               This function is called either when an Error Interrupt occurred (If Error Interrupts enabled) or after
               a read or write operations to handle device errors (If Error Interrupts disabled). This callback 
               can be used to handle all device errors. It is available only when the define USE_SINGLE_ERROR_CALLBACK
               is enabled (Section 5).

          ** CPAL_I2C_BERR_UserCallback(CPAL_DevTypeDef pDevInstance) 
               This function is called either when a Bus Error Interrupt occurred (If Error Interrupts enabled) or 
               after a read or write operations to handle this error (If Error Interrupts disabled). This callback is
               available only when USE_MULTIPLE_ERROR_CALLBACK is enabled (Section 5).

          ** CPAL_I2C_ARLO_UserCallback(CPAL_DevTypeDef pDevInstance) 
               This function is called either when an Arbitration Lost Interrupt occurred (If Error Interrupts 
               enabled) or after a read or write operations to handle this error (If Error Interrupts disabled).

          ** CPAL_I2C_OVR_UserCallback(CPAL_DevTypeDef pDevInstance) 
               This function is called either when an Overrun Interrupt occurred (If Error Interrupts enabled) or 
               after a read or write operations to handle this error (If Error Interrupts disabled). This callback is
               available only when USE_MULTIPLE_ERROR_CALLBACK is enabled (Section 5).

          ** CPAL_I2C_AF_UserCallback(CPAL_DevTypeDef pDevInstance) 
               This function is called either when an Acknowledge Failure Interrupt occurred (If Error Interrupts 
               enabled) or after a read or write operations to handle this error (If Error Interrupts disabled).
                This callback is available only when USE_MULTIPLE_ERROR_CALLBACK is enabled (Section 5).


        -c- Timeout User Callbacks : 
          ** CPAL_TIMEOUT_UserCallback(void)
               This function is called when a Timeout occurred in communication.
      
          ** CPAL_TIMEOUT_INIT()
              This function allows to configure and enable the counting peripheral/function (ie. SysTick Timer)
              It is called into all CPAL_PPP_Init() functions.

          ** CPAL_TIMEOUT_DEINIT()
               This function allow to free the resources of counting peripheral/function and stop the count.
               (ie. disable the SysTick timer and its interrupt).

          ** CPAL_PPP_TIMEOUT_Manager()
               WARNING: DO NOT IMPLEMENT THIS FUNCTION (already implemented in CPAL drivers)
               This function is already implemented in the CPAL drivers (cpal_i2c.c file). It should be called periodically
               (using the count mechanism interrupt for example). This function checks all PPP devices and 
               manages timeout conditions. In case of timeout occurring, this function calls the 
               CPAL_TIMEOUT_UserCallback() function that may be implemented by user to manage the cases of
               timeout errors (ie. reset the device/microcontroller...).
               In order to facilitate implementation, this function (instead to be called periodically by user
               application), may be mapped directly to a periodic event/interrupt: 
               Example: 
               #define CPAL_I2C_TIMEOUT_Manager        SysTick_Handler



     To implement Transfer and Error Callbacks, you should comment relative defines in Section 4 and implement Callback function (body) into
     your application (their prototypes are declared in cpal_i2c.h file). 
      
     Example: How to implement CPAL_I2C_TX_UserCallback() callback:
                        
             -1- Comment the relative define in this file :
                          //#define CPAL_I2C_TX_UserCallback        (void)  
                        
             -2- Add CPAL_I2C_TX_UserCallback code source in application file ( example : main.c )
                          void CPAL_I2C_TX_UserCallback (CPAL_InitTypeDef* pDevInitStruct)
                          {
                            //..........
                            // user code
                            //..........
                          }
                    
     There are two types of Error Callbacks :
             -1- Single Error Callback : Only one Callback is used to manage all device errors.
             -2- Multiple Error Callback : Each device error is managed by its own separate Callback. 
     
     Example of using CPAL_I2C_BERR_UserCallback :
                        
             -1- Select Multiple Error Callback type :
             //#define USE_SINGLE_ERROR_CALLBACK     
             #define USE_MULTIPLE_ERROR_CALLBACK
            
             -2- Comment define relative to CPAL_I2C_BERR_UserCallback in cpal_conf.h file:
             //#define CPAL_I2C_BERR_UserCallback        (void)  
                                   
             -3- Add CPAL_I2C_BERR_UserCallback code source in application file ( example: main.c )
                          void CPAL_I2C_BERR_UserCallback (CPAL_DevTypeDef pDevInstance)
                          {
                            //..........
                            // user code
                            //..........
                          }

------ The driver API functions Prototypes are in cpal_i2c.h file.

*********END OF User Notes***************************************************************************************************************/




/*=======================================================================================================================================
                                       CPAL Firmware Functionality Configuration
=========================================================================================================================================*/

/*-----------------------------------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------*/

/*   -- Section 1 :                   **** I2Cx Device Selection ****

    Description: This section provide an easy way to select I2Cx devices in user application. 
                 Choosing device allows to save memory resources.
                 If you need I2C1 device, uncomment relative define: #define CPAL_USE_I2C1.
                 All available I2Cx device can be used at the same time.
                 At least one I2C device should be selected. */
#define CPAL_USE_I2C1          /*<! Uncomment to use I2C1 device */
#define CPAL_USE_I2C2          /*<! Uncomment to use I2C2 device */
#define CPAL_USE_I2C3          /*<! Uncomment to use I2C3 device */

/*-----------------------------------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------*/

/*  -- Section 2 :                **** Transfer Options Configuration ****

    Description: This section allows user to enable/disable some Transfer Options. The benefits of these 
                 defines is to minimize the size of the source code  */                             

/* Enable the use of Master Mode */
#define CPAL_I2C_MASTER_MODE


/* Enable the use of Slave Mode */
//#define CPAL_I2C_SLAVE_MODE


/* Enable Listen mode for slave device */
//#define CPAL_I2C_LISTEN_MODE


/* Enable the use of DMA Programming Model */
#define CPAL_I2C_DMA_PROGMODEL


/* Enable 1 Byte reception with DMA Programming Model */
/* NOTE : This option must be set if user will use DMA for receiving 1 Byte */
#define CPAL_I2C_DMA_1BYTE_CASE


/* Enable the use of IT Programming Model */
#define CPAL_I2C_IT_PROGMODEL


/* Enable the use of 10Bit Addressing Mode */
//#define CPAL_I2C_10BIT_ADDR_MODE


/* Enable the use of 16Bit Address memory register option 
      !! This define is available only when CPAL_I2C_MASTER_MODE is enabled !!  */
#define CPAL_16BIT_REG_OPTION


/* Select which Closing communication Method is used for master receiver */
/* !! WARNING: These two defines are EXCLUSIVE, only one define should be uncommented !*/

/* Method1 used for closing communication with master receiver: This method is for the case when
   the I2C interrupts have the highest priority in the application */
//#define CPAL_I2C_CLOSECOM_METHOD1

/* Method2 used for closing communication with master receiver: This method is for the case when 
   the I2C interrupts do not have the highest priority in the application */
#define CPAL_I2C_CLOSECOM_METHOD2

/* Critical section CallBack can be used only when Method2 of Closing communication for master receiver is enabled.
   Uncomment this line to use the CPAL critical section callback (it disables then enables all interrupts) */
#define USE_CPAL_CRITICAL_CALLBACK

/*------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------------------------------------*/

/*  -- Section 3 :           **** UserCallbacks Selection and Configuration ****

    Description: This section provides an easy way to enable UserCallbacks and select type of Error UserCallbacks.
                 By default, All UserCallbacks are disabled (UserCallbacks are defined as void functions).
                 To implement a UserCallbacks in your application, comment the relative define and 
                 implement the callback body in your application file.*/
                 

/* Error UserCallbacks Type : Uncomment to select UserCallbacks type. One type must be selected */
/* Note : if Error UserCallbacks are not used the two following defines must be commented 
 
   WARNING: These two defines are EXCLUSIVE, only one define should be uncommented ! 
 */
#define USE_SINGLE_ERROR_CALLBACK   /*<! select single UserCallbacks type */
//#define USE_MULTIPLE_ERROR_CALLBACK /*<! select multiple UserCallbacks type */

/* Error UserCallbacks : To use an Error UserCallback comment the relative define */

/* Single Error Callback */
//#define CPAL_I2C_ERR_UserCallback       (void)

/* Multiple Error Callback */
#define CPAL_I2C_BERR_UserCallback      (void)
#define CPAL_I2C_ARLO_UserCallback      (void)
#define CPAL_I2C_OVR_UserCallback       (void)
#define CPAL_I2C_AF_UserCallback        (void)

/* Transfer UserCallbacks : To use a Transfer callback comment the relative define */
#define CPAL_I2C_TX_UserCallback        (void)    
#define CPAL_I2C_RX_UserCallback        (void)
//#define CPAL_I2C_TXTC_UserCallback      (void)
//#define CPAL_I2C_RXTC_UserCallback      (void)

/* DMA Transfer UserCallbacks : To use a DMA Transfer UserCallbacks comment the relative define */
#define CPAL_I2C_DMATXTC_UserCallback   (void)
#define CPAL_I2C_DMATXHT_UserCallback   (void)
#define CPAL_I2C_DMATXTE_UserCallback   (void) 
#define CPAL_I2C_DMARXTC_UserCallback   (void)
#define CPAL_I2C_DMARXHT_UserCallback   (void)
#define CPAL_I2C_DMARXTE_UserCallback   (void)

/* Address Mode UserCallbacks : To use an Address Mode UserCallbacks comment the relative define */
#define CPAL_I2C_GENCALL_UserCallback   (void)
#define CPAL_I2C_DUALF_UserCallback     (void)

/* CriticalSectionCallback : Call User callback for critical section (should typically disable interrupts) */
#define CPAL_EnterCriticalSection_UserCallback        __disable_irq
#define CPAL_ExitCriticalSection_UserCallback         __enable_irq

/* Listen mode Callback : Used to handle communication in listen mode */
#define CPAL_I2C_SLAVE_READ_UserCallback        (void)    
#define CPAL_I2C_SLAVE_WRITE_UserCallback       (void)

/*------------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------------------------------------------------------*/

/*  -- Section 4 :         **** Configure Timeout method, TimeoutCallback ****

    Description: This section allows you to implement your own Timeout Procedure.
                 By default Timeout procedure is implemented with Systick timer and 
                 CPAL_I2C_TIMEOUT_Manager is defined as SysTick_Handler.
                 */


#define _CPAL_TIMEOUT_INIT()           ;
																			/*SysTick_Config((SystemCoreClock / 1000));\
                                       NVIC_SetPriority (SysTick_IRQn, 0)*/ 
                                       /*<! Configure and enable the systick timer
                                       to generate an interrupt when counter value
                                       reaches 0. In the Systick interrupt handler 
                                       the Timeout Error function is called. Time base is 1 ms */

#define _CPAL_TIMEOUT_DEINIT()          ;
																				//SysTick->CTRL = 0        /*<! Disable the systick timer */ 


#define CPAL_I2C_TIMEOUT_Manager       tickI2C     							     /*<! This callback is used to handle Timeout error.
                                                                     When a timeout occurs CPAL_TIMEOUT_UserCallback
                                                                     is called to handle this error */
#ifndef CPAL_I2C_TIMEOUT_Manager
   void CPAL_I2C_TIMEOUT_Manager(void);
#else   
   void SysTick_Handler(void);  
#endif /* CPAL_I2C_TIMEOUT_Manager */ 
                                                                     

/*#define CPAL_TIMEOUT_UserCallback        (void)      */            /*<! Comment this line and implement the callback body in your 
                                                                      application in order to use the Timeout Callback. 
                                                                      It is strongly advised to implement this callback, since it
                                                                      is the only way to manage timeout errors. */

/* Maximum Timeout values for each communication operation (preferably, Time base should be 1 Millisecond).
   The exact maximum value is the sum of event timeout value and the CPAL_I2C_TIMEOUT_MIN value defined below */
#define CPAL_I2C_TIMEOUT_SB             30             
#define CPAL_I2C_TIMEOUT_ADDR           3
#define CPAL_I2C_TIMEOUT_ADD10          3
#define CPAL_I2C_TIMEOUT_TXE            2
#define CPAL_I2C_TIMEOUT_RXNE           2
#define CPAL_I2C_TIMEOUT_BTF            4
#define CPAL_I2C_TIMEOUT_BUSY           5

/* DO NOT MODIFY THESE VALUES ---------------------------------------------------------*/
#define CPAL_I2C_TIMEOUT_DEFAULT        ((uint32_t)0xFFFFFFFF)
#define CPAL_I2C_TIMEOUT_MIN            ((uint32_t)0x00000001)
#define CPAL_I2C_TIMEOUT_DETECTED       ((uint32_t)0x00000000)

/*-----------------------------------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------*/

/*   -- Section 5 :             **** NVIC Priority Group Selection and Interrupt Priority Offset ****
  
  Description: This section allows user to select NVIC Priority Group and configure Interrupt Priority Offset. 
               To change CPAL_NVIC_PRIOGROUP uncomment wanted Priority Group and comment others. Only one 
               define is possible. 
               By default Priority Offset of I2Cx device (ERR, EVT, DMA) are set to 0 */
                     
  
/*-----------NVIC Group Priority-------------*/
  
/* #define CPAL_NVIC_PRIOGROUP      NVIC_PriorityGroup_0 */ /*!< 0 bits for preemption priority
                                                                       4 bits for subpriority */
  
/* #define CPAL_NVIC_PRIOGROUP      NVIC_PriorityGroup_1 */ /*!< 1 bits for preemption priority
                                                                       3 bits for subpriority */
  
/* #define CPAL_NVIC_PRIOGROUP      NVIC_PriorityGroup_2 */  /*!< 2 bits for preemption priority
                                                                       2 bits for subpriority */
  
/* #define CPAL_NVIC_PRIOGROUP       NVIC_PriorityGroup_3 */ /*!< 3 bits for preemption priority
                                                                       1 bits for subpriority */
  
#define CPAL_NVIC_PRIOGROUP       NVIC_PriorityGroup_4  /*!< 4 bits for preemption priority */
  
/*-----------Interrupt Priority Offset-------------*/

/* This defines can be used to decrease the Level of Interrupt Priority for I2Cx Device (ERR, EVT, DMA_TX, DMA_RX).
   The value of I2Cx_IT_OFFSET_SUBPRIO is added to I2Cx_IT_XXX_SUBPRIO and the value of I2Cx_IT_OFFSET_PREPRIO 
   is added to I2Cx_IT_XXX_PREPRIO (XXX: ERR, EVT, DMATX, DMARX). 
   I2Cx Interrupt Priority are defined in cpal_i2c_hal_stm32f10x.h file in Section 3  */

#define I2C1_IT_OFFSET_SUBPRIO          0      /* I2C1 SUB-PRIORITY Offset */ 
#define I2C1_IT_OFFSET_PREPRIO          7      /* I2C1 PREEMPTION PRIORITY Offset */

#define I2C2_IT_OFFSET_SUBPRIO          0      /* I2C2 SUB-PRIORITY Offset */ 
#define I2C2_IT_OFFSET_PREPRIO          7      /* I2C2 PREEMPTION PRIORITY Offset */

#define I2C3_IT_OFFSET_SUBPRIO          0      /* I2C3 SUB-PRIORITY Offset */ 
#define I2C3_IT_OFFSET_PREPRIO          7      /* I2C3 PREEMPTION PRIORITY Offset */

/*-----------------------------------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------*/

/*  -- Section 6 :                  **** CPAL DEBUG Configuration ****

    Description: This section allow user to enable or disable CPAL Debug option. Enabling this option provide 
                 to user an easy way to debug the application code. This option use CPAL_LOG Macro that integrate 
                 printf function. User can retarget printf function to USART ( use hyperterminal), LCD Screen 
                 on ST Eval Board or development toolchain debugger.
                 In this example, the log is managed through printf function routed to USART peripheral and allowing
                 to display messages on Hyperterminal-like terminals. This is performed through redefining the 
                 function PUTCHAR_PROTOTYPE (depending on the compiler) as follows:
 
                   #ifdef __GNUC__
                // With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
                // set to 'Yes') calls __io_putchar() 
                    #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
                   #else
                    #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
                   #endif 
    
    WARNING      Be aware that enabling this feature may slow down the communication process, increase the code size
                 significantly, and may in some cases cause communication errors (when print/display mechanism is too slow)*/
                 

/* To Enable CPAL_DEBUG Option Uncomment the define below */
//#define CPAL_DEBUG

#ifdef CPAL_DEBUG
#define CPAL_LOG(Str)                   uartPrintf(Str)
#include <stdio.h>                     /* This header file must be included when using CPAL_DEBUG option   */
#else
#define CPAL_LOG(Str)                   ((void)0)
#endif /* CPAL_DEBUG */   


/*-----------------------------------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------*/

/*********END OF CPAL Firmware Functionality Configuration****************************************************************/

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#ifdef __cplusplus
}
#endif

#endif /* __CPAL_CONF_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
