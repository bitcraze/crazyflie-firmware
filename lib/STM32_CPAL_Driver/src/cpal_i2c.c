/**
  ******************************************************************************
  * @file    STM32_CPAL_Driver/src/cpal_i2c.c 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    21-December-2012
  * @brief   This file provides all the CPAL firmware functions for I2C peripheral.
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
#include "cpal_i2c.h"

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/


/* This macro allows to test on a flag status and to start Timeout procedure if the 
   waiting time exceeds the allowed timeout period. 
   @note This macro has not been implemented as a function because the entered parameter 
   'cmd' itself can be a macro (if it was implemented as a function, the check on the 
   flag would be done only once, while the required behavior is to check the flag
   continuously). */

#define __CPAL_I2C_TIMEOUT_DETECT                ((pDevInitStruct->wCPAL_Timeout == CPAL_I2C_TIMEOUT_MIN) ||\
                                                 (pDevInitStruct->wCPAL_Timeout == CPAL_I2C_TIMEOUT_DEFAULT))

#define __CPAL_I2C_TIMEOUT(cmd, timeout)         pDevInitStruct->wCPAL_Timeout = CPAL_I2C_TIMEOUT_MIN + (timeout);\
                                                 while (((cmd) == 0) && (!__CPAL_I2C_TIMEOUT_DETECT));\
                                                 if (__CPAL_I2C_TIMEOUT_DETECT)\
                                                 {\
                                                   return CPAL_I2C_Timeout (pDevInitStruct); \
                                                 }\
                                                 pDevInitStruct->wCPAL_Timeout = CPAL_I2C_TIMEOUT_DEFAULT

#define __CPAL_I2C_TIMEOUT_SPINLOOP(cmd, timeout){\
                                                   volatile uint32_t cntdown = 5000 * timeout;\
                                                   while (((cmd) == 0) && (cntdown--));\
                                                   if (cntdown == 0)\
                                                   {\
                                                     return CPAL_I2C_Timeout (pDevInitStruct);\
                                                   }\
                                                 }\

/* Private variables ---------------------------------------------------------*/

/*========= Table Exported from HAL =========*/

extern I2C_TypeDef* CPAL_I2C_DEVICE[];

#ifdef CPAL_I2C_DMA_PROGMODEL
extern DMA_TypeDef* CPAL_I2C_DMA[];

#if defined (STM32F2XX) || defined (STM32F4XX)
extern DMA_Stream_TypeDef* CPAL_I2C_DMA_TX_Stream[]; 
extern DMA_Stream_TypeDef* CPAL_I2C_DMA_RX_Stream[];
#else
extern DMA_Channel_TypeDef* CPAL_I2C_DMA_TX_Channel[]; 
extern DMA_Channel_TypeDef* CPAL_I2C_DMA_RX_Channel[];
#endif

extern const uint32_t CPAL_I2C_DMA_TX_TC_FLAG[];
extern const uint32_t CPAL_I2C_DMA_RX_TC_FLAG[];

extern const uint32_t CPAL_I2C_DMA_TX_HT_FLAG[];
extern const uint32_t CPAL_I2C_DMA_RX_HT_FLAG[];

extern const uint32_t CPAL_I2C_DMA_TX_TE_FLAG[];
extern const uint32_t CPAL_I2C_DMA_RX_TE_FLAG[];
#endif /* CPAL_I2C_DMA_PROGMODEL */

/*========= Local structures used in CPAL_I2C_StructInit() function =========*/ 

I2C_InitTypeDef I2C_InitStructure;

/* Private function prototypes -----------------------------------------------*/

/*========= Local Master events handlers =========*/

#ifdef CPAL_I2C_MASTER_MODE 
  static uint32_t I2C_MASTER_START_Handle(CPAL_InitTypeDef* pDevInitStruct); /* Handle Master SB Interrupt event */  
  static uint32_t I2C_MASTER_ADDR_Handle(CPAL_InitTypeDef* pDevInitStruct);  /* Handle Master ADDR Interrupt event */
 #ifdef CPAL_I2C_10BIT_ADDR_MODE
  static uint32_t I2C_MASTER_ADD10_Handle(CPAL_InitTypeDef* pDevInitStruct); /* Handle Master ADD10 Interrupt event */
 #endif /* CPAL_I2C_10BIT_ADDR_MODE */
 #ifdef CPAL_I2C_IT_PROGMODEL
  static uint32_t I2C_MASTER_TXE_Handle(CPAL_InitTypeDef* pDevInitStruct);   /* Handle Master TXE Interrupt event */
 #endif /* CPAL_I2C_IT_PROGMODEL */
 #if defined (CPAL_I2C_IT_PROGMODEL) || defined (CPAL_I2C_DMA_1BYTE_CASE)
  static uint32_t I2C_MASTER_RXNE_Handle(CPAL_InitTypeDef* pDevInitStruct);  /* Handle Master RXNE Interrupt event */
 #endif /* CPAL_I2C_IT_PROGMODEL || CPAL_I2C_DMA_1BYTE_CASE */
#endif /* CPAL_I2C_MASTER_MODE */ 
 
/*========= Local Slave events handlers =========*/ 
 
#ifdef CPAL_I2C_SLAVE_MODE
  static uint32_t I2C_SLAVE_ADDR_Handle(CPAL_InitTypeDef* pDevInitStruct);   /* Handle Slave ADDR Interrupt event */  
  static uint32_t I2C_SLAVE_STOP_Handle(CPAL_InitTypeDef* pDevInitStruct);   /* Handle Slave STOPF Interrupt event */
 #ifdef CPAL_I2C_IT_PROGMODEL
  static uint32_t I2C_SLAVE_TXE_Handle(CPAL_InitTypeDef* pDevInitStruct);    /* Handle Slave TXE Interrupt event */
  static uint32_t I2C_SLAVE_RXNE_Handle(CPAL_InitTypeDef* pDevInitStruct);   /* Handle Slave RXNE Interrupt event */
 #endif /* CPAL_I2C_IT_PROGMODEL */
#endif /* CPAL_I2C_SLAVE_MODE */


/*========= CPAL Timeout handler =========*/
static uint32_t CPAL_I2C_Timeout (CPAL_InitTypeDef* pDevInitStruct);


/* Private functions ---------------------------------------------------------*/

/*================== USER CPAL Functions ==================*/


/**
  * @brief  Initialize the peripheral and all related clocks, GPIOs, DMA and 
  *         Interrupts according to the specified parameters in the 
  *         CPAL_InitTypeDef structure.
  * @param  pDevInitStruct : Pointer to the peripheral configuration structure.
  * @retval CPAL_PASS or CPAL_FAIL 
  */
uint32_t CPAL_I2C_Init(CPAL_InitTypeDef* pDevInitStruct) 
{
  CPAL_LOG("\n\r\n\rLOG <CPAL_I2C_Init> : I2C Device Init");
  
  /* If CPAL_State is not BUSY */
  if ((pDevInitStruct->CPAL_State == CPAL_STATE_READY) 
     || (pDevInitStruct->CPAL_State == CPAL_STATE_ERROR) 
     || (pDevInitStruct->CPAL_State == CPAL_STATE_DISABLED))
  {
    /* 
    - If CPAL_State is CPAL_STATE_ERROR (an Error occurred in transaction):
      Perform the initialization routines (device will be deinitialized during initialization).
    - If CPAL_State is CPAL_STATE_READY:  
      Perform the initialization routines 
    - If CPAL_State is CPAL_STATE_DISABLED:  
      Perform the Initialization routines                                   */    
    
#ifndef CPAL_I2C_DMA_PROGMODEL
    if (pDevInitStruct->CPAL_ProgModel == CPAL_PROGMODEL_DMA) 
    {
      /* update State */
      pDevInitStruct->CPAL_State = CPAL_STATE_ERROR;
      
      /* Exit Init function */
      return CPAL_FAIL;
    }
#endif /* CPAL_I2C_DMA_PROGMODEL */
    
#ifndef CPAL_I2C_IT_PROGMODEL
    if (pDevInitStruct->CPAL_ProgModel == CPAL_PROGMODEL_INTERRUPT) 
    {
      /* update State */
      pDevInitStruct->CPAL_State = CPAL_STATE_ERROR;
      
      /* Exit Init function */
      return CPAL_FAIL;
    }
#endif /* CPAL_I2C_IT_PROGMODEL */ 
        
    /* Disable I2Cx Device */
    __CPAL_I2C_HAL_DISABLE_DEV(pDevInitStruct->CPAL_Dev);
    
    CPAL_LOG("\n\rLOG : I2C Device Disabled"); 
    
    /* Deinitialize I2Cx GPIO */
    CPAL_I2C_HAL_GPIODeInit(pDevInitStruct->CPAL_Dev);
    
    CPAL_LOG("\n\rLOG : I2C Device IOs Deinit");
    
    /* Deinitialize I2Cx Clock */
    CPAL_I2C_HAL_CLKDeInit(pDevInitStruct->CPAL_Dev);
    
    CPAL_LOG("\n\rLOG : I2C Device Clock Deinit");
    
#ifdef CPAL_I2C_DMA_PROGMODEL    
    /* Deinitialize DMA Channels */
    if (pDevInitStruct->CPAL_ProgModel == CPAL_PROGMODEL_DMA)
    {
      CPAL_I2C_HAL_DMADeInit(pDevInitStruct->CPAL_Dev, pDevInitStruct->CPAL_Direction);
      
      CPAL_LOG("\n\rLOG : I2C Device DMA Deinit");  
    } 
#endif /* CPAL_I2C_DMA_PROGMODEL */
    
    /*----------------------------------------------------------------------------
    Peripheral Clock Initialization
    ---------------------------------------------------------------------------*/   
    /* Initialize I2Cx Clock */
    CPAL_I2C_HAL_CLKInit(pDevInitStruct->CPAL_Dev);
    
    CPAL_LOG("\n\rLOG : I2C Device Clock Init"); 
    
	/*----------------------------------------------------------------------------
    GPIO pins configuration
    ---------------------------------------------------------------------------*/
    /* Initialize I2Cx GPIO */
    CPAL_I2C_HAL_GPIOInit(pDevInitStruct->CPAL_Dev);
    
    CPAL_LOG("\n\rLOG : I2C Device IOs Init");     
	       
    /*----------------------------------------------------------------------------
    Peripheral Initialization
    ---------------------------------------------------------------------------*/   
    /* Enable I2Cx Device */
    __CPAL_I2C_HAL_ENABLE_DEV(pDevInitStruct->CPAL_Dev); 
    
    CPAL_LOG("\n\rLOG : I2C Device Enabled"); 
    
    /* Initialize I2Cx device with parameters stored in pCPAL_I2C_Struct */
    I2C_Init(CPAL_I2C_DEVICE[pDevInitStruct->CPAL_Dev], pDevInitStruct->pCPAL_I2C_Struct);
    
    CPAL_LOG("\n\rLOG : I2C Device Config");   
    
    /* If General Call Mode Option Bit Selected */
    if ((pDevInitStruct->wCPAL_Options & CPAL_OPT_I2C_GENCALL) != 0)
    {
      /* Enable GENERAL CALL Address Mode */
      __CPAL_I2C_HAL_ENABLE_GENCALL(pDevInitStruct->CPAL_Dev); 
      
      CPAL_LOG("\n\rLOG : I2C Device GENCALL Mode Enabled"); 
    }
    
    /* If Dual Address Mode Option Bit Selected */
    if ((pDevInitStruct->wCPAL_Options & CPAL_OPT_I2C_DUALADDR) != 0)
    {
      /* Enable Dual Address Mode */
     __CPAL_I2C_HAL_ENABLE_DUALADDR(pDevInitStruct->CPAL_Dev);
      
      /* Configure OAR2 */
      __CPAL_I2C_HAL_OAR2_CONF(pDevInitStruct->CPAL_Dev, (uint8_t)(pDevInitStruct->wCPAL_Options & 0x000000FE));
         
      CPAL_LOG("\n\rLOG : I2C Device DUAL ADDR Mode Enabled"); 
    }    

    /* If NACK Slave Own Address option bit selected */
    if ((pDevInitStruct->wCPAL_Options & CPAL_OPT_I2C_NACK_ADD) != 0)
    {
      /* Disable Acknowledgement of own Address */
      __CPAL_I2C_HAL_DISABLE_ACK(pDevInitStruct->CPAL_Dev);

      CPAL_LOG("\n\rLOG : I2C Device NACK Own Address Mode Enabled");
    }
    
#ifdef CPAL_I2C_DMA_PROGMODEL
    /*----------------------------------------------------------------------------
    DMA Initialization : 
    ---------------------------------------------------------------------------*/   
    /* If DMA Programming model is selected*/
    if (pDevInitStruct->CPAL_ProgModel == CPAL_PROGMODEL_DMA) 
    {
      /* Initialize I2Cx DMA Channels */
      CPAL_I2C_HAL_DMAInit(pDevInitStruct->CPAL_Dev, pDevInitStruct->CPAL_Direction, pDevInitStruct->wCPAL_Options);
      
      CPAL_LOG("\n\rLOG : I2C Device DMA Init");  
    }
#endif /* CPAL_I2C_DMA_PROGMODEL */
    
    /*----------------------------------------------------------------------------
    Peripheral and DMA interrupts Initialization
    ---------------------------------------------------------------------------*/
    /* Initialize I2Cx Interrupts */
    CPAL_I2C_HAL_ITInit(pDevInitStruct->CPAL_Dev, pDevInitStruct->wCPAL_Options, pDevInitStruct->CPAL_Direction, pDevInitStruct->CPAL_ProgModel);
    
    CPAL_LOG("\n\rLOG : I2C Device IT Init");
    
    /* Update CPAL_State to CPAL_STATE_READY */
    pDevInitStruct->CPAL_State = CPAL_STATE_READY;
    
    CPAL_LOG("\n\rLOG : I2C Device Ready"); 
    
    /* Initialize Timeout Procedure */
    _CPAL_TIMEOUT_INIT();
    
    return CPAL_PASS;
  }    
  /* If CPAL_State is BUSY (a transaction is still on going) Exit Init function */
  else 
  {
    CPAL_LOG("\n\rERROR : I2C Device Busy"); 
    
    return CPAL_FAIL; 
  }
}


/**
  * @brief  Deinitialize the peripheral and all related clocks, GPIOs, DMA and NVIC 
  *         to their reset values.
  * @param  pDevInitStruct: Pointer to the peripheral configuration structure.
  * @retval CPAL_PASS or CPAL_FAIL
  * @note   The Peripheral clock is disabled but the GPIO Ports clocks remains 
  *         enabled after this deinitialization. 
  */
uint32_t CPAL_I2C_DeInit(CPAL_InitTypeDef* pDevInitStruct) 
{
  CPAL_LOG("\n\r\n\rLOG <CPAL_I2C_DeInit> : I2C Device Deinit");
  
  /* If CPAL_State is not BUSY */
  if ((pDevInitStruct->CPAL_State == CPAL_STATE_READY) 
     || (pDevInitStruct->CPAL_State == CPAL_STATE_ERROR) 
     || (pDevInitStruct->CPAL_State == CPAL_STATE_DISABLED))
  {
    /* 
    - If CPAL_State is CPAL_STATE_ERROR (an Error occurred in transaction):
    Perform the deinitialization routines 
    - If CPAL_State is CPAL_STATE_READY:  
    Perform the deinitialization routines 
    - If CPAL_State is CPAL_STATE_DISABLED:  
    Perform the deinitialization routines                                   */
    
    /*----------------------------------------------------------------------------
    GPIO pins Deinitialization
    Note: The GPIO clock remains enabled after this deinitialization
    ---------------------------------------------------------------------------*/
    /* Deinitialize I2Cx GPIO */
    CPAL_I2C_HAL_GPIODeInit(pDevInitStruct->CPAL_Dev);
    
    CPAL_LOG("\n\rLOG : I2C Device IOs Deinit");
    
    /*----------------------------------------------------------------------------
    Peripheral Deinitialization
    ---------------------------------------------------------------------------*/   
    /* Disable I2Cx Device */
    __CPAL_I2C_HAL_DISABLE_DEV(pDevInitStruct->CPAL_Dev);
    
    CPAL_LOG("\n\rLOG : I2C Device Disabled"); 
    
    /*----------------------------------------------------------------------------
    Peripheral Clock Deinitialization
    ---------------------------------------------------------------------------*/  
    /* Deinitialize I2Cx Clock */
    CPAL_I2C_HAL_CLKDeInit(pDevInitStruct->CPAL_Dev);
    
    CPAL_LOG("\n\rLOG : I2C Device Clock Deinit");       
    
#ifdef CPAL_I2C_DMA_PROGMODEL
    /*----------------------------------------------------------------------------
    DMA Deinitialization : if DMA Programming model is selected
    ---------------------------------------------------------------------------*/   
    if (pDevInitStruct->CPAL_ProgModel == CPAL_PROGMODEL_DMA)
    {
      CPAL_I2C_HAL_DMADeInit(pDevInitStruct->CPAL_Dev, pDevInitStruct->CPAL_Direction);
      
      CPAL_LOG("\n\rLOG : I2C Device DMA Deinit");  
    } 
#endif /* CPAL_I2C_DMA_PROGMODEL */
    
    /*----------------------------------------------------------------------------
    Interrupts Deinitialization
    ---------------------------------------------------------------------------*/
    CPAL_I2C_HAL_ITDeInit(pDevInitStruct->CPAL_Dev, pDevInitStruct->wCPAL_Options, pDevInitStruct->CPAL_Direction, pDevInitStruct->CPAL_ProgModel);
    
    CPAL_LOG("\n\rLOG : I2C Device IT Deinit"); 
    
    /*----------------------------------------------------------------------------
    Structure fields initialization
    -----------------------------------------------------------------------------*/    
    /* Initialize pDevInitStruct state parameters to their default values */
    pDevInitStruct-> CPAL_State     = CPAL_STATE_DISABLED;     /* Device Disabled */
    pDevInitStruct-> wCPAL_DevError = CPAL_I2C_ERR_NONE;       /* No Device Error */
    pDevInitStruct-> wCPAL_Timeout  = ((uint32_t)CPAL_I2C_TIMEOUT_DEFAULT);  /* Set timeout value to CPAL_I2C_TIMEOUT_DEFAULT */  

    CPAL_LOG("\n\rLOG :Set State fields to default"); 
    
    /*----------------------------------------------------------------------------
    Deinitialize Timeout Procedure
    -----------------------------------------------------------------------------*/
    _CPAL_TIMEOUT_DEINIT();  
    
    return CPAL_PASS;
  }  
  /* If CPAL_State is BUSY (a transaction is still on going) Exit Init function */
  else 
  {
    CPAL_LOG("\n\rERROR : I2C Device Busy"); 
    
    return CPAL_FAIL; 
  }
}


/**
  * @brief  Initialize the peripheral structure with default values according
  *         to the specified parameters in the CPAL_I2CDevTypeDef structure.
  * @param  pDevInitStruct: Pointer to the peripheral configuration structure.
  * @retval CPAL_PASS or CPAL_FAIL. 
  */
uint32_t CPAL_I2C_StructInit(CPAL_InitTypeDef* pDevInitStruct) 
{   
  /* Initialize I2C_InitStructure to their default values */
  I2C_InitStructure.I2C_ClockSpeed          = 100000;                        /* Initialize the I2C_ClockSpeed member */
  I2C_InitStructure.I2C_Mode                = I2C_Mode_I2C;                  /* Initialize the I2C_Mode member */
  I2C_InitStructure.I2C_DutyCycle           = I2C_DutyCycle_2;               /* Initialize the I2C_DutyCycle member */
  I2C_InitStructure.I2C_OwnAddress1         = 0;                             /* Initialize the I2C_OwnAddress1 member */
  I2C_InitStructure.I2C_Ack                 = I2C_Ack_Enable;                /* Initialize the I2C_Ack member */
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;  /* Initialize the I2C_AcknowledgedAddress member */
  
  /* Initialize pDevInitStruct parameter to their default values */
  pDevInitStruct-> CPAL_Direction     = CPAL_DIRECTION_TXRX;                  /* Transmitter and Receiver direction selected */
  pDevInitStruct-> CPAL_Mode          = CPAL_MODE_MASTER;                     /* Mode Master selected */
  pDevInitStruct-> CPAL_ProgModel     = CPAL_PROGMODEL_DMA;                   /* DMA Programming Model selected */
  pDevInitStruct-> pCPAL_TransferTx   = pNULL;                                /* Point pCPAL_TransferTx to a Null pointer */
  pDevInitStruct-> pCPAL_TransferRx   = pNULL;                                /* Point pCPAL_TransferRx to a Null pointer */ 
  pDevInitStruct-> CPAL_State         = CPAL_STATE_DISABLED;                  /* Device Disabled */
  pDevInitStruct-> wCPAL_DevError     = CPAL_I2C_ERR_NONE;                    /* No Device Error */
  pDevInitStruct-> wCPAL_Options      = ((uint32_t)0x00000000);               /* No Options selected */
  pDevInitStruct-> wCPAL_Timeout      = ((uint32_t)CPAL_I2C_TIMEOUT_DEFAULT); /* Set timeout value to CPAL_I2C_TIMEOUT_DEFAULT */
  pDevInitStruct-> pCPAL_I2C_Struct   = &I2C_InitStructure;                   /* Point to I2C_InitStructure (with default values) */
  
  CPAL_LOG("\n\r\n\rLOG <CPAL_I2C_StructInit> : I2C Device Structure set to Default Value"); 
  
  return CPAL_PASS;
}


#if defined (CPAL_I2C_MASTER_MODE) || ! defined (CPAL_I2C_LISTEN_MODE) 
/**
  * @brief  Allows to send a data or a buffer of data through the peripheral to 
  *         a selected device in a selected location address.
  * @param  pDevInitStruct: Pointer to the peripheral configuration structure.
  * @retval CPAL_PASS or CPAL_FAIL.
  */
uint32_t CPAL_I2C_Write(CPAL_InitTypeDef* pDevInitStruct)
{      
  /* Check I2C State: 
  - If busy       --> exit Write operation  
  - If disabled   --> exit Write operation    
  - If error      --> exit Write operation
  - If ready      --> 
                      - If Master Mode selected with No memory Address Mode OR Slave Mode Selected. 
                           - Test the value of programming model
                              - If DMA Programming Model        --> enable TX DMA Channel.
                              - If Interrupt Programming Model  --> enable Buffer Interrupt.
                      - Update CPAL_STATE to CPAL_STATE_READY_TX.
                      - If Master mode selected --> generate start condition 
                      - Enable Event Interrupt                                                 */
  
  CPAL_LOG("\n\r\n\rLOG <CPAL_I2C_Write> : I2C Device Write OP");
  
  /* If Device is Busy (a transaction is still on going) Exit Write function */
  if (((pDevInitStruct->CPAL_State & CPAL_STATE_BUSY) != 0) 
     || (pDevInitStruct->CPAL_State == CPAL_STATE_READY_TX) 
     || (pDevInitStruct->CPAL_State == CPAL_STATE_READY_RX))
  {
    CPAL_LOG("\n\rERROR : I2C Device Busy"); 
    
    return CPAL_FAIL;
  }  
  /* If CPAL_State is CPAL_STATE_DISABLED (device is not initialized) Exit Write function */  
  else if (pDevInitStruct->CPAL_State == CPAL_STATE_DISABLED)  
  {
    CPAL_LOG("\n\rERROR : I2C Device Not Initialized"); 
    
    return CPAL_FAIL;
  }  
  /* If CPAL_State is CPAL_STATE_ERROR (Error occurred ) */
  else if (pDevInitStruct->CPAL_State == CPAL_STATE_ERROR)
  {
    CPAL_LOG("\n\rERROR : I2C Device Error"); 
    
    return CPAL_FAIL;
  }  
  /* If CPAL_State is CPAL_STATE_READY ( Start Communication )*/
  else
  {   
    /* Update CPAL_State to CPAL_STATE_BUSY */
    pDevInitStruct->CPAL_State = CPAL_STATE_BUSY;
    
    /* No Stop Condition Generation option Mode bit not selected */   
    if ((pDevInitStruct->wCPAL_Options & CPAL_OPT_I2C_NOSTOP_MODE) == 0)
    {
      /* Wait until Busy flag is reset */ 
      __CPAL_I2C_TIMEOUT(!(__CPAL_I2C_HAL_GET_BUSY(pDevInitStruct->CPAL_Dev)), CPAL_I2C_TIMEOUT_BUSY);
    } 
  
    /* If Master Mode selected */
    if (pDevInitStruct->CPAL_Mode == CPAL_MODE_MASTER)
    {
#ifdef CPAL_I2C_MASTER_MODE       
      CPAL_LOG("\n\rLOG : I2C Device Master");
      
      /* Generate Start */
      __CPAL_I2C_HAL_START(pDevInitStruct->CPAL_Dev); 
      
      /* If No memory Address Option Bit is Selected  */   
      if ((pDevInitStruct->wCPAL_Options & CPAL_OPT_NO_MEM_ADDR) != 0)
        
      {             
        /* Switch Programing Mode Enable DMA or IT Buffer */
        CPAL_I2C_Enable_DMA_IT(pDevInitStruct, CPAL_DIRECTION_TX);      
      }   
      
      /* Update CPAL_State to CPAL_STATE_READY_TX */
      pDevInitStruct->CPAL_State = CPAL_STATE_READY_TX;
      
      CPAL_LOG("\n\rLOG : I2C Device Ready TX");
      
      CPAL_LOG("\n\rLOG : I2C Device Generates Start");
      
      /* Initialize timeout value */
      pDevInitStruct->wCPAL_Timeout = CPAL_I2C_TIMEOUT_MIN + CPAL_I2C_TIMEOUT_SB;
#endif /* CPAL_I2C_MASTER_MODE */       
    }     
    /* If Slave Mode selected */
    else      
    {
#ifdef CPAL_I2C_SLAVE_MODE       
      /* If NACK Slave Own Address option bit selected */
      if ((pDevInitStruct->wCPAL_Options & CPAL_OPT_I2C_NACK_ADD) != 0)
      {
        /* Enable Acknowledgement of Own address */
        __CPAL_I2C_HAL_ENABLE_ACK(pDevInitStruct->CPAL_Dev);
      }
      
      /* Switch Programing Mode Enable DMA or IT Buffer */
      CPAL_I2C_Enable_DMA_IT(pDevInitStruct, CPAL_DIRECTION_TX);
      
      /* Update CPAL_State to CPAL_STATE_READY_TX */
      pDevInitStruct->CPAL_State = CPAL_STATE_READY_TX;
      
      CPAL_LOG("\n\rLOG : I2C Device Ready TX");
      
      CPAL_LOG("\n\rLOG : I2C Device Slave");
#endif /* CPAL_I2C_SLAVE_MODE */      
    } 
    
    /* Enable EVENT Interrupts*/
    CPAL_LOG("\n\rLOG : I2C Device EVT IT Enabled"); 
    
    __CPAL_I2C_HAL_ENABLE_EVTIT(pDevInitStruct->CPAL_Dev);
  }
  
   return CPAL_PASS;
}

/**
  * @brief  Allows to receive a data or a buffer of data through the peripheral 
  *         from a selected device in a selected location address.
  * @param  pDevInitStruct: Pointer to the peripheral configuration structure.
  * @retval CPAL_PASS or CPAL_FAIL. 
  */
uint32_t CPAL_I2C_Read(CPAL_InitTypeDef* pDevInitStruct)
{    
  /* Check I2C State: 
  - If busy       --> exit Read operation  
  - If disabled   --> exit Read operation  
  - If error      --> exit Read operation
  - If ready      --> 
          - If Master Mode with Memory Address Mode.
              - Generate start
              - Send Slave Address
              - Send Memory Address
          - Else Test the value of programming model
              - If DMA Programming Model        --> enable RX DMA Channel 
              - If Interrupt Programming Model  --> enable Buffer Interrupt 
          - Update CPAL_STATE to CPAL_STATE_READY_RX.
          - If Master mode selected --> generate start condition 
          - Enable Event Interrupt                                               */
  
  CPAL_LOG("\n\r\n\rLOG <CPAL_I2C_Read> : I2C Device Perform Read OP");
  
   /* If Device is Busy (a transaction is still on going) Exit Read function */
   if (((pDevInitStruct->CPAL_State & CPAL_STATE_BUSY) != 0)
      || (pDevInitStruct->CPAL_State == CPAL_STATE_READY_TX)
      || (pDevInitStruct->CPAL_State == CPAL_STATE_READY_RX))
  {
    CPAL_LOG("\n\rERROR : I2C Device Busy"); 
    
    return CPAL_FAIL;
  }  
  /* If CPAL_State is CPAL_STATE_DISABLED (device is not initialized) Exit Read function */  
  else if (pDevInitStruct->CPAL_State == CPAL_STATE_DISABLED)  
  {
    CPAL_LOG("\n\rERROR : I2C Device Not Initialized"); 
    
    return CPAL_FAIL;
  }  
  /* If CPAL_State is CPAL_STATE_ERROR (Error occurred ) */
  else if (pDevInitStruct->CPAL_State == CPAL_STATE_ERROR)
  {
    CPAL_LOG("\n\rERROR : I2C Device Error"); 
    
    return CPAL_FAIL;
  }  
  /* If CPAL_State is CPAL_STATE_READY */
  else
  {
    /* Update CPAL_State to CPAL_STATE_BUSY */
    pDevInitStruct->CPAL_State = CPAL_STATE_BUSY;
   
    /* No Stop Condition Generation Mode option bit not selected */   
    if ((pDevInitStruct->wCPAL_Options & CPAL_OPT_I2C_NOSTOP_MODE) == 0)
    {
      /* Wait until Busy flag is reset */ 
      __CPAL_I2C_TIMEOUT(!(__CPAL_I2C_HAL_GET_BUSY(pDevInitStruct->CPAL_Dev)), CPAL_I2C_TIMEOUT_BUSY);
    }

#ifdef CPAL_I2C_DMA_1BYTE_CASE              
    /* If One byte transfer with DMA programming model */
    if ((pDevInitStruct->CPAL_ProgModel == CPAL_PROGMODEL_DMA) 
       && (pDevInitStruct->pCPAL_TransferRx->wNumData == 1))
    {
      /* Affect 1Byte DMA option to wCPAL_Options */
      pDevInitStruct->wCPAL_Options |= CPAL_DMA_1BYTE_CASE;
      
      /* Change ProgModel to Interrupt */
      pDevInitStruct->CPAL_ProgModel = CPAL_PROGMODEL_INTERRUPT;
    }
#endif /* CPAL_I2C_DMA_1BYTE_CASE */
    
#ifdef CPAL_I2C_MASTER_MODE    
    /* If "No Memory Address" Option Bit is not selected and Master Mode selected */
    if (((pDevInitStruct->wCPAL_Options & CPAL_OPT_NO_MEM_ADDR) == 0) 
       && (pDevInitStruct->CPAL_Mode == CPAL_MODE_MASTER ))
    {       
      CPAL_LOG("\n\rLOG : I2C Device Master No Addr Mem Mode");
      
      /* Generate Start */
      __CPAL_I2C_HAL_START(pDevInitStruct->CPAL_Dev);
      
      /* Wait until SB flag is set */
      __CPAL_I2C_TIMEOUT(__CPAL_I2C_HAL_GET_SB(pDevInitStruct->CPAL_Dev), CPAL_I2C_TIMEOUT_SB);
      
      /* Send Device Address */
      /* If 7 Bit Addressing Mode */
      if (pDevInitStruct->pCPAL_I2C_Struct->I2C_AcknowledgedAddress == I2C_AcknowledgedAddress_7bit)
      {             
        /* Send Slave address with bit0 reset for write */
        __CPAL_I2C_HAL_SEND((pDevInitStruct->CPAL_Dev), (uint8_t)((pDevInitStruct->pCPAL_TransferRx->wAddr1) & (uint8_t)(~I2C_OAR1_ADD0)));   
        
        /* Wait until ADDR flag is reset */ 
        __CPAL_I2C_TIMEOUT(__CPAL_I2C_HAL_GET_ADDR(pDevInitStruct->CPAL_Dev), CPAL_I2C_TIMEOUT_ADDR);        
      }      
  #ifdef CPAL_I2C_10BIT_ADDR_MODE
      /* If 10 Bit Addressing Mode */
      else
      {
        /* Declare local variable that contains Address Header */
        uint8_t I2CHeaderAddress = 0x00;
        
        /* Calculate Header Address  */ 
        I2CHeaderAddress = (uint8_t)((((pDevInitStruct->pCPAL_TransferRx->wAddr1) & 0xFF00) >>7) | 0xF0);
        
        /* Send Header Address */ 
        __CPAL_I2C_HAL_SEND((pDevInitStruct->CPAL_Dev), I2CHeaderAddress);   
        
        /* Wait until ADD10 flag is set */ 
        __CPAL_I2C_TIMEOUT(__CPAL_I2C_HAL_GET_ADD10(pDevInitStruct->CPAL_Dev), CPAL_I2C_TIMEOUT_ADD10); 
        
        /* Send Slave address with bit0 reset for write */
        __CPAL_I2C_HAL_SEND((pDevInitStruct->CPAL_Dev), (uint8_t)(pDevInitStruct->pCPAL_TransferRx->wAddr1));   
        
        /* Wait until ADDR flag is set */ 
        __CPAL_I2C_TIMEOUT(__CPAL_I2C_HAL_GET_ADDR(pDevInitStruct->CPAL_Dev), CPAL_I2C_TIMEOUT_ADDR);         
      }      
  #endif /* CPAL_I2C_10BIT_ADDR_MODE */     
      
      CPAL_LOG("\n\rLOG : I2C Device Target Address Sent ");
      
      /* Clear ADDR flag: (Read SR1 followed by read of SR2), SR1 read operation is already done */
      __CPAL_I2C_HAL_CLEAR_ADDR(pDevInitStruct->CPAL_Dev); 
      
      /* Wait until TXE flag is set */ 
      __CPAL_I2C_TIMEOUT(__CPAL_I2C_HAL_GET_TXE(pDevInitStruct->CPAL_Dev), CPAL_I2C_TIMEOUT_TXE); 
      
      /* If 8 Bit register mode */
      if ((pDevInitStruct->wCPAL_Options & CPAL_OPT_16BIT_REG) == 0)
      {
        /* Send Register Address */
        __CPAL_I2C_HAL_SEND((pDevInitStruct->CPAL_Dev), (uint8_t)(((pDevInitStruct->pCPAL_TransferRx->wAddr2)& 0x00FF))); 
        
        /* Wait until TXE flag is set */ 
        __CPAL_I2C_TIMEOUT(__CPAL_I2C_HAL_GET_TXE(pDevInitStruct->CPAL_Dev), CPAL_I2C_TIMEOUT_TXE); 
      }      
  #ifdef CPAL_16BIT_REG_OPTION
      /* If 16 Bit register mode */
      else
      {
        /* Send MSB Register Address */
        __CPAL_I2C_HAL_SEND((pDevInitStruct->CPAL_Dev), (uint8_t)(((pDevInitStruct->pCPAL_TransferRx->wAddr2)& 0xFF00) >>8));  
        
        /* Wait until TXE flag is set */ 
        __CPAL_I2C_TIMEOUT(__CPAL_I2C_HAL_GET_TXE(pDevInitStruct->CPAL_Dev), CPAL_I2C_TIMEOUT_TXE);
        
        /* Send LSB Register Address */
        __CPAL_I2C_HAL_SEND((pDevInitStruct->CPAL_Dev), (uint8_t)((pDevInitStruct->pCPAL_TransferRx->wAddr2)& 0x00FF));  
        
        /* Initialize Timeout value */
        pDevInitStruct->wCPAL_Timeout = CPAL_I2C_TIMEOUT_MIN + CPAL_I2C_TIMEOUT_TXE;
        
        /* Wait until TXE flag is set */ 
        __CPAL_I2C_TIMEOUT(__CPAL_I2C_HAL_GET_TXE(pDevInitStruct->CPAL_Dev), CPAL_I2C_TIMEOUT_TXE); 
      }      
  #endif /* CPAL_16BIT_REG_OPTION */
      
      CPAL_LOG("\n\rLOG : I2C Device Target Memory Address Sent");      
    }  
   else 
#endif /* CPAL_I2C_MASTER_MODE */   
      if (pDevInitStruct->CPAL_ProgModel == CPAL_PROGMODEL_DMA)
    {     
      CPAL_LOG("\n\rLOG : I2C Device Master No Memory Address Mode ");
      
       /* Switch Programing Mode Enable DMA or IT Buffer */
      CPAL_I2C_Enable_DMA_IT(pDevInitStruct, CPAL_DIRECTION_RX);
    }   
    
    /* Update CPAL_State to CPAL_STATE_READY_RX */
    pDevInitStruct->CPAL_State = CPAL_STATE_READY_RX;
    
    CPAL_LOG("\n\rLOG : I2C Device Ready RX"); 
        
    
    /* If Master Mode selected */
    if (pDevInitStruct->CPAL_Mode == CPAL_MODE_MASTER )
    {
#ifdef CPAL_I2C_MASTER_MODE 
      CPAL_LOG("\n\rLOG : I2C Device Master");
      
      /* Generate Start */
     __CPAL_I2C_HAL_START(pDevInitStruct->CPAL_Dev);
      
     CPAL_LOG("\n\rLOG : I2C Device Generates Start"); 
         
      /* Initialize Timeout value */
      pDevInitStruct->wCPAL_Timeout = CPAL_I2C_TIMEOUT_MIN + CPAL_I2C_TIMEOUT_SB;      
#endif /* CPAL_I2C_MASTER_MODE */      
    }    
    /* If Slave Mode selected */
    else   
    {   
#ifdef CPAL_I2C_SLAVE_MODE      
      /* If NACK Slave Own Address option bit selected */
      if ((pDevInitStruct->wCPAL_Options & CPAL_OPT_I2C_NACK_ADD) != 0)
      {
        /* Enable Acknowledgement of Own address */
        __CPAL_I2C_HAL_ENABLE_ACK(pDevInitStruct->CPAL_Dev);
      }   
      
      if (pDevInitStruct->CPAL_ProgModel == CPAL_PROGMODEL_INTERRUPT)
      {       
        /* Enable IT Buffer */
        CPAL_I2C_Enable_DMA_IT(pDevInitStruct, CPAL_DIRECTION_RX);
      }  
      
      CPAL_LOG("\n\rLOG : I2C Device Slave");
#endif /* CPAL_I2C_SLAVE_MODE */       
    } 
    
    CPAL_LOG("\n\rLOG : I2C Device EVT IT Enabled");   
    
    /* Enable EVENT Interrupts*/
     __CPAL_I2C_HAL_ENABLE_EVTIT(pDevInitStruct->CPAL_Dev);
  }
  
  return CPAL_PASS;
}
#endif /* CPAL_I2C_MASTER_MODE || ! CPAL_I2C_LISTEN_MODE */ 

#if defined (CPAL_I2C_LISTEN_MODE) && defined (CPAL_I2C_SLAVE_MODE)
/**
  * @brief  Allows slave device to start a communication without knowing in advance 
  *         the nature of the operation (read or write). Slave waits until it receive
  *         its own address.CPAL_I2C_SLAVE_READ_UserCallback is called for a read request 
  *         and CPAL_I2C_SLAVE_WRITE_UserCallback for a write request in I2C_SLAVE_ADDR_Handle.
  *         User must implement inorder to configure DMA, interrupts and transfer parameters.  
  * @param  pDevInitStruct: Pointer to the peripheral configuration structure.
  * @retval CPAL_PASS or CPAL_FAIL. 
  */
uint32_t CPAL_I2C_Listen(CPAL_InitTypeDef* pDevInitStruct)
{    
  /* Check I2C State: 
  - If busy       --> exit operation  
  - If disabled   --> exit operation  
  - If error      --> exit operation
  - If ready      --> 
          - Enable Event Interrupt                                               */
  
  CPAL_LOG("\n\r\n\rLOG <CPAL_I2C_Listen> : I2C Device in listen mode");
  
  /* If Device is Busy (a transaction is still on going) Exit function */
  if (((pDevInitStruct->CPAL_State & CPAL_STATE_BUSY) != 0)
      || (pDevInitStruct->CPAL_State == CPAL_STATE_READY_TX)
        || (pDevInitStruct->CPAL_State == CPAL_STATE_READY_RX))
  {
    CPAL_LOG("\n\rERROR : I2C Device Busy"); 
    
    return CPAL_FAIL;
  }  
  /* If CPAL_State is CPAL_STATE_DISABLED (device is not initialized) Exit function */  
  else if (pDevInitStruct->CPAL_State == CPAL_STATE_DISABLED)  
  {
    CPAL_LOG("\n\rERROR : I2C Device Not Initialized"); 
    
    return CPAL_FAIL;
  }  
  /* If CPAL_State is CPAL_STATE_ERROR (Error occurred ) */
  else if (pDevInitStruct->CPAL_State == CPAL_STATE_ERROR)
  {
    CPAL_LOG("\n\rERROR : I2C Device Error"); 
    
    return CPAL_FAIL;
  }  
  /* If CPAL_State is CPAL_STATE_READY */
  else
  {
    /* Set device to slave mode */
    pDevInitStruct->CPAL_Mode = CPAL_MODE_SLAVE; 
    
    /* Update CPAL_State to CPAL_STATE_BUSY */
    pDevInitStruct->CPAL_State = CPAL_STATE_BUSY;
    
#ifdef CPAL_I2C_DMA_1BYTE_CASE              
    /* If One byte transfer with DMA programming model */
    if ((pDevInitStruct->CPAL_ProgModel == CPAL_PROGMODEL_DMA) 
        && (pDevInitStruct->pCPAL_TransferRx->wNumData == 1))
    {
      /* Affect 1Byte DMA option to wCPAL_Options */
      pDevInitStruct->wCPAL_Options |= CPAL_DMA_1BYTE_CASE;
      
      /* Change ProgModel to Interrupt */
      pDevInitStruct->CPAL_ProgModel = CPAL_PROGMODEL_INTERRUPT;
    }
#endif /* CPAL_I2C_DMA_1BYTE_CASE */
    
    /* If NACK Slave Own Address option bit selected */
    if ((pDevInitStruct->wCPAL_Options & CPAL_OPT_I2C_NACK_ADD) != 0)
    {
      /* Enable Acknowledgement of Own address */
      __CPAL_I2C_HAL_ENABLE_ACK(pDevInitStruct->CPAL_Dev);
    }
    
    CPAL_LOG("\n\rLOG : I2C Device EVT IT Enabled");   
    
    /* Enable EVENT Interrupts*/
    __CPAL_I2C_HAL_ENABLE_EVTIT(pDevInitStruct->CPAL_Dev);    
  }
  
  return CPAL_PASS;
}
#endif /* CPAL_I2C_LISTEN_MODE && CPAL_I2C_SLAVE_MODE */


/**
  * @brief  Wait until target device is ready for communication (This function is 
  *         used with Memory devices).           
  * @param  pDevInitStruct: Pointer to the peripheral configuration structure.
  * @retval CPAL_PASS or CPAL_FAIL. 
  */
uint32_t CPAL_I2C_IsDeviceReady(CPAL_InitTypeDef* pDevInitStruct)
{ 
  __IO uint32_t Timeout = 0xFFFF;
   
  CPAL_LOG("\n\r\n\rLOG <CPAL_I2C_DeviceIsReady> : Wait until I2C Device is Ready");
  
  /* Set CPAL state to CPAL_STATE_DISABLED */
  pDevInitStruct->CPAL_State = CPAL_STATE_BUSY;
  
  /* Disable ERROR Interrupt */
  __CPAL_I2C_HAL_DISABLE_ERRIT(pDevInitStruct->CPAL_Dev);
  
  /* Disable I2Cx Device */
  __CPAL_I2C_HAL_DISABLE_DEV(pDevInitStruct->CPAL_Dev);
  
  /* Enable I2Cx Device */
  __CPAL_I2C_HAL_ENABLE_DEV(pDevInitStruct->CPAL_Dev);
  
  /* Generate Start */
  __CPAL_I2C_HAL_START(pDevInitStruct->CPAL_Dev);
  
  /* Wait until SB flag is set */ 
  __CPAL_I2C_TIMEOUT(__CPAL_I2C_HAL_GET_SB(pDevInitStruct->CPAL_Dev), CPAL_I2C_TIMEOUT_SB); 
  
  /* Send Slave address with bit0 reset for write */
  __CPAL_I2C_HAL_SEND((pDevInitStruct->CPAL_Dev), (uint8_t)((pDevInitStruct->pCPAL_TransferTx->wAddr1) & (uint8_t)(~I2C_OAR1_ADD0)));   
  
  /* wait until timeout elapsed or target device acknowledge its address*/
  while ((__CPAL_I2C_HAL_GET_ADDR(pDevInitStruct->CPAL_Dev) == 0) && (Timeout-- != 0));
  
  /* If Timeout occurred  */
  if (Timeout == 0) 
  {    
    return CPAL_FAIL;    
  }  
  /* If ADDR flag is set */
  else
  {      
    /* Clear AF flag */
    __CPAL_I2C_HAL_CLEAR_AF(pDevInitStruct->CPAL_Dev);
    
    /* Generate Stop */
    __CPAL_I2C_HAL_STOP(pDevInitStruct->CPAL_Dev);  
    
    /* wait until Busy flag is reset */
    while(__CPAL_I2C_HAL_GET_BUSY(pDevInitStruct->CPAL_Dev));
    
    /* Disable I2Cx Device */
    __CPAL_I2C_HAL_DISABLE_DEV(pDevInitStruct->CPAL_Dev);
    
    CPAL_LOG("\n\rLOG : I2C Device Disabled"); 
    
    /* Enable I2Cx Device */
    __CPAL_I2C_HAL_ENABLE_DEV(pDevInitStruct->CPAL_Dev);
    
    CPAL_LOG("\n\rLOG : I2C Device Enabled"); 
    
    /* Enable ACK */
    __CPAL_I2C_HAL_ENABLE_ACK(pDevInitStruct->CPAL_Dev);
    
    /* Enable ERROR Interrupt */
    __CPAL_I2C_HAL_ENABLE_ERRIT(pDevInitStruct->CPAL_Dev);
    
    /* Set CPAL state to ready */
    pDevInitStruct->CPAL_State = CPAL_STATE_READY;
   
    CPAL_LOG("\n\rLOG : I2C Target device Ready");  
    
    return CPAL_PASS;
  }  
}


/*================== CPAL_I2C_Interrupt_Handler ==================*/

/**
  * @brief  This function handles I2C interrupt request for preparing communication
  *         and for transfer phase in case of using Interrupt Programming Model.
  * @param  pDevInitStruct: Pointer to the peripheral configuration structure.
  * @retval CPAL_PASS. 
  */
uint32_t CPAL_I2C_EV_IRQHandler( CPAL_InitTypeDef* pDevInitStruct)
{     
  __IO uint16_t I2CFlagStatus = 0x0000;
  
  /* Read I2C1 Status Registers 1 and 2 */
  I2CFlagStatus = __CPAL_I2C_HAL_GET_EVENT(pDevInitStruct->CPAL_Dev); 
 
#ifdef CPAL_I2C_MASTER_MODE
  /*----------------------------------------------------------------------------------------------*/
  /*---------------------------------- If Master Mode selected ----------------------------------*/
  if (pDevInitStruct->CPAL_Mode == CPAL_MODE_MASTER)
  { 
    /*----------------------------------------*/  
    /*------------- If SB event --------------*/
    if ((I2CFlagStatus & (uint16_t)CPAL_I2C_EVT_SB ) != 0)
    {       
      return I2C_MASTER_START_Handle(pDevInitStruct);        
    } 
    
    /*----------------------------------------*/
    /*------------- If ADDR event ------------*/
    if((I2CFlagStatus & (uint16_t)CPAL_I2C_EVT_ADDR ) != 0)
    {  
      return I2C_MASTER_ADDR_Handle(pDevInitStruct);              
    }
    
 #ifdef CPAL_I2C_10BIT_ADDR_MODE
    /*----------------------------------------*/
    /*------------- If ADD10 event *----------*/
    if ((I2CFlagStatus & (uint16_t)CPAL_I2C_EVT_ADD10) != 0)
    { 
      return I2C_MASTER_ADD10_Handle(pDevInitStruct);  
    }    
 #endif /* CPAL_I2C_10BIT_ADDR_MODE */
    
 #ifdef CPAL_I2C_IT_PROGMODEL   
    /*----------------------------------------*/
    /*------------- If TXE event -------------*/
    if (((I2CFlagStatus & (uint16_t)CPAL_I2C_EVT_TXE) != 0) && (pDevInitStruct->CPAL_State == CPAL_STATE_BUSY_TX))
    {  
      return I2C_MASTER_TXE_Handle(pDevInitStruct); 
    }
 #endif /* CPAL_I2C_IT_PROGMODEL */
    
 #if defined (CPAL_I2C_IT_PROGMODEL) || defined (CPAL_I2C_DMA_1BYTE_CASE)    
    /*----------------------------------------*/
    /*------------- If RXNE event ------------*/
    if (((I2CFlagStatus & (uint16_t)CPAL_I2C_EVT_RXNE) != 0) && (pDevInitStruct->CPAL_State == CPAL_STATE_BUSY_RX))
    { 
      return I2C_MASTER_RXNE_Handle(pDevInitStruct); 
    }      
 #endif /* CPAL_I2C_IT_PROGMODEL || CPAL_I2C_DMA_1BYTE_CASE */
  }
#endif /* CPAL_I2C_MASTER_MODE */
 
#ifdef CPAL_I2C_SLAVE_MODE  
  /*----------------------------------------------------------------------------------------------*/
  /*---------------------------------- If Slave Mode selected ------------------------------------*/
  if (pDevInitStruct->CPAL_Mode == CPAL_MODE_SLAVE)
  {  
    /*----------------------------------------*/        
    /*------------- If ADDR event ------------*/
    if ((I2CFlagStatus & (uint16_t)CPAL_I2C_EVT_ADDR ) != 0)
    { 
      return I2C_SLAVE_ADDR_Handle(pDevInitStruct); 
    }    

 #ifdef CPAL_I2C_IT_PROGMODEL    
    /*----------------------------------------*/
    /*------------- If TXE event -------------*/
    if ((I2CFlagStatus & (uint16_t)CPAL_I2C_EVT_TXE) != 0)
    { 
      return I2C_SLAVE_TXE_Handle(pDevInitStruct); 
    }  
    
    /*----------------------------------------*/
    /*------------- If RXNE event ------------*/
    if ((I2CFlagStatus & (uint16_t)CPAL_I2C_EVT_RXNE) != 0)
    { 
      return I2C_SLAVE_RXNE_Handle(pDevInitStruct); 
    }    
 #endif /* CPAL_I2C_IT_PROGMODEL */
    
    /*----------------------------------------*/
    /*------------- If STOPF event ------------*/
    if ((I2CFlagStatus & (uint16_t)CPAL_I2C_EVT_STOPF) != 0)
    { 
      return I2C_SLAVE_STOP_Handle(pDevInitStruct); 
    }
  }
#endif /* CPAL_I2C_SLAVE_MODE */
  
  return CPAL_PASS;
}


/**
  * @brief  Allows to handle errors occurred during initialization or communication 
  *         in order to recover the correct communication status or call specific 
  *         user functions.
  * @param  pDevInitStruct: Pointer to the peripheral configuration structure.
  * @retval CPAL_PASS. 
  */
uint32_t CPAL_I2C_ER_IRQHandler(CPAL_InitTypeDef* pDevInitStruct)
{  
  /* If AF detected in Slave mode transmitter */
  if ((pDevInitStruct->CPAL_Mode == CPAL_MODE_SLAVE) && (pDevInitStruct->pCPAL_TransferTx->wNumData == 0) &&
      ((pDevInitStruct->CPAL_State == CPAL_STATE_READY) || (pDevInitStruct->CPAL_State == CPAL_STATE_BUSY_TX)))
  {      
    /* Clear error flags that can be cleared by writing to SR register */
    __CPAL_I2C_HAL_CLEAR_ERROR((pDevInitStruct->CPAL_Dev));  
    
    /* If Interrupt Programming Model */
    if (pDevInitStruct->CPAL_ProgModel == CPAL_PROGMODEL_INTERRUPT)
    {  
#ifdef CPAL_I2C_IT_PROGMODEL  
      
      /* Disable EVENT Interrupt */
      __CPAL_I2C_HAL_DISABLE_EVTIT(pDevInitStruct->CPAL_Dev);
      
      CPAL_LOG("\n\rLOG : I2C Device EVT IT Disabled");
      
      /* Disable Buffer interrupt */
      __CPAL_I2C_HAL_DISABLE_BUFIT(pDevInitStruct->CPAL_Dev);
      
      CPAL_LOG("\n\rLOG : I2C Device BUFF IT Disabled"); 
      
      /* Wait until Busy flag is reset */ 
      __CPAL_I2C_TIMEOUT_SPINLOOP(!(__CPAL_I2C_HAL_GET_BUSY(pDevInitStruct->CPAL_Dev)), CPAL_I2C_TIMEOUT_BUSY);
      
      /* Update CPAL_State to CPAL_STATE_READY */
      pDevInitStruct->CPAL_State = CPAL_STATE_READY;  
      
      /* Call TX TC UserCallback */
      CPAL_I2C_TXTC_UserCallback(pDevInitStruct);     
#endif /* CPAL_I2C_IT_PROGMODEL */
    }   
  }  
  else
  {
    /* Read Error Register and affect to wCPAL_DevError */
    pDevInitStruct->wCPAL_DevError = __CPAL_I2C_HAL_GET_ERROR(pDevInitStruct->CPAL_Dev);
    
    /* Set Device state to CPAL_STATE_ERROR */
    pDevInitStruct->CPAL_State = CPAL_STATE_ERROR;
    
    CPAL_LOG("\n\r\n\rERROR <CPAL_I2C_ErrorHandler> : I2C Device Error"); 
    
    /* Clear error flags that can be cleared by writing to SR register */
    __CPAL_I2C_HAL_CLEAR_ERROR((pDevInitStruct->CPAL_Dev)); 
    
    /* If Bus error occurred ---------------------------------------------------*/
    if ((pDevInitStruct->wCPAL_DevError & CPAL_I2C_ERR_BERR) != 0)
    {      
      CPAL_LOG("\n\rERROR : I2C Device BERR"); 
      
      /* Generate I2C software reset in order to release SDA and SCL lines */
      __CPAL_I2C_HAL_SWRST(pDevInitStruct->CPAL_Dev);
      
      CPAL_LOG("\n\r I2C Device Software reset"); 
      
#ifdef USE_MULTIPLE_ERROR_CALLBACK
      /* Call Bus Error UserCallback */
      CPAL_I2C_BERR_UserCallback(pDevInitStruct->CPAL_Dev);    
#endif /* USE_MULTIPLE_ERROR_CALLBACK */
    }
    
    /* If Arbitration Loss error occurred --------------------------------------*/
    if ((pDevInitStruct->wCPAL_DevError & CPAL_I2C_ERR_ARLO) != 0)
    {
      CPAL_LOG("\n\rERROR : I2C Device ARLO"); 
      
      /* Generate I2C software reset in order to release SDA and SCL lines */    
      __CPAL_I2C_HAL_SWRST(pDevInitStruct->CPAL_Dev);
      
      CPAL_LOG("\n\r I2C Device Software reset"); 
      
#ifdef USE_MULTIPLE_ERROR_CALLBACK    
      /* Call Arbitration Lost UserCallback */ 
      CPAL_I2C_ARLO_UserCallback(pDevInitStruct->CPAL_Dev);  
#endif /* USE_MULTIPLE_ERROR_CALLBACK */    
    }
    
    /* If Overrun error occurred -----------------------------------------------*/
    if ((pDevInitStruct->wCPAL_DevError & CPAL_I2C_ERR_OVR) != 0)
    {
      CPAL_LOG("\n\rERROR : I2C Device OVR");
      
      /* No I2C software reset is performed here in order to allow user to get back
      the last data received correctly */
      
#ifdef USE_MULTIPLE_ERROR_CALLBACK    
      /* Call Overrun error UserCallback */
      CPAL_I2C_OVR_UserCallback(pDevInitStruct->CPAL_Dev);
#endif /* USE_MULTIPLE_ERROR_CALLBACK */    
    }
        
    /* If Acknowledge Failure error occurred -----------------------------------*/
    if ((pDevInitStruct->wCPAL_DevError & CPAL_I2C_ERR_AF) != 0)
    {        
      CPAL_LOG("\n\rERROR : I2C Device AF"); 
      
      /* No I2C software reset is performed here in order to allow user to recover 
      communication */
      
#ifdef USE_MULTIPLE_ERROR_CALLBACK    
      /* Call Acknowledge Failure UserCallback */
      CPAL_I2C_AF_UserCallback(pDevInitStruct->CPAL_Dev);  
#endif /* USE_MULTIPLE_ERROR_CALLBACK */   
      
    }   
        
    /* USE_SINGLE_ERROR_CALLBACK is defined in cpal_conf.h file */
#if defined(USE_SINGLE_ERROR_CALLBACK)  
    /* Call Error UserCallback */  
    CPAL_I2C_ERR_UserCallback(pDevInitStruct->CPAL_Dev, pDevInitStruct->wCPAL_DevError);
#endif /* USE_SINGLE_ERROR_CALLBACK */
  }
  
  return CPAL_PASS;
}


#ifdef CPAL_I2C_DMA_PROGMODEL
/**
  * @brief  Handle I2C DMA TX interrupt request when DMA programming Model is 
  *         used for data transmission. 
  * @param  pDevInitStruct: Pointer to the peripheral configuration structure.
  * @retval CPAL_PASS. 
  */
uint32_t CPAL_I2C_DMA_TX_IRQHandler(CPAL_InitTypeDef* pDevInitStruct)
{
  /* Reinitialize Timeout Value to default (no timeout initiated) */
  pDevInitStruct->wCPAL_Timeout = CPAL_I2C_TIMEOUT_DEFAULT; 
  
  CPAL_LOG("\n\r\n\rLOG <CPAL_I2C_DMA_TX_IRQHandler> : I2C Device TX DMA ");
  
  /*------------- If TC interrupt ------------*/
  if((__CPAL_I2C_HAL_GET_DMATX_TCIT(pDevInitStruct->CPAL_Dev)) != 0)
  {  
    CPAL_LOG("\n\rLOG : I2C Device TX Complete");
    
    /* Update remaining number of data */
    pDevInitStruct->pCPAL_TransferTx->wNumData = 0;
    
    /* Call DMA TX TC UserCallback */
    CPAL_I2C_DMATXTC_UserCallback(pDevInitStruct);
    
    /* If DMA Normal mode */
    if ((pDevInitStruct->wCPAL_Options & CPAL_OPT_DMATX_CIRCULAR) == 0)
    {           
      /* If Master Mode selected */
      if (pDevInitStruct->CPAL_Mode == CPAL_MODE_MASTER) 
      {
 #ifdef CPAL_I2C_MASTER_MODE         
        /* Disable DMA Request */
        __CPAL_I2C_HAL_DISABLE_DMAREQ(pDevInitStruct->CPAL_Dev); 
        
        /* Wait until BTF flag is set */ 
        __CPAL_I2C_TIMEOUT_SPINLOOP(__CPAL_I2C_HAL_GET_BTF(pDevInitStruct->CPAL_Dev), CPAL_I2C_TIMEOUT_BTF);
        
        /* No Stop Condition Generation option bit not selected */   
        if ((pDevInitStruct->wCPAL_Options & CPAL_OPT_I2C_NOSTOP) == 0)
        {          
          /* Generate Stop Condition */
          __CPAL_I2C_HAL_STOP(pDevInitStruct->CPAL_Dev);
          
          /* Wait until Busy flag is reset */         
          __CPAL_I2C_TIMEOUT_SPINLOOP(!(__CPAL_I2C_HAL_GET_BUSY(pDevInitStruct->CPAL_Dev)), CPAL_I2C_TIMEOUT_BUSY);
        }
        
        /* Disable DMA Channel */                 
        __CPAL_I2C_HAL_DISABLE_DMATX(pDevInitStruct->CPAL_Dev);        
        
        /* Disable EVENT Interrupt */
        __CPAL_I2C_HAL_DISABLE_EVTIT(pDevInitStruct->CPAL_Dev);   
        
        CPAL_LOG("\n\rLOG : I2C Device Master TX DMA Disabled");
        
        /* Update CPAL_State to CPAL_STATE_READY */
        pDevInitStruct->CPAL_State = CPAL_STATE_READY; 
      }
 #endif /* CPAL_I2C_MASTER_MODE */  
      else
      {
 #ifdef CPAL_I2C_SLAVE_MODE    	      
        /* Disable DMA Request and Channel */
        __CPAL_I2C_HAL_DISABLE_DMAREQ(pDevInitStruct->CPAL_Dev);      
        __CPAL_I2C_HAL_DISABLE_DMATX(pDevInitStruct->CPAL_Dev);      
        
        /* Disable EVENT Interrupt */
        __CPAL_I2C_HAL_DISABLE_EVTIT(pDevInitStruct->CPAL_Dev);
        
        /* Wait until Busy flag is reset */ 
        __CPAL_I2C_TIMEOUT_SPINLOOP(!(__CPAL_I2C_HAL_GET_BUSY(pDevInitStruct->CPAL_Dev)), CPAL_I2C_TIMEOUT_BUSY);
        
        CPAL_LOG("\n\rLOG : I2C Device Slave TX DMA Disabled");
        
        /* Update CPAL_State to CPAL_STATE_READY */
        pDevInitStruct->CPAL_State = CPAL_STATE_READY; 
 #endif /* CPAL_I2C_SLAVE_MODE */       
      } 
    } 
    /* Call TX TC UserCallback */
    CPAL_I2C_TXTC_UserCallback(pDevInitStruct);
  }
  /*------------- If HT interrupt ------------*/
  else if ((__CPAL_I2C_HAL_GET_DMATX_HTIT(pDevInitStruct->CPAL_Dev)) != 0)
  {         
    CPAL_LOG("\n\rLOG : I2C Device TX DMA Half Transfer ");
    
    /* Call DMA TX HT UserCallback */
    CPAL_I2C_DMATXHT_UserCallback(pDevInitStruct);
  }  
  /*------------- If TE interrupt ------------*/
  else if ((__CPAL_I2C_HAL_GET_DMATX_TEIT(pDevInitStruct->CPAL_Dev)) != 0)
  { 
    CPAL_LOG("\n\rERROR : I2C Device TX DMA Transfer Error ");
    
    /* Update CPAL_State to CPAL_STATE_ERROR */
    pDevInitStruct->CPAL_State = CPAL_STATE_ERROR; 
    
    /* Update remaining number of data */
    pDevInitStruct->pCPAL_TransferTx->wNumData = __CPAL_I2C_HAL_DMATX_GET_CNDT(pDevInitStruct->CPAL_Dev);
    
    /* Call DMA TX TE UserCallback */
    CPAL_I2C_DMATXTE_UserCallback(pDevInitStruct); 
  }  
  
   /* Clear DMA Interrupt Flag */
    __CPAL_I2C_HAL_CLEAR_DMATX_IT(pDevInitStruct->CPAL_Dev);
  
  return CPAL_PASS;
}


/**
  * @brief  Handle I2C DMA RX interrupt request when DMA programming Model is 
  *         used for data reception.  
  * @param  pDevInitStruct: Pointer to the peripheral configuration structure.
  * @retval CPAL_PASS. 
  */
uint32_t CPAL_I2C_DMA_RX_IRQHandler(CPAL_InitTypeDef* pDevInitStruct)
{
  /* Reinitialize Timeout Value to default (no timeout initiated) */
  pDevInitStruct->wCPAL_Timeout = CPAL_I2C_TIMEOUT_DEFAULT; 
  
  CPAL_LOG("\n\r\n\rLOG <CPAL_I2C_DMA_RX_IRQHandler> : I2C Device RX DMA ");
  
  /*------------- If TC interrupt ------------*/
  if ((__CPAL_I2C_HAL_GET_DMARX_TCIT(pDevInitStruct->CPAL_Dev)) != 0)
  {   
    CPAL_LOG("\n\rLOG : I2C Device RX Complete");
    
    /* Update remaining number of data */
    pDevInitStruct->pCPAL_TransferRx->wNumData = 0;
       
    /* Call DMA RX TC UserCallback */
    CPAL_I2C_DMARXTC_UserCallback(pDevInitStruct);
    
    /* If DMA Normal model */
    if ((pDevInitStruct->wCPAL_Options & CPAL_OPT_DMARX_CIRCULAR) == 0)
    {      
      /* If Master Mode selected */
      if ((pDevInitStruct->CPAL_Mode == CPAL_MODE_MASTER))
      {         
 #ifdef CPAL_I2C_MASTER_MODE 
        /* Generate Stop Condition */
        __CPAL_I2C_HAL_STOP(pDevInitStruct->CPAL_Dev);
        
        /* Disable DMA Request and Channel */          
        __CPAL_I2C_HAL_DISABLE_DMAREQ(pDevInitStruct->CPAL_Dev);
        
        /* Wait until Busy flag is reset */ 
        __CPAL_I2C_TIMEOUT_SPINLOOP(!(__CPAL_I2C_HAL_GET_BUSY(pDevInitStruct->CPAL_Dev)), CPAL_I2C_TIMEOUT_BUSY);
        
        /* Disable DMA Channel */
        __CPAL_I2C_HAL_DISABLE_DMARX(pDevInitStruct->CPAL_Dev);
        
        /* Disable EVENT Interrupt */
        __CPAL_I2C_HAL_DISABLE_EVTIT(pDevInitStruct->CPAL_Dev);
        
        /* Disable DMA automatic NACK generation */
        __CPAL_I2C_HAL_DISABLE_LAST(pDevInitStruct->CPAL_Dev);
        
        CPAL_LOG("\n\rLOG : I2C Device Master RX DMA Disabled");
        
        /* Update CPAL_State to CPAL_STATE_READY */
        pDevInitStruct->CPAL_State = CPAL_STATE_READY; 
        
        /* Call RX TC UserCallback */
        CPAL_I2C_RXTC_UserCallback(pDevInitStruct);
      
 #endif /* CPAL_I2C_MASTER_MODE */        
      }      
      else if ((pDevInitStruct->CPAL_Mode == CPAL_MODE_SLAVE) && ((pDevInitStruct->wCPAL_Options & CPAL_OPT_I2C_NOSTOP) != 0))
      {     
 #ifdef CPAL_I2C_SLAVE_MODE     
        /* Disable DMA Request */          
        __CPAL_I2C_HAL_DISABLE_DMAREQ(pDevInitStruct->CPAL_Dev);          
        
        /* Disable DMA Channel */
        __CPAL_I2C_HAL_DISABLE_DMARX(pDevInitStruct->CPAL_Dev);
        
        /* Disable EVENT Interrupt */
        __CPAL_I2C_HAL_DISABLE_EVTIT(pDevInitStruct->CPAL_Dev);
        
        CPAL_LOG("\n\rLOG : I2C Device Slave RX DMA Disabled");
        
        /* Update CPAL_State to CPAL_STATE_READY */
        pDevInitStruct->CPAL_State = CPAL_STATE_READY;
        
        /* Call RX TC UserCallback */
        CPAL_I2C_RXTC_UserCallback(pDevInitStruct);
 #endif /* CPAL_I2C_SLAVE_MODE */        
      }
    }
    else
    {
      /* Call RX TC UserCallback */
      CPAL_I2C_RXTC_UserCallback(pDevInitStruct);
    }
  }  
  /*------------- If HT interrupt ------------*/
  else if ((__CPAL_I2C_HAL_GET_DMARX_HTIT(pDevInitStruct->CPAL_Dev)) != 0)
  {   
    CPAL_LOG("\n\rLOG : I2C Device RX DMA Half Transfer");
    
    /* Call DMA RX HT UserCallback */
    CPAL_I2C_DMARXHT_UserCallback(pDevInitStruct);
  }  
  /*------------- If TE interrupt ------------*/
  else if ((__CPAL_I2C_HAL_GET_DMARX_TEIT(pDevInitStruct->CPAL_Dev)) != 0)
  {   
    CPAL_LOG("\n\rERROR : I2C Device RX DMA Transfer Error ");
    
    /* Update CPAL_State to CPAL_STATE_ERROR */
    pDevInitStruct->CPAL_State = CPAL_STATE_ERROR; 
    
    /* Update remaining number of data */
    pDevInitStruct->pCPAL_TransferRx->wNumData = __CPAL_I2C_HAL_DMARX_GET_CNDT(pDevInitStruct->CPAL_Dev);
    
    /* Call DMA RX TE UserCallback */
    CPAL_I2C_DMARXTE_UserCallback(pDevInitStruct); 
  }
  
  /* Clear DMA Interrupt Flag */
  __CPAL_I2C_HAL_CLEAR_DMARX_IT(pDevInitStruct->CPAL_Dev);
  
  return CPAL_PASS;
}
#endif /* CPAL_I2C_DMA_PROGMODEL */


/*================== CPAL_I2C_Timeout_Function ==================*/

/**
  * @brief  This function Manages I2C Timeouts when waiting for specific events.
  * @param  None
  * @retval CPAL_PASS or CPAL_FAIL. 
  */
void CPAL_I2C_TIMEOUT_Manager(void)
{
  uint32_t index = 0;
  
  /* Manage I2C timeouts conditions */
  for (index = 0; index < CPAL_I2C_DEV_NUM; index ++)
  {
    if (I2C_DevStructures[index] != pNULL)
    {
      /* If Timeout occurred  */
      if (I2C_DevStructures[index]->wCPAL_Timeout == CPAL_I2C_TIMEOUT_DETECTED)
      {
        /* Reinitialize Timeout Value */
        I2C_DevStructures[index]->wCPAL_Timeout = CPAL_I2C_TIMEOUT_DEFAULT;
        
        /* update CPAL_State to CPAL_STATE_ERROR */
        I2C_DevStructures[index]->CPAL_State = CPAL_STATE_ERROR;
        
        /* In case of Device Error Timeout_Callback should not be called */
        if (I2C_DevStructures[index]->wCPAL_DevError == CPAL_I2C_ERR_NONE)
        {        
          /* update wCPAL_DevError to CPAL_I2C_ERR_TIMEOUT */
          I2C_DevStructures[index]->wCPAL_DevError = CPAL_I2C_ERR_TIMEOUT;
          
          CPAL_LOG("\n\r\n\rLOG <CPAL_I2C_TIMEOUT_Manager> : I2C Device Timeout Error");
          
          /* Call CPAL_TIMEOUT_UserCallback */
          CPAL_TIMEOUT_UserCallback(I2C_DevStructures[index]);
        }              
      }     
       /* If Timeout is triggered (wCPAL_Timeout != CPAL_I2C_TIMEOUT_DEFAULT)*/
      else if (I2C_DevStructures[index]->wCPAL_Timeout != CPAL_I2C_TIMEOUT_DEFAULT)
      {
        /* Decrement the timeout value */
        I2C_DevStructures[index]->wCPAL_Timeout--;
      } 
    }
  }  
}


/**
  * @brief  This function Manages I2C Timeouts when Timeout occurred.
  * @param  pDevInitStruct: Pointer to the peripheral configuration structure.
  * @retval CPAL_PASS or CPAL_FAIL. 
  */
uint32_t CPAL_I2C_Timeout (CPAL_InitTypeDef* pDevInitStruct)
{
  /* Reinitialize Timeout Value */
  pDevInitStruct->wCPAL_Timeout = CPAL_I2C_TIMEOUT_DEFAULT;
  
  /* update CPAL_State to CPAL_STATE_ERROR */
  pDevInitStruct->CPAL_State = CPAL_STATE_ERROR;
  
  /* update wCPAL_DevError to CPAL_I2C_ERR_TIMEOUT */
  pDevInitStruct->wCPAL_DevError = CPAL_I2C_ERR_TIMEOUT;
  
  /* Call Timeout Callback and quit current function */
  return (CPAL_TIMEOUT_UserCallback(pDevInitStruct));
}


/*================== CPAL_I2C_Event_Handler ==================*/

#ifdef CPAL_I2C_MASTER_MODE 
/**
  * @brief  Handles Master Start condition (SB) interrupt event.
  * @param  pDevInitStruct: Pointer to the peripheral configuration structure.
  * @retval CPAL_PASS or CPAL_FAIL. 
  */
static uint32_t I2C_MASTER_START_Handle(CPAL_InitTypeDef* pDevInitStruct)
{
  #ifdef CPAL_I2C_10BIT_ADDR_MODE  
  /* Declare local variable that contains Address Header */
  uint8_t I2CHeaderAddress = 0x00;
  #endif /* CPAL_I2C_10BIT_ADDR_MODE */

  /* Reinitialize Timeout Value */
  pDevInitStruct->wCPAL_Timeout = CPAL_I2C_TIMEOUT_DEFAULT;
  
  CPAL_LOG("\n\r\n\rLOG <I2C_EV_IRQHandler> : I2C Device Master IT"); 
  
  CPAL_LOG("\n\rLOG : I2C Device Start Acknowledged"); 
  
  /* If 7 bit Addressing Mode selected */
  if (pDevInitStruct->pCPAL_I2C_Struct->I2C_AcknowledgedAddress == I2C_AcknowledgedAddress_7bit)
  {        
    CPAL_LOG("\n\rLOG : I2C Device 7bit Address");
    
    /* Send Address */
    /* If Master run as receiver */
    if (pDevInitStruct->CPAL_State == CPAL_STATE_READY_RX)
    {
      /* Send Slave address with bit0 set for read */
      __CPAL_I2C_HAL_SEND((pDevInitStruct->CPAL_Dev), (uint8_t)((pDevInitStruct->pCPAL_TransferRx->wAddr1) | I2C_OAR1_ADD0));  
      
      /* Update CPAL_State to CPAL_STATE_BUSY */
      pDevInitStruct->CPAL_State = CPAL_STATE_BUSY_RX; 
      
      CPAL_LOG("\n\rLOG : I2C Device Busy RX");
    }    
    /* If Master run as Transmitter */
    else
    {
      /* Send Slave address with bit0 reset for write */
      __CPAL_I2C_HAL_SEND((pDevInitStruct->CPAL_Dev), (uint8_t)((pDevInitStruct->pCPAL_TransferTx->wAddr1) & (~I2C_OAR1_ADD0)));        
      
      /* Update CPAL_State to CPAL_STATE_BUSY */
      pDevInitStruct->CPAL_State = CPAL_STATE_BUSY_TX; 
      
      CPAL_LOG("\n\rLOG : I2C Device Busy TX");
    }
    
    CPAL_LOG("\n\rLOG : I2C Device Target Address Sent");
    
    /* Initialize Timeout value */
    pDevInitStruct->wCPAL_Timeout = CPAL_I2C_TIMEOUT_MIN + CPAL_I2C_TIMEOUT_ADDR;             
  }  
 #ifdef CPAL_I2C_10BIT_ADDR_MODE  
  /* If 10 bit Addressing Mode selected */
  else
  {  
    CPAL_LOG("\n\rLOG : I2C Device 10bit Address");
    								      
    /* If Master run as receiver */
    if (pDevInitStruct->CPAL_State == CPAL_STATE_READY_RX)
    {
      /* Calculate RX Header Address  */ 
      I2CHeaderAddress = ((((pDevInitStruct->pCPAL_TransferRx->wAddr1) & 0xFF00) >>7) | 0xF0);
    }    
    /* If Master run as Transmitter */
    else if (pDevInitStruct->CPAL_State == CPAL_STATE_READY_TX)
    {
      /* Calculate TX Header Address */ 
      I2CHeaderAddress = ((((pDevInitStruct->pCPAL_TransferTx->wAddr1) & 0xFF00) >>7) | 0xF0); 
    }      
    /* If Master run as Receiver */
    else if (pDevInitStruct->CPAL_State == CPAL_STATE_BUSY_RX)
    {
      /* Calculate RX Header Address */ 
      I2CHeaderAddress = ((((pDevInitStruct->pCPAL_TransferRx->wAddr1) & 0xFF00) >>7) | 0xF1);       
    }       
    
     /* Send Header */ 
    __CPAL_I2C_HAL_SEND((pDevInitStruct->CPAL_Dev), I2CHeaderAddress); 
    
    CPAL_LOG("\n\rLOG : I2C Device Target Header Sent "); 
    
    /* Initialize Timeout value */
    pDevInitStruct->wCPAL_Timeout = CPAL_I2C_TIMEOUT_MIN + CPAL_I2C_TIMEOUT_ADD10;                 
  }   
 #endif /* CPAL_I2C_10BIT_ADDR_MODE */
  
#if defined (STM32F10X_LD) || defined (STM32F10X_LD_VL) || defined (STM32F10X_MD) || defined (STM32F10X_MD_VL)\
 || defined (STM32F10X_HD) || defined (STM32F10X_HD_VL) || defined (STM32F10X_XL) || defined (STM32F10X_CL)
 #ifdef CPAL_I2C_CLOSECOM_METHOD2  
  if ((pDevInitStruct->CPAL_ProgModel == CPAL_PROGMODEL_INTERRUPT) &&(pDevInitStruct->CPAL_State == CPAL_STATE_BUSY_RX) 
           && (pDevInitStruct->pCPAL_TransferRx->wNumData == 2))
  {
    /* Activate POS bit */
    __CPAL_I2C_HAL_ENABLE_POS(pDevInitStruct->CPAL_Dev);
  }
 #endif /* CPAL_I2C_CLOSECOM_METHOD2 */
#endif  
  return CPAL_PASS;
}


/**
  * @brief  Handles Master address matched (ADDR) interrupt event. 
  * @param  pDevInitStruct: Pointer to the peripheral configuration structure.
  * @retval CPAL_PASS or CPAL_FAIL. 
  */
static uint32_t I2C_MASTER_ADDR_Handle(CPAL_InitTypeDef* pDevInitStruct)
{     
  /* Initialize Timeout value (1 ms for each data to be sent/received) */
  if (pDevInitStruct->CPAL_ProgModel != CPAL_PROGMODEL_DMA)
  {
    /* Reinitialize Timeout Value to default (no timeout initiated) */
    pDevInitStruct->wCPAL_Timeout = CPAL_I2C_TIMEOUT_DEFAULT;                
  }  
  else if (pDevInitStruct->CPAL_State == CPAL_STATE_BUSY_TX)
  {
    /* Set 1ms timeout for each data transfer in case of DMA Tx mode */
    pDevInitStruct->wCPAL_Timeout = CPAL_I2C_TIMEOUT_MIN + pDevInitStruct->pCPAL_TransferTx->wNumData;
  }  
  else if (pDevInitStruct->CPAL_State == CPAL_STATE_BUSY_RX)
  {
    /* Set 1ms timeout for each data transfer in case of DMA Rx mode */ 
    pDevInitStruct->wCPAL_Timeout = CPAL_I2C_TIMEOUT_MIN + pDevInitStruct->pCPAL_TransferRx->wNumData;
  }  
  else
  {
    /* Reinitialize Timeout Value to default (no timeout initiated) */
    pDevInitStruct->wCPAL_Timeout = CPAL_I2C_TIMEOUT_DEFAULT;        
  }
  
  if ((pDevInitStruct->CPAL_State == CPAL_STATE_BUSY_RX) && (pDevInitStruct->CPAL_ProgModel == CPAL_PROGMODEL_INTERRUPT) && (pDevInitStruct->pCPAL_TransferRx->wNumData == 0))
  {    
    /* Program STOP bit then clear ADDR flag */
    __CPAL_I2C_HAL_STOP(pDevInitStruct->CPAL_Dev); 
    __CPAL_I2C_HAL_CLEAR_ADDR(pDevInitStruct->CPAL_Dev);
    
    /* Update CPAL_State to CPAL_STATE_READY */
    pDevInitStruct->CPAL_State = CPAL_STATE_READY; 
  }
  else
  {
    if ((pDevInitStruct->CPAL_State == CPAL_STATE_BUSY_RX) && (pDevInitStruct->CPAL_ProgModel == CPAL_PROGMODEL_INTERRUPT))
    {       
      /* Switch Programing Mode Enable DMA or IT Buffer */
      CPAL_I2C_Enable_DMA_IT(pDevInitStruct, CPAL_DIRECTION_RX);
    }
  
#if defined (STM32L1XX_MD) || defined (STM32L1XX_HD) || defined (STM32F2XX) || defined (STM32F4XX) 
    /* If CPAL_State is CPAL_STATE_BUSY_RX and receiving one byte */  
    if ((pDevInitStruct->CPAL_State == CPAL_STATE_BUSY_RX) && (pDevInitStruct->pCPAL_TransferRx->wNumData == 1))
    { 
      /* Disable Acknowledge */
      __CPAL_I2C_HAL_DISABLE_ACK(pDevInitStruct->CPAL_Dev);
      
      /* Clear ADDR Flag by reading SR1 then SR2 */
      __CPAL_I2C_HAL_CLEAR_ADDR(pDevInitStruct->CPAL_Dev);   
      
      /* Program Generation of Stop Condition */
      __CPAL_I2C_HAL_STOP(pDevInitStruct->CPAL_Dev);
    }  
    else if ((pDevInitStruct->CPAL_ProgModel == CPAL_PROGMODEL_INTERRUPT) &&(pDevInitStruct->CPAL_State == CPAL_STATE_BUSY_RX) && (pDevInitStruct->pCPAL_TransferRx->wNumData == 2))
    {
      /* Disable Acknowledge */
      __CPAL_I2C_HAL_DISABLE_ACK(pDevInitStruct->CPAL_Dev);
      
      /* Enable Pos */
      __CPAL_I2C_HAL_ENABLE_POS(pDevInitStruct->CPAL_Dev);
      
      /* Clear ADDR Flag by reading SR1 then SR2 */
      __CPAL_I2C_HAL_CLEAR_ADDR(pDevInitStruct->CPAL_Dev);      
    }
    else
    {
      /* Clear ADDR Flag by reading SR1 then SR2 */
      __CPAL_I2C_HAL_CLEAR_ADDR(pDevInitStruct->CPAL_Dev);
    }  
#else  
  
 #ifdef CPAL_I2C_CLOSECOM_METHOD1
    __CPAL_I2C_HAL_CLEAR_ADDR(pDevInitStruct->CPAL_Dev); 
 #endif /* CPAL_I2C_CLOSECOM_METHOD1 */
  
    /* If CPAL_State is CPAL_STATE_BUSY_RX and receiving one byte */  
    if ((pDevInitStruct->CPAL_State == CPAL_STATE_BUSY_RX) && (pDevInitStruct->pCPAL_TransferRx->wNumData == 1))
    { 
      /* Disable Acknowledge */
      __CPAL_I2C_HAL_DISABLE_ACK(pDevInitStruct->CPAL_Dev);
      
 #ifdef CPAL_I2C_CLOSECOM_METHOD2
    
  #ifdef USE_CPAL_CRITICAL_CALLBACK
      /* Call Critical section Callback */
      CPAL_EnterCriticalSection_UserCallback();  
  #endif /* USE_CPAL_CRITICAL_CALLBACK */
    
      /* Clear ADDR Flag by reading SR1 then SR2 */
      __CPAL_I2C_HAL_CLEAR_ADDR(pDevInitStruct->CPAL_Dev); 
      
      /* Program Generation of Stop Condition */
      __CPAL_I2C_HAL_STOP(pDevInitStruct->CPAL_Dev);
      
  #ifdef USE_CPAL_CRITICAL_CALLBACK
      /* Call Critical section Callback */
      CPAL_ExitCriticalSection_UserCallback();
  #endif /* USE_CPAL_CRITICAL_CALLBACK */
 #else
      /* Program Generation of Stop Condition */
      __CPAL_I2C_HAL_STOP(pDevInitStruct->CPAL_Dev);
 #endif /* CPAL_I2C_CLOSECOM_METHOD2 */
    }
 #ifdef CPAL_I2C_CLOSECOM_METHOD2  
    else if ((pDevInitStruct->CPAL_ProgModel == CPAL_PROGMODEL_INTERRUPT) &&(pDevInitStruct->CPAL_State == CPAL_STATE_BUSY_RX) 
             && (pDevInitStruct->pCPAL_TransferRx->wNumData == 2))
    {        
      /* Clear ADDR Flag by reading SR1 then SR2 */
      __CPAL_I2C_HAL_CLEAR_ADDR(pDevInitStruct->CPAL_Dev); 
      
      /* Disable Acknowledge */
      __CPAL_I2C_HAL_DISABLE_ACK(pDevInitStruct->CPAL_Dev);
    }
    else
    {
      /* Clear ADDR Flag by reading SR1 then SR2 */
      __CPAL_I2C_HAL_CLEAR_ADDR(pDevInitStruct->CPAL_Dev);
    }
 #endif /* CPAL_I2C_CLOSECOM_METHOD2 */ 
#endif /* STM32L1XX_MD || STM32L1XX_HD || STM32F2XX || STM32F4XX */
  
#ifdef CPAL_I2C_10BIT_ADDR_MODE
    /* If CPAL_State is not CPAL_STATE_BUSY */
    if (((pDevInitStruct->CPAL_State & (CPAL_STATE_READY_TX | CPAL_STATE_READY_RX)) != 0) 
        && (pDevInitStruct->pCPAL_I2C_Struct->I2C_AcknowledgedAddress == I2C_AcknowledgedAddress_10bit))
    {        
      /* If Master run as receiver */
      if (pDevInitStruct->CPAL_State == CPAL_STATE_READY_RX)
      {
        /* Update CPAL_State to CPAL_STATE_BUSY_RX */
        pDevInitStruct->CPAL_State = CPAL_STATE_BUSY_RX; 
        
        CPAL_LOG("\n\rLOG : I2C Device Busy RX");
        
        /* Generate Repeated start bit  */
        __CPAL_I2C_HAL_START(pDevInitStruct->CPAL_Dev);
        
        /* Initialize Timeout value */
        pDevInitStruct->wCPAL_Timeout = CPAL_I2C_TIMEOUT_MIN + CPAL_I2C_TIMEOUT_SB;          
      }
      
      /* If Master run as Transmitter */
      if  (pDevInitStruct->CPAL_State == CPAL_STATE_READY_TX)
      {
        /* Update CPAL_State to CPAL_STATE_BUSY_TX */
        pDevInitStruct->CPAL_State = CPAL_STATE_BUSY_TX; 
        
        CPAL_LOG("\n\rLOG : I2C Device Busy TX");
      }
    }
    else if ((pDevInitStruct->wCPAL_Options & CPAL_OPT_NO_MEM_ADDR) == 0)      
#endif /* CPAL_I2C_10BIT_ADDR_MODE */
    
#ifndef CPAL_I2C_10BIT_ADDR_MODE
      /* If CPAL_OPT_NO_MEM_ADDR is not enabled */
      if ((pDevInitStruct->wCPAL_Options & CPAL_OPT_NO_MEM_ADDR) == 0)
#endif  /* CPAL_I2C_10BIT_ADDR_MODE */
      {
        /* If CPAL_State is CPAL_STATE_BUSY_TX */  
        if (pDevInitStruct->CPAL_State == CPAL_STATE_BUSY_TX)
        {         
          /* If 8 Bit register mode */
          if ((pDevInitStruct->wCPAL_Options & CPAL_OPT_16BIT_REG) == 0)
          {
            /* Send Register Address */
            __CPAL_I2C_HAL_SEND((pDevInitStruct->CPAL_Dev), (uint8_t)((pDevInitStruct->pCPAL_TransferTx->wAddr2)& 0x00FF)); 
            
            /* Wait until TXE flag is set */ 
            __CPAL_I2C_TIMEOUT_SPINLOOP(__CPAL_I2C_HAL_GET_TXE(pDevInitStruct->CPAL_Dev), CPAL_I2C_TIMEOUT_TXE);
          }          
#ifdef CPAL_16BIT_REG_OPTION
          /* If 16 Bit register mode */
          else
          {
            /* Send MSB Register Address */
            __CPAL_I2C_HAL_SEND((pDevInitStruct->CPAL_Dev), (uint8_t)(((pDevInitStruct->pCPAL_TransferTx->wAddr2)& 0xFF00) >>8));  
            
            /* Wait until TXE flag is set */ 
            __CPAL_I2C_TIMEOUT_SPINLOOP(__CPAL_I2C_HAL_GET_TXE(pDevInitStruct->CPAL_Dev), CPAL_I2C_TIMEOUT_TXE);
            
            /* Send LSB Register Address */
            __CPAL_I2C_HAL_SEND((pDevInitStruct->CPAL_Dev), (uint8_t)((pDevInitStruct->pCPAL_TransferTx->wAddr2)& 0x00FF));  
            
            /* Wait until TXE flag is set */ 
            __CPAL_I2C_TIMEOUT_SPINLOOP(__CPAL_I2C_HAL_GET_TXE(pDevInitStruct->CPAL_Dev), CPAL_I2C_TIMEOUT_TXE);
          }     
#endif /* CPAL_16BIT_REG_OPTION */
        }  
        
        /* Switch Programing Mode Enable DMA or IT Buffer */
        CPAL_I2C_Enable_DMA_IT(pDevInitStruct, CPAL_DIRECTION_TXRX);   
      }      
  }
  return CPAL_PASS;
}


 #ifdef CPAL_I2C_10BIT_ADDR_MODE
/**
  * @brief  Handles Master 10bit address matched (ADD10) interrupt event.
  * @param  pDevInitStruct: Pointer to the peripheral configuration structure.
  * @retval CPAL_PASS or CPAL_FAIL. 
  */
static uint32_t I2C_MASTER_ADD10_Handle(CPAL_InitTypeDef* pDevInitStruct)
{ 
  /* Reinitialize Timeout Value */
  pDevInitStruct->wCPAL_Timeout = CPAL_I2C_TIMEOUT_DEFAULT;
  
  CPAL_LOG("\n\r\n\rLOG <I2C_EV_IRQHandler> : I2C Device Master IT");
  
  CPAL_LOG("\n\rLOG : I2C Device Header Address Acknowledged");
  
  /* Send Address */
  /* If Master run as receiver */
  if (pDevInitStruct->CPAL_State == CPAL_STATE_READY_RX)
  {
    /* Send Slave Address */
    __CPAL_I2C_HAL_SEND((pDevInitStruct->CPAL_Dev), (uint8_t)(pDevInitStruct->pCPAL_TransferRx->wAddr1));  
  }  
  /* If Master run as Transmitter */
  else if (pDevInitStruct->CPAL_State == CPAL_STATE_READY_TX)
  {
    /* Send Slave Address */
    __CPAL_I2C_HAL_SEND((pDevInitStruct->CPAL_Dev), (uint8_t)(pDevInitStruct->pCPAL_TransferTx->wAddr1));        
  }
  
  CPAL_LOG("\n\rLOG : I2C Device Target Address Sent");  
  
  /* Initialize Timeout value */
  pDevInitStruct->wCPAL_Timeout = CPAL_I2C_TIMEOUT_MIN + CPAL_I2C_TIMEOUT_ADDR; 
  
  return CPAL_PASS;
}
 #endif /* CPAL_I2C_10BIT_ADDR_MODE */


 #ifdef CPAL_I2C_IT_PROGMODEL
/**
  * @brief  Handles Master transmission (TXE) interrupt event.
  * @param  pDevInitStruct: Pointer to the peripheral configuration structure.
  * @retval CPAL_PASS or CPAL_FAIL. 
  */
static uint32_t I2C_MASTER_TXE_Handle(CPAL_InitTypeDef* pDevInitStruct)
{ 
  /* If Interrupt Programming Model selected */
  if (pDevInitStruct->CPAL_ProgModel == CPAL_PROGMODEL_INTERRUPT)
  {                   
    /* If Buffer end */
    if (pDevInitStruct->pCPAL_TransferTx->wNumData != 0)
    {   
      /* Call TX UserCallback */
      CPAL_I2C_TX_UserCallback(pDevInitStruct);
      
      /* Write Byte */
      __CPAL_I2C_HAL_SEND((pDevInitStruct->CPAL_Dev), (*(pDevInitStruct->pCPAL_TransferTx->pbBuffer))); 
      
      /* Decrement remaining number of data */
      pDevInitStruct->pCPAL_TransferTx->wNumData--;
      
      /* If Buffer end */
      if (pDevInitStruct->pCPAL_TransferTx->wNumData != 0)
      {  
        /* Point to next data */
        pDevInitStruct->pCPAL_TransferTx->pbBuffer++;      
      }
    }    
    else 
    {
      /* No Stop Condition Generation option bit not selected */ 
      if ((pDevInitStruct->wCPAL_Options & CPAL_OPT_I2C_NOSTOP) == 0)
      {      
        /* Generate Stop Condition */
        __CPAL_I2C_HAL_STOP(pDevInitStruct->CPAL_Dev);
        
        CPAL_LOG("\n\r\n\rLOG <I2C_EV_IRQHandler> : I2C Device Master IT");
        
        CPAL_LOG("\n\rLOG : I2C Device Generates Stop");        
      }
      
      CPAL_LOG("\n\rLOG : I2C Device TX Complete");
      
      /* Disable EVENT Interrupt */
      __CPAL_I2C_HAL_DISABLE_EVTIT(pDevInitStruct->CPAL_Dev);
      
      CPAL_LOG("\n\rLOG : I2C Device TX EVT IT Disabled");
      
      /* Disable Buffer interrupt */
      __CPAL_I2C_HAL_DISABLE_BUFIT(pDevInitStruct->CPAL_Dev);
      
      CPAL_LOG("\n\rLOG : I2C Device TX BUFF IT Disabled");
      
      /* No Stop Condition Generation option bit not selected */ 
      if ((pDevInitStruct->wCPAL_Options & CPAL_OPT_I2C_NOSTOP) == 0)
      { 
        /* Wait until BTF and TXE flags are reset */ 
        __CPAL_I2C_TIMEOUT_SPINLOOP(!(__CPAL_I2C_HAL_GET_EVENT(pDevInitStruct->CPAL_Dev) & (I2C_SR1_BTF | I2C_SR1_TXE )), CPAL_I2C_TIMEOUT_BUSY);
      }
      else
      {
        /* Wait until BTF flags is reset */ 
        __CPAL_I2C_TIMEOUT_SPINLOOP(!(__CPAL_I2C_HAL_GET_EVENT(pDevInitStruct->CPAL_Dev) & I2C_SR1_TXE ), CPAL_I2C_TIMEOUT_BUSY);
        
      }
      
      /* Update CPAL_State to CPAL_STATE_READY */
      pDevInitStruct->CPAL_State = CPAL_STATE_READY; 
      
      /* Call TX Transfer complete Callback */
      CPAL_I2C_TXTC_UserCallback(pDevInitStruct);       
    }        
  }
  
  return CPAL_PASS;
}
 #endif /* CPAL_I2C_IT_PROGMODEL */

 #if defined (CPAL_I2C_IT_PROGMODEL) || defined (CPAL_I2C_DMA_1BYTE_CASE)
/**
  * @brief  Handles Master reception (RXNE flag) interrupt event.
  * @param  pDevInitStruct: Pointer to the peripheral configuration structure.
  * @retval CPAL_PASS or CPAL_FAIL. 
  */
static uint32_t I2C_MASTER_RXNE_Handle(CPAL_InitTypeDef* pDevInitStruct)
{  
  /* If Interrupt Programming Model selected */
  if (pDevInitStruct->CPAL_ProgModel == CPAL_PROGMODEL_INTERRUPT)
  {  
#if defined (STM32L1XX_MD) || defined (STM32L1XX_HD) || defined (STM32F2XX) || defined (STM32F4XX)
    /* if less than 3 bytes remaining for reception */ 
    if (pDevInitStruct->pCPAL_TransferRx->wNumData <= 3)
    {  
      /* One byte */
      if (pDevInitStruct->pCPAL_TransferRx->wNumData == 1)
      {              
        /* Read Byte */
        *(pDevInitStruct->pCPAL_TransferRx->pbBuffer) = __CPAL_I2C_HAL_RECEIVE(pDevInitStruct->CPAL_Dev);
        
        /* Point to next data and Decrement remaining number of data */
        pDevInitStruct->pCPAL_TransferRx->wNumData--;   
        
        CPAL_LOG("\n\r\n\rLOG <I2C_EV_IRQHandler> : I2C Device Master IT");
        
        CPAL_LOG("\n\rLOG : I2C Device RX Nack Programmed");
        
        CPAL_LOG("\n\rLOG : I2C Device RX Stop Programmed");
      }
      
      /* Two bytes */
      if (pDevInitStruct->pCPAL_TransferRx->wNumData == 2)
      {           
        /* Disable Buffer interrupt */
        __CPAL_I2C_HAL_DISABLE_BUFIT(pDevInitStruct->CPAL_Dev);
        
        /* Wait until BTF flag is set */ 
        __CPAL_I2C_TIMEOUT_SPINLOOP(__CPAL_I2C_HAL_GET_BTF(pDevInitStruct->CPAL_Dev), CPAL_I2C_TIMEOUT_BTF);
        
        /* Generate Stop Condition */
        __CPAL_I2C_HAL_STOP(pDevInitStruct->CPAL_Dev);
        
        /* Read Byte */
        *(pDevInitStruct->pCPAL_TransferRx->pbBuffer) = __CPAL_I2C_HAL_RECEIVE(pDevInitStruct->CPAL_Dev);
        
        /* Point to next data and Decrement remaining number of data */
        pDevInitStruct->pCPAL_TransferRx->pbBuffer++;
        
        pDevInitStruct->pCPAL_TransferRx->wNumData--; 
        
        /* Read Byte */
        *(pDevInitStruct->pCPAL_TransferRx->pbBuffer) = __CPAL_I2C_HAL_RECEIVE(pDevInitStruct->CPAL_Dev);
        
        /*Decrement remaining number of data */
        pDevInitStruct->pCPAL_TransferRx->wNumData--;           
        
        CPAL_LOG("\n\r\n\rLOG <I2C_EV_IRQHandler> : I2C Device Master IT");
        
        CPAL_LOG("\n\rLOG : I2C Device RX Nack Programmed");
        
        CPAL_LOG("\n\rLOG : I2C Device RX Stop Programmed");
      }
      
      /* 3 Last bytes */
      if (pDevInitStruct->pCPAL_TransferRx->wNumData == 3)
      {
        /* Disable Buffer interrupt */
        __CPAL_I2C_HAL_DISABLE_BUFIT(pDevInitStruct->CPAL_Dev);
        
        /* Wait until BTF flag is set */ 
        __CPAL_I2C_TIMEOUT_SPINLOOP(__CPAL_I2C_HAL_GET_BTF(pDevInitStruct->CPAL_Dev), CPAL_I2C_TIMEOUT_BTF);
        
        /* Program NACK Generation */
        __CPAL_I2C_HAL_DISABLE_ACK(pDevInitStruct->CPAL_Dev);
        
        /* Read Byte */
        *(pDevInitStruct->pCPAL_TransferRx->pbBuffer) = __CPAL_I2C_HAL_RECEIVE(pDevInitStruct->CPAL_Dev);
        
        /* Point to next data and Decrement remaining number of data */
        pDevInitStruct->pCPAL_TransferRx->pbBuffer++;
        
        pDevInitStruct->pCPAL_TransferRx->wNumData--; 
        
         /* Wait until BTF flag is set */ 
        __CPAL_I2C_TIMEOUT_SPINLOOP(__CPAL_I2C_HAL_GET_BTF(pDevInitStruct->CPAL_Dev), CPAL_I2C_TIMEOUT_BTF);
        
        /* Generate Stop Condition */
        __CPAL_I2C_HAL_STOP(pDevInitStruct->CPAL_Dev);        
        
        /* Read Byte */
        *(pDevInitStruct->pCPAL_TransferRx->pbBuffer) = __CPAL_I2C_HAL_RECEIVE(pDevInitStruct->CPAL_Dev);
        
        /* Point to next data and Decrement remaining number of data */
        pDevInitStruct->pCPAL_TransferRx->pbBuffer++;
        
        pDevInitStruct->pCPAL_TransferRx->wNumData--; 
          
        /* Read Byte */
        *(pDevInitStruct->pCPAL_TransferRx->pbBuffer) = __CPAL_I2C_HAL_RECEIVE(pDevInitStruct->CPAL_Dev);
        
        /* Decrement remaining number of data */
        pDevInitStruct->pCPAL_TransferRx->wNumData--;   
        
        CPAL_LOG("\n\r\n\rLOG <I2C_EV_IRQHandler> : I2C Device Master IT");
        
        CPAL_LOG("\n\rLOG : I2C Device RX Nack Programmed");
        
        CPAL_LOG("\n\rLOG : I2C Device RX Stop Programmed");
      }          
    }     
#else    
    
 #ifdef CPAL_I2C_CLOSECOM_METHOD1
    /* if Two bytes remaining for reception */
    if (pDevInitStruct->pCPAL_TransferRx->wNumData == 2)
    {         
      /* Read Byte */
      *(pDevInitStruct->pCPAL_TransferRx->pbBuffer) = __CPAL_I2C_HAL_RECEIVE(pDevInitStruct->CPAL_Dev);
      
      /* Disable Acknowledge */
      __CPAL_I2C_HAL_DISABLE_ACK(pDevInitStruct->CPAL_Dev);
      
      /* Program Generation of Stop Condition */
      __CPAL_I2C_HAL_STOP(pDevInitStruct->CPAL_Dev);
      
      /* Point to next data and Decrement remaining number of data */
      pDevInitStruct->pCPAL_TransferRx->wNumData--; 
      
      /* Point to next data and Decrement remaining number of data */
      pDevInitStruct->pCPAL_TransferRx->pbBuffer++;      
    }
     /* if One byte remaining for reception */
    else if (pDevInitStruct->pCPAL_TransferRx->wNumData == 1)
    {
      /* Read Byte */
      *(pDevInitStruct->pCPAL_TransferRx->pbBuffer) = __CPAL_I2C_HAL_RECEIVE(pDevInitStruct->CPAL_Dev);
      
      /* Point to next data and Decrement remaining number of data */
      pDevInitStruct->pCPAL_TransferRx->wNumData--; 
      
      CPAL_LOG("\n\r\n\rLOG <I2C_EV_IRQHandler> : I2C Device Master IT");
      
      CPAL_LOG("\n\rLOG : I2C Device RX Nack Programmed");
      
      CPAL_LOG("\n\rLOG : I2C Device RX Stop Programmed");
    }      
 #endif /* CPAL_I2C_CLOSECOM_METHOD1 */
    
 #ifdef CPAL_I2C_CLOSECOM_METHOD2
    /* if less than 3 bytes remaining for reception */ 
    if (pDevInitStruct->pCPAL_TransferRx->wNumData <= 3)
    {  
      /* One byte */
      if (pDevInitStruct->pCPAL_TransferRx->wNumData == 1)
      {              
        /* Read Byte */
        *(pDevInitStruct->pCPAL_TransferRx->pbBuffer) = __CPAL_I2C_HAL_RECEIVE(pDevInitStruct->CPAL_Dev);
        
        /* Point to next data and Decrement remaining number of data */
        pDevInitStruct->pCPAL_TransferRx->wNumData--; 
        
        CPAL_LOG("\n\r\n\rLOG <I2C_EV_IRQHandler> : I2C Device Master IT");
        
        CPAL_LOG("\n\rLOG : I2C Device RX Nack Programmed");
        
        CPAL_LOG("\n\rLOG : I2C Device RX Stop Programmed");
      }
      
      /* Two bytes */
      if (pDevInitStruct->pCPAL_TransferRx->wNumData == 2)
      {           
        /* Disable Buffer interrupt */
        __CPAL_I2C_HAL_DISABLE_BUFIT(pDevInitStruct->CPAL_Dev);
        
        /* Wait until BTF flag is set */ 
        __CPAL_I2C_TIMEOUT_SPINLOOP(__CPAL_I2C_HAL_GET_BTF(pDevInitStruct->CPAL_Dev), CPAL_I2C_TIMEOUT_BTF);
        
        /* Generate Stop Condition */
        __CPAL_I2C_HAL_STOP(pDevInitStruct->CPAL_Dev);
        
        /* Read Byte */
        *(pDevInitStruct->pCPAL_TransferRx->pbBuffer) = __CPAL_I2C_HAL_RECEIVE(pDevInitStruct->CPAL_Dev);
        
        /* Point to next data and Decrement remaining number of data */
        pDevInitStruct->pCPAL_TransferRx->pbBuffer++;
        
        pDevInitStruct->pCPAL_TransferRx->wNumData--; 
        
        /* Read Byte */
        *(pDevInitStruct->pCPAL_TransferRx->pbBuffer) = __CPAL_I2C_HAL_RECEIVE(pDevInitStruct->CPAL_Dev);
        
        /*Decrement remaining number of data */
        pDevInitStruct->pCPAL_TransferRx->wNumData--; 
        
        /* Reset POS */
        __CPAL_I2C_HAL_DISABLE_POS(pDevInitStruct->CPAL_Dev);
        
        CPAL_LOG("\n\r\n\rLOG <I2C_EV_IRQHandler> : I2C Device Master IT");
        
        CPAL_LOG("\n\rLOG : I2C Device RX Nack Programmed");
        
        CPAL_LOG("\n\rLOG : I2C Device RX Stop Programmed");
      }
      
      /* 3 Last bytes */
      if (pDevInitStruct->pCPAL_TransferRx->wNumData == 3)
      {
        /* Disable Buffer interrupt */
        __CPAL_I2C_HAL_DISABLE_BUFIT(pDevInitStruct->CPAL_Dev);
        
        /* Wait until BTF flag is set */ 
        __CPAL_I2C_TIMEOUT_SPINLOOP(__CPAL_I2C_HAL_GET_BTF(pDevInitStruct->CPAL_Dev), CPAL_I2C_TIMEOUT_BTF);
        
        /* Program NACK Generation */
        __CPAL_I2C_HAL_DISABLE_ACK(pDevInitStruct->CPAL_Dev);
        
        /* Read Byte */
        *(pDevInitStruct->pCPAL_TransferRx->pbBuffer) = __CPAL_I2C_HAL_RECEIVE(pDevInitStruct->CPAL_Dev);
        
        /* Point to next data and Decrement remaining number of data */
        pDevInitStruct->pCPAL_TransferRx->pbBuffer++;
        
        pDevInitStruct->pCPAL_TransferRx->wNumData--; 
        
        /* Generate Stop Condition */
        __CPAL_I2C_HAL_STOP(pDevInitStruct->CPAL_Dev);        
        
        /* Read Byte */
        *(pDevInitStruct->pCPAL_TransferRx->pbBuffer) = __CPAL_I2C_HAL_RECEIVE(pDevInitStruct->CPAL_Dev);
        
        /* Point to next data and Decrement remaining number of data */
        pDevInitStruct->pCPAL_TransferRx->pbBuffer++;
        
        pDevInitStruct->pCPAL_TransferRx->wNumData--; 
        
        /* Wait until RXNE flag is set */ 
        __CPAL_I2C_TIMEOUT_SPINLOOP(__CPAL_I2C_HAL_GET_RXNE(pDevInitStruct->CPAL_Dev), CPAL_I2C_TIMEOUT_RXNE);
        
        /* Read Byte */
        *(pDevInitStruct->pCPAL_TransferRx->pbBuffer) = __CPAL_I2C_HAL_RECEIVE(pDevInitStruct->CPAL_Dev);
        
        /* Decrement remaining number of data */
        pDevInitStruct->pCPAL_TransferRx->wNumData--;   
        
        CPAL_LOG("\n\r\n\rLOG <I2C_EV_IRQHandler> : I2C Device Master IT");
        
        CPAL_LOG("\n\rLOG : I2C Device RX Nack Programmed");
        
        CPAL_LOG("\n\rLOG : I2C Device RX Stop Programmed");
      }          
    } 
 #endif /* CPAL_I2C_CLOSECOM_METHOD2 */
#endif /* STM32L1XX_MD || STM32L1XX_HD || STM32F2XX || STM32F4XX */
    
    /* if bytes remaining for reception */ 
    else
    {
      /* Read Byte */
      *(pDevInitStruct->pCPAL_TransferRx->pbBuffer) = __CPAL_I2C_HAL_RECEIVE(pDevInitStruct->CPAL_Dev);
      
      /* Point to next data and Decrement remaining number of data */
      pDevInitStruct->pCPAL_TransferRx->pbBuffer++;
      
      pDevInitStruct->pCPAL_TransferRx->wNumData--; 
      
      /* Call RX UserCallback */
      CPAL_I2C_RX_UserCallback(pDevInitStruct);
    }
    
    /* If All data are received */
    if (pDevInitStruct->pCPAL_TransferRx->wNumData == 0)
    {      
      CPAL_LOG("\n\rLOG : I2C Device Nack and Stop Generated ");
      
      CPAL_LOG("\n\rLOG : I2C Device RX Complete"); 
      
      /* Disable EVENT Interrupt */
      __CPAL_I2C_HAL_DISABLE_EVTIT(pDevInitStruct->CPAL_Dev);
      
      CPAL_LOG("\n\rLOG : I2C Device RX EVT IT Disabled");
      
      /* Disable Buffer interrupt */
      __CPAL_I2C_HAL_DISABLE_BUFIT(pDevInitStruct->CPAL_Dev);
      
      CPAL_LOG("\n\rLOG : I2C Device RX BUFF IT Disabled");
      
      /* Clear BTF Flag */
      __CPAL_I2C_HAL_CLEAR_BTF(pDevInitStruct->CPAL_Dev);   
      
      /* If 1Byte DMA option is selected */
      if ((pDevInitStruct->wCPAL_Options & CPAL_DMA_1BYTE_CASE) != 0)
      {
        /* Clear 1Byte DMA option from wCPAL_Options */
        pDevInitStruct->wCPAL_Options &= ~CPAL_DMA_1BYTE_CASE;
        
        /* Change ProgModel to DMA */
        pDevInitStruct->CPAL_ProgModel = CPAL_PROGMODEL_DMA;
      }
      
      /* Wait until Busy flag is reset */ 
      __CPAL_I2C_TIMEOUT_SPINLOOP(!(__CPAL_I2C_HAL_GET_BUSY(pDevInitStruct->CPAL_Dev)), CPAL_I2C_TIMEOUT_BUSY);
      
      /* Enable ACK generation and disable POS */
      __CPAL_I2C_HAL_ENABLE_ACK(pDevInitStruct->CPAL_Dev);      
      __CPAL_I2C_HAL_DISABLE_POS(pDevInitStruct->CPAL_Dev);
      
      /* Update CPAL_State to CPAL_STATE_READY */
      pDevInitStruct->CPAL_State = CPAL_STATE_READY;
      
      /* Call RX Transfer complete Callback */
      CPAL_I2C_RXTC_UserCallback(pDevInitStruct);
    }
  }  
  return CPAL_PASS;
}
 #endif /* CPAL_I2C_IT_PROGMODEL || CPAL_I2C_DMA_1BYTE_CASE */
#endif /* CPAL_I2C_MASTER_MODE */ 


#ifdef CPAL_I2C_SLAVE_MODE
/**
  * @brief  Handles Slave address matched (ADDR) interrupt event.
  * @param  pDevInitStruct: Pointer to the peripheral configuration structure.
  * @retval CPAL_PASS or CPAL_FAIL. 
  */
static uint32_t I2C_SLAVE_ADDR_Handle(CPAL_InitTypeDef* pDevInitStruct)
{       
  CPAL_LOG("\n\r\n\rLOG <I2C_EV_IRQHandler> : I2C Device Slave IT");
  
  CPAL_LOG("\n\rLOG : I2C Device Address Matched");
  
#ifdef CPAL_I2C_LISTEN_MODE  
  /* If slave receive request for write */
  if (__CPAL_I2C_HAL_GET_TRA(pDevInitStruct->CPAL_Dev) != 0)
  {   
    pDevInitStruct->CPAL_State = CPAL_STATE_BUSY_TX;
    
    /* Call Slave Transmit UserCallback */
    CPAL_I2C_SLAVE_WRITE_UserCallback(pDevInitStruct);     
  }
  /* If slave receive request for read */
  else
  {
    pDevInitStruct->CPAL_State = CPAL_STATE_BUSY_RX;
    
    /* Call Slave receive UserCallback */
    CPAL_I2C_SLAVE_READ_UserCallback(pDevInitStruct);
  }    
#else   
  /* If General Call Addressing Mode selected */
  if (__CPAL_I2C_HAL_GET_GENCALL(pDevInitStruct->CPAL_Dev) != 0)
  {       
    CPAL_LOG("\n\rLOG : I2C Device GENCALL Mode");
    
    /* Call GENCALL UserCallback */
    CPAL_I2C_GENCALL_UserCallback(pDevInitStruct);
  }
  
  /* If DUAL Addressing Mode is not selected */
  if (__CPAL_I2C_HAL_GET_DUALF(pDevInitStruct->CPAL_Dev) != 0)
  {         
    CPAL_LOG("\n\rLOG : I2C Device DUAL ADDR Mode Selected");
    
    /* Call DUALF UserCallback */
    CPAL_I2C_DUALF_UserCallback(pDevInitStruct);
  }    
  
  /* If device is ready for transmission */
  if (pDevInitStruct->CPAL_State == CPAL_STATE_READY_TX)
  {              
    /* Update CPAL_State to CPAL_STATE_BUSY */
    pDevInitStruct->CPAL_State = CPAL_STATE_BUSY_TX;
    
    CPAL_LOG("\n\rLOG : I2C Device Busy TX");
  }
  /* If device is ready for reception */
  else if (pDevInitStruct->CPAL_State == CPAL_STATE_READY_RX)
  {              
    /* Update CPAL_State to CPAL_STATE_BUSY_RX */
    pDevInitStruct->CPAL_State = CPAL_STATE_BUSY_RX;
    
    CPAL_LOG("\n\rLOG : I2C Device Busy RX");
  }
#endif /* CPAL_I2C_LISTEN_MODE */
 
  return CPAL_PASS;
}

/**
  * @brief  Handles Slave Stop condiction (STOPF) interrupt event.
  * @param  pDevInitStruct: Pointer to the peripheral configuration structure.
  * @retval CPAL_PASS or CPAL_FAIL. 
  */
static uint32_t I2C_SLAVE_STOP_Handle(CPAL_InitTypeDef* pDevInitStruct)
{   
  /* Clear STOPF */
  __CPAL_I2C_HAL_CLEAR_STOPF(pDevInitStruct->CPAL_Dev);     
  
  CPAL_LOG("\n\r\n\rLOG <I2C_EV_IRQHandler> : I2C Device Slave IT");
  
  CPAL_LOG("\n\rLOG : I2C Device Stop Detected");  
  
  /* If NACK Slave Own Address option bit selected */
  if ((pDevInitStruct->wCPAL_Options & CPAL_OPT_I2C_NACK_ADD) != 0)
  {
    /* Disable Acknowledgement of own Address */
    __CPAL_I2C_HAL_DISABLE_ACK(pDevInitStruct->CPAL_Dev);
  }
  
  /* If Interrupt Programming Model */
  if (pDevInitStruct->CPAL_ProgModel == CPAL_PROGMODEL_INTERRUPT)
  {  
 #ifdef CPAL_I2C_IT_PROGMODEL    
    /* Disable EVENT Interrupt */
    __CPAL_I2C_HAL_DISABLE_EVTIT(pDevInitStruct->CPAL_Dev);
    
    CPAL_LOG("\n\rLOG : I2C Device EVT IT Disabled");
    
    /* Disable Buffer interrupt */
    __CPAL_I2C_HAL_DISABLE_BUFIT(pDevInitStruct->CPAL_Dev);
    
    CPAL_LOG("\n\rLOG : I2C Device BUFF IT Disabled");    
    
 #endif /* CPAL_I2C_IT_PROGMODEL */
  }  
  /* If DMA Programming model */
  else 
  {  
 #ifdef CPAL_I2C_DMA_PROGMODEL
    /* If Slave run as receiver */
    if (pDevInitStruct->CPAL_State == CPAL_STATE_BUSY_RX)
    {
      /* Disable DMA Request and Channel */
      __CPAL_I2C_HAL_DISABLE_DMAREQ(pDevInitStruct->CPAL_Dev);      
      __CPAL_I2C_HAL_DISABLE_DMARX(pDevInitStruct->CPAL_Dev);
      
      /* Disable EVENT Interrupt */
      __CPAL_I2C_HAL_DISABLE_EVTIT(pDevInitStruct->CPAL_Dev);
      
      /* Update remaining number of data */
      pDevInitStruct->pCPAL_TransferRx->wNumData = __CPAL_I2C_HAL_DMARX_GET_CNDT(pDevInitStruct->CPAL_Dev);
      
      CPAL_LOG("\n\rLOG : I2C Device RX Complete");
      
      CPAL_LOG("\n\rLOG : I2C Device RX DMA Disabled"); 
    }        
 #endif /* CPAL_I2C_DMA_PROGMODEL */
  }  
           
  /* Wait until Busy flag is reset */ 
  __CPAL_I2C_TIMEOUT_SPINLOOP(!(__CPAL_I2C_HAL_GET_BUSY(pDevInitStruct->CPAL_Dev)), CPAL_I2C_TIMEOUT_BUSY);
  
  /* If Slave run as receiver */
  if (pDevInitStruct->CPAL_State == CPAL_STATE_BUSY_RX)
  { 
    /* Update CPAL_State to CPAL_STATE_READY */
    pDevInitStruct->CPAL_State = CPAL_STATE_READY; 
    
    /* Call RX Transfer complete Callback */
    CPAL_I2C_RXTC_UserCallback(pDevInitStruct);
  }    
  
  return CPAL_PASS;
}


 #ifdef CPAL_I2C_IT_PROGMODEL
/**
  * @brief  Handles Slave transmission (TXE) interrupt event.
  * @param  pDevInitStruct: Pointer to the peripheral configuration structure.
  * @retval CPAL_PASS or CPAL_FAIL. 
  */
static uint32_t I2C_SLAVE_TXE_Handle(CPAL_InitTypeDef* pDevInitStruct)
{  
  /* If Interrupt Programming Model selected */
  if (pDevInitStruct->CPAL_ProgModel == CPAL_PROGMODEL_INTERRUPT)
  {  
    if (pDevInitStruct->pCPAL_TransferTx->wNumData != 0)
    {   
      /* Call TX UserCallback */
      CPAL_I2C_TX_UserCallback(pDevInitStruct);
      
      /* Write Byte */
      __CPAL_I2C_HAL_SEND((pDevInitStruct->CPAL_Dev), (*(pDevInitStruct->pCPAL_TransferTx->pbBuffer)));
      
      /* Decrement remaining number of data */      
      pDevInitStruct->pCPAL_TransferTx->wNumData--;
      
      if (pDevInitStruct->pCPAL_TransferTx->wNumData != 0)
      {  
        /* Point to next data */
        pDevInitStruct->pCPAL_TransferTx->pbBuffer++;  
      }     
      else
      {       
        CPAL_LOG("\n\r\n\rLOG <I2C_EV_IRQHandler> : I2C Device Slave IT");
        
        CPAL_LOG("\n\rLOG : I2C Device TX Complete");
      }
    }
  }
  
  return CPAL_PASS;
}


/**
  * @brief  Handles Slave reception (RXNE) interrupt event.
  * @param  pDevInitStruct: Pointer to the peripheral configuration structure.
  * @retval CPAL_PASS or CPAL_FAIL. 
  */
static uint32_t I2C_SLAVE_RXNE_Handle(CPAL_InitTypeDef* pDevInitStruct)
{  
  /* If Interrupt Programming Model selected */
  if (pDevInitStruct->CPAL_ProgModel == CPAL_PROGMODEL_INTERRUPT)
  {   
    /* If data remaining for reception */
    if (pDevInitStruct->pCPAL_TransferRx->wNumData != 0)
    {          
      /* Read Byte */
      *(pDevInitStruct->pCPAL_TransferRx->pbBuffer) = __CPAL_I2C_HAL_RECEIVE(pDevInitStruct->CPAL_Dev);
      
      /* Call RX UserCallback */
      CPAL_I2C_RX_UserCallback(pDevInitStruct); 
      
      /* Decrement remaining number of data */
      pDevInitStruct->pCPAL_TransferRx->wNumData--;
      
      /* If data remaining for reception */
      if (pDevInitStruct->pCPAL_TransferRx->wNumData != 0)
      {  
        /* Point to next data */
        pDevInitStruct->pCPAL_TransferRx->pbBuffer++;
      }
      else
      {
        /* No Stop Condition Generation option bit selected */ 
        if ((pDevInitStruct->wCPAL_Options & CPAL_OPT_I2C_NOSTOP) != 0)
        {   
          /* Disable EVENT Interrupt */
          __CPAL_I2C_HAL_DISABLE_EVTIT(pDevInitStruct->CPAL_Dev);
          
          CPAL_LOG("\n\rLOG : I2C Device EVT IT Disabled");
          
          /* Disable Buffer interrupt */
          __CPAL_I2C_HAL_DISABLE_BUFIT(pDevInitStruct->CPAL_Dev);
          
          CPAL_LOG("\n\rLOG : I2C Device BUFF IT Disabled");
          
          /* Update CPAL_State to CPAL_STATE_READY */
          pDevInitStruct->CPAL_State = CPAL_STATE_READY; 
          
          /* Call RX Transfer complete Callback */
          CPAL_I2C_RXTC_UserCallback(pDevInitStruct);        
        }     
        
        CPAL_LOG("\n\r\n\rLOG <I2C_EV_IRQHandler> : I2C Device Slave IT");
        
        CPAL_LOG("\n\rLOG : I2C Device RX Complete");  
      }
    }
  }  
  return CPAL_PASS;
}
 #endif /* CPAL_I2C_IT_PROGMODEL */
#endif /* CPAL_I2C_SLAVE_MODE */



/*================== Local DMA and IT Manager ==================*/

/**
  * @brief  This function Configure I2C DMA and Interrupts before starting transfer phase.
  * @param  pDevInitStruct: Pointer to the peripheral configuration structure.
  * @param  Direction : Transfer direction.
  * @retval CPAL_PASS or CPAL_FAIL. 
  */
uint32_t CPAL_I2C_Enable_DMA_IT (CPAL_InitTypeDef* pDevInitStruct, CPAL_DirectionTypeDef Direction)
{
  /* Switch the value of CPAL_ProgModel */
  switch (pDevInitStruct->CPAL_ProgModel)
  { 
    
#if defined (CPAL_I2C_IT_PROGMODEL) || defined (CPAL_I2C_DMA_1BYTE_CASE)
    /*----------------------------------------------------------------------------
    Interrupt mode : if CPAL_ProgModel = CPAL_PROGMODEL_INTERRUPT
    ---------------------------------------------------------------------------*/            
  case CPAL_PROGMODEL_INTERRUPT:
   
    /* Enable BUFFER Interrupt*/
    __CPAL_I2C_HAL_ENABLE_BUFIT(pDevInitStruct->CPAL_Dev);
    
    CPAL_LOG("\n\rLOG : I2C Device BUFF IT Enabled"); 
    
    return CPAL_PASS;
#endif /* CPAL_I2C_IT_PROGMODEL || CPAL_I2C_DMA_1BYTE_CASE */
    
#ifdef CPAL_I2C_DMA_PROGMODEL
    /*----------------------------------------------------------------------------
    DMA mode : if CPAL_ProgModel = CPAL_PROGMODEL_DMA
    ---------------------------------------------------------------------------*/      
    case CPAL_PROGMODEL_DMA:
    
     /* Disable EVENT Interrupt */
     __CPAL_I2C_HAL_DISABLE_EVTIT(pDevInitStruct->CPAL_Dev);
    
     /* Enable DMA request */
     __CPAL_I2C_HAL_ENABLE_DMAREQ(pDevInitStruct->CPAL_Dev);
    
    /* If a data transmission will be performed */
    if ((pDevInitStruct->CPAL_State == CPAL_STATE_BUSY_TX) || (Direction == CPAL_DIRECTION_TX))
    {
      /* Configure TX DMA Channels */
      CPAL_I2C_HAL_DMATXConfig(pDevInitStruct->CPAL_Dev, pDevInitStruct->pCPAL_TransferTx, pDevInitStruct->wCPAL_Options);
      
      /* Disable DMA automatic NACK generation */
      __CPAL_I2C_HAL_DISABLE_LAST(pDevInitStruct->CPAL_Dev); 
    
      /* Enable TX DMA Channels */
      __CPAL_I2C_HAL_ENABLE_DMATX(pDevInitStruct->CPAL_Dev);
      
      CPAL_LOG("\n\rLOG : I2C Device DMA TX Enabled");       
    }    
     /* If a data reception will be performed */
    else if ((pDevInitStruct->CPAL_State == CPAL_STATE_BUSY_RX) || (Direction == CPAL_DIRECTION_RX))
    {
      /* Configure RX DMA Channels */
      CPAL_I2C_HAL_DMARXConfig(pDevInitStruct->CPAL_Dev, pDevInitStruct->pCPAL_TransferRx, pDevInitStruct->wCPAL_Options);
      
      /* If Master Mode Selected */
      if(pDevInitStruct->CPAL_Mode == CPAL_MODE_MASTER )
      {
        /* Enable DMA automatic NACK generation */
        __CPAL_I2C_HAL_ENABLE_LAST(pDevInitStruct->CPAL_Dev);
      }
    
      /* Enable RX DMA Channels */
      __CPAL_I2C_HAL_ENABLE_DMARX(pDevInitStruct->CPAL_Dev);                  
    }
    
    return CPAL_PASS; 
#endif /* CPAL_I2C_DMA_PROGMODEL */
    
    /*----------------------------------------------------------------------------
    Default: return error and exit Write Operation
    ---------------------------------------------------------------------------*/      
  default:
    
    /* Update CPAL_State to CPAL_STATE_ERROR */
    pDevInitStruct->CPAL_State = CPAL_STATE_ERROR;
    
    CPAL_LOG("\n\rERROR : I2C Device Error"); 
    
    /* exit function */
    return CPAL_FAIL;
  }  
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
