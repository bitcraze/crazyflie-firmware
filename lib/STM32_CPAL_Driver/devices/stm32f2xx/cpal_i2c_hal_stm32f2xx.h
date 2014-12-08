/**
  *********************************************************************************
  * @file    Libraries/STM32_CPAL_Driver/devices/stm32f2xx/cpal_i2c_hal_stm32f2xx.h
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    21-December-2012
  * @brief   This file contains all the functions prototypes for the CPAL_I2C_HAL 
  *          firmware layer.
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
#ifndef ___CPAL_I2C_HAL_STM32F2XX_H
#define ___CPAL_I2C_HAL_STM32F2XX_H

#ifdef __cplusplus
extern "C" {
#endif
  
/* Includes ------------------------------------------------------------------*/
/*========= STM32 Standard library files includes =========*/
#include "stm32f2xx.h"
#include "stm32f2xx_i2c.h"  
#include "stm32f2xx_dma.h" 
#include "stm32f2xx_gpio.h"
#include "stm32f2xx_rcc.h"
#include "misc.h"
  
/*========= CPAL library files includes =========*/
#include "cpal.h" 

 
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
  
/*======================================================================================================================================
                                                 CPAL Hardware Configuration 
========================================================================================================================================*/

/*  ------ Configure the communication device and all related peripherals ( GPIO Pin, DMA Channels, 
            NVIC Priority) with this file, by referring to configuration Sections:
  
                - Section 1 : Select the pins to be used for each device instance.
  
                - Section 2 : Select TX and RX DMA Channels (if DMA mode will be used).
  
                - Section 3 : Set device's Events, Errors and DMA Interrupts Priorities.                   */
  
  
/*-----------------------------------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------*/

/*    -- Section 1 :                 **** Device IO Pins Selection ****
  
  Description: This section allows user to choose IO Pins for each device if possible (in accordance with 
               used product: some products have only one possibility for the IO pins).
               Each device instance (I2C1, I2C2, I2C3 ..) has its specific defines: one for each Pin.
               For each device instance, you will change existing defines with adequate IO Pins and Port 
               ( Refer to Product Pin mapping in related datasheet). */
 
/* To configure SCL and SDA Pin change these defines with adequate value :
  
#define CPAL_I2C1_SCL_GPIO_PORT         GPIOX                  (X : Name of the GPIO PORT  (A,B,C,....))     
#define CPAL_I2C1_SCL_GPIO_CLK          RCC_APB2Periph_GPIOX   (X : Name of the GPIO PORT  (A,B,C,....))   
#define CPAL_I2C1_SCL_GPIO_PIN          GPIO_Pin_X             (X : Pin number (1,2,3,....))   
#define CPAL_I2C1_SCL_GPIO_PINSOURCE    GPIO_PinSourceX        (X : Pin number (1,2,3,....)) 
  
#define CPAL_I2C1_SDA_GPIO_PORT         GPIOX                  (X : Name of the GPIO PORT  (A,B,C,....)) 
#define CPAL_I2C1_SDA_GPIO_CLK          RCC_APB2Periph_GPIOX   (X : Name of the GPIO PORT  (A,B,C,....))  
#define CPAL_I2C1_SDA_GPIO_PIN          GPIO_Pin_X             (X : Pin number (1,2,3,....))                         
#define CPAL_I2C1_SDA_GPIO_PINSOURCE    GPIO_PinSourceX        (X : Pin number (1,2,3,....))                   */

/* IO Pins selection possibilities 
  
|--------|---------|--------------|-----------|------------------|-------------------------|
| Device | I2C PIN |   GPIO_PIN   | GPIO_PORT |  GPIO_PinSource  |        GPIO_CLK         |                                          
|--------|---------|--------------|-----------|------------------|-------------------------|
|        |         |  GPIO_Pin_6  |   GPIOB   | GPIO_PinSource6  |   RCC_AHB1Periph_GPIOB  |                                          
|        |   SCL   |--------------|-----------|------------------|-------------------------|
|        |         |  GPIO_Pin_8  |   GPIOB   | GPIO_PinSource7  |   RCC_AHB1Periph_GPIOB  |                                          
|  I2C1  |---------|--------------|-----------|------------------|-------------------------|
|        |         |  GPIO_Pin_7  |   GPIOB   | GPIO_PinSource8  |   RCC_AHB1Periph_GPIOB  |                                          
|        |   SDA   |--------------|-----------|------------------|-------------------------|
|        |         |  GPIO_Pin_9  |   GPIOB   | GPIO_PinSource9  |   RCC_AHB1Periph_GPIOB  |                                          
|--------|---------|--------------|-----------|------------------|-------------------------|
|        |         |  GPIO_Pin_10 |   GPIOB   | GPIO_PinSource10 |   RCC_AHB1Periph_GPIOB  |                                          
|        |         |--------------|-----------|------------------|-------------------------|
|        |   SCL   |  GPIO_Pin_1  |   GPIOF   | GPIO_PinSource1  |   RCC_AHB1Periph_GPIOF  |                                           
|        |         |--------------|-----------|------------------|-------------------------|    
|        |         |  GPIO_Pin_4  |   GPIOH   | GPIO_PinSource4  |   RCC_AHB1Periph_GPIOH  |                                           
|  I2C2  |---------|--------------|-----------|------------------|-------------------------|
|        |         |  GPIO_Pin_11 |   GPIOB   | GPIO_PinSource11 |   RCC_AHB1Periph_GPIOB  |                                           
|        |         |--------------|-----------|------------------|-------------------------|
|        |   SDA   |  GPIO_Pin_0  |   GPIOF   | GPIO_PinSource0  |   RCC_AHB1Periph_GPIOF  |                                           
|        |         |--------------|-----------|------------------|-------------------------|
|        |         |  GPIO_Pin_5  |   GPIOH   | GPIO_PinSource5  |   RCC_AHB1Periph_GPIOH  |                                           
|--------|---------|--------------|-----------|------------------|-------------------------|
|        |         |  GPIO_Pin_7  |   GPIOH   | GPIO_PinSource7  |   RCC_AHB1Periph_GPIOH  |                                           
|        |   SCL   |--------------|-----------|------------------|-------------------------|
|        |         |  GPIO_Pin_8  |   GPIOA   | GPIO_PinSource8  |   RCC_AHB1Periph_GPIOA  |                                           
|  I2C3  |---------|--------------|-----------|------------------|-------------------------|
|        |         |  GPIO_Pin_8  |   GPIOH   | GPIO_PinSource8  |   RCC_AHB1Periph_GPIOH  |                                           
|        |   SDA   |--------------|-----------|------------------|-------------------------|
|        |         |  GPIO_Pin_9  |   GPIOC   | GPIO_PinSource9  |   RCC_AHB1Periph_GPIOC  |                                           
|--------|---------|--------------|-----------|------------------|-------------------------|  */ 
  
  
/*----------- I2C1 Device -----------*/
  
#define CPAL_I2C1_SCL_GPIO_PORT         GPIOB       
#define CPAL_I2C1_SCL_GPIO_CLK          RCC_AHB1Periph_GPIOB 
#define CPAL_I2C1_SCL_GPIO_PIN          GPIO_Pin_6
#define CPAL_I2C1_SCL_GPIO_PINSOURCE    GPIO_PinSource6 
  
#define CPAL_I2C1_SDA_GPIO_PORT         GPIOB       
#define CPAL_I2C1_SDA_GPIO_CLK          RCC_AHB1Periph_GPIOB 
#define CPAL_I2C1_SDA_GPIO_PIN          GPIO_Pin_9 
#define CPAL_I2C1_SDA_GPIO_PINSOURCE    GPIO_PinSource9  
  
/*-----------I2C2 Device -----------*/
  
#define CPAL_I2C2_SCL_GPIO_PORT         GPIOB       
#define CPAL_I2C2_SCL_GPIO_CLK          RCC_AHB1Periph_GPIOB 
#define CPAL_I2C2_SCL_GPIO_PIN          GPIO_Pin_10
#define CPAL_I2C2_SCL_GPIO_PINSOURCE    GPIO_PinSource10 
  
#define CPAL_I2C2_SDA_GPIO_PORT         GPIOB       
#define CPAL_I2C2_SDA_GPIO_CLK          RCC_AHB1Periph_GPIOB 
#define CPAL_I2C2_SDA_GPIO_PIN          GPIO_Pin_11 
#define CPAL_I2C2_SDA_GPIO_PINSOURCE    GPIO_PinSource11  
  
/*-----------I2C3 Device -----------*/
  
#define CPAL_I2C3_SCL_GPIO_PORT         GPIOH       
#define CPAL_I2C3_SCL_GPIO_CLK          RCC_AHB1Periph_GPIOH 
#define CPAL_I2C3_SCL_GPIO_PIN          GPIO_Pin_7
#define CPAL_I2C3_SCL_GPIO_PINSOURCE    GPIO_PinSource7 
  
#define CPAL_I2C3_SDA_GPIO_PORT         GPIOH       
#define CPAL_I2C3_SDA_GPIO_CLK          RCC_AHB1Periph_GPIOH 
#define CPAL_I2C3_SDA_GPIO_PIN          GPIO_Pin_8 
#define CPAL_I2C3_SDA_GPIO_PINSOURCE    GPIO_PinSource8  
  
/*-----------------------------------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------*/

/*    -- Section 2 :           **** Device TX and RX DMA Channels Selection ****
  
  Description: This section allows user to choose TX and RX DMA Channels if possible (in accordance with 
               used product) for each device.
               Each device instance (I2C1, I2C2 ..) has its specific defines: one for DMA TX Channel and 
               another one for DMA RX Channel.
               For each device instance, you find all TX an RX DMA Channel possibilities ( Refer to Product
               Reference Manual).*/
 
/* DMA Stream selection possibilities 
  
|--------|-----------|-------------------------|--------------|
| Device | Direction |    Define Configuration |  DMA Stream  |
|--------|-----------|-------------------------|--------------|
|        |           | CPAL_I2C1_DMA_TX_CONFIG1| DMA1_Stream6 |
|        |     TX    |-------------------------|--------------|
|        |           | CPAL_I2C1_DMA_TX_CONFIG2| DMA1_Stream7 |
|  I2C1  |-----------|-------------------------|--------------|
|        |           | CPAL_I2C1_DMA_RX_CONFIG1| DMA1_Stream0 |
|        |     RX    |-------------------------|--------------|
|        |           | CPAL_I2C1_DMA_RX_CONFIG2| DMA1_Stream5 |
|--------|-----------|-------------------------|--------------|
|        |     TX    | CPAL_I2C2_DMA_TX_CONFIG1| DMA1_Stream7 |
|        |-----------|-------------------------|--------------|
|  I2C2  |           | CPAL_I2C2_DMA_RX_CONFIG1| DMA1_Stream3 |
|        |     RX    |-------------------------|--------------|
|        |           | CPAL_I2C2_DMA_RX_CONFIG2| DMA1_Stream2 |
|--------|-----------|-------------------------|--------------|
|        |     TX    | CPAL_I2C3_DMA_TX_CONFIG1| DMA1_Stream4 |
|  I2C3  |-----------|-------------------------|--------------|
|        |     RX    | CPAL_I2C3_DMA_TX_CONFIG1| DMA1_Stream2 |
|--------|-----------|-------------------------|--------------| */
  
/*----------- I2C1 Device -----------*/
/* TX Direction */
#define CPAL_I2C1_DMA_TX_CONFIG1
//#define CPAL_I2C1_DMA_TX_CONFIG2

/* RX Direction */
#define CPAL_I2C1_DMA_RX_CONFIG1  
//#define CPAL_I2C1_DMA_RX_CONFIG2  

/*----------- I2C2 Device -----------*/
/* TX Direction */
/* DMA Stream is fixed to DMA1_Stream7 */
  
/* RX Direction */
#define CPAL_I2C2_DMA_RX_CONFIG1
//#define CPAL_I2C2_DMA_RX_CONFIG2

/*----------- I2C3 Device -----------*/
/* TX Direction */
/* DMA Stream is fixed to DMA1_Stream4 */

/* RX Direction */
/* DMA Stream is fixed to DMA1_Stream2 */
 
/*-----------------------------------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------*/

/*   -- Section 3 :             **** I2C and DMA Interrupts Priority Selection ****
  
  Description: This section allows user to select Interrupt Priority of I2C Event Interrupts and DMA Interrupts.
                 
               Make sure that the following rules are always respected:
                   - I2C event and error interrupts should be interruptible (mainly, the timeout interrupt should 
                     be able to interrupt all device ISR)
                   - I2C Error interrupt priority should be higher than Event interrupt
                   - The timeout mechanism interrupt priority should be the highest one and it should be able to 
                     interrupt any other ISR.
                   - It is advised that DMA interrupts have higher priority than the device event interrupts.*/              

  
/*----------- NVIC Group Priority -------------*/
  
/* Refer To cpal_conf.h file "CPAL Firmware Functionality Configuration" section 5.
  !WARNING!:
  --------
   For the default configuration (I2C EVT and Error interrupt groups are respectively 
   1 and 2) the CPAL_NVIC_PRIOGROUP value should be set to NVIC_PriorityGroup_2
   otherwise the application might not work correctly. 
  */
  
/*----------- I2Cx Interrupt Priority -------------*/
  
/*----------- I2C1 Device -----------*/
#define I2C1_IT_EVT_SUBPRIO             I2C1_IT_OFFSET_SUBPRIO + 0   /* I2C1 EVT SUB-PRIORITY */ 
#define I2C1_IT_EVT_PREPRIO             I2C1_IT_OFFSET_PREPRIO + 2   /* I2C1 EVT PREEMPTION PRIORITY */ 
#define I2C1_IT_ERR_SUBPRIO             I2C1_IT_OFFSET_SUBPRIO + 0   /* I2C1 ERR SUB-PRIORITY */
#define I2C1_IT_ERR_PREPRIO             I2C1_IT_OFFSET_PREPRIO + 0   /* I2C1 ERR PREEMPTION PRIORITY */
#define I2C1_IT_DMATX_SUBPRIO           I2C1_IT_OFFSET_SUBPRIO + 0   /* I2C1 DMA TX SUB-PRIORITY */
#define I2C1_IT_DMATX_PREPRIO           I2C1_IT_OFFSET_PREPRIO + 1   /* I2C1 DMA TX PREEMPTION PRIORITY */
#define I2C1_IT_DMARX_SUBPRIO           I2C1_IT_OFFSET_SUBPRIO + 0   /* I2C1 DMA RX SUB-PRIORITY */
#define I2C1_IT_DMARX_PREPRIO           I2C1_IT_OFFSET_PREPRIO + 1   /* I2C1 DMA RX PREEMPTION PRIORITY */

/*----------- I2C2 Device -----------*/
#define I2C2_IT_EVT_SUBPRIO             I2C2_IT_OFFSET_SUBPRIO + 0   /* I2C2 EVT SUB-PRIORITY */ 
#define I2C2_IT_EVT_PREPRIO             I2C2_IT_OFFSET_PREPRIO + 2   /* I2C2 EVT PREEMPTION PRIORITY */ 
#define I2C2_IT_ERR_SUBPRIO             I2C2_IT_OFFSET_SUBPRIO + 0   /* I2C2 ERR SUB-PRIORITY */
#define I2C2_IT_ERR_PREPRIO             I2C2_IT_OFFSET_PREPRIO + 0   /* I2C2 ERR PREEMPTION PRIORITY */
#define I2C2_IT_DMATX_SUBPRIO           I2C2_IT_OFFSET_SUBPRIO + 0   /* I2C2 DMA TX SUB-PRIORITY */
#define I2C2_IT_DMATX_PREPRIO           I2C2_IT_OFFSET_PREPRIO + 1   /* I2C2 DMA TX PREEMPTION PRIORITY */
#define I2C2_IT_DMARX_SUBPRIO           I2C2_IT_OFFSET_SUBPRIO + 0   /* I2C2 DMA RX SUB-PRIORITY */
#define I2C2_IT_DMARX_PREPRIO           I2C2_IT_OFFSET_PREPRIO + 1   /* I2C2 DMA RX PREEMPTION PRIORITY */

/*----------- I2C3 Device -----------*/
#define I2C3_IT_EVT_SUBPRIO             I2C3_IT_OFFSET_SUBPRIO + 0   /* I2C3 EVT SUB-PRIORITY */ 
#define I2C3_IT_EVT_PREPRIO             I2C3_IT_OFFSET_PREPRIO + 2   /* I2C3 EVT PREEMPTION PRIORITY */ 
#define I2C3_IT_ERR_SUBPRIO             I2C3_IT_OFFSET_SUBPRIO + 0   /* I2C3 ERR SUB-PRIORITY */
#define I2C3_IT_ERR_PREPRIO             I2C3_IT_OFFSET_PREPRIO + 0   /* I2C3 ERR PREEMPTION PRIORITY */
#define I2C3_IT_DMATX_SUBPRIO           I2C3_IT_OFFSET_SUBPRIO + 0   /* I2C3 DMA TX SUB-PRIORITY */
#define I2C3_IT_DMATX_PREPRIO           I2C3_IT_OFFSET_PREPRIO + 1   /* I2C3 DMA TX PREEMPTION PRIORITY */
#define I2C3_IT_DMARX_SUBPRIO           I2C3_IT_OFFSET_SUBPRIO + 0   /* I2C3 DMA RX SUB-PRIORITY */
#define I2C3_IT_DMARX_PREPRIO           I2C3_IT_OFFSET_PREPRIO + 1   /* I2C3 DMA RX PREEMPTION PRIORITY */
  
/*-----------------------------------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------*/
 
/*****END OF CPAL Hardware Configuration***************************************************************************************************/
  
  /* !WARNING!:
     --------- 
     The following code should not be modified by user.
     Any modification may cause Library dysfunction.
  */  

/*========= Common Defines =========*/  
  
/* This define set the number of I2C devices that can be used with this product family */
#define CPAL_I2C_DEV_NUM     3   

/* This define is used to enable DMA Channel */
#define DMA_CR_EN                       ((uint32_t)0x00000001)
  
/* This define is used to get Interrupt status */
#define DMA_HIGH_ISR_MASK               ((uint32_t)0x20000000)
  
/* This define is used to check I2C errors ( BERR, ARLO, AF and OVR) */ 
#define CPAL_I2C_STATUS_ERR_MASK        ((uint16_t)0x0F00)  /*!< I2C errors Mask  */
  
/* This define is used to check I2C events ( TXE, RXNE, STOPF, ADD10, BTF, ADDR and SB) */ 
#define CPAL_I2C_STATUS1_EVT_MASK       ((uint16_t)0x00DF)  /*!< I2C event Mask for Status Register 1  */
  
/* This define is used to check I2C events ( DUALF, GENCALL, TRA, BUSY and MSL) */ 
#define CPAL_I2C_STATUS2_EVT_MASK       ((uint16_t)0x0097)  /*!< I2C event Mask for Status Register 2  */
  
 /* I2C Event Defines */  
#define CPAL_I2C_EVT_SB                 I2C_SR1_SB       /*!<Start Bit (Master mode) */
#define CPAL_I2C_EVT_ADDR               I2C_SR1_ADDR     /*!<Address sent (master mode)/matched (slave mode) */
#define CPAL_I2C_EVT_ADD10              I2C_SR1_ADD10    /*!<10-bit header sent (Master mode) */
#define CPAL_I2C_EVT_STOPF              I2C_SR1_STOPF    /*!<Stop detection (Slave mode) */
#define CPAL_I2C_EVT_RXNE               I2C_SR1_RXNE     /*!<Data Register not Empty (receivers) */
#define CPAL_I2C_EVT_TXE                I2C_SR1_TXE      /*!<Data Register Empty (transmitters) */
  
  
/*========= I2C1 specific defines (Clocks and DMA) =========*/   
  
#define CPAL_I2C1_CLK                   RCC_APB1Periph_I2C1
#define CPAL_I2C1_DR                    ((uint32_t)0x40005410)
#define CPAL_I2C1_AF                    GPIO_AF_I2C1  
  
#define CPAL_I2C1_DMA                   DMA1
#define CPAL_I2C1_DMA_CLK               RCC_AHB1Periph_DMA1 
#define CPAL_I2C1_DMA_CHANNEL           DMA_Channel_1  
   
#define CPAL_I2C1_IT_EVT_IRQn           I2C1_EV_IRQn  
#define CPAL_I2C1_IT_ERR_IRQn           I2C1_ER_IRQn   
    
/* TX Direction */
#ifdef CPAL_I2C1_DMA_TX_CONFIG1
 #define CPAL_I2C1_DMA_TX_Stream         DMA1_Stream6
 #define CPAL_I2C1_DMA_TX_IRQn           DMA1_Stream6_IRQn
 #define CPAL_I2C1_DMA_TX_IRQHandler     DMA1_Stream6_IRQHandler
 #define CPAL_I2C1_DMA_TX_TC_FLAG        DMA_FLAG_TCIF6
 #define CPAL_I2C1_DMA_TX_HT_FLAG        DMA_FLAG_HTIF6
 #define CPAL_I2C1_DMA_TX_TE_FLAG        DMA_FLAG_TEIF6
#elif defined (CPAL_I2C1_DMA_TX_CONFIG2)
 #define CPAL_I2C1_DMA_TX_Stream         DMA1_Stream7
 #define CPAL_I2C1_DMA_TX_IRQn           DMA1_Stream7_IRQn
 #define CPAL_I2C1_DMA_TX_IRQHandler     DMA1_Stream7_IRQHandler
 #define CPAL_I2C1_DMA_TX_TC_FLAG        DMA_FLAG_TCIF7
 #define CPAL_I2C1_DMA_TX_HT_FLAG        DMA_FLAG_HTIF7
 #define CPAL_I2C1_DMA_TX_TE_FLAG        DMA_FLAG_TEIF7
#endif
  
/* RX Direction */
#ifdef CPAL_I2C1_DMA_RX_CONFIG1
 #define CPAL_I2C1_DMA_RX_Stream         DMA1_Stream0
 #define CPAL_I2C1_DMA_RX_IRQn           DMA1_Stream0_IRQn
 #define CPAL_I2C1_DMA_RX_IRQHandler     DMA1_Stream0_IRQHandler
 #define CPAL_I2C1_DMA_RX_TC_FLAG        DMA_FLAG_TCIF0
 #define CPAL_I2C1_DMA_RX_HT_FLAG        DMA_FLAG_HTIF0
 #define CPAL_I2C1_DMA_RX_TE_FLAG        DMA_FLAG_TEIF0     
#elif defined (CPAL_I2C1_DMA_RX_CONFIG2)
 #define CPAL_I2C1_DMA_RX_Stream         DMA1_Stream5
 #define CPAL_I2C1_DMA_RX_IRQn           DMA1_Stream5_IRQn
 #define CPAL_I2C1_DMA_RX_IRQHandler     DMA1_Stream5_IRQHandler
 #define CPAL_I2C1_DMA_RX_TC_FLAG        DMA_FLAG_TCIF5
 #define CPAL_I2C1_DMA_RX_HT_FLAG        DMA_FLAG_HTIF5
 #define CPAL_I2C1_DMA_RX_TE_FLAG        DMA_FLAG_TEIF5     
#endif    
 
/*========= I2C2 specific defines (Clocks and DMA) =========*/   
  
#define CPAL_I2C2_CLK                   RCC_APB1Periph_I2C2
#define CPAL_I2C2_DR                    ((uint32_t)0x40005810)
#define CPAL_I2C2_AF                    GPIO_AF_I2C2  
  
#define CPAL_I2C2_DMA                   DMA1
#define CPAL_I2C2_DMA_CLK               RCC_AHB1Periph_DMA1 
#define CPAL_I2C2_DMA_CHANNEL           DMA_Channel_7  
  
#define CPAL_I2C2_IT_EVT_IRQn           I2C2_EV_IRQn  
#define CPAL_I2C2_IT_ERR_IRQn           I2C2_ER_IRQn   

/* TX Direction */
#define CPAL_I2C2_DMA_TX_Stream         DMA1_Stream7
#define CPAL_I2C2_DMA_TX_IRQn           DMA1_Stream7_IRQn
#define CPAL_I2C2_DMA_TX_IRQHandler     DMA1_Stream7_IRQHandler
#define CPAL_I2C2_DMA_TX_TC_FLAG        DMA_FLAG_TCIF7
#define CPAL_I2C2_DMA_TX_HT_FLAG        DMA_FLAG_HTIF7
#define CPAL_I2C2_DMA_TX_TE_FLAG        DMA_FLAG_TEIF7  
  
/* RX Direction */
#ifdef CPAL_I2C2_DMA_RX_CONFIG1
 #define CPAL_I2C2_DMA_RX_Stream         DMA1_Stream3
 #define CPAL_I2C2_DMA_RX_IRQn           DMA1_Stream3_IRQn
 #define CPAL_I2C2_DMA_RX_IRQHandler     DMA1_Stream3_IRQHandler
 #define CPAL_I2C2_DMA_RX_TC_FLAG        DMA_FLAG_TCIF3
 #define CPAL_I2C2_DMA_RX_HT_FLAG        DMA_FLAG_HTIF3
 #define CPAL_I2C2_DMA_RX_TE_FLAG        DMA_FLAG_TEIF3
#elif defined (CPAL_I2C2_DMA_RX_CONFIG2)
 #define CPAL_I2C2_DMA_RX_Stream         DMA1_Stream2
 #define CPAL_I2C2_DMA_RX_IRQn           DMA1_Stream2_IRQn
 #define CPAL_I2C2_DMA_RX_IRQHandler     DMA1_Stream2_IRQHandler
 #define CPAL_I2C2_DMA_RX_TC_FLAG        DMA_FLAG_TCIF2
 #define CPAL_I2C2_DMA_RX_HT_FLAG        DMA_FLAG_HTIF2
 #define CPAL_I2C2_DMA_RX_TE_FLAG        DMA_FLAG_TEIF2 
#endif
   
/*========= I2C3 specific defines (Clocks and DMA) =========*/   
  
#define CPAL_I2C3_CLK                   RCC_APB1Periph_I2C3
#define CPAL_I2C3_DR                    ((uint32_t)0x40005C10)
#define CPAL_I2C3_AF                    GPIO_AF_I2C3  
  
#define CPAL_I2C3_DMA                   DMA1
#define CPAL_I2C3_DMA_CLK               RCC_AHB1Periph_DMA1 
#define CPAL_I2C3_DMA_CHANNEL           DMA_Channel_3  
    
#define CPAL_I2C3_IT_EVT_IRQn           I2C3_EV_IRQn  
#define CPAL_I2C3_IT_ERR_IRQn           I2C3_ER_IRQn   

/* TX Direction */
#define CPAL_I2C3_DMA_TX_Stream         DMA1_Stream4
#define CPAL_I2C3_DMA_TX_IRQn           DMA1_Stream4_IRQn
#define CPAL_I2C3_DMA_TX_IRQHandler     DMA1_Stream4_IRQHandler
#define CPAL_I2C3_DMA_TX_TC_FLAG        DMA_FLAG_TCIF4
#define CPAL_I2C3_DMA_TX_HT_FLAG        DMA_FLAG_HTIF4
#define CPAL_I2C3_DMA_TX_TE_FLAG        DMA_FLAG_TEIF4  
  
/* RX Direction */
#define CPAL_I2C3_DMA_RX_Stream         DMA1_Stream2
#define CPAL_I2C3_DMA_RX_IRQn           DMA1_Stream2_IRQn
#define CPAL_I2C3_DMA_RX_IRQHandler     DMA1_Stream2_IRQHandler
#define CPAL_I2C3_DMA_RX_TC_FLAG        DMA_FLAG_TCIF2
#define CPAL_I2C3_DMA_RX_HT_FLAG        DMA_FLAG_HTIF2
#define CPAL_I2C3_DMA_RX_TE_FLAG        DMA_FLAG_TEIF2 
  
/* Exported macro ------------------------------------------------------------*/  

/*========= Peripheral Clock Command =========*/
  
#define __I2C_CLK_CMD(clk,cmd)                   RCC_APB1PeriphClockCmd((clk),(cmd))

#define __I2C_RCC_RESET(clk)                     RCC_APB1PeriphResetCmd((clk),ENABLE);\
                                                 RCC_APB1PeriphResetCmd((clk),DISABLE)  

  
#define __I2C_GPIO_CLK_CMD(clk,cmd)              RCC_AHB1PeriphClockCmd((clk),(cmd))
    
#define __DMA_CLK_CMD(clk,cmd)                   RCC_AHB1PeriphClockCmd((clk),(cmd))

#define __DMA_RESET_CMD(clk,cmd)                 RCC_AHB1PeriphResetCmd((clk),(cmd)) 
  
  
/*========= DMA =========*/
  
/* DMA channels enable/disable */  

#define __CPAL_I2C_HAL_ENABLE_DMATX(device)      CPAL_I2C_DMA_TX_Stream[(device)]->CR |= DMA_CR_EN  
  
#define __CPAL_I2C_HAL_DISABLE_DMATX(device)     CPAL_I2C_DMA_TX_Stream[(device)]->CR &= ~DMA_CR_EN 
  
#define __CPAL_I2C_HAL_ENABLE_DMARX(device)      CPAL_I2C_DMA_RX_Stream[(device)]->CR |= DMA_CR_EN  

#define __CPAL_I2C_HAL_DISABLE_DMARX(device)     CPAL_I2C_DMA_RX_Stream[(device)]->CR &= ~DMA_CR_EN

/* DMA interrupts enable/disable */  

#define __I2C_HAL_ENABLE_DMATX_TCIT(device)      CPAL_I2C_DMA_TX_Stream[(device)]->CR |= DMA_IT_TC
  
#define __I2C_HAL_ENABLE_DMATX_HTIT(device)      CPAL_I2C_DMA_TX_Stream[(device)]->CR |= DMA_IT_HT
  
#define __I2C_HAL_ENABLE_DMATX_TEIT(device)      CPAL_I2C_DMA_TX_Stream[(device)]->CR |= DMA_IT_TE
  
#define __I2C_HAL_ENABLE_DMARX_TCIT(device)      CPAL_I2C_DMA_RX_Stream[(device)]->CR |= DMA_IT_TC
  
#define __I2C_HAL_ENABLE_DMARX_HTIT(device)      CPAL_I2C_DMA_RX_Stream[(device)]->CR |= DMA_IT_HT
  
#define __I2C_HAL_ENABLE_DMARX_TEIT(device)      CPAL_I2C_DMA_RX_Stream[(device)]->CR |= DMA_IT_TE
  
/* DMA interrupts flag management */

  
#define __CPAL_I2C_HAL_GET_DMATX_TCIT(device)    ((CPAL_I2C_DMA_TX_TC_FLAG[(device)] & DMA_HIGH_ISR_MASK) != 0) ? \
                                                 (uint32_t)(CPAL_I2C_DMA[(device)]->HISR & CPAL_I2C_DMA_TX_TC_FLAG [(device)]) :\
                                                 (uint32_t)(CPAL_I2C_DMA[(device)]->LISR & CPAL_I2C_DMA_TX_TC_FLAG [(device)])
  
#define __CPAL_I2C_HAL_GET_DMATX_HTIT(device)    ((CPAL_I2C_DMA_TX_HT_FLAG[(device)] & DMA_HIGH_ISR_MASK) != 0 ? \
                                                 (uint32_t)(CPAL_I2C_DMA[(device)]->HISR & CPAL_I2C_DMA_TX_HT_FLAG [(device)]):\
                                                 (uint32_t)(CPAL_I2C_DMA[(device)]->LISR & CPAL_I2C_DMA_TX_HT_FLAG [(device)]))

#define __CPAL_I2C_HAL_GET_DMATX_TEIT(device)    ((CPAL_I2C_DMA_TX_TE_FLAG[(device)] & DMA_HIGH_ISR_MASK) != 0 ? \
                                                 (uint32_t)(CPAL_I2C_DMA[(device)]->HISR & CPAL_I2C_DMA_TX_TE_FLAG [(device)]):\
                                                 (uint32_t)(CPAL_I2C_DMA[(device)]->LISR & CPAL_I2C_DMA_TX_TE_FLAG [(device)]))

#define __CPAL_I2C_HAL_GET_DMARX_TCIT(device)    ((CPAL_I2C_DMA_RX_TC_FLAG[(device)] & DMA_HIGH_ISR_MASK) != 0 ? \
                                                 (uint32_t)(CPAL_I2C_DMA[(device)]->HISR & CPAL_I2C_DMA_RX_TC_FLAG [(device)]):\
                                                 (uint32_t)(CPAL_I2C_DMA[(device)]->LISR & CPAL_I2C_DMA_RX_TC_FLAG [(device)]))  

#define __CPAL_I2C_HAL_GET_DMARX_HTIT(device)    ((CPAL_I2C_DMA_RX_HT_FLAG[(device)] & DMA_HIGH_ISR_MASK) != 0 ? \
                                                 (uint32_t)(CPAL_I2C_DMA[(device)]->HISR & CPAL_I2C_DMA_RX_HT_FLAG [(device)]):\
                                                 (uint32_t)(CPAL_I2C_DMA[(device)]->LISR & CPAL_I2C_DMA_RX_HT_FLAG [(device)]))

#define __CPAL_I2C_HAL_GET_DMARX_TEIT(device)    ((CPAL_I2C_DMA_RX_TE_FLAG[(device)] & DMA_HIGH_ISR_MASK) != 0 ? \
                                                 (uint32_t)(CPAL_I2C_DMA[(device)]->HISR & CPAL_I2C_DMA_RX_TE_FLAG [(device)]):\
                                                 (uint32_t)(CPAL_I2C_DMA[(device)]->LISR & CPAL_I2C_DMA_RX_TE_FLAG [(device)]))
  
#define __CPAL_I2C_HAL_CLEAR_DMATX_IT(device)    ((CPAL_I2C_DMA_TX_TC_FLAG[(device)] & DMA_HIGH_ISR_MASK) != 0) ? \
                                                 (CPAL_I2C_DMA[(device)]->HIFCR = (CPAL_I2C_DMA_TX_TC_FLAG[(device)] |\
                                                 CPAL_I2C_DMA_TX_HT_FLAG[(device)] | CPAL_I2C_DMA_TX_TE_FLAG[(device)])) :\
                                                 (CPAL_I2C_DMA[(device)]->LIFCR = (CPAL_I2C_DMA_TX_TC_FLAG[(device)]|\
                                                 CPAL_I2C_DMA_TX_HT_FLAG[(device)] | CPAL_I2C_DMA_TX_TE_FLAG[(device)]))
                                                   
#define __CPAL_I2C_HAL_CLEAR_DMARX_IT(device)    ((CPAL_I2C_DMA_RX_TC_FLAG[(device)] & DMA_HIGH_ISR_MASK) != 0) ? \
                                                 (CPAL_I2C_DMA[(device)]->HIFCR = (CPAL_I2C_DMA_RX_TC_FLAG[(device)] |\
                                                 CPAL_I2C_DMA_RX_HT_FLAG[(device)] | CPAL_I2C_DMA_RX_TE_FLAG[(device)])) :\
                                                 (CPAL_I2C_DMA[(device)]->LIFCR = (CPAL_I2C_DMA_RX_TC_FLAG[(device)]|\
                                                 CPAL_I2C_DMA_RX_HT_FLAG[(device)] | CPAL_I2C_DMA_RX_TE_FLAG[(device)]))
                                                   
/* Get DMA data counter */  

#define __CPAL_I2C_HAL_DMATX_GET_CNDT(device)    (uint32_t)(CPAL_I2C_DMA_TX_Stream[(device)]->NDTR)
 
#define __CPAL_I2C_HAL_DMARX_GET_CNDT(device)    (uint32_t)(CPAL_I2C_DMA_RX_Stream[(device)]->NDTR) 
 

/*========= I2C =========*/  
  
/* I2C enable/disable */  

#define __CPAL_I2C_HAL_ENABLE_DEV(device)        CPAL_I2C_DEVICE[(device)]->CR1 |= I2C_CR1_PE  
  
#define __CPAL_I2C_HAL_DISABLE_DEV(device)       CPAL_I2C_DEVICE[(device)]->CR1 &= ~I2C_CR1_PE  
     
/* I2C software reset */

#define __CPAL_I2C_HAL_SWRST(device)             CPAL_I2C_DEVICE[(device)]->CR1 |= I2C_CR1_SWRST; \
                                                 CPAL_I2C_DEVICE[(device)]->CR1 &= ~I2C_CR1_SWRST     
  
/* I2C interrupts enable/disable */  

#define __CPAL_I2C_HAL_ENABLE_ERRIT(device)      CPAL_I2C_DEVICE[(device)]->CR2 |= I2C_CR2_ITERREN   
   
#define __CPAL_I2C_HAL_DISABLE_ERRIT(device)     CPAL_I2C_DEVICE[(device)]->CR2 &= ~I2C_CR2_ITERREN   
  
#define __CPAL_I2C_HAL_ENABLE_EVTIT(device)      CPAL_I2C_DEVICE[(device)]->CR2 |= I2C_CR2_ITEVTEN   
   
#define __CPAL_I2C_HAL_DISABLE_EVTIT(device)     CPAL_I2C_DEVICE[(device)]->CR2 &= ~I2C_CR2_ITEVTEN 
  
#define __CPAL_I2C_HAL_ENABLE_BUFIT(device)      CPAL_I2C_DEVICE[(device)]->CR2 |= I2C_CR2_ITBUFEN   
   
#define __CPAL_I2C_HAL_DISABLE_BUFIT(device)     CPAL_I2C_DEVICE[(device)]->CR2 &= ~I2C_CR2_ITBUFEN 
  

/* I2C Addressing configuration */  

#define __CPAL_I2C_HAL_OAR2_CONF(device,value)   CPAL_I2C_DEVICE[(device)]->OAR2 &= ((uint16_t)0xFF01);\
                                                 CPAL_I2C_DEVICE[(device)]->OAR2 |= (uint16_t)((value) & 0x00FE)
 

#define __CPAL_I2C_HAL_ENABLE_GENCALL(device)    CPAL_I2C_DEVICE[(device)]->CR1 |= I2C_CR1_ENGC   
       
#define __CPAL_I2C_HAL_ENABLE_DUALADDR(device)   CPAL_I2C_DEVICE[(device)]->OAR2 |= I2C_OAR2_ENDUAL  
  

/* I2C misc configuration */   

#define __CPAL_I2C_HAL_ENABLE_DMAREQ(device)     CPAL_I2C_DEVICE[(device)]->CR2 |= I2C_CR2_DMAEN  
  
#define __CPAL_I2C_HAL_DISABLE_DMAREQ(device)    CPAL_I2C_DEVICE[(device)]->CR2 &= ~I2C_CR2_DMAEN 
  
  
#define __CPAL_I2C_HAL_ENABLE_ACK(device)        CPAL_I2C_DEVICE[(device)]->CR1 |= I2C_CR1_ACK  
  
#define __CPAL_I2C_HAL_DISABLE_ACK(device)       CPAL_I2C_DEVICE[(device)]->CR1 &= ~I2C_CR1_ACK  
  
 
#define __CPAL_I2C_HAL_ENABLE_POS(device)        CPAL_I2C_DEVICE[(device)]->CR1 |= I2C_CR1_POS   
   
#define __CPAL_I2C_HAL_DISABLE_POS(device)       CPAL_I2C_DEVICE[(device)]->CR1 &= ~I2C_CR1_POS 
  
    
#define __CPAL_I2C_HAL_ENABLE_LAST(device)       CPAL_I2C_DEVICE[(device)]->CR2 |= I2C_CR2_LAST   
   
#define __CPAL_I2C_HAL_DISABLE_LAST(device)      CPAL_I2C_DEVICE[(device)]->CR2 &= ~I2C_CR2_LAST   
  
  
#define __CPAL_I2C_HAL_ENABLE_NOSTRETCH(device)  CPAL_I2C_DEVICE[(device)]->CR1 |= I2C_CR1_NOSTRETCH   
   
#define __CPAL_I2C_HAL_DISABLE_NOSTRETCH(device) CPAL_I2C_DEVICE[(device)]->CR1 &= ~I2C_CR1_NOSTRETCH   
 

#define __CPAL_I2C_HAL_START(device)             CPAL_I2C_DEVICE[(device)]->CR1 |= I2C_CR1_START
  
#define __CPAL_I2C_HAL_STOP(device)              CPAL_I2C_DEVICE[(device)]->CR1 |= I2C_CR1_STOP 


/* I2C data management */

#define __CPAL_I2C_HAL_RECEIVE(device)           (uint8_t)(CPAL_I2C_DEVICE[(device)]->DR) 

#define __CPAL_I2C_HAL_SEND(device,value)        CPAL_I2C_DEVICE[(device)]->DR = (uint8_t)((value)) 
  

/* I2C flags management */

#define __CPAL_I2C_HAL_GET_EVENT(device)         (uint16_t)(CPAL_I2C_DEVICE[(device)]->SR1 & CPAL_I2C_STATUS1_EVT_MASK)

#define __CPAL_I2C_HAL_GET_ERROR(device)         (uint16_t)(CPAL_I2C_DEVICE[(device)]->SR1 & CPAL_I2C_STATUS_ERR_MASK)
  
#define __CPAL_I2C_HAL_GET_SB(device)            (uint16_t)(CPAL_I2C_DEVICE[(device)]->SR1 & I2C_SR1_SB) 

#define __CPAL_I2C_HAL_GET_ADDR(device)          (uint16_t)(CPAL_I2C_DEVICE[(device)]->SR1 & I2C_SR1_ADDR) 
  
#define __CPAL_I2C_HAL_GET_ADD10(device)         (uint16_t)(CPAL_I2C_DEVICE[(device)]->SR1 & I2C_SR1_ADD10) 
  
#define __CPAL_I2C_HAL_GET_STOPF(device)         (uint16_t)(CPAL_I2C_DEVICE[(device)]->SR1 & I2C_SR1_STOPF) 
  
#define __CPAL_I2C_HAL_GET_BTF(device)           (uint16_t)(CPAL_I2C_DEVICE[(device)]->SR1 & I2C_SR1_BTF) 
  
#define __CPAL_I2C_HAL_GET_TXE(device)           (uint16_t)(CPAL_I2C_DEVICE[(device)]->SR1 & I2C_SR1_TXE) 
  
#define __CPAL_I2C_HAL_GET_RXNE(device)          (uint16_t)(CPAL_I2C_DEVICE[(device)]->SR1 & I2C_SR1_RXNE) 
  
#define __CPAL_I2C_HAL_GET_BUSY(device)          (uint16_t)(CPAL_I2C_DEVICE[(device)]->SR2 & I2C_SR2_BUSY) 
  
#define __CPAL_I2C_HAL_GET_GENCALL(device)       (uint16_t)(CPAL_I2C_DEVICE[(device)]->SR2 & I2C_SR2_GENCALL) 
  
#define __CPAL_I2C_HAL_GET_DUALF(device)         (uint16_t)(CPAL_I2C_DEVICE[(device)]->SR2 & I2C_SR2_DUALF) 

#define __CPAL_I2C_HAL_GET_TRA(device)           (uint16_t)(CPAL_I2C_DEVICE[(device)]->SR2 & I2C_SR2_TRA)
  
#define __CPAL_I2C_HAL_GET_OVR(device)           (uint16_t)(CPAL_I2C_DEVICE[(device)]->SR1 & I2C_SR1_OVR) 
  
#define __CPAL_I2C_HAL_GET_AF(device)            (uint16_t)(CPAL_I2C_DEVICE[(device)]->SR1 & I2C_SR1_AF) 
  
#define __CPAL_I2C_HAL_GET_ARLO(device)          (uint16_t)(CPAL_I2C_DEVICE[(device)]->SR1 & I2C_SR1_ARLO) 
  
#define __CPAL_I2C_HAL_GET_BERR(device)          (uint16_t)(CPAL_I2C_DEVICE[(device)]->SR1 & I2C_SR1_BERR)   

#define __CPAL_I2C_HAL_CLEAR_SB(device)          CPAL_I2C_DEVICE[(device)]->SR1;\
                                                 CPAL_I2C_DEVICE[(device)]->SR2

#define __CPAL_I2C_HAL_CLEAR_ADDR(device)        CPAL_I2C_DEVICE[(device)]->SR1;\
                                                 CPAL_I2C_DEVICE[(device)]->SR2

#define __CPAL_I2C_HAL_CLEAR_BTF(device)         CPAL_I2C_DEVICE[(device)]->SR1;\
                                                 CPAL_I2C_DEVICE[(device)]->DR 

#define __CPAL_I2C_HAL_CLEAR_STOPF(device)       CPAL_I2C_DEVICE[(device)]->SR1;\
                                                 CPAL_I2C_DEVICE[(device)]->CR1 |= I2C_CR1_PE 

#define __CPAL_I2C_HAL_CLEAR_AF(device)          CPAL_I2C_DEVICE[(device)]->SR1 = ~I2C_SR1_AF  

#define __CPAL_I2C_HAL_CLEAR_ERROR(device)       CPAL_I2C_DEVICE[(device)]->SR1 = ~CPAL_I2C_STATUS_ERR_MASK
  

/* Exported functions --------------------------------------------------------*/    
  
/*========= I2CX IRQHandler =========*/      

#ifdef CPAL_USE_I2C1
   uint32_t I2C1_EV_IRQHandler(void); /*<!I2C1 Event Interrupt Handler : handle Communication of I2C1 Device */
   uint32_t I2C1_ER_IRQHandler(void); /*<!I2C1 Error Interrupt Handler : handle error of I2C1 Device if Error Interrupt enabled */
#endif /* CPAL_USE_I2C1 */
   
#ifdef CPAL_USE_I2C2
   uint32_t I2C2_EV_IRQHandler(void); /*<!I2C2 Event Interrupt Handler  : handle Communication of I2C2 Device */
   uint32_t I2C2_ER_IRQHandler(void); /*<!I2C2 Error Interrupt Handler  : handle error of I2C2 Device if Error Interrupt enabled */
#endif /* CPAL_USE_I2C2 */

#ifdef CPAL_USE_I2C3
   uint32_t I2C3_EV_IRQHandler(void); /*<!I2C3 Event Interrupt Handler  : handle Communication of I2C3 Device */
   uint32_t I2C3_ER_IRQHandler(void); /*<!I2C3 Error Interrupt Handler  : handle error of I2C3 Device if Error Interrupt enabled */
#endif /* CPAL_USE_I2C3 */
   
#ifdef CPAL_I2C_DMA_PROGMODEL   

/*========= DMA I2CX IRQHandler =========*/      

#ifdef CPAL_USE_I2C1
   uint32_t CPAL_I2C1_DMA_TX_IRQHandler(void); /*<!I2C1 DMA TX Interrupt Handler : handle data Transmission with DMA */
   uint32_t CPAL_I2C1_DMA_RX_IRQHandler(void); /*<!I2C1 DMA RX Interrupt Handler : handle data reception with DMA */
#endif /* CPAL_USE_I2C1 */

#ifdef CPAL_USE_I2C2
   uint32_t CPAL_I2C2_DMA_TX_IRQHandler(void); /*<!I2C2 DMA TX Interrupt Handler : handle data Transmission with DMA */
   uint32_t CPAL_I2C2_DMA_RX_IRQHandler(void); /*<!I2C2 DMA RX Interrupt Handler : handle data reception with DMA */
#endif /* CPAL_USE_I2C2 */
   
#ifdef CPAL_USE_I2C3
   uint32_t CPAL_I2C3_DMA_TX_IRQHandler(void); /*<!I2C3 DMA TX Interrupt Handler : handle data Transmission with DMA */
   uint32_t CPAL_I2C3_DMA_RX_IRQHandler(void); /*<!I2C3 DMA RX Interrupt Handler : handle data reception with DMA */
#endif /* CPAL_USE_I2C3 */
   
#endif /* CPAL_I2C_DMA_PROGMODEL */   

/*========= Hardware Abstraction Layer local =========*/      

  void CPAL_I2C_HAL_CLKInit(CPAL_DevTypeDef Device); /*<!This function resets then enable the I2C device clock */
  
  void CPAL_I2C_HAL_CLKDeInit(CPAL_DevTypeDef Device); /*<!This function resets then disable the I2C device clock */
  
  void CPAL_I2C_HAL_GPIOInit(CPAL_DevTypeDef Device);  /*<!This function configures the IO pins used by the I2C device */
  
  void CPAL_I2C_HAL_GPIODeInit(CPAL_DevTypeDef Device); /*<!This function deinitialize the IO pins used by the I2C device 
                                                            (configured to their default state) */ 
  
#ifdef CPAL_I2C_DMA_PROGMODEL
  void CPAL_I2C_HAL_DMAInit(CPAL_DevTypeDef Device, CPAL_DirectionTypeDef Direction, uint32_t Options); /*<!This function enable the DMA clock and initialize 
                                                                                                            needed DMA Streams used by the I2C device   */
  
  void CPAL_I2C_HAL_DMATXConfig(CPAL_DevTypeDef Device,CPAL_TransferTypeDef* TransParameter, uint32_t Options); /*<!This function configures the DMA Stream specific
                                                                                                                    for TX transfer */
  
  void CPAL_I2C_HAL_DMARXConfig(CPAL_DevTypeDef Device,CPAL_TransferTypeDef* TransParameter, uint32_t Options); /*<!This function configures the DMA Stream specific
                                                                                                                    for RX transfer */
  
  void CPAL_I2C_HAL_DMADeInit(CPAL_DevTypeDef Device, CPAL_DirectionTypeDef Direction); /*<!This function deinitialize the DMA Stream used 
                                                                                            by I2C Device (Configure them to their default
                                                                                            values). DMA clock is not disabled */
#endif /* CPAL_I2C_DMA_PROGMODEL */
  
  void CPAL_I2C_HAL_ITInit(CPAL_DevTypeDef Device, uint32_t Options, CPAL_DirectionTypeDef Direction, CPAL_ProgModelTypeDef CPAL_ProgModel); /*<!This function configures NVIC and interrupts used 
                                                                                                                                                 by I2C Device according to enabled options */  
  
  void CPAL_I2C_HAL_ITDeInit(CPAL_DevTypeDef Device, uint32_t Options, CPAL_DirectionTypeDef Direction, CPAL_ProgModelTypeDef ProgModel); /*<!This function deinitialize NVIC and interrupts used 
                                                                                                                                              by I2C Device  */  
   

  
#ifdef __cplusplus
}
#endif

#endif /*___CPAL_I2C_HAL_STM32F2XX_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


