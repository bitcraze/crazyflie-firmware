/** \mainpage
*
****************************************************************************
* Copyright (C) 2010 - 2015 Bosch Sensortec GmbH
*
* File : bmg160.h
*
* Date : 2015/04/29
*
* Revision : 2.0.4 $
*
* Usage: Sensor Driver for BMG160 sensor
*
****************************************************************************
*
* \section License
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*   Redistributions of source code must retain the above copyright
*   notice, this list of conditions and the following disclaimer.
*
*   Redistributions in binary form must reproduce the above copyright
*   notice, this list of conditions and the following disclaimer in the
*   documentation and/or other materials provided with the distribution.
*
*   Neither the name of the copyright holder nor the names of the
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
* OR CONTRIBUTORS BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
* OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
* PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*
* The information provided is believed to be accurate and reliable.
* The copyright holder assumes no responsibility
* for the consequences of use
* of such information nor for any infringement of patents or
* other rights of third parties which may result from its use.
* No license is granted by implication or otherwise under any patent or
* patent rights of the copyright holder.
**************************************************************************/
/*! \file BMG160.h
    \brief Header for BMG160 API */
/* user defined code to be added here ... */
#ifndef __BMG160_H__
#define __BMG160_H__

#include "bstdr_comm_support.h"
#include "bstdr_types.h"

/***************************************************************/
/**\name	DEVICE ADDRESS OF BMG160       */
/***************************************************************/
#define	BMG160_I2C_ADDR1				(0x68)
#define	BMG160_I2C_ADDR2				(0x69)

/***************************************************************/
/**\name	REGISTER ADDRESS DEFINITION        */
/***************************************************************/

/*******************************************/
/**\name	CHIP ID       */
/*******************************************/
#define BMG160_CHIP_ID_ADDR						 (0x00)
/**<Address of Chip ID Register*/
/*******************************************/
/**\name	DATA REGISTER       */
/*******************************************/
#define BMG160_RATE_X_LSB_ADDR                     (0x02)
/**<        Address of X axis Rate LSB Register       */
#define BMG160_RATE_X_MSB_ADDR                     (0x03)
/**<        Address of X axis Rate MSB Register       */
#define BMG160_RATE_Y_LSB_ADDR                     (0x04)
/**<        Address of Y axis Rate LSB Register       */
#define BMG160_RATE_Y_MSB_ADDR                     (0x05)
/**<        Address of Y axis Rate MSB Register       */
#define BMG160_RATE_Z_LSB_ADDR                     (0x06)
/**<        Address of Z axis Rate LSB Register       */
#define BMG160_RATE_Z_MSB_ADDR                     (0x07)
/**<        Address of Z axis Rate MSB Register       */
#define BMG160_TEMP_ADDR                           (0x08)
/**<        Address of Temperature Data LSB Register  */
/*******************************************/
/**\name	STATUS REGISTER     */
/*******************************************/
#define BMG160_INTR_STAT0_ADDR                     (0x09)
/**<        Address of Interrupt status Register    */
#define BMG160_INTR_STAT1_ADDR                     (0x0A)
/**<        Address of Interrupt status Register     */
#define BMG160_INTR_STAT2_ADDR                     (0x0B)
/**<        Address of Interrupt status Register     */
#define BMG160_INTR_STAT3_ADDR                     (0x0C)
/**<        Address of Interrupt status Register     */
#define BMG160_FIFO_STAT_ADDR                      (0x0E)
/**<        Address of FIFO status Register           */
/*******************************************/
/**\name	CONTROL REGISTER    */
/*******************************************/
#define BMG160_RANGE_ADDR                         (0x0F)
/**<        Address of Range address Register     */
#define BMG160_BW_ADDR                            (0x10)
/**<        Address of Bandwidth Register         */
#define BMG160_MODE_LPM1_ADDR                     (0x11)
/**<        Address of Mode LPM1 Register         */
#define BMG160_MODE_LPM2_ADDR                     (0x12)
/**<        Address of Mode LPM2 Register         */
#define BMG160_HIGH_BW_ADDR                       (0x13)
/**<        Address of Rate HIGH_BW Register       */
#define BMG160_BGW_SOFT_RST_ADDR                  (0x14)
/**<        Address of BGW Softreset Register      */
/*******************************************/
/**\name	INTERRUPT STATUS REGISTERS    */
/*******************************************/
#define BMG160_INTR_ENABLE0_ADDR                  (0x15)
/**<        Address of Interrupt Enable             */
#define BMG160_INTR_ENABLE1_ADDR                  (0x16)
/**<        Address of Interrupt Enable             */
#define BMG160_INTR_MAP_ZERO_ADDR                 (0x17)
/**<        Address of Interrupt MAP                */
#define BMG160_INTR_MAP_ONE_ADDR                  (0x18)
/**<        Address of Interrupt MAP                */
#define BMG160_INTR_MAP_TWO_ADDR                  (0x19)
/**<        Address of Interrupt MAP                */
#define BMG160_INTR_ZERO_ADDR                     (0x1A)
/**<        Address of Interrupt  register   */
#define BMG160_INTR_ONE_ADDR                      (0x1B)
/**<        Address of Interrupt  register   */
#define BMG160_INTR_TWO_ADDR                      (0x1C)
/**<        Address of Interrupt  register   */
#define BMG160_INTR_4_ADDR                        (0x1E)
/**<        Address of Interrupt register   */
#define BMG160_RST_LATCH_ADDR                     (0x21)
/**<        Address of Reset Latch Register           */
/***********************************************/
/**\name	INTERRUPT HIGH RATE CONFIGURATION REGISTER  */
/***********************************************/
#define BMG160_HIGHRATE_THRES_X_ADDR              (0x22)
/**<        Address of High Th x Address register     */
#define BMG160_HIGHRATE_DURN_X_ADDR               (0x23)
/**<        Address of High Dur x Address register    */
#define BMG160_HIGHRATE_THRES_Y_ADDR              (0x24)
/**<        Address of High Th y  Address register    */
#define BMG160_HIGHRATE_DURN_Y_ADDR               (0x25)
/**<        Address of High Dur y Address register    */
#define BMG160_HIGHRATE_THRES_Z_ADDR              (0x26)
/**<        Address of High Th z Address register  */
#define BMG160_HIGHRATE_DURN_Z_ADDR               (0x27)
/**<        Address of High Dur z Address register  */
#define BMG160_SOC_ADDR                           (0x31)
/**<        Address of SOC register        */
/***********************************************/
/**\name	OFFSET REGISTER  */
/***********************************************/
#define BMG160_A_FOC_ADDR                         (0x32)
/**<        Address of A_FOC Register        */
/***********************************************/
/**\name	NVM CONTROL REGISTER  */
/***********************************************/
#define BMG160_TRIM_NVM_CTRL_ADDR                 (0x33)
/**<        Address of Trim NVM control register    */
#define BMG160_BGW_SPI3_WDT_ADDR                  (0x34)
/**<        Address of BGW SPI3,WDT Register           */

/***********************************************/
/**\name	OFFSET OCNFIGURATION REGISTER  */
/***********************************************/
/* Trim Register */
#define BMG160_OFFSET_OFC1_ADDR            (0x36)
/**<        Address of OFC1 Register          */
#define BMG160_OFC2_ADDR                   (0x37)
/**<        Address of OFC2 Register          */
#define BMG160_OFC3_ADDR                   (0x38)
/**<        Address of OFC3 Register          */
#define BMG160_OFC4_ADDR                   (0x39)
/**<        Address of OFC4 Register          */
#define BMG160_TRIM_GP0_ADDR               (0x3A)
/**<        Address of Trim GP0 Register              */
#define BMG160_TRIM_GP1_ADDR               (0x3B)
/**<        Address of Trim GP1 Register              */
/***********************************************/
/**\name	SELFTEST REGISTER  */
/***********************************************/
#define BMG160_SELFTEST_ADDR            (0x3C)
/**<        Address of BGW Self test Register           */
/***********************************************/
/**\name	FIFO REGISTER  */
/***********************************************/
/* Control Register */
#define BMG160_FIFO_CGF1_ADDR              (0x3D)
/**<        Address of FIFO CGF0 Register             */
#define BMG160_FIFO_CGF0_ADDR              (0x3E)
/**<        Address of FIFO CGF1 Register             */

/* Data Register */
#define BMG160_FIFO_DATA_ADDR              (0x3F)
/**<        Address of FIFO Data Register             */



/***********************************************/
/**\name	BIT LENGTH, MASK, POSITION FOR THE DATA REGISTERS  */
/***********************************************/
/* Rate X LSB Register */
#define BMG160_RATE_X_LSB_BIT__POS        (0)
/**< Last (8) bits of RateX LSB Registers */
#define BMG160_RATE_X_LSB_BIT__LEN        (8)
#define BMG160_RATE_X_LSB_BIT__MSK        (0xFF)
#define BMG160_RATE_X_LSB_BIT__REG        (BMG160_RATE_X_LSB_ADDR)

/* Rate Y LSB Register */
/**<  Last (8) bits of RateY LSB Registers */
#define BMG160_RATE_Y_LSB_BIT__POS        (0)
#define BMG160_RATE_Y_LSB_BIT__LEN        (8)
#define BMG160_RATE_Y_LSB_BIT__MSK        (0xFF)
#define BMG160_RATE_Y_LSB_BIT__REG        (BMG160_RATE_Y_LSB_ADDR)

/* Rate Z LSB Register */
/**< Last (8) bits of RateZ LSB Registers */
#define BMG160_RATE_Z_LSB_BIT__POS        (0)
#define BMG160_RATE_Z_LSB_BIT__LEN        (8)
#define BMG160_RATE_Z_LSB_BIT__MSK        (0xFF)
#define BMG160_RATE_Z_LSB_BIT__REG        (BMG160_RATE_Z_LSB_ADDR)
/***********************************************/
/**\name	BIT LENGTH, MASK,
POSITION FOR THE INTERRUPT STATUS REGISTERS  */
/***********************************************/
/* Interrupt status (0) Register */
   /**< 2th bit of Interrupt status  register */
#define BMG160_INTR_STAT0_ANY_MOTION_INTR__POS     (2)
#define BMG160_INTR_STAT0_ANY_MOTION_INTR__LEN     (1)
#define BMG160_INTR_STAT0_ANY_MOTION_INTR__MSK     (0x04)
#define BMG160_INTR_STAT0_ANY_MOTION_INTR__REG     (BMG160_INTR_STAT0_ADDR)

/**< 1st bit of Interrupt status register */
#define BMG160_INTR_STAT0_HIGHRATE_INTR__POS    (1)
#define BMG160_INTR_STAT0_HIGHRATE_INTR__LEN    (1)
#define BMG160_INTR_STAT0_HIGHRATE_INTR__MSK    (0x02)
#define BMG160_INTR_STAT0_HIGHRATE_INTR__REG    (BMG160_INTR_STAT0_ADDR)

 /**< 1st and 2nd bit of Interrupt status  register */
#define BMG160_INTR_STAT_ZERO__POS    (1)
#define BMG160_INTR_STAT_ZERO__LEN    (2)
#define BMG160_INTR_STAT_ZERO__MSK    (0x06)
#define BMG160_INTR_STAT_ZERO__REG    (BMG160_INTR_STAT0_ADDR)

/* Interrupt status (1) Register */
/**< 7th bit of Interrupt status  register */
#define BMG160_INTR_STAT1_DATA_INTR__POS           (7)
#define BMG160_INTR_STAT1_DATA_INTR__LEN           (1)
#define BMG160_INTR_STAT1_DATA_INTR__MSK           (0x80)
#define BMG160_INTR_STAT1_DATA_INTR__REG           (BMG160_INTR_STAT1_ADDR)
/***********************************************/
/**\name	BIT LENGTH, MASK, POSITION FOR THE OFFSET STATUS REGISTERS  */
/***********************************************/
 /**< 6th bit of Interrupt status  register */
#define BMG160_INTR_STAT1_AUTO_OFFSET_INTR__POS    (6)
#define BMG160_INTR_STAT1_AUTO_OFFSET_INTR__LEN    (1)
#define BMG160_INTR_STAT1_AUTO_OFFSET_INTR__MSK    (0x40)
#define BMG160_INTR_STAT1_AUTO_OFFSET_INTR__REG    (BMG160_INTR_STAT1_ADDR)

/**< 5th bit of Interrupt status  register */
#define BMG160_INTR_STAT1_FAST_OFFSET_INTR__POS    (5)
#define BMG160_INTR_STAT1_FAST_OFFSET_INTR__LEN    (1)
#define BMG160_INTR_STAT1_FAST_OFFSET_INTR__MSK    (0x20)
#define BMG160_INTR_STAT1_FAST_OFFSET_INTR__REG    (BMG160_INTR_STAT1_ADDR)
/***********************************************/
/**\name	BIT LENGTH, MASK, POSITION FOR THE FIFO STATUS REGISTERS  */
/***********************************************/
/**< 4th bit of Interrupt status  register */
#define BMG160_INTR_STAT1_FIFO_INTR__POS           (4)
#define BMG160_INTR_STAT1_FIFO_INTR__LEN           (1)
#define BMG160_INTR_STAT1_FIFO_INTR__MSK           (0x10)
#define BMG160_INTR_STAT1_FIFO_INTR__REG           (BMG160_INTR_STAT1_ADDR)

/**< MSB (4) bits of Interrupt status1 register */
#define BMG160_INTR_STAT_ONE__POS           (4)
#define BMG160_INTR_STAT_ONE__LEN           (4)
#define BMG160_INTR_STAT_ONE__MSK           (0xF0)
#define BMG160_INTR_STAT_ONE__REG           (BMG160_INTR_STAT1_ADDR)
/***********************************************/
/**\name	BIT LENGTH, MASK, POSITION FOR
THE ANY MOTION CONFIGURATION REGISTERS  */
/***********************************************/
/* Interrupt status (2) Register */
/**< 3th bit of Interrupt status  register */
#define BMG160_INTR_STAT2_ANY_MOTION_SIGN_INTR__POS     (3)
#define BMG160_INTR_STAT2_ANY_MOTION_SIGN_INTR__LEN     (1)
#define BMG160_INTR_STAT2_ANY_MOTION_SIGN_INTR__MSK     (0x08)
#define BMG160_INTR_STAT2_ANY_MOTION_SIGN_INTR__REG     (BMG160_INTR_STAT2_ADDR)

/**< 2th bit of Interrupt status register */
#define BMG160_INTR_STAT2_ANY_MOTION_FIRSTZ_INTR__POS   (2)
#define BMG160_INTR_STAT2_ANY_MOTION_FIRSTZ_INTR__LEN   (1)
#define BMG160_INTR_STAT2_ANY_MOTION_FIRSTZ_INTR__MSK   (0x04)
#define BMG160_INTR_STAT2_ANY_MOTION_FIRSTZ_INTR__REG   (BMG160_INTR_STAT2_ADDR)

/**< 1st bit of Interrupt status  register */
#define BMG160_INTR_STAT2_ANY_MOTION_FIRSTY_INTR__POS   (1)
#define BMG160_INTR_STAT2_ANY_MOTION_FIRSTY_INTR__LEN   (1)
#define BMG160_INTR_STAT2_ANY_MOTION_FIRSTY_INTR__MSK   (0x02)
#define BMG160_INTR_STAT2_ANY_MOTION_FIRSTY_INTR__REG   (BMG160_INTR_STAT2_ADDR)

/**< 0th bit of Interrupt status register */
#define BMG160_INTR_STAT2_ANY_MOTION_FIRSTX_INTR__POS   (0)
#define BMG160_INTR_STAT2_ANY_MOTION_FIRSTX_INTR__LEN   (1)
#define BMG160_INTR_STAT2_ANY_MOTION_FIRSTX_INTR__MSK   (0x01)
#define BMG160_INTR_STAT2_ANY_MOTION_FIRSTX_INTR__REG   (BMG160_INTR_STAT2_ADDR)

/**< (4) bits of Interrupt status  register */
#define BMG160_INTR_STAT_TWO__POS   (0)
#define BMG160_INTR_STAT_TWO__LEN   (4)
#define BMG160_INTR_STAT_TWO__MSK   (0x0F)
#define BMG160_INTR_STAT_TWO__REG   (BMG160_INTR_STAT2_ADDR)
/***********************************************/
/**\name	BIT LENGTH, MASK, POSITION FOR
THE HIGH RATE XYZ SIGN REGISTERS  */
/***********************************************/
/* Interrupt status (3) Register */
/**< 3th bit of Interrupt status  register */
#define BMG160_INTR_STAT3_HIGHRATE_SIGN_INTR__POS     (3)
#define BMG160_INTR_STAT3_HIGHRATE_SIGN_INTR__LEN     (1)
#define BMG160_INTR_STAT3_HIGHRATE_SIGN_INTR__MSK     (0x08)
#define BMG160_INTR_STAT3_HIGHRATE_SIGN_INTR__REG     (BMG160_INTR_STAT3_ADDR)

/**< 2th bit of Interrupt status  register */
#define BMG160_INTR_STAT3_HIGHRATE_FIRSTZ_INTR__POS   (2)
#define BMG160_INTR_STAT3_HIGHRATE_FIRSTZ_INTR__LEN   (1)
#define BMG160_INTR_STAT3_HIGHRATE_FIRSTZ_INTR__MSK   (0x04)
#define BMG160_INTR_STAT3_HIGHRATE_FIRSTZ_INTR__REG  (BMG160_INTR_STAT3_ADDR)

/**< 1st bit of Interrupt status  register */
#define BMG160_INTR_STAT3_HIGHRATE_FIRSTY_INTR__POS   (1)
#define BMG160_INTR_STAT3_HIGHRATE_FIRSTY_INTR__LEN   (1)
#define BMG160_INTR_STAT3_HIGHRATE_FIRSTY_INTR__MSK   (0x02)
#define BMG160_INTR_STAT3_HIGHRATE_FIRSTY_INTR__REG   (BMG160_INTR_STAT3_ADDR)

/**< 0th bit of Interrupt status  register */
#define BMG160_INTR_STAT3_HIGHRATE_FIRSTX_INTR__POS   (0)
#define BMG160_INTR_STAT3_HIGHRATE_FIRSTX_INTR__LEN   (1)
#define BMG160_INTR_STAT3_HIGHRATE_FIRSTX_INTR__MSK   (0x01)
#define BMG160_INTR_STAT3_HIGHRATE_FIRSTX_INTR__REG   (BMG160_INTR_STAT3_ADDR)

/**< LSB (4) bits of Interrupt status  register */
#define BMG160_INTR_STAT_THREE__POS   (0)
#define BMG160_INTR_STAT_THREE__LEN   (4)
#define BMG160_INTR_STAT_THREE__MSK   (0x0F)
#define BMG160_INTR_STAT_THREE__REG   (BMG160_INTR_STAT3_ADDR)
/***********************************************/
/**\name	BIT LENGTH, MASK, POSITION FOR THE FIFO OVERRUN  */
/***********************************************/
/* BMG160 FIFO Status Register */
/**< 7th bit of FIFO status Register */
#define BMG160_FIFO_STAT_OVERRUN__POS         (7)
#define BMG160_FIFO_STAT_OVERRUN__LEN         (1)
#define BMG160_FIFO_STAT_OVERRUN__MSK         (0x80)
#define BMG160_FIFO_STAT_OVERRUN__REG         (BMG160_FIFO_STAT_ADDR)
/***********************************************/
/**\name	BIT LENGTH, MASK, POSITION FOR THE FIFO FRAME COUNT  */
/***********************************************/
/**< First (7) bits of FIFO status Register */
#define BMG160_FIFO_STAT_FRAME_COUNTER__POS   (0)
#define BMG160_FIFO_STAT_FRAME_COUNTER__LEN   (7)
#define BMG160_FIFO_STAT_FRAME_COUNTER__MSK   (0x7F)
#define BMG160_FIFO_STAT_FRAME_COUNTER__REG   (BMG160_FIFO_STAT_ADDR)
/***********************************************/
/**\name	BIT LENGTH, MASK, POSITION FOR THE RANGE  */
/***********************************************/
/**< First (3) bits of range Registers */
#define BMG160_RANGE_ADDR_RANGE__POS           (0)
#define BMG160_RANGE_ADDR_RANGE__LEN           (3)
#define BMG160_RANGE_ADDR_RANGE__MSK           (0x07)
#define BMG160_RANGE_ADDR_RANGE__REG           (BMG160_RANGE_ADDR)
/***********************************************/
/**\name	BIT LENGTH, MASK, POSITION FOR THE BANDWIDTH  */
/***********************************************/
/**< First (3) bits of Bandwidth Registers */
#define BMG160_BW_ADDR__POS             (0)
#define BMG160_BW_ADDR__LEN             (3)
#define BMG160_BW_ADDR__MSK             (0x07)
#define BMG160_BW_ADDR__REG             (BMG160_BW_ADDR)

/**< 5th and 7th bit of LPM1 Register */
#define BMG160_MODE_LPM1__POS             (5)
#define BMG160_MODE_LPM1__LEN             (3)
#define BMG160_MODE_LPM1__MSK             (0xA0)
#define BMG160_MODE_LPM1__REG             (BMG160_MODE_LPM1_ADDR)
/***********************************************/
/**\name	BIT LENGTH, MASK, POSITION FOR THE POWER MODE  */
/***********************************************/
/**< 1st to 3rd bit of LPM1 Register */
#define BMG160_MODELPM1_ADDR_SLEEP_DURN__POS              (1)
#define BMG160_MODELPM1_ADDR_SLEEP_DURN__LEN              (3)
#define BMG160_MODELPM1_ADDR_SLEEP_DURN__MSK              (0x0E)
#define BMG160_MODELPM1_ADDR_SLEEP_DURN__REG              \
(BMG160_MODE_LPM1_ADDR)

/**< 7th bit of Mode LPM2 Register */
#define BMG160_MODE_LPM2_ADDR_FAST_POWERUP__POS         (7)
#define BMG160_MODE_LPM2_ADDR_FAST_POWERUP__LEN         (1)
#define BMG160_MODE_LPM2_ADDR_FAST_POWERUP__MSK         (0x80)
#define BMG160_MODE_LPM2_ADDR_FAST_POWERUP__REG         \
(BMG160_MODE_LPM2_ADDR)

/**< 6th bit of Mode LPM2 Register */
#define BMG160_MODE_LPM2_ADDR_ADV_POWERSAVING__POS      (6)
#define BMG160_MODE_LPM2_ADDR_ADV_POWERSAVING__LEN      (1)
#define BMG160_MODE_LPM2_ADDR_ADV_POWERSAVING__MSK      (0x40)
#define BMG160_MODE_LPM2_ADDR_ADV_POWERSAVING__REG      \
(BMG160_MODE_LPM2_ADDR)

/**< 4th & 5th bit of Mode LPM2 Register */
#define BMG160_MODE_LPM2_ADDR_EXT_TRI_SELECT__POS          (4)
#define BMG160_MODE_LPM2_ADDR_EXT_TRI_SELECT__LEN          (2)
#define BMG160_MODE_LPM2_ADDR_EXT_TRI_SELECT__MSK          (0x30)
#define BMG160_MODE_LPM2_ADDR_EXT_TRI_SELECT__REG          \
(BMG160_MODE_LPM2_ADDR)
/***********************************************/
/**\name	BIT LENGTH, MASK, POSITION FOR THE AUTO SLEEP DURATION  */
/***********************************************/
/**< 0th to 2nd bit of LPM2 Register */
#define BMG160_MODE_LPM2_ADDR_AUTO_SLEEP_DURN__POS  (0)
#define BMG160_MODE_LPM2_ADDR_AUTO_SLEEP_DURN__LEN  (3)
#define BMG160_MODE_LPM2_ADDR_AUTO_SLEEP_DURN__MSK  (0x07)
#define BMG160_MODE_LPM2_ADDR_AUTO_SLEEP_DURN__REG  (BMG160_MODE_LPM2_ADDR)
/***********************************************/
/**\name	BIT LENGTH, MASK, POSITION FOR THE HIGH BANDWIDTH  */
/***********************************************/
/**< 7th bit of HIGH_BW Register */
#define BMG160_HIGH_BW__POS         (7)
#define BMG160_HIGH_BW__LEN         (1)
#define BMG160_HIGH_BW__MSK         (0x80)
#define BMG160_HIGH_BW__REG         (BMG160_HIGH_BW_ADDR)

/**< 6th bit of HIGH_BW Register */
#define BMG160_SHADOW_DIS__POS          (6)
#define BMG160_SHADOW_DIS__LEN          (1)
#define BMG160_SHADOW_DIS__MSK          (0x40)
#define BMG160_SHADOW_DIS__REG          (BMG160_HIGH_BW_ADDR)
/***********************************************/
/**\name	BIT LENGTH, MASK, POSITION FOR THE DATA INTERRUPT  */
/***********************************************/
/**< 7th bit of Interrupt Enable Registers */
#define BMG160_INTR_ENABLE0_DATA__POS               (7)
#define BMG160_INTR_ENABLE0_DATA__LEN               (1)
#define BMG160_INTR_ENABLE0_DATA__MSK               (0x80)
#define BMG160_INTR_ENABLE0_DATA__REG               (BMG160_INTR_ENABLE0_ADDR)
/***********************************************/
/**\name	BIT LENGTH, MASK, POSITION FOR THE FIFO INTERRUPT  */
/***********************************************/
/**< 6th bit of Interrupt Enable Registers */
#define BMG160_INTR_ENABLE0_FIFO__POS               (6)
#define BMG160_INTR_ENABLE0_FIFO__LEN               (1)
#define BMG160_INTR_ENABLE0_FIFO__MSK               (0x40)
#define BMG160_INTR_ENABLE0_FIFO__REG               (BMG160_INTR_ENABLE0_ADDR)
/***********************************************/
/**\name	BIT LENGTH, MASK, POSITION
FOR THE AUTO OFFSET INTERRUPT  */
/***********************************************/
/**< 2nd bit of Interrupt Enable Registers */
#define BMG160_INTR_ENABLE0_AUTO_OFFSET__POS        (2)
#define BMG160_INTR_ENABLE0_AUTO_OFFSET__LEN        (1)
#define BMG160_INTR_ENABLE0_AUTO_OFFSET__MSK        (0x04)
#define BMG160_INTR_ENABLE0_AUTO_OFFSET__REG        (BMG160_INTR_ENABLE0_ADDR)
/***********************************************/
/**\name	BIT LENGTH, MASK, POSITION FOR THE OUTPUT TYPE  */
/***********************************************/
/**< 3rd bit of Interrupt Enable Registers */
#define BMG160_INTR_ENABLE1_IT2_OUTPUT_TYPE__POS               (3)
#define BMG160_INTR_ENABLE1_IT2_OUTPUT_TYPE__LEN               (1)
#define BMG160_INTR_ENABLE1_IT2_OUTPUT_TYPE__MSK               (0x08)
#define BMG160_INTR_ENABLE1_IT2_OUTPUT_TYPE__REG               \
(BMG160_INTR_ENABLE1_ADDR)
/***********************************************/
/**\name	BIT LENGTH, MASK, POSITION FOR THE LEVEL  */
/***********************************************/
/**< 2nd bit of Interrupt Enable Registers */
#define BMG160_INTR_ENABLE1_IT2_LEVEL__POS              (2)
#define BMG160_INTR_ENABLE1_IT2_LEVEL__LEN              (1)
#define BMG160_INTR_ENABLE1_IT2_LEVEL__MSK              (0x04)
#define BMG160_INTR_ENABLE1_IT2_LEVEL__REG              \
(BMG160_INTR_ENABLE1_ADDR)
/***********************************************/
/**\name	BIT LENGTH, MASK, POSITION FOR THE OUTPUT TYPE  */
/***********************************************/
/**< 1st bit of Interrupt Enable Registers */
#define BMG160_INTR_ENABLE1_IT1_OUTPUT_TYPE__POS               (1)
#define BMG160_INTR_ENABLE1_IT1_OUTPUT_TYPE__LEN               (1)
#define BMG160_INTR_ENABLE1_IT1_OUTPUT_TYPE__MSK               (0x02)
#define BMG160_INTR_ENABLE1_IT1_OUTPUT_TYPE__REG               \
(BMG160_INTR_ENABLE1_ADDR)
/***********************************************/
/**\name	BIT LENGTH, MASK, POSITION FOR THE LEVEL  */
/***********************************************/
/**< 0th bit of Interrupt Enable Registers */
#define BMG160_INTR_ENABLE1_IT1_LEVEL__POS              (0)
#define BMG160_INTR_ENABLE1_IT1_LEVEL__LEN              (1)
#define BMG160_INTR_ENABLE1_IT1_LEVEL__MSK              (0x01)
#define BMG160_INTR_ENABLE1_IT1_LEVEL__REG              \
(BMG160_INTR_ENABLE1_ADDR)
/***********************************************/
/**\name	BIT LENGTH, MASK, POSITION FOR HIGH RATE INTERRUPT  */
/***********************************************/
/**< 3rd bit of Interrupt MAP (0) Registers */
#define BMG160_INTR_MAP_ZERO_INTR1_HIGHRATE__POS            (3)
#define BMG160_INTR_MAP_ZERO_INTR1_HIGHRATE__LEN            (1)
#define BMG160_INTR_MAP_ZERO_INTR1_HIGHRATE__MSK            (0x08)
#define BMG160_INTR_MAP_ZERO_INTR1_HIGHRATE__REG            \
(BMG160_INTR_MAP_ZERO_ADDR)
/***********************************************/
/**\name	BIT LENGTH, MASK, POSITION FOR ANY_MOTION INTERRUPT  */
/***********************************************/
/**< 1st bit of Interrupt MAP  Registers */
#define BMG160_INTR_MAP_ZERO_INTR1_ANY_MOTION__POS             (1)
#define BMG160_INTR_MAP_ZERO_INTR1_ANY_MOTION__LEN             (1)
#define BMG160_INTR_MAP_ZERO_INTR1_ANY_MOTION__MSK             (0x02)
#define BMG160_INTR_MAP_ZERO_INTR1_ANY_MOTION__REG             \
(BMG160_INTR_MAP_ZERO_ADDR)

/**< 7th bit of MAP_1Registers */
#define BMG160_MAP_ONE_INTR2_DATA__POS                  (7)
#define BMG160_MAP_ONE_INTR2_DATA__LEN                  (1)
#define BMG160_MAP_ONE_INTR2_DATA__MSK                  (0x80)
#define BMG160_MAP_ONE_INTR2_DATA__REG                  \
(BMG160_INTR_MAP_ONE_ADDR)
/***********************************************/
/**\name	BIT LENGTH, MASK, POSITION FOR FAST OFFSET INTERRUPT  */
/***********************************************/
/**< 6th bit of MAP_1Registers */
#define BMG160_MAP_ONE_INTR2_FAST_OFFSET__POS           (6)
#define BMG160_MAP_ONE_INTR2_FAST_OFFSET__LEN           (1)
#define BMG160_MAP_ONE_INTR2_FAST_OFFSET__MSK           (0x40)
#define BMG160_MAP_ONE_INTR2_FAST_OFFSET__REG           \
(BMG160_INTR_MAP_ONE_ADDR)
/***********************************************/
/**\name	BIT LENGTH, MASK, POSITION FOR FIFO INTERRUPT  */
/***********************************************/
/**< 5th bit of MAP_1Registers */
#define BMG160_MAP_ONE_INTR2_FIFO__POS                  (5)
#define BMG160_MAP_ONE_INTR2_FIFO__LEN                  (1)
#define BMG160_MAP_ONE_INTR2_FIFO__MSK                  (0x20)
#define BMG160_MAP_ONE_INTR2_FIFO__REG                  \
(BMG160_INTR_MAP_ONE_ADDR)
/***********************************************/
/**\name	BIT LENGTH, MASK, POSITION FOR AUTO OFFSET INTERRUPT  */
/***********************************************/
/**< 4th bit of MAP_1Registers */
#define BMG160_MAP_ONE_INTR2_AUTO_OFFSET__POS           (4)
#define BMG160_MAP_ONE_INTR2_AUTO_OFFSET__LEN           (1)
#define BMG160_MAP_ONE_INTR2_AUTO_OFFSET__MSK           (0x10)
#define BMG160_MAP_ONE_INTR2_AUTO_OFFSET__REG           \
(BMG160_INTR_MAP_ONE_ADDR)

/**< 3rd bit of MAP_1Registers */
#define BMG160_MAP_ONE_INTR1_AUTO_OFFSET__POS           (3)
#define BMG160_MAP_ONE_INTR1_AUTO_OFFSET__LEN           (1)
#define BMG160_MAP_ONE_INTR1_AUTO_OFFSET__MSK           (0x08)
#define BMG160_MAP_ONE_INTR1_AUTO_OFFSET__REG           \
(BMG160_INTR_MAP_ONE_ADDR)
/***********************************************/
/**\name	BIT LENGTH, MASK, POSITION FOR FIFO INTERRUPT  */
/**********************************************/
/**< 2nd bit of MAP_1Registers */
#define BMG160_MAP_ONE_INTR1_FIFO__POS                  (2)
#define BMG160_MAP_ONE_INTR1_FIFO__LEN                  (1)
#define BMG160_MAP_ONE_INTR1_FIFO__MSK                  (0x04)
#define BMG160_MAP_ONE_INTR1_FIFO__REG                  \
(BMG160_INTR_MAP_ONE_ADDR)
/***********************************************/
/**\name	BIT LENGTH, MASK, POSITION FOR FAST OFFSET INTERRUPT  */
/**********************************************/
/**< 1st bit of MAP_1Registers */
#define BMG160_MAP_ONE_INTR1_FAST_OFFSET__POS           (1)
#define BMG160_MAP_ONE_INTR1_FAST_OFFSET__LEN           (1)
#define BMG160_MAP_ONE_INTR1_FAST_OFFSET__MSK           (0x02)
#define BMG160_MAP_ONE_INTR1_FAST_OFFSET__REG           \
(BMG160_INTR_MAP_ONE_ADDR)
/***********************************************/
/**\name	BIT LENGTH, MASK, POSITION FOR DATA INTERRUPT  */
/**********************************************/
/**< 0th bit of MAP_1Registers */
#define BMG160_MAP_ONE_INTR1_DATA__POS                  (0)
#define BMG160_MAP_ONE_INTR1_DATA__LEN                  (1)
#define BMG160_MAP_ONE_INTR1_DATA__MSK                  (0x01)
#define BMG160_MAP_ONE_INTR1_DATA__REG                  \
(BMG160_INTR_MAP_ONE_ADDR)
/***********************************************/
/**\name	BIT LENGTH, MASK, POSITION FOR HIGH RATE INTERRUPT  */
/**********************************************/
/**< 3rd bit of Interrupt Map  Registers */
#define BMG160_INTR_MAP_TWO_INTR2_HIGHRATE__POS            (3)
#define BMG160_INTR_MAP_TWO_INTR2_HIGHRATE__LEN            (1)
#define BMG160_INTR_MAP_TWO_INTR2_HIGHRATE__MSK            (0x08)
#define BMG160_INTR_MAP_TWO_INTR2_HIGHRATE__REG            \
(BMG160_INTR_MAP_TWO_ADDR)
/***********************************************/
/**\name	BIT LENGTH, MASK, POSITION FOR ANY MOTION INTERRUPT  */
/**********************************************/
/**< 1st bit of Interrupt Map  Registers */
#define BMG160_INTR_MAP_TWO_INTR2_ANY_MOTION__POS             (1)
#define BMG160_INTR_MAP_TWO_INTR2_ANY_MOTION__LEN             (1)
#define BMG160_INTR_MAP_TWO_INTR2_ANY_MOTION__MSK             (0x02)
#define BMG160_INTR_MAP_TWO_INTR2_ANY_MOTION__REG             \
(BMG160_INTR_MAP_TWO_ADDR)
/***********************************************/
/**\name	BIT LENGTH, MASK, POSITION FOR SLOW OFFSET UNFILT  */
/**********************************************/
/**< 5th bit of Interrupt  Registers */
#define BMG160_INTR_ZERO_ADDR_SLOW_OFFSET_UNFILT__POS          (5)
#define BMG160_INTR_ZERO_ADDR_SLOW_OFFSET_UNFILT__LEN          (1)
#define BMG160_INTR_ZERO_ADDR_SLOW_OFFSET_UNFILT__MSK          (0x20)
#define BMG160_INTR_ZERO_ADDR_SLOW_OFFSET_UNFILT__REG          \
(BMG160_INTR_ZERO_ADDR)
/***********************************************/
/**\name	BIT LENGTH, MASK, POSITION FOR HIGH RATE UNFILT  */
/**********************************************/
/**< 3rd bit of Interrupt  Registers */
#define BMG160_INTR_ZERO_ADDR_HIGHRATE_UNFILT_DATA__POS            (3)
#define BMG160_INTR_ZERO_ADDR_HIGHRATE_UNFILT_DATA__LEN            (1)
#define BMG160_INTR_ZERO_ADDR_HIGHRATE_UNFILT_DATA__MSK            (0x08)
#define BMG160_INTR_ZERO_ADDR_HIGHRATE_UNFILT_DATA__REG            \
(BMG160_INTR_ZERO_ADDR)
/***********************************************/
/**\name	BIT LENGTH, MASK, POSITION FOR ANY MOTION UNFILT  */
/**********************************************/
/**< 1st bit of Interrupt (0) Registers */
#define BMG160_INTR_ZERO_ADDR_ANY_MOTION_UNFILT_DATA__POS             (1)
#define BMG160_INTR_ZERO_ADDR_ANY_MOTION_UNFILT_DATA__LEN             (1)
#define BMG160_INTR_ZERO_ADDR_ANY_MOTION_UNFILT_DATA__MSK             (0x02)
#define BMG160_INTR_ZERO_ADDR_ANY_MOTION_UNFILT_DATA__REG             \
(BMG160_INTR_ZERO_ADDR)
/***********************************************/
/**\name	BIT LENGTH, MASK, POSITION FOR FAST OFFSET UNFILT  */
/**********************************************/
/**< 7th bit of INT_1  Registers */
#define BMG160_INTR_ONE_ADDR_FAST_OFFSET_UNFILT__POS            (7)
#define BMG160_INTR_ONE_ADDR_FAST_OFFSET_UNFILT__LEN            (1)
#define BMG160_INTR_ONE_ADDR_FAST_OFFSET_UNFILT__MSK            (0x80)
#define BMG160_INTR_ONE_ADDR_FAST_OFFSET_UNFILT__REG            \
(BMG160_INTR_ONE_ADDR)
/***********************************************/
/**\name	BIT LENGTH, MASK, POSITION FOR ANY MOTION THRESHOLD */
/**********************************************/
/**< First (7) bits of INT_1  Registers */
#define BMG160_INTR_ONE_ADDR_ANY_MOTION_THRES__POS                       (0)
#define BMG160_INTR_ONE_ADDR_ANY_MOTION_THRES__LEN                       (7)
#define BMG160_INTR_ONE_ADDR_ANY_MOTION_THRES__MSK                       (0x7F)
#define BMG160_INTR_ONE_ADDR_ANY_MOTION_THRES__REG                       \
(BMG160_INTR_ONE_ADDR)
/***********************************************/
/**\name	BIT LENGTH, MASK, POSITION FOR AWAKE DURATION */
/**********************************************/
/**< Last (2) bits of INT 2Registers */
#define BMG160_INTR_TWO_ADDR_AWAKE_DURN__POS          (6)
#define BMG160_INTR_TWO_ADDR_AWAKE_DURN__LEN          (2)
#define BMG160_INTR_TWO_ADDR_AWAKE_DURN__MSK          (0xC0)
#define BMG160_INTR_TWO_ADDR_AWAKE_DURN__REG          (BMG160_INTR_TWO_ADDR)
/***********************************************/
/**\name	BIT LENGTH, MASK, POSITION FOR ANY MOTION SAMPLE */
/**********************************************/
/**< 4th & 5th bit of INT 2Registers */
#define BMG160_INTR_TWO_ADDR_ANY_MOTION_DURN_SAMPLE__POS      (4)
#define BMG160_INTR_TWO_ADDR_ANY_MOTION_DURN_SAMPLE__LEN      (2)
#define BMG160_INTR_TWO_ADDR_ANY_MOTION_DURN_SAMPLE__MSK      (0x30)
#define BMG160_INTR_TWO_ADDR_ANY_MOTION_DURN_SAMPLE__REG      \
(BMG160_INTR_TWO_ADDR)
/***********************************************/
/**\name	BIT LENGTH, MASK, POSITION FOR ANY MOTION XYZ AXIS */
/**********************************************/
/**< 2nd bit of INT 2Registers */
#define BMG160_INTR_TWO_ADDR_ANY_MOTION_ENABLE_Z__POS           (2)
#define BMG160_INTR_TWO_ADDR_ANY_MOTION_ENABLE_Z__LEN           (1)
#define BMG160_INTR_TWO_ADDR_ANY_MOTION_ENABLE_Z__MSK           (0x04)
#define BMG160_INTR_TWO_ADDR_ANY_MOTION_ENABLE_Z__REG           \
(BMG160_INTR_TWO_ADDR)

/**< 1st bit of INT 2Registers */
#define BMG160_INTR_TWO_ADDR_ANY_MOTION_ENABLE_Y__POS           (1)
#define BMG160_INTR_TWO_ADDR_ANY_MOTION_ENABLE_Y__LEN           (1)
#define BMG160_INTR_TWO_ADDR_ANY_MOTION_ENABLE_Y__MSK           (0x02)
#define BMG160_INTR_TWO_ADDR_ANY_MOTION_ENABLE_Y__REG           \
(BMG160_INTR_TWO_ADDR)

/**< 0th bit of INT 2Registers */
#define BMG160_INTR_TWO_ADDR_ANY_MOTION_ENABLE_X__POS           (0)
#define BMG160_INTR_TWO_ADDR_ANY_MOTION_ENABLE_X__LEN           (1)
#define BMG160_INTR_TWO_ADDR_ANY_MOTION_ENABLE_X__MSK           (0x01)
#define BMG160_INTR_TWO_ADDR_ANY_MOTION_ENABLE_X__REG           \
(BMG160_INTR_TWO_ADDR)
/***********************************************/
/**\name	BIT LENGTH, MASK, POSITION FOR FIFO WATER MARK*/
/**********************************************/
/**< Last bit of INT  Registers */
#define BMG160_INTR_4_FIFO_WM_ENABLE__POS           (7)
#define BMG160_INTR_4_FIFO_WM_ENABLE__LEN           (1)
#define BMG160_INTR_4_FIFO_WM_ENABLE__MSK           (0x80)
#define BMG160_INTR_4_FIFO_WM_ENABLE__REG           (BMG160_INTR_4_ADDR)
/***********************************************/
/**\name	BIT LENGTH, MASK, POSITION FOR LATCH*/
/**********************************************/
/**< Last bit of Reset Latch Registers */
#define BMG160_RST_LATCH_ADDR_RST_INTR__POS           (7)
#define BMG160_RST_LATCH_ADDR_RST_INTR__LEN           (1)
#define BMG160_RST_LATCH_ADDR_RST_INTR__MSK           (0x80)
#define BMG160_RST_LATCH_ADDR_RST_INTR__REG           (BMG160_RST_LATCH_ADDR)

/**< 6th bit of Reset Latch Registers */
#define BMG160_RST_LATCH_ADDR_OFFSET_RST__POS        (6)
#define BMG160_RST_LATCH_ADDR_OFFSET_RST__LEN        (1)
#define BMG160_RST_LATCH_ADDR_OFFSET_RST__MSK        (0x40)
#define BMG160_RST_LATCH_ADDR_OFFSET_RST__REG        (BMG160_RST_LATCH_ADDR)

/**< 4th bit of Reset Latch Registers */
#define BMG160_RST_LATCH_ADDR_LATCH_STAT__POS        (4)
#define BMG160_RST_LATCH_ADDR_LATCH_STAT__LEN        (1)
#define BMG160_RST_LATCH_ADDR_LATCH_STAT__MSK        (0x10)
#define BMG160_RST_LATCH_ADDR_LATCH_STAT__REG        (BMG160_RST_LATCH_ADDR)

/**< First (4) bits of Reset Latch Registers */
#define BMG160_RST_LATCH_ADDR_LATCH_INTR__POS           (0)
#define BMG160_RST_LATCH_ADDR_LATCH_INTR__LEN           (4)
#define BMG160_RST_LATCH_ADDR_LATCH_INTR__MSK           (0x0F)
#define BMG160_RST_LATCH_ADDR_LATCH_INTR__REG           (BMG160_RST_LATCH_ADDR)
/***********************************************/
/**\name	BIT LENGTH, MASK, POSITION FOR HIGH RATE CONFIGURATION*/
/**********************************************/
/**< Last (2) bits of HIGHRATE_THRES_X Registers */
#define BMG160_HIGHRATE_HYST_X__POS        (6)
#define BMG160_HIGHRATE_HYST_X__LEN        (2)
#define BMG160_HIGHRATE_HYST_X__MSK        (0xC0)
#define BMG160_HIGHRATE_HYST_X__REG        (BMG160_HIGHRATE_THRES_X_ADDR)

/**< (5) bits of HIGHRATE_THRES_X Registers */
#define BMG160_HIGHRATE_THRES_X__POS        (1)
#define BMG160_HIGHRATE_THRES_X__LEN        (5)
#define BMG160_HIGHRATE_THRES_X__MSK        (0x3E)
#define BMG160_HIGHRATE_THRES_X__REG        (BMG160_HIGHRATE_THRES_X_ADDR)

/**< 0th bit of HIGHRATE_THRES_X Registers */
#define BMG160_HIGHRATE_ENABLE_X__POS        (0)
#define BMG160_HIGHRATE_ENABLE_X__LEN        (1)
#define BMG160_HIGHRATE_ENABLE_X__MSK        (0x01)
#define BMG160_HIGHRATE_ENABLE_X__REG        (BMG160_HIGHRATE_THRES_X_ADDR)

/**< Last (2) bits of HIGHRATE_THRES_Y Registers */
#define BMG160_HIGHRATE_HYST_Y__POS        (6)
#define BMG160_HIGHRATE_HYST_Y__LEN        (2)
#define BMG160_HIGHRATE_HYST_Y__MSK        (0xC0)
#define BMG160_HIGHRATE_HYST_Y__REG        (BMG160_HIGHRATE_THRES_Y_ADDR)

/**< (5) bits of HIGHRATE_THRES_Y Registers */
#define BMG160_HIGHRATE_THRES_Y__POS        (1)
#define BMG160_HIGHRATE_THRES_Y__LEN        (5)
#define BMG160_HIGHRATE_THRES_Y__MSK        (0x3E)
#define BMG160_HIGHRATE_THRES_Y__REG        (BMG160_HIGHRATE_THRES_Y_ADDR)

/**< 0th bit of HIGHRATE_THRES_Y Registers */
#define BMG160_HIGHRATE_ENABLE_Y__POS        (0)
#define BMG160_HIGHRATE_ENABLE_Y__LEN        (1)
#define BMG160_HIGHRATE_ENABLE_Y__MSK        (0x01)
#define BMG160_HIGHRATE_ENABLE_Y__REG        (BMG160_HIGHRATE_THRES_Y_ADDR)

/**< Last (2) bits of HIGHRATE_THRES_Z Registers */
#define BMG160_HIGHRATE_HYST_Z__POS        (6)
#define BMG160_HIGHRATE_HYST_Z__LEN        (2)
#define BMG160_HIGHRATE_HYST_Z__MSK        (0xC0)
#define BMG160_HIGHRATE_HYST_Z__REG        (BMG160_HIGHRATE_THRES_Z_ADDR)

/**< (5) bits of HIGHRATE_THRES_Z Registers */
#define BMG160_HIGHRATE_THRES_Z__POS        (1)
#define BMG160_HIGHRATE_THRES_Z__LEN        (5)
#define BMG160_HIGHRATE_THRES_Z__MSK        (0x3E)
#define BMG160_HIGHRATE_THRES_Z__REG        (BMG160_HIGHRATE_THRES_Z_ADDR)

/**< 0th bit of HIGHRATE_THRES_Z Registers */
#define BMG160_HIGHRATE_ENABLE_Z__POS        (0)
#define BMG160_HIGHRATE_ENABLE_Z__LEN        (1)
#define BMG160_HIGHRATE_ENABLE_Z__MSK        (0x01)
#define BMG160_HIGHRATE_ENABLE_Z__REG        (BMG160_HIGHRATE_THRES_Z_ADDR)
/***********************************************/
/**\name	BIT LENGTH, MASK, POSITION FOR SLOW OFFSET */
/**********************************************/
/**< Last 3 bits of INT OFF0 Registers */
#define BMG160_SLOW_OFFSET_THRES__POS          (6)
#define BMG160_SLOW_OFFSET_THRES__LEN          (2)
#define BMG160_SLOW_OFFSET_THRES__MSK          (0xC0)
#define BMG160_SLOW_OFFSET_THRES__REG          (BMG160_SOC_ADDR)

/**< 2  bits of INT OFF0 Registers */
#define BMG160_SLOW_OFFSET_DURN__POS         (3)
#define BMG160_SLOW_OFFSET_DURN__LEN         (3)
#define BMG160_SLOW_OFFSET_DURN__MSK         (0x38)
#define BMG160_SLOW_OFFSET_DURN__REG         (BMG160_SOC_ADDR)

/**< 2nd bit of INT OFF0 Registers */
#define BMG160_SLOW_OFFSET_ENABLE_Z__POS        (2)
#define BMG160_SLOW_OFFSET_ENABLE_Z__LEN        (1)
#define BMG160_SLOW_OFFSET_ENABLE_Z__MSK        (0x04)
#define BMG160_SLOW_OFFSET_ENABLE_Z__REG        (BMG160_SOC_ADDR)

/**< 1st bit of INT OFF0 Registers */
#define BMG160_SLOW_OFFSET_ENABLE_Y__POS        (1)
#define BMG160_SLOW_OFFSET_ENABLE_Y__LEN        (1)
#define BMG160_SLOW_OFFSET_ENABLE_Y__MSK        (0x02)
#define BMG160_SLOW_OFFSET_ENABLE_Y__REG        (BMG160_SOC_ADDR)

/**< 0th bit of INT OFF0 Registers */
#define BMG160_SLOW_OFFSET_ENABLE_X__POS        (0)
#define BMG160_SLOW_OFFSET_ENABLE_X__LEN        (1)
#define BMG160_SLOW_OFFSET_ENABLE_X__MSK        (0x01)
#define BMG160_SLOW_OFFSET_ENABLE_X__REG        (BMG160_SOC_ADDR)
/***********************************************/
/**\name	BIT LENGTH, MASK, POSITION FOR AUTO OFFSET*/
/**********************************************/
/**< Last 2 bits of INT OFF1 Registers */
#define BMG160_AUTO_OFFSET_WORD_LENGHTH__POS        (6)
#define BMG160_AUTO_OFFSET_WORD_LENGHTH__LEN        (2)
#define BMG160_AUTO_OFFSET_WORD_LENGHTH__MSK        (0xC0)
#define BMG160_AUTO_OFFSET_WORD_LENGHTH__REG        (BMG160_A_FOC_ADDR)
/***********************************************/
/**\name	BIT LENGTH, MASK, POSITION FOR FATS OFFSET*/
/**********************************************/
/**< 2  bits of INT OFF1 Registers */
#define BMG160_FAST_OFFSET_WORD_LENGHTH__POS        (4)
#define BMG160_FAST_OFFSET_WORD_LENGHTH__LEN        (2)
#define BMG160_FAST_OFFSET_WORD_LENGHTH__MSK        (0x30)
#define BMG160_FAST_OFFSET_WORD_LENGHTH__REG        (BMG160_A_FOC_ADDR)

/**< 3nd bit of INT OFF1 Registers */
#define BMG160_FAST_OFFSET_ENABLE__POS        (3)
#define BMG160_FAST_OFFSET_ENABLE__LEN        (1)
#define BMG160_FAST_OFFSET_ENABLE__MSK        (0x08)
#define BMG160_FAST_OFFSET_ENABLE__REG        (BMG160_A_FOC_ADDR)

/**< 2nd bit of INT OFF1 Registers */
#define BMG160_FAST_OFFSET_ENABLE_Z__POS      (2)
#define BMG160_FAST_OFFSET_ENABLE_Z__LEN      (1)
#define BMG160_FAST_OFFSET_ENABLE_Z__MSK      (0x04)
#define BMG160_FAST_OFFSET_ENABLE_Z__REG      (BMG160_A_FOC_ADDR)

/**< 1st bit of INT OFF1 Registers */
#define BMG160_FAST_OFFSET_ENABLE_Y__POS      (1)
#define BMG160_FAST_OFFSET_ENABLE_Y__LEN      (1)
#define BMG160_FAST_OFFSET_ENABLE_Y__MSK      (0x02)
#define BMG160_FAST_OFFSET_ENABLE_Y__REG      (BMG160_A_FOC_ADDR)

/**< 0th bit of INT OFF1 Registers */
#define BMG160_FAST_OFFSET_ENABLE_X__POS      (0)
#define BMG160_FAST_OFFSET_ENABLE_X__LEN      (1)
#define BMG160_FAST_OFFSET_ENABLE_X__MSK      (0x01)
#define BMG160_FAST_OFFSET_ENABLE_X__REG      (BMG160_A_FOC_ADDR)

/**< 0 to (2) bits of INT OFF1 Registers */
#define BMG160_FAST_OFFSET_ENABLE_XYZ__POS      (0)
#define BMG160_FAST_OFFSET_ENABLE_XYZ__LEN      (3)
#define BMG160_FAST_OFFSET_ENABLE_XYZ__MSK      (0x07)
#define BMG160_FAST_OFFSET_ENABLE_XYZ__REG      (BMG160_A_FOC_ADDR)
/***********************************************/
/**\name	BIT LENGTH, MASK, POSITION FOR NVM*/
/**********************************************/
/**< Last 4 bits of Trim NVM control Registers */
#define BMG160_TRIM_NVM_CTRL_ADDR_NVM_REMAIN__POS        (4)
#define BMG160_TRIM_NVM_CTRL_ADDR_NVM_REMAIN__LEN        (4)
#define BMG160_TRIM_NVM_CTRL_ADDR_NVM_REMAIN__MSK        (0xF0)
#define BMG160_TRIM_NVM_CTRL_ADDR_NVM_REMAIN__REG        \
(BMG160_TRIM_NVM_CTRL_ADDR)

/**< 3rd bit of Trim NVM control Registers */
#define BMG160_TRIM_NVM_CTRL_ADDR_NVM_LOAD__POS          (3)
#define BMG160_TRIM_NVM_CTRL_ADDR_NVM_LOAD__LEN          (1)
#define BMG160_TRIM_NVM_CTRL_ADDR_NVM_LOAD__MSK          (0x08)
#define BMG160_TRIM_NVM_CTRL_ADDR_NVM_LOAD__REG          \
(BMG160_TRIM_NVM_CTRL_ADDR)

/**< 2nd bit of Trim NVM control Registers */
#define BMG160_TRIM_NVM_CTRL_ADDR_NVM_RDY__POS           (2)
#define BMG160_TRIM_NVM_CTRL_ADDR_NVM_RDY__LEN           (1)
#define BMG160_TRIM_NVM_CTRL_ADDR_NVM_RDY__MSK           (0x04)
#define BMG160_TRIM_NVM_CTRL_ADDR_NVM_RDY__REG           \
(BMG160_TRIM_NVM_CTRL_ADDR)

 /**< 1st bit of Trim NVM control Registers */
#define BMG160_TRIM_NVM_CTRL_ADDR_NVM_PROG_TRIG__POS     (1)
#define BMG160_TRIM_NVM_CTRL_ADDR_NVM_PROG_TRIG__LEN     (1)
#define BMG160_TRIM_NVM_CTRL_ADDR_NVM_PROG_TRIG__MSK     (0x02)
#define BMG160_TRIM_NVM_CTRL_ADDR_NVM_PROG_TRIG__REG     \
(BMG160_TRIM_NVM_CTRL_ADDR)

/**< 0th bit of Trim NVM control Registers */
#define BMG160_TRIM_NVM_CTRL_ADDR_NVM_PROG_MODE__POS     (0)
#define BMG160_TRIM_NVM_CTRL_ADDR_NVM_PROG_MODE__LEN     (1)
#define BMG160_TRIM_NVM_CTRL_ADDR_NVM_PROG_MODE__MSK     (0x01)
#define BMG160_TRIM_NVM_CTRL_ADDR_NVM_PROG_MODE__REG     \
(BMG160_TRIM_NVM_CTRL_ADDR)
/***********************************************/
/**\name	BIT LENGTH, MASK, POSITION FOR I2C CONFIGURATION*/
/**********************************************/
 /**< 2nd bit of SPI3 WDT Registers */
#define BMG160_BGW_SPI3_WDT_ADDR_I2C_WDT_ENABLE__POS      (2)
#define BMG160_BGW_SPI3_WDT_ADDR_I2C_WDT_ENABLE__LEN      (1)
#define BMG160_BGW_SPI3_WDT_ADDR_I2C_WDT_ENABLE__MSK      (0x04)
#define BMG160_BGW_SPI3_WDT_ADDR_I2C_WDT_ENABLE__REG      \
(BMG160_BGW_SPI3_WDT_ADDR)

 /**< 1st bit of SPI3 WDT Registers */
#define BMG160_BGW_SPI3_WDT_ADDR_I2C_WDT_SELECT__POS     (1)
#define BMG160_BGW_SPI3_WDT_ADDR_I2C_WDT_SELECT__LEN     (1)
#define BMG160_BGW_SPI3_WDT_ADDR_I2C_WDT_SELECT__MSK     (0x02)
#define BMG160_BGW_SPI3_WDT_ADDR_I2C_WDT_SELECT__REG     \
(BMG160_BGW_SPI3_WDT_ADDR)

/**< 0th bit of SPI3 WDT Registers */
#define BMG160_BGW_SPI3_WDT_ADDR_SPI3__POS            (0)
#define BMG160_BGW_SPI3_WDT_ADDR_SPI3__LEN            (1)
#define BMG160_BGW_SPI3_WDT_ADDR_SPI3__MSK            (0x01)
#define BMG160_BGW_SPI3_WDT_ADDR_SPI3__REG            \
(BMG160_BGW_SPI3_WDT_ADDR)
/***********************************************/
/**\name	BIT LENGTH, MASK, POSITION FOR SELFTEST*/
/**********************************************/
/**< 4th bit of Self test Registers */
#define BMG160_SELFTEST_ADDR_RATEOK__POS            (4)
#define BMG160_SELFTEST_ADDR_RATEOK__LEN            (1)
#define BMG160_SELFTEST_ADDR_RATEOK__MSK            (0x10)
#define BMG160_SELFTEST_ADDR_RATEOK__REG            \
(BMG160_SELFTEST_ADDR)

/**< 2nd bit of Self test Registers */
#define BMG160_SELFTEST_ADDR_BISTFAIL__POS          (2)
#define BMG160_SELFTEST_ADDR_BISTFAIL__LEN          (1)
#define BMG160_SELFTEST_ADDR_BISTFAIL__MSK          (0x04)
#define BMG160_SELFTEST_ADDR_BISTFAIL__REG          \
(BMG160_SELFTEST_ADDR)

/**< 1st bit of Self test Registers */
#define BMG160_SELFTEST_ADDR_BISTRDY__POS           (1)
#define BMG160_SELFTEST_ADDR_BISTRDY__LEN           (1)
#define BMG160_SELFTEST_ADDR_BISTRDY__MSK           (0x02)
#define BMG160_SELFTEST_ADDR_BISTRDY__REG           \
(BMG160_SELFTEST_ADDR)

/**< 0th bit of Self test Registers */
#define BMG160_SELFTEST_ADDR_TRIGBIST__POS          (0)
#define BMG160_SELFTEST_ADDR_TRIGBIST__LEN          (1)
#define BMG160_SELFTEST_ADDR_TRIGBIST__MSK          (0x01)
#define BMG160_SELFTEST_ADDR_TRIGBIST__REG          \
(BMG160_SELFTEST_ADDR)
/***********************************************/
/**\name	BIT LENGTH, MASK, POSITION FOR FIFO CONFIGURATION*/
/**********************************************/
/**< 7th bit of FIFO CGF1 Registers */
#define BMG160_FIFO_CGF1_ADDR_TAG__POS     (7)
#define BMG160_FIFO_CGF1_ADDR_TAG__LEN     (1)
#define BMG160_FIFO_CGF1_ADDR_TAG__MSK     (0x80)
#define BMG160_FIFO_CGF1_ADDR_TAG__REG     (BMG160_FIFO_CGF1_ADDR)

/**< First 7 bits of FIFO CGF1 Registers */
#define BMG160_FIFO_CGF1_ADDR_WML__POS     (0)
#define BMG160_FIFO_CGF1_ADDR_WML__LEN     (7)
#define BMG160_FIFO_CGF1_ADDR_WML__MSK     (0x7F)
#define BMG160_FIFO_CGF1_ADDR_WML__REG     (BMG160_FIFO_CGF1_ADDR)

/**< Last 2 bits of FIFO CGF0 Addr Registers */
#define BMG160_FIFO_CGF0_ADDR_MODE__POS         (6)
#define BMG160_FIFO_CGF0_ADDR_MODE__LEN         (2)
#define BMG160_FIFO_CGF0_ADDR_MODE__MSK         (0xC0)
#define BMG160_FIFO_CGF0_ADDR_MODE__REG         (BMG160_FIFO_CGF0_ADDR)

/**< First 2 bits of FIFO CGF0 Addr Registers */
#define BMG160_FIFO_CGF0_ADDR_DATA_SELECT__POS     (0)
#define BMG160_FIFO_CGF0_ADDR_DATA_SELECT__LEN     (2)
#define BMG160_FIFO_CGF0_ADDR_DATA_SELECT__MSK     (0x03)
#define BMG160_FIFO_CGF0_ADDR_DATA_SELECT__REG     (BMG160_FIFO_CGF0_ADDR)
/***********************************************/
/**\name	BIT LENGTH, MASK, POSITION FOR OFFSET*/
/**********************************************/
 /**< Last 2 bits of INL Offset MSB Registers */
#define BMG160_OFC1_ADDR_OFFSET_X__POS       (6)
#define BMG160_OFC1_ADDR_OFFSET_X__LEN       (2)
#define BMG160_OFC1_ADDR_OFFSET_X__MSK       (0xC0)
#define BMG160_OFC1_ADDR_OFFSET_X__REG       (BMG160_OFFSET_OFC1_ADDR)

/**< 3 bits of INL Offset MSB Registers */
#define BMG160_OFC1_ADDR_OFFSET_Y__POS       (3)
#define BMG160_OFC1_ADDR_OFFSET_Y__LEN       (3)
#define BMG160_OFC1_ADDR_OFFSET_Y__MSK       (0x38)
#define BMG160_OFC1_ADDR_OFFSET_Y__REG       (BMG160_OFFSET_OFC1_ADDR)

/**< First 3 bits of INL Offset MSB Registers */
#define BMG160_OFC1_ADDR_OFFSET_Z__POS       (0)
#define BMG160_OFC1_ADDR_OFFSET_Z__LEN       (3)
#define BMG160_OFC1_ADDR_OFFSET_Z__MSK       (0x07)
#define BMG160_OFC1_ADDR_OFFSET_Z__REG       (BMG160_OFFSET_OFC1_ADDR)

/**< 4 bits of Trim GP0 Registers */
#define BMG160_TRIM_GP0_ADDR_GP0__POS            (4)
#define BMG160_TRIM_GP0_ADDR_GP0__LEN            (4)
#define BMG160_TRIM_GP0_ADDR_GP0__MSK            (0xF0)
#define BMG160_TRIM_GP0_ADDR_GP0__REG            (BMG160_TRIM_GP0_ADDR)

/**< 2 bits of Trim GP0 Registers */
#define BMG160_TRIM_GP0_ADDR_OFFSET_X__POS       (2)
#define BMG160_TRIM_GP0_ADDR_OFFSET_X__LEN       (2)
#define BMG160_TRIM_GP0_ADDR_OFFSET_X__MSK       (0x0C)
#define BMG160_TRIM_GP0_ADDR_OFFSET_X__REG       (BMG160_TRIM_GP0_ADDR)

/**< 1st bit of Trim GP0 Registers */
#define BMG160_TRIM_GP0_ADDR_OFFSET_Y__POS       (1)
#define BMG160_TRIM_GP0_ADDR_OFFSET_Y__LEN       (1)
#define BMG160_TRIM_GP0_ADDR_OFFSET_Y__MSK       (0x02)
#define BMG160_TRIM_GP0_ADDR_OFFSET_Y__REG       (BMG160_TRIM_GP0_ADDR)

/**< First bit of Trim GP0 Registers */
#define BMG160_TRIM_GP0_ADDR_OFFSET_Z__POS       (0)
#define BMG160_TRIM_GP0_ADDR_OFFSET_Z__LEN       (1)
#define BMG160_TRIM_GP0_ADDR_OFFSET_Z__MSK       (0x01)
#define BMG160_TRIM_GP0_ADDR_OFFSET_Z__REG       (BMG160_TRIM_GP0_ADDR)

/***********************************************/
/**\name	CONSTANT DEFINITIONS*/
/**********************************************/
/***********************************************/
/**\name	AXIS ENABLE DEFINITIONS*/
/**********************************************/
/* For Axis Selection   */
/**< It refers BMG160 X-axis */
#define BMG160_X_AXIS           (0)
/**< It refers BMG160 Y-axis */
#define BMG160_Y_AXIS           (1)
/**< It refers BMG160 Z-axis */
#define BMG160_Z_AXIS           (2)
/***********************************************/
/**\name	POWER MODE*/
/**********************************************/
/* For Mode Settings    */
#define BMG160_MODE_NORMAL              (0)
#define BMG160_MODE_DEEPSUSPEND         (1)
#define BMG160_MODE_SUSPEND             (2)
#define BMG160_MODE_FASTPOWERUP			(3)
#define BMG160_MODE_ADVANCEDPOWERSAVING (4)
/***********************************************/
/**\name	BIT SLICE FUNCTIONS */
/**********************************************/
/* get bit slice  */
#define BMG160_GET_BITSLICE(regvar, bitname)\
((regvar & bitname##__MSK) >> bitname##__POS)

/* Set bit slice */
#define BMG160_SET_BITSLICE(regvar, bitname, val)\
((regvar&~bitname##__MSK)|((val<<bitname##__POS)&bitname##__MSK))

/****************************************************/
/**\name	ARRAY SIZE DEFINITIONS      */
/***************************************************/
#define	BMG160_X_DATA_SIZE			(2)
#define	BMG160_Y_DATA_SIZE			(2)
#define	BMG160_Z_DATA_SIZE			(2)
#define	BMG160_XYZ_DATA_SIZE		(6)
#define	BMG160_XYZ_INTR_DATA_SIZE	(12)

#define	BMG160_X_LSB_DATA	(0)
#define	BMG160_X_MSB_DATA	(1)
#define	BMG160_Y_LSB_DATA	(0)
#define	BMG160_Y_MSB_DATA	(1)
#define	BMG160_Z_LSB_DATA	(0)
#define	BMG160_Z_MSB_DATA	(1)
#define	BMG160_INTR0_STAT	(0)
#define	BMG160_INTR1_STAT	(1)
#define	BMG160_INTR2_STAT	(2)
#define	BMG160_INTR3_STAT	(3)
#define	BMG160_INTR4_STAT	(4)

#define	BMG160_DATA_FRAME_X_LSB_BYTE		(0)
#define	BMG160_DATA_FRAME_X_MSB_BYTE		(1)
#define	BMG160_DATA_FRAME_Y_LSB_BYTE		(2)
#define	BMG160_DATA_FRAME_Y_MSB_BYTE		(3)
#define	BMG160_DATA_FRAME_Z_LSB_BYTE		(4)
#define	BMG160_DATA_FRAME_Z_MSB_BYTE		(5)
#define	BMG160_DATA_FRAME_INTR0_BYTE	(7)
#define	BMG160_DATA_FRAME_INTR1_BYTE	(8)
#define	BMG160_DATA_FRAME_INTR2_BYTE	(9)
#define	BMG160_DATA_FRAME_INTR3_BYTE	(10)
#define	BMG160_DATA_FRAME_INTR4_BYTE	(11)
/***********************************************/
/**\name	COMMON DEFINITIONS*/
/**********************************************/

#define BMG160_NULL                             (0)
/**< constant declaration of NULL */
#define BMG160_DISABLE                          (0)
/**< It refers BMG160 disable */
#define BMG160_ENABLE                           (1)
/**< It refers BMG160 enable */
#define BMG160_OFF                              (0)
/**< It refers BMG160 OFF state */
#define BMG160_ON                               (1)
/**< It refers BMG160 ON state  */


#define BMG160_INTR1                              (0)
/**< It refers BMG160 INT1 */
#define BMG160_INTR2                              (1)
/**< It refers BMG160 INT2 */
/***********************************************/
/**\name	OFFSET DEFINITIONS*/
/**********************************************/
#define BMG160_SLOW_OFFSET                         (0)
/**< It refers BMG160 Slow Offset */
#define BMG160_AUTO_OFFSET                         (1)
/**< It refers BMG160 Auto Offset */
#define BMG160_FAST_OFFSET                         (2)
/**< It refers BMG160 Fast Offset */
/***********************************************/
/**\name	INTERRUPT DEFINITIONS*/
/**********************************************/
#define BMG160_S_TAP                               (0)
/**< It refers BMG160 Single Tap */
#define BMG160_D_TAP                               (1)
/**< It refers BMG160 Double Tap */
#define BMG160_INTR1_DATA                          (0)
/**< It refers BMG160 Int1 Data */
#define BMG160_INTR2_DATA                          (1)
/**< It refers BMG160 Int2 Data */
#define BMG160_TAP_UNFILT_DATA                     (0)
/**< It refers BMG160 Tap unfilt data */
#define BMG160_HIGHRATE_UNFILT_DATA                (1)
/**< It refers BMG160 High unfilt data */
#define BMG160_CONST_UNFILT_DATA                   (2)
/**< It refers BMG160 Const unfilt data */
#define BMG160_ANY_MOTION_UNFILT_DATA              (3)
/**< It refers BMG160 Any unfilt data */
#define BMG160_SHAKE_UNFILT_DATA                   (4)
/**< It refers BMG160 Shake unfilt data */
#define BMG160_SHAKE_TH                            (0)
/**< It refers BMG160 Shake Threshold */
#define BMG160_SHAKE_TH2                           (1)
/**< It refers BMG160 Shake Threshold2 */
#define BMG160_AUTO_OFFSET_WORD_LENGHTH            (0)
/**< It refers BMG160 Auto Offset word length */
#define BMG160_FAST_OFFSET_WORD_LENGHTH            (1)
/**< It refers BMG160 Fast Offset word length */
/***********************************************/
/**\name	I2C CONFIGURATION DEFINITIONS*/
/**********************************************/
#define BMG160_I2C_WDT_ENABLE                   (0)
/**< It refers BMG160 I2C WDT En */
#define BMG160_I2C_WDT_SELECT                   (1)
/**< It refers BMG160 I2C WDT Sel */
#define BMG160_EXT_MODE                         (0)
/**< It refers BMG160 Ext Mode */
#define BMG160_EXT_PAGE                         (1)
/**< It refers BMG160 Ext page */
#define BMG160_START_ADDR                       (0)
/**< It refers BMG160 Start Address */
#define BMG160_STOP_ADDR                        (1)
/**< It refers BMG160 Stop Address */
#define BMG160_SLOW_CMD                         (0)
/**< It refers BMG160 Slow Command */
#define BMG160_FAST_CMD                         (1)
/**< It refers BMG160 Fast Command */
#define BMG160_TRIM_VRA                         (0)
/**< It refers BMG160 Trim VRA */
#define BMG160_TRIM_VRD                         (1)
/**< It refers BMG160 Trim VRD */
#define BMG160_LOGBIT_EM                        (0)
/**< It refers BMG160 LogBit Em */
#define BMG160_LOGBIT_VM                        (1)
/**< It refers BMG160 LogBit VM */
#define BMG160_GP0                              (0)
/**< It refers BMG160 GP0 */
#define BMG160_GP1                              (1)
/**< It refers BMG160 GP1*/
#define BMG160_LOW_SPEED                        (0)
/**< It refers BMG160 Low Speed Oscillator */
#define BMG160_HIGHRATE_SPEED                   (1)
/**< It refers BMG160 High Speed Oscillator */
#define BMG160_DRIVE_OFFSET_P                   (0)
/**< It refers BMG160 Drive Offset P */
#define BMG160_DRIVE_OFFSET_N                   (1)
/**< It refers BMG160 Drive Offset N */
#define BMG160_TEST_MODE_ENABLE                 (0)
/**< It refers BMG160 Test Mode Enable */
#define BMG160_TEST_MODE_REG                    (1)
/**< It refers BMG160 Test Mode reg */
#define BMG160_IBIAS_DRIVE_TRIM                 (0)
/**< It refers BMG160 IBIAS Drive Trim */
#define BMG160_IBIAS_RATE_TRIM                  (1)
/**< It refers BMG160 IBIAS Rate Trim */
#define BMG160_BAA_MODE                         (0)
/**< It refers BMG160 BAA Mode Trim */
#define BMG160_BMA_MODE                         (1)
/**< It refers BMG160 BMA Mode Trim */
#define BMG160_PI_KP                            (0)
/**< It refers BMG160 PI KP */
#define BMG160_PI_KI                            (1)
/**< It refers BMG160 PI KI */

/***********************************************/
/**\name	ERROR/SUCCESS DEFINITIONS*/
/**********************************************/
#define C_BMG160_SUCCESS						(0)
/**< It refers BMG160 operation is success */
#define C_BMG160_FAILURE						(1)
/**< It refers BMG160 operation is Failure */
#define E_BMG160_NULL_PTR               ((int8_t)-127)
#define E_BMG160_OUT_OF_RANGE           ((int8_t)-2)
#define ERROR							((int8_t)-1)
/***********************************************/
/**\name	SPI DEFINITIONS*/
/**********************************************/
#define BMG160_SPI_RD_MASK     (0x80)
/**< Read mask **/
#define BMG160_READ_SET       (0x01)
/**< Setting for reading data **/
/***********************************************/
/**\name	BIT SHIFTING DEFINITIONS*/
/**********************************************/
#define BMG160_SHIFT_BIT_POSITION_BY_01_BIT   (1)
/**< Shift bit by 1 Position **/
#define BMG160_SHIFT_BIT_POSITION_BY_02_BITS  (4)
/**< Shift bit by 2 Position **/
#define BMG160_SHIFT_BIT_POSITION_BY_04_BITS  (4)
/**< Shift bit by (4) Position **/
#define BMG160_SHIFT_BIT_POSITION_BY_08_BITS  (8)
/**< Shift bit by (8) Position **/
/***********************************************/
/**\name	NUMERIC DEFINITIONS*/
/**********************************************/
#define	BMG160_INIT_VALUE					((uint8_t)0)
#define	BMG160_GEN_READ_WRITE_DATA_LENGTH	((uint8_t)1)
#define	BMG160_X_DATA_LENGTH				((uint8_t)2)
#define	BMG160_Y_DATA_LENGTH				((uint8_t)2)
#define	BMG160_Z_DATA_LENGTH				((uint8_t)2)
#define	BMG160_ALL_DATA_FRAME_LENGTH		((uint8_t)6)
#define	BMG160_BIT_LENGTH_HIGH_BW			((uint8_t)2)
#define	BMG160_BIT_LENGTH_SHADOW_DIS		((uint8_t)2)
#define	BMG160_BIT_LENGTH_FIFO				((uint8_t)2)
#define	BMG160_BIT_LENGTH_FIFO_WM			((uint8_t)2)
#define	BMG160_BIT_LENGTH_FIFO_TAG			((uint8_t)2)
#define	BMG160_BIT_LENGTH_FIFO_MODE			((uint8_t)4)
#define	BMG160_BIT_LENGTH_FIFO_DATA_SELECT	((uint8_t)4)
#define	BMG160_BIT_MASK_MODE_LPM1			((uint8_t)4)
#define	BMG160_BIT_LENGTH_RANGE				((uint8_t)5)
#define	BMG160_BIT_LENGTH_BW				((uint8_t)8)
#define	BMG160_BIT_LENGTH_DURN				((uint8_t)8)
#define	BMG160_BIT_LENGTH_POWER_MODE		((uint8_t)5)
#define	BMG160_XYZ_AND_INTR_DATA_LENGTH		((uint8_t)12)
#define	BMG160_FIFO_WM_LENGTH				((uint8_t)128)
#define	BMG160_POWER_MODE_DELAY		((uint8_t)1)
#define	BMG160_SELFTEST_DELAY		((uint8_t)10)
#define	BMG160_SELFTEST_BISTFAIL	((uint8_t)0x00)
#define	BMG160_SELFTEST_RATEOK		((uint8_t)0x01)
/***********************************************/
/**\name	BANDWIDTH DEFINITIONS*/
/**********************************************/
#define C_BMG160_NO_FILTER_U8X			(0)
#define	C_BMG160_BW_230HZ_U8X			(1)
#define	C_BMG160_BW_116HZ_U8X			(2)
#define	C_BMG160_BW_47HZ_U8X			(3)
#define	C_BMG160_BW_23HZ_U8X			(4)
#define	C_BMG160_BW_12HZ_U8X			(5)
#define	C_BMG160_BW_64HZ_U8X			(6)
#define	C_BMG160_BW_32HZ_U8X			(7)

#define BMG160_BW_500_HZ	(0x01)
#define BMG160_BW_230_HZ	(0x01)
#define BMG160_BW_116_HZ	(0x02)
#define BMG160_BW_47_HZ		(0x03)
#define BMG160_BW_23_HZ		(0x04)
#define BMG160_BW_12_HZ		(0x05)
#define BMG160_BW_64_HZ		(0x06)
#define BMG160_BW_32_HZ		(0x07)
/***********************************************/
/**\name	SLEEP DURATION DEFINITIONS*/
/**********************************************/

#define C_BMG160_NO_AUTO_SLEEP_DURN_U8X		(0)
#define	C_BMG160_4MS_AUTO_SLEEP_DURN_U8X	(1)
#define	C_BMG160_5MS_AUTO_SLEEP_DURN_U8X	(2)
#define	C_BMG160_8MS_AUTO_SLEEP_DURN_U8X	(3)
#define	C_BMG160_10MS_AUTO_SLEEP_DURN_U8X	(4)
#define	C_BMG160_15MS_AUTO_SLEEP_DURN_U8X	(5)
#define	C_BMG160_20MS_AUTO_SLEEP_DURN_U8X	(6)
#define	C_BMG160_40MS_AUTO_SLEEP_DURN_U8X	(7)
/***********************************************/
/**\name	RANGE DEFINITIONS*/
/**********************************************/
#define BMG160_RANGE_2000	(0x00)
#define BMG160_RANGE_1000	(0x01)
#define BMG160_RANGE_500	(0x02)
#define BMG160_RANGE_250	(0x03)
#define BMG160_RANGE_125	(0x04)

/***********************************************/
/**\name	SOFT RESET DEFINITIONS*/
/**********************************************/
#define BMG160_SOFT_RESET	(0xB6)
/***********************************************/
/**\name	LATCH  DEFINITIONS*/
/**********************************************/
#define BMG160_NON_LATCH			(0x00)
#define BMG160_LATCH_250_MS			(0x01)
#define BMG160_LATCH_500_MS			(0x02)
#define BMG160_LATCH_1_SEC			(0x03)
#define BMG160_LATCH_2_SEC			(0x04)
#define BMG160_LATCH_4_SEC			(0x05)
#define BMG160_LATCH_8_SEC			(0x06)
#define BMG160_LATCH_LATCHED		(0x07)
#define BMG160_LATCH_NON_LATCHED	(0x08)
#define BMG160_LATCH_250_MICRO_SEC	(0x09)
#define BMG160_LATCH_500_MICRO_SEC	(0x0A)
#define BMG160_LATCH_1_MILLI_SEC	(0x0B)
#define BMG160_LATCH_12_5_MILLI_SEC	(0x0C)
#define BMG160_LATCH_25_MILLI_SEC	(0x0D)
#define BMG160_LATCH_50_MILLI_SEC	(0x0E)
/***********************************************/
/**\name	OFFSET DEFINITIONS*/
/**********************************************/
#define	BMG160_OFFSET_MASK_BYTE_OF_DATA	(0x0FF0)
#define BMG160_OFFSET_X_BIT_MASK1		(0x000C)
#define BMG160_OFFSET_X_BIT_MASK2		(0x0003)
#define BMG160_OFFSET_Y_Z_BIT_MASK1		(0x0001)
#define BMG160_OFFSET_Y_Z_BIT_MASK2		(0x000E)
/***********************************************/
/**\name	BUS READ AND WRITE FUNCTION POINTERS DEFINITIONS*/
/**********************************************/
#define BMG160_WR_FUNC_PTR int8_t (*bus_write)\
(uint8_t, uint8_t, uint8_t *, uint8_t)
#define BMG160_RD_FUNC_PTR int8_t (*bus_read)\
(uint8_t, uint8_t, uint8_t *, uint8_t)
#define BMG160_BRD_FUNC_PTR int8_t (*burst_read)\
(uint8_t, uint8_t, uint8_t *, s32)
#define BMG160_MDELAY_DATA_TYPE uint32_t
/**************************************************************/
/**\name	STRUCTURE DEFINITIONS                         */
/**************************************************************/
/*!
 * @brief Structure containing gyro xyz data
 */
typedef struct {
	int16_t x;/**<gyro X  data*/
	int16_t y;/**<gyro Y  data*/
	int16_t z;/**<gyro Z  data*/
}bmg160_xyz_t;

/*!
*	@brief bmg160_t structure
*	This structure holds all relevant information about bmg160
*/
typedef struct {
	uint8_t chip_id;/**< chip id of BMG160 */
	uint8_t dev_addr;/**< device address of BMG160 */

	sensor_read bus_read;/**< burst read function pointer of BMG160 */
	sensor_write bus_write;/**< bus write function pointer of BMG160 */
	delay_msec delay;	/**< delay function pointer (msec)*/
}bmg160_t;

/***********************************************/
/**\name	FUNCTION DECLARATIONS */
/**********************************************/
bstdr_ret_t bmg160_init(bmg160_t *bmg160);

bstdr_ret_t bmg160_check_connection(void);

bstdr_ret_t bmg160_get_data_XYZ(bmg160_xyz_t *data);
bstdr_ret_t bmg160_get_temp(int8_t *v_temp_s8);

bstdr_ret_t bmg160_get_range(uint8_t *v_range_u8);
bstdr_ret_t bmg160_set_range(uint8_t v_range_u8);
bstdr_ret_t bmg160_get_bw(uint8_t *v_bw_u8);
bstdr_ret_t bmg160_set_bw(uint8_t v_bw_u8);

bstdr_ret_t bmg160_set_soft_rst(void);

bstdr_ret_t bmg160_get_fifo_wm_enable(uint8_t *v_fifo_wm_enable_u8);
bstdr_ret_t bmg160_set_fifo_wm_enable(uint8_t v_fifo_wm_enable_u8);
bstdr_ret_t bmg160_get_fifo_tag(uint8_t *v_fifo_tag_u8);
bstdr_ret_t bmg160_set_fifo_tag(uint8_t v_fifo_tag_u8);
bstdr_ret_t bmg160_get_fifo_wm_level(uint8_t *v_fifo_wm_level_u8);
bstdr_ret_t bmg160_set_fifo_wm_level(uint8_t v_fifo_wm_level_u8);
bstdr_ret_t bmg160_get_FIFO_data_reg(uint8_t *v_fifo_data_u8);
bstdr_ret_t bmg160_get_fifo_stat_reg(uint8_t *v_fifo_stat_u8);
bstdr_ret_t bmg160_get_fifo_frame_count(uint8_t *v_fifo_frame_count_u8);
bstdr_ret_t bmg160_get_fifo_overrun(uint8_t *v_fifo_overrun_u8);
bstdr_ret_t bmg160_get_fifo_mode(uint8_t *v_fifo_mode_u8);
bstdr_ret_t bmg160_set_fifo_mode(uint8_t v_fifo_mode_u8);
bstdr_ret_t bmg160_get_fifo_data_select(uint8_t *v_fifo_data_select_u8);
bstdr_ret_t bmg160_set_fifo_data_select(uint8_t v_fifo_data_select_u8);

bstdr_ret_t bmg160_get_power_mode(uint8_t *v_power_mode_u8);
bstdr_ret_t bmg160_set_power_mode(uint8_t v_power_mode_u8);

bstdr_ret_t bmg160_get_auto_sleep_durn(uint8_t *v_durn_u8);
bstdr_ret_t bmg160_set_auto_sleep_durn(uint8_t v_durn_u8,uint8_t v_bw_u8);

bstdr_ret_t bmg160_selftest(uint8_t *v_result_u8);

bstdr_ret_t bmg160_read_register(uint8_t v_addr_u8,uint8_t *v_data_u8, uint8_t v_len_u8);
bstdr_ret_t bmg160_burst_read(uint8_t v_addr_u8,uint8_t *v_data_u8, uint32_t v_len_u32);
bstdr_ret_t bmg160_write_register(uint8_t v_addr_u8,uint8_t *v_data_u8, uint8_t v_len_u8);

#endif
