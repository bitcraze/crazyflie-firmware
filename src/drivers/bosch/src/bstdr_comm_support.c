/*
 * Copyright (C) 2014 Bosch Sensortec GmbH
 *
 * \section Disclaimer
 *
 * Common:
 * Bosch Sensortec products are developed for the consumer goods industry.
 * They may only be used within the parameters of the respective valid
 * product data sheet.  Bosch Sensortec products are provided with the
 * express understanding that there is no warranty of fitness for a
 * particular purpose.They are not fit for use in life-sustaining,
 * safety or security sensitive systems or any system or device
 * that may lead to bodily harm or property damage if the system
 * or device malfunctions. In addition,Bosch Sensortec products are
 * not fit for use in products which interact with motor vehicle systems.
 * The resale and or use of products are at the purchasers own risk and
 * his own responsibility. The examination of fitness for the intended use
 * is the sole responsibility of the Purchaser.
 *
 * The purchaser shall indemnify Bosch Sensortec from all third party
 * claims, including any claims for incidental, or consequential damages,
 * arising from any product use not covered by the parameters of
 * the respective valid product data sheet or not approved by
 * Bosch Sensortec and reimburse Bosch Sensortec for all costs in
 * connection with such claims.
 *
 * The purchaser must monitor the market for the purchased products,
 * particularly with regard to product safety and inform Bosch Sensortec
 * without delay of all security relevant incidents.
 *
 * Engineering Samples are marked with an asterisk (*) or (e).
 * Samples may vary from the valid technical specifications of the product
 * series. They are therefore not intended or fit for resale to third
 * parties or for use in end products. Their sole purpose is internal
 * client testing. The testing of an engineering sample may in no way
 * replace the testing of a product series. Bosch Sensortec assumes
 * no liability for the use of engineering samples.
 * By accepting the engineering samples, the Purchaser agrees to indemnify
 * Bosch Sensortec from all claims arising from the use of engineering
 * samples.
 *
 * Special:
 * This software module (hereinafter called "Software") and any information
 * on application-sheets (hereinafter called "Information") is provided
 * free of charge for the sole purpose to support your application work.
 * The Software and Information is subject to the following
 * terms and conditions:
 *
 * The Software is specifically designed for the exclusive use for
 * Bosch Sensortec products by personnel who have special experience
 * and training. Do not use this Software if you do not have the
 * proper experience or training.
 *
 * This Software package is provided `` as is `` and without any expressed
 * or implied warranties,including without limitation, the implied warranties
 * of merchantability and fitness for a particular purpose.
 *
 * Bosch Sensortec and their representatives and agents deny any liability
 * for the functional impairment
 * of this Software in terms of fitness, performance and safety.
 * Bosch Sensortec and their representatives and agents shall not be liable
 * for any direct or indirect damages or injury, except as
 * otherwise stipulated in mandatory applicable law.
 *
 * The Information provided is believed to be accurate and reliable.
 * Bosch Sensortec assumes no responsibility for the consequences of use
 * of such Information nor for any infringement of patents or
 * other rights of third parties which may result from its use.
 * No license is granted by implication or otherwise under any patent or
 * patent rights of Bosch. Specifications mentioned in the Information are
 * subject to change without notice.
 */

 /*!
 * @file		bstdr_comm_support.c
 *
 * @brief
 * Contains the code for the communication wrapper APIs
 *
 */


#include "bstdr_comm_support.h"

#include "stm32fxxx.h"
#include "FreeRTOS.h"
#include "task.h"

// TA: Maybe not so good to bring in these dependencies...
#include "debug.h"
#include "eprintf.h"
#include "i2cdev.h"


/*!
 * @brief Communication initialization
 *
 * This is optional. Depends on the system you are using
 *
 * @return Zero if successful, otherwise an error code
 */
bstdr_ret_t bstdr_comm_init(void)
{
	/**< Communication initialization --Optional!*/
	return (bstdr_ret_t)0;
}


/*!
 * @brief Generic burst read
 *
 * @param [out] dev_id I2C address, SPI chip select or user desired identifier
 *
 * @return Zero if successful, otherwise an error code
 */
bstdr_ret_t bstdr_burst_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint32_t len)
{
	/**< Burst read code comes here */
	if (i2cdevReadReg8(I2C1_DEV, dev_id, reg_addr, (uint16_t) len, reg_data))
	{
	  return BSTDR_OK;
	}
	else
	{
    return BSTDR_E_CON_ERROR;
	}
}


/*!
 * @brief Generic burst write
 *
 * @param [out] dev_id I2C address, SPI chip select or user desired identifier
 *
 * @return Zero if successful, otherwise an error code
 */
bstdr_ret_t bstdr_burst_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint32_t len)
{
	/**< Burst write code comes here */
	if (i2cdevWriteReg8(I2C1_DEV, dev_id,reg_addr,(uint16_t) len, reg_data))
  {
    return BSTDR_OK;
  }
  else
  {
    return BSTDR_E_CON_ERROR;
  }
}


/*!
 * @brief Generic burst read
 *
 * @param [in] period Delay period in milliseconds
 *
 * @return None
 */
void bstdr_ms_delay(uint32_t period)
{
	/**< Delay code comes */
	vTaskDelay(M2T(period)); // Delay a while to let the device stabilize
}

