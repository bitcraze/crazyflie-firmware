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
 * @file		bstdr_types.h
 *
 * @brief
 * Contains the datatypes common to the sensor APIs
 *
 */

#ifndef __BSTDR_TYPES_H__
#define __BSTDR_TYPES_H__


#include <stdint.h>
#include <stdbool.h>

/*!
 * @brief Enumeration for function return codes
 *
 */
typedef enum
{
	BSTDR_OK = 0,
	BSTDR_E_GEN_ERROR = -1,
	BSTDR_E_OUT_OF_RANGE = -2,
	BSTDR_E_BMI160_BUSY = -3,
	BSTDR_E_CON_ERROR= -4,
	BSTDR_E_CHIPID_ERROR= -5,
	BSTDR_E_NULL_PTR = -127
}bstdr_ret_t;

/*!
 * @brief Enumeration for sensor interfaces
 *
 */
typedef enum
{
	BSTDR_NO_INTERFACE = 0,
	BSTDR_I2C_INTERFACE = 1,
	BSTDR_SPI3_INTERFACE = 3,
	BSTDR_SPI4_INTERFACE = 4
}bstdr_interface_t;

#define BSTDR_BITS_MSK(bitspos,bitslen) \
		(((0xff>>(8-bitslen))<<bitspos))
#define BSTDR_GET_BITSLICE(regvar, bitspos, bitslen) \
			((regvar & BSTDR_BITS_MSK(bitspos,bitslen)) >> bitspos)
#define BSTDR_SET_BITSLICE(regvar, bitspos,bitslen, val)\
		((regvar & ~BSTDR_BITS_MSK(bitspos,bitslen)) | ((val<<bitspos)&BSTDR_BITS_MSK(bitspos,bitslen)))

/**< Function pointers */
/**< function pointer for the burst write operation in either I2C or SPI*/
typedef  bstdr_ret_t(*sensor_write)(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint32_t len);
/**< function pointer for the burst read operation in either I2C or SPI*/
typedef  bstdr_ret_t(*sensor_read)(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint32_t len);
/**< delay function pointer */
typedef void (*delay_msec)(uint32_t);

#endif /* __BSTDR_TYPES_H__ */
