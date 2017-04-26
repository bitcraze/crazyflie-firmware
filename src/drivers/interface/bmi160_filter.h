/*
 * bmi160_filter.h
 *
 *  Created on: 30.01.2017
 *      Author: sse2rt
 */

#ifndef SRC_DRIVERS_INTERFACE_BMI160_FILTER_H_
#define SRC_DRIVERS_INTERFACE_BMI160_FILTER_H_

#include <stdint.h>

/*************************** IIR Filter Structure ************************/
#define IIR_FILTER_ORDER_MAX		4

/** FILTER COEFFICIENTS DEFINEs
 * */
//#define IIR4_CHEBY2_WN_0_3125 		// iir 4th order, cheby type II, fc1 =
#define IIR4_CHEBY2_WN_0_1125			// iir4 4th order, cheby type II,

/** IIR FILTER OBJECT
 */
struct iir_filter_1d{
  float *a;		// nominator coefficients --> output multiplicator
  float *b;		// denominator coefficients --> input multiplicator
  float out[IIR_FILTER_ORDER_MAX+1];	// outputs i=0 --> current filtered value
  float in[IIR_FILTER_ORDER_MAX+1];		// inputs
  uint8_t n;							// filter order
  uint8_t init_flag;					// initialization flag, + counter for number of samples acquired
};

/******************* API's for Digital Filter Interfaces ************************/

void bmi160_lfilter_init(struct iir_filter_1d *filter);				// initialization of the filter (coefficients, order etc...)
float bmi160_lfilter(struct iir_filter_1d *filter,float in_val);	// filter input value
void bmi160_lfilter_reset(struct iir_filter_1d *filter);			// resets internal states of the filter ()


#endif /* SRC_DRIVERS_INTERFACE_BMI160_FILTER_H_ */
