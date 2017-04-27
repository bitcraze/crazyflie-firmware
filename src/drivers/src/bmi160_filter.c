/*
 * bmi160_filter.c
 *
 *  Created on: 30.01.2017
 *      Author: sse2rt
 */

#include "bmi160_filter.h"

/* FILTER TYPES AND COEFFICIENTS */
#if defined IIR4_CHEBY2_WN_0_3125
	/* cheby2; 4th order; lowpass -3dB@80Hz; -40dB@200Hz, -60dB@250Hz; Fs = 1.6kHz --> Wn=0.3125
 	 a = [1.         -3.18024122  3.86083933 -2.11185936  0.43817731]
	b = [ 0.00260715 -0.00091876  0.00353929 -0.00091876  0.00260715]
	 */
	#define IIR_ORDER 			4
	float IIRF_A_COEF[5] = { 1.0,         -3.18024122,  3.86083933, -2.11185936,  0.43817731};
	float IIRF_B_COEF[5] = {0.00260715, -0.00091876,  0.00353929, -0.00091876,  0.00260715};

#elif defined IIR4_CHEBY2_WN_0_1125
	/* cheby2; 4th order; lowpass -3dB@45Hz; -40dB@110Hz, -60dB@150Hz; Fs = 1.6kHz --> Wn=0.1125
 	 a=[ 1.         -3.53166825  4.70168224 -2.79457704  0.62542019]
 	 b=[ 0.00142658 -0.00294905  0.00390208 -0.00294905  0.00142658]
	 */
	#define IIR_ORDER 			4
	float IIRF_A_COEF[5] = { 1. ,        -3.53166825,  4.70168224, -2.79457704,  0.62542019};
	float IIRF_B_COEF[5] = {0.00142658, -0.00294905,  0.00390208, -0.00294905,  0.00142658};

#else
	/* butter; 3th order; lowpass; -3dB@90Hz; -25dB@200Hz; -60dB@500Hz; Fs = 1.6kHz --> Wn=0.1125
	a = [ 1.         -2.37409474  1.92935567 -0.53207537]
	b = [ 0.00289819  0.00869458  0.00869458  0.00289819]
	*/
	#define IIR_ORDER 3
	float IIRF_A_COEF[4] = { 1. ,        -2.37409474,  1.92935567, -0.53207537};
	float IIRF_B_COEF[4] = { 0.00289819,  0.00869458,  0.00869458, 0.00289819};
#endif

/**
 * @brief initialization of the filter structure.
 * */
void bmi160_lfilter_init(struct iir_filter_1d *filter){
	// initialize filter
	filter->n = IIR_ORDER;
	filter->init_flag = 1;	// initialized
	// initialize coefficients
	filter->a = IIRF_A_COEF; // nominator
	filter->b = IIRF_B_COEF; // denominator
	bmi160_lfilter_reset(filter); // reset internal states
}

/**
 * @brief apply digital filtering on input data.
 * */
float bmi160_lfilter(struct iir_filter_1d *filter,float in_val){
	uint8_t indx;

	/* update internal states --> shift by 1*/
	for (indx=filter->n;indx>0;indx--){
		filter->out[indx] = filter->out[indx-1];
		filter->in[indx] = filter->in[indx-1];
	}
	/* copy input value */
	filter->in[0] = in_val;

	/* calculate first sample */
	filter->out[0] = filter->b[0]*filter->in[0];

	/*run iir algorithms for all samples*/
	for (indx=1;indx<=filter->n;indx++)
		filter->out[0] += 	filter->b[indx]*filter->in[indx] - filter->a[indx]*filter->out[indx];

	/* increase counter for filtered input*/
	filter->init_flag++;
	if(filter->init_flag>filter->n)
		filter->init_flag=filter->n;

	return filter->out[0];
}

/*!
 * @brief Resets internal states of the filter.
 * */
void bmi160_lfilter_reset(struct iir_filter_1d *filter){
	uint8_t indx;
	// resets internal states (input/output states)
	for (indx=0;indx<=filter->n;indx++){
		filter->in[indx] = 0;
		filter->out[indx] = 0;
	}
	filter->init_flag = 1; // initialized but not filled.
}

