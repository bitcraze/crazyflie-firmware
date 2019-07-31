/*
 *
 * Copyright (c) 2019 Ewoud Smeur and Andre Luis Ogando Paraense
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * This control algorithm is the Incremental Nonlinear Dynamic Inversion (INDI)
 * controller.
 *
 * This is an implementation of the publication in the
 * journal of Control Guidance and Dynamics: Adaptive Incremental Nonlinear
 * Dynamic Inversion for Attitude Control of Micro Aerial Vehicles
 * http://arc.aiaa.org/doi/pdf/10.2514/1.G001490
 */

#include "controller_indi.h"

struct IndiVariables indi = {
  .max_rate = STABILIZATION_INDI_MAX_RATE,
  .attitude_max_yaw_rate = STABILIZATION_INDI_MAX_R,

  .g1 = {STABILIZATION_INDI_G1_P, STABILIZATION_INDI_G1_Q, STABILIZATION_INDI_G1_R},
  .g2 = STABILIZATION_INDI_G2_R,
  .reference_acceleration = {
    STABILIZATION_INDI_REF_ERR_P,
    STABILIZATION_INDI_REF_ERR_Q,
    STABILIZATION_INDI_REF_ERR_R,
    STABILIZATION_INDI_REF_RATE_P,
    STABILIZATION_INDI_REF_RATE_Q,
    STABILIZATION_INDI_REF_RATE_R
  },

  /* Estimation parameters for adaptive INDI */
  .est = {
    .g1 = {
      STABILIZATION_INDI_G1_P / INDI_EST_SCALE,
      STABILIZATION_INDI_G1_Q / INDI_EST_SCALE,
      STABILIZATION_INDI_G1_R / INDI_EST_SCALE
    },
    .g2 = STABILIZATION_INDI_G2_R / INDI_EST_SCALE,
    .mu = STABILIZATION_INDI_ADAPTIVE_MU,
  },

#if STABILIZATION_INDI_USE_ADAPTIVE
  .adaptive = true,
#else
  .adaptive = false,
#endif
};

static inline void float_rates_zero(struct FloatRates *fr) {
	fr->p = 0.0;
	fr->q = 0.0;
	fr->r = 0.0;
}

void indi_init_filters(void)
{
  // tau = 1/(2*pi*Fc)
  float tau = 1.0 / (2.0 * M_PI * STABILIZATION_INDI_FILT_CUTOFF);
  float tau_r = 1.0 / (2.0 * M_PI * STABILIZATION_INDI_FILT_CUTOFF_R);
  float tau_axis[3] = {tau, tau, tau_r};
  float tau_est = 1.0 / (2.0 * M_PI * STABILIZATION_INDI_ESTIMATION_FILT_CUTOFF);
  float sample_time = 1.0 / PERIODIC_FREQUENCY;
  // Filtering of gyroscope and actuators
  for (int8_t i = 0; i < 3; i++) {
    init_butterworth_2_low_pass(&indi.u[i], tau_axis[i], sample_time, 0.0);
    init_butterworth_2_low_pass(&indi.rate[i], tau_axis[i], sample_time, 0.0);
    init_butterworth_2_low_pass(&indi.est.u[i], tau_est, sample_time, 0.0);
    init_butterworth_2_low_pass(&indi.est.rate[i], tau_est, sample_time, 0.0);
  }
}

/**
 * @brief Update butterworth filter for p, q and r of a FloatRates struct
 *
 * @param filter The filter array to use
 * @param new_values The new values
 */
static inline void filter_pqr(Butterworth2LowPass *filter, struct FloatRates *new_values)
{
  update_butterworth_2_low_pass(&filter[0], new_values->p);
  update_butterworth_2_low_pass(&filter[1], new_values->q);
  update_butterworth_2_low_pass(&filter[2], new_values->r);
}

/**
 * @brief Caclulate finite difference form a filter array
 * The filter already contains the previous values
 *
 * @param output The output array
 * @param filter The filter array input
 */
static inline void finite_difference_from_filter(float *output, Butterworth2LowPass *filter)
{
  for (int8_t i = 0; i < 3; i++) {
    output[i] = (filter[i].o[0] - filter[i].o[1]) * PERIODIC_FREQUENCY;
  }
}

void controllerINDIInit(void)
{
	float_rates_zero(&indi.angular_accel_ref);
	float_rates_zero(&indi.u_act_dyn);
	float_rates_zero(&indi.u_in);

	// Re-initialize filters
	indi_init_filters();
}

bool controllerINDITest(void)
{
	return true;
}

void controllerINDI(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{

	/*
	 * 1 - Update the gyro filter with the new measurements.
	 */

	float stateAttitudeRateRoll = radians(sensors->gyro.x);
	float stateAttitudeRatePitch = -radians(sensors->gyro.y);
	float stateAttitudeRateYaw = radians(sensors->gyro.z);

	 struct FloatRates body_rates = {
			  .p = stateAttitudeRateRoll,
			  .q = stateAttitudeRatePitch,
			  .r = stateAttitudeRateYaw,
	 };
	 filter_pqr(indi.rate, &body_rates);


	/*
	 * 2 - Calculate the derivative with finite difference.
	 */

	 finite_difference_from_filter(indi.rate_d, indi.rate);


	/*
	 * 3 - same filter on the actuators (or control_t values), using the commands from the previous timestep.
	 */
	 filter_pqr(indi.u, &indi.u_act_dyn);


	/*
	 * 4 - Calculate the desired angular acceleration by:
	 * 4.1 - Rate_reference = P * attitude_error, where attitude error can be calculated with your favorite
	 * algorithm. You may even use a function that is already there, such as attitudeControllerCorrectAttitudePID(),
	 * though this will be inaccurate for large attitude errors, but it will be ok for now.
	 * 4.2 Angular_acceleration_reference = D * (rate_reference – rate_measurement)
	 */

	/*
	 * 5. Update the For each axis: delta_command = 1/control_effectiveness * (angular_acceleration_reference – angular_acceleration)
	 */

	/*
	 * 6. Add delta_commands to commands and bound to allowable values
	*/


}
