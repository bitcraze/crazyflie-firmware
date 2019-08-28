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

static float thrust_threshold = 300.0f;
static float bound_control_input = 20000.0f;

static struct IndiVariables indi = {
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
  .adaptive = STABILIZATION_INDI_USE_ADAPTIVE,
};

static inline void float_rates_zero(struct FloatRates *fr) {
	fr->p = 0.0f;
	fr->q = 0.0f;
	fr->r = 0.0f;
}

void indi_init_filters(void)
{
  // tau = 1/(2*pi*Fc)
  float tau = 1.0f / (2.0f * PI * STABILIZATION_INDI_FILT_CUTOFF);
  float tau_r = 1.0f / (2.0f * PI * STABILIZATION_INDI_FILT_CUTOFF_R);
  float tau_axis[3] = {tau, tau, tau_r};
  float tau_est = 1.0f / (2.0f * PI * STABILIZATION_INDI_ESTIMATION_FILT_CUTOFF);
  float sample_time = 1.0f / ATTITUDE_RATE;
  // Filtering of gyroscope and actuators
  for (int8_t i = 0; i < 3; i++) {
    init_butterworth_2_low_pass(&indi.u[i], tau_axis[i], sample_time, 0.0f);
    init_butterworth_2_low_pass(&indi.rate[i], tau_axis[i], sample_time, 0.0f);
    init_butterworth_2_low_pass(&indi.est.u[i], tau_est, sample_time, 0.0f);
    init_butterworth_2_low_pass(&indi.est.rate[i], tau_est, sample_time, 0.0f);
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
    output[i] = (filter[i].o[0] - filter[i].o[1]) * ATTITUDE_RATE;
  }
}

void controllerINDIInit(void)
{
	/*
	 * TODO
	 * Can this also be called during flight, for instance when switching controllers?
	 * Then the filters should not be reset to zero but to the current values of sensors and actuators.
	 */
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
	 * Skipping calls faster than ATTITUDE_RATE
	 */
	if (!RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
	    return;
	}

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
	 float referenceAttitudeRateRoll = radians(setpoint->attitudeRate.roll);
	 float referenceAttitudeRatePitch = -radians(setpoint->attitudeRate.pitch);
	 float referenceAttitudeRateYaw = radians(setpoint->attitudeRate.yaw);

	 indi.angular_accel_ref.p = indi.reference_acceleration.rate_p * (referenceAttitudeRateRoll - body_rates.p);
	 indi.angular_accel_ref.q = indi.reference_acceleration.rate_q * (referenceAttitudeRatePitch - body_rates.q);
	 indi.angular_accel_ref.r = indi.reference_acceleration.rate_r * (referenceAttitudeRateYaw - body_rates.r);

	/*
	 * 5. Update the For each axis: delta_command = 1/control_effectiveness * (angular_acceleration_reference – angular_acceleration)
	 */

	 //Increment in angular acceleration requires increment in control input
	 //G1 is the control effectiveness. In the yaw axis, we need something additional: G2.
	 //It takes care of the angular acceleration caused by the change in rotation rate of the propellers
	 //(they have significant inertia, see the paper mentioned in the header for more explanation)
	 indi.du.p = 1.0f / indi.g1.p * (indi.angular_accel_ref.p - indi.rate_d[0]);
	 indi.du.q = 1.0f / indi.g1.q * (indi.angular_accel_ref.q - indi.rate_d[1]);
	 indi.du.r = 1.0f / (indi.g1.r + indi.g2) * (indi.angular_accel_ref.r - indi.rate_d[2] + indi.g2 * indi.du.r);


	/*
	 * 6. Add delta_commands to commands and bound to allowable values
	*/

	 indi.u_in.p = indi.u[0].o[0] + indi.du.p;
	 indi.u_in.q = indi.u[1].o[0] + indi.du.q;
	 indi.u_in.r = indi.u[2].o[0] + indi.du.r;

	  //bound the total control input
	if(STABILIZATION_INDI_FULL_AUTHORITY){
	  indi.u_in.p = clamp(indi.u_in.p, -1.0f*bound_control_input, bound_control_input);
	  indi.u_in.q = clamp(indi.u_in.q, -1.0f*bound_control_input, bound_control_input);
	  float rlim = bound_control_input - fabsf(indi.u_in.q);
	  indi.u_in.r = clamp(indi.u_in.r, -rlim, rlim);
	  indi.u_in.r = clamp(indi.u_in.r, -1.0f*bound_control_input, bound_control_input);
	}else{
	  indi.u_in.p = clamp(indi.u_in.p, -1.0f*bound_control_input, bound_control_input);
	  indi.u_in.q = clamp(indi.u_in.q, -1.0f*bound_control_input, bound_control_input);
	  indi.u_in.r = clamp(indi.u_in.r, -1.0f*bound_control_input, bound_control_input);
	}

	 //Propagate input filters
	 //first order actuator dynamics
	 indi.u_act_dyn.p = indi.u_act_dyn.p + STABILIZATION_INDI_ACT_DYN_P * (indi.u_in.p - indi.u_act_dyn.p);
	 indi.u_act_dyn.q = indi.u_act_dyn.q + STABILIZATION_INDI_ACT_DYN_Q * (indi.u_in.q - indi.u_act_dyn.q);
	 indi.u_act_dyn.r = indi.u_act_dyn.r + STABILIZATION_INDI_ACT_DYN_R * (indi.u_in.r - indi.u_act_dyn.r);

	 indi.thrust = setpoint->thrust;

	 //Don't increment if thrust is off
	 //TODO: this should be something more elegant, but without this the inputs
	 //will increment to the maximum before even getting in the air.
	 if(indi.thrust < thrust_threshold) {

		 controllerINDIInit();
	 }

	 /*  INDI feedback */
	 control->thrust = indi.thrust;
	 control->roll = indi.u_in.p;
	 control->pitch = indi.u_in.q;
	 control->yaw  = indi.u_in.r;
}

PARAM_GROUP_START(ctrlINDI)
PARAM_ADD(PARAM_UINT8, adptive, &indi.adaptive)
PARAM_ADD(PARAM_FLOAT, thrust_threshold, &thrust_threshold)
PARAM_ADD(PARAM_FLOAT, bound_ctrl_input, &bound_control_input)
PARAM_GROUP_STOP(ctrlINDI)

LOG_GROUP_START(ctrlINDI)
LOG_ADD(LOG_FLOAT, cmd_thrust, &indi.thrust)
LOG_ADD(LOG_FLOAT, cmd_roll, &indi.u_in.p)
LOG_ADD(LOG_FLOAT, cmd_pitch, &indi.u_in.q)
LOG_ADD(LOG_FLOAT, cmd_yaw, &indi.u_in.r)
LOG_ADD(LOG_FLOAT, u_act_dyn.p, &indi.u_act_dyn.p)
LOG_ADD(LOG_FLOAT, u_act_dyn.q, &indi.u_act_dyn.q)
LOG_ADD(LOG_FLOAT, u_act_dyn.r, &indi.u_act_dyn.r)
LOG_ADD(LOG_FLOAT, du.p, &indi.du.p)
LOG_ADD(LOG_FLOAT, du.q, &indi.du.q)
LOG_ADD(LOG_FLOAT, du.r, &indi.du.r)
LOG_ADD(LOG_FLOAT, ang_accel_ref.p, &indi.angular_accel_ref.p)
LOG_ADD(LOG_FLOAT, ang_accel_ref.q, &indi.angular_accel_ref.q)
LOG_ADD(LOG_FLOAT, ang_accel_ref.r, &indi.angular_accel_ref.r)
LOG_ADD(LOG_FLOAT, rate_d[0], &indi.rate_d[0])
LOG_ADD(LOG_FLOAT, rate_d[1], &indi.rate_d[1])
LOG_ADD(LOG_FLOAT, rate_d[2], &indi.rate_d[2])
LOG_ADD(LOG_FLOAT, ref_accel.err_p, &indi.reference_acceleration.err_p)
LOG_ADD(LOG_FLOAT, ref_accel.err_q, &indi.reference_acceleration.err_q)
LOG_ADD(LOG_FLOAT, ref_accel.err_r, &indi.reference_acceleration.err_r)
LOG_ADD(LOG_FLOAT, ref_accel.rate_p, &indi.reference_acceleration.rate_p)
LOG_ADD(LOG_FLOAT, ref_accel.rate_q, &indi.reference_acceleration.rate_q)
LOG_ADD(LOG_FLOAT, ref_accel.rate_r, &indi.reference_acceleration.rate_r)
LOG_GROUP_STOP(ctrlINDI)
