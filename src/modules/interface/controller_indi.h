/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
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
 * controller_indi.h - INDI Controller Interface
 */
#ifndef __CONTROLLER_INDI_H__
#define __CONTROLLER_INDI_H__

#include "stabilizer_types.h"
#include "filter.h"
#include "math3d.h"

//The G values are scaled to avoid numerical problems during the estimation
#define INDI_EST_SCALE 0.001

// these parameters are used in the filtering of the angular acceleration
#define STABILIZATION_INDI_FILT_CUTOFF 8.0

// the yaw sometimes requires more filtering
#define STABILIZATION_INDI_FILT_CUTOFF_R STABILIZATION_INDI_FILT_CUTOFF

#define STABILIZATION_INDI_ESTIMATION_FILT_CUTOFF 4.0

#define PERIODIC_FREQUENCY 512

// these parameters are used in the filtering of the angular acceleration
#define STABILIZATION_INDI_MAX_RATE 6.0
#define STABILIZATION_INDI_MAX_R 120.0
#define STABILIZATION_INDI_G1_P 0.0639
#define STABILIZATION_INDI_G1_Q 0.0361
#define STABILIZATION_INDI_G1_R 0.0022
#define STABILIZATION_INDI_G2_R 0.1450
#define STABILIZATION_INDI_REF_ERR_P 600.0
#define STABILIZATION_INDI_REF_ERR_Q 600.0
#define STABILIZATION_INDI_REF_ERR_R 600.0
#define STABILIZATION_INDI_REF_RATE_P 28.0
#define STABILIZATION_INDI_REF_RATE_Q 28.0
#define STABILIZATION_INDI_REF_RATE_R 28.0
#define STABILIZATION_INDI_USE_ADAPTIVE false
#define STABILIZATION_INDI_ADAPTIVE_MU 0.0001

/**
 * @brief angular rates
 * @details Units: rad/s */
struct FloatRates {
  float p; ///< in rad/s
  float q; ///< in rad/s
  float r; ///< in rad/s
};

struct ReferenceSystem {
  float err_p;
  float err_q;
  float err_r;
  float rate_p;
  float rate_q;
  float rate_r;
};

struct IndiEstimation {
  Butterworth2LowPass u[3];
  Butterworth2LowPass rate[3];
  float rate_d[3];
  float rate_dd[3];
  float u_d[3];
  float u_dd[3];
  struct FloatRates g1;
  float g2;
  float mu;
};

struct IndiVariables {
  struct FloatRates angular_accel_ref;
  struct FloatRates du;
  struct FloatRates u_in;
  struct FloatRates u_act_dyn;
  float rate_d[3];

  Butterworth2LowPass u[3];
  Butterworth2LowPass rate[3];
  struct FloatRates g1;
  float g2;

  struct ReferenceSystem reference_acceleration;

  bool adaptive;             ///< Enable adataptive estimation
  float max_rate;            ///< Maximum rate in rate control in rad/s
  float attitude_max_yaw_rate; ///< Maximum yaw rate in atttiude control in rad/s
  struct IndiEstimation est; ///< Estimation parameters for adaptive INDI
};

#define FLOAT_RATES_ZERO(_r) {          \
    RATES_ASSIGN(_r, 0., 0., 0.);       \
}

/* ra =  {p, q, r} */
#define RATES_ASSIGN(_ra, _p, _q, _r) {   \
    (_ra).p = (_p);       \
    (_ra).q = (_q);       \
    (_ra).r = (_r);       \
  }

void controllerINDIInit(void);
bool controllerINDITest(void);
void controllerINDI(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick);

#endif //__CONTROLLER_INDI_H__
