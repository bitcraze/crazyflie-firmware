/*
 *
 * Copyright (c) 2019 Andre Luis Ogando Paraense and Ewoud Smeur
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

void controllerINDIInit(void)
{

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
	 * 2 - Calculate the derivative with finite difference.
	 * 3 - same filter on the actuators (or control_t values), using the commands from the previous timestep.
	 * 3.1 - Calculate the desired angular acceleration by:
	 * 3.2 - Rate_reference = P * attitude_error, where attitude error can be calculated with your favorite algorithm. You may even use a function that is already there, such as attitudeControllerCorrectAttitudePID(), though this will be inaccurate for large attitude errors, but it will be ok for now.
	 * 4. Angular_acceleration_reference = D * (rate_reference – rate_measurement)
	 * 5. Update the For each axis: delta_command = 1/control_effectiveness * (angular_acceleration_reference – angular_acceleration)
	 * 6. Add delta_commands to commands and bound to allowable values
	 */

}
