

/**
 * Authored by Michael Hamer (http://www.mikehamer.info), November 2016.
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
 * ============================================================================
 *
 * The controller implemented in this file is based on the paper:
 *
 * "Nonlinear Quadrocopter Attitude Control"
 * http://e-collection.library.ethz.ch/eserv/eth:7387/eth-7387-01.pdf
 *
 * Academic citation would be appreciated.
 *
 * BIBTEX ENTRIES:
      @ARTICLE{BrescianiniNonlinearController2013,
               title={Nonlinear quadrocopter attitude control},
               author={Brescianini, Dario and Hehn, Markus and D'Andrea, Raffaello},
               year={2013},
               publisher={ETH Zurich}}
 *
 * ============================================================================
 */

#pragma once

#include "stabilizer_types.h"

void controllerBrescianiniInit(void);
bool controllerBrescianiniTest(void);
void controllerBrescianini(control_t *control,
                        const setpoint_t *setpoint,
                        const sensorData_t *sensors,
                        const state_t *state,
                        const stabilizerStep_t stabilizerStep);
