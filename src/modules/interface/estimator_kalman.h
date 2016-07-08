/**
 * Authored by Michael Hamer (http://www.mikehamer.info), June 2016
 * Thank you to Mark Mueller (www.mwm.im) for advice during implementation,
 * and for derivation of the original filter in the below-cited paper.
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
 * The Kalman filter implemented in this file is based on the papers:
 *
 * "Fusing ultra-wideband range measurements with accelerometers and rate gyroscopes for quadrocopter state estimation"
 * http://ieeexplore.ieee.org/xpl/articleDetails.jsp?arnumber=7139421
 *
 * and
 *
 * "Kalman filtering with an attitude" as published in the PhD thesis "Increased autonomy for quadrocopter systems: trajectory generation, fail-safe strategies, and state estimation"
 * http://dx.doi.org/10.3929/ethz-a-010655275
 * TODO: Update the above reference once the paper has been published
 *
 * Academic citation would be appreciated.
 *
 * BIBTEX ENTRIES:
      @INPROCEEDINGS{MuellerHamer2015,
      author  = {Mueller, M. W. and Hamer, M. and D'Andrea, R.},
      title   = {Fusing ultra-wideband range measurements with accelerometers and rate gyroscopes for quadrocopter state estimation},
      booktitle = {2015 IEEE International Conference on Robotics and Automation (ICRA)},
      year    = {2015},
      month   = {May},
      pages   = {1730-1736},
      doi     = {10.1109/ICRA.2015.7139421},
      ISSN    = {1050-4729}}
      
      @PHDTHESIS {Mueller2016,
      author  = {Mueller, M. W.},
      title   = {Increased autonomy for quadrocopter systems: trajectory generation, fail-safe strategies, and state-estimation},
      school  = {ETH Zurich},
      year    = {2016},
      doi     = {10.3929/ethz-a-010655275}}
 *
 * ============================================================================
 */

#pragma once

#include <stdint.h>
#include "stabilizer_types.h"

void stateEstimatorInit(void);
void stateEstimatorUpdate(state_t *state, sensorData_t *sensors, control_t *control);
bool stateEstimatorTest(void);


/**
 * The filter supports the incorporation of additional sensors into the state estimate via the following functions:
 */
bool stateEstimatorEnqueueTDOA(tdoaMeasurement_t *uwb);
bool stateEstimatorEnqueuePosition(positionMeasurement_t *pos);
bool stateEstimatorEnqueueDistance(distanceMeasurement_t *dist);
