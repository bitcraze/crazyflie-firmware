/** 
 * The robust M-estimation-based Kalman filter is originally implemented as a part of the work in
 * the below-cited paper. 
 * 
 * "Learning-based Bias Correction for Time Difference of Arrival Ultra-wideband Localization of 
 *  Resource-constrained Mobile Robots"
 *
 * Academic citation would be appreciated.
 *
 * BIBTEX ENTRIES:
      @ARTICLE{WendaBiasLearning2021,
      author={Wenda Zhao, Jacopo Panerati, and Angela P. Schoellig},
      title={Learning-based Bias Correction for Time Difference of Arrival Ultra-wideband Localization of 
             Resource-constrained Mobile Robots},
      journal={IEEE Robotics and Automation Letters},
      year={2021},
      publisher={IEEE}
 *
 * The authors are with the Dynamic Systems Lab, Institute for Aerospace Studies,
 * University of Toronto, Canada, and affiliated with the Vector Institute for Artificial
 * Intelligence in Toronto. 
 * ============================================================================
 */

#pragma once

#include "kalman_core.h"

// M-estimation based robust Kalman filter update for UWB TWR measurements
void kalmanCoreRobustUpdateWithDistance(kalmanCoreData_t* this, distanceMeasurement_t *d); 