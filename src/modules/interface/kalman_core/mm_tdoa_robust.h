/**
 * This robust M-estimation-based Kalman filter was originally implemented in
 * work by the Dynamic Systems Lab (DSL) at the University of Toronto
 * Institute for Aerospace Studies (UTIAS) and the Vector Institute for
 * Artificial Intelligence, Toronto, ON, Canada.
 *
 * It can be cited as:
   @ARTICLE{Zhao2021Learningbased,
    author={Zhao, Wenda and Panerati, Jacopo and Schoellig, Angela P.},
    title={Learning-based Bias Correction for Time Difference of Arrival
           Ultra-wideband Localization of Resource-constrained Mobile Robots},
    journal={IEEE Robotics and Automation Letters},
    year={2021},
    publisher={IEEE}}
 *
 */

#pragma once

#include "kalman_core.h"
#include "outlierFilterTdoa.h"

// M-estimation based robust Kalman filter update for UWB TDOA measurements
void kalmanCoreRobustUpdateWithTdoa(kalmanCoreData_t* this, tdoaMeasurement_t *tdoa, OutlierFilterTdoaState_t* outlierFilterState);
