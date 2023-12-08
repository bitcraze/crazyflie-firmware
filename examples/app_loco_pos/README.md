# Simple position estimator for the Loco positioning system in TDoA mode

This folder contains the app layer application for the Crazyflie that contains an Out Of Tree estimator.

The estimator is a simple position (only) estimator that reuses the measurement model and outlier filter from the
kalman estimator. The estimated position is continuously updated with the h vector from the measurement model
implementation. The mixin factor determines how much influence the update has.

Set the stabilizer.estimator parameter to an appropriate value (3 by default) to enable the estimator.

This estimator is not intended for flying, but might be useful for other tracking applications.

See App layer API guide and build instructions [here](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/userguides/app_layer/)
