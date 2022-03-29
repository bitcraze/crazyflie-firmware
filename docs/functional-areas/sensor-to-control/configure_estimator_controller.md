---
title: Configure estimator and controller
page_id: config_est_ctrl
---

All the estimators and controllers are part of the standard firmware build and the active estimator and controller can be configured in runtime or compile time.

## Estimators

The available estimators are defined in the `StateEstimatorType` enum in `src/modules/interface/estimator.h`.

### Setting in runtime

To activate a specific estimator, set the `stabilizer.estimator` parameter to the appropriate value based on the `StateEstimatorType`.

The parameter can be set from the [python client](https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/), the [python lib](https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/) or from [an app in on-board the Crazyflie](/docs/userguides/app_layer.md).

### Default estimator

The complementary estimator is the default estimator.

Some decks require the Kalman estimator and it is automatically activated when one of these decks are detected. The activated estimator is based on the `.requiredEstimator` member of [the DeckDriver API](/docs/userguides/deck.md).

### Setting default estimator at compile time

It is possible to force the use of a specific estimator at compile time by setting `ESTIMATOR`, see [Configure the build in kbuild](/docs/development/kbuild.md).

Example:

`ESTIMATOR=kalman`

## Controller

The available controllers are defined in the `ControllerType` enum in `src/modules/interface/controller.h`.

### Setting in runtime

To activate a specific controller, set the `stabilizer.controller` parameter to the appropriate value based on the `ControllerType`.

The parameter can be set from the python client, the python lib or from an app in the Crazyflie.

### Default controller

The PID controller is the default controller.

### Setting at compile time

It is possible to force the use of a specific controller at compile time by setting `CONTROLLER`, see [Configure the build in kbuild](/docs/development/kbuild.md).

Example:

`CONTROLLER=Mellinger`
