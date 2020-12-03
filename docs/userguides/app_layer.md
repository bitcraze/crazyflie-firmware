---
title: App layer
page_id: app_layer
---

## Introduction

The app-layer is a set of functionality and APIs to allow user to add custom code to the Crazyflie.
This is still an experimental functionalities but the intention is to eventually provide documentation and APIs to easily extend the Crazyflie and implement autonomous capabilities.

## App entry-point

When compiling the Crazyflie with ```APP=1``` writen either in the Makefile or in ```tools/make/config.mk``` the firmware will call a function ```void appMain()``` from a task after the startup sequence has completed.
This function should not return.

If you want more control, you can define a function ```void appInit()```. ```appInit()``` will be called by the firmware during initialisation, no task will be created and so ```appMain()``` will not be automaticall called.
This function must return to allow the Crazyflie initialization sequence to continue.

The folowing Makefile variables can be used for configuration:

 - **APP**: Set to '1' to enable the app entry-point
 - **APP_STACKSIZE**: Set the task stack size in 32bit word (4 Bytes). The default is 300 (1.2KBytes)
 - **APP_PRIORITY**: Set the task priority between 0 and 5. Default is 0 (same as IDLE).

## Internal log and param system

For the app-layer, it would be good to have access to log and/or parameter values and to set parameter values. This way, your app will be able to read out sensor data or to switch controller/estimator on air. To check out these functions, look at src/modules/interface/log.h or .../param.h for the internal access functions. There is also an example to be found in /examples/app_internal_param_log/.

## LED sequences

It is possible to run LED sequences from the app layer to control the four LEDs on the Crazyflie and provide runtime information to the user. See the src/hal/interface/ledseq.h file for more information.

## App channel: packet based communication between the Crazyflie and the Python lib

The Appchannel API allows to communicate using radio packets with an app.
The packets can contain anything of a size up to 31 bytes, the protocol is defined by the app.

For more information about the API see the header file src/modules/interface/app_channel.h.
An example of how to use the app channel is in examples/app_appchannel_test/

## Examples

In the [example folder](https://github.com/bitcraze/crazyflie-firmware/tree/master/examples) of the crazyflie-firmware repository, there are several examples shown tha tuse the app layer, including a simple hello world example.
