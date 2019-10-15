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
