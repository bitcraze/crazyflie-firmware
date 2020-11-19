# API App for Crazyflie 2.X

This folder contains an app layer application for the Crazyflie that is intended to document which functions that are part of the
app API. This is not an example app with useful code, see the example directroy for examples of how to use functions.

The app is built by CI servers to make sure we do not modify functions that is part of the official app API by misstake.
Please add any new functions that should be part of the API to it.

See App layer API guide [here](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/userguides/app_layer/)

## How to use the app

Do NOT flash and use it! It does not do anything useful and may have unpredictable behaviour, it is only intended to be compiled but not to run.

## Build

Make sure that you are in the app_api folder (not the main folder of the crazyflie firmware). Then type the following to build and flash it while the crazyflie is put into bootloader mode:

```
make clean
make
```
