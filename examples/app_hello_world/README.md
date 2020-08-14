# Hello world App for Crazyflie 2.X

This folder contains the app layer application for the Crazyflie to print a hello world debug message, which can be read in the console tab of the [cfclient](https://github.com/bitcraze/crazyflie-clients-python). 

See App layer API guide [here](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/userguides/app_layer/)

## Build

Make sure that you are in the app_hello_world folder (not the main folder of the crazyflie firmware). Then type the following to build and flash it while the crazyflie is put into bootloader mode:

```
make clean
make 
make cload
```

If you want to compile the application elsewhere in your machine, just make sure that this line in the Makefile points to the right location of the crazyflie-firmware repo:

```
CRAZYFLIE_BASE=../../../crazyflie-firmware
```