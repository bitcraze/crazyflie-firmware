# Internal parameters and loggin api App example for Crazyflie 2.X

This folder contains the app layer application for the Crazyflie to use the internal logging and parameter API, which can be monitord in the console tab of the [cfclient](https://github.com/bitcraze/crazyflie-clients-python). 

See App layer API guide [here](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/userguides/app_layer/)

## Build

Make sure that you are in the app_hello_world folder (not the main folder of the crazyflie firmware). Then type the following to build and flash it while the crazyflie is put into bootloader mode:

```
make clean
make 
make cload
```

If you want to compile the application elsewhere in your machine, make sure to update ```CRAZYFLIE_BASE``` in the **Makefile**.