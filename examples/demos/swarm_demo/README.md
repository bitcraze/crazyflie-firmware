# Demo application

This folder contains an app layer application for the Crazyflie that is used as a demo.

## Build

Make sure that you are in the demo folder (not the main folder of the crazyflie firmware). Then type the following to build and flash it while the crazyflie is put into bootloader mode:

```
make clean
make
make cload
```

## Running the demo

The demo is started by running the control_tower.py script
