# Peer to Peer App for Crazyflie 2.X

This folder contains the app layer application for the Crazyflie to send and receive peer to peer messages. The debug messages of the received messages can be read in the console tab of the [cfclient](https://github.com/bitcraze/crazyflie-clients-python). Two Crazyflies need to be flashed with this program in order to work. Make sure that they are both on the same channel, and that they have different IDs.

API documentation for P2P can be found [here](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/p2p_api/). See the App layer API guide [here](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/userguides/app_layer/)

## Build

Make sure that you are in the app_peer_to_peer folder (not the main folder of the crazyflie firmware). Then type the following to build and flash it while the crazyflie is put into bootloader mode:

```
make clean
make 
make cload
```

If you want to compile the application elsewhere in your machine, just make sure that this line in the Makefile points to the right location of the crazyflie-firmware repo:

```
CRAZYFLIE_BASE=../../../crazyflie-firmware
```