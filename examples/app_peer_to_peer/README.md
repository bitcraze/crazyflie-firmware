# Peer to Peer App for Crazyflie 2.X

This folder contains the app layer application for the Crazyflie to send and receive peer to peer messages. The debug messages of the received messages can be read in the console tab of the [cfclient](https://github.com/bitcraze/crazyflie-clients-python). Two Crazyflies need to be flashed with this program in order to work. Make sure that they are both on the same channel, and that they have different IDs.

This example is going to blink the M4 LED when packets are sent and received, if you run 2 Crazyflies with this example you should see M4 blinking red and green in sequence indicating bidirectional P2P communication.

You can find on Bitcraze's website the [API documentation for P2P](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/p2p_api/) as well as the App layer API guide and build instructions [here](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/userguides/app_layer/)
## Limitations

Since P2P communication happens asynchronously on the radio, this example does not work well when connecting a PC to the Crazyflies via the Radio. You should connect the Crazyflies using the USB port. This is a fundamental limitation of the current P2P implementation.
