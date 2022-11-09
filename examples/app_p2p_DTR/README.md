# Dynamic Token Ring Protocol for Crazyflie 2.X

This folder contains the app layer application for the Crazyflie to send and receive peer to peer messages while utilising the DTR(Dynamic Token Ring) protocol. This protocol is used to ensure that each time only one Crazyflie broadcasts data which leads to less packet collisions and losses. It also provides a way to ensure that the transmitted data will be sent to the other copters since it receives an acknowledgement from each receiver.

The debug messages of the received messages can be read in the console tab of the [cfclient](https://github.com/bitcraze/crazyflie-clients-python). Four Crazyflies with ids 0-3 need to be flashed with this program in order to work. Make sure that they are both on the same channel, and that they have different IDs.

This example is going to be used to send and receive messages from the Crazyflie with ID 0 to all the other Crazyflies with ID 1-3. When ID 1 receives the message with the first byte of data being 104, it will send a reply message with the same data but the first byte being 123. The verification of the received messages can be done by looking at the console tab of the client and the amount of time each broadcast took.

You can find on Bitcraze's website the [API documentation for Token Ring Protocol](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/DTR_p2p_api/) as well as the app layer API guide and build instructions [here](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/userguides/app_layer/)

## Limitations

Since P2P communication happens asynchronously on the radio, this example does not work well when connecting a PC to the Crazyflies via the Radio. You should connect the Crazyflies using the USB port. This is a fundamental limitation of the current P2P implementation.
