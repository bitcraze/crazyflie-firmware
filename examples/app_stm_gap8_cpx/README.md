# STM-GAP8 CPX communication example

This folder contains the app layer application for an example app that communicates with the GAP8 on the AI-deck.
The intention is to show how CPX can be used to feed data from the STM to the GAP8 or the other way around.
One possible use case for this type of communication could be to control the Crzyflie from a neural network for instance.

This application is intended to be used with an AI-deck, together with the "stm_gap8_cpx" example in the
[aideck-gap8-examples](https://github.com/bitcraze/aideck-gap8-examples) repository.

CPX packets are sent from the STM (this app) to the GAP8, containing a counter that is increased for each packet. The
same number is sent back to the STM in a new CPX packet and the number is printed on the console, that is when a
number is printed on the console, the counter value has done a round trip STM-GAP8-STM.


See the [CPX documentation](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/cpx/).

See App layer API guide and build instructions [here](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/userguides/app_layer/)
