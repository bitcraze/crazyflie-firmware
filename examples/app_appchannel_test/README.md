# Appchannel test app for Crazyflie 2.x

This application demonstrates how to use the appchannel API to send and receive
radio packets between a Crazyflie app and the python lib.

This demo defines a protocol where the Crazyflie waits for 3 floas (x, y, z) and sends back the sum as one float.

To run this example, compile and flash the app with ```make && make cload```.

When the Crazyflie is flashed and started, you can run the python example with ```python3 tools/appchannelTest.py```.