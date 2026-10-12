# AI-deck State Streamer for Crazyflie 2.x

This folder contains an app layer application that streams the state estimate of the Crazyflie to the GAP8 on the AI-deck, 100 times per second over UART1. The GAP8 attaches the latest state to each camera frame, which lets the NanoCockpit viewer show it and save it alongside the images.

It is the Crazyflie-side counterpart to the NanoCockpit examples in the [aideck-gap8-examples](https://github.com/bitcraze/aideck-gap8-examples) repository. Both sides need to be flashed for the state to reach the host.

The message format comes from [NanoCockpit](https://github.com/idsia-robotics/nanocockpit), developed by Elia Cereda, Alessandro Giusti and Daniele Palossi (IDSIA, USI-SUPSI; IIS, ETH Zürich).

## Limitations

- UART1 is also used by the Lighthouse deck and the camera deck, so this app cannot be combined with them.
- The state is read from the `stateEstimateZ` log variables. The values are read one by one and stamped when read, so a message is not an exact snapshot of one estimator update (the difference is about a millisecond). Upstream NanoCockpit modifies the Crazyflie firmware to forward a timestamped snapshot instead.

See the App layer API guide and build instructions [here](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/userguides/app_layer/)
