# Push demo for Crazyflie 2.x

This repos contains the push demo for Cazyflie 2.x.
It uses out-of-tree build functionality of the Crazyflie firmware and is implemented using the app entry-point.

This demo works with a Crazyflie 2.x with Flow deck and Multiranger deck attached.
When running, the crazyflie will wait for an object to get close to the top ranging sensor.
When an oject (ie. an hand) has passed close to the top sensor, the Crazyflie takes off and hover.
If anything is detected on side sensors, the Crazyflie moves in the oposite direction.
So, it is possible to push the Crazyflie around.

## Build

You must have the required tools to build the [Crazyflie firmware](https://github.com/bitcraze/crazyflie-firmware).

Clone the repos with ```--recursive```. If you did not do so, pull submodules with:
```
git submodule update --init --recursive
```

Then build and bootload:
```
make -j$(nproc)
make cload
```


