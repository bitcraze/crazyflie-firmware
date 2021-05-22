English | [中文](./README.zh-CN.md)
## ✨Introduction
This repo contains the source code of [*Swarm Ranging Protocol*](http://twinhorse.net/papers/SZLLW-INFOCOM21p.pdf), a UWB ranging protocol for dynamic and dense swarm of robots and devices.

This repo is forked from the [Official Firmware](https://github.com/bitcraze/crazyflie-firmware) and all our work is stored in the `crazyflie-firmware/src/deck/drivers/src/swarming` folder.

## 🔨Build

Clone this repository.

```
git clone --recursive https://github.com/SEU-NetSI/crazyflie-firmware.git
```

Go to the `swarming` folder.

```
cd crazyflie-firmware/src/deck/drivers/src/swarming
```

Build the firmware.

```
make clean
make
```

Flash the **cf2.bin**.

```
cfloader flash path/to/cf2.bin stm32-fw
```

