 [English](./README.md) | 中文

## ✨简介

[*Swarm Ranging Protocol*](http://twinhorse.net/papers/SZLLW-INFOCOM21p.pdf)是为高密集、高动态的集群所设计的超宽频测距协议，本仓库中包含了其所有源码。

本仓库基于[官方固件](https://github.com/bitcraze/crazyflie-firmware)二次开发，源代码主要放在`crazyflie-firmware/src/deck/drivers/src/swarming`文件夹。

## 🔨编译

克隆本仓库到本地

```
git clone --recursive https://github.com/SEU-NetSI/crazyflie-firmware.git
```

切换当前工作目录至`swarming`文件夹

```
cd crazyflie-firmware/src/deck/drivers/src/swarming
```

编译固件

```
make clean
make
```

烧录固件

```
cfloader flash path/to/cf2.bin stm32-fw
```







