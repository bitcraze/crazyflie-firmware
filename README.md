# Flying Pen  [![CI](https://github.com/bitcraze/crazyflie-firmware/workflows/CI/badge.svg)](https://github.com/bitcraze/crazyflie-firmware/actions?query=workflow%3ACI)

---

## Project Note

**Flying Pen Project**  
Mobile Robotics Lab

Forked from the official Bitcraze crazyflie-firmware repository on **December 8, 2025**, and extended for research on aerial physical interaction and contact-based control using the Crazyflie platform.


### Branches

- **`flyingpen`**: Stable branch (verified working version)
- **`flyingpen_temp`**: Development branch (work in progress)

---

## How to Clone

```bash
git clone --recursive https://github.com/SEOSUK/crazyflie-firmware.git
cd crazyflie-firmware
git checkout flyingpen
```

## How to Build

```bash
cd <your_directory>
make distclean
make cf21bl_defconfig
make -j$(nproc)
```


---

## Legacy (Upstream Documentation)

> The following content is copied from the original Bitcraze
> crazyflie-firmware repository and is kept here for reference.
>  
> No functional changes are introduced in this section.



# Crazyflie firmware  [![CI](https://github.com/bitcraze/crazyflie-firmware/workflows/CI/badge.svg)](https://github.com/bitcraze/crazyflie-firmware/actions?query=workflow%3ACI)

This project contains the source code for the firmware used in the Crazyflie range of platforms, including the Crazyflie 2.x and the Roadrunner.

### Crazyflie 1.0 support

The 2017.06 release was the last release with Crazyflie 1.0 support. If you want
to play with the Crazyflie 1.0 and modify the code, please clone this repo and
branch off from the 2017.06 tag.

## Building and Flashing
See the [building and flashing instructions](https://github.com/bitcraze/crazyflie-firmware/blob/master/docs/building-and-flashing/build.md) in the github docs folder.


## Official Documentation

Check out the [Bitcraze crazyflie-firmware documentation](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/) on our website.

## Generated documentation

The easiest way to generate the API documentation is to use the [toolbelt](https://github.com/bitcraze/toolbelt)

```tb build-docs```

and to view it in a web page

```tb docs```

## Contribute
Go to the [contribute page](https://www.bitcraze.io/contribute/) on our website to learn more.

### Test code for contribution

To run the tests please have a look at the [unit test documentation](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/development/unit_testing/).

## License

The code is licensed under LGPL-3.0
