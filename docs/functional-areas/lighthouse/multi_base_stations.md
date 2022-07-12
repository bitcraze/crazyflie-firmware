---
title: Lighthouse more than 2 base stations
page_id: lh_multi_base_stations
---

WARNING: This is experimental functionality and may change over time, it is possibly instable or unreliable.

## Using 2+ base stations

2 base stations cover around 4x4 meters of flying space and if it was possible to add more base stations larger flying
spaces could be used. The lighthouse positioning system currently supports only 2 base stations, but most of the code
is (more or less) in reality working with up to 16. This guide will outline the necessary steps to set up a larger
system. Please first consult the ["Getting started with the Lighthouse system"](https://www.bitcraze.io/documentation/tutorials/getting-started-with-lighthouse/)
guide on our web to get a good understanding of basic use.

1. Configure the base stations to use channels 1, 2, 3 and so on. Use the "Set BS channel" button in the Lighthouse tab
in the python client for this task
2. Place your base stations. They must overlap, but avoid too many in one spot, there should not be more than 4 visible
at the same time.
3. Re-flash the Crazyflie with support for more base stations. Run `make menuconfig` and go to the `Expansion deck configuration`
menu and set `Max number of base stations` to the desired value. Note: more base stations use more RAM. Build the code and
flash it to the Crazyflie, see the documentation in this repo for instructions on building and flashing.
4. Acquire calibration data from all the base stations in the system. The Crazyflie gets the calibration data from the
base stations through the light sweeps and this process takes from 20 seconds and up. If more than 2 base stations are
visible at the same time, the process may take longer and it might be a good idea to hold the Crazyflie closer to one
base station at a time. Use the Lighthouse tab in the crazyflie client and the base station status matrix to see for which
base stations the Crazyflie has received calibration data.
As the calibration data is stored in the Crazyflie, it is only necessary to do this for new base stations or when the
channel is changed.
5. Estimate system geometry. The geometry estimation functionality in the client only works for 1 or 2 base stations
and it can not be used for larger systems. Instead you should run the `examples/lighthouse/multi_bs_geometry_estimation.py`
script in the [`crazyflie-lib-python` repository](https://github.com/bitcraze/crazyflie-lib-python). The script
will display instructions and the end result will be an estimated system geometry that is written to the Crazyflie.
6. The system is now ready to use! You can connect to the Crazyflie and open the Lighthouse tab to see a 3D rendering
of base stations and the Crazyflie position. Note: The client only shows the full status of the 2 first base stations.
7. Give us feedback! We still need more testing, especially for the initial position estimation. So please
let us know if you've tested it and how it worked.

Enjoy!

## System geometry estimation

The system geometry estimation process described in step 5 above, can be used for 2 or more base stations. It is
better than the standard method currently used in the client and it provides a solution that hopefully is better
aligned with the physical world. Errors should be smaller and more equally distributed in the flying space which
leads to less "jerking" when base stations are obscured or when there is interference. The estimated position should
also have a better linearity, that is if you instruct the Crazyflie to follow a straight line, it should actually do
that and not a curved line.

It is possible to visualize the estimated geometry in the `multi_bs_geometry_estimation.py` script by setting the
`visualize_positions` variable to True (requires pyplot).
