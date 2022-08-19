---
title: Lighthouse more than 4 base stations
page_id: lh_multi_base_stations
---

4 base stations cover around 8x8 meters of flying space and if it was possible to add more base stations larger flying
spaces could be used. The lighthouse positioning system currently supports only 4 base stations, but most of the code
is (more or less) in reality working with up to 16. This guide will outline the necessary steps to set up a larger
system. Please first consult the
["Getting started with the Lighthouse system"](https://www.bitcraze.io/documentation/tutorials/getting-started-with-lighthouse/)
guide on our web to get a good understanding of basic use.

1. Make sure that all base stations have a unique channel like 1, 2, 3, 4, 5 and so on.  Use the "Set BS channel" button in the Lighthouse tab
in the python client for this task
2. In placement of the LH base stations, make sure that they must overlap, but avoid too many in one spot. The lighthouse deck can not handle more than 4 visible
at the same time.
3. Re-flash the Crazyflie with support for more base stations. Run `make menuconfig` and go to the `Expansion deck configuration`
menu and set `Max number of base stations` to the desired value. Note: more base stations use more RAM. Build the code and
flash it to the Crazyflie, see the documentation in this repo for [instructions on building and flashing](/docs/building-and-flashing/build.md)
