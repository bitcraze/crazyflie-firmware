---
title: Lighthouse more than 4 base stations
page_id: lh_multi_base_stations
---

We have tested with 4 lighthouse base stations, which cover around 8x8x3 meters of flying space. The software supports
up to 16 base stations, which should make it possible to add more base stations to cover multiple rooms and maybe larger
flying spaces, but this is largely untested and experimental functionality. Note that the Lighthouse deck only works
with up to 4 visible base stations at a time, if more than 4 base stations are used they must be set up in such a way
that this condition is met.

The firmware is compiled with support for 4 base stations by default.
This guide will outline the necessary steps to set up a larger system. Please first consult the
["Getting started with the Lighthouse system"](https://www.bitcraze.io/documentation/tutorials/getting-started-with-lighthouse/)
guide on our web to get a good understanding of basic use.

1. Make sure that all base stations have a unique channel like 1, 2, 3, 4, 5 and so on.  Use the "Set BS channel" button in the Lighthouse tab
in the python client for this task
2. In placement of the LH base stations, make sure that they must overlap, but avoid too many in one spot. The
lighthouse deck can not handle more than 4 visible base stations at the same time.
3. Re-flash the Crazyflie with support for more base stations. Run `make menuconfig` and go to the `Expansion deck configuration`
menu and set `Max number of base stations` to the desired value. Note: more base stations use more RAM. Build the code and
flash it to the Crazyflie, see the documentation in this repo for [instructions on building and flashing](/docs/building-and-flashing/build.md)
