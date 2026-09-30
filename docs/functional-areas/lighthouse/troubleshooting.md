---
title: Lighthouse troubleshooting
page_id: lh_troubleshooting
---

This page lists common causes of poor or unstable positioning with the Lighthouse system, and what to check.
Make sure to first follow the
["Getting started with the Lighthouse system"](https://www.bitcraze.io/documentation/tutorials/getting-started-with-lighthouse/)
tutorial.

## Environment

* **Reflective surfaces** - Mirrors, windows, glossy floors and other shiny surfaces can reflect the laser sweeps and
  give false measurements. If you suspect a reflection, cover the surface and see if it makes a difference.
* **Other infrared sources** - Direct sunlight, IR lamps, motion capture cameras and other IR emitters can
  disturb the sensors on the deck.
* **Moved base stations** - The geometry is only valid as long as the base stations stay put. If a base station has
  been bumped or moved, estimate the geometry again.

## Mounting the deck

* **Blocked sensors** - The four sensors on the deck need a clear line of sight to the base stations. Make sure
  nothing blocks them, for instance long pins sticking out from the top of the deck, cables, or other decks
  mounted above the Lighthouse deck. The Lighthouse deck should be the top-most deck.
* **Power** - The FPGA on the deck is sensitive to supply voltage drops. If the voltage drops under ~2.6V, the deck resets
and the Crazyflie loses positioning data.

## Setup and configuration

* **Firmware** - Make sure the Crazyflie firmware is up to date. The deck FPGA is updated as part of the normal
  firmware flashing, with the deck mounted.
* **Base station channels** - Every V2 base station in the system must be set to a unique channel. Two base stations on
  the same channel will corrupt each other's data.
* **Calibration data** - The Crazyflie must receive calibration data from each base station before the geometry is
  estimated. Keep the Crazyflie in view of all base stations for about 20 seconds before starting the estimation.

