---
title: Loco UWB TX power
page_id: loco-tx-power-config
---

The transmit power used in the Ultra Wide Band radios in the Loco positioning system can be configured, both in the
Crazyflie and the Nodes. Increasing the TX power increases the distance that the radio reaches.

Note that the rules and regulations of different countries may restrict which settings that are allowed.

## Configuring the Crazyflie

The TX power of the Loco deck UWB radio is configured at build time through a [kbuild setting](/docs/development/kbuild/)
. There are only two
options, default and max power.

Note that the Loco deck is only transmitting in TWR mode or when setting anchor positions. Setting full TX power for the
Loco deck does not increase the space that a TDoA system covers.

To change to full power:
1. In a terminal, run `make meuconfig`
2. Go to `Expansion deck configuration`
3. Check the `Full TX power` option under the `Support the Loco positioning deck` section
4. Save and exit

Build and flash the firmware to the Crazyflie

## Configuring the Node

See the [node documentation](https://www.bitcraze.io/documentation/repository/lps-node-firmware/master/user-guides/configure-tx-power/) for details.
