---
title: TDoA3 Long Range Configuration
page_id: tdoa3-long-range-config
---

TDoA3 Long Range increases the distances that a Loco System can use when running in TDoA3, see
[the Loco Node documentation](https://www.bitcraze.io/documentation/repository/lps-node-firmware/master/user-guides/tdoa3_long_range/)
for more information.

The Crazyflie must run a special version of the firmware to use TDoA3 Long Range mode. The configuration is set using
the kbuild menu and you should:
1. Enable `Expansion deck configuration/Support the Loco positioning deck`
2. Chose `Expansion deck configuration/Algorithm to use/Use the Time Difference of Arrival (3) algorithm`
3. Enable `Expansion deck configuration/Longer range mode`

If you want to set configurations manually you should use:
``` make
CONFIG_DECK_LOCO=y
CONFIG_DECK_LOCO_ALGORITHM_TDOA3=y
CONFIG_DECK_LOCO_LONGER_RANGE=y
```
