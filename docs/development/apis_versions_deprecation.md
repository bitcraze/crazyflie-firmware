---
title: APIs, versions and deprecations
page_id: api-deprecation
---

Please see the [general information on versions and how the code base is managed](https://www.bitcraze.io/development/development-overview/#apis-versioning-and-evolution-of-the-code-base)
for information on how we handle APIs, versions and deprecations.

The following areas and protocols in this repo are updated with a structured process and should be considered to be
more stable. Applications using these APIs will be easier to maintain over time.

* App API - the functions called from `app_api/src/app_main.c`
* [Parameters](/docs/api/params.md) - parameters marked as `CORE`
* [Logs](/docs/api/logs.md) - logs marked as `CORE`
* [Memory subsystem](/docs/functional-areas/memory-subsystem)
* [Trajectory formats](/docs/functional-areas/trajectory_formats.md)
* [CRTP](/docs/functional-areas/crtp)
* [Loco TWR protocol](https://www.bitcraze.io/documentation/repository/lps-node-firmware/master/protocols/twr-protocol/)
* [Loco TDoA2 protocol](https://www.bitcraze.io/documentation/repository/lps-node-firmware/master/protocols/tdoa2_protocol/)
* [Loco TDoA3 protocol](https://www.bitcraze.io/documentation/repository/lps-node-firmware/master/protocols/tdoa3_protocol/)
