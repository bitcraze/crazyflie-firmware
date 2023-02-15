---
title: APIs, versions and deprecations
page_id: api-deprecation
---

There are a number of APIs in the Crazyflie eco system on multiple levels, some might be obvious while others are a bit
more subtle. Some of the APIs are documented and versioned while there is room for improvement in other cases.

Since all code can be used by a programmer, one point of view would be that all functions and protocols are APIs, and
should not change without some form of version change. To
make it possible for us to manage the code base, we have chosen a few areas (APIs) that we will update in a structured way
(see below). Code that does not belong to these APIs can be changed at any time, which we believe is necessary to keep
the system dynamic and evolve over time.

The purpose of managing APIs in a structured way is to help a developer to understand if a protocol or a set of
functions is likely to change over time and if it will break the application. It should also be possible to detect if
(and how) the API has changed and what changes that are required to the code to make the application work.

## The update process and deprecations

Areas that can be versioned using a version number, for instance protocols, should update the version number when
something (of instance the protocol) is changed. Check the documentation for the specific area/protocol to see if there
is a specific update policy for this area.

For functions in the code base, there is not really a version number tied to the function (except the release version) that
clearly tells the programmer if there has been an API break or not. The approach we use is to mark functions that we
want to remove as 'deprecated`. The deprecated function will continue to exist and work for a minimum of 6 months before
being removed. Deprecated functions are documented in the release notes to give a heads up that they will be removed in
a future release.

## Current APIs with structured updates

The following areas and protocols are updated with a structured process:

* [CRTP](/docs/functional-areas/crtp)
* App API - the functions called from `app_api/src/app_main.c`
* [Parmeters](/docs/api/params.md) - parameters marked as `CORE`
* [Logs](/docs/api/logs.md) - logs marked as `CORE`
* [Trajectory formats](/docs/functional-areas/trajectory_formats.md)
* [Loco TWR protocol](https://www.bitcraze.io/documentation/repository/lps-node-firmware/master/protocols/twr-protocol/)
* [Loco TDoA2 protocol](https://www.bitcraze.io/documentation/repository/lps-node-firmware/master/protocols/tdoa2_protocol/)
* [Loco TDoA3 protocol](https://www.bitcraze.io/documentation/repository/lps-node-firmware/master/protocols/tdoa3_protocol/)
