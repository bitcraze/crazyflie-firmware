Crazyflie firmware release checklist
====================================

Build
------
 1. Generate release candidates usging the build server

Checks
---------
 - Verify basic connectivity and flight with client and Crzyradio
     - CF 2.0
     - CF 2.1
     - Bolt
     - Roadrunner (no flight obiously)
 - Verify basic connectivity and flight with BLE, Android and Iphone clients
     - CF 2.0
     - CF 2.1
     - Bolt
     - Roadrunner (no flight obiously)
 - Verify that LOG and params are still working
 - Verify swarm connectivity using the multi test bench
 - Veirfy basic functionality of all decks
 - Verify the examples in the python lib
 - Verify that positioning works using python example scripts
     - LPS
     - Lighthouse
     - Mocap
     - Flow
