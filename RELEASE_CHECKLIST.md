Crazyflie firmware release checklist
====================================

Build
------
 1. Generate release candidates usging the build server

Checks
---------
 - Verify that Crazyflie 1.0/2.0 still flies:
	 - Using a BLE client, Android or iPhone
     - Using the PC Crazyflie client and Crazyradio
 - Verify that the Roadrunner can connect to the pyhton client and has basic functionality
 - Verify that the Bolt can connect to the pyhton client and has basic functionality
 - Verify that LOG and params are still working
 - Verify that the radio connectivity is still working properly
     - Verify swarm connectivity using the multi test bench
 - Veirfy basic functionality of all decks
 - Verify the examples in the python lib
