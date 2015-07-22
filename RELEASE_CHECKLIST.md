Crazyflie firmware release checklist
=============================

Checks
---------
 - Verify that Crazyflie 1.0/2.0 still flies:
	 - Using a BLE client, Android or iPhone
	 - Using the PC Crazyflie client and Crazyradio
 - Verify that LOG and params are still working
 - Verify that the radio connectivity is still working properly

Build
------
 1. Reset tree into a clean tree with "git reset --hard HEAD"
	 - **Warning: this removes all changes from the source tree!**
 2. Tag commit with "year.month[.patch]". For example 2014.12.2 or 2015.2 and push the tag to Github
 3. Remove or rename config.mk
 4. Build with make, without arguments

Distribute
------------

 1. Rename cf1.bin and cf2.bin into Crazyflie-{tagged-version}.bin
 2. Upload this file to Github release for this tag
