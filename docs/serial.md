---
title: Getting the serial number of the Crazyflie 2.X
page_id: serial
---

The Crazyflie 2.X ID is a character string of 24 characters. It is
unique to each Crazyflie 2.X.

Windows
-------

-   Open the \"Device manager\" and find the Crazyflie 2.0/2.1.
-   Right click on Crazyflie 2.0/2.1, go in \"Properties\" then in the
    tab \"Details\".
-   In the drop-box select \"Device instance path\".

The serial number is located just after \"5740\\\", in the following
screen it starts by 41:

![windows serial](/images/windows_serial.png){:width=500x}

By right-clicking on the string you can copy it.

Linux
-----

Open a terminal and type (ie. cop-paste) the following command:

    sudo lsusb -v -d 0483:5740 | grep iSerial

The Crazyflie serial is then displayed (here it starts by 41):

![linux serial](/images/linux_serial.png)

Mac OS X
--------

From the \"About this mac\" menu, click on \"System reports\...\"

![mac serial](/images/mac_serial_about.png){:width=400px}

Then click on \"USB\", locate the Crazyflie 2 and the serial number can
be copied from there

![mac serial 2](/images/mac_serial.png){:width=500px}
