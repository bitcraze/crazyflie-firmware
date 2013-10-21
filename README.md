#Hover Mode
This extends the functionality of the original crazy flie firmware by adding a hover mode.

**Warning:** the code is experimental and far from a final state. It is not really being 
maintained either, but I hope that someone might find it useful.

You will also need the modified [pc client](https://bitbucket.org/bitcraze/crazyflie-pc-client "pc-client OMWDUNKLEY").


See the [original forum post](http://forum.bitcraze.se/viewtopic.php?f=6&t=523&p=3351#p3351 "Original Post"):


Hey guys,

some of you might have seen my post [here](http://forum.bitcraze.se/viewtopic.php?f=6&t=331&start=10#p2405 "First mention of hover mode implementation").

A while back I was experimenting with using the barometer to implement a hover mode. 
It worked relatively well in stable pressure conditions. I never released the code 
because I wanted to clean it up and do it properly (EKF, etc). However Ive not got 
around to doing that yet so I decided to expose the code. Yes its messy, yes lots 
is redundant, yes lots is suboptimal, but yes it works too:). I not going to 
continue to develop this fork (maybe some minor bug fixes and gui updates), but 
someone might find it useful.

##How to use:

* Set up your hover button from the GUI. By default for PS3_Mode_1 its the L1 button of the PS3 controller.
* When you press this button, it sets the target altitude as the current altitude. 
* Hold the button to remain in hover mode.
* While in hover mode, the throttle can be used to change the target altitude. Eg holding throttle up for a second raises it about a meter.
* Release the button to return to manual mode.
* Next time you enter hover mode, the target altitude is reset to the current altitude again.
* **A good tip:** Let go of the throttle immediately after entering hover mode. Its very easy to forget that one is holding it up and the flie will continue to rise.

##Some details:
The ms5611 driver has been partly rewritten to enable pressure measurements at 50-100hz.

I wrote the code a while ago, but if I remember correctly it sort of works as follows:

All pressure readings are converted to an altitude above sea level.
When entering hover mode, we set the target altitude. 

We can then define a PID controller that should take the flie from its current 
altitude to the target altitude. The P part is just the difference, eg 1 meter too high.
For the D term we use the vertical speed...here the code is ugly. First one needs to 
compute the vertical acceleration, then subtract gravity. This vertical acceleration 
is then integrated to get a speed estimate. To stop it accumulating error forever, 
it converges to the speed estimate from the barometer. This is also computed in a 
non mathematical way: some factor * (super_smoothed_altitude-smoothed_altitude).
The I term is just the integrated error - and is very very very important as it 
makes up for the voltage drop. The P and D term are reset every time hover mode is 
entered, and the I term is only reset when you start charging the flie. The default 
I value right now is set up to be a pretty good value for a stock flie at 80% battery. 
The default values takes around 1-2 to converge on a flie with a depleted battery 
during which time the flie might oscillate within a meter range or so.

Note that hover mode only works well in pressure stable environments. Trying to hover 
with people opening/closing windows/doors or during a thunderstorm does not work very well!

##Videos of it working:

[StaticExample](http://www.youtube.com/watch?v=aRsvPyRQaFA "Youtube video")

[ManeuoveringExample](http://www.youtube.com/watch?v=0oYzMVUKZKI "Youtube video")



##CrazyFlie Firmware
```
Folder description:
./              | Root, contains the Makefile
 + init         | Contains the main.c
 + config       | Configuration files
 + drivers      | Hardware driver layer
 |  + src       | Drivers source code
 |  + interface | Drivers header files. Interface to the HAL layer
 + hal          | Hardware abstaction layer
 |  + src       | HAL source code
 |  + interface | HAL header files. Interface with the other parts of the program
 + modules      | Firmware operating code and headers
 |  + src       | Firmware tasks source code and main.c
 |  + interface | Operating headers. Configure the firmware environement
 + utils        | Utils code. Implement utility block like the console.
 |  + src       | Utils source code
 |  + interface | Utils header files. Interface with the other parts of the program
 + scripts      | Misc. scripts for LD, OpenOCD, make, version control, ...
 |              | *** The two following folders contains the unmodified files ***
 + lib          | Libraries
 |  + FreeRTOS  | Source FreeRTOS folder. Cleaned up from the useless files
 |  + STM32F... | Library folder of the St STM32 peripheral lib
 |  + CMSIS     | Core abstraction layer

Make targets:

all        : Shortcut for build
compile    : Compile cflie.hex. WARNING: Do NOT update version.c
build      : Update version.c and compile cflie.elf/hex
clean_o    : Clean only the Objects files, keep the executables (ie .elf, .hex)
clean      : Clean every compiled files
mrproper   : Clean every compiled files and the classical editors backup files

flash      : Flash cflie.elf using OpenOCD
halt       : Halt the target using OpenOCD
reset      : Reset the target using OpenOCD
openocd    : Launch OpenOCD
```
