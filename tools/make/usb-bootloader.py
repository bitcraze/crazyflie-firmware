#send a USB request to the crazyflie, causing it to enter DFU mode (bootloader) automatically
#flash new firmware using 'make flash_dfu'

from time import sleep
import usb
import usb.core

#find connected crazyflie, usb vendor and product id = 0483:5740
dev = usb.core.find(idVendor=0x0483, idProduct=0x5740)
if dev is None:
    raise ValueError('Could not find any USB connected crazyflie')

#send signal to enter bootloader
try:
    dev.ctrl_transfer(bmRequestType=usb.TYPE_VENDOR, 
        bRequest=0x01, wValue=0x01, wIndex=2)
except IOError:
    #io error expected because the crazyflie will not respond to USB request as it resets into the bootloader
    #TODO usbd_cf_Setup function in firmware needs to return USBD_OK before rebooting to fix this
        #sleep to allow time for crazyflie to get into DFU mode
        sleep(0.5)