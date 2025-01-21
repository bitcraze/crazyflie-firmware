"""
Ramp motors and collect data using the system-id deck
"""
import argparse
import logging
import time
import yaml
from threading import Thread

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.localization import Localization

from collect_data import CollectData


class CollectDataRamp(CollectData):
    """
    Simple logging example class that logs the Stabilizer from a supplied
    link uri.
    """

    def __init__(self, link_uri, calib_a, calib_b, comb, type):
        """ Initialize and run the example with the specified link_uri """
        super().__init__(link_uri, calib_a, calib_b, comb, type)

        # super()._cf.fully_connected.remove_callback(super()._fully_connected)
        # super()._cf.fully_connected.add_callback(self._fully_connected)

        # Wait for connection
        # while not self.is_connected:
        #     time.sleep(0.1)
        # print('Connecting to %s' % link_uri)

        # # Try to connect to the Crazyflie
        # self._cf.open_link(link_uri)

    def _ramp_motors(self):
        pwm_mult = 1
        pwm_step = 500
        time_step = 0.25
        pwm = 0
        max_pwm = 65535 # max = 65535
        pitch = 0
        roll = 0
        yawrate = 0

        # Unlock startup thrust protection
        for i in range(0, 100):
            self._cf.commander.send_setpoint(0, 0, 0, 0)

        localization = Localization(self._cf)

        # self._cf.param.set_value('motor.batCompensation', 0)
        self._cf.param.set_value('motorPowerSet.m1', 0)
        self._cf.param.set_value('motorPowerSet.enable', 2)
        # self._cf.param.set_value('system.forceArm', 1)

        print("Starting ramping motors")

        while self.is_connected and pwm >= 0:
            pwm += pwm_step * pwm_mult
            if pwm > max_pwm:
            # if thrust >= 20000 or thrust < 0:
                pwm_mult *= -1
                pwm += pwm_step * pwm_mult
            if pwm < 0:
                break
            print(f"commanded PWM = {pwm}")
            # self._cf.commander.send_setpoint(roll, pitch, yawrate, thrust)
            localization.send_emergency_stop_watchdog()
            self._cf.param.set_value('motorPowerSet.m1', str(pwm))
            time.sleep(time_step)

        self._cf.commander.send_setpoint(0, 0, 0, 0)
        self._cf.param.set_value('motorPowerSet.enable', 0)
        # Make sure that the last packet leaves before the link is closed
        # since the message queue is not flushed before closing
        time.sleep(1)
        self._cf.close_link()
        self._file.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--calibration", default="calibration.yaml", help="Input file containing the calibration")
    parser.add_argument("--uri", default="radio://0/42/2M/E7E7E7E7E7", help="URI of Crazyflie")
    parser.add_argument("--type", default="ramp_motors", help="Type of data collected, for more information see readme")
    parser.add_argument("--comb", default="-B250", help="Combination of propellers, motors, and battery")
    # First char is the version of the propellers (- for the regular or + for the 2.X+ propellers)
    # Second char is the motors (B for base 17mm or T for thrust upgrade 20mm motors) # TODO Brushless?
    # Rest is the capacity of the battery (250mAh or 350mAh)
    args = parser.parse_args()

    # Only output errors from the logging framework
    logging.basicConfig(level=logging.ERROR)

    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    with open(args.calibration, 'r') as f:
        r = yaml.safe_load(f)
        a = r['a']
        b = r['b']

    # collect data
    le = CollectDataRamp(args.uri, a, b, args.comb)
    time.sleep(1)

    # The Crazyflie lib doesn't contain anything to keep the application alive,
    # so this is where your application should do something. In our case we
    # are just waiting until we are disconnected.
    while le.is_connected:
        time.sleep(1)
