"""
Example that uses a load cell to measure thrust using different RPMs
"""
import argparse
import logging
import time
from threading import Thread
import numpy as np
import yaml

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.localization import Localization


# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


class CollectData:
    """
    Simple logging example class that logs the Stabilizer from a supplied
    link uri and disconnects after 5s.
    """

    def __init__(self, link_uri, calib_a, calib_b):
        """ Initialize and run the example with the specified link_uri """
        self.measurements = []
        self.desiredThrust = 0
        self.calib_a = calib_a
        self.calib_b = calib_b

        self._cf = Crazyflie(rw_cache='./cache')

        # Connect some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        print('Connecting to %s' % link_uri)

        # Try to connect to the Crazyflie
        self._cf.open_link(link_uri)

        # Variable used to keep main loop occupied until disconnect
        self.is_connected = True

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print('Connected to %s' % link_uri)

        print(self.calib_a, self.calib_b)
        self._cf.param.set_value('loadcell.a', str(self.calib_a))
        self._cf.param.set_value('loadcell.b', str(self.calib_b))

        self._file = open("data.csv", "w+")
        self._file.write("thrust[g],pwm,vbat[V],maxThrust[g],maxThrustVbat[V],rpm\n");

        # The definition of the logconfig can be made before connecting
        self._lg_stab = LogConfig(name='data', period_in_ms=10)
        self._lg_stab.add_variable('loadcell.weight', 'float')
        self._lg_stab.add_variable('motor.m1', 'uint16_t')
        self._lg_stab.add_variable('pm.vbat', 'float')
        self._lg_stab.add_variable('rpm.m1', 'uint16_t')
        self._lg_stab.add_variable('rpm.m2', 'uint16_t')
        self._lg_stab.add_variable('rpm.m3', 'uint16_t')
        self._lg_stab.add_variable('rpm.m4', 'uint16_t')

        # Adding the configuration cannot be done until a Crazyflie is
        # connected, since we need to check that the variables we
        # would like to log are in the TOC.
        try:
            self._cf.log.add_config(self._lg_stab)
            # This callback will receive the data
            self._lg_stab.data_received_cb.add_callback(self._stab_log_data)
            # This callback will be called on errors
            self._lg_stab.error_cb.add_callback(self._stab_log_error)
            # Start the logging
            self._lg_stab.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Stabilizer log config, bad configuration.')

        self._localization = Localization(self._cf)

        # Start a separate thread to do the motor test.
        # Do not hijack the calling thread!
        Thread(target=self._ramp_motors).start()

        # # Start a timer to disconnect in 10s
        # t = Timer(5, self._cf.close_link)
        # t.start()

    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        """Callback froma the log API when data arrives"""
        # print('[%d][%s]: %s' % (timestamp, logconf.name, data))
        # self._file.write("{},{},{}\n".format(data['loadcell.weight'], data['motor.m1'], data['pm.vbat']))
        if self.desiredThrust == data['motor.m1']:
            self.measurements.append(np.array([data['loadcell.weight'], data['pm.vbat']]))
        # pass

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the speficied address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)
        self.is_connected = False

    def _measure(self, thrust, min_samples = 50):
        self.desiredThrust = thrust
        self.measurements = []
        self._cf.param.set_value('motorPowerSet.m1', str(thrust))
        while len(self.measurements) < min_samples:
            self._localization.send_emergency_stop_watchdog()
            time.sleep(0.1)
        m = np.array(self.measurements)
        # only return the last few samples
        return m[40:]

    def _ramp_motors(self):

        self._cf.param.set_value('motorPowerSet.m1', 0)
        self._cf.param.set_value('motorPowerSet.enable', 2)

        while self.is_connected: #thrust >= 0:
            # randomply sample PWM
            thrust = int(np.random.uniform(15000, 65535))
            measurements = self._measure(thrust)
            # print(measurements)
            achievedThrust = np.mean(measurements[:,0])
            vbat = np.mean(measurements[:,1])

            # go to full thrust
            measurements = self._measure(65535)
            # print(measurements)
            maxThrust = np.mean(measurements[:,0])
            vbatAtMaxThrust = np.mean(measurements[:,1])

            # write result
            print(achievedThrust, thrust, vbat, maxThrust, vbatAtMaxThrust)
            self._file.write("{},{},{},{},{}\n".format(
                achievedThrust,
                thrust,
                vbat,
                maxThrust,
                vbatAtMaxThrust))

            if vbatAtMaxThrust < 2.8:
                break

        # self._cf.commander.send_setpoint(0, 0, 0, 0)
        # Make sure that the last packet leaves before the link is closed
        # since the message queue is not flushed before closing
        time.sleep(0.1)
        self._cf.close_link()
        self._file.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--calibration", default="calibration.yaml", help="Input file containing the calibration")
    parser.add_argument("--uri", default="radio://0/42/2M/E7E7E7E7E7", help="URI of Crazyflie")
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
    le = CollectData(args.uri, a, b)

    # The Crazyflie lib doesn't contain anything to keep the application alive,
    # so this is where your application should do something. In our case we
    # are just waiting until we are disconnected.
    while le.is_connected:
        time.sleep(1)
