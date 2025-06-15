"""
Calibrate load cell
"""
import argparse
import logging
import time
import yaml

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig

import numpy as np
import matplotlib.pyplot as plt
import os


class CalibScale:
    def __init__(self, link_uri, calib_a, calib_b):
        """ Initialize and run with the specified link_uri """

        self.measurements = []
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
        self.is_connected = False

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print('Connected to %s' % link_uri)

        # The definition of the logconfig can be made before connecting
        self._lg_stab = LogConfig(name='data', period_in_ms=10)
        self._lg_stab.add_variable('loadcell.weight', 'float')
        self._lg_stab.add_variable('loadcell.rawWeight', 'int32_t')

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

        self.is_connected = True

    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        """Callback froma the log API when data arrives"""
        # print('[%d][%s]: %s' % (timestamp, logconf.name, data))
        self.measurements.append(data['loadcell.rawWeight'])

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

    def measure(self, num_measurements = 100):
        weights = []
        readings = []

        while True:
            data = float(input("enter weight in grams (-1 to end calibration): "))
            if data < 0:
                break
            self.measurements = []
            while len(self.measurements) < num_measurements:
                time.sleep(0.1)
            print(np.mean(self.measurements))
            weights.append(data)
            readings.append(np.mean(self.measurements))

        self._cf.close_link()

        # If there is a previous value, we can simply change the intercept (b)
        # This is possible because a is constant. Allows for quicker calibration in case the battery is changed
        # One should still fully recalibrate after moving the loadcell or mounting a different drone
        if len(readings) == 1 and self.calib_a is not None and self.calib_b is not None:
            ...
            p_old = np.poly1d([self.calib_a, self.calib_b])
            xp = readings[0]
            yp = p_old(xp)
            delta = yp - weights[0]

            self.calib_b -= delta

            xp = np.linspace(readings[0], readings[0]-1e6, 100)
            plt.plot(readings, weights, '.', label="data")
            plt.plot(xp, p_old(xp), ':', label="previous calibration")
            p_new = np.poly1d([self.calib_a, self.calib_b])
            plt.plot(xp, p_new(xp), ':', label="calibration")
            plt.xlabel("Loadcell reading")
            plt.ylabel("Weight in [g]")
            plt.legend()
            plt.show()

            return float(self.calib_a), float(self.calib_b)
        else:
            z = np.polyfit(readings, weights, 1)
            p = np.poly1d(z)

            xp = np.linspace(readings[0], readings[-1], 100)

            plt.plot(readings, weights, '.', label="data")
            plt.plot(xp, p(xp), '--', label="calibration")
            # old values for comparison
            if self.calib_a is not None and self.calib_b is not None:
                p_old = np.poly1d([self.calib_a, self.calib_b])
                plt.plot(xp, p_old(xp), ':', label="previous calibration")
            plt.xlabel("Loadcell reading")
            plt.ylabel("Weight in [g]")
            plt.legend()
            plt.show()
            return float(z[0]), float(z[1])


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default="calibration.yaml", help="Output file containing the calibration")
    parser.add_argument("--uri", default="radio://0/42/2M/E7E7E7E7E7", help="URI of Crazyflie")
    args = parser.parse_args()

    # Only output errors from the logging framework
    logging.basicConfig(level=logging.ERROR)
    
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    a, b = None, None
    if os.path.isfile(args.output):
        with open(args.output, 'r') as f:
            r = yaml.safe_load(f)
            a = r['a']
            b = r['b']
    
    le = CalibScale(args.uri, a, b)

    while not le.is_connected:
        time.sleep(0.1)

    a,b = le.measure()
    result = dict()
    result['a'] = a
    result['b'] = b

    with open(args.output, 'w') as yaml_file:
        yaml.dump(result, yaml_file)
        print(f"Calibration saved in {args.output}")


