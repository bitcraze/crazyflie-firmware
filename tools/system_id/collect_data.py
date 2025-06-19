"""
Collect data class
"""

import argparse
import logging
import time
from threading import Thread
import numpy as np
import yaml
import os.path
from abc import ABC, abstractmethod

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.localization import Localization


# Only output errors from the logging framework
# logging.basicConfig(level=logging.ERROR)

PWM_MIN = 15000  # 20000 onboard, going lower for more datapoints
PWM_MAX = 65535  # 65535
# Voltage at which we abort experiments: 2.8 (O250 or P250) or 2.6 (T350)
VBAT_MIN = 2.6


class CollectData(ABC):
    """
    Simple logging example class that logs the Stabilizer from a supplied
    link uri and disconnects after 5s.
    """

    def __init__(
        self, link_uri, calib_a, calib_b, comb, mode, batComp=False, verbose=False
    ):
        """Initialize and run the example with the specified link_uri"""
        self.measurements = []
        self.desiredThrust = 0
        self.batComp = batComp
        self.comb = comb
        self.mode = mode
        self.verbose = verbose
        self.calib_a = calib_a
        self.calib_b = calib_b
        self.g = 9.807  # Munich, Germany
        self.start_time = 0

        self._cf = Crazyflie(rw_cache="./cache")

        # Connect some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.fully_connected.add_callback(self._fully_connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        self.is_connected = False
        self.all_params_ok = False

        # Open the csv file
        i = 0
        filename = f"data_{self.mode}_{self.comb}_0{i}.csv"
        while os.path.isfile(filename):  # check if file already exists
            i += 1
            filename = f"data_{self.mode}_{self.comb}_0{i}.csv"
            if i > 9:
                filename = f"data_{self.mode}_{self.comb}_{i}.csv"
            if i > 99:
                break
        print(f"Storing data in {filename}")
        self._file = open(filename, "w+")
        self._file.write(
            "time,thrust[N],pwm,vbat[V],rpm1,rpm2,rpm3,rpm4,v[V],i[A],p[W],thrust_cmd[PWM]\n"
        )

        print("Connecting to %s" % link_uri)

        # Try to connect to the Crazyflie
        self._cf.open_link(link_uri)

    def _connected(self, link_uri):
        """This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print("Connected to %s" % link_uri)

    def _fully_connected(self, link_uri):
        """This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        # time.sleep(0.1)
        if self.verbose:
            print(f"Calibration Parameters: a={self.calib_a}, b={self.calib_b}")
        self._cf.param.set_value("loadcell.a", str(self.calib_a))
        self._cf.param.set_value("loadcell.b", str(self.calib_b))

        # The definition of the logconfig can be made before connecting
        self._lg_stab = LogConfig(name="data", period_in_ms=10)
        self._lg_stab.add_variable("loadcell.weight", "float")
        self._lg_stab.add_variable("motor.m1", "uint16_t")
        self._lg_stab.add_variable("pm.vbatMV", "float")
        self._lg_stab.add_variable("rpm.m1", "uint16_t")
        self._lg_stab.add_variable("rpm.m2", "uint16_t")
        self._lg_stab.add_variable("rpm.m3", "uint16_t")
        self._lg_stab.add_variable("rpm.m4", "uint16_t")
        self._lg_stab.add_variable("asc37800.v_mV", "int16_t")
        self._lg_stab.add_variable("asc37800.i_mA", "int16_t")
        self._lg_stab.add_variable("asc37800.p_mW", "int16_t")

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
            print(
                "Could not start log configuration,{} not found in TOC".format(str(e))
            )
        except AttributeError:
            print("Could not add Stabilizer log config, bad configuration.")

        self._localization = Localization(self._cf)

        # Variable used to keep main loop occupied until disconnect
        self.is_connected = True

        Thread(target=self._ramp_motors).start()

    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print("Error when logging %s: %s" % (logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        """Callback froma the log API when data arrives"""
        if self.start_time == 0:
            self.start_time = timestamp
        if self.verbose:
            print("[%d][%s]: %s" % (timestamp - self.start_time, logconf.name, data))
        self._write(timestamp, data)

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the speficied address)"""
        print("Connection to %s failed: %s" % (link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print("Connection to %s lost: %s" % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print("Disconnected from %s" % link_uri)
        time.sleep(2.0)
        self._file.close()
        self.is_connected = False

    def _write(self, timestamp, data):
        """Write data to file"""
        # Make sure maxThrust and maxThrustVbat are in the dict
        # data['loadcell.weight_max'] = data.get('loadcell.weight_max', 0)
        # data['pm.vbatMV_max'] = data.get('pm.vbatMV_max', 0)
        data["cmd"] = data.get("cmd", self.desiredThrust)
        self._file.write(
            "{},{},{},{},{},{},{},{},{},{},{},{}\n".format(
                timestamp,
                data["loadcell.weight"] / 1000 * self.g,
                data["motor.m1"],
                data["pm.vbatMV"] / 1000,
                data["rpm.m1"],
                data["rpm.m2"],
                data["rpm.m3"],
                data["rpm.m4"],
                data["asc37800.v_mV"] / 1000,
                data["asc37800.i_mA"] / 1000,
                data["asc37800.p_mW"] / 1000,
                data["cmd"],
            )
        )
        self._file.flush()

    def _check_parameters(self):
        """Checks the set parameters of the loadcell"""
        while not self.all_params_ok:
            try:
                np.testing.assert_approx_equal(
                    float(self._cf.param.get_value("loadcell.a")), self.calib_a
                )
                np.testing.assert_approx_equal(
                    float(self._cf.param.get_value("loadcell.b")), self.calib_b
                )
                self.all_params_ok = True
            except Exception:
                print("Loadcell parameter a could not be set correctly... Retrying")
                self._cf.param.set_value("loadcell.a", str(self.calib_a))
                self._cf.param.set_value("loadcell.b", str(self.calib_b))
                time.sleep(2)
        if self.verbose:
            print("All parameters set correctly")

    def _close(self):
        """Closes the connection"""
        print("Disconnecting and saving...")
        self._cf.param.set_value("motorPowerSet.m1", 0)
        self._cf.commander.send_setpoint(0, 0, 0, 0)
        # Make sure that the last packet leaves before the link is closed
        # since the message queue is not flushed before closing
        time.sleep(0.5)
        self._cf.close_link()

    @abstractmethod
    def _ramp_motors(self):
        """Fuction that is called when the Crazyflie is connected."""
        pass


class CollectDataRamp(CollectData):
    """
    Simple logging example class that logs the Stabilizer from a supplied
    link uri.
    """

    def __init__(self, link_uri, calib_a, calib_b, comb):
        """Initialize and run the example with the specified link_uri"""
        super().__init__(link_uri, calib_a, calib_b, comb, "ramp_motors")

    def _ramp_motors(self):
        self._check_parameters()

        pwm_mult = 1
        pwm_step = 500
        time_step = 0.25
        pwm = 0
        max_pwm = PWM_MAX  # max = 65535

        self._cf.param.set_value("motorPowerSet.m1", 0)
        if self.batComp:
            self._cf.param.set_value("motorPowerSet.enable", 2)
        else:
            self._cf.param.set_value("motorPowerSet.enable", 2)

        print("Starting ramping motors")

        while self.is_connected and pwm >= 0:
            pwm += pwm_step * pwm_mult
            if pwm > max_pwm:
                pwm_mult *= -1
                pwm += pwm_step * pwm_mult
            if pwm < 0:
                break
            if self.verbose:
                print(f"Commanded PWM = {pwm}")
            self._localization.send_emergency_stop_watchdog()
            self._cf.param.set_value("motorPowerSet.m1", str(pwm))
            time.sleep(time_step)

        self._close()


class CollectDataStatic(CollectData):
    """
    Simple logging example class that logs the Stabilizer from a supplied
    link uri and disconnects after 5s.
    """

    def __init__(self, link_uri, calib_a, calib_b, comb, extra="", batComp=False):
        """Initialize and run the example with the specified link_uri"""
        self.measurements = []
        self.desiredThrust = 0
        if extra != "":  # Add an underscore if extra is not empty
            extra = f"_{extra}"
        if batComp:  # verification mode
            super().__init__(
                link_uri, calib_a, calib_b, comb, f"static_verification{extra}", batComp
            )
        else:
            super().__init__(
                link_uri, calib_a, calib_b, comb, f"static{extra}", batComp
            )

    def _stab_log_data(self, timestamp, data, logconf):
        """Callback froma the log API when data arrives"""
        if self.verbose:
            print("[%d][%s]: %s" % (timestamp, logconf.name, data))
        if self.batComp:  # In verification mode, directly store data
            self.measurements.append(data)
        elif (
            self.desiredThrust == data["motor.m1"]
        ):  # In collection mode, wait for command to arrive
            # self.measurements.append(np.array([data['loadcell.weight']/1000*self.g, data['pm.vbatMV']/1000]))
            self.measurements.append(data)

    def _average_dict(self, dictionary_list):
        """Converts a list of dictionaries into a single dictionary with the averages."""
        sums = {}
        for d in dictionary_list:
            for key, value in d.items():
                sums[key] = sums.get(key, 0) + value
        averages = {key: value / len(dictionary_list) for key, value in sums.items()}
        return averages

    def _measure(
        self, thrust, min_samples=100
    ):  # time per measurement = min_samples * log prediod_in_ms
        self.desiredThrust = thrust
        self.measurements = []
        self._cf.param.set_value("motorPowerSet.m1", str(thrust))
        while len(self.measurements) < min_samples:
            self._localization.send_emergency_stop_watchdog()
            time.sleep(0.1)
        # m = np.array(self.measurements)
        # only return the last few samples
        return self.measurements[-int(0.2 * min_samples) :]

    def _ramp_motors(self):
        self._check_parameters()

        self._cf.param.set_value("motorPowerSet.m1", 0)
        if self.batComp:
            self._cf.param.set_value("motorPowerSet.enable", 3)
            print("Collecting data with battery compensation")
        else:
            self._cf.param.set_value("motorPowerSet.enable", 2)
            print("Collecting data without battery compensation")

        t_max = 600  # Set to np.inf to run until battery is empty
        t_start = time.time()

        # Here we care about the thrust generated for different battery levels
        # We store a random PWM with its corresponding thrust
        # and thrust @ max PWM + the battery voltage for both cases
        while self.is_connected:  # thrust >= 0:
            # randomply sample PWM
            pwm = int(np.random.uniform(PWM_MIN, PWM_MAX))

            data = self._measure(pwm)
            # average thrust and vbat
            data = self._average_dict(data)

            print(
                f"time={time.time() - t_start:.1f}s, pwm={pwm}, vbat={data['pm.vbatMV'] / 1000:.3f}V, vmotors={data['pm.vbatMV'] / 1000 * pwm / PWM_MAX:.3f}V, thrust={data['loadcell.weight'] * self.g:.3f}mN"
            )

            # write result
            self._write(time.time() - t_start, data)

            # if data['pm.vbatMV_max']/1000 < VBAT_MIN:
            if data["pm.vbatMV"] / 1000 < VBAT_MIN:
                print("Warning: Battery low, stopping...")
                break

            if time.time() - t_start > t_max:
                break

        self._close()


class CollectDataDynamic(CollectData):
    """
    Simple logging example class that logs the Stabilizer from a supplied
    link uri and disconnects after 5s.
    """

    def __init__(self, link_uri, calib_a, calib_b, comb, extra=""):
        """Initialize and run the example with the specified link_uri"""
        self.samplerate = 7  # has to be in [0,7], where 7 is the highest.
        # 0 = 10Hz (default), 1 = 20Hz, 2 = 40Hz, 3 = 80Hz, 7 = 320Hz
        # See datasheet of NAU7802
        # However, the data of the loadcell only gets sent with ~2Hz anyway
        if extra != "":  # Add an underscore if extra is not empty
            extra = f"_{extra}"
        super().__init__(link_uri, calib_a, calib_b, comb, f"dynamic{extra}")

    def _fully_connected(self, link_uri):
        """This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        self._cf.param.set_value("loadcell.sampleRate", self.samplerate)

        super()._fully_connected(link_uri)

    def _check_parameters(self):
        """Checks the set parameters of the loadcell"""
        while not self.all_params_ok:
            try:
                np.testing.assert_approx_equal(
                    int(self._cf.param.get_value("loadcell.sampleRate")),
                    self.samplerate,
                )
                super()._check_parameters()
                self.all_params_ok = True
            except Exception:
                print("Loadcell parameter a could not be set correctly... Retrying")
                self._cf.param.set_value("loadcell.sampleRate", str(self.samplerate))
                time.sleep(2)

    def _applyThrust(self, thrust, duration):
        self.desiredThrust = thrust
        start = time.time()
        self._cf.param.set_value("motorPowerSet.m1", int(thrust))
        print(f"Applied pwm={thrust}, waiting for {duration}s")
        while time.time() - start < duration:
            self._localization.send_emergency_stop_watchdog()
            # print(time.time() - start)
            time.sleep(0.1)

    def _ramp_motors(self):
        self._check_parameters()

        self._cf.param.set_value("motorPowerSet.m1", 0)

        duration = 2.0

        # collecting data twice:
        # Once with and once without batter compensation
        while self.is_connected:
            if self.batComp:
                self._cf.param.set_value("motorPowerSet.enable", 3)
                print("Collecting data with battery compensation")
            else:
                self._cf.param.set_value("motorPowerSet.enable", 2)
                print("Collecting data without battery compensation")

            # base speed
            self._applyThrust(PWM_MIN, duration)

            # 0 -> 1
            self._applyThrust(PWM_MAX, duration)
            # 1 -> 0
            self._applyThrust(PWM_MIN, duration)

            # 0 -> 0.5
            self._applyThrust((PWM_MAX + PWM_MIN) / 2, duration)
            # 0.5 -> 0
            self._applyThrust(PWM_MIN, duration)

            # 0.5 -> 1.0
            self._applyThrust((PWM_MAX + PWM_MIN) / 2, duration)
            self._applyThrust(PWM_MAX, duration)
            # 1.0 -> 0.5
            self._applyThrust((PWM_MAX + PWM_MIN) / 2, duration)

            self._applyThrust(0, duration)

            if self.batComp:
                break
            else:
                self.batComp = True

        self._close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--calibration",
        default="calibration.yaml",
        help="Input file containing the calibration",
    )
    parser.add_argument(
        "--uri", default="radio://0/42/2M/E7E7E7E7E7", help="URI of Crazyflie"
    )
    parser.add_argument(
        "--mode",
        default="ramp_motors",
        help="Type of data collected, for more information see readme",
    )
    parser.add_argument(
        "--comb",
        default="",
        help="Combination of propellers, motors, and battery, for more information see readme",
    )
    parser.add_argument(
        "--extra", default="", help="Additional information for labeling the csv file"
    )

    args = parser.parse_args()

    # Only output errors from the logging framework
    logging.basicConfig(level=logging.ERROR)

    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    with open(args.calibration, "r") as f:
        r = yaml.safe_load(f)
        a = r["a"]
        b = r["b"]

    # collect data
    if args.mode == "ramp_motors":
        le = CollectDataRamp(args.uri, a, b, args.comb)
    elif args.mode == "static":
        le = CollectDataStatic(args.uri, a, b, args.comb, extra=args.extra)
    elif args.mode == "static_verification":  # activate battery compensation to test it
        le = CollectDataStatic(
            args.uri, a, b, args.comb, extra=args.extra, batComp=True
        )
    elif args.mode == "dynamic":
        le = CollectDataDynamic(args.uri, a, b, args.comb, extra=args.extra)
    else:
        raise NotImplementedError(
            f"Data Collection Type {args.mode} is not implemented."
        )
    time.sleep(2)

    # This will block until the the Crazyflie is disconnected
    # and catch a KeyboardInterrupt to securely stop the script
    try:
        while le.is_connected:
            time.sleep(2)
    except KeyboardInterrupt:
        le.is_connected = False
        time.sleep(2)
