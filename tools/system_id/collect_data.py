"""
Collect data class
"""

import argparse
import logging
import time
from threading import Thread
import numpy as np
import toml
import os.path
from abc import ABC, abstractmethod

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.localization import Localization

# Only output errors from the logging framework
# logging.basicConfig(level=logging.ERROR)

PWM_MIN = 7000  # as in firmware
PWM_MAX = 65535  # as in firmware
# Voltage at which we abort the experiments. Set to the same value as the firmware.
VBAT_MIN = 3.2
# During the experiments, the voltage can drop so much, such that the uC cant get
# enough power anymore. If we reach that voltge, we abort early
VBAT_MIN_CRIT = 2.7


class CollectData(ABC):
    """
    Simple logging example class that logs the Stabilizer from a supplied
    link uri and disconnects after 5s.
    """

    def __init__(
        self,
        link_uri,
        calib_a,
        calib_b,
        mode: str,
        comb: str,
        extra: str,
        batComp=False,
        verbose=False,
        loadcell=None,
    ):
        """Initialize and run the example with the specified link_uri"""
        self.measurements = []
        self.desiredThrust = 0
        self.batComp = batComp
        self.mode = mode
        self.comb = comb
        self.extra = extra
        self.verbose = verbose
        self.calib_a = calib_a
        self.calib_b = calib_b
        self.g = 9.807  # Munich, Germany
        self.start_time = -1
        self.loadcell = loadcell

        self._cf = Crazyflie(rw_cache="./cache")

        # Connect some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.fully_connected.add_callback(self._fully_connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        self.is_connected = False
        self.all_params_ok = False

        self._data_last = {}

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
        self._lg_stab.add_variable("motor.m2", "uint16_t")
        self._lg_stab.add_variable("motor.m3", "uint16_t")
        self._lg_stab.add_variable("motor.m4", "uint16_t")
        self._lg_stab.add_variable("pm.vbatMV", "uint16_t")
        self._lg_stab.add_variable("rpm.m1", "uint16_t")
        self._lg_stab.add_variable("rpm.m2", "uint16_t")
        self._lg_stab.add_variable("rpm.m3", "uint16_t")
        self._lg_stab.add_variable("rpm.m4", "uint16_t")
        self._lg_stab.add_variable("asc37800.v_mV", "uint16_t")
        self._lg_stab.add_variable("asc37800.i_mA", "uint16_t")

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

        # Open the csv file
        comb = self.comb + "_" + self.extra if self.extra != "" else self.comb
        i = 0
        filename = f"data/data_{self.mode}_{comb}_0{i}.csv"
        while os.path.isfile(filename):  # check if file already exists
            i += 1
            filename = f"data/data_{self.mode}_{comb}_{i:02d}.csv"
        print(f"Storing data in {filename}")
        self._file = open(filename, "w+")
        self._file.write(
            "time[s],thrust[N],pwm,vbat[V],rpm1,rpm2,rpm3,rpm4,v[V],i[A],p[W],thrust_cmd[PWM],torque_x[N],torque_y[N],torque_z[N]\n"
        )

        # Variable used to keep main loop occupied until disconnect
        self.is_connected = True

        Thread(target=self._ramp_motors).start()

    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print("Error when logging %s: %s" % (logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        """Callback froma the log API when data arrives"""
        timestamp /= 1000.0  # ms to s
        if self.start_time == -1:
            self.start_time = timestamp
        timestamp -= self.start_time  # remove initial offset
        if self.loadcell is not None:
            try:
                reading = self.loadcell.read_data()
            except Exception as e:
                print(f"Error when reading the loadcell: {e}")
            data["loadcell.weight"] = -reading[2] * 1000 / self.g
            data["loadcell.torque_x"] = -reading[3]
            data["loadcell.torque_y"] = -reading[4]
            data["loadcell.torque_z"] = -reading[5]
        else:
            data["loadcell.torque_x"] = 0.0
            data["loadcell.torque_y"] = 0.0
            data["loadcell.torque_z"] = 0.0
        if self.verbose:
            print("[%d][%s]: %s" % (timestamp, logconf.name, data))
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
        # Determine PWM value: if m1 and m2 are equal, use either; if one is zero, use the sum
        if data["motor.m1"] == 0 or data["motor.m2"] == 0:
            pwm = data["motor.m1"] + data["motor.m2"]
        else:
            pwm = (data["motor.m1"] + data["motor.m2"]) / 2

        self._file.write(
            "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}\n".format(
                timestamp,
                data["loadcell.weight"] / 1000 * self.g,
                pwm,
                data["pm.vbatMV"] / 1000,
                data["rpm.m1"],
                data["rpm.m2"],
                data["rpm.m3"],
                data["rpm.m4"],
                data["asc37800.v_mV"] / 1000,
                data["asc37800.i_mA"] / 1000,
                data["asc37800.v_mV"] / 1000 * data["asc37800.i_mA"] / 1000,
                data["cmd"],
                data["loadcell.torque_x"],
                data["loadcell.torque_y"],
                data["loadcell.torque_z"],
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
        self._cf.param.set_value("motorPowerSet.m2", 0)
        self._cf.param.set_value("motorPowerSet.m3", 0)
        self._cf.param.set_value("motorPowerSet.m4", 0)
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

    def __init__(
        self, link_uri, calib_a, calib_b, comb, extra, loadcell=None, verbose=False
    ):
        """Initialize and run the example with the specified link_uri"""
        super().__init__(
            link_uri,
            calib_a,
            calib_b,
            "ramp_motors",
            comb,
            extra,
            loadcell=loadcell,
            verbose=verbose,
        )

    def _ramp_motors(self):
        self._check_parameters()

        pwm_mult = 1
        pwm_step = 500
        time_step = 0.1
        pwm = 0
        max_pwm = PWM_MAX

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

    def __init__(
        self,
        link_uri,
        calib_a,
        calib_b,
        comb,
        extra,
        batComp=False,
        loadcell=None,
        torques=False,
        verbose=False,
    ):
        """Initialize and run the example with the specified link_uri"""
        self.measurements = []
        self.desiredThrust = 0
        self.torques = False
        if loadcell is not None:
            self.torques = torques
        elif torques:
            raise ValueError("Torques can only be collected with a separate loadcell.")
        self.torques = torques
        mode = "static_verification" if batComp else "static"
        super().__init__(
            link_uri,
            calib_a,
            calib_b,
            mode,
            comb,
            extra,
            batComp,
            loadcell=loadcell,
            verbose=verbose,
        )

    def _stab_log_data(self, timestamp, data, logconf):
        """Callback froma the log API when data arrives"""
        if self.loadcell is not None:
            loadcell_data = self.loadcell.read_data()
            if self.torques:
                # To account for the fact that only 2 motors are spinning and the sysid script assumes 4
                loadcell_data *= 2
            data["loadcell.weight"] = -loadcell_data[2] * 1000 / self.g
            data["loadcell.torque_x"] = loadcell_data[3]
            data["loadcell.torque_y"] = loadcell_data[4]
            data["loadcell.torque_z"] = np.abs(loadcell_data[5])
        else:
            data["loadcell.torque_x"] = 0.0
            data["loadcell.torque_y"] = 0.0
            data["loadcell.torque_z"] = 0.0

        if self.verbose:
            print("[%d][%s]: %s" % (timestamp, logconf.name, data))

        if self.batComp:  # In verification mode, directly store data
            self.measurements.append(data)
        elif (  # In collection mode, wait for command to arrive
            self.desiredThrust == data["motor.m1"]
            and not self.torques
            or data["motor.m1"] == data["motor.m3"] == self.desiredThrust
            and np.allclose(data["rpm.m1"], data["rpm.m3"], rtol=0.1)
            and data["rpm.m2"] == data["rpm.m4"] == 0
            or data["motor.m2"] == data["motor.m4"] == self.desiredThrust
            and np.allclose(data["rpm.m2"], data["rpm.m4"], rtol=0.1)
            and data["rpm.m1"] == data["rpm.m3"] == 0
        ):
            self.measurements.append(data)
        else:
            if self.verbose:
                print("Waiting for motor speed to converge")

    def _average_dict(self, dictionary_list: list[dict]) -> dict:
        """Converts a list of dictionaries into a single dictionary with the averages."""
        sums = {}
        for d in dictionary_list:
            for key, value in d.items():
                sums[key] = sums.get(key, 0) + value
        averages = {key: value / len(dictionary_list) for key, value in sums.items()}
        return averages

    def _measure(self, thrust, min_samples=100, a_motors=True) -> list[dict]:
        # time per measurement = min_samples * log period
        self.desiredThrust = thrust
        self.measurements = []

        while len(self.measurements) == 0:
            if self.torques:
                if a_motors:
                    self._cf.param.set_value("motorPowerSet.m1", thrust)
                    self._cf.param.set_value("motorPowerSet.m2", 0)
                    self._cf.param.set_value("motorPowerSet.m3", thrust)
                    self._cf.param.set_value("motorPowerSet.m4", 0)
                else:
                    self._cf.param.set_value("motorPowerSet.m1", 0)
                    self._cf.param.set_value("motorPowerSet.m2", thrust)
                    self._cf.param.set_value("motorPowerSet.m3", 0)
                    self._cf.param.set_value("motorPowerSet.m4", thrust)
                time.sleep(0.5)
            else:
                self._cf.param.set_value("motorPowerSet.m1", thrust)
                time.sleep(0.1)

        tstart = time.time()
        while len(self.measurements) < min_samples:
            if time.time() - tstart > 10:
                return []
            time.sleep(0.01)

        # only return the last few samples
        return self.measurements[-int(0.6 * min_samples) :]

    def _ramp_motors(self):
        self._check_parameters()

        self._cf.param.set_value("motorPowerSet.m1", 0)
        self._cf.param.set_value("motorPowerSet.m2", 0)
        self._cf.param.set_value("motorPowerSet.m3", 0)
        self._cf.param.set_value("motorPowerSet.m4", 0)
        if self.batComp:
            if self.torques:
                error = (
                    "Trying to collect torque measurements with battery compensation "
                    + "enabled (i.e. verification). This is not implemented yet!"
                )
                raise NotImplementedError(error)
            self._cf.param.set_value("motorPowerSet.enable", 3)
            global PWM_MIN
            PWM_MIN = 0  # in testing mode, we also want to check 0 thrust commands
            print("Collecting data with battery compensation")
        else:
            if self.torques:
                self._cf.param.set_value("motorPowerSet.enable", 1)
                print("Collecting torque data without battery compensation")
            else:
                self._cf.param.set_value("motorPowerSet.enable", 2)
                print("Collecting data without battery compensation")

        t_max = 1200  # Set to np.inf to run until battery is empty
        t_start = time.time()
        i = 0

        # Here we care about the thrust generated for different battery levels and PWM values
        while self.is_connected:
            # Randomply sample PWM
            # Every so often, apply lowest PWM for checking vbat without load
            pwm = PWM_MIN if i % 10 == 0 else int(np.random.uniform(PWM_MIN, PWM_MAX))
            i += 1

            data1 = self._measure(pwm).copy()
            data2 = []
            if self.torques:
                data2 = self._measure(pwm, a_motors=False).copy()
            if len(data1) == 0 or len(data2) == 0 and self.torques:
                print("WARNING: Data collection timed out. Collecting next data point")
            else:
                # average thrust and vbat
                data = self._average_dict(data1 + data2)
                if self.torques:
                    # Doubling the pwm values, since they have been 0 for half of the time
                    # TODO this is imprecise, since arrays could be of different length!
                    data["motor.m1"] *= 2
                    data["motor.m2"] *= 2
                    data["rpm.m1"] *= 2
                    data["rpm.m2"] *= 2
                    data["rpm.m3"] *= 2
                    data["rpm.m4"] *= 2

                info = f"time={time.time() - t_start:.1f}s, "
                info += f"pwm cmd={pwm}, "
                info += f"pwm actual={(data['motor.m1']) if not self.torques else (data['motor.m1'] + data['motor.m2']) / 2:.1f}, "
                info += f"rotor speed={(data['rpm.m1']) if not self.torques else (data['rpm.m1'] + data['rpm.m2']) / 2:.1f}, "
                info += f"vbat={data['pm.vbatMV'] / 1000:.3f}V, "
                info += f"vmotors={data['pm.vbatMV'] / 1000 * pwm / PWM_MAX:.3f}V, "
                info += f"thrust={data['loadcell.weight'] * self.g:.3f}mN"
                print(info)

                if (
                    data["pm.vbatMV"] / 1000 < VBAT_MIN
                    and pwm == PWM_MIN
                    or data["pm.vbatMV"] / 1000 < VBAT_MIN_CRIT
                ):
                    print("Warning: Battery low, stopping...")
                    break
                else:
                    self._write(time.time() - t_start, data)

            # Recalibrate loadcell now and then
            if self.loadcell is not None and i % 20 == 0:
                self._measure(0, min_samples=10)
                self._measure(0, min_samples=10, a_motors=False)
                time.sleep(2.0)
                self.loadcell.calibrate()

            if time.time() - t_start > t_max:
                print("Time limit reached, stopping...")
                break

        self._close()


class CollectDataDynamic(CollectData):
    """
    Simple logging example class that logs the Stabilizer from a supplied
    link uri and disconnects after 5s.
    """

    def __init__(
        self,
        link_uri,
        calib_a,
        calib_b,
        comb,
        extra,
        loadcell=None,
        verbose=False,
    ):
        """Initialize and run the example with the specified link_uri"""
        self.samplerate = 7  # has to be in [0,7], where 7 is the highest.
        # 0 = 10Hz (default), 1 = 20Hz, 2 = 40Hz, 3 = 80Hz, 7 = 320Hz
        # See datasheet of NAU7802
        super().__init__(
            link_uri,
            calib_a,
            calib_b,
            "dynamic",
            comb,
            extra,
            batComp=True,
            loadcell=loadcell,
            verbose=verbose,
        )

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
            # self._localization.send_emergency_stop_watchdog()
            time.sleep(0.001)

    def _ramp_motors(self):
        self._check_parameters()

        self._cf.param.set_value("motorPowerSet.m1", 0)

        if self.batComp:
            self._cf.param.set_value("motorPowerSet.enable", 3)
            print("Collecting data with battery compensation")
        else:
            self._cf.param.set_value("motorPowerSet.enable", 2)
            print("Collecting data without battery compensation")

        self._ramp_motors_chirp()
        self._ramp_motors_step()

        self._close()

    def _ramp_motors_step(self, duration: float = 1.0):
        # base speed
        self._applyThrust(PWM_MIN, 2 * duration)

        # 0 -> 0.25
        self._applyThrust((PWM_MAX + PWM_MIN) * 1 / 4, duration)
        # 0.25 -> 0
        self._applyThrust(PWM_MIN, duration)

        # 0 -> 0.5
        self._applyThrust((PWM_MAX + PWM_MIN) / 2, duration)
        # 0.5 -> 0
        self._applyThrust(PWM_MIN, duration)

        # 0 -> 0.75
        self._applyThrust((PWM_MAX + PWM_MIN) * 3 / 4, duration)
        # 0.75 -> 0
        self._applyThrust(PWM_MIN, duration)

        # 0 -> 1
        self._applyThrust(PWM_MAX, duration)
        # 1 -> 0
        self._applyThrust(PWM_MIN, duration)

        # 0.5 -> 0.25
        self._applyThrust((PWM_MAX + PWM_MIN) / 2, duration)
        self._applyThrust((PWM_MAX + PWM_MIN) * 1 / 4, duration)
        # 0.25 -> 0.5
        self._applyThrust((PWM_MAX + PWM_MIN) / 2, duration)

        # 0.5 -> 0.75
        self._applyThrust((PWM_MAX + PWM_MIN) * 3 / 4, duration)
        # 0.75 -> 0.5
        self._applyThrust((PWM_MAX + PWM_MIN) / 2, duration)

        # 0.5 -> 1.0
        self._applyThrust(PWM_MAX, duration)
        # 1.0 -> 0.5
        self._applyThrust((PWM_MAX + PWM_MIN) / 2, duration)

        # base speed
        self._applyThrust(PWM_MIN, 2 * duration)

    def _ramp_motors_chirp(self):
        # base speed
        self._applyThrust(PWM_MAX / 4, 2.0)

        chirp_duration = 30  # s
        chirp_send_frequency = 100  # Hz
        chirp_f0 = 0.1  # Hz
        chirp_f1 = 5  # Hz

        from scipy.signal import chirp

        t = np.linspace(0, chirp_duration, chirp_duration * chirp_send_frequency)
        c = chirp(t, chirp_f0, chirp_duration, chirp_f1, method="lin", phi=180)
        # [PWM_MIN, PWM_MIN+PWM_MAX/4]
        for sine in c:
            sine = (sine + 1) / 2  # Scaling to [0,1]
            pwm = sine * PWM_MAX / 4 + PWM_MIN  # Scaling to useful PWM values
            self._applyThrust(pwm, 1 / chirp_send_frequency)

        # [PWM_MIN, PWM_MIN+PWM_MAX/2]
        for sine in c:
            sine = (sine + 1) / 2  # Scaling to [0,1]
            pwm = sine * PWM_MAX / 2 + PWM_MIN  # Scaling to useful PWM values
            self._applyThrust(pwm, 1 / chirp_send_frequency)

        # [PWM_MAX/4, PWM_MAX*3/4]
        for sine in c:
            sine = (sine + 1) / 2  # Scaling to [0,1]
            pwm = sine * PWM_MAX / 2 + PWM_MAX / 4  # Scaling to useful PWM values
            self._applyThrust(pwm, 1 / chirp_send_frequency)

        # [PWM_MAX/2, PWM_MAX]
        for sine in c:
            sine = (sine + 1) / 2  # Scaling to [0,1]
            pwm = sine * PWM_MAX / 2 + PWM_MAX / 2  # Scaling to useful PWM values
            self._applyThrust(pwm, 1 / chirp_send_frequency)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--calibration",
        default="calibration.toml",
        help="Input file containing the calibration",
    )
    parser.add_argument(
        "--uri", default="radio://0/80/2M/E7E7E7E7E7", help="URI of Crazyflie"
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
    parser.add_argument(
        "--torques",
        action="store_true",
        help="If set, torque data will be collected (default: False)",
    )
    parser.add_argument(
        "--use_loadcell",
        action="store_true",
        help="If set, a loadcell from the Loadcell class is used (default: False)",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="If set, verbose output is enabled (default: False)",
    )
    args = parser.parse_args()

    # Only output errors from the logging framework
    logging.basicConfig(level=logging.ERROR)

    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    if args.use_loadcell:
        a, b = 0, 0  # not needed for the precalibrated loadcells

        from loadcell import Loadcell

        # Replace the name with your actual interface name. You can check that with the
        # loadcell.py scripts. Note that PySOEM needs access to your ethernet ports.
        # For that, we need to give Python permission:
        # sudo setcap cap_net_raw,cap_net_admin+eip $(readlink -f $(which python3))
        loadcell = Loadcell("enp0s31f6", verbose=False)
        loadcell.calibrate()
    else:
        with open(args.calibration, "r") as f:
            r = toml.load(f)
            a, b = r["a"], r["b"]
        loadcell = None

    # collect data
    if args.mode == "ramp_motors":
        le = CollectDataRamp(
            args.uri,
            a,
            b,
            args.comb,
            args.extra,
            loadcell=loadcell,
            verbose=args.verbose,
        )
    elif args.mode == "static":
        le = CollectDataStatic(
            args.uri,
            a,
            b,
            args.comb,
            args.extra,
            loadcell=loadcell,
            torques=args.torques,
            verbose=args.verbose,
        )
    elif args.mode == "static_verification":  # activate battery compensation to test it
        le = CollectDataStatic(
            args.uri,
            a,
            b,
            args.comb,
            args.extra,
            batComp=True,
            loadcell=loadcell,
            verbose=args.verbose,
        )
    elif args.mode == "dynamic":
        le = CollectDataDynamic(
            args.uri,
            a,
            b,
            args.comb,
            args.extra,
            loadcell=loadcell,
            verbose=args.verbose,
        )
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
