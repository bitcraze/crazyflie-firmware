import tools.usdlog.cfusdlog as cfusdlog
from bindings.util.estimator_kalman_emulator import EstimatorKalmanEmulator

class SdCardFileRunner:
    """
    This class is used to read data from a file and feed it to a kalman estimator emulator, usually for testing
    purposes.
    """

    def __init__(self, file_name: str) -> None:
        """
        Read sampled data from a file and feed to a kalman estimator emulator.
        The supported file format is what is recorded using a uSD-card deck on the Crazyflie.
        In the common use case the file should contain accelerometer and gyro data, as well as some other sensor
        data that is to be fed to the kalman estimator.

        Args:
            file_name (str): Name of the file
        """
        self.samples = self._read_sensor_data_sorted(file_name)

    def run_estimator_loop(self, emulator: EstimatorKalmanEmulator):
        result = []
        while len(self.samples):
            now_ms, external_state = emulator.run_one_1khz_iteration(self.samples)
            result.append((now_ms, (external_state.position.x, external_state.position.y, external_state.position.z)))

        return result

    def _read_sensor_data_sorted(self, file_name: str):
        """Read sensor data from a file recorded using the uSD-card on a Crazyflie

        Args:
            file_name: The name of the file with recorded data

        Returns:
            A list of sensor samples, sorted in time order. The first field of each row identifies the sensor
            that produced the sample
        """
        log_data = cfusdlog.decode(file_name)

        samples = []
        for log_type, data in log_data.items():
            for i in range(len(data['timestamp'])):
                sample_data = {}
                for name, data_list in data.items():
                    sample_data[name] = data_list[i]
                samples.append((log_type, sample_data))

        samples.sort(key=lambda x: x[1]['timestamp'])
        return samples
