#!/usr/bin/env python

import yaml
import cffirmware
import tools.usdlog.cfusdlog as cfusdlog
import math
import numpy as np

def test_kalman_core_with_tdoa3():
    # Fixture
    fixture_base = 'test_python/fixtures/kalman_core'
    anchor_positions = read_loco_anchor_positions(fixture_base + '/anchor_positions.yaml')
    sensor_samples = read_sensor_data_sorted(fixture_base + '/log05')
    emulator = EstimatorKalmanEmulator(anchor_positions)

    # Test
    actual = emulator.run_estimator_loop(sensor_samples)

    # Assert
    # Verify that the final position is close-ish to (0, 0, 0)
    actual_final_pos = np.array(actual[-1][1])
    assert np.linalg.norm(actual_final_pos - [0.0, 0.0, 0.0]) < 0.4


## Helpers ###########################

def read_loco_anchor_positions(file_name: str) -> dict[int, cffirmware.vec3_s]:
    """
    Read anchor position data from a file exported from the client.

    Args:
        file_name (str): The name of the file

    Returns:
        dict[int, cffirmware.vec3_s]: A dictionary from anchor id to a 3D-vector
    """
    result = {}

    with open(file_name, 'r') as file:
        data = yaml.safe_load(file)
        for id, vals in data.items():
            point = cffirmware.vec3_s()
            point.x = vals['x']
            point.y = vals['y']
            point.z = vals['z']
            result[id] = point

    return result


def read_sensor_data_sorted(file_name: str):
    """Read sensor data from a file recorded using the uSD-card on a Crazyflie

    Args:
        file_name: The name of the file with recorded data

    Returns:
        _type_: A list of sensor samples, sorted in time order. The first field of each row identifies the sensor
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


class EstimatorKalmanEmulator:
    """
    This class emulates the behavior of estimator_kalman.c and is used as a helper to enable testing of the kalman
    core functionatlity. Estimator_kalman.c is tightly coupled to FreeRTOS (using
    tasks for instance) and can not really be tested on this level, instead this class can be used to drive the
    kalman core functionlity.
    """
    def __init__(self, anchor_positions) -> None:
        self.anchor_positions = anchor_positions
        self.accSubSampler = cffirmware.Axis3fSubSampler_t()
        self.gyroSubSampler = cffirmware.Axis3fSubSampler_t()
        self.coreData = cffirmware.kalmanCoreData_t()
        self.outlierFilterState = cffirmware.OutlierFilterTdoaState_t()

        self.TDOA_ENGINE_MEASUREMENT_NOISE_STD = 0.30
        self.PREDICT_RATE = 100
        self.PREDICT_STEP_MS = 1000 / self.PREDICT_RATE

    def run_estimator_loop(self, sensor_samples):
        """
        Emulation of the main loop in estimator_kalman.c

        Args:
            sensor_samples: Ordered list of sensor samples

        Returns:
            estimated positions: A list of tuples containing the time and estimated position
        """
        start_index = 0
        os_time_first_ms = int(sensor_samples[start_index][1]['timestamp'])
        now_ms = os_time_first_ms
        next_prediction_ms = now_ms + self.PREDICT_STEP_MS

        GRAVITY_MAGNITUDE = 9.81
        DEG_TO_RAD = math.pi / 180.0
        cffirmware.axis3fSubSamplerInit(self.accSubSampler, GRAVITY_MAGNITUDE)
        cffirmware.axis3fSubSamplerInit(self.gyroSubSampler, DEG_TO_RAD)

        coreParams = cffirmware.kalmanCoreParams_t()
        cffirmware.kalmanCoreDefaultParams(coreParams)
        cffirmware.outlierFilterTdoaReset(self.outlierFilterState)
        cffirmware.kalmanCoreInit(self.coreData, coreParams, now_ms)

        # Simplification, assume always flying
        quad_is_flying = True

        # Main loop
        index = start_index
        result = []
        while index < len(sensor_samples):
            if now_ms > next_prediction_ms:
                cffirmware.axis3fSubSamplerFinalize(self.accSubSampler)
                cffirmware.axis3fSubSamplerFinalize(self.gyroSubSampler)

                cffirmware.kalmanCorePredict(self.coreData, self.accSubSampler.subSample, self.gyroSubSampler.subSample,
                                             now_ms, quad_is_flying)

                next_prediction_ms += self.PREDICT_STEP_MS

            cffirmware.kalmanCoreAddProcessNoise(self.coreData, coreParams, now_ms)

            index = self._update_queued_measurements(now_ms, sensor_samples, index)

            cffirmware.kalmanCoreFinalize(self.coreData)

            external_state = cffirmware.state_t()
            acc_latest = cffirmware.Axis3f()
            cffirmware.kalmanCoreExternalizeState(self.coreData, external_state, acc_latest)
            result.append((now_ms, (external_state.position.x, external_state.position.y, external_state.position.z)))

            # Main loop called at 1000 Hz in the firmware
            now_ms += 1

        return result

    def _update_queued_measurements(self, now_ms: int, sensor_samples, current_index: int) -> int:
        index = current_index
        done = False

        while not done:
            sample = sensor_samples[index]
            time_ms = int(sample[1]['timestamp'])
            if time_ms <= now_ms:
                self._update_with_sample(sample, now_ms)

                index += 1
                done = index >= len(sensor_samples)
            else:
                done = True

        return index

    def _update_with_sample(self, sample, now_ms):
        if sample[0] == 'estTDOA':
            tdoa_data = sample[1]
            tdoa = cffirmware.tdoaMeasurement_t()

            tdoa.anchorIdA = int(tdoa_data['idA'])
            tdoa.anchorIdB = int(tdoa_data['idB'])
            tdoa.anchorPositionA = self.anchor_positions[tdoa.anchorIdA]
            tdoa.anchorPositionB = self.anchor_positions[tdoa.anchorIdB]
            tdoa.distanceDiff = float(tdoa_data['distanceDiff'])
            tdoa.stdDev = self.TDOA_ENGINE_MEASUREMENT_NOISE_STD

            cffirmware.kalmanCoreUpdateWithTdoa(self.coreData, tdoa, now_ms, self.outlierFilterState)

        if sample[0] == 'estAcceleration':
            acc_data = sample[1]

            acc = cffirmware.Axis3f()
            acc.x = float(acc_data['acc.x'])
            acc.y = float(acc_data['acc.y'])
            acc.z = float(acc_data['acc.z'])

            cffirmware.axis3fSubSamplerAccumulate(self.accSubSampler, acc)

        if sample[0] == 'estGyroscope':
            gyro_data = sample[1]

            gyro = cffirmware.Axis3f()
            gyro.x = float(gyro_data['gyro.x'])
            gyro.y = float(gyro_data['gyro.y'])
            gyro.z = float(gyro_data['gyro.z'])

            cffirmware.axis3fSubSamplerAccumulate(self.gyroSubSampler, gyro)
