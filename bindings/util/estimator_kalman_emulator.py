import math
import cffirmware
import logging


class EstimatorKalmanEmulator:
    """
    This class emulates the behavior of estimator_kalman.c and is used as a helper to enable testing of the kalman
    core functionatlity. Estimator_kalman.c is tightly coupled to FreeRTOS (using
    tasks for instance) and can not really be tested on this level, instead this class can be used to drive the
    kalman core functionality.

    The class emulates the measurement queue, the main loop in the task and the various calls to kalman core.

    Methods are named in a similar way to the functions in estimator_kalman.c to make it easier to understand
    how they are connected.

    """

    def __init__(
        self,
        anchor_positions=None,
        basestation_poses=None,
        basestation_calibration=None,
    ) -> None:
        self.logger = logging.getLogger(self.__class__.__name__)
        self.anchor_positions = anchor_positions
        self.basestation_poses = basestation_poses
        self.basestation_calibration = basestation_calibration
        self.accSubSampler = cffirmware.Axis3fSubSampler_t()
        self.gyroSubSampler = cffirmware.Axis3fSubSampler_t()
        self.coreData = cffirmware.kalmanCoreData_t()
        self.outlierFilterTDOA = cffirmware.OutlierFilterTdoaState_t()
        self.outlierFilterLH = cffirmware.OutlierFilterLhState_t()
        self.onboard_state_estimate = []

        # Simplification, assume always flying
        self.logger.warning(
            'Assuming the quad is always flying. This simplification must be considered '
            'as it can affect the results. If you need to handle both flying and non-flying '
            'states, ensure this matches your recorded data. Automatic handling is not '
            'currently implemented.'
        )
        self.quad_is_flying = True

        self.TDOA_ENGINE_MEASUREMENT_NOISE_STD = 0.30
        self.PREDICT_RATE = 100
        self.PREDICT_STEP_MS = 1000 / self.PREDICT_RATE

        self._is_initialized = False

    def run_one_1khz_iteration(
        self, sensor_samples
    ) -> tuple[float, cffirmware.state_t]:
        """
        Run one iteration of the estimation loop (runs at 1kHz)

        Args:
            sensor_samples : a list of samples to be consumed. The samples with time stamps that are used in this
                             iteration will be popped from the list.

        Returns:
            tuple[float, cffirmware.state_t]: A tuple containing the time stamp of this iteration and the
                                              estimated state
        """
        if not self._is_initialized:
            first_sample = sensor_samples[0]
            time_ms = int(first_sample[1]['timestamp'])
            self._lazy_init(time_ms)

        if self.now_ms > self.next_prediction_ms:
            cffirmware.axis3fSubSamplerFinalize(self.accSubSampler)
            cffirmware.axis3fSubSamplerFinalize(self.gyroSubSampler)

            cffirmware.kalmanCorePredict(
                self.coreData,
                self.accSubSampler.subSample,
                self.gyroSubSampler.subSample,
                self.now_ms,
                self.quad_is_flying,
            )

            self.next_prediction_ms += self.PREDICT_STEP_MS

        cffirmware.kalmanCoreAddProcessNoise(
            self.coreData, self.coreParams, self.now_ms
        )

        self._update_queued_measurements(self.now_ms, sensor_samples)

        cffirmware.kalmanCoreFinalize(self.coreData)

        # Main loop called at 1000 Hz in the firmware
        self.now_ms += 1

        external_state = cffirmware.state_t()
        acc_latest = cffirmware.Axis3f()
        cffirmware.kalmanCoreExternalizeState(self.coreData, external_state, acc_latest)

        return self.now_ms, external_state

    def _lazy_init(self, sample_time_ms):
        self.now_ms = sample_time_ms
        self.next_prediction_ms = self.now_ms + self.PREDICT_STEP_MS

        GRAVITY_MAGNITUDE = 9.81
        DEG_TO_RAD = math.pi / 180.0
        cffirmware.axis3fSubSamplerInit(self.accSubSampler, GRAVITY_MAGNITUDE)
        cffirmware.axis3fSubSamplerInit(self.gyroSubSampler, DEG_TO_RAD)

        self.coreParams = cffirmware.kalmanCoreParams_t()
        cffirmware.kalmanCoreDefaultParams(self.coreParams)
        cffirmware.outlierFilterTdoaReset(self.outlierFilterTDOA)
        cffirmware.outlierFilterLighthouseReset(self.outlierFilterLH, self.now_ms)
        cffirmware.kalmanCoreInit(self.coreData, self.coreParams, self.now_ms)

        self._is_initialized = True

    def _update_queued_measurements(self, now_ms: int, sensor_samples):
        done = False

        while len(sensor_samples):
            sample = sensor_samples.pop(0)
            time_ms = int(sample[1]['timestamp'])
            if time_ms <= now_ms:
                self._update_with_sample(sample, now_ms)
            else:
                return

    def _update_with_sample(self, sample, now_ms):
        if sample[0] == 'estTDOA':
            self.logger.debug('Processing a TDOA sample')
            tdoa_data = sample[1]
            tdoa = cffirmware.tdoaMeasurement_t()

            tdoa.anchorIdA = int(tdoa_data['idA'])
            tdoa.anchorIdB = int(tdoa_data['idB'])
            tdoa.anchorPositionA = self.anchor_positions[tdoa.anchorIdA]
            tdoa.anchorPositionB = self.anchor_positions[tdoa.anchorIdB]
            tdoa.distanceDiff = float(tdoa_data['distanceDiff'])
            tdoa.stdDev = self.TDOA_ENGINE_MEASUREMENT_NOISE_STD

            cffirmware.kalmanCoreUpdateWithTdoa(
                self.coreData, tdoa, now_ms, self.outlierFilterTDOA
            )

        if sample[0] == 'estAcceleration':
            self.logger.debug('Processing an acceleration sample')
            acc_data = sample[1]

            acc = cffirmware.Axis3f()
            acc.x = float(acc_data['acc.x'])
            acc.y = float(acc_data['acc.y'])
            acc.z = float(acc_data['acc.z'])

            cffirmware.axis3fSubSamplerAccumulate(self.accSubSampler, acc)

        if sample[0] == 'estGyroscope':
            self.logger.debug('Processing a gyroscope sample')
            gyro_data = sample[1]

            gyro = cffirmware.Axis3f()
            gyro.x = float(gyro_data['gyro.x'])
            gyro.y = float(gyro_data['gyro.y'])
            gyro.z = float(gyro_data['gyro.z'])

            cffirmware.axis3fSubSamplerAccumulate(self.gyroSubSampler, gyro)

        if sample[0] == 'estYawError':
            self.logger.debug('Processing a yaw error sample')
            yaw_error_data  = sample[1]
            yaw_error = cffirmware.yawErrorMeasurement_t()
            yaw_error.yawError = float(yaw_error_data['yawError'])
            yaw_error.stdDev = 0.01

            cffirmware.kalmanCoreUpdateWithYawError(self.coreData, yaw_error)

        if sample[0] == 'estSweepAngle':
            self.logger.debug('Processing a sweep angle sample')
            sweep_data = sample[1]

            sweep = cffirmware.sweepAngleMeasurement_t()

            sweep.timestamp = int(sweep_data['timestamp'])
            sweep.sensorId = int(sweep_data['sensorId'])
            sweep.baseStationId = int(sweep_data['baseStationId'])
            sweep.sweepId = int(sweep_data['sweepId'])
            sweep.t = float(sweep_data['t'])
            sweep.measuredSweepAngle = float(sweep_data['sweepAngle'])
            sweep.stdDev = 0.001  # fixed in firmware

            cffirmware.set_calibration_model(
                sweep, self.basestation_calibration[sweep.baseStationId][sweep.sweepId]
            )

            sensor_pos_w = 0.015 / 2.0
            sensor_pos_l = 0.030 / 2.0
            sensor_position = {}
            sensor_position[0] = [-sensor_pos_w, sensor_pos_l, 0.0]
            sensor_position[1] = [-sensor_pos_w, -sensor_pos_l, 0.0]
            sensor_position[2] = [sensor_pos_w, sensor_pos_l, 0.0]
            sensor_position[3] = [sensor_pos_w, -sensor_pos_l, 0.0]

            sensorPos = cffirmware.make_vec3d(
                sensor_position[int(sweep.sensorId)][0],
                sensor_position[int(sweep.sensorId)][1],
                sensor_position[int(sweep.sensorId)][2],
            )
            sweep.sensorPos = sensorPos

            rotorPos = cffirmware.make_vec3d(
                self.basestation_poses[sweep.baseStationId]['origin'].x,
                self.basestation_poses[sweep.baseStationId]['origin'].y,
                self.basestation_poses[sweep.baseStationId]['origin'].z,
            )
            sweep.rotorPos = rotorPos

            rotorRot = cffirmware.make_mat3d(
                self.basestation_poses[sweep.baseStationId]['rotation_matrix'].i11,
                self.basestation_poses[sweep.baseStationId]['rotation_matrix'].i12,
                self.basestation_poses[sweep.baseStationId]['rotation_matrix'].i13,
                self.basestation_poses[sweep.baseStationId]['rotation_matrix'].i21,
                self.basestation_poses[sweep.baseStationId]['rotation_matrix'].i22,
                self.basestation_poses[sweep.baseStationId]['rotation_matrix'].i23,
                self.basestation_poses[sweep.baseStationId]['rotation_matrix'].i31,
                self.basestation_poses[sweep.baseStationId]['rotation_matrix'].i32,
                self.basestation_poses[sweep.baseStationId]['rotation_matrix'].i33,
            )
            sweep.rotorRot = rotorRot

            rotorRotInv = cffirmware.make_mat3d(
                self.basestation_poses[sweep.baseStationId]['rotation_matrix'].i11,
                self.basestation_poses[sweep.baseStationId]['rotation_matrix'].i21,
                self.basestation_poses[sweep.baseStationId]['rotation_matrix'].i31,
                self.basestation_poses[sweep.baseStationId]['rotation_matrix'].i12,
                self.basestation_poses[sweep.baseStationId]['rotation_matrix'].i22,
                self.basestation_poses[sweep.baseStationId]['rotation_matrix'].i32,
                self.basestation_poses[sweep.baseStationId]['rotation_matrix'].i13,
                self.basestation_poses[sweep.baseStationId]['rotation_matrix'].i23,
                self.basestation_poses[sweep.baseStationId]['rotation_matrix'].i33,
            )
            sweep.rotorRotInv = rotorRotInv

            self.logger.debug(
                f'\nProcessing Sweep Angle Sample:\n'
                f'{"Sensor ID":<15}: {sweep.sensorId}\n'
                f'{"Base Station ID":<15}: {sweep.baseStationId}\n'
                f'{"Sweep ID":<15}: {sweep.sweepId}\n'
                f'{"Time (t)":<15}: {sweep.t:.3f}\n'
                f'{"Sweep Angle":<15}: {sweep.measuredSweepAngle:.3f}'
            )

            cffirmware.kalmanCoreUpdateWithSweepAngles(
                self.coreData, sweep, now_ms, self.outlierFilterLH
            )
