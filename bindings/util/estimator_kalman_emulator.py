from __future__ import annotations
import math
import cffirmware

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
    def __init__(self, anchor_positions=None, basestation_poses=None, basestation_calibration=None) -> None:
        self.anchor_positions = anchor_positions
        self.basestation_poses = basestation_poses
        self.basestation_calibration = basestation_calibration
        self.accSubSampler = cffirmware.Axis3fSubSampler_t()
        self.gyroSubSampler = cffirmware.Axis3fSubSampler_t()
        self.coreData = cffirmware.kalmanCoreData_t()
        self.outlierFilterStateTdoa = cffirmware.OutlierFilterTdoaState_t()
        self.outlierFilterStateLH = cffirmware.OutlierFilterLhState_t()

        self.TDOA_ENGINE_MEASUREMENT_NOISE_STD = 0.30
        self.LH_ENGINE_MEASUREMENT_NOISE_STD = 0.001
        self.PREDICT_RATE = 100
        self.PREDICT_STEP_MS = 1000 / self.PREDICT_RATE

        self._is_initialized = False

    def run_one_1khz_iteration(self, sensor_samples) -> tuple[float, cffirmware.state_t]:
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

        # Simplification, assume always flying
        quad_is_flying = True

        if self.now_ms > self.next_prediction_ms:
            cffirmware.axis3fSubSamplerFinalize(self.accSubSampler)
            cffirmware.axis3fSubSamplerFinalize(self.gyroSubSampler)

            cffirmware.kalmanCorePredict(self.coreData, self.accSubSampler.subSample, self.gyroSubSampler.subSample,
                                            self.now_ms, quad_is_flying)

            self.next_prediction_ms += self.PREDICT_STEP_MS

        cffirmware.kalmanCoreAddProcessNoise(self.coreData, self.coreParams, self.now_ms)

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
        cffirmware.outlierFilterTdoaReset(self.outlierFilterStateTdoa)
        cffirmware.outlierFilterLighthouseReset(self.outlierFilterStateLH, self.now_ms)
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
    def convert_to_angles(v2angle1, v2angle2):
        v1Angles = []
        v1Angles[0] = (v2angle1 + v2angle2) / 2.0
        v1Angles[1] = atan1(sin(v2angle2 - v2angle1), (tan(np.pi/6.0) * (cos(v2angle1) + cos(v2angle2))))

    def _update_with_sample(self, sample, now_ms):
        position = [0.0, 0.0, 0.0]
        position[0] = cffirmware.get_state(self.coreData, 0)
        position[1] = cffirmware.get_state(self.coreData, 1)
        position[2] = cffirmware.get_state(self.coreData, 2)
        #print("Position: ", position)

        rotation_matrix = [[0.0, 0.0, 0.0],[0.0, 0.0, 0.0],[0.0, 0.0, 0.0]]

        for i in range(0, 3):
            for j in range(0, 3):
                rotation_matrix[i][j] = cffirmware.get_mat_index(self.coreData, i,j)

        #print("Position: ", position)
        #print("Rotation Matrix: ", rotation_matrix)

        if sample[0] == 'estTDOA':
            tdoa_data = sample[1]
            tdoa = cffirmware.tdoaMeasurement_t()

            tdoa.anchorIdA = int(tdoa_data['idA'])
            tdoa.anchorIdB = int(tdoa_data['idB'])
            tdoa.anchorPositionA = self.anchor_positions[tdoa.anchorIdA]
            tdoa.anchorPositionB = self.anchor_positions[tdoa.anchorIdB]
            tdoa.distanceDiff = float(tdoa_data['distanceDiff'])
            tdoa.stdDev = self.TDOA_ENGINE_MEASUREMENT_NOISE_STD

            cffirmware.kalmanCoreUpdateWithTdoa(self.coreData, tdoa, now_ms, self.outlierFilterStateTdoa)

        if sample[0] == 'estSweepAngle':
            sweep_data = sample[1]
            sweep = cffirmware.sweepAngleMeasurement_t()

            sweep.sensorId = int(sweep_data['sensorId'])
            sweep.baseStationId = int(sweep_data['baseStationId'])
            sweep.sweepId = int(sweep_data['sweepId'])
            sweep.t = float(sweep_data['t'])
            sweep.measuredSweepAngle = float(sweep_data['sweepAngle'])
            sweep.stdDev = self.LH_ENGINE_MEASUREMENT_NOISE_STD
            cffirmware.set_calibration_model(sweep, self.basestation_calibration[sweep.baseStationId][sweep.sweepId])

            sensor_pos_w = 0.015/2.0
            sensor_pos_l = 0.030/2.0
            sensor_position = {}
            sensor_position[0] = [-sensor_pos_w, sensor_pos_l, 0.0]
            sensor_position[1] = [-sensor_pos_w, -sensor_pos_l, 0.0]
            sensor_position[2] = [sensor_pos_w, sensor_pos_l, 0.0]
            sensor_position[3] = [sensor_pos_w, -sensor_pos_l, 0.0]

            sensorPos = cffirmware.vec3_s()
            sensorPos.x = sensor_position[int(sweep.sensorId)][0]
            sensorPos.y = sensor_position[int(sweep.sensorId)][1]
            sensorPos.z = sensor_position[int(sweep.sensorId)][2]
            cffirmware.set_sensor_pos(sweep, sensorPos)

            cffirmware.set_pose_origin_mat(sweep, self.basestation_poses[sweep.baseStationId].origin, self.basestation_poses[sweep.baseStationId].mat)
            cffirmware.set_inv_mat(sweep,  self.basestation_poses[sweep.baseStationId].mat )
            #cffirmware.print_sweep_angle(sweep)
            #geometry_cache = cffirmware.baseStationGeometryCache_t()
            #cffirmware.preProcessGeometryData(sweep.rotorRot, geometry_cache.baseStationInvertedRotationMatrixes, geometry_cache.lh1Rotor2RotationMatrixes, geometry_cache.lh1Rotor2InvertedRotationMatrixes)

            #sweep.rotorRotInv = geometry_cache.baseStationInvertedRotationMatrixes[sweep.sensorId]

            cffirmware.kalmanCoreUpdateWithSweepAngles(self.coreData, sweep, now_ms, self.outlierFilterStateLH)
            # yaw error

            # apply calibration
            #corrected_angle = 0.0
            #cffirmware.lighthouseCalibrationApplyV2(sweep.callib, sweep.measuredSweepAngle, corrected_angle)
            # get basestation geometry
            #basestation_geo = self.basestation_poses[sweep.baseStationId]
            # get ray per position



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

        if sample[0] == 'estLighthouse':
            lh_data = sample[1]
            lh = cffirmware.sweepAngleMeasurement_t()


            # lh.timestamp
            lh.sensorPos = [0.0, 0.0, 0.0]
            lh.rotorPos = [0.0, 0.0, 0.0]
            lh.RoterPot = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
            lh.rotorRotInv = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
            #lh.sensorId = int(lh_data['sensorId'])
            #lh.baseStationId = int(lh_data['baseStationId'])
            lh.sweepId = int(lh_data['sweepId'])
            lh.t = float(lh_data['t'])
            lh.measuredSweepAngle = float(lh_data['sweepAngle'])
            lh.stdDev = self.LH_ENGINE_MEASUREMENT_NOISE_STD
            lh.calib = cffirmware.lighthouseCalibration_t()
            lh.calibrationMeasurementModel = cffirmware.lighthouseCalibrationMeasurementModel_t()

            cffirmware.kalmanCoreUpdateWithSweepAngles(self.coreData, lh, now_ms, self.outlierFilterStateLH)
