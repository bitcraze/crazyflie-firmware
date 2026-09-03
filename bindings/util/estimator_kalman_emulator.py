import math
import cffirmware

TDOA_MODELS = ('standard', 'robust')


class EstimatorKalmanEmulator:
    """
    This class emulates the behavior of estimator_kalman.c and is used as a helper to enable testing of the kalman
    core functionatlity. Estimator_kalman.c is tightly coupled to FreeRTOS (using
    tasks for instance) and can not really be tested on this level, instead this class can be used to drive the
    kalman core functionality.

    The class emulates the measurement queue, the main loop in the task and the various calls to kalman core.

    Methods are named in a similar way to the functions in estimator_kalman.c to make it easier to understand
    how they are connected.

    An optional outlier_filter (see bindings/util/tdoa_outlier.py) externalizes
    the TDoA outlier decision: None keeps the firmware's built-in behavior
    (standard model: integrator filter in C inside kalmanCoreUpdateWithTdoa;
    robust model: ungated, like kalman.robustTdoa = 1), while a filter object
    routes each TDoA sample through kalmanCoreTdoaInnovation -> validate ->
    the tdoa_model's update (kalmanCoreUpdateWithTdoaUnfiltered or
    kalmanCoreRobustUpdateWithTdoa).

    """
    def __init__(self, anchor_positions, tdoa_model='standard',
                 outlier_filter=None, std_model=None,
                 kalman_params=None) -> None:
        if tdoa_model not in TDOA_MODELS:
            raise ValueError(
                f"unknown tdoa_model '{tdoa_model}', expected one of {TDOA_MODELS}")
        self.tdoa_model = tdoa_model
        self.outlier_filter = outlier_filter
        self.std_model = std_model
        # Overrides applied on top of kalmanCoreDefaultParams at init, e.g.
        # {'procNoiseVel': 0.3} to A/B process-noise tuning in replay.
        self.kalman_params = kalman_params or {}
        self.anchor_positions = anchor_positions
        self.accSubSampler = cffirmware.Axis3fSubSampler_t()
        self.gyroSubSampler = cffirmware.Axis3fSubSampler_t()
        self.coreData = cffirmware.kalmanCoreData_t()
        self.outlierFilterState = cffirmware.OutlierFilterTdoaState_t()

        self.TDOA_ENGINE_MEASUREMENT_NOISE_STD = 0.30
        self.PREDICT_RATE = 100
        self.PREDICT_STEP_MS = 1000 / self.PREDICT_RATE

        self.EXT_POS_STD_DEV = 0.01
        self.EXT_QUAT_STD_DEV = 4.5e-3

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

        # TDoA measurements processed during this iteration, appended by
        # _update_with_sample; read back by _run_replay to build the
        # innovation trace (see tdoa_replay.replay_full_state).
        self.tdoa_events = []

        # Simplification, assume always flying
        quad_is_flying = True

        if self.now_ms > self.next_prediction_ms:
            cffirmware.axis3fSubSamplerFinalize(self.accSubSampler)
            cffirmware.axis3fSubSamplerFinalize(self.gyroSubSampler)

            cffirmware.kalmanCorePredict(self.coreData, self.coreParams, self.accSubSampler.subSample,
                                         self.gyroSubSampler.subSample, self.now_ms, quad_is_flying)

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
        for key, value in self.kalman_params.items():
            if not hasattr(self.coreParams, key):
                raise ValueError(f"kalmanCoreParams_t has no field '{key}'")
            setattr(self.coreParams, key, value)
        # Note: If the emulator is used with data from a deck that uses roll/pitch/yaw zero reversion, this should be
        # set to a non-zero value to behave like the CF. See estimatorKalmanInit() in estimator_kalman.c
        # self.coreParams.AttitudeReversion = 0.001

        cffirmware.outlierFilterTdoaReset(self.outlierFilterState)
        if self.outlier_filter is not None:
            self.outlier_filter.reset()
        if self.std_model is not None:
            self.std_model.reset()
        cffirmware.kalmanCoreInit(self.coreData, self.coreParams, self.now_ms)

        self._is_initialized = True

    def _update_queued_measurements(self, now_ms: int, sensor_samples):
        # Continue processing as long as there is data
        while len(sensor_samples):
            # Peek at the first sample without removing it
            sample = sensor_samples[0]
            time_ms = int(sample[1]['timestamp'])

            # If the sample is ready to be processed (past or present time)
            if time_ms <= now_ms:
                sensor_samples.pop(0)  # Now it's safe to remove it
                self._update_with_sample(sample, now_ms)
            else:
                # The next sample is in the future; stop processing for this step
                return

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

            # Read-only (const kalmanCoreData_t*): safe to compute up front
            # for every sample, independent of std_model/outlier_filter, so
            # the innovation trace below always has a value (NaN on
            # degenerate geometry).
            error = cffirmware.kalmanCoreTdoaInnovation(self.coreData, tdoa)

            if self.std_model is not None:
                tdoa.stdDev = self.std_model.stddev(tdoa, tdoa_data, error, now_ms)

            accepted = None
            if self.outlier_filter is None:
                # Built-in behavior: standard is gated by the C integrator
                # filter inside kalmanCoreUpdateWithTdoa; robust ignores the
                # filter state (ungated in the firmware too: robustTdoa = 1).
                # Whether this particular sample was gated in/out is internal
                # to that C call and not observable here, hence accepted=None.
                if self.tdoa_model == 'robust':
                    cffirmware.kalmanCoreRobustUpdateWithTdoa(
                        self.coreData, tdoa, self.outlierFilterState)
                else:
                    cffirmware.kalmanCoreUpdateWithTdoa(
                        self.coreData, tdoa, now_ms, self.outlierFilterState)
            else:
                # NaN = degenerate geometry; firmware skips the sample without
                # consulting the filter, so neither do we.
                accepted = not math.isnan(error) and self.outlier_filter.validate(
                    tdoa, error, now_ms)
                if accepted:
                    if self.tdoa_model == 'robust':
                        cffirmware.kalmanCoreRobustUpdateWithTdoa(
                            self.coreData, tdoa, self.outlierFilterState)
                    else:
                        cffirmware.kalmanCoreUpdateWithTdoaUnfiltered(self.coreData, tdoa)

            self.tdoa_events.append({
                'timestamp': tdoa_data['timestamp'],
                'idA': tdoa.anchorIdA,
                'idB': tdoa.anchorIdB,
                'error': error,
                'std_dev': tdoa.stdDev,
                'accepted': accepted,
            })

        elif sample[0] == 'estAcceleration':
            acc_data = sample[1]

            acc = cffirmware.Axis3f()
            acc.x = float(acc_data['acc.x'])
            acc.y = float(acc_data['acc.y'])
            acc.z = float(acc_data['acc.z'])

            cffirmware.axis3fSubSamplerAccumulate(self.accSubSampler, acc)

        elif sample[0] == 'estGyroscope':
            gyro_data = sample[1]

            gyro = cffirmware.Axis3f()
            gyro.x = float(gyro_data['gyro.x'])
            gyro.y = float(gyro_data['gyro.y'])
            gyro.z = float(gyro_data['gyro.z'])

            cffirmware.axis3fSubSamplerAccumulate(self.gyroSubSampler, gyro)

        elif sample[0] == 'estExtPose':
            ext_pose = cffirmware.poseMeasurement_t()
            ext_quat = cffirmware.quaternion_t()

            pose_data = sample[1]

            ext_pose.x = float(pose_data['pos_x'])
            ext_pose.y = float(pose_data['pos_y'])
            ext_pose.z = float(pose_data['pos_z'])

            ext_quat.x = float(pose_data['quat_x'])
            ext_quat.y = float(pose_data['quat_y'])
            ext_quat.z = float(pose_data['quat_z'])
            ext_quat.w = float(pose_data['quat_w'])

            ext_pose.quat = ext_quat

            ext_pose.stdDevPos = float(pose_data.get('stdDevPos', self.EXT_POS_STD_DEV))
            ext_pose.stdDevQuat = float(pose_data.get('stdDevQuat', self.EXT_QUAT_STD_DEV))

            cffirmware.kalmanCoreUpdateWithPose(self.coreData, ext_pose)

        else:
            print(f"Unhandled measurement!: {sample[0]}")
