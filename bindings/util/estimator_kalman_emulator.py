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

    def __init__(self) -> None:
        self.accSubSampler = cffirmware.Axis3fSubSampler_t()
        self.gyroSubSampler = cffirmware.Axis3fSubSampler_t()
        self.coreData = cffirmware.kalmanCoreData_t()
        print(self.coreData.baroReferenceHeight)
        self.last_gyro_sample = None

        self.PREDICT_RATE = 100
        self.PREDICT_STEP_MS = 1000 / self.PREDICT_RATE

        self._is_initialized = False

    def set_parameters(self, params):
        pass

    def run_one_1khz_iteration(
        self, sensor_samples, quad_is_flying=True
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
            time_ms = int(first_sample[1]["timestamp"])
            # self._fetch_core_params()
            self._lazy_init(time_ms)

        if self.now_ms > self.next_prediction_ms:
            cffirmware.axis3fSubSamplerFinalize(self.accSubSampler)
            cffirmware.axis3fSubSamplerFinalize(self.gyroSubSampler)

            cffirmware.kalmanCorePredict(
                self.coreData,
                self.accSubSampler.subSample,
                self.gyroSubSampler.subSample,
                self.now_ms,
                quad_is_flying,
            )

            self.next_prediction_ms += self.PREDICT_STEP_MS
        cffirmware.kalmanCoreAddProcessNoise(
            self.coreData, self.coreParams, self.now_ms
        )

        self._update_queued_measurements(self.now_ms, sensor_samples, quad_is_flying)

        cffirmware.kalmanCoreFinalize(self.coreData)

        # Main loop called at 1000 Hz in the firmware
        self.now_ms += 1

        external_state = cffirmware.state_t()
        acc_latest = cffirmware.Axis3f()
        cffirmware.kalmanCoreExternalizeState(self.coreData, external_state, acc_latest)

        return self.now_ms, external_state

    def _fetch_core_params(self):
        self.coreParams = cffirmware.kalmanCoreParams_t()
        cffirmware.kalmanCoreDefaultParams(self.coreParams)

    def _lazy_init(self, sample_time_ms):
        self.now_ms = sample_time_ms
        self.next_prediction_ms = self.now_ms + self.PREDICT_STEP_MS

        GRAVITY_MAGNITUDE = 9.81
        DEG_TO_RAD = math.pi / 180.0
        cffirmware.axis3fSubSamplerInit(self.accSubSampler, GRAVITY_MAGNITUDE)
        cffirmware.axis3fSubSamplerInit(self.gyroSubSampler, DEG_TO_RAD)

        cffirmware.kalmanCoreInit(self.coreData, self.coreParams, self.now_ms)

        self._is_initialized = True

    def _update_queued_measurements(
        self, now_ms: int, sensor_samples, quad_is_flying=True
    ):
        done = False

        while len(sensor_samples):
            sample = sensor_samples.pop(0)
            time_ms = int(sample[1]["timestamp"])
            if time_ms <= now_ms:
                self._update_with_sample(sample, now_ms, quad_is_flying)
            else:
                return

    def _update_with_sample(self, sample, now_ms, quad_is_flying=True):
        if sample[0] == "estTDOA":
            tdoa_data = sample[1]
            tdoa = cffirmware.tdoaMeasurement_t()

            tdoa.anchorIdA = int(tdoa_data["idA"])
            tdoa.anchorIdB = int(tdoa_data["idB"])
            tdoa.anchorPositionA = self.anchor_positions[tdoa.anchorIdA]
            tdoa.anchorPositionB = self.anchor_positions[tdoa.anchorIdB]
            tdoa.distanceDiff = float(tdoa_data["distanceDiff"])
            tdoa.stdDev = self.TDOA_ENGINE_MEASUREMENT_NOISE_STD

            cffirmware.kalmanCoreUpdateWithTdoa(
                self.coreData, tdoa, now_ms, self.outlierFilterState
            )

        elif sample[0] == "estAcceleration":
            # print("Acceleration sample")
            acc_data = sample[1]

            acc = cffirmware.Axis3f()
            acc.x = float(acc_data["acc.x"])
            acc.y = float(acc_data["acc.y"])
            acc.z = float(acc_data["acc.z"])

            cffirmware.axis3fSubSamplerAccumulate(self.accSubSampler, acc)

        elif sample[0] == "estGyroscope":
            # print("Gyroscope sample")
            gyro_data = sample[1]

            gyro = cffirmware.Axis3f()
            gyro.x = float(gyro_data["gyro.x"])
            gyro.y = float(gyro_data["gyro.y"])
            gyro.z = float(gyro_data["gyro.z"])

            self.last_gyro_sample = gyro

            cffirmware.axis3fSubSamplerAccumulate(self.gyroSubSampler, gyro)

        elif sample[0] == "estFlow":
            # print("Flow sample")
            flow_data = sample[1]
            flow = cffirmware.flowMeasurement_t()

            flow.deltaX = float(flow_data["motion.deltaX"])
            flow.deltaY = float(flow_data["motion.deltaY"])
            # flow.stdDev = self.FLOW_ENGINE_MEASUREMENT_NOISE_STD

            cffirmware.kalmanCoreUpdateWithFlow(
                self.coreData, flow, self.last_gyro_sample
            )

        elif sample[0] == "estTOF":
            # print("TOF sample")
            tof_data = sample[1]
            tof = cffirmware.tofMeasurement_t()

            # tof.timestamp = tof_data['timestamp']
            tof.distance = float(tof_data["kalman_mm.tofDistance"])
            tof.stdDev = float(tof_data["kalman_mm.tofStdDev"])

            cffirmware.kalmanCoreUpdateWithTof(self.coreData, tof)

        elif sample[0] == "estBarometer":
            # print("Baro sample")
            baro_data = sample[1]
            baro_asl = baro_data["baro.asl"]
            cffirmware.kalmanCoreUpdateWithBaro(self.coreData, self.coreParams, baro_asl, quad_is_flying)

        elif sample[0] == "fixedFrequency":
            pos_data = sample[1]
            pos = cffirmware.positionMeasurement_t()
            # print(pos_data.keys())
            pos.x = float(pos_data["stateEstimate.x"])
            pos.y = float(pos_data["stateEstimate.y"])
            # pos.z = float(pos_data["pos.z"])
            # pos.stdDev = float(pos_data["pos.stdDev"])

            cffirmware.kalmanCoreUpdateWithPosition(self.coreData, pos)

        else:
            print("Unknown sample type: {}".format(sample[0]))
            pass