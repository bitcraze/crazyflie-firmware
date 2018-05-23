import ctypes
import yaml
import math

tester = ctypes.cdll.LoadLibrary(
    '../../generated-test/module-test/tdoa3/tester.so')
tester.callTdoaEngineProcessPacket.restype = ctypes.c_bool


class RangeData(ctypes.Structure):
    _fields_ = [
        ("id", ctypes.c_uint),
        ("seqNr", ctypes.c_uint),
        ("rxTimestamp", ctypes.c_uint),
        ("distance", ctypes.c_uint)
    ]


class Point(ctypes.Structure):
    _fields_ = [
        ('timestamp', ctypes.c_uint),
        ("x", ctypes.c_float),
        ("y", ctypes.c_float),
        ("z", ctypes.c_float)
    ]


class TdoaMeasurement(ctypes.Structure):
    _fields_ = [
        ("anchorPosition", Point * 2),
        ("distanceDiff", ctypes.c_float),
        ("stdDev", ctypes.c_float)
    ]


def simulateClock(time):
    tick = (int(time * 1000) & 0xffffffff)
    tester.simulateSetTestCurrentSysTime(tick)


def sendPacketToEngine(packet):
    anchorId = packet['from']
    txAn_in_cl_An = packet['txTimeStamp']
    rxAn_by_T_in_cl_T = packet['ts']
    seqNr = packet['seq']

    position = None
    if 'lpp_data' in packet and packet['lpp_data']['type'] == 1:
        position = (Point)()
        pos = packet['lpp_data']['position']
        position.x = pos['x']
        position.y = pos['y']
        position.z = pos['z']

    remoteCount = packet['remoteCount']
    remoteData = (RangeData * 8)()
    i = 0
    for packetRemoteData in packet['remoteAnchorData']:
        remoteData[i].id = packetRemoteData['id']
        remoteData[i].seqNr = packetRemoteData['seq']
        remoteData[i].rxTimestamp = packetRemoteData['rxTimeStamp']
        if 'distance' in packetRemoteData:
            remoteData[i].distance = packetRemoteData['distance']
        else:
            remoteData[i].distance = 0

        i += 1

    return tester.callTdoaEngineProcessPacket(
        anchorId,
        txAn_in_cl_An,
        rxAn_by_T_in_cl_T,
        seqNr,
        remoteCount,
        ctypes.byref(remoteData),
        ctypes.byref(position))


class GroundTruth:
    def __init__(self, stream):
        self.generator = yaml.load_all(stream, Loader=yaml.CLoader)

        self.current = next(self.generator)
        self.next = next(self.generator)

    def fast_forward_to(self, time):
        try:
            while time > self.current['rxSys']:
                self.current = self.next
                self.next = next(self.generator)
        except StopIteration:
            pass

    def x(self):
        return self.current['x']

    def y(self):
        return self.current['y']

    def z(self):
        return self.current['z']

    def position(self):
        return self.current['x'], self.current['y'], self.current['z']


def calculate_diff(gt, measurement):
    x, y, z = gt.position()

    dxa = x - measurement.anchorPosition[0].x
    dya = y - measurement.anchorPosition[0].y
    dza = z - measurement.anchorPosition[0].z

    dxb = x - measurement.anchorPosition[1].x
    dyb = y - measurement.anchorPosition[1].y
    dzb = z - measurement.anchorPosition[1].z

    da = math.sqrt(dxa * dxa + dya * dya + dza * dza)
    db = math.sqrt(dxb * dxb + dyb * dyb + dzb * dzb)

    return db - da


start_time = 0
with open("input/ground_truth.yaml", 'r') as gt_stream:
    gt = GroundTruth(gt_stream)

    with open("input/tdoa3_8_anchors.yaml", 'r') as stream:
        for packet in yaml.load_all(stream, Loader=yaml.CLoader):
            if not packet:
                continue

            time = packet['rxSys']
            if start_time == 0:
                start_time = time
            local_time = time - start_time

            gt.fast_forward_to(time)

            simulateClock(local_time)
            result = sendPacketToEngine(packet)
            if (result):
                measurement = (TdoaMeasurement)()
                tester.simulateGetEnqueuedResult(ctypes.byref(measurement))

                expected_diff = calculate_diff(gt, measurement)
                print(local_time, measurement.distanceDiff, expected_diff,
                      measurement.distanceDiff - expected_diff)
            else:
                print(local_time, "Nothing...")
