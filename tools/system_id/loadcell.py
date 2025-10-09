"""Class to calibrate and read data from a ATI loadcell via EtherCAT."""

import time
import struct
import pysoem
import numpy as np


class Loadcell:
    def __init__(self, interface: str, slave_id: int = 0, verbose: bool = False):
        self._verbose = verbose

        self._ethercat_master = pysoem.Master()
        self._ethercat_master.open(interface)

        if self._ethercat_master.config_init() <= 0:
            raise RuntimeError("No EtherCAT slave found")
        self._ethercat_master.config_map()
        self._ethercat_master.config_dc()  # optional, for sync

        self._slave_id = slave_id
        self.slave = self._ethercat_master.slaves[self._slave_id]

        # Activate low pass filter
        control1 = struct.unpack("<I", self.slave.sdo_read(0x7010, 0x01, 4))[0]
        # print(bin(control1))
        FILTER_MASK = 0b1111 << 4
        control1 &= ~FILTER_MASK
        # control1 |= 0x8 << 4  # 0-8 per Table 4.1, 2=140 Hz, 8=2 Hz
        # print(bin(control1))
        self.slave.sdo_write(0x7010, 0x01, struct.pack("<I", control1))
        verify_raw = self.slave.sdo_read(0x7010, 0x01, 4)
        verify_value = struct.unpack("<I", verify_raw)[0]
        # print(
        #     f"Control Codes: 0x{verify_value:08X}, Filter bits: {(verify_value >> 4) & 0xF}"
        # )
        time.sleep(0.5)
        self._get_counts_per_force()

        self._interface = interface
        self._slave_id = slave_id

        self._offset = np.zeros(6)  # fx, fy, fz, tx, ty, tz
        self._counts_per_SI = np.array(
            [
                self._cp_force,
                self._cp_force,
                self._cp_force,
                self._cp_torque,
                self._cp_torque,
                self._cp_torque,
            ]
        )

    def _get_counts_per_force(self):
        force_unit_raw = self.slave.sdo_read(0x2040, 0x29, 4)
        torque_unit_raw = self.slave.sdo_read(0x2040, 0x2A, 4)
        if self._verbose:
            print(f"Force unit = {force_unit_raw}")
            print(f"Torque unit = {torque_unit_raw}")

        cp_force_raw = self.slave.sdo_read(0x2040, 0x31, 4)  # count per force [N]
        cp_torque_raw = self.slave.sdo_read(0x2040, 0x32, 4)  # count per torque [Nmm]
        # Note that force is returned as N, but torque as Nmm. However, we want
        # Nm, so we scale counts per torque by 1000
        self._cp_force = struct.unpack("<I", cp_force_raw)[0]
        self._cp_torque = struct.unpack("<I", cp_torque_raw)[0] * 1000
        if self._verbose:
            print(f"Counts/Force: {self._cp_force}, Counts/Torque: {self._cp_torque}")

    def calibrate(self):
        """Calibrate the offset of the loadcell."""
        if self._verbose:
            print("Calibrating loadcell. Don't apply any force.")
        self._offset = np.zeros(6)
        measurements = []
        for _ in range(100):
            measurements.append(self.read_data())
            time.sleep(0.01)  # wait for stable readings

        self._offset = np.mean(measurements, axis=0)

        if self._verbose:
            print(f"Calibration complete. Offset: {self._offset}")

    def read_data(self):
        """Read data from the loadcell."""
        self._ethercat_master.send_processdata()
        self._ethercat_master.receive_processdata(2000)

        status = struct.unpack("<1i", self.slave.sdo_read(0x6010, 0x0, 32))[0]
        if self._verbose or status != 0:
            print(f"status {status}")

        data = self.slave.input  # raw byte array
        # Parse Fx, Fy, Fz, Tx, Ty, Tz (6 × int32)
        fx, fy, fz, tx, ty, tz = struct.unpack("<6i", data[0:24])
        self._reading_raw = np.array([fx, fy, fz, tx, ty, tz])
        self._reading = self._reading_raw / self._counts_per_SI - self._offset

        if self._verbose:
            txt = f"Raw Fx: {fx}, Fy: {fy}, Fz: {fz}, "
            txt += f"Tx: {tx}, Ty: {ty}, Tz: {tz}"
            print(txt)

        if self._verbose:
            txt = f"Fx: {self._reading[0]:.3f}, Fy: {self._reading[1]:.3f}, Fz: {self._reading[2]:.3f}, "
            txt += f"Tx: {self._reading[3]:.3f}, Ty: {self._reading[4]:.3f}, Tz: {self._reading[5]:.3f}"
            print(txt)

        return self._reading

    @staticmethod
    def list_interfaces():
        """List available EtherCAT interfaces."""
        print("Interfaces found:", pysoem.find_adapters())

    @staticmethod
    def list_slaves(interface: str):
        """List EtherCAT slaves on the specified interface."""
        master = pysoem.Master()
        master.open(interface)
        slaves = master.config_init()
        print("Number of slaves found:", slaves)
        if slaves:
            for i, slave in enumerate(master.slaves):
                print(f"Slave {i}: {slave.name}")
        master.close()


# Short test (Needs to be executed as sudo)
if __name__ == "__main__":
    # Replace the name with your actual interface name. You can check that with
    # list_interfaces(). Note that PySOEM needs access to your ethernet ports.
    # For that, we need to give Python permission:
    # sudo setcap cap_net_raw,cap_net_admin+eip $(readlink -f $(which python3))
    # or run the script as root: sudo python loadcell.py
    interface_name = "enp0s31f6"  # enp0s31f6, enp121s0
    inf = False  # set this for infinite reading until killing the script

    Loadcell.list_interfaces()
    Loadcell.list_slaves(interface_name)

    loadcell = Loadcell(interface_name, verbose=False)
    loadcell.read_data()  # Read data from the loadcell
    loadcell.calibrate()  # Calibrate the loadcell

    readings = []
    readings_raw = []
    f = 100  # Hz
    while inf:
        reading = loadcell.read_data()
        readings.append(reading)
        if len(readings) >= 100:
            print(np.mean(readings, axis=0))
            readings = []
        time.sleep(1 / f)

    T = 300  # s
    N = T * f
    t = np.linspace(0, N / f, N)

    import csv

    # Save readings as CSV
    with open("loadcell_data.csv", "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        # Write header
        writer.writerow(
            [
                "t",
                "fx",
                "fy",
                "fz",
                "tx",
                "ty",
                "tz",
                "fx_raw",
                "fy_raw",
                "fz_raw",
                "tx_raw",
                "ty_raw",
                "tz_raw",
            ]
        )
        for i in range(N):
            loadcell.read_data()
            readings.append(loadcell._reading)
            readings_raw.append(loadcell._reading_raw)
            time.sleep(1 / f)
            row = [t[i]] + readings[i].tolist() + readings_raw[i].tolist()
            writer.writerow(row)
