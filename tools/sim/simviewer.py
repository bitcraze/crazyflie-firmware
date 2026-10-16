#!/usr/bin/env python3
# /// script
# requires-python = ">=3.10"
# dependencies = [
#     "vispy~=0.16",
#     "PyQt6~=6.7",
#     "numpy",
# ]
# ///
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2026 Bitcraze AB
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, in version 3.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program. If not, see <http://www.gnu.org/licenses/>.
"""
Simviewer: shows all Simmyflies on this host in one 3D view, using only
their beacons. It never connects to a Simmyflie over CRTP.

Usage:
    uv run tools/sim/simviewer.py [--beacon=<group>:<port>]
"""

import argparse
import ipaddress
import math
import socket
import struct
import sys
import time
from dataclasses import dataclass

import numpy as np

DEFAULT_BEACON = "239.255.50.0:49999"

LOST_TIMEOUT_S = 1.0
REMOVE_TIMEOUT_S = 10.0
RESTART_THRESHOLD_MS = 1000
REDRAW_INTERVAL_MS = 20

# Beacon wire format, version 1. See the beacon design specification.
BEACON_V1_FORMAT = "<BBHI11f"
BEACON_V1_SIZE = struct.calcsize(BEACON_V1_FORMAT)

STATUS_AT_REST = 0
STATUS_FLYING = 1
STATUS_CRASHED = 2
STATUS_UNKNOWN = -1


# ---------------------------------------------------------------------------
# Receiver: decoding and tracks. Kept free of Qt and vispy.
# ---------------------------------------------------------------------------

@dataclass
class Beacon:
    status: int
    udp_port: int
    sim_time_ms: int
    position: tuple
    quaternion: tuple  # w, x, y, z, body to world
    rotor_speeds: tuple


def decode_beacon(data):
    """Return a Beacon, or None if the datagram is not a known beacon."""
    if len(data) < 1:
        return None
    version = data[0]
    if version == 1 and len(data) == BEACON_V1_SIZE:
        fields = struct.unpack(BEACON_V1_FORMAT, data)
        status = fields[1]
        if status not in (STATUS_AT_REST, STATUS_FLYING, STATUS_CRASHED):
            status = STATUS_UNKNOWN
        return Beacon(
            status=status,
            udp_port=fields[2],
            sim_time_ms=fields[3],
            position=fields[4:7],
            quaternion=fields[7:11],
            rotor_speeds=fields[11:15],
        )
    return None


@dataclass
class Track:
    beacon: Beacon
    last_rx_time: float

    def is_lost(self, now):
        return now - self.last_rx_time >= LOST_TIMEOUT_S


def sim_time_age_ms(new_ms, old_ms):
    """How much older new_ms is than old_ms, in ms, taking u32 wrap into
    account. Zero or negative if new_ms is newer or equal."""
    diff = (new_ms - old_ms) & 0xFFFFFFFF
    if diff == 0:
        return 0
    if diff < 0x80000000:
        return -diff
    return 0x100000000 - diff


class Tracks:
    """One track per UDP port."""

    def __init__(self):
        self._tracks = {}

    def on_beacon(self, beacon, now):
        track = self._tracks.get(beacon.udp_port)
        if track is None:
            self._tracks[beacon.udp_port] = Track(beacon, now)
            return

        age_ms = sim_time_age_ms(beacon.sim_time_ms, track.beacon.sim_time_ms)
        if age_ms <= 0 and beacon.sim_time_ms != track.beacon.sim_time_ms:
            # Newer sim time
            accept = True
        elif age_ms >= RESTART_THRESHOLD_MS:
            # The Simmyflie has been restarted
            accept = True
        else:
            # Stale, duplicate or out of order
            accept = False

        if accept:
            track.beacon = beacon
            track.last_rx_time = now

    def remove_expired(self, now):
        expired = [port for port, track in self._tracks.items()
                   if now - track.last_rx_time >= REMOVE_TIMEOUT_S]
        for port in expired:
            del self._tracks[port]

    def items(self):
        return self._tracks.items()


def open_beacon_socket(group, port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    # Several simviewers on one host each get every beacon
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # Binding to the group, not INADDR_ANY, so that only this group is heard
    sock.bind((group, port))
    membership = socket.inet_aton(group) + socket.inet_aton("127.0.0.1")
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, membership)
    sock.setblocking(False)
    return sock


def parse_beacon_address(value):
    """Parse '<group>:<port>'. Raises argparse.ArgumentTypeError."""
    group, sep, port_str = value.rpartition(":")
    if not sep:
        raise argparse.ArgumentTypeError(
            f"invalid beacon address '{value}', expected <group>:<port>")
    try:
        address = ipaddress.IPv4Address(group)
    except ipaddress.AddressValueError:
        raise argparse.ArgumentTypeError(
            f"invalid beacon group '{group}', expected an IPv4 address")
    if not address.is_multicast:
        raise argparse.ArgumentTypeError(
            f"invalid beacon group '{group}', expected a multicast address "
            "(224.0.0.0/4)")
    try:
        port = int(port_str, 10)
    except ValueError:
        port = 0
    if not 1 <= port <= 65535:
        raise argparse.ArgumentTypeError(
            f"invalid beacon port '{port_str}', expected 1-65535")
    return str(address), port


# ---------------------------------------------------------------------------
# Scene
# ---------------------------------------------------------------------------

STATUS_COLORS = {
    STATUS_AT_REST: (0.6, 0.6, 0.6),
    STATUS_FLYING: (0.1, 0.8, 0.2),
    STATUS_CRASHED: (0.9, 0.1, 0.1),
    STATUS_UNKNOWN: (0.9, 0.1, 0.9),
}
LOST_ALPHA = 0.3
LOST_FADE = 0.5  # Blend towards white

# Body geometry, in m. About 10 cm across, front along +x.
ARM_LENGTH = 0.046  # Centre to motor
ARM_WIDTH = 0.008
ARM_THICKNESS = 0.004
ROTOR_RADIUS = 0.022
ROTOR_SEGMENTS = 24
LABEL_OFFSET_Z = 0.08


def status_color(status, lost):
    r, g, b = STATUS_COLORS.get(status, STATUS_COLORS[STATUS_UNKNOWN])
    if lost:
        r, g, b = (c + (1.0 - c) * LOST_FADE for c in (r, g, b))
        return (r, g, b, LOST_ALPHA)
    return (r, g, b, 1.0)


def _box(center, size):
    cx, cy, cz = center
    sx, sy, sz = (s / 2 for s in size)
    vertices = np.array([
        [cx + dx * sx, cy + dy * sy, cz + dz * sz]
        for dx in (-1, 1) for dy in (-1, 1) for dz in (-1, 1)])
    faces = np.array([
        [0, 1, 3], [0, 3, 2], [4, 6, 7], [4, 7, 5],
        [0, 4, 5], [0, 5, 1], [2, 3, 7], [2, 7, 6],
        [0, 2, 6], [0, 6, 4], [1, 5, 7], [1, 7, 3]])
    return vertices, faces


def _disc(center, radius, segments):
    cx, cy, cz = center
    angles = np.linspace(0, 2 * math.pi, segments, endpoint=False)
    rim = np.column_stack([cx + radius * np.cos(angles),
                           cy + radius * np.sin(angles),
                           np.full(segments, cz)])
    vertices = np.vstack([[cx, cy, cz], rim])
    faces = np.array([[0, 1 + i, 1 + (i + 1) % segments]
                      for i in range(segments)])
    return vertices, faces


def _merge(parts):
    vertices = []
    faces = []
    offset = 0
    for v, f in parts:
        vertices.append(v)
        faces.append(f + offset)
        offset += len(v)
    return np.vstack(vertices), np.vstack(faces)


def build_body_mesh():
    """An X-frame with four rotor discs and a marker at the front."""
    parts = []
    arm_span = 2 * ARM_LENGTH + ARM_WIDTH
    for angle in (45, -45):
        v, f = _box((0, 0, 0), (arm_span, ARM_WIDTH, ARM_THICKNESS))
        a = math.radians(angle)
        rot = np.array([[math.cos(a), -math.sin(a), 0],
                        [math.sin(a), math.cos(a), 0],
                        [0, 0, 1]])
        parts.append((v @ rot.T, f))
    for angle in (45, 135, 225, 315):
        a = math.radians(angle)
        center = (ARM_LENGTH * math.cos(a), ARM_LENGTH * math.sin(a),
                  ARM_THICKNESS)
        parts.append(_disc(center, ROTOR_RADIUS, ROTOR_SEGMENTS))
    # Front marker: a small block on the +x side of the centre
    parts.append(_box((0.014, 0, ARM_THICKNESS), (0.016, 0.006, 0.006)))
    # Centre plate
    parts.append(_box((0, 0, 0), (0.024, 0.016, 0.006)))
    return _merge(parts)


def quaternion_to_matrix(w, x, y, z):
    """Rotation matrix (column vector convention) from a unit quaternion."""
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-9:
        return np.eye(3)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def pose_matrix(position, quaternion):
    """4x4 matrix in vispy's row vector convention."""
    m = np.eye(4)
    m[:3, :3] = quaternion_to_matrix(*quaternion).T
    m[3, :3] = position
    return m


def main():
    parser = argparse.ArgumentParser(
        description="Show all Simmyflies on this host in one 3D view, "
                    "using only their beacons.")
    parser.add_argument(
        "--beacon", type=parse_beacon_address, default=DEFAULT_BEACON,
        metavar="<group>:<port>",
        help=f"beacon multicast group and port (default: {DEFAULT_BEACON})")
    args = parser.parse_args()
    group, port = args.beacon

    try:
        sock = open_beacon_socket(group, port)
    except OSError as e:
        print(f"simviewer: cannot listen on {group}:{port}: {e}",
              file=sys.stderr)
        sys.exit(1)

    # Imported here so that argument errors are reported without a display
    from PyQt6.QtCore import QSocketNotifier, QTimer
    from vispy import app, scene

    app.use_app("pyqt6")

    class SimmyflieVisual:
        def __init__(self, parent, body_vertices, body_faces, label):
            self.body = scene.visuals.Mesh(
                vertices=body_vertices, faces=body_faces, shading=None,
                parent=parent)
            self.body.transform = scene.transforms.MatrixTransform()
            self.label = scene.visuals.Text(
                label, font_size=14, anchor_x="left", anchor_y="bottom",
                parent=parent)
            self._color = None

        def update(self, track, lost):
            b = track.beacon
            self.body.transform.matrix = pose_matrix(b.position, b.quaternion)
            color = status_color(b.status, lost)
            if color != self._color:
                self._color = color
                self.body.color = color
                self.label.color = color
                gl_state = "translucent" if lost else "opaque"
                self.body.set_gl_state(gl_state, depth_test=True)
                self.label.order = 1 if lost else 0
            x, y, z = b.position
            self.label.pos = (x, y, z + LABEL_OFFSET_Z)

        def remove(self):
            self.body.parent = None
            self.label.parent = None

    class SimViewerCanvas(scene.SceneCanvas):
        def __init__(self, sock, beacon_text):
            super().__init__(keys="interactive", size=(1000, 750),
                             title=f"Simviewer - {beacon_text}",
                             bgcolor="black")
            self.unfreeze()
            self._sock = sock
            self._tracks = Tracks()
            self._visuals = {}
            self._body_vertices, self._body_faces = build_body_mesh()

            self._view = self.central_widget.add_view()
            self._view.camera = scene.TurntableCamera(
                fov=45, distance=5.0, elevation=25, azimuth=-60,
                center=(0, 0, 0))
            self._add_world()

            self._notifier = QSocketNotifier(
                sock.fileno(), QSocketNotifier.Type.Read)
            self._notifier.activated.connect(self._read_beacons)

            self._timer = QTimer()
            self._timer.timeout.connect(self._redraw)
            self._timer.start(REDRAW_INTERVAL_MS)
            self.freeze()

        def _add_world(self):
            extent = 5
            lines = []
            for i in range(-extent, extent + 1):
                lines += [(i, -extent, 0), (i, extent, 0),
                          (-extent, i, 0), (extent, i, 0)]
            scene.visuals.Line(
                pos=np.array(lines, dtype=np.float32), connect="segments",
                color=(0.3, 0.3, 0.3, 1.0), parent=self._view.scene)
            axis = scene.visuals.XYZAxis(width=2, parent=self._view.scene)
            axis.transform = scene.transforms.STTransform(scale=(0.5, 0.5, 0.5))

        def _read_beacons(self, *_):
            now = time.monotonic()
            while True:
                try:
                    data = self._sock.recv(1024)
                except BlockingIOError:
                    break
                except OSError:
                    break
                beacon = decode_beacon(data)
                if beacon is not None:
                    self._tracks.on_beacon(beacon, now)

        def _redraw(self):
            now = time.monotonic()
            self._tracks.remove_expired(now)
            live_ports = set()
            for port, track in self._tracks.items():
                live_ports.add(port)
                visual = self._visuals.get(port)
                if visual is None:
                    visual = SimmyflieVisual(
                        self._view.scene, self._body_vertices,
                        self._body_faces, str(port))
                    self._visuals[port] = visual
                visual.update(track, track.is_lost(now))
            for port in list(self._visuals):
                if port not in live_ports:
                    self._visuals.pop(port).remove()
            self.update()

    canvas = SimViewerCanvas(sock, f"{group}:{port}")
    canvas.show()
    app.run()


if __name__ == "__main__":
    main()
