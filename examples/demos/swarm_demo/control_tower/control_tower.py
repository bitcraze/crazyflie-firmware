#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2019 Bitcraze AB
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.

import time
import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
import statistics
import sys
import threading
import math

import zmq

class TrafficController:
    CS_DISCONNECTED = 0
    CS_CONNECTING = 1
    CS_CONNECTED = 2

    STATE_UNKNOWN = -1
    STATE_IDLE = 0
    STATE_WAIT_FOR_POSITION_LOCK = 1
    STATE_WAIT_FOR_TAKE_OFF = 2  # Charging
    STATE_TAKING_OFF = 3
    STATE_HOVERING = 4
    STATE_WAITING_TO_GO_TO_INITIAL_POSITION = 5
    STATE_GOING_TO_INITIAL_POSITION = 6
    STATE_RUNNING_TRAJECTORY = 7
    STATE_GOING_TO_PAD = 8
    STATE_WAITING_AT_PAD = 9
    STATE_LANDING = 10
    STATE_CHECK_CHARGING = 11
    STATE_REPOSITION_ON_PAD = 12
    STATE_CRASHED = 13

    NO_PROGRESS = -1000.0

    PRE_STATE_TIMEOUT = 3

    def __init__(self, uri):
        self.uri = uri
        self.stay_alive = True
        self.reset_internal()
        self.connection_thread = threading.Thread(target=self.process)
        self.connection_thread.start()

    def reset_internal(self):
        self.connection_state = self.CS_DISCONNECTED
        self._cf = None
        self._log_conf = None
        self.copter_state = self.STATE_UNKNOWN
        self.vbat = -1.0
        self._time_for_next_connection_attempt = 0
        self.traj_cycles = None
        self.est_x = 0.0
        self.est_y = 0.0
        self.est_z = 0.0
        self.up_time_ms = 0
        self.flight_time_ms = 0

        # Pre states are used to prevent multiple calls to a copter
        # when waiting for the remote state to change
        self._pre_state_taking_off_end_time = 0
        self._pre_state_going_to_initial_position_end_time = 0

    def _pre_state_taking_off(self):
        return self._pre_state_taking_off_end_time > time.time()

    def _pre_state_going_to_initial_position(self):
        return self._pre_state_going_to_initial_position_end_time > time.time()

    def is_connected(self):
        return self.connection_state == self.CS_CONNECTED

    def has_found_position(self):
        return self.copter_state > self.STATE_WAIT_FOR_POSITION_LOCK

    def is_taking_off(self):
        return self.copter_state == self.STATE_TAKING_OFF or self._pre_state_taking_off()

    def is_ready_for_flight(self):
        return self.copter_state == self.STATE_HOVERING and not self._pre_state_going_to_initial_position()

    def is_flying(self):
        return self.copter_state == self.STATE_RUNNING_TRAJECTORY or \
               self.copter_state == self.STATE_WAITING_TO_GO_TO_INITIAL_POSITION or \
               self.copter_state == self.STATE_GOING_TO_INITIAL_POSITION or \
               self._pre_state_going_to_initial_position()

    def is_landing(self):
        return self.copter_state == self.STATE_GOING_TO_PAD or \
               self.copter_state == self.STATE_WAITING_AT_PAD or \
               self.copter_state == self.STATE_LANDING or \
               self.copter_state == self.STATE_CHECK_CHARGING or \
               self.copter_state == self.STATE_REPOSITION_ON_PAD

    def is_charging(self):
        return self.copter_state == self.STATE_WAIT_FOR_TAKE_OFF and not self._pre_state_taking_off()

    def is_crashed(self):
        return self.copter_state == self.STATE_CRASHED

    def take_off(self):
        if self.is_charging():
            if self._cf:
                self._pre_state_taking_off_end_time = time.time() + self.PRE_STATE_TIMEOUT
                self._cf.param.set_value('app.takeoff', 1)

    def start_trajectory(self, trajectory_delay, offset_x=0.0, offset_y=0.0, offset_z=0.0):
        if self.is_ready_for_flight():
            if self._cf:
                self._cf.param.set_value('app.offsx', offset_x)
                self._cf.param.set_value('app.offsy', offset_y)
                self._cf.param.set_value('app.offsz', offset_z)

                self._pre_state_going_to_initial_position_end_time = time.time() + self.PRE_STATE_TIMEOUT
                self._cf.param.set_value('app.start', trajectory_delay)

    def force_land(self):
        if self.connection_state == self.CS_CONNECTED:
            self._cf.param.set_value('app.stop', 1)

    def set_trajectory_count(self, count):
        if self.connection_state == self.CS_CONNECTED:
            self._cf.param.set_value('app.trajcount', count)

    def get_charge_level(self):
        return self.vbat

    def is_charged_for_flight(self):
        return self.vbat > 4.10

    def get_traj_cycles(self):
        return self.traj_cycles

    def process(self):
        while self.stay_alive:
            if self.connection_state == self.CS_DISCONNECTED:
                if time.time() > self._time_for_next_connection_attempt:
                    self._connect()

            time.sleep(1)
        self._cf.close_link()

    def _connected(self, link_uri):
        self.connection_state = self.CS_CONNECTED
        print('Connected to %s' % link_uri)

        self.set_trajectory_count(2)
        self._setup_logging()

    def _connection_failed(self, link_uri, msg):
        print('Connection to %s failed: %s' % (link_uri, msg))
        self._set_disconnected(5)

    def _connection_lost(self, link_uri, msg):
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        print('Disconnected from %s' % link_uri)
        self._set_disconnected()

    def _set_disconnected(self, hold_back_time=5):
        self.reset_internal()
        self._time_for_next_connection_attempt = time.time() + hold_back_time

    def _connect(self):
        if self.connection_state != self.CS_DISCONNECTED:
            print("Can only connect when disconnected")
            return

        self.connection_state = self.CS_CONNECTING

        self._cf = Crazyflie(rw_cache='./cache')

        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        print("Connecting to " + self.uri)
        self._cf.open_link(self.uri)

    def _setup_logging(self):
        # print("Setting up logging")
        self._log_conf = LogConfig(name='Tower', period_in_ms=100)
        self._log_conf.add_variable('app.state', 'uint8_t')
        self._log_conf.add_variable('app.prgr', 'float')
        self._log_conf.add_variable('app.uptime', 'uint32_t')
        self._log_conf.add_variable('app.flighttime', 'uint32_t')
        self._log_conf.add_variable('pm.vbat', 'float')
        self._log_conf.add_variable('stateEstimate.x', 'float')
        self._log_conf.add_variable('stateEstimate.y', 'float')

        self._cf.log.add_config(self._log_conf)
        self._log_conf.data_received_cb.add_callback(self._log_data)
        self._log_conf.start()

    def _log_data(self, timestamp, data, logconf):
        self.copter_state = data['app.state']

        if self.copter_state != self.STATE_WAIT_FOR_TAKE_OFF:
            self._pre_state_taking_off_end_time = 0

        if self.copter_state != self.STATE_HOVERING:
            self._pre_state_going_to_initial_position_end_time = 0

        self.vbat = data['pm.vbat']

        self.up_time_ms = data['app.uptime']
        self.flight_time_ms = data['app.flighttime']

        self.traj_cycles = data['app.prgr']
        if self.traj_cycles <= self.NO_PROGRESS:
            self.traj_cycles = None

        self.est_x = data['stateEstimate.x']
        self.est_y = data['stateEstimate.y']

    def dump(self):
        print("***", self.uri)
        print("  Connection state:", self.connection_state)
        print("  Copter state:", self.copter_state)
        print("  Bat:", self.vbat)
        print("  Up time:", self.up_time_ms / 1000)
        print("  Flight time:", self.flight_time_ms / 1000)
        print("  _pre_state_taking_off:", self._pre_state_taking_off())
        print("  _pre_state_going_to_initial_position:", self._pre_state_going_to_initial_position())

    def terminate(self):
        self.stay_alive = False


class TowerBase:
    def __init__(self, uris, report_socket=None):
        self.controllers = []
        self._uris = uris
        for uri in uris:
            self.controllers.append(TrafficController(uri))
        self.report_socket = report_socket

    def connected_count(self):
        count = 0
        for controller in self.controllers:
            if controller.is_connected():
                count += 1
        return count

    def flying_count(self):
        count = 0
        for controller in self.controllers:
            if controller.is_flying():
                count += 1
        return count

    def find_best_controllers(self):
        too_low_battery = []

        charging_controllers = []
        for controller in self.controllers:
            if controller.is_charging():
                charge = controller.get_charge_level()
                if controller.is_charged_for_flight():
                    charging_controllers.append((controller, charge))
                else:
                    too_low_battery.append(
                        "{} ({:.2f}V)".format(controller.uri, charge))

        if len(too_low_battery) > 0:
            print("Ready but must charge:", too_low_battery)

        charging_controllers.sort(key=lambda d: d[1], reverse=True)

        return list(map(lambda d: d[0], charging_controllers))

    def land_all(self):
        for controller in self.controllers:
            controller.force_land()

    def dump_state(self):
        print('Waiting for connections...')
        end_time = time.time() + 40
        while time.time() < end_time and self.connected_count() < len(
                self._uris):
            time.sleep(1)

        print("Dumping state")
        print()
        for controller in self.controllers:
            controller.dump()
            controller.terminate()

    def send_report(self):
        if self.report_socket is None:
            return

        for i, controller in enumerate(self.controllers):
            state = "idle"
            if not controller.is_connected():
                state = "disconnected"
            elif controller.is_crashed():
                state = "crashed"
            elif controller.is_flying():
                state = "flying"
            elif controller.is_taking_off():
                state = "hovering"
            elif controller.is_landing():
                state = "landing"
            elif controller.is_charged_for_flight():
                state = "ready"
            elif controller.is_charging():
                state = "charging"

            try:
                report = {
                    'id': i,
                    'state': state,
                    'battery': controller.get_charge_level(),
                    'uptime': controller.up_time_ms,
                    'flighttime': controller.flight_time_ms,
                }
                self.report_socket.send_json(report, zmq.NOBLOCK)
            except Exception:
                pass


class Tower(TowerBase):
    def __init__(self, uris, report_socket=None):
        TowerBase.__init__(self, uris, report_socket)

    def fly(self, wanted):
         # Wait for all CF to connect (to avoid race)
        time.sleep(10)

        while True:
            # print()
            if wanted:
                currently_flying = self.flying_count()
                missing = wanted - currently_flying
                if missing > 0:
                    print("Want", missing, "more copters")
                    self.prepare_copters(missing)
                    self.start_copters(missing, wanted)
            else:
                self.land_all()

            self.send_report()

            time.sleep(0.2)

    def prepare_copters(self, count):
        prepared_count = 0
        for controller in self.controllers:
            if controller.is_taking_off() or controller.is_ready_for_flight():
                prepared_count += 1

        missing = count - prepared_count
        new_prepared_count = 0
        if missing > 0:
            print("Trying to prepare", missing, "copter(s)")
            best_controllers = self.find_best_controllers()
            for best_controller in best_controllers[:missing]:
                if best_controller:
                    print("Preparing " + best_controller.uri)
                    new_prepared_count += 1
                    best_controller.take_off()
            print("Prepared", new_prepared_count, "copter(s)")

    def start_copters(self, count, total):
        unused_slot_times = self.find_unused_slot_times(total)
        # print("Unused slot times:", unused_slot_times)

        slot_index = 0
        for controller in self.controllers:
            if controller.is_ready_for_flight():
                if slot_index < count and slot_index < len(unused_slot_times):
                    trajectory_delay = 1.0 - unused_slot_times[slot_index]
                    if trajectory_delay == 1.0:
                        trajectory_delay = 0.0
                    print("Starting prepared copter", controller.uri,
                          'with a delay of', trajectory_delay)
                    controller.start_trajectory(trajectory_delay, offset_z=0.25)
                    slot_index += 1
                else:
                    return

    def find_unused_slot_times(self, total_slots):
        # Times are measured in trajectory cycles
        start_times = []
        for controller in self.controllers:
            if controller.is_flying():
                start_time = controller.get_traj_cycles()

                # If a is flying but has not updated the start time yes we do
                # not have enough information to calculate empty slots.
                # Return no unused slots for now
                if start_time is None:
                    # print("Start time is unknown, hold back")
                    return []

                start_times.append(start_time)

        # print("Used start times", start_times)
        return self.crunch_slot_times(start_times, total_slots)

    def crunch_slot_times(self, start_times, total_slots):
        # Start times may be multiple cycles ago, remove integer parts
        start_time_fractions = list(map(lambda t: t % 1.0, start_times))

        # Find the average offset
        offsets = list(
            map(lambda t: (t * total_slots) % 1.0, start_time_fractions))
        offset = 0.0
        if len(start_times) > 0:
            offset = statistics.mean(offsets) / total_slots

        adjusted_start_times = list(
            map(lambda t: t - offset, start_time_fractions))

        closest_slots = list(
            map(lambda t: round(t * total_slots), adjusted_start_times))
        unused_slots = list(
            filter(lambda s: s not in closest_slots, range(total_slots)))

        unsued_slot_times = list(
            map(lambda s: offset + s / total_slots, unused_slots))
        return unsued_slot_times


class SyncTower(TowerBase):
    def __init__(self, uris, report_socket=None):
        TowerBase.__init__(self, uris, report_socket)
        self.spacing = 0.40
        self.line_orientation = math.radians(40)

        master_offset = [0, 0]
        self._start_position = [
            [   0 + master_offset[0],    0 + master_offset[1], 0],
            [   0 + master_offset[0],  0.5 + master_offset[1], 0],
            [   0 + master_offset[0], -0.5 + master_offset[1], 0],
            [ 0.5 + master_offset[0],    0 + master_offset[1], 0],
            [-0.5 + master_offset[0],    0 + master_offset[1], 0],
            [ 0.5 + master_offset[0],  0.5 + master_offset[1], 0],
            [-0.5 + master_offset[0], -0.5 + master_offset[1], 0],
            [-0.5 + master_offset[0],  0.5 + master_offset[1], 0],
            [ 0.5 + master_offset[0], -0.5 + master_offset[1], 0]
        ]


    def fly(self, wanted):
        while True:
            if wanted:
                best = self.find_best_controllers()
                ready = list(
                    filter(lambda ctrlr: ctrlr.has_found_position(), best))
                found_count = len(ready)

                if found_count >= wanted:
                    self.start_line(wanted, ready)
                    print("Started, I'm done")
                    sys.exit(0)
                else:
                    print('Can only find ', found_count,
                          'copter(s) that are charged and ready')
            else:
                self.land_all()

            self.send_report()

            time.sleep(0.2)

    def start_line(self, wanted, best):
        self.prepare_copters(wanted, best)

        while not self.start_copters(wanted, best):
            time.sleep(1)

    def prepare_copters(self, count, best_controllers):
        prepared_count = 0
        for controller in self.controllers:
            if controller.is_taking_off() or controller.is_ready_for_flight():
                prepared_count += 1

        missing = count - prepared_count
        new_prepared_count = 0
        if missing > 0:
            print("Trying to prepare", missing, "copter(s)")
            for best_controller in best_controllers[:missing]:
                if best_controller:
                    print("Preparing " + best_controller.uri)
                    new_prepared_count += 1
                    best_controller.take_off()
            print("Prepared", new_prepared_count, "copter(s)")

    def start_copters(self, wanted, best):
        ready = []
        for controller in best:
            if controller.is_ready_for_flight():
                ready.append(controller)

        if len(ready) >= wanted:
            ready_positions = []
            for controller in ready:
                ready_positions.append([controller.est_x, controller.est_y, controller.est_z])

            offsets = self.get_start_offsets(ready_positions, self._start_position[:wanted])

            index = 0
            for controller in ready:
                offset_x = offsets[index][0]
                offset_y = offsets[index][1]
                offset_z = offsets[index][2]
                controller.start_trajectory(0.0, offset_x=offset_x,
                                            offset_y=offset_y,
                                            offset_z=offset_z)
                index += 1

            return True
        else:
            return False

    def calculate_distance(self, p1, p2):
        diff = [0,0,0]
        diff[0] = p1[0]-p2[0]
        diff[1] = p1[1]-p2[1]
        diff[2] = p1[2]-p2[2]
        return math.sqrt( (diff[0]*diff[0]) + (diff[1]*diff[1]) + (diff[2]*diff[2]) )

    def find_closest_target(self, start_position, targets_position):
        min_distance = None
        closest_index = 0
        for index, target in enumerate(targets_position):
            dist = self.calculate_distance(start_position, target)
            if min_distance is None or dist < min_distance:
                min_distance = dist
                closest_index = index
        
        return targets_position[closest_index]

    def get_start_offsets(self, start_positions, targets_positions):
        offsets = []
        target_used = [False,] * len(targets_positions)
        for start in start_positions:
            candidate_targets = []
            for i in range(len(targets_positions)):
                if not target_used[i]:
                    candidate_targets.append(targets_positions[i])
            
            closest_target = self.find_closest_target(start, candidate_targets)
            target_used[targets_positions.index(closest_target)] = True
            offsets.append(closest_target)
        
        return offsets


uris = [
    'radio://0/10/2M/E7E7E7E701',
    'radio://0/10/2M/E7E7E7E702',
    'radio://0/10/2M/E7E7E7E703',
    'radio://0/10/2M/E7E7E7E704',
    'radio://0/10/2M/E7E7E7E705',
    'radio://0/10/2M/E7E7E7E706',
    'radio://0/10/2M/E7E7E7E707',
    'radio://0/10/2M/E7E7E7E708',
    'radio://0/10/2M/E7E7E7E709'
]

count = 1
mode = 'normal'

if len(sys.argv) > 1:
    if sys.argv[1] == 'd':
        mode = 'dump'
    else:
        count = int(sys.argv[1])

if len(sys.argv) > 2:
    if sys.argv[2] == 's':
        mode = 'synch'

context = zmq.Context()
socket = context.socket(zmq.PUSH)
socket.bind("tcp://*:5555")

cflib.crtp.init_drivers(enable_debug_driver=False)

print('Starting tower with', count, 'Crazyflie(s)')
if mode == 'synch':
    print('Flying with synchronized trajectories')
    tower = SyncTower(uris, socket)
else:
    print('Flying with interleaved trajectories')
    tower = Tower(uris, socket)

if not mode == 'dump':
    tower.fly(count)
else:
    tower.dump_state()
