from vispy import scene
from vispy.scene import XYZAxis, LinePlot, Node, Mesh, TurntableCamera, Markers
import numpy as np
from vispy.visuals.transforms import MatrixTransform
import math
import threading
import time
import logging
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie.mem import MemoryElement

logging.basicConfig(level=logging.ERROR)


class AnimCanvas(scene.SceneCanvas):
    def __init__(self):
        scene.SceneCanvas.__init__(self, keys='interactive', size=(800, 600),
                                   show=True)


class Visulizer:
    def __init__(self):
        self.terminate = False

        self.canvas = AnimCanvas()

        self.view = self.canvas.central_widget.add_view()
        self.view.bgcolor = '#ffffff'
        self.view.camera = TurntableCamera(fov=10.0, distance=40.0, up='+z',
                                           center=(0.0, 1.0, 0.0))

        self.scene = self.view.scene

        XYZAxis(parent=self.scene)

        self.base_stations = [
            {
                'origin': [-1.9584829807281494, 0.5422989726066589, 3.152726888656616],
                'mat': [
                    [0.7972149848937988, -0.004273999948054552, 0.6036810278892517],
                    [0.0, 0.9999750256538391, 0.007079999893903732],
                    [-0.6036959886550903, -0.005644999910145998, 0.7971950173377991]]
            },
            {
                'origin': [1.0623979568481445, -2.563488006591797, 3.1123669147491455],
                'mat': [
                    [0.018067000433802605, -0.9993360042572021, 0.03164700046181679],
                    [0.7612509727478027, 0.034269001334905624, 0.6475520133972168],
                    [-0.6482059955596924, 0.012392000295221806, 0.7613639831542969]]
            },
        ]

        self.angles = [
            [0.19170784950256348, 0.01734159141778946],
            [-0.12198804318904877, -0.3505938947200775],
        ]

        self.objs = [{}, {}]
        self.generate_scene()

    def generate_scene(self):
        p = [0, 0, 0]

        for i in range(2):
            objs = self.objs[i]

            if i == 0:
                objs['bs'] = self.marker(p, color='green')
            else:
                objs['bs'] = self.marker(p, color='red')

            objs['tripod'] = self.line(p, p, color='blue')
            objs['center_line'] = self.line(p, p, color="green")
            objs['beam'] = self.line(p, p, color="red")

    def update_scene(self):
        normal = [1, 0, 0]
        for i in range(2):
            objs = self.objs[i]

            bs = self.base_stations[i]
            origin = bs['origin']
            self.update_marker(objs['bs'], origin)
            self.update_line(objs['tripod'], [origin[0], origin[1], 0], origin)

            rot_bs = np.array(bs['mat'])
            center_line = np.dot(rot_bs, normal)
            self.update_line(objs['center_line'], origin,
                             np.add(center_line, origin))

            ################

            a = self.angles[i]

            a_x = a[0]
            a_y = a[1]

            beam_direction = np.array([1, math.tan(a_x), math.tan(a_y)])
            beam_line_local = 4 * beam_direction / np.linalg.norm(beam_direction)
            beam_line = np.dot(rot_bs, beam_line_local)

            self.update_line(objs['beam'], origin, np.add(beam_line, origin))

    def marker(self, pos, color='black'):
        return Markers(pos=np.array(pos, ndmin=2), face_color=color,
                       parent=self.scene)

    def update_marker(self, marker, pos):
        marker.set_data(np.array(pos, ndmin=2))

    def line(self, p1, p2, color='black'):
        return LinePlot([p1, p2], color=color, parent=self.scene,
                        marker_size=0)

    def update_line(self, line, p1, p2):
        line.set_data(data=[p1, p2], marker_size=0)

    def run(self):
        print('starting CF thread')
        self.t = threading.Thread(target=self.cf_handler)
        self.t.start()

        self.canvas.app.run()
        self.terminate = True

    def cf_handler(self):
        uri = "radio://0/30"

        lg_block = LogConfig(name='LH', period_in_ms=50)
        lg_block.add_variable('lighthouse.angle0x', 'float')
        lg_block.add_variable('lighthouse.angle0y', 'float')
        lg_block.add_variable('lighthouse.angle1x', 'float')
        lg_block.add_variable('lighthouse.angle1y', 'float')

        cf = Crazyflie(rw_cache='./cache')
        with SyncCrazyflie(uri, cf=cf) as scf:
            print("Getting LH geometry")
            mems = scf.cf.mem.get_mems(MemoryElement.TYPE_LH)
            mems[0].update(self._update_geometry)

            with SyncLogger(scf, lg_block) as logger:
                for log_entry in logger:
                    data = log_entry[1]

                    self.angles[0][0] = data['lighthouse.angle0x']
                    self.angles[0][1] = data['lighthouse.angle0y']
                    self.angles[1][0] = data['lighthouse.angle1x']
                    self.angles[1][1] = data['lighthouse.angle1y']

                    if self.terminate:
                        break

                    self.update_scene()

    def _update_geometry(self, mem):
        print("Received LH geometry")
        for i in range(2):
            self.base_stations[i]['origin'] = mem.geometry_data[i].origin
            self.base_stations[i]['mat'] = mem.geometry_data[i].rotation_matrix
            print(self.base_stations[i])

        self.update_scene()


# Initialize the low-level drivers (don't list the debug drivers)
cflib.crtp.init_drivers(enable_debug_driver=False)

viz = Visulizer()
viz.run()
