import os
import math

import ffmpeg
import numpy as np
from vispy import scene, app, io, geometry
from vispy.color import Color
from vispy.visuals import transforms
from vispy.scene.cameras import TurntableCamera

from .. import util as util


CF_MESH_PATH = os.path.join(os.path.dirname(__file__), "crazyflie2.obj.gz")
# Convert millimeters to meters, but make twice as big so easier to see.
MESHFILE_SCALE = 2.0 * 0.001
# The matrix that rotates the coordinates of the .obj file to agree with the
# Crazyflie's standard coordinate system. VisPy uses [row vector] * [matrix]
# (like DirectX), so this is the transpose of what we would expect.
UNROT_MESHFILE_TRANSPOSE = MESHFILE_SCALE * np.array([
    [-1,  0,  0],
    [ 0,  0,  1],
    [ 0, -1,  0],
])
ELLIPSOID_COLOR_OK = Color("#11FF22", alpha=0.1)
ELLIPSOID_COLOR_COLLISION  = Color("#FF0000", alpha=0.1)


class VisVispy:
    def __init__(self, show=True, resizable=True):
        self.canvas = scene.SceneCanvas(
            keys='interactive', size=(1024, 768), show=show, config=dict(samples=4), resizable=resizable
        )

        self.plane_color = 0.25 * np.ones((1, 3))
        self.bg_color = 0.9 * np.ones((1, 3))
        self.line_color = 0.7 * np.ones((1, 3))

        # Set up a viewbox to display the cube with interactive arcball
        self.view = self.canvas.central_widget.add_view()
        self.view.bgcolor = self.bg_color
        self.view.camera = TurntableCamera(
            fov=30.0, elevation=30.0, azimuth=90.0, center=(0.0, 0.0, 1.25)
        )
        self.cam_state = self.view.camera.get_state()

        # add a colored 3D axis for orientation
        axis = scene.visuals.XYZAxis(parent=self.view.scene)
        self.cfs = []
        self.led_color_cache = []

        ground = scene.visuals.Plane(
            32.0, 32.0, direction="+z", color=self.plane_color, parent=self.view.scene
        )

        # Lazy-constructed vispy objects and data for connectivity graph gfx.
        self.graph_edges = None
        self.graph_lines = None
        self.graph = None

        # Lazy-constructed vispy objects for collision ellipsoids.
        self.ellipsoids = None
        self.ellipsoid_radii = None

    def setGraph(self, edges):
        """Set edges of graph visualization - sequence of (i,j) tuples."""

        # Only allocate new memory if we need to.
        n_edges = len(edges)
        if self.graph_edges is None or n_edges != len(self.graph_edges):
            self.graph_lines = np.zeros((2 * n_edges, 3))
        self.graph_edges = edges

        # Lazily construct VisPy object for graph.
        if self.graph is None:
            self.graph = scene.visuals.Line(
                parent=self.view.scene,
                color=self.line_color,
                pos=self.graph_lines,
                connect="segments",
                method="gl",
                antialias=True,
            )

    def showEllipsoids(self, radii):
        self.ellipsoid_radii = np.array(radii)

    def update(self, t, crazyflies):
        if len(self.cfs) == 0:
            verts, faces, normals, nothin = io.read_mesh(CF_MESH_PATH)
            for i, cf in enumerate(crazyflies):
                color = cf.ledRGB
                mesh = scene.visuals.Mesh(
                    parent=self.view.scene,
                    vertices=verts,
                    faces=faces,
                    color=color,
                    shading="smooth",
                )
                mesh.light_dir = (0.1, 0.1, 1.0)
                mesh.shininess = 0.01
                mesh.ambient_light_color = [0.5] * 3
                mesh.transform = transforms.MatrixTransform()
                self.cfs.append(mesh)
                self.led_color_cache.append(color)

        if self.ellipsoid_radii is not None and self.ellipsoids is None:
            sphere_mesh = geometry.create_sphere(radius=1.0)
            self.ellipsoids = [
                scene.visuals.Mesh(
                    parent=self.view.scene,
                    meshdata=sphere_mesh,
                    color=ELLIPSOID_COLOR_OK,
                    shading="smooth",
                )
                for _ in self.cfs
            ]
            for ell in self.ellipsoids:
                ell.light_dir = (0.1, 0.1, 1.0)
                ell.shininess = 0.0
                ell.ambient_light_color = [0.5] * 3
                ell.transform = transforms.MatrixTransform()

        positions = np.stack([cf.position() for cf in crazyflies])

        for i in range(0, len(self.cfs)):
            R_state = crazyflies[i].rotBodyToWorld()
            # Recall VisPy uses [row vector] * [matrix]!!
            T = np.eye(4)
            T[:3, :3] = np.dot(UNROT_MESHFILE_TRANSPOSE, R_state.T)
            T[3, :3] = positions[i]
            self.cfs[i].transform = transforms.MatrixTransform(T)
            # vispy does not do this check
            color = crazyflies[i].ledRGB
            if color != self.led_color_cache[i]:
                self.led_color_cache[i] = color
                self.cfs[i].color = color  # sets dirty flag

        # Update graph line segments to match new Crazyflie positions.
        if self.graph is not None:
            for k, (i, j) in enumerate(self.graph_edges):
                self.graph_lines[2 * k, :] = positions[i]
                self.graph_lines[2 * k + 1, :] = positions[j]
            self.graph.set_data(self.graph_lines)

        # Update collsiion ellipsoids.
        if self.ellipsoids is not None:
            colliding = util.check_ellipsoid_collisions(positions, self.ellipsoid_radii)
            for i, pos in enumerate(positions):
                ell = self.ellipsoids[i]
                tf = ell.transform
                tf.reset()
                tf.scale(self.ellipsoid_radii)
                tf.translate(pos)
                new_color = ELLIPSOID_COLOR_COLLISION if colliding[i] else ELLIPSOID_COLOR_OK
                if not (new_color == ell.color):  # vispy Color lacks != override.
                    ell.color = new_color

        self.canvas.app.process_events()

    def render(self):
        frame = self.canvas.render()
        # Do not render alpha channel - we always use rgb24 format.
        if frame.shape[2] == 4:
            frame = frame[:, :, :3]
        return frame
