import warnings

from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Line3DCollection
import matplotlib.pyplot as plt
import numpy as np


class VisMatplotlib:
    def __init__(self):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlim([-5, 5])
        self.ax.set_ylim([-5, 5])
        self.ax.set_zlim([0, 3])
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")
        self.plot = None
        self.timeAnnotation = self.ax.annotate("Time", xy=(0, 0), xycoords='axes fraction', fontsize=12, ha='right', va='bottom')

        self.line_color = 0.3 * np.ones(3)

        # Lazy-constructed data for connectivity graph gfx.
        self.graph_edges = None
        self.graph_lines = None
        self.graph = None

    def setGraph(self, edges):
        """Set edges of graph visualization - sequence of (i,j) tuples."""

        # Only allocate new memory if we need to.
        n_edges = len(edges)
        if self.graph_edges is None or n_edges != len(self.graph_edges):
            self.graph_lines = np.zeros((n_edges, 2, 3))
        self.graph_edges = edges

        # Lazily construct Matplotlib object for graph.
        if self.graph is None:
            self.graph = Line3DCollection(self.graph_lines, edgecolor=self.line_color)
            self.ax.add_collection(self.graph)

    def showEllipsoids(self, radii):
        warnings.warn("showEllipsoids not implemented in Matplotlib visualizer.")

    def update(self, t, crazyflies):
        xs = []
        ys = []
        zs = []
        cs = []
        for cf in crazyflies:
            x, y, z = cf.position()
            color = cf.ledRGB
            xs.append(x)
            ys.append(y)
            zs.append(z)
            cs.append(color)

        if self.plot is None:
            self.plot = self.ax.scatter(xs, ys, zs, c=cs)
        else:
            # TODO: Don't use protected members.
            self.plot._offsets3d = (xs, ys, zs)
            self.plot.set_facecolors(cs)
            self.plot.set_edgecolors(cs)
            self.plot._facecolor3d = self.plot.get_facecolor()
            self.plot._edgecolor3d = self.plot.get_edgecolor()

        if self.graph is not None:
            # Update graph line segments to match new Crazyflie positions.
            for k, (i, j) in enumerate(self.graph_edges):
                self.graph_lines[k, 0, :] = xs[i], ys[i], zs[i]
                self.graph_lines[k, 1, :] = xs[j], ys[j], zs[j]
                self.graph.set_segments(self.graph_lines)

        self.timeAnnotation.set_text("{} s".format(t))
        plt.pause(0.0001)

    def render(self):
        warnings.warn("Rendering video not supported in VisMatplotlib yet.")
        return None
