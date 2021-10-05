"""No-op visualizer for real hardware runs, so script can be oblivious."""

class VisNull:
    def __init__(self):
        pass

    def setGraph(self, edges):
        pass

    def showEllipsoids(self, radii):
        pass

    def update(self, t, crazyflies):
        pass

    def render(self):
        return None
