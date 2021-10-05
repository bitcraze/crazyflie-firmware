import argparse
import atexit

import numpy as np


# Building the parser in a separate function allows sphinx-argparse to
# auto-generate the documentation for the command-line flags.
def build_argparser(parent_parsers=[]):
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        parents=parent_parsers
    )
    parser.add_argument("--sim", help="Run using simulation.", action="store_true")

    group = parser.add_argument_group("Simulation-only", "")
    group.add_argument("--vis", help="Visualization backend.", choices=['mpl', 'vispy', 'null'], default="mpl")
    group.add_argument("--dt", help="Duration of seconds between rendered visualization frames.", type=float, default=0.1)
    group.add_argument("--writecsv", help="Enable CSV output.", action="store_true")
    group.add_argument("--disturbance", help="Simulate Gaussian-distributed disturbance when using cmdVelocityWorld.", type=float, default=0.0)
    group.add_argument("--maxvel", help="Limit simulated velocity (meters/sec).", type=float, default=np.inf)
    group.add_argument("--video", help="Video output path.", type=str)

    return parser


class Crazyswarm:
    def __init__(self, crazyflies_yaml=None, parent_parser=None, args=None):
        if parent_parser is not None:
            parents = [parent_parser]
        else:
            parents = []
        parser = build_argparser(parents)
        if isinstance(args, str):
            args = args.split()
        args, unknown = parser.parse_known_args(args)

        if crazyflies_yaml is None:
            crazyflies_yaml = "../launch/crazyflies.yaml"
        if crazyflies_yaml.endswith(".yaml"):
            crazyflies_yaml = open(crazyflies_yaml, 'r').read()

        if args.sim:
            from .crazyflieSim import TimeHelper, CrazyflieServer
            self.timeHelper = TimeHelper(args.vis, args.dt, args.writecsv, disturbanceSize=args.disturbance, maxVel=args.maxvel, videopath=args.video)
            self.allcfs = CrazyflieServer(self.timeHelper, crazyflies_yaml)
            atexit.register(self.timeHelper._atexit)
        else:
            raise NotImplementedError("Sim only now!")
