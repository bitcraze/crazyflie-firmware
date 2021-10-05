#!/usr/bin/env python

import numpy as np
from pycrazyswarm import *

def test_yaml_string_load():

    crazyflies_yaml = """
    crazyflies:
    - channel: 100
      id: 1
      initialPosition: [1.0, 0.0, 0.0]
    - channel: 100
      id: 10
      initialPosition: [0.0, -1.0, 0.0]
    """

    swarm = Crazyswarm(crazyflies_yaml=crazyflies_yaml, args="--sim --vis null")
    timeHelper = swarm.timeHelper
    cfs = swarm.allcfs.crazyflies
    byId = swarm.allcfs.crazyfliesById

    assert len(cfs) == 2
    cf1 = byId[1]
    assert np.all(cf1.initialPosition == [1.0, 0.0, 0.0])

    cf10 = byId[10]
    assert np.all(cf10.initialPosition == [0.0, -1.0, 0.0])
