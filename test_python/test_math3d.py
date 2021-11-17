#!/usr/bin/env python

import numpy as np
import cffirmware

def test_conversion_to_numpy():
    v_cf = cffirmware.mkvec(1, 2, 3)
    v_np = np.array(v_cf)
    assert np.allclose(v_np, np.array([1,2,3]))
