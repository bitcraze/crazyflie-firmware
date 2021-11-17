#!/usr/bin/env python

import numpy as np
import cffirmware

def test_that_vec_is_converted_to_numpy_array():
    # Fixture
    v_cf = cffirmware.mkvec(1, 2, 3)

    # Test
    actual = np.array(v_cf)

    # Assert
    expected = np.array([1, 2, 3])
    assert np.allclose(expected, actual)
