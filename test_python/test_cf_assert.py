#!/usr/bin/env python

import cffirmware
import pytest

def test_that_assert_in_the_firmware_is_mapped_to_pytest_assert():
    # Fixture
    assert_message = 'Assert in firmware (triggered from python)'

    # Test
    # Assert
    with pytest.raises(SystemError):
        cffirmware.assertFail(assert_message, "Some file", 4711)
