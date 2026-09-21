# Copyright 2026 Yadunund Vijay
# SPDX-License-Identifier: Apache-2.0
"""The board measured by touching its four corners."""

import math

import numpy as np
import pytest
from chessbot_brain import calibration_flow as calib

SQUARE_M = 0.0254
YAW_RAD = math.radians(92.0)
ORIGIN = (0.30, -0.16, 0.025)


def touches(square_m=SQUARE_M, yaw_rad=YAW_RAD, origin=ORIGIN, jitter=0.0, seed=0):
    cos_yaw, sin_yaw = math.cos(yaw_rad), math.sin(yaw_rad)
    side = square_m * 8
    rng = np.random.default_rng(seed)
    points = {}
    for name, (bx, by) in zip(calib.CAMERA_TOUCH_POINTS,
                              [(0, 0), (side, 0), (side, side), (0, side)]):
        point = np.array(origin, float) + np.array(
            [cos_yaw * bx - sin_yaw * by, sin_yaw * bx + cos_yaw * by, 0.0])
        points[name] = point + (rng.normal(0, jitter, 3) if jitter else 0.0)
    return points


def test_board_from_touches_recovers_the_board():
    board = calib.board_from_touches(touches())
    assert board.square_size_m == pytest.approx(SQUARE_M)
    assert board.yaw_rad == pytest.approx(YAW_RAD)
    assert board.origin_xyz == pytest.approx(ORIGIN)
    assert board.squareness_m == pytest.approx(0.0, abs=1e-12)


def test_touch_height_is_the_board_surface():
    """The tips were on the squares, so nobody has to say how high the board is."""
    board = calib.board_from_touches(touches(origin=(0.3, -0.16, 0.041)))
    assert board.origin_xyz[2] == pytest.approx(0.041)


def test_hand_jitter_does_not_amplify():
    """Four corners over eight squares average a shaky hand down, not up."""
    errors = [abs(calib.board_from_touches(touches(jitter=0.001, seed=s)).square_size_m - SQUARE_M)
              for s in range(50)]
    assert max(errors) < 0.001


def test_an_incomplete_touch_set_measures_nothing():
    assert calib.board_from_touches({"a1": (0.0, 0.0, 0.0)}) is None
    assert calib.board_from_touches({}) is None
