# Copyright 2026 Yadunund Vijay
# SPDX-License-Identifier: Apache-2.0
"""IK round-trip tests against the real SO-101 description (skipped if not built)."""

import os
import subprocess

import numpy as np
import pytest

from chessbot_motion.kinematics import ArmKinematics

JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_flex_joint", "wrist_flex_joint", "wrist_roll_joint"]


@pytest.fixture(scope="module")
def kin():
    try:
        from ament_index_python.packages import get_package_share_directory

        share = get_package_share_directory("chessbot_description")
    except Exception:
        pytest.skip("chessbot_description not installed")
    urdf = subprocess.run(
        ["xacro", os.path.join(share, "urdf", "so101_sim.urdf.xacro")], check=True, capture_output=True, text=True
    ).stdout
    return ArmKinematics(urdf, JOINTS, "gripper_frame_link", "gripper_link")


def test_forward_then_inverse_round_trip(kin):
    rng = np.random.default_rng(0)
    solved = 0
    for _ in range(20):
        q = rng.uniform(kin.lower * 0.6, kin.upper * 0.6)
        position, approach = kin.forward(q)
        result = kin.solve_with_restarts(position, approach, np.zeros(len(JOINTS)))
        if result.success:
            solved += 1
            p2, a2 = kin.forward(result.q)
            assert np.linalg.norm(p2 - position) < 2e-3
    assert solved >= 18


def test_top_down_grasp_over_board_centre(kin):
    # Board centre from the reachability analysis: 19.5 cm from the pan axis.
    target = np.array([0.0388 + 0.195, 0.0, 0.03])
    result = kin.solve(target, np.array([0.0, 0.0, -1.0]), np.zeros(len(JOINTS)))
    assert result.success, (result.position_error, np.degrees(result.approach_error_rad))


def test_cartesian_descent(kin):
    start_target = np.array([0.0388 + 0.195, 0.0, 0.09])
    down = np.array([0.0, 0.0, -1.0])
    start = kin.solve(start_target, down, np.zeros(len(JOINTS)))
    assert start.success
    configs, fraction = kin.cartesian_path(start.q, [(start_target - [0, 0, 0.06], down)])
    assert fraction == 1.0
    assert len(configs) > 5
