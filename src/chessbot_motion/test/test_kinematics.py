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


def test_far_edge_of_workspace_solves_cold(kin):
    # 30 cm forward of the pan axis at grasp height is reachable with a near-vertical
    # tool; a zero seed alone does not converge there.
    target = np.array([0.0388 + 0.30, 0.0, 0.015])
    result = kin.solve_with_restarts(target, np.array([0.0, 0.0, -1.0]), np.zeros(len(JOINTS)))
    assert result.success


def test_jaw_opening_direction_is_honoured(kin):
    # Vertical grasp over the board with the jaws opening along a diagonal, both ways.
    target = np.array([0.0388 + 0.195, 0.03, 0.03])
    down = np.array([0.0, 0.0, -1.0])
    # The wrist roll's limits rule out some yaws at a given spot, so require most diagonals,
    # and the right roll whenever a solution is reported.
    solved = 0
    for yaw in np.radians([45.0, 135.0, -45.0, -135.0]):
        opening = np.array([np.cos(yaw), np.sin(yaw), 0.0])
        result = kin.solve_with_restarts(target, down, np.zeros(len(JOINTS)), target_opening=opening)
        if result.success:
            solved += 1
            _, approach, achieved = kin.forward_full(result.q)
            assert np.degrees(kin._roll_error(achieved, approach, opening)) < 5.0
    assert solved >= 3
