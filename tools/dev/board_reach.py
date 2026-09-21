#!/usr/bin/env python3
# Copyright 2026 Yadunund Vijay
# SPDX-License-Identifier: Apache-2.0
"""Check every square and graveyard slot of the nominal calibration against the real IK.

For each point: the highest reachable hover height (as the pick/place skills
try them) and whether grasp height is reachable.
"""

import argparse
import math
import subprocess

import numpy as np
from ament_index_python.packages import get_package_share_directory

from chessbot_brain.board import square_name
from chessbot_brain.geometry import BoardGeometry
from chessbot_brain.capabilities import ArmConfig
from chessbot_brain.skills import TRANSIT_HEIGHTS
from chessbot_calibration.profile import CalibrationProfile
from chessbot_motion.kinematics import ArmKinematics

JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_flex_joint", "wrist_flex_joint", "wrist_roll_joint"]
DOWN = np.array([0.0, 0.0, -1.0])
# The brain reads this from its grasp_height parameter; this is the same default.
GRASP_HEIGHT = ArmConfig.grasp_height

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--near-edge", type=float, default=None, help="board near edge from the pan axis (m); default: calibration")
parser.add_argument("--max-tilt-deg", type=float, default=25.0)
parser.add_argument("--critical", action="store_true", help="only the corner and centre squares of the near and far ranks")
args = parser.parse_args()

urdf = subprocess.run(
    ["xacro", get_package_share_directory("chessbot_description") + "/urdf/so101.urdf.xacro"],
    check=True, capture_output=True, text=True,
).stdout
kin = ArmKinematics(urdf, JOINTS, "gripper_frame_link", "gripper_link", max_approach_tilt_rad=math.radians(args.max_tilt_deg))
b = CalibrationProfile().board
origin = list(b.origin_xyz)
if args.near_edge is not None:
    # Robot at the rank-8 edge: origin (a1 corner) is the far edge.
    origin[0] = 0.0388 + args.near_edge + 8 * b.square_size_m
geometry = BoardGeometry(b.frame_id, tuple(origin), b.yaw_rad, b.square_size_m)


def reach(xyz):
    hover = None
    for h in TRANSIT_HEIGHTS:
        if kin.solve_with_restarts(np.add(xyz, (0, 0, h)), DOWN, np.zeros(5)).success:
            hover = h
            break
    grasp = kin.solve_with_restarts(np.add(xyz, (0, 0, GRASP_HEIGHT)), DOWN, np.zeros(5))
    return hover, grasp.success, np.degrees(grasp.approach_error_rad)


failures = []
if args.critical:
    for sq in ("a8", "e8", "h8", "a2", "e2", "h2", "a1", "e1", "h1"):
        hover, grasp_ok, tilt = reach(geometry.square_centre(sq))
        if hover is None or not grasp_ok:
            failures.append(sq)
    print(f"near_edge={args.near_edge} max_tilt={args.max_tilt_deg}: unreachable {failures or 'none'}")
    raise SystemExit(0)
print("hover height (cm) per square, robot at the rank-8 edge; '!' = grasp unreachable, '--' = no hover")
for rank in range(8, 0, -1):
    row = []
    for f in "abcdefgh":
        sq = f"{f}{rank}"
        hover, grasp_ok, _ = reach(geometry.square_centre(sq))
        cell = "--" if hover is None else f"{hover * 100:.1f}".rstrip("0").rstrip(".")
        if not grasp_ok:
            cell += "!"
        if hover is None or not grasp_ok:
            failures.append(sq)
        row.append(f"{cell:>5}")
    print(f"{rank} " + "".join(row))
print("   " + "".join(f"{f:>5}" for f in "abcdefgh"))

for colour in ("white", "black"):
    for slot in range(16):
        hover, grasp_ok, _ = reach(geometry.graveyard_slot(colour, slot))
        if hover is None or not grasp_ok:
            failures.append(f"slot:{colour}/{slot}")
print("unreachable:", failures or "none")
