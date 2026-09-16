#!/usr/bin/env python3
# Copyright 2026 Yadunund Vijay
# SPDX-License-Identifier: Apache-2.0
"""Ground truth for forward reach: random joint samples vs the IK solver."""

import subprocess

import numpy as np
from ament_index_python.packages import get_package_share_directory

from chessbot_motion.kinematics import ArmKinematics

JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_flex_joint", "wrist_flex_joint", "wrist_roll_joint"]
urdf = subprocess.run(
    ["xacro", get_package_share_directory("chessbot_description") + "/urdf/so101_sim.urdf.xacro"],
    check=True, capture_output=True, text=True,
).stdout
kin = ArmKinematics(urdf, JOINTS, "gripper_frame_link", "gripper_link", max_approach_tilt_rad=np.radians(45))
down = np.array([0.0, 0.0, -1.0])
rng = np.random.default_rng(1)
print("joint limits:", np.round(kin.lower, 2), np.round(kin.upper, 2))

best = {}
for _ in range(400000):
    q = rng.uniform(kin.lower, kin.upper)
    q[0] = 0.0
    p, a = kin.forward(q)
    if abs(p[2] - 0.015) > 0.004:
        continue
    r = p[0] - 0.0388  # signed: forward of the pan axis
    tilt = np.degrees(np.arccos(np.clip(a @ down, -1, 1)))
    for limit in (10, 25, 45, 90):
        if tilt <= limit and r > best.get(limit, (-1,))[0]:
            best[limit] = (r, q.copy(), p.copy(), tilt)
for limit, (r, q, p, tilt) in sorted(best.items()):
    seeded = kin.solve(p, down, q, approach_tolerance_rad=np.radians(limit))
    cold = kin.solve_with_restarts(p, down, np.zeros(5))
    print(f"tilt<={limit:>2}: max forward reach {r:.3f} m (tilt {tilt:.1f}) | IK seeded ok={seeded.success} | IK cold ok={cold.success}")
