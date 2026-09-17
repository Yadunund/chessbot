#!/usr/bin/env python3
# Copyright 2026 Yadunund Vijay
# SPDX-License-Identifier: Apache-2.0
"""Compare the IK solver against a brute-force joint sweep for one target point."""

import itertools
import subprocess
import sys

import numpy as np
from ament_index_python.packages import get_package_share_directory

from chessbot_motion.kinematics import ArmKinematics

JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_flex_joint", "wrist_flex_joint", "wrist_roll_joint"]
target = np.array([float(v) for v in sys.argv[1:4]])
down = np.array([0.0, 0.0, -1.0])

urdf = subprocess.run(
    ["xacro", get_package_share_directory("chessbot_description") + "/urdf/so101.urdf.xacro"],
    check=True, capture_output=True, text=True,
).stdout
kin = ArmKinematics(urdf, JOINTS, "gripper_frame_link", "gripper_link")

result = kin.solve_with_restarts(target, down, np.zeros(5), restarts=60)
print(f"solver: success={result.success} pos_err={result.position_error*1000:.1f}mm tilt={np.degrees(result.approach_error_rad):.1f}deg")

pan = np.arctan2(target[1], target[0] - 0.0388)
best = None
for lift, elbow, wrist in itertools.product(*[np.linspace(kin.lower[i], kin.upper[i], 70) for i in (1, 2, 3)]):
    q = np.array([pan, lift, elbow, wrist, 0.0])
    p, a = kin.forward(q)
    err = np.linalg.norm(p - target)
    tilt = np.degrees(np.arccos(np.clip(a @ down, -1, 1)))
    if err < 0.01 and (best is None or tilt < best[1]):
        best = (err, tilt, q)
if best:
    print(f"sweep: within 1cm, best tilt {best[1]:.1f}deg at err {best[0]*1000:.1f}mm, q={np.round(best[2], 2)}")
    refined = kin.solve_with_restarts(target, down, best[2], restarts=0)
    print(f"solver seeded from sweep: success={refined.success} pos_err={refined.position_error*1000:.1f}mm tilt={np.degrees(refined.approach_error_rad):.1f}deg")
else:
    print("sweep: no configuration within 1 cm")
