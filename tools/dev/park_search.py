# Copyright 2026 Yadunund Vijay
# SPDX-License-Identifier: Apache-2.0
"""Search for a park pose that keeps the whole arm clear of the board and graveyards.

Samples arm configurations and scores each by the minimum distance between the
robot's collision meshes and a keep-out box over the board and both graveyards,
up to the tallest piece plus margin. Prefers compact poses near the base.

    pixi run python tools/dev/park_search.py
"""

from __future__ import annotations

import argparse
import math
import os
import sys

import coal
import numpy as np
import pinocchio as pin

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gripper_geometry import load  # noqa: E402

ARM = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_flex_joint", "wrist_flex_joint", "wrist_roll_joint"]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--urdf-url", default="http://127.0.0.1:8000/api/robot_description")
    parser.add_argument("--share", default=os.path.expanduser("~/chessbot/install/so_arm101_description/share"))
    parser.add_argument("--origin", type=float, nargs=3, default=[0.3488, -0.105, 0.0])
    parser.add_argument("--yaw", type=float, default=math.pi / 2)
    parser.add_argument("--square", type=float, default=0.02625)
    parser.add_argument("--keepout-height", type=float, default=0.042 + 0.02)
    parser.add_argument("--samples", type=int, default=20000)
    args = parser.parse_args()

    model, geom = load(args.urdf_url, args.share)
    data, gdata = model.createData(), pin.GeometryData(geom)
    s = args.square
    # Board plus graveyards (two squares out, two rows each side), in the board frame.
    bx_min, bx_max, by_min, by_max = -4 * s, 12 * s, 0.0, 8 * s
    c, sn = math.cos(args.yaw), math.sin(args.yaw)
    centre_b = np.array([(bx_min + bx_max) / 2, (by_min + by_max) / 2])
    centre = np.array(args.origin[:2]) + np.array([c * centre_b[0] - sn * centre_b[1], sn * centre_b[0] + c * centre_b[1]])
    box = coal.Box(bx_max - bx_min, by_max - by_min, args.keepout_height)
    rot = np.array([[c, -sn, 0], [sn, c, 0], [0, 0, 1]])
    box_pose = coal.Transform3s(rot, np.array([centre[0], centre[1], args.keepout_height / 2]))

    robot = [i for i, g in enumerate(geom.geometryObjects) if not g.name.startswith("base_link")]
    idx = [model.joints[model.getJointId(j)].idx_q for j in ARM]
    lower, upper = model.lowerPositionLimit[idx], model.upperPositionLimit[idx]
    rng = np.random.default_rng(0)
    tip = model.getFrameId("gripper_frame_link")
    results = []
    for q_arm in rng.uniform(lower, upper, size=(args.samples, len(ARM))):
        q = pin.neutral(model)
        q[idx] = q_arm
        pin.framesForwardKinematics(model, data, q)
        pin.updateGeometryPlacements(model, data, geom, gdata, q)
        clearance = min(
            coal.distance(geom.geometryObjects[i].geometry, gdata.oMg[i], box, box_pose, coal.DistanceRequest(), coal.DistanceResult())
            for i in robot
        )
        # No part below the table.
        lowest = min(gdata.oMg[i].translation[2] for i in robot)
        if lowest < 0.01:
            continue
        reach = float(np.linalg.norm(data.oMf[tip].translation[:2]))
        results.append((clearance, reach, q_arm))
    good = [r for r in results if r[0] > 0.03]
    good.sort(key=lambda r: (r[1], -r[0]))
    print(f"{len(good)} of {len(results)} samples clear the keep-out box by more than 30 mm")
    for clearance, reach, q_arm in good[:8]:
        print(f"clearance {clearance * 1000:5.1f} mm  tool reach {reach * 1000:5.0f} mm  joints [{', '.join(f'{v:.2f}' for v in q_arm)}]")


if __name__ == "__main__":
    main()
