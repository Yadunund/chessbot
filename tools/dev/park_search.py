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
    parser.add_argument("--pan", type=float, help="fix the base joint (e.g. 1.5708 to swing the arm to one side)")
    parser.add_argument("--prefer", choices=["compact", "low", "perched"], default="perched",
                        help="compact: tool near the base; low: lowest arm; perched: folded near the base, as low as possible, out of camera view")
    parser.add_argument("--max-spread", type=float, default=0.12, help="perched: furthest the arm may reach from the base axis, m")
    parser.add_argument("--camera", type=float, nargs=3, default=[0.234, 0.0, 0.51], help="overhead camera position")
    parser.add_argument("--camera-half-fov", type=float, nargs=2, default=[0.3248, 0.5774],
                        help="tan of half the field of view along base x and y")
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
    vertices = {i: np.asarray(geom.geometryObjects[i].geometry.vertices())[::20] for i in robot}
    # The shoulder only turns about the vertical and always sits at table level.
    moving = [i for i in robot if not geom.geometryObjects[i].name.startswith("shoulder_link")]
    results = []
    samples = rng.uniform(lower, upper, size=(args.samples, len(ARM)))
    if args.pan is not None:
        samples[:, 0] = args.pan
        samples[:, 4] = 0.0
    for q_arm in samples:
        q = pin.neutral(model)
        q[idx] = q_arm
        pin.framesForwardKinematics(model, data, q)
        pin.updateGeometryPlacements(model, data, geom, gdata, q)
        clearance = min(
            coal.distance(geom.geometryObjects[i].geometry, gdata.oMg[i], box, box_pose, coal.DistanceRequest(), coal.DistanceResult())
            for i in robot
        )
        # Heights of the actual mesh surfaces: nothing may touch the table.
        heights = np.concatenate([
            (gdata.oMg[i].rotation @ vertices[i].T)[2] + gdata.oMg[i].translation[2] for i in moving
        ])
        lowest, highest = float(heights.min()), float(heights.max())
        if lowest < 0.02:
            continue
        reach = float(np.linalg.norm(data.oMf[tip].translation[:2]))
        points = np.concatenate([(gdata.oMg[i].rotation @ vertices[i].T).T + gdata.oMg[i].translation for i in moving])
        spread = float(np.max(np.linalg.norm(points[:, :2], axis=1)))
        if args.prefer == "perched":
            cx, cy, cz = args.camera
            depth = np.maximum(cz - points[:, 2], 1e-3)
            seen = (np.abs(points[:, 0] - cx) < depth * args.camera_half_fov[0]) & (np.abs(points[:, 1] - cy) < depth * args.camera_half_fov[1])
            if seen.any():
                continue
        if args.prefer == "perched" and spread > args.max_spread:
            continue
        score = {"compact": reach, "low": highest, "perched": highest}[args.prefer]
        results.append((clearance, score, q_arm))
    good = [r for r in results if r[0] > 0.03]
    good.sort(key=lambda r: (r[1], -r[0]))
    print(f"{len(good)} of {len(results)} samples clear the keep-out box by more than 30 mm")
    label = {"compact": "tool reach", "low": "highest point", "perched": "highest point"}[args.prefer]
    for clearance, score, q_arm in good[:8]:
        print(f"clearance {clearance * 1000:5.1f} mm  {label} {score * 1000:5.0f} mm  joints [{', '.join(f'{v:.2f}' for v in q_arm)}]")


if __name__ == "__main__":
    main()
