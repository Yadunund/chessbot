# Copyright 2026 Yadunund Vijay
# SPDX-License-Identifier: Apache-2.0
"""Measure the gripper from its collision meshes.

For a range of gripper joint angles, reports at the grasp point (the IK tip frame):
- the opening: free space between the two jaws, through the grasp point;
- the offset of the grasp point from the middle of the opening;
- the direction the jaws open along, in the tool frame.

Run with the stack up (the URDF comes from the brain):
    pixi run python tools/dev/gripper_geometry.py
"""

from __future__ import annotations

import argparse
import os
import tempfile
import urllib.request

import coal
import numpy as np
import pinocchio as pin

ARM = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_flex_joint", "wrist_flex_joint", "wrist_roll_joint"]


def load(urdf_url: str, share: str):
    urdf = urllib.request.urlopen(urdf_url).read().decode()
    path = os.path.join(tempfile.mkdtemp(), "robot.urdf")
    with open(path, "w") as f:
        f.write(urdf)
    model = pin.buildModelFromXML(urdf)
    geom = pin.buildGeomFromUrdf(model, path, pin.GeometryType.COLLISION, package_dirs=[share])
    return model, geom


def jaw_geometries(geom, fixed_link: str, moving_link: str):
    fixed = [i for i, g in enumerate(geom.geometryObjects) if g.name.startswith(fixed_link)]
    moving = [i for i, g in enumerate(geom.geometryObjects) if g.name.startswith(moving_link)]
    return fixed, moving


def nearest(geom, gdata, ids, probe, probe_pose):
    """Distance and nearest points between a probe shape and a set of geometries."""
    best = (np.inf, None, None)
    for i in ids:
        request, result = coal.DistanceRequest(), coal.DistanceResult()
        request.enable_nearest_points = True
        obj = geom.geometryObjects[i]
        d = coal.distance(obj.geometry, gdata.oMg[i], probe, probe_pose, request, result)
        if d < best[0]:
            best = (d, np.array(result.getNearestPoint1()), np.array(result.getNearestPoint2()))
    return best


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--urdf-url", default="http://127.0.0.1:8000/api/robot_description")
    parser.add_argument("--share", default=os.path.expanduser("~/chessbot/install/so_arm101_description/share"))
    parser.add_argument("--tip", default="gripper_frame_link")
    parser.add_argument("--fixed-jaw", default="gripper_link")
    parser.add_argument("--moving-jaw", default="jaw_link")
    parser.add_argument("--gripper-joint", default="gripper_joint")
    parser.add_argument("--band", type=float, default=0.010, help="height of the probed band along the tool axis, m")
    args = parser.parse_args()

    model, geom = load(args.urdf_url, args.share)
    data, gdata = model.createData(), pin.GeometryData(geom)
    fixed, moving = jaw_geometries(geom, args.fixed_jaw, args.moving_jaw)
    if not fixed or not moving:
        raise SystemExit(f"no collision geometry for {args.fixed_jaw!r} / {args.moving_jaw!r}")
    tip = model.getFrameId(args.tip)
    grip = model.joints[model.getJointId(args.gripper_joint)]
    lower, upper = model.lowerPositionLimit[grip.idx_q], model.upperPositionLimit[grip.idx_q]
    print(f"gripper joint limits: {lower:.3f} .. {upper:.3f} rad")

    # A thin segment along the tool axis through the grasp point.
    probe = coal.Cylinder(1e-4, args.band)
    print(f"{'angle':>7} {'opening':>9} {'to fixed':>9} {'to moving':>10} {'centre offset':>14}  opening direction (tool frame)")
    for angle in np.linspace(lower, upper, 15):
        q = pin.neutral(model)
        q[grip.idx_q] = angle
        pin.framesForwardKinematics(model, data, q)
        pin.updateGeometryPlacements(model, data, geom, gdata, q)
        tool = data.oMf[tip]
        # coal cylinders run along their local z; the tool frame's approach axis is found
        # from the tip frame, so align the probe with the tool z axis.
        probe_pose = coal.Transform3s(tool.rotation, tool.translation)
        d_fixed, p_fixed, _ = nearest(geom, gdata, fixed, probe, probe_pose)
        d_moving, p_moving, _ = nearest(geom, gdata, moving, probe, probe_pose)
        # Opening: distance between the nearest points on each jaw, when they lie on
        # opposite sides of the grasp point.
        axis = p_moving - p_fixed
        opening = float(np.linalg.norm(axis))
        direction = tool.rotation.T @ (axis / opening) if opening > 0 else np.zeros(3)
        centre = (p_fixed + p_moving) / 2
        offset = float(np.linalg.norm((centre - tool.translation) - ((centre - tool.translation) @ tool.rotation[:, 2]) * tool.rotation[:, 2]))
        print(f"{angle:7.3f} {opening * 1000:8.1f}mm {d_fixed * 1000:8.1f}mm {d_moving * 1000:9.1f}mm {offset * 1000:13.1f}mm  "
              f"[{direction[0]:+.2f} {direction[1]:+.2f} {direction[2]:+.2f}]")


if __name__ == "__main__":
    main()
