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


def print_footprint(model, data, geom, gdata, tip, grip, args):
    """Convex hull, in the tool's (opening, side) plane, of the gripper's collision vertices
    that are low enough to meet a piece when the tool is at grasp height and vertical."""
    from scipy.spatial import ConvexHull

    q = pin.neutral(model)
    q[grip.idx_q] = args.footprint
    pin.framesForwardKinematics(model, data, q)
    pin.updateGeometryPlacements(model, data, geom, gdata, q)
    tool = data.oMf[tip]
    # Tool axes: z along the approach (from gripper_link towards the tip), x the opening.
    approach = tool.translation - data.oMf[model.getFrameId(args.fixed_jaw)].translation
    approach /= np.linalg.norm(approach)
    opening = tool.rotation @ np.array([-1.0, 0.0, 0.0])
    opening -= (opening @ approach) * approach
    opening /= np.linalg.norm(opening)
    side = np.cross(approach, opening)
    points = []
    for i, g in enumerate(geom.geometryObjects):
        if not (g.name.startswith(args.fixed_jaw) or g.name.startswith(args.moving_jaw)):
            continue
        vertices = np.asarray(g.geometry.vertices())
        world = (gdata.oMg[i].rotation @ vertices.T).T + gdata.oMg[i].translation
        rel = world - tool.translation
        depth = rel @ approach  # + towards the table
        # From the table (grasp height below the tip) up to the tallest piece.
        keep = (depth <= args.grasp_height) & (depth >= args.grasp_height - args.tallest)
        points.extend(np.stack([rel[keep] @ opening, rel[keep] @ side], axis=1))
    points = np.asarray(points)
    hull = points[ConvexHull(points).vertices]
    print(f"footprint at gripper {args.footprint} rad: {len(points)} vertices, hull (opening, side) mm:")
    print("[" + ", ".join(f"[{x:.4f}, {y:.4f}]" for x, y in hull) + "]")
    print(f"extent opening {points[:, 0].min() * 1000:.1f}..{points[:, 0].max() * 1000:.1f} mm, "
          f"side {points[:, 1].min() * 1000:.1f}..{points[:, 1].max() * 1000:.1f} mm")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--urdf-url", default="http://127.0.0.1:8000/api/robot_description")
    parser.add_argument("--share", default=os.path.expanduser("~/chessbot/install/so_arm101_description/share"))
    parser.add_argument("--tip", default="gripper_frame_link")
    parser.add_argument("--fixed-jaw", default="gripper_link")
    parser.add_argument("--moving-jaw", default="jaw_link")
    parser.add_argument("--gripper-joint", default="gripper_joint")
    parser.add_argument("--band", type=float, default=0.010, help="height of the probed band along the tool axis, m")
    parser.add_argument("--footprint", type=float, metavar="ANGLE", help="print the gripper's footprint at this gripper angle and exit")
    parser.add_argument("--grasp-height", type=float, default=0.015, help="grasp point height above the table, m")
    parser.add_argument("--tallest", type=float, default=0.042, help="tallest piece, m")
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

    if args.footprint is not None:
        print_footprint(model, data, geom, gdata, tip, grip, args)
        return

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
