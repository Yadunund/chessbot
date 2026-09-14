#!/usr/bin/env python3
# Copyright 2026 Yadunund Vijay
# SPDX-License-Identifier: Apache-2.0
"""Check which squares of a chessboard an arm can service with near-vertical picks.

For every square the tool reports the smallest tilt from vertical at which the
tool frame can reach both the grasp height and the transit height above the
square centre. It also searches for the board placement (distance of the near
edge from the pan axis) that minimises the worst-square tilt.

Assumptions:
  * The first joint (``--pan-joint``) is a vertical pan axis, so reach is
    rotationally symmetric about it and only the joints in ``--sweep-joints``
    need to be sampled. This holds for SO-101 and most tabletop arms.
  * The approach direction is the vector from ``--approach-ref-frame`` to
    ``--tip-frame``.
  * The robot sits centred on the rank-1 edge, facing along ``--forward-axis``.
"""

from __future__ import annotations

import argparse
import itertools
import sys
from dataclasses import dataclass

import numpy as np
import pinocchio as pin


@dataclass
class Samples:
    radial: np.ndarray  # signed distance from the pan axis along forward (m)
    height: np.ndarray  # tip height in the base frame (m)
    tilt_deg: np.ndarray  # angle between approach vector and straight down


def sample_workspace(model: pin.Model, args: argparse.Namespace) -> Samples:
    data = model.createData()
    tip = model.getFrameId(args.tip_frame)
    ref = model.getFrameId(args.approach_ref_frame)
    sweep_idx = [model.joints[model.getJointId(n)].idx_q for n in args.sweep_joints]
    pan_joint_id = model.getJointId(args.pan_joint)
    forward = np.array({"x": [1, 0], "-x": [-1, 0], "y": [0, 1], "-y": [0, -1]}[args.forward_axis], float)

    q = pin.neutral(model)
    pin.forwardKinematics(model, data, q)
    pan_xy = data.oMi[pan_joint_id].translation[:2].copy()

    grids = [np.linspace(model.lowerPositionLimit[i], model.upperPositionLimit[i], args.samples) for i in sweep_idx]
    n = args.samples ** len(sweep_idx)
    radial, height, tilt = np.empty(n), np.empty(n), np.empty(n)
    for k, values in enumerate(itertools.product(*grids)):
        q[sweep_idx] = values
        pin.framesForwardKinematics(model, data, q)
        p = data.oMf[tip].translation
        approach = p - data.oMf[ref].translation
        approach /= np.linalg.norm(approach)
        radial[k] = (p[:2] - pan_xy) @ forward
        height[k] = p[2]
        tilt[k] = np.degrees(np.arccos(np.clip(-approach[2], -1.0, 1.0)))
    return Samples(radial, height, tilt)


def min_tilt(s: Samples, r: float, z: float, tol: float) -> float:
    mask = (np.abs(s.radial - r) < tol) & (np.abs(s.height - z) < tol)
    return float(s.tilt_deg[mask].min()) if mask.any() else float("inf")


def square_tilts(s: Samples, near_edge: float, args: argparse.Namespace) -> np.ndarray:
    """Tilt needed per square, indexed [rank, file] with rank 0 nearest the robot."""
    sq = args.board_size / 8
    z_grasp = args.board_thickness + args.grasp_height
    z_transit = args.board_thickness + args.transit_height
    tilts = np.empty((8, 8))
    for rank, file in itertools.product(range(8), range(8)):
        x = near_edge + (rank + 0.5) * sq
        y = (file - 3.5) * sq
        r = float(np.hypot(x, y))  # pan rotates the arm onto the square
        tilts[rank, file] = max(min_tilt(s, r, z_grasp, args.tol), min_tilt(s, r, z_transit, args.tol))
    return tilts


def print_board(tilts: np.ndarray) -> None:
    for rank in reversed(range(8)):
        cells = " ".join("   -" if np.isinf(v) else f"{v:4.0f}" for v in tilts[rank])
        print(f"  {rank + 1} {cells}")
    print("    " + "".join(f"{c:>5}" for c in "abcdefgh"))
    print("       (robot below rank 1; values are degrees from vertical, '-' = unreachable)")


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--urdf", required=True, help="expanded URDF file (run xacro first)")
    p.add_argument("--tip-frame", default="gripper_frame_link")
    p.add_argument("--approach-ref-frame", default="gripper_link")
    p.add_argument("--pan-joint", default="shoulder_pan_joint")
    p.add_argument("--sweep-joints", nargs="+", default=["shoulder_lift_joint", "elbow_flex_joint", "wrist_flex_joint"])
    p.add_argument("--forward-axis", choices=["x", "-x", "y", "-y"], default="x")
    p.add_argument("--board-size", type=float, default=0.21, help="board side length (m)")
    p.add_argument("--board-thickness", type=float, default=0.010, help="(m)")
    p.add_argument("--grasp-height", type=float, default=0.015, help="tip height above board top when grasping (m)")
    p.add_argument("--transit-height", type=float, default=0.070, help="tip height above board top when carrying (m)")
    p.add_argument("--near-edge", type=float, nargs=3, default=[0.04, 0.16, 0.01], metavar=("MIN", "MAX", "STEP"))
    p.add_argument("--samples", type=int, default=120, help="grid samples per swept joint")
    p.add_argument("--tol", type=float, default=0.004, help="radial/height match tolerance (m)")
    args = p.parse_args(argv)

    model = pin.buildModelFromUrdf(args.urdf)
    print(f"Sampling {args.samples}^{len(args.sweep_joints)} configurations...", file=sys.stderr)
    samples = sample_workspace(model, args)

    best: tuple[float, float, np.ndarray] | None = None
    print(f"Board {args.board_size * 100:.1f} cm, square {args.board_size / 8 * 100:.2f} cm")
    print("near edge (cm)  worst tilt (deg)  mean tilt (deg)")
    for near in np.arange(args.near_edge[0], args.near_edge[1] + 1e-9, args.near_edge[2]):
        tilts = square_tilts(samples, near, args)
        worst = float(tilts.max())
        finite = tilts[np.isfinite(tilts)]
        print(f"{near * 100:14.0f}  {worst:16.1f}  {finite.mean() if finite.size else float('nan'):15.1f}")
        if best is None or worst < best[1]:
            best = (near, worst, tilts)

    assert best is not None
    near, worst, tilts = best
    if np.isinf(worst):
        print("\nNo placement reaches every square. Try a smaller board or lower transit height.")
        return 1
    print(f"\nBest placement: near edge {near * 100:.0f} cm from the pan axis, worst tilt {worst:.1f} deg")
    print_board(tilts)
    return 0


if __name__ == "__main__":
    sys.exit(main())
