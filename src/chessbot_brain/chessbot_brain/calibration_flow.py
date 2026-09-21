# Copyright 2026 Yadunund Vijay
# SPDX-License-Identifier: Apache-2.0
"""Calibrating a real rig, as a sequence of steps a person can be walked through.

Nothing about a real setup can be assumed: the camera is mounted wherever it fits, the board
is put down wherever there is room, and the arm's resting pose has to keep clear of both. So
calibration is a conversation, not a measurement the robot can take alone:

  1. board   - the person marks the board's four corners in the camera image. Those pixels
               back-project onto the table plane, which gives the board's origin, rotation
               and square size in the robot's frame.
  2. camera  - the person hand-guides the gripper to touch those same four corners (torque
               released, so the arm moves freely and is read passively) while the board step's
               pixel clicks are reused as the other half of each correspondence. The camera
               pose is fitted to the resulting (touched 3D, clicked 2D) pairs.
  3. reach   - every square is checked against IK, so a board placed out of reach is caught
               here rather than by the arm straining at it.
  4. park    - the arm is jogged to where it should wait, and that pose is saved.

Each step writes into a draft profile; saving hands the finished draft to the calibration
node, which owns the stored profile and the key-value store the rest of the stack reads.

The geometry here is deliberately simple: the table is the plane z = 0 in the robot's base
frame, which is what the board calibration already assumes.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np


# A Logitech C920 at 1280x720: 70.4 degrees across. Only a starting point - camera_info is
# used when it carries something real.
DEFAULT_FOCAL_PX = 907.0
# The camera step touches the same four corners the board step already had clicked, in the
# same a1, h1, h8, a8 order - no separate point list to keep in sync.
CAMERA_TOUCH_POINTS = ("a1", "h1", "h8", "a8")


@dataclass
class CameraModel:
    """A pinhole camera in the robot's base frame."""

    position: tuple[float, float, float]
    rpy: tuple[float, float, float]
    focal_px: float
    centre_px: tuple[float, float]
    # Mean and worst reprojection error of the fit that produced it, pixels.
    error_mean_px: float = 0.0
    error_max_px: float = 0.0

    def rotation(self) -> np.ndarray:
        roll, pitch, yaw = self.rpy
        cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                                  math.sin(pitch), math.cos(yaw), math.sin(yaw))
        return np.array([
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ])

    def project(self, point) -> tuple[float, float]:
        local = self.rotation().T @ (np.asarray(point, float) - np.asarray(self.position, float))
        depth = max(float(local[2]), 1e-3)
        return (self.centre_px[0] + self.focal_px * local[0] / depth,
                self.centre_px[1] + self.focal_px * local[1] / depth)

    def ray(self, pixel) -> np.ndarray:
        """Unit direction in the base frame for a pixel."""
        direction = np.array([(pixel[0] - self.centre_px[0]) / self.focal_px,
                              (pixel[1] - self.centre_px[1]) / self.focal_px, 1.0])
        world = self.rotation() @ direction
        return world / np.linalg.norm(world)

    def to_dict(self) -> dict:
        return {
            "position_xyz": list(self.position),
            "rpy": list(self.rpy),
            "focal_px": self.focal_px,
            "centre_px": list(self.centre_px),
            "error_mean_px": self.error_mean_px,
            "error_max_px": self.error_max_px,
        }

    @staticmethod
    def from_dict(data: dict) -> "CameraModel":
        return CameraModel(
            position=tuple(data["position_xyz"]),
            rpy=tuple(data["rpy"]),
            focal_px=float(data["focal_px"]),
            centre_px=tuple(data["centre_px"]),
            error_mean_px=float(data.get("error_mean_px", 0.0)),
            error_max_px=float(data.get("error_max_px", 0.0)),
        )


def fit_camera(points, pixels, centre_px, focal_px: float) -> CameraModel:
    """Fit a camera pose to gripper positions seen at known pixels.

    The focal length is given rather than solved: the poses a short arm can hold are nearly
    coplanar, and a free focal length simply trades against the camera's height.
    """
    from scipy.optimize import least_squares

    points = np.asarray(points, float)
    pixels = np.asarray(pixels, float)

    def residual(params):
        model = CameraModel(tuple(params[0:3]), tuple(params[3:6]), focal_px, centre_px)
        return np.array([model.project(p) for p in points]).ravel() - pixels.ravel()

    best, best_cost = None, np.inf
    # A camera looking down is a half turn about x; its yaw depends on how it was mounted, so
    # every yaw is tried rather than assumed.
    for yaw in np.linspace(-math.pi, math.pi, 12, endpoint=False):
        guess = np.array([0.2, 0.0, 0.5, math.pi, 0.0, yaw])
        lower = [-1.0, -1.0, 0.10, -2 * math.pi, -2 * math.pi, -2 * math.pi]
        upper = [1.0, 1.0, 1.50, 2 * math.pi, 2 * math.pi, 2 * math.pi]
        result = least_squares(residual, np.clip(guess, lower, upper), bounds=(lower, upper), max_nfev=4000)
        if result.cost < best_cost:
            best, best_cost = result, result.cost

    model = CameraModel(tuple(best.x[0:3]), tuple(best.x[3:6]), focal_px, centre_px)
    errors = np.linalg.norm(np.array([model.project(p) for p in points]) - pixels, axis=1)
    model.error_mean_px = float(errors.mean())
    model.error_max_px = float(errors.max())
    return model


@dataclass
class BoardFromCorners:
    """The board pose implied by its four corners, marked in the camera image.

    Corners are given a1, h1, h8, a8, so the board frame follows the same convention as the
    stored calibration: origin at the outer corner of a1, +x towards the h file, +y towards
    rank 8.
    """

    origin_xyz: tuple[float, float, float]
    yaw_rad: float
    square_size_m: float
    # How far the four corners are from a perfect square, metres: large means the camera pose
    # or the marked corners are wrong.
    squareness_m: float
    side_lengths_m: tuple[float, float, float, float]


def board_from_points(points) -> BoardFromCorners | None:
    """Board pose from its four outer corners in the robot frame, given a1, h1, h8, a8."""
    points = [np.asarray(p, float) for p in points]
    if len(points) != 4:
        return None
    a1, h1, h8, a8 = points
    x_axis = ((h1 - a1) + (h8 - a8)) / 2.0
    y_axis = ((a8 - a1) + (h8 - h1)) / 2.0
    sides = (float(np.linalg.norm(h1 - a1)), float(np.linalg.norm(h8 - h1)),
             float(np.linalg.norm(a8 - h8)), float(np.linalg.norm(a1 - a8)))
    board_size = (float(np.linalg.norm(x_axis)) + float(np.linalg.norm(y_axis))) / 2.0
    return BoardFromCorners(
        origin_xyz=(float(a1[0]), float(a1[1]), float(np.mean([p[2] for p in points]))),
        yaw_rad=float(math.atan2(x_axis[1], x_axis[0])),
        square_size_m=board_size / 8.0,
        squareness_m=float(max(sides) - min(sides)),
        side_lengths_m=sides,
    )


def board_from_touches(touched: dict) -> BoardFromCorners | None:
    """Board pose measured by touching its four outer corners with the gripper.

    The arm's own kinematics are the instrument, so the result carries no camera error and
    needs no guess at how high the squares sit: the tips were on them.
    """
    try:
        return board_from_points([touched[name] for name in CAMERA_TOUCH_POINTS])
    except (KeyError, TypeError):
        return None


def square_centres(origin_xyz, yaw_rad: float, square_size_m: float) -> dict[str, tuple[float, float, float]]:
    """Centre of every square in the base frame, for reach checks and overlays."""
    cos_yaw, sin_yaw = math.cos(yaw_rad), math.sin(yaw_rad)
    centres = {}
    for file_index, file_letter in enumerate("abcdefgh"):
        for rank in range(8):
            bx = (file_index + 0.5) * square_size_m
            by = (rank + 0.5) * square_size_m
            centres[f"{file_letter}{rank + 1}"] = (
                origin_xyz[0] + cos_yaw * bx - sin_yaw * by,
                origin_xyz[1] + sin_yaw * bx + cos_yaw * by,
                origin_xyz[2],
            )
    return centres


@dataclass
class Draft:
    """What the workflow has established so far."""

    camera: CameraModel | None = None
    board_origin_xyz: tuple[float, float, float] | None = None
    board_yaw_rad: float | None = None
    square_size_m: float | None = None
    squareness_m: float | None = None
    park_joints: list[float] | None = None
    unreachable: list[str] = field(default_factory=list)
    # Empty when the board is the right way round, otherwise what is wrong with it.
    orientation: str = ""
    # The board step's raw pixel clicks (a1, h1, h8, a8 order), kept so the camera step can
    # reuse them as the 2D half of its correspondences instead of asking again.
    board_corners_px: list[tuple[float, float]] | None = None
    # Measured from the touches: how far the squares sit above the table.
    surface_height_m: float = 0.0
    # A known square size, if the person has one, checked against what the touches measure.
    expected_square_size_m: float = 0.0
    # Whether the arm is released (torque off), and where the jaw tips were for each of
    # CAMERA_TOUCH_POINTS touched so far. All four are needed: the fit has no spare.
    arm_released: bool = False
    touched: dict[str, tuple[float, float, float]] = field(default_factory=dict)

    def steps(self) -> dict:
        """What is done and what is still needed, for the UI to render."""
        return {
            "camera": None if self.camera is None else {
                "position_xyz": list(self.camera.position),
                "rpy": list(self.camera.rpy),
                "error_mean_px": round(self.camera.error_mean_px, 1),
                "error_max_px": round(self.camera.error_max_px, 1),
            },
            "board": None if self.board_origin_xyz is None else {
                "origin_xyz": [round(v, 4) for v in self.board_origin_xyz],
                "yaw_deg": round(math.degrees(self.board_yaw_rad or 0.0), 1),
                "square_size_mm": round((self.square_size_m or 0.0) * 1000.0, 1),
                "squareness_mm": round((self.squareness_m or 0.0) * 1000.0, 1),
                "surface_height_mm": round(self.surface_height_m * 1000.0, 1),
                "expected_square_size_mm": round(self.expected_square_size_m * 1000.0, 1),
                "square_size_error_mm": (round((self.square_size_m - self.expected_square_size_m) * 1000.0, 1)
                                         if self.expected_square_size_m and self.square_size_m else None),
            },
            "reach": {"unreachable": list(self.unreachable), "orientation": self.orientation}
            if self.board_origin_xyz is not None else None,
            "park": None if self.park_joints is None else {"joints": [round(v, 4) for v in self.park_joints]},
            # Independent of the board step until fit time, when each name picks up its pixel.
            "touch": {
                "arm_released": self.arm_released,
                "points": [
                    {"name": name, "done": name in self.touched} for name in CAMERA_TOUCH_POINTS
                ],
                "next": next((name for name in CAMERA_TOUCH_POINTS if name not in self.touched), None),
            },
        }
