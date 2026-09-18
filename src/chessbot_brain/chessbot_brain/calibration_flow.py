# Copyright 2026 Yadunund Vijay
# SPDX-License-Identifier: Apache-2.0
"""Calibrating a real rig, as a sequence of steps a person can be walked through.

Nothing about a real setup can be assumed: the camera is mounted wherever it fits, the board
is put down wherever there is room, and the arm's resting pose has to keep clear of both. So
calibration is a conversation, not a measurement the robot can take alone:

  1. camera  - the arm shows the camera its own gripper, and the camera pose is fitted to it.
               Fully automatic, but it needs the arm to be free to move.
  2. board   - the person marks the board's four corners in the camera image. Those pixels
               back-project onto the table plane, which gives the board's origin, rotation
               and square size in the robot's frame.
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
# Tool-down poses used to show the camera the gripper. Kept low, because a short arm cannot
# hold the tool down much higher, and inside the same safe box as manual control.
# Heights above the playing surface to hold the gripper at, when sampling over a known board.
CAMERA_SAMPLE_HEIGHTS = (0.08, 0.12)
# Fallback poses in the robot's own frame, for a rig whose board is not marked yet.
CAMERA_SAMPLES = [
    (0.16, -0.10, 0.08), (0.16, 0.10, 0.08), (0.20, -0.06, 0.08), (0.20, 0.06, 0.08),
    (0.23, -0.07, 0.08), (0.23, 0.07, 0.08), (0.20, -0.06, 0.11), (0.20, 0.06, 0.11),
    (0.23, -0.07, 0.11), (0.23, 0.07, 0.11),
]


def as_array(msg) -> np.ndarray:
    """A sensor_msgs/Image as an RGB array."""
    data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    image = data.reshape(msg.height, msg.step // 3, 3)[:, : msg.width, :]
    return image[:, :, ::-1] if msg.encoding == "bgr8" else image


def moved_pixel(before, after, threshold: int = 35, min_pixels: int = 40):
    """Where the picture changed between two frames, and how much of it changed.

    Used with the jaws closed and then open: the jaws are then the only thing that moved, so
    this is where the gripper is in the image. Returns (pixel, changed pixel count) so the
    caller can throw out samples where something else moved too - see `keep_consistent`.
    """
    if before is None or after is None:
        return None, 0
    first, second = as_array(before).astype(np.int16), as_array(after).astype(np.int16)
    if first.shape != second.shape:
        return None, 0
    difference = np.abs(second - first).sum(axis=2)
    mask = difference > threshold * 3
    changed = int(mask.sum())
    if changed < min_pixels:
        return None, changed
    ys, xs = np.nonzero(mask)
    weights = difference[ys, xs].astype(float)
    centre = (float((xs * weights).sum() / weights.sum()), float((ys * weights).sum() / weights.sum()))
    return centre, changed


def keep_consistent(samples, low: float = 0.4, high: float = 2.5):
    """Drop samples whose changed area is nothing like the rest.

    The jaws sweep about the same area whichever pose they are in, so a sample that changed
    several times more pixels than the others had something else moving in it - usually the
    whole arm flexing as the gripper actuates - and its centroid is not the gripper. Those
    samples pull the fit badly, and are cheaper to discard than to model.
    """
    if len(samples) < 3:
        return samples
    median = float(np.median([count for _position, _pixel, count in samples]))
    return [s for s in samples if low * median <= s[2] <= high * median]


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

    def on_plane(self, pixel, height: float = 0.0) -> tuple[float, float, float] | None:
        """Where a pixel's ray meets the plane z = `height`, or None if it never does."""
        origin = np.asarray(self.position, float)
        direction = self.ray(pixel)
        if abs(direction[2]) < 1e-6 or (height - origin[2]) / direction[2] <= 0.0:
            return None
        distance = (height - origin[2]) / direction[2]
        point = origin + distance * direction
        return (float(point[0]), float(point[1]), float(point[2]))

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


def board_from_corners(camera: CameraModel, corners, surface_height_m: float = 0.0) -> BoardFromCorners | None:
    """Back-project four marked corners onto the playing surface and read off the board's pose.

    `surface_height_m` is how far the squares sit above the table, which for a folding board
    is the thickness of its case. It matters twice over, and getting it wrong is not obvious:
    the corners are back-projected onto that plane, so assuming the table pushes the board
    outwards from under the camera and inflates the square size; and the stored origin carries
    the height the arm descends to, so an arm told the squares are on the table drives its
    gripper into a board that is two centimetres higher.

    Projecting the result back onto the image cannot catch this - back-projecting to the wrong
    plane and projecting back through the same camera is self consistent, and the overlay looks
    perfect either way. Only a ruler, or the arm touching the surface, can tell.
    """
    points = []
    for pixel in corners:
        point = camera.on_plane(pixel, surface_height_m)
        if point is None:
            return None
        points.append(np.array(point, float))
    if len(points) != 4:
        return None

    a1, h1, h8, a8 = points
    height = float(a1[2])
    x_axis = ((h1 - a1) + (h8 - a8)) / 2.0
    y_axis = ((a8 - a1) + (h8 - h1)) / 2.0
    sides = (float(np.linalg.norm(h1 - a1)), float(np.linalg.norm(h8 - h1)),
             float(np.linalg.norm(a8 - h8)), float(np.linalg.norm(a1 - a8)))
    board_size = (float(np.linalg.norm(x_axis)) + float(np.linalg.norm(y_axis))) / 2.0
    return BoardFromCorners(
        origin_xyz=(float(a1[0]), float(a1[1]), height),
        yaw_rad=float(math.atan2(x_axis[1], x_axis[0])),
        square_size_m=board_size / 8.0,
        squareness_m=float(max(sides) - min(sides)),
        side_lengths_m=sides,
    )


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
            },
            "reach": {"unreachable": list(self.unreachable), "orientation": self.orientation}
            if self.board_origin_xyz is not None else None,
            "park": None if self.park_joints is None else {"joints": [round(v, 4) for v in self.park_joints]},
        }
