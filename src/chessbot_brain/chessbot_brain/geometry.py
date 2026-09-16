# Copyright 2026 Yadunund Vijay
# SPDX-License-Identifier: Apache-2.0
"""Board geometry: turns squares and graveyard slots into positions in the robot frame.

Everything is derived from the calibration profile (read from the key-value
store), so a different board or placement only changes the calibration.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from chessbot_brain.board import square_index


# Heights of the piece set, metres (base diameter 15 mm).
PIECE_HEIGHTS = {"k": 0.042, "q": 0.037, "b": 0.033, "n": 0.030, "r": 0.026, "p": 0.022}


@dataclass
class GripperFootprint:
    """The open gripper's outline from the table up to some height, around the grasp point,
    in the tool's (opening, side) axes, metres, with the grasp point 15 mm above the table.
    Measured with tools/dev/gripper_geometry.py --footprint 0.24 --tallest <height>."""

    height: float
    opening_min: float
    opening_max: float
    side_min: float
    side_max: float


# Wider parts of the gripper sit higher up, so a short piece only meets its lower part.
GRIPPER_FOOTPRINTS = [
    GripperFootprint(0.022, -0.0097, 0.0302, -0.0063, 0.0064),
    GripperFootprint(0.026, -0.0124, 0.0310, -0.0069, 0.0070),
    GripperFootprint(0.030, -0.0132, 0.0327, -0.0072, 0.0072),
    GripperFootprint(0.033, -0.0150, 0.0339, -0.0075, 0.0075),
    GripperFootprint(0.037, -0.0174, 0.0340, -0.0077, 0.0078),
    GripperFootprint(0.042, -0.0193, 0.0340, -0.0080, 0.0081),
]


def footprint_below(height: float, footprints: list[GripperFootprint] = GRIPPER_FOOTPRINTS) -> GripperFootprint:
    """The outline of the gripper parts that can meet a piece of this height."""
    for footprint in footprints:
        if footprint.height >= height - 1e-6:
            return footprint
    return footprints[-1]


def jaw_clearances(
    piece_xyz, others, offset: float, piece_radius: float, step_deg: float = 15.0,
    footprints: list[GripperFootprint] = GRIPPER_FOOTPRINTS,
) -> list[tuple[float, float]]:
    """(yaw, clearance) for jaw yaws around the circle, clearest first.

    For each yaw the grasp point sits `offset` behind the piece along the opening axis,
    and the clearance is the smallest gap between the gripper and any other piece
    (negative when they overlap). `others` holds (x, y, z[, height]); a piece without a
    height is treated as the tallest.
    """
    px, py = piece_xyz[0], piece_xyz[1]
    out = []
    for i in range(int(round(360.0 / step_deg))):
        yaw = math.radians(i * step_deg)
        ux, uy = math.cos(yaw), math.sin(yaw)
        gx, gy = px - offset * ux, py - offset * uy
        clearance = math.inf
        for other in others:
            ox, oy = other[0], other[1]
            footprint = footprint_below(other[3] if len(other) > 3 else math.inf, footprints)
            dx, dy = ox - gx, oy - gy
            along, side = dx * ux + dy * uy, -dx * uy + dy * ux
            out_along = max(footprint.opening_min - along, 0.0, along - footprint.opening_max)
            out_side = max(footprint.side_min - side, 0.0, side - footprint.side_max)
            if out_along == 0.0 and out_side == 0.0:
                gap = -min(along - footprint.opening_min, footprint.opening_max - along,
                           side - footprint.side_min, footprint.side_max - side)
            else:
                gap = math.hypot(out_along, out_side)
            clearance = min(clearance, gap - piece_radius)
        out.append((yaw, clearance))
    out.sort(key=lambda pair: -pair[1])
    return out


@dataclass
class BoardGeometry:
    frame_id: str
    origin_xyz: tuple[float, float, float]
    yaw_rad: float
    square_size_m: float

    @staticmethod
    def from_calibration(data: dict) -> "BoardGeometry":
        board = data["board"]
        return BoardGeometry(
            frame_id=board["frame_id"],
            origin_xyz=tuple(board["origin_xyz"]),
            yaw_rad=float(board["yaw_rad"]),
            square_size_m=float(board["square_size_m"]),
        )

    def over_play_area(self, x: float, y: float, margin: float = 0.05) -> bool:
        """Whether a robot-frame point is above the board or its graveyards (plus a margin)."""
        c, s = math.cos(self.yaw_rad), math.sin(self.yaw_rad)
        dx, dy = x - self.origin_xyz[0], y - self.origin_xyz[1]
        bx, by = c * dx + s * dy, -s * dx + c * dy
        sq = self.square_size_m
        return -4 * sq - margin <= bx <= 12 * sq + margin and -margin <= by <= 8 * sq + margin

    def board_to_robot(self, bx: float, by: float) -> tuple[float, float, float]:
        c, s = math.cos(self.yaw_rad), math.sin(self.yaw_rad)
        ox, oy, oz = self.origin_xyz
        return ox + c * bx - s * by, oy + s * bx + c * by, oz

    def square_centre(self, square: str) -> tuple[float, float, float]:
        index = square_index(square)
        file, rank = index % 8, index // 8
        return self.board_to_robot((file + 0.5) * self.square_size_m, (rank + 0.5) * self.square_size_m)

    def graveyard_slot(self, colour: str, slot: int) -> tuple[float, float, float]:
        bx, by = graveyard_cell(colour, slot)
        return self.board_to_robot(bx * self.square_size_m, by * self.square_size_m)


def graveyard_cell(colour: str, slot: int) -> tuple[float, float]:
    """Centre of a graveyard slot in the board frame, in units of squares.

    Captured pieces go in two rows beside the board, one side per colour.
    White pieces beside the a-file edge, black beside the h-file edge, two
    squares out from the board so the arm clears the edge pieces. Slots fill
    from the rank-8 end, where the robot sits, so the hardest-to-reach slots
    at the far end of the second row are used last.
    """
    column, row = slot % 8, slot // 8
    bx = -2.0 - row if colour == "white" else 10.0 + row
    return bx, 7 - column + 0.5
