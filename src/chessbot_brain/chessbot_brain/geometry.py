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

    def board_to_robot(self, bx: float, by: float) -> tuple[float, float, float]:
        c, s = math.cos(self.yaw_rad), math.sin(self.yaw_rad)
        ox, oy, oz = self.origin_xyz
        return ox + c * bx - s * by, oy + s * bx + c * by, oz

    def square_centre(self, square: str) -> tuple[float, float, float]:
        index = square_index(square)
        file, rank = index % 8, index // 8
        return self.board_to_robot((file + 0.5) * self.square_size_m, (rank + 0.5) * self.square_size_m)

    def graveyard_slot(self, colour: str, slot: int) -> tuple[float, float, float]:
        """Captured pieces go in two rows beside the board, one side per colour.

        White pieces beside the a-file edge, black beside the h-file edge, two
        squares out from the board so the arm clears the edge pieces.
        """
        column, row = slot % 8, slot // 8
        bx = -2.0 * self.square_size_m - row * self.square_size_m if colour == "white" else (
            10.0 * self.square_size_m + row * self.square_size_m
        )
        by = (column + 0.5) * self.square_size_m
        return self.board_to_robot(bx, by)
