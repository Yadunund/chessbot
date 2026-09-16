# Copyright 2026 Yadunund Vijay
# SPDX-License-Identifier: Apache-2.0
import math

from chessbot_brain.board import Board, square_name
from chessbot_brain.geometry import PIECE_HEIGHTS, BoardGeometry, jaw_clearances

GEOMETRY = BoardGeometry("base_link", (0.3488, -0.105, 0.0), math.pi / 2, 0.02625)


def others(board: Board, square: str):
    return [
        (*GEOMETRY.square_centre(square_name(i)), PIECE_HEIGHTS[p.lower()])
        for i, p in enumerate(board.squares)
        if p and square_name(i) != square
    ]


def test_back_rank_knight_has_a_clear_grasp():
    # The start position is too tight for 5 mm with a vertical grasp; about 3.5 mm is the best.
    board = Board()
    ranked = jaw_clearances(GEOMETRY.square_centre("b8"), others(board, "b8"), 0.010, 0.0075)
    assert ranked[0][1] >= 0.003


def test_fully_surrounded_piece_still_fits_along_a_diagonal():
    # All eight neighbours occupied by tall pieces: the jaws still fit between the diagonals.
    board = Board("8/8/8/3qqq2/3qPq2/3qqq2/8/8 w - - 0 1")
    ranked = jaw_clearances(GEOMETRY.square_centre("e4"), others(board, "e4"), 0.010, 0.0075)
    assert 0.0 < ranked[0][1] < 0.005


def test_crowded_off_grid_pieces_leave_no_grasp():
    # Pieces knocked close together (20 mm apart all round) cannot be grasped vertically.
    x, y, z = GEOMETRY.square_centre("e4")
    crowd = [(x + 0.02 * math.cos(a), y + 0.02 * math.sin(a), z, 0.042) for a in [i * math.pi / 6 for i in range(12)]]
    ranked = jaw_clearances((x, y, z), crowd, 0.010, 0.0075)
    assert ranked[0][1] < 0.0


def test_lone_piece_is_clear_in_every_direction():
    ranked = jaw_clearances(GEOMETRY.square_centre("e4"), [], 0.010, 0.0075)
    assert len(ranked) == 24 and all(c == math.inf for _, c in ranked)
