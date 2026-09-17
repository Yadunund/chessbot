#!/usr/bin/env python3
# Copyright 2026 Yadunund Vijay
# SPDX-License-Identifier: Apache-2.0
"""Play the human's move in simulation by moving pieces in Gazebo, as a person would.

Handles captures, castling (the rook moves too) and en passant. Promotion leaves the
pawn in place of the new piece. With --press, also presses the clock; with
--no-move-hint, the clock press carries no move, so the brain must read it from the camera.

    pixi run python tools/sim/human_move.py e2e4 --press
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import gzsim  # noqa: E402
import pieces  # noqa: E402
from scenario import calibration, http, set_piece, state  # noqa: E402

from chessbot_brain.board import Board  # noqa: E402


def square_xyz(board: dict, square: str):
    s = board["square_size_m"]
    f, r = "abcdefgh".index(square[0]), int(square[1]) - 1
    return pieces.board_to_world(board, (f + 0.5) * s, (r + 0.5) * s)


def piece_on(board: dict, poses: dict, square: str, tolerance: float = 0.012) -> str | None:
    target = square_xyz(board, square)
    best = None
    for name, (xyz, _) in poses.items():
        if not name.startswith("pc_"):
            continue
        d = math.dist(xyz[:2], target[:2])
        if d < tolerance and (best is None or d < best[0]):
            best = (d, name)
    return best[1] if best else None


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("move", help="UCI, e.g. e2e4")
    parser.add_argument("--press", action="store_true", help="press the clock afterwards")
    parser.add_argument("--no-move-hint", action="store_true", help="press the clock without telling the brain the move")
    args = parser.parse_args()

    board = calibration()
    belief = state()
    effects = Board(belief["fen"]).effects(args.move)
    poses = gzsim.model_poses()

    def move(src: str, dst: str):
        name = piece_on(board, poses, src)
        if name is None:
            raise SystemExit(f"no piece found on {src} in the simulation")
        set_piece(name, square_xyz(board, dst))

    if effects.captured and effects.capture_square:
        victim = piece_on(board, poses, effects.capture_square)
        if victim:
            gzsim.remove(victim)
    move(args.move[:2], args.move[2:4])
    if effects.rook_from and effects.rook_to:
        move(effects.rook_from, effects.rook_to)
    time.sleep(1.0)
    print(f"played {args.move} in the simulation")

    if args.press:
        http("POST", "/api/press_clock", {} if args.no_move_hint else {"move": args.move})
        print("pressed the clock" + ("" if args.no_move_hint else " (with the move)"))


if __name__ == "__main__":
    main()
