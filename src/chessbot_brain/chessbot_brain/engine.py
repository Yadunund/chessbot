# Copyright 2026 Yadunund Vijay
# SPDX-License-Identifier: Apache-2.0
"""Choosing the robot's reply.

Placeholder until the rules engine and Stockfish are wired in: plays a short
book line when it fits the position, otherwise the first pawn push or piece
move that lands on an empty or opponent-occupied square. It never returns a move
for a piece the robot does not own.
"""

from __future__ import annotations

from chessbot_brain.board import Board, piece_colour, square_index, square_name

BOOK = {
    "black": ["e7e5", "b8c6", "g8f6", "f8c5", "d7d6", "c8g4", "e8g8"],
    "white": ["e2e4", "g1f3", "f1c4", "b1c3", "d2d3", "c1g5", "e1g1"],
}


def _is_candidate(board: Board, uci: str, side: str) -> bool:
    piece = board.piece_at(uci[:2])
    if piece is None or piece_colour(piece) != side:
        return False
    target = board.piece_at(uci[2:4])
    return target is None or piece_colour(target) != side


def choose_reply(board: Board, side: str) -> str | None:
    for uci in BOOK[side]:
        if _is_candidate(board, uci, side):
            return uci
    direction = 8 if side == "white" else -8
    pawn = "P" if side == "white" else "p"
    for index, piece in enumerate(board.squares):
        if piece == pawn and 0 <= index + direction < 64 and board.squares[index + direction] is None:
            return square_name(index) + square_name(index + direction)
    for index, piece in enumerate(board.squares):
        if piece and piece_colour(piece) == side:
            for step in (8, -8, 1, -1, 17, 15, -17, -15):
                target = index + step
                if 0 <= target < 64 and _is_candidate(board, square_name(index) + square_name(target), side):
                    return square_name(index) + square_name(target)
    return None


__all__ = ["choose_reply", "square_index"]
