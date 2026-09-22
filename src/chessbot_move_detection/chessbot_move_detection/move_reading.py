# Copyright 2026 Yadunund Vijay
# SPDX-License-Identifier: Apache-2.0
"""Work out the human's move from what perception sees on the board.

Perception reports, per square, empty / white / black with a confidence. The rules
say which moves are legal. The move is the legal one whose resulting occupancy best
matches the observation, provided it matches well and clearly better than the
next candidate. Promotions to different pieces look identical, so they count as
one candidate (the queen).
"""

from __future__ import annotations

from dataclasses import dataclass, field

from chessbot_brain.board import Board, square_name

EMPTY, WHITE, BLACK, UNKNOWN = 0, 1, 2, 3


@dataclass
class MoveReading:
    move: str | None  # the accepted move, or None
    reason: str  # why it was accepted or not, for the thought feed
    candidates: list[tuple[str, float]] = field(default_factory=list)  # best first: (move, mismatch)
    changed: list[str] = field(default_factory=list)  # squares that differ from the believed position


def candidate_key(move: str) -> str:
    """Moves the board cannot tell apart, under one name: promotions differ only in the piece."""
    return move[:4] + ("q" if len(move) > 4 else "")


def moves_between(legal_moves: list[str], source: str, destination: str) -> list[str]:
    """The legal moves from `source` to `destination`, promotions collapsed to one."""
    return sorted({candidate_key(m) for m in legal_moves if m[:2] == source and m[2:4] == destination})


def occupancy(board: Board) -> list[int]:
    return [EMPTY if p is None else (WHITE if p.isupper() else BLACK) for p in board.squares]


def mismatch(expected: list[int], observed: list[int], confidence: list[float], floor: float) -> float:
    """Confidence-weighted count of squares where the observation disagrees with the expectation."""
    total = 0.0
    for e, o, c in zip(expected, observed, confidence):
        if o != UNKNOWN and o != e:
            total += max(c, floor)
    return total


def read_move(
    board: Board,
    legal_moves: list[str],
    observed: list[int],
    confidence: list[float],
    *,
    accept_mismatch: float = 0.6,
    margin: float = 0.8,
    floor: float = 0.1,
) -> MoveReading:
    before = occupancy(board)
    changed = [square_name(i) for i, (b, o) in enumerate(zip(before, observed)) if o != UNKNOWN and o != b]

    candidates: dict[str, float] = {}
    for move in legal_moves:
        key = candidate_key(move)
        if key in candidates:
            continue
        after = Board(board.fen())
        after.apply(key)
        candidates[key] = mismatch(occupancy(after), observed, confidence, floor)
    ranked = sorted(candidates.items(), key=lambda pair: pair[1])
    unchanged = mismatch(before, observed, confidence, floor)

    if not changed:
        return MoveReading(None, "the board looks unchanged", ranked[:3], changed)
    if not ranked:
        return MoveReading(None, "there are no legal moves", [], changed)
    best_move, best = ranked[0]
    second = ranked[1][1] if len(ranked) > 1 else float("inf")
    if best > accept_mismatch:
        return MoveReading(None, f"no legal move explains the changes on {', '.join(changed)}", ranked[:3], changed)
    if second - best < margin:
        return MoveReading(None, f"{best_move} and {ranked[1][0]} both fit", ranked[:3], changed)
    if unchanged <= best:
        return MoveReading(None, "the changes are too faint to trust", ranked[:3], changed)
    return MoveReading(best_move, f"it explains the changes on {', '.join(changed)}", ranked[:3], changed)
