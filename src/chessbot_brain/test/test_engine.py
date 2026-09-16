# Copyright 2026 Yadunund Vijay
# SPDX-License-Identifier: Apache-2.0
"""UCI engine tests against a real Stockfish binary (skipped when not built)."""

import os

import pytest

from chessbot_brain.board import START_FEN
from chessbot_brain.engine import UciEngine

STOCKFISH = os.path.join(os.environ.get("PIXI_PROJECT_ROOT", ""), "generated", "bin", "stockfish")


@pytest.fixture(scope="module")
def engine():
    if not os.path.isfile(STOCKFISH):
        pytest.skip("Stockfish not built (pixi run stockfish)")
    eng = UciEngine(STOCKFISH, threads=1)
    yield eng
    eng.close()


def test_start_position_has_twenty_legal_moves(engine):
    moves = engine.legal_moves(START_FEN)
    assert len(moves) == 20
    assert "e2e4" in moves


def test_fools_mate_is_checkmate(engine):
    fen = "rnb1kbnr/pppp1ppp/8/4p3/6Pq/5P2/PPPPP2P/RNBQKBNR w KQkq - 1 3"
    assert engine.legal_moves(fen) == []
    assert engine.in_check(fen)


def test_best_move_is_legal(engine):
    move = engine.best_move(START_FEN, movetime_ms=100, elo=1400)
    assert move in engine.legal_moves(START_FEN)
