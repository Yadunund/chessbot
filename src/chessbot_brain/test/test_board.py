# Copyright 2026 Yadunund Vijay
# SPDX-License-Identifier: Apache-2.0

import pytest

from chessbot_brain.board import START_FEN, Board, square_index, square_name
from chessbot_brain.engine import choose_reply
from chessbot_brain.geometry import BoardGeometry


def test_square_indexing():
    assert square_index("a1") == 0
    assert square_index("h8") == 63
    assert square_name(square_index("e4")) == "e4"
    with pytest.raises(ValueError):
        square_index("i9")


def test_fen_round_trip():
    assert Board().fen() == START_FEN


def test_double_push_sets_en_passant():
    board = Board()
    board.apply("e2e4")
    assert board.fen() == "rnbqkbnr/pppppppp/8/8/4P3/8/PPPP1PPP/RNBQKBNR b KQkq e3 0 1"


def test_en_passant_capture_removes_the_pawn_behind():
    board = Board("4k3/8/8/3pP3/8/8/8/4K3 w - d6 0 1")
    effects = board.apply("e5d6")
    assert effects.capture_square == "d5"
    assert board.piece_at("d5") is None
    assert board.piece_at("d6") == "P"


def test_castling_moves_the_rook():
    board = Board("r3k2r/8/8/8/8/8/8/R3K2R w KQkq - 0 1")
    effects = board.apply("e1g1")
    assert (effects.rook_from, effects.rook_to) == ("h1", "f1")
    assert board.piece_at("f1") == "R"
    assert board.piece_at("g1") == "K"


def test_promotion():
    board = Board("4k3/P7/8/8/8/8/8/4K3 w - - 0 1")
    effects = board.apply("a7a8")
    assert effects.promotion == "Q"
    assert board.piece_at("a8") == "Q"


def test_placeholder_engine_only_moves_its_own_pieces():
    board = Board()
    board.apply("e2e4")
    reply = choose_reply(board, "black")
    assert reply is not None
    assert board.piece_at(reply[:2]).islower()


def test_geometry_square_centres_follow_the_calibration():
    geometry = BoardGeometry("base_link", (0.1288, 0.105, 0.0), -1.5707963267948966, 0.02625)
    x, y, _ = geometry.square_centre("a1")
    assert x == pytest.approx(0.1288 + 0.013125, abs=1e-6)
    assert y == pytest.approx(0.105 - 0.013125, abs=1e-6)
    x8, _, _ = geometry.square_centre("a8")
    assert x8 > x
