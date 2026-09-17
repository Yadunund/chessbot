# Copyright 2026 Yadunund Vijay
# SPDX-License-Identifier: Apache-2.0
from chessbot_brain.board import Board
from chessbot_brain.move_reading import UNKNOWN, occupancy, read_move

START = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"


def legal(fen: str, moves: list[str]) -> list[str]:
    # The rules come from Stockfish in the brain; these tests supply the relevant ones.
    return moves


def seen_after(fen: str, move: str, confidence: float = 0.8):
    board = Board(fen)
    board.apply(move)
    return occupancy(board), [confidence] * 64


def test_reads_a_simple_move():
    observed, conf = seen_after(START, "e2e4")
    reading = read_move(Board(START), ["e2e4", "e2e3", "d2d4", "g1f3"], observed, conf)
    assert reading.move == "e2e4"
    assert set(reading.changed) == {"e2", "e4"}


def test_reads_castling_rather_than_a_king_move():
    fen = "rnbqk2r/pppp1ppp/5n2/2b1p3/2B1P3/5N2/PPPP1PPP/RNBQK2R w KQkq - 4 4"
    observed, conf = seen_after(fen, "e1g1")
    reading = read_move(Board(fen), ["e1g1", "e1f1", "e1e2", "h1g1"], observed, conf)
    assert reading.move == "e1g1"


def test_reads_en_passant():
    fen = "rnbqkbnr/ppp1p1pp/8/3pPp2/8/8/PPPP1PPP/RNBQKBNR w KQkq f6 0 3"
    observed, conf = seen_after(fen, "e5f6")
    reading = read_move(Board(fen), ["e5f6", "e5e6", "d2d4"], observed, conf)
    assert reading.move == "e5f6"


def test_promotions_collapse_to_one_candidate():
    fen = "8/4P3/8/8/8/8/k7/4K3 w - - 0 1"
    observed, conf = seen_after(fen, "e7e8q")
    reading = read_move(Board(fen), ["e7e8q", "e7e8r", "e7e8b", "e7e8n", "e1d1"], observed, conf)
    assert reading.move == "e7e8q"


def test_unchanged_board_is_not_a_move():
    board = Board(START)
    reading = read_move(board, ["e2e4"], occupancy(board), [0.9] * 64)
    assert reading.move is None and "unchanged" in reading.reason


def test_unexplained_change_is_rejected():
    board = Board(START)
    observed = occupancy(board)
    observed[12] = 0  # e2 emptied, nothing appeared anywhere
    observed[20] = 0
    observed[36] = 2  # a black piece on e5 out of nowhere
    reading = read_move(board, ["e2e4", "e2e3"], observed, [0.9] * 64)
    assert reading.move is None


def test_unknown_squares_do_not_count_against_a_move():
    observed, conf = seen_after(START, "g1f3")
    observed[0] = UNKNOWN  # a1 could not be seen
    reading = read_move(Board(START), ["g1f3", "g1h3", "e2e4"], observed, conf)
    assert reading.move == "g1f3"
