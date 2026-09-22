# Copyright 2026 Yadunund Vijay
# SPDX-License-Identifier: Apache-2.0
"""Both detectors answering DetectMove, without a camera, a model or a router.

The detectors are called directly rather than over the service, so these run anywhere.
What they check is the contract: every outcome a caller has to handle comes back as the
right result code with the move field filled in or left empty to match.
"""

import pytest
import rclpy
from chessbot_brain.board import Board
from chessbot_move_detection.classic_detector import ClassicDetector
from chessbot_move_detection.gemma_detector import GemmaDetector
from chessbot_move_detection.move_reading import occupancy

from chessbot_interfaces.srv import DetectMove

START = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"


@pytest.fixture(scope="module")
def ros():
    rclpy.init()
    yield
    rclpy.try_shutdown()


@pytest.fixture
def classic(ros):
    node = ClassicDetector()
    yield node
    node.destroy_node()


@pytest.fixture
def gemma(ros):
    node = GemmaDetector()
    yield node
    node.destroy_node()


def request_after(fen: str, move: str, legal: list[str], confidence: float = 0.8) -> DetectMove.Request:
    """A request whose `after` observation is the position with `move` played."""
    played = Board(fen)
    played.apply(move)
    request = DetectMove.Request(fen_before=fen, legal_moves=legal)
    request.after.state.squares = occupancy(played)
    request.after.state.confidence = [confidence] * 64
    request.after.state.board_detected = True
    return request


def test_classic_reads_the_move_that_was_played(classic):
    response = classic.detect(request_after(START, "e2e4", ["e2e4", "e2e3", "d2d4", "g1f3"]))
    assert response.result == DetectMove.Response.RESULT_OK
    assert response.move == "e2e4"
    assert not response.model_used


def test_classic_says_so_when_the_board_was_not_read(classic):
    request = DetectMove.Request(fen_before=START, legal_moves=["e2e4"])
    response = classic.detect(request)
    assert response.result == DetectMove.Response.RESULT_NO_BOARD
    assert response.move == ""


def test_classic_will_not_name_a_move_it_cannot_tell_apart(classic):
    """Two candidates fit, no picture to settle it: unclear, not a guess."""
    fen = "4k3/8/8/8/8/8/4P3/4K3 w - - 0 1"
    request = request_after(fen, "e2e4", ["e2e4", "e2e3"])
    # Perception saw e2 empty but could not read either square the pawn might be on.
    request.after.state.squares[28] = 3  # e4 unknown
    request.after.state.squares[20] = 3  # e3 unknown
    response = classic.detect(request)
    assert response.result == DetectMove.Response.RESULT_UNCLEAR
    assert response.move == ""


def test_a_request_without_a_position_is_a_failure_not_a_crash(classic):
    response = classic._on_detect(DetectMove.Request(), DetectMove.Response())
    assert response.result == DetectMove.Response.RESULT_FAILED


def test_the_service_times_every_answer(classic):
    response = classic._on_detect(request_after(START, "e2e4", ["e2e4", "d2d4"]), DetectMove.Response())
    assert response.latency_s > 0.0


def test_gemma_needs_a_picture(gemma):
    response = gemma.detect(request_after(START, "e2e4", ["e2e4", "d2d4"]))
    assert response.result == DetectMove.Response.RESULT_NO_BOARD


def test_gemma_reports_the_model_being_down_rather_than_falling_back(gemma):
    """The two detectors are compared against each other, so neither may stand in for the other."""
    request = request_after(START, "e2e4", ["e2e4", "d2d4"])
    request.after.image.format = "png"
    request.after.image.data = b"not really a png, but there is a picture"
    response = gemma.detect(request)
    assert response.result == DetectMove.Response.RESULT_FAILED
    assert response.move == ""
