# Copyright 2026 Yadunund Vijay
# SPDX-License-Identifier: Apache-2.0
"""Move detection by asking a vision-language model to compare two photographs.

The model is shown the board before and after the human's move and asked the one
question it is actually equipped to answer: which square did a piece leave, and which
square did it arrive on. It is never asked to name a move. Naming a move invites it to
answer from the opening book rather than from the pixels, and such an answer reads no
differently from one taken off the board.

The squares it may name are the squares the rules allow, so the answer is resolved
back to a legal move here, deterministically. An answer is only believed if the model
also declines the same question with the true squares taken out of the choices: a model
that confidently picks a square it has already been shown is empty is guessing.
"""

from __future__ import annotations

import rclpy
from chessbot_brain.board import Board

from chessbot_interfaces.srv import DetectMove, Reason
from chessbot_move_detection.detector import MoveDetector, answer, spin
from chessbot_move_detection.model_client import ModelClient
from chessbot_move_detection.move_reading import moves_between

UNCLEAR = "unclear"


def squares_schema(sources: list[str], destinations: list[str]) -> dict:
    return {
        "type": "object",
        "properties": {
            "from_square": {"enum": [*sources, UNCLEAR]},
            "to_square": {"enum": [*destinations, UNCLEAR]},
            "confidence": {"type": "number"},
        },
        "required": ["from_square", "to_square", "confidence"],
    }


class GemmaDetector(MoveDetector):
    def __init__(self):
        super().__init__("gemma_move_detector")
        self.min_confidence = float(self.declare_parameter("min_confidence", 0.6).value)
        self.decoy_guard = bool(self.declare_parameter("decoy_guard", True).value)
        self.model = ModelClient(self, float(self.declare_parameter("model_timeout_s", 120.0).value))

    def detect(self, request: DetectMove.Request) -> DetectMove.Response:
        if not request.after.image.data:
            return answer(DetectMove.Response.RESULT_NO_BOARD, "there is no picture of the board to look at")
        if not self.model.available():
            return answer(DetectMove.Response.RESULT_FAILED, "the reasoning model isn't running")

        legal = list(request.legal_moves)
        sources = sorted({move[:2] for move in legal})
        destinations = sorted({move[2:4] for move in legal})
        board = Board(request.fen_before)

        source, destination, confidence = self._ask(request, board, sources, destinations)
        if source is None:
            return answer(DetectMove.Response.RESULT_FAILED, "the model didn't answer")
        if source == UNCLEAR or destination == UNCLEAR:
            return answer(
                DetectMove.Response.RESULT_UNCLEAR, "the model couldn't tell which piece moved", model_used=True
            )
        if confidence < self.min_confidence:
            return answer(
                DetectMove.Response.RESULT_UNCLEAR,
                f"the model read {source} to {destination} but wasn't sure (confidence {confidence:.2f})",
                model_used=True,
            )

        candidates = moves_between(legal, source, destination)
        if not candidates:
            return answer(
                DetectMove.Response.RESULT_UNCLEAR,
                f"the model read {source} to {destination}, which is not a legal move here",
                model_used=True,
            )
        if len(candidates) > 1:
            return answer(
                DetectMove.Response.RESULT_UNCLEAR,
                f"{source} to {destination} is more than one move: {', '.join(candidates)}",
                candidates=[(move, 0.0) for move in candidates],
                model_used=True,
            )

        if self.decoy_guard:
            picked = self._decoy(request, board, sources, destinations, source, destination)
            if picked is not None:
                return answer(
                    DetectMove.Response.RESULT_UNCLEAR,
                    f"the model read {source} to {destination}, but with those squares taken out of the choices "
                    f"it just as confidently read {picked}, so it is guessing rather than looking",
                    model_used=True,
                )
        move = candidates[0]
        return answer(
            DetectMove.Response.RESULT_OK,
            f"the model saw a piece go from {source} to {destination} (confidence {confidence:.2f})",
            move=move,
            confidence=confidence,
            candidates=[(move, 0.0)],
            model_used=True,
        )

    # --- the question ----------------------------------------------------------------

    def _ask(self, request, board: Board, sources: list[str], destinations: list[str]):
        """(from_square, to_square, confidence), each None when the model did not answer."""
        images = [request.before.image, request.after.image]
        if request.before.image.data:
            pictures = (
                "Two overhead photographs of the same chess board: the first was taken before the "
                "human's move, the second after it."
            )
        else:
            images = [request.after.image]
            pictures = "An overhead photograph of a chess board, taken after the human's move."
        prompt = (
            f"{pictures} Before the move the position was (FEN) {board.fen()}, which is what the board looked "
            f"like in the first picture; use it to work out which square is which. "
            f"{'White' if board.white_to_move else 'Black'} then made exactly one move. Compare the pictures "
            "square by square. Name the square the moved piece left (from_square) and the square it arrived on "
            "(to_square). For castling, name the king's squares. For a capture, to_square is the square the "
            "captured piece was on. Answer 'unclear' for either square you cannot read from the pictures; do not "
            "answer from what would be a good move."
        )
        parsed = self.model.ask(
            Reason.Request.ROLE_MOVE_TIEBREAK, prompt, images, squares_schema(sources, destinations)
        )
        if parsed is None:
            return None, None, 0.0
        return parsed.get("from_square"), parsed.get("to_square"), float(parsed.get("confidence", 0.0))

    def _decoy(self, request, board: Board, sources, destinations, source: str, destination: str) -> str | None:
        """Ask again without the squares it named. What it picks instead, if it picks confidently."""
        decoy_sources = [square for square in sources if square != source]
        decoy_destinations = [square for square in destinations if square != destination]
        if not decoy_sources or not decoy_destinations:
            return None
        picked_from, picked_to, confidence = self._ask(request, board, decoy_sources, decoy_destinations)
        if picked_from in (None, UNCLEAR) or picked_to in (None, UNCLEAR):
            return None
        if confidence < self.min_confidence:
            return None
        return f"{picked_from} to {picked_to}"


def main():
    rclpy.init()
    spin(GemmaDetector())


if __name__ == "__main__":
    main()
