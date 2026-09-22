# Copyright 2026 Yadunund Vijay
# SPDX-License-Identifier: Apache-2.0
"""Move detection from per-square occupancy.

Perception reports, per square, empty / white / black with a confidence. The rules say
which moves are legal. The move is the legal one whose resulting occupancy best matches
the observation, provided it matches well and clearly better than the next candidate.

When two candidates fit equally well, the vision-language model behind
``/inference/reason`` is asked to look at the picture and choose between them, and is
believed only if it also declines a decoy. See `_tiebreak`.
"""

from __future__ import annotations

import rclpy
from chessbot_brain.board import Board

from chessbot_interfaces.srv import DetectMove, Reason
from chessbot_move_detection.detector import MoveDetector, answer, spin
from chessbot_move_detection.model_client import ModelClient
from chessbot_move_detection.move_reading import read_move


class ClassicDetector(MoveDetector):
    def __init__(self):
        super().__init__("classic_move_detector")
        self.accept_mismatch = float(self.declare_parameter("accept_mismatch", 0.6).value)
        self.margin = float(self.declare_parameter("margin", 0.8).value)
        self.tiebreak = bool(self.declare_parameter("tiebreak_with_model", True).value)
        self.tiebreak_confidence = float(self.declare_parameter("tiebreak_confidence", 0.7).value)
        self.model = ModelClient(self, float(self.declare_parameter("model_timeout_s", 60.0).value))

    def detect(self, request: DetectMove.Request) -> DetectMove.Response:
        observation = request.after.state
        if not observation.board_detected:
            return answer(DetectMove.Response.RESULT_NO_BOARD, "I can't see the board")

        board = Board(request.fen_before)
        legal = list(request.legal_moves)
        reading = read_move(
            board,
            legal,
            list(observation.squares),
            list(observation.confidence),
            accept_mismatch=self.accept_mismatch,
            margin=self.margin,
        )
        if reading.move is not None:
            return answer(
                DetectMove.Response.RESULT_OK,
                reading.reason,
                move=reading.move,
                confidence=1.0,
                candidates=reading.candidates,
            )

        if self.tiebreak and len(reading.candidates) >= 2 and reading.changed:
            choice, confidence, why = self._tiebreak(
                board, request.after.image, [move for move, _ in reading.candidates], legal
            )
            if choice is not None:
                return answer(
                    DetectMove.Response.RESULT_OK,
                    why,
                    move=choice,
                    confidence=confidence,
                    candidates=reading.candidates,
                    model_used=True,
                )
            return answer(
                DetectMove.Response.RESULT_UNCLEAR,
                f"{reading.reason}, and {why}",
                candidates=reading.candidates,
                model_used=True,
            )
        return answer(DetectMove.Response.RESULT_UNCLEAR, reading.reason, candidates=reading.candidates)

    # --- the model as a tiebreaker ---------------------------------------------------

    def _ask_which_move(self, board: Board, image, candidates: list[str]) -> tuple[str | None, float]:
        """Ask the model which of `candidates` the image shows. Returns (move, confidence)."""
        schema = {
            "type": "object",
            "properties": {"move": {"enum": [*candidates, "unclear"]}, "confidence": {"type": "number"}},
            "required": ["move", "confidence"],
        }
        prompt = (
            f"The image is the overhead camera view of a chess board. Before the human's move the position was "
            f"(FEN) {board.fen()}. {'White' if board.white_to_move else 'Black'} just moved. Which of these moves "
            f"(UCI notation) does the image show: {', '.join(candidates)}? Answer 'unclear' if you cannot tell."
        )
        parsed = self.model.ask(Reason.Request.ROLE_MOVE_TIEBREAK, prompt, [image], schema)
        if parsed is None:
            return None, 0.0
        return parsed.get("move"), float(parsed.get("confidence", 0.0))

    def _tiebreak(self, board: Board, image, candidates: list[str], legal: list[str]) -> tuple[str | None, float, str]:
        """The model picks which candidate the image shows, if it can be shown to be looking.

        A vision model asked to choose between plausible chess moves can answer from what the
        opening book makes likely rather than from the image, and sound no less sure for it. So
        its answer only counts if it also declines a decoy - the same question with every real
        candidate replaced by moves that are legal but were not played. A model that picks one
        of those is guessing, and is not trusted for this frame.
        """
        if not image.data:
            return None, 0.0, "there was no picture to show the model"
        move, confidence = self._ask_which_move(board, image, candidates)
        if move is None and confidence == 0.0:
            return None, 0.0, "the model isn't available to look"
        if move not in candidates or confidence < self.tiebreak_confidence:
            return None, 0.0, f"the model couldn't tell either ({move}, confidence {confidence:.2f})"

        decoys = [m for m in legal if m not in candidates][: len(candidates)]
        if decoys:
            decoy_move, decoy_confidence = self._ask_which_move(board, image, decoys)
            if decoy_move in decoys and decoy_confidence >= self.tiebreak_confidence:
                return (
                    None,
                    0.0,
                    (
                        f"the model answered {move} but also picked {decoy_move} from moves that were not played, "
                        "so it is guessing rather than looking"
                    ),
                )
        return move, confidence, f"the model says you played {move} (confidence {confidence:.2f})"


def main():
    rclpy.init()
    spin(ClassicDetector())


if __name__ == "__main__":
    main()
