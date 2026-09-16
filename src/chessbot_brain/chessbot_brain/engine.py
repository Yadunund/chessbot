# Copyright 2026 Yadunund Vijay
# SPDX-License-Identifier: Apache-2.0
"""Chess engine access.

``UciEngine`` drives Stockfish as a separate process over UCI. Stockfish is
GPL-3.0, so it is never linked or vendored: it is built from unmodified upstream
source (``pixi run stockfish``) and only spoken to over stdin/stdout. Besides
choosing moves it answers the rules questions the brain needs, using standard
engine commands: ``go perft 1`` lists the legal moves, ``d`` reports checkers.

``choose_reply`` is the fallback when no engine binary is available: a short
book line, then the first plausible move. It checks no rules.
"""

from __future__ import annotations

import queue
import re
import subprocess
import threading
import time

from chessbot_brain.board import Board, piece_colour, square_name


class EngineError(RuntimeError):
    pass


# One line of `go perft 1` output per legal move, e.g. "e7e8q: 1".
_PERFT_LINE = re.compile(r"^([a-h][1-8][a-h][1-8][qrbn]?): \d+$")


class UciEngine:
    def __init__(self, path: str, threads: int = 2):
        self._proc = subprocess.Popen(
            [path], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1
        )
        self._lines: queue.Queue[str] = queue.Queue()
        self._lock = threading.Lock()
        threading.Thread(target=self._pump, daemon=True).start()
        with self._lock:
            self._send("uci")
            self._read_until(lambda line: line == "uciok", 10.0)
            self._send(f"setoption name Threads value {threads}")
            self._sync()

    def _pump(self):
        assert self._proc.stdout is not None
        for line in self._proc.stdout:
            self._lines.put(line.rstrip("\n"))

    def _send(self, command: str):
        if self._proc.poll() is not None:
            raise EngineError("engine process exited")
        assert self._proc.stdin is not None
        self._proc.stdin.write(command + "\n")
        self._proc.stdin.flush()

    def _read_until(self, done, timeout_s: float) -> list[str]:
        lines = []
        deadline = time.monotonic() + timeout_s
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise EngineError("engine did not respond in time")
            try:
                line = self._lines.get(timeout=remaining)
            except queue.Empty as exc:
                raise EngineError("engine did not respond in time") from exc
            lines.append(line)
            if done(line):
                return lines

    def _sync(self):
        self._send("isready")
        self._read_until(lambda line: line == "readyok", 10.0)

    def legal_moves(self, fen: str) -> list[str]:
        with self._lock:
            self._send(f"position fen {fen}")
            self._send("go perft 1")
            lines = self._read_until(lambda line: line.startswith("Nodes searched"), 10.0)
            self._sync()
        # Other output (e.g. "info string ...") also contains colons, so match strictly.
        return [m.group(1) for m in map(_PERFT_LINE.match, lines) if m]

    def in_check(self, fen: str) -> bool:
        with self._lock:
            self._send(f"position fen {fen}")
            self._send("d")
            lines = self._read_until(lambda line: line.startswith("Checkers:"), 10.0)
            self._sync()
        return lines[-1].split(":", 1)[1].strip() != ""

    def best_move(self, fen: str, movetime_ms: int, elo: int = 0) -> str:
        with self._lock:
            if elo > 0:
                self._send("setoption name UCI_LimitStrength value true")
                self._send(f"setoption name UCI_Elo value {elo}")
            else:
                self._send("setoption name UCI_LimitStrength value false")
            self._send(f"position fen {fen}")
            self._send(f"go movetime {movetime_ms}")
            lines = self._read_until(lambda line: line.startswith("bestmove"), movetime_ms / 1000.0 + 10.0)
        move = lines[-1].split()[1]
        if move == "(none)":
            raise EngineError("no legal move")
        return move

    def close(self):
        try:
            self._send("quit")
            self._proc.wait(timeout=2.0)
        except Exception:  # noqa: BLE001 - best effort
            self._proc.kill()


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
    """Fallback move choice without an engine. Checks no rules."""
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
