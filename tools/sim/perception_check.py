#!/usr/bin/env python3
# Copyright 2026 Yadunund Vijay
# SPDX-License-Identifier: Apache-2.0
"""Does perception read the move that was actually played?

Each round puts a random position on the simulated board, tells the brain that is what it
believes, plays one random legal move physically, and presses the clock without saying what
the move was. The brain has to name it from the camera alone.

Run the stack with the arm held still, so a round is only perception and move matching:

    pixi run sim play_moves:=false
    pixi run python tools/sim/perception_check.py --rounds 20
"""

from __future__ import annotations

import argparse
import os
import random
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from chessbot_brain.board import Board  # noqa: E402
from chessbot_brain.engine import UciEngine  # noqa: E402
from scenario import http, state, wait_idle  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
STOCKFISH = os.path.join(os.environ.get("PIXI_PROJECT_ROOT", ""), "generated", "bin", "stockfish")


def random_position(engine: UciEngine, rng: random.Random, plies: int) -> Board:
    board = Board()
    for _ in range(plies):
        moves = engine.legal_moves(board.fen())
        if not moves:
            break
        board.apply(rng.choice(moves))
    return board


def run_round(engine: UciEngine, rng: random.Random, plies: int, settle_s: float) -> tuple[bool, str]:
    board = random_position(engine, rng, plies)
    legal = engine.legal_moves(board.fen())
    if not legal:
        return True, "skipped: no legal move"
    fen = board.fen()
    robot = "black" if board.white_to_move else "white"

    subprocess.run([sys.executable, os.path.join(HERE, "scenario.py"), "spawn", "--fen", fen], check=True)
    http("POST", "/api/dev/set_position", {"fen": fen, "robot_side": robot})
    wait_idle()
    # Perception learns how the pieces look at the start of the human's turn.
    time.sleep(settle_s)

    move = rng.choice(legal)
    before = state()
    subprocess.run([sys.executable, os.path.join(HERE, "human_move.py"), move, "--press", "--no-move-hint"],
                   check=True)
    after = wait_idle(900)
    new = after["moves"][len(before["moves"]):]
    read = new[0] if new else None
    if read == move:
        return True, f"{move} read correctly"
    seen = [t["text"] for t in after["thoughts"] if t["kind"] == "perceive"][-1:]
    return False, f"played {move}, brain said {read or 'nothing'}" + (f"; {seen[0]}" if seen else "")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--rounds", type=int, default=10)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--plies", type=int, default=6, help="random plies used to build each position")
    parser.add_argument("--settle", type=float, default=3.0, help="seconds to let the camera settle")
    args = parser.parse_args()

    if not os.path.isfile(STOCKFISH):
        print(f"no engine at {STOCKFISH}; run `pixi run stockfish`")
        return 2
    engine = UciEngine(STOCKFISH)
    rng = random.Random(args.seed)
    passed = []
    for round_index in range(1, args.rounds + 1):
        ok, detail = run_round(engine, rng, rng.randint(0, args.plies), args.settle)
        passed.append(ok)
        print(f"[{round_index:3d}/{args.rounds}] {'READ ' if ok else 'MISS '} {detail}", flush=True)
    print(f"\n{sum(passed)}/{len(passed)} moves read from the camera")
    return 0 if all(passed) else 1


if __name__ == "__main__":
    sys.exit(main())
