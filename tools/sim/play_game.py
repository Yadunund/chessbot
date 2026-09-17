#!/usr/bin/env python3
# Copyright 2026 Yadunund Vijay
# SPDX-License-Identifier: Apache-2.0
"""Play human moves in simulation and check the robot reads each one from its camera.

For each move: the pieces are moved in Gazebo, the clock is pressed without saying
the move, and the brain's move list is checked. The robot replies physically in
between, so later moves are read from a board the robot has changed.

    pixi run python tools/sim/play_game.py e2e4 g1f3 f1c4
"""

from __future__ import annotations

import os
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from scenario import http, state, wait_idle  # noqa: E402

START = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"
HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    moves = sys.argv[1:] or ["e2e4", "g1f3", "f1c4"]
    # Pieces first, then the new game: perception learns the board's appearance when
    # the human's turn begins, so the pieces must already be there.
    subprocess.run([sys.executable, os.path.join(HERE, "scenario.py"), "spawn", "--fen", START], check=True)
    http("POST", "/api/new_game", {"robot_side": "black", "engine_elo": 0})
    wait_idle()
    time.sleep(3.0)

    read = 0
    for move in moves:
        before = state()
        if before["phase"] != "human_turn":
            print(f"stopped: phase is {before['phase']} ({before.get('last_error') or 'no error'})")
            break
        subprocess.run([sys.executable, os.path.join(HERE, "human_move.py"), move, "--press", "--no-move-hint"], check=True)
        after = wait_idle(900)
        new_moves = after["moves"][len(before["moves"]):]
        perceived = [t["text"] for t in after["thoughts"] if t["kind"] == "perceive"][-1:]
        ok = bool(new_moves) and new_moves[0] == move
        read += ok
        print(f"{'READ ' if ok else 'MISS '} {move}: brain recorded {new_moves or 'nothing'}; {perceived[0] if perceived else ''}")
        time.sleep(3.0)
    print(f"\n{read}/{len(moves)} human moves read from the camera")
    return 0 if read == len(moves) else 1


if __name__ == "__main__":
    sys.exit(main())
