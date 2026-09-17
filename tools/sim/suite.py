#!/usr/bin/env python3
# Copyright 2026 Yadunund Vijay
# SPDX-License-Identifier: Apache-2.0
"""Run the motion scenarios in simulation and print a pass/fail table.

Start the stack first, ideally without cameras (much faster):
    pixi run sim cameras:=false viewer:=none
    pixi run sim-test            # or: pixi run sim-test castle en_passant
"""

import os
import subprocess
import sys
import time

START = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"
ITALIAN = "r1bqk2r/pppp1ppp/2n2n2/2b1p3/2B1P3/3P1N2/PPP2PPP/RNBQK2R w KQkq - 3 5"

# name: (action, believed position)
SCENARIOS = {
    "park": ("park", START),
    "pawn_e7e5": ("transfer:e7e5", START),
    "knight_g8f6": ("transfer:g8f6", START),
    "knight_b8c6": ("transfer:b8c6", START),
    "edge_a7a6": ("transfer:a7a6", START),
    "edge_h7h6": ("transfer:h7h6", START),
    "far_a2a3": ("transfer:a2a3", START),
    "far_h2h3": ("transfer:h2h3", START),
    "reply_to_e2e4": ("robot:e2e4", START),
    "capture_reply": ("robot:e4d5", "rnbqkbnr/ppp1pppp/8/3p4/4P3/8/PPPP1PPP/RNBQKBNR w KQkq - 0 2"),
    "middlegame_capture": ("robot:f3e5", ITALIAN),
    "castle": ("force:e8g8", "rnbqk2r/pppp1ppp/5n2/2b1p3/2B1P3/5N2/PPPP1PPP/RNBQK2R b KQkq - 4 4"),
    "castle_long": ("force:e8c8", "r3kbnr/pppqpppp/2n5/3p1b2/3P1B2/2N5/PPPQPPPP/R3KBNR b KQkq - 6 5"),
    "en_passant": ("force:d4e3", "rnbqkbnr/ppp1pppp/8/8/3pP3/8/PPPP1PPP/RNBQKBNR b KQkq e3 0 3"),
    "promotion": ("force:b2b1q", "4k3/8/8/8/8/8/1p6/4K3 b - - 0 1"),
}


def main():
    names = sys.argv[1:] or list(SCENARIOS)
    here = os.path.dirname(os.path.abspath(__file__))
    rows = []
    for name in names:
        action, fen = SCENARIOS[name]
        t0 = time.time()
        out = subprocess.run(
            [sys.executable, os.path.join(here, "scenario.py"), action, "--fen", fen], capture_output=True, text=True
        )
        lines = out.stdout.splitlines()
        passed = out.returncode == 0 and "PASS" in lines
        details = [line.strip()[2:] for line in lines if line.strip().startswith("- ")]
        errors = [line.split("error: ", 1)[1] for line in lines if line.startswith("action ") and "error: " in line]
        details += [f"brain: {e}" for e in errors if e != "-"]
        if out.returncode not in (0, 1):
            details.append((out.stderr.strip().splitlines() or ["crashed"])[-1])
        rows.append((name, passed, time.time() - t0, details))
        print(f"{'PASS' if passed else 'FAIL'}  {name:<20} {time.time() - t0:5.0f} s", flush=True)
        for detail in details:
            print(f"        {detail}", flush=True)
    failed = [name for name, passed, _, _ in rows if not passed]
    print(f"\n{len(rows) - len(failed)}/{len(rows)} scenarios passed" + (f"; failed: {', '.join(failed)}" if failed else ""))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
