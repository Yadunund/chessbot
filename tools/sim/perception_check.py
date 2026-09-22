#!/usr/bin/env python3
# Copyright 2026 Yadunund Vijay
# SPDX-License-Identifier: Apache-2.0
"""Does move detection read the move that was actually played?

Each round puts a random position on the simulated board, tells the brain that is what it
believes, plays one random legal move physically, and presses the clock without saying what
the move was. Whichever detector is running has to name it from the camera alone.

Run the stack with the arm held still, so a round is only perception and move detection:

    pixi run sim play_moves:=false
    pixi run python tools/sim/perception_check.py --rounds 20 --label classic

    pixi run sim play_moves:=false move_detection:=gemma
    pixi run python tools/sim/perception_check.py --rounds 20 --label gemma

Both runs use the same --seed, so both see the same positions and the same moves, and the
two accuracy numbers are comparable. `--out` writes the rounds to JSON; pass two of those
files to `--compare` to print them side by side.
"""

from __future__ import annotations

import argparse
import json
import os
import random
import statistics
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


def run_round(engine: UciEngine, rng: random.Random, plies: int, settle_s: float) -> dict:
    board = random_position(engine, rng, plies)
    legal = engine.legal_moves(board.fen())
    if not legal:
        return {"skipped": True, "detail": "no legal move"}
    fen = board.fen()
    robot = "black" if board.white_to_move else "white"

    subprocess.run([sys.executable, os.path.join(HERE, "scenario.py"), "spawn", "--fen", fen], check=True)
    # Let the pieces come to rest before the brain is told the position: that is when it
    # takes the "before" picture detection is given.
    time.sleep(settle_s)
    http("POST", "/api/dev/set_position", {"fen": fen, "robot_side": robot})
    wait_idle()
    # Perception learns how the pieces look at the start of the human's turn.
    time.sleep(settle_s)

    move = rng.choice(legal)
    before = state()
    started = time.time()
    subprocess.run([sys.executable, os.path.join(HERE, "human_move.py"), move, "--press", "--no-move-hint"],
                   check=True)
    after = wait_idle(900)
    elapsed = time.time() - started
    new = after["moves"][len(before["moves"]):]
    read = new[0] if new else None
    said = [t["text"] for t in after["thoughts"] if t["kind"] == "perceive"][-1:]
    return {
        "skipped": False,
        "fen": fen,
        "played": move,
        "read": read,
        "correct": read == move,
        # Wall clock from playing the move to the brain being idle again: it includes the
        # simulation settling, so compare detectors with it, not the arm.
        "seconds": elapsed,
        "said": said[0] if said else "",
    }


def summarise(label: str, rounds: list[dict]) -> dict:
    played = [r for r in rounds if not r["skipped"]]
    correct = [r for r in played if r["correct"]]
    # A wrong move is a different failure from no move: one puts the game out of step with
    # the board, the other only asks the player to type it.
    wrong = [r for r in played if not r["correct"] and r["read"]]
    seconds = [r["seconds"] for r in played]
    return {
        "label": label,
        "rounds": len(played),
        "correct": len(correct),
        "wrong": len(wrong),
        "missed": len(played) - len(correct) - len(wrong),
        "accuracy": len(correct) / len(played) if played else 0.0,
        "median_s": statistics.median(seconds) if seconds else 0.0,
    }


def print_summary(summary: dict):
    print(f"\n{summary['label']}: {summary['correct']}/{summary['rounds']} read correctly "
          f"({summary['accuracy']:.0%}), {summary['wrong']} read as the wrong move, "
          f"{summary['missed']} not read at all, median {summary['median_s']:.1f} s per move")


def compare(paths: list[str]) -> int:
    summaries = []
    for path in paths:
        with open(path) as handle:
            saved = json.load(handle)
        summaries.append(saved["summary"])
    width = max(len(s["label"]) for s in summaries)
    print(f"{'detector'.ljust(width)}  correct    wrong   missed  accuracy  median")
    for s in summaries:
        print(f"{s['label'].ljust(width)}  {s['correct']:7d}  {s['wrong']:7d}  {s['missed']:7d}  "
              f"{s['accuracy']:7.0%}  {s['median_s']:5.1f} s")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--rounds", type=int, default=10)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--plies", type=int, default=6, help="random plies used to build each position")
    parser.add_argument("--settle", type=float, default=3.0, help="seconds to let the camera settle")
    parser.add_argument("--label", default="detector", help="what to call this run in the summary")
    parser.add_argument("--out", help="write the rounds and the summary here as JSON")
    parser.add_argument("--compare", nargs="+", metavar="JSON", help="print saved runs side by side and exit")
    args = parser.parse_args()

    if args.compare:
        return compare(args.compare)
    if not os.path.isfile(STOCKFISH):
        print(f"no engine at {STOCKFISH}; run `pixi run stockfish`")
        return 2
    engine = UciEngine(STOCKFISH)
    rng = random.Random(args.seed)
    rounds = []
    for index in range(1, args.rounds + 1):
        result = run_round(engine, rng, rng.randint(0, args.plies), args.settle)
        rounds.append(result)
        if result["skipped"]:
            print(f"[{index:3d}/{args.rounds}] SKIP  {result['detail']}", flush=True)
            continue
        mark = "READ " if result["correct"] else "MISS "
        detail = (f"{result['played']} read correctly" if result["correct"]
                  else f"played {result['played']}, brain said {result['read'] or 'nothing'}"
                       + (f"; {result['said']}" if result["said"] else ""))
        print(f"[{index:3d}/{args.rounds}] {mark} ({result['seconds']:5.1f} s) {detail}", flush=True)

    summary = summarise(args.label, rounds)
    print_summary(summary)
    if args.out:
        with open(args.out, "w") as handle:
            json.dump({"summary": summary, "rounds": rounds, "seed": args.seed}, handle, indent=2)
        print(f"written to {args.out}")
    return 0 if summary["correct"] == summary["rounds"] else 1


if __name__ == "__main__":
    sys.exit(main())
