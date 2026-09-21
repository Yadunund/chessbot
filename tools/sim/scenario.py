#!/usr/bin/env python3
# Copyright 2026 Yadunund Vijay
# SPDX-License-Identifier: Apache-2.0
"""Run a robot action in simulation and measure what it did to the pieces.

Before each run, the board and pieces are respawned in Gazebo from the brain's
belief (optionally after setting a position). During the run, contact sensors
on the arm's links report every touch. Afterwards every piece is compared with
the believed position.

A run fails on:
- any touch between the robot and something other than the piece being moved,
  or between a non-gripper link and anything;
- a piece the belief says stayed put that moved more than --still-tol or tipped;
- a moved piece that is not within --place-tol of where the belief now puts it.

Examples (stack running in simulation):
    pixi run python tools/sim/scenario.py park
    pixi run python tools/sim/scenario.py transfer:e7e6
    pixi run python tools/sim/scenario.py robot:e2e4 --fen "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import gzsim  # noqa: E402
import pieces  # noqa: E402

BRAIN = os.environ.get("CHESSBOT_BRAIN", "http://127.0.0.1:8000")
ROUTER = os.environ.get("CHESSBOT_ROUTER", "http://127.0.0.1:8080")
CONTACT_TOPIC_SUFFIX = "_contact/contact"
GRIPPER_LINKS = ("gripper_link", "jaw_link")


# --- brain ------------------------------------------------------------------------------


def http(method: str, path: str, body: dict | None = None, base: str = BRAIN):
    data = json.dumps(body).encode() if body is not None else None
    request = urllib.request.Request(base + path, data=data, method=method, headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(request, timeout=10) as response:
        return json.loads(response.read() or b"null")


def state() -> dict:
    return http("GET", "/api/state")


def wait_idle(timeout_s: float = 600.0) -> dict:
    deadline = time.time() + timeout_s
    time.sleep(0.5)
    while time.time() < deadline:
        s = state()
        if not s["busy"]:
            return s
        time.sleep(0.5)
    raise TimeoutError("brain still busy")


def calibration() -> dict:
    [sample] = http("GET", "/chessbot/kv/calibration", base=ROUTER)
    value = sample["value"]
    return (json.loads(value) if isinstance(value, str) else value)["board"]


# --- world ------------------------------------------------------------------------------


def respawn(board: dict, belief: dict) -> list[tuple[str, str, tuple[float, float, float]]]:
    existing = [name for name in gzsim.model_poses() if name.startswith("pc_") or name == "board"]
    for name in existing:
        gzsim.remove(name)
    paths = pieces.write_models(board["square_size_m"])
    placed = pieces.layout(belief["fen"], belief.get("graveyard", []), board)
    models = [("board", paths["board"], tuple(board["origin_xyz"]), board["yaw_rad"])]
    # Pieces take the board's yaw, so knights face the opponent.
    models += [(name, paths[letter], xyz, board["yaw_rad"]) for name, letter, xyz in placed]
    gzsim.spawn(models)
    time.sleep(2.0)  # let pieces settle
    return placed


def tilt_deg(q) -> float:
    x, y, _, _ = q
    return math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y)))))


def set_piece(name: str, xyz):
    x, y, z = xyz
    gzsim.service(
        f"/world/{gzsim.WORLD}/set_pose/blocking",
        "gz.msgs.Pose",
        "gz.msgs.Boolean",
        f'name: "{name}" position {{ x: {x} y: {y} z: {z} }} orientation {{ w: 1 }}',
    )


# --- analysis ---------------------------------------------------------------------------


def summarise_contacts(messages: list[dict]) -> dict[tuple[str, str], dict]:
    """(robot link, other model) -> first/last sim time and count."""
    touches: dict[tuple[str, str], dict] = {}
    for msg in messages:
        t = gzsim.sim_time(msg)
        for contact in msg.get("contact", []):
            names = [contact.get("collision1", {}).get("name", ""), contact.get("collision2", {}).get("name", "")]
            robot = [n for n in names if n.startswith("so101::")]
            other = [n for n in names if not n.startswith("so101::")]
            if not robot:
                continue
            link = robot[0].split("::")[1]
            model = other[0].split("::")[0] if other else robot[-1].split("::")[1] + " (self)"
            entry = touches.setdefault((link, model), {"first": t, "last": t, "count": 0})
            entry["last"], entry["count"] = t, entry["count"] + 1
    return touches


def analyse(before_names, poses_before, poses_after, belief_before, belief_after, board, touches, args) -> list[str]:
    failures = []
    s = board["square_size_m"]
    before_slots = {(letter, tuple(round(v, 4) for v in xyz)) for _, letter, xyz in pieces.layout(belief_before["fen"], belief_before.get("graveyard", []), board)}
    after_layout = pieces.layout(belief_after["fen"], belief_after.get("graveyard", []), board)
    after_slots = {(letter, tuple(round(v, 4) for v in xyz)) for _, letter, xyz in after_layout}
    unchanged = before_slots & after_slots
    new_slots = [slot for slot in after_slots if slot not in unchanged]

    moving_models = set()
    for name, letter, xyz in before_names:
        if name not in poses_after:
            failures.append(f"{name}: missing from the world")
            continue
        start = poses_before[name][0]
        end, q = poses_after[name]
        slot = (letter, tuple(round(v, 4) for v in xyz))
        if slot in unchanged:
            moved = math.dist(start[:2], end[:2])
            if moved > args.still_tol or tilt_deg(q) > args.tilt_tol:
                failures.append(f"{name}: should stay, moved {moved * 1000:.1f} mm, tilted {tilt_deg(q):.0f} deg")
        else:
            moving_models.add(name)
            # The nearest newly believed slot for this piece letter.
            # A promoted pawn is still a pawn in the world (swapping pieces is not implemented).
            same = [sl for sl in new_slots if sl[0] == letter] or ([sl for sl in new_slots if sl[0].lower() in "qrbn"] if letter.lower() == "p" else [])
            candidates = [(math.dist(end[:2], sxyz[:2]), sxyz) for _, sxyz in same]
            if not candidates:
                if end[2] > -0.05:  # captured by the human: removed by the runner, or still on the board
                    failures.append(f"{name}: believed gone but still in the world at {end}")
                continue
            dist, target = min(candidates)
            # Error in the board frame (+x towards the h-file, +y towards rank 8).
            ex, ey = end[0] - target[0], end[1] - target[1]
            c, sn = math.cos(board["yaw_rad"]), math.sin(board["yaw_rad"])
            bx, by = c * ex + sn * ey, -sn * ex + c * ey
            detail = f"{dist * 1000:.1f} mm (board x {bx * 1000:+.1f}, y {by * 1000:+.1f}), tilted {tilt_deg(q):.0f} deg"
            if dist > args.place_tol or tilt_deg(q) > args.tilt_tol:
                failures.append(f"{name}: placed {detail}")
            else:
                print(f"  {name}: placed {detail}")

    for (link, model), info in sorted(touches.items()):
        allowed = model in moving_models and link in GRIPPER_LINKS
        if not allowed:
            failures.append(f"contact {link} <-> {model}: {info['count']} msgs, sim t {info['first']:.2f}-{info['last']:.2f}s")
    return failures


# --- main -------------------------------------------------------------------------------


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("action", help="park | transfer:<src><dst> | robot:<human move> | force:<robot move> | spawn")
    parser.add_argument("--fen", help="set this believed position first")
    parser.add_argument("--robot-side", default="black")
    parser.add_argument("--still-tol", type=float, default=0.002, help="m a stationary piece may move")
    parser.add_argument("--place-tol", type=float, default=0.004, help="m a moved piece may land off its square centre")
    parser.add_argument("--tilt-tol", type=float, default=5.0, help="deg")
    args = parser.parse_args()

    # Start every run from the park pose, so one failure cannot carry into the next.
    # The brain parks itself on startup, so let whatever it is doing finish first.
    wait_idle()
    http("POST", "/api/park")
    wait_idle()
    if args.fen:
        http("POST", "/api/dev/set_position", {"fen": args.fen, "robot_side": args.robot_side})
        wait_idle()
    board = calibration()
    belief_before = state()
    placed = respawn(board, belief_before)
    print(f"spawned {len(placed)} pieces from {belief_before['fen']}")
    if args.action == "spawn":
        return 0

    kind, _, arg = args.action.partition(":")
    poses_before = gzsim.model_poses()
    t0 = time.time()
    contact_topics = gzsim.topics(CONTACT_TOPIC_SUFFIX)
    if not contact_topics:
        raise SystemExit("no contact sensor topics: is the sim running the chessbot URDF with contact sensors?")
    with gzsim.Recorder(contact_topics) as recorder:
        if kind == "park":
            http("POST", "/api/park")
        elif kind == "force":
            http("POST", "/api/dev/robot_move", {"move": arg})
        elif kind == "transfer":
            http("POST", "/api/demo_transfer", {"src": arg[:2], "dst": arg[2:4]})
        elif kind == "robot":
            # Play the human's move in the world, then press the clock for the robot's reply.
            src, dst = arg[:2], arg[2:4]
            mover = next(name for name, _, _ in placed if name.startswith(f"pc_{src}_"))
            captured = [name for name, _, _ in placed if name.startswith(f"pc_{dst}_")]
            for name in captured:
                gzsim.remove(name)
            s = board["square_size_m"]
            f, r = "abcdefgh".index(dst[0]), int(dst[1]) - 1
            dst_xyz = pieces.board_to_world(board, (f + 0.5) * s, (r + 0.5) * s)
            set_piece(mover, dst_xyz)
            time.sleep(1.0)
            poses_before = gzsim.model_poses()
            placed = [(n, letter, dst_xyz if n == mover else xyz) for n, letter, xyz in placed if n not in captured]
            # Judge only the robot's reply: compare against the position after the human's move.
            from chessbot_brain.board import Board

            human = Board(belief_before["fen"])
            human.apply(arg)
            belief_before = {"fen": human.fen(), "graveyard": belief_before.get("graveyard", [])}
            http("POST", "/api/press_clock", {"move": arg})
        else:
            raise SystemExit(f"unknown action {args.action!r}")
        belief_after = wait_idle()
    elapsed = time.time() - t0
    time.sleep(1.0)
    poses_after = gzsim.model_poses()

    expected_after = belief_after
    if kind == "transfer":
        # A demo transfer moves a piece without changing the game, so expect it moved.
        from chessbot_brain.board import Board, square_index

        moved = Board(belief_before["fen"])
        moved.squares[square_index(arg[2:4])] = moved.squares[square_index(arg[:2])]
        moved.squares[square_index(arg[:2])] = None
        expected_after = {"fen": moved.fen(), "graveyard": belief_before.get("graveyard", [])}

    touches = summarise_contacts(recorder.messages)
    failures = analyse(placed, poses_before, poses_after, belief_before, expected_after, board, touches, args)
    last = belief_after["thoughts"][-3:] if belief_after.get("thoughts") else []
    print(f"action {args.action}: {elapsed:.0f} s wall, {len(recorder.messages)} contact messages, error: {belief_after.get('last_error') or '-'}")
    for t in last:
        print(f"  thought: {t.get('text')}")
    if failures:
        print(f"FAIL ({len(failures)})")
        for f in failures:
            print(f"  - {f}")
        return 1
    print("PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
