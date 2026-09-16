# Copyright 2026 Yadunund Vijay
# SPDX-License-Identifier: Apache-2.0
"""Board and piece models for simulation, placed from the brain's belief.

Pieces are cylinders with the real set's base diameter and heights: a
conservative stand-in for turned pieces, which are no wider than their base.
"""

from __future__ import annotations

import math
import os

BASE_DIAMETER = 0.015
HEIGHT = {"k": 0.042, "q": 0.037, "b": 0.033, "n": 0.030, "r": 0.026, "p": 0.022}
MASS = {"k": 0.020, "q": 0.018, "b": 0.014, "n": 0.014, "r": 0.013, "p": 0.010}
FILES = "abcdefgh"
MODEL_DIR = "/tmp/chessbot_sim_models"


def piece_sdf(letter: str) -> str:
    kind = letter.lower()
    r, h, m = BASE_DIAMETER / 2, HEIGHT[kind], MASS[kind]
    ixx = m * (3 * r * r + h * h) / 12
    izz = m * r * r / 2
    colour = "0.95 0.93 0.9 1" if letter.isupper() else "0.15 0.14 0.13 1"
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="piece">
    <link name="link">
      <pose>0 0 {h / 2} 0 0 0</pose>
      <inertial>
        <mass>{m}</mass>
        <inertia><ixx>{ixx}</ixx><iyy>{ixx}</iyy><izz>{izz}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <collision name="collision">
        <geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>
        <surface><friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction></surface>
      </collision>
      <visual name="visual">
        <geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>
        <material><ambient>{colour}</ambient><diffuse>{colour}</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>
"""


def board_sdf(square: float) -> str:
    """Visual-only squares (no collision): pieces stand on the table surface, as in calibration z = 0."""
    visuals = []
    for rank in range(8):
        for file in range(8):
            colour = "0.46 0.42 0.37 1" if (file + rank) % 2 == 0 else "0.93 0.91 0.88 1"
            visuals.append(
                f'<visual name="sq_{file}_{rank}"><pose>{(file + 0.5) * square} {(rank + 0.5) * square} 0.0005 0 0 0</pose>'
                f"<geometry><box><size>{square} {square} 0.001</size></box></geometry>"
                f"<material><ambient>{colour}</ambient><diffuse>{colour}</diffuse></material></visual>"
            )
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="board">
    <static>true</static>
    <link name="link">
      {"".join(visuals)}
    </link>
  </model>
</sdf>
"""


def write_models(square: float) -> dict[str, str]:
    """Writes one SDF per piece letter and the board; returns their paths."""
    os.makedirs(MODEL_DIR, exist_ok=True)
    paths = {}
    for letter in "KQBNRPkqbnrp":
        path = os.path.join(MODEL_DIR, f"piece_{'w' if letter.isupper() else 'b'}{letter.lower()}.sdf")
        with open(path, "w") as f:
            f.write(piece_sdf(letter))
        paths[letter] = path
    path = os.path.join(MODEL_DIR, "board.sdf")
    with open(path, "w") as f:
        f.write(board_sdf(square))
    paths["board"] = path
    return paths


def board_to_world(board: dict, bx: float, by: float) -> tuple[float, float, float]:
    ox, oy, oz = board["origin_xyz"]
    c, s = math.cos(board["yaw_rad"]), math.sin(board["yaw_rad"])
    return (ox + c * bx - s * by, oy + s * bx + c * by, oz)


def layout(fen: str, graveyard: list[dict], board: dict) -> list[tuple[str, str, tuple[float, float, float]]]:
    """(model name, piece letter, xyz) for every believed piece. Names say where the piece started."""
    s = board["square_size_m"]
    out = []
    for i, row in enumerate(fen.split()[0].split("/")):
        file = 0
        for ch in row:
            if ch.isdigit():
                file += int(ch)
                continue
            square = f"{FILES[file]}{8 - i}"
            out.append((f"pc_{square}_{ch}", ch, board_to_world(board, (file + 0.5) * s, (8 - i - 0.5) * s)))
            file += 1
    for n, entry in enumerate(graveyard):
        cx, cy = entry["cell"]
        out.append((f"pc_grave{n}_{entry['piece']}", entry["piece"], board_to_world(board, cx * s, cy * s)))
    return out
