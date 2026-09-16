# Copyright 2026 Yadunund Vijay
# SPDX-License-Identifier: Apache-2.0
"""The calibration profile: what it contains, where it lives, how it is shared.

The profile is stored as YAML on disk (the durable record) and published into
the Zenoh key-value store as JSON under ``KV_KEY`` (how every other node reads
it). ``SCHEMA_VERSION`` is bumped on any incompatible change, so a reader can
tell a stale value.
"""

from __future__ import annotations

import json
import math
import os
import time
from dataclasses import asdict, dataclass, field

import yaml

SCHEMA_VERSION = 1
KV_KEY = "chessbot/kv/calibration"


@dataclass
class BoardCalibration:
    """Where the board is, in the robot's base frame.

    The board frame has its origin at the outer corner of a1 on the playing
    surface, +x towards the h-file, +y towards rank 8, +z up.
    """

    frame_id: str = "base_link"
    # Board frame origin (a1 outer corner) in `frame_id`, metres.
    #
    # Nominal values: a 21 cm board whose robot-side edge is 9 cm from the pan
    # axis (the best placement from the reachability analysis), with the robot at
    # the rank-8 edge so its own (black) pieces are the ones nearest it and the
    # human sits opposite.
    origin_xyz: list[float] = field(default_factory=lambda: [0.3388, -0.105, 0.0])
    # Rotation of the board frame about +z in `frame_id`, radians.
    yaw_rad: float = math.pi / 2
    square_size_m: float = 0.02625


@dataclass
class CalibrationProfile:
    version: int = SCHEMA_VERSION
    board: BoardCalibration = field(default_factory=BoardCalibration)
    # "nominal" when built from design dimensions rather than measured.
    source: str = "nominal"
    created_unix: float = field(default_factory=time.time)

    def to_json(self) -> str:
        return json.dumps(asdict(self))

    @staticmethod
    def from_dict(data: dict) -> "CalibrationProfile":
        board = BoardCalibration(**data.get("board", {}))
        return CalibrationProfile(
            version=int(data.get("version", SCHEMA_VERSION)),
            board=board,
            source=str(data.get("source", "nominal")),
            created_unix=float(data.get("created_unix", time.time())),
        )


def load(path: str) -> CalibrationProfile | None:
    if not os.path.exists(path):
        return None
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    profile = CalibrationProfile.from_dict(data)
    if profile.version != SCHEMA_VERSION:
        return None
    return profile


def save(path: str, profile: CalibrationProfile) -> None:
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        yaml.safe_dump(asdict(profile), f, sort_keys=False)
    os.replace(tmp, path)
