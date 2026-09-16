# Copyright 2026 Yadunund Vijay
# SPDX-License-Identifier: Apache-2.0
"""Skills: sequences composed from capabilities.

Plain functions for now; each could later become a node in a behaviour tree
that Gemma assembles. Skills never wait on a human and keep no state between
calls: they do one physical thing and either return or raise.
"""

from __future__ import annotations

import math
from typing import Callable, Sequence

from chessbot_brain.board import MoveEffects, piece_colour
from chessbot_brain.capabilities import Capabilities, CapabilityError, tool_down
from chessbot_brain.geometry import BoardGeometry

# Heights above the playing surface, metres.
GRASP_HEIGHT = 0.015
# Hover heights tried in order. The highest clears every piece, but close to the
# robot's base it is out of reach, so the arm hovers lower there.
TRANSIT_HEIGHTS = (0.08, 0.065, 0.05)

Narrate = Callable[[str], None]


def _above(xyz, height):
    x, y, z = xyz
    return (x, y, z + height)


def tool_point(piece_xyz, yaw: float, offset: float):
    """Where the grasp point (on the fixed jaw) goes for a piece at piece_xyz, jaws opening along yaw."""
    x, y, z = piece_xyz
    return (x - offset * math.cos(yaw), y - offset * math.sin(yaw), z)


def move_above(caps: Capabilities, piece_xyz, yaws: Sequence[float], offset: float) -> tuple[float, float]:
    """Joint-space move to hover over a piece, as high as is reachable.

    Tries each jaw yaw at each hover height. Returns (height, yaw) used.
    """
    last_error: CapabilityError | None = None
    for height in TRANSIT_HEIGHTS:
        for yaw in yaws:
            try:
                target = caps.solve_ik(_above(tool_point(piece_xyz, yaw, offset), height), tool_down(yaw))
            except CapabilityError as exc:
                last_error = exc
                continue
            caps.execute(caps.joint_move_trajectory(target))
            return height, yaw
    raise last_error or CapabilityError("no reachable hover pose")


def descend_and(caps: Capabilities, tool_xyz, yaw: float, gripper_position: float, hover_height: float):
    """Straight down to grasp height, set the gripper, straight back up to the hover height."""
    quat = tool_down(yaw)
    caps.execute(caps.plan_cartesian([_above(tool_xyz, GRASP_HEIGHT)], quat))
    caps.gripper(gripper_position)
    caps.execute(caps.plan_cartesian([_above(tool_xyz, hover_height)], quat))


def pick(caps: Capabilities, xyz, yaws: Sequence[float]):
    caps.gripper(caps.arm.gripper_open)
    height, yaw = move_above(caps, xyz, yaws, caps.arm.pick_offset)
    descend_and(caps, tool_point(xyz, yaw, caps.arm.pick_offset), yaw, caps.arm.gripper_closed, height)


def place(caps: Capabilities, xyz, yaws: Sequence[float]):
    height, yaw = move_above(caps, xyz, yaws, caps.arm.place_offset)
    descend_and(caps, tool_point(xyz, yaw, caps.arm.place_offset), yaw, caps.arm.gripper_open, height)


def transfer(caps: Capabilities, src_xyz, dst_xyz, yaws: Sequence[float]):
    pick(caps, src_xyz, yaws)
    place(caps, dst_xyz, yaws)


def park(caps: Capabilities, park_joints: list[float]):
    caps.execute(caps.joint_move_trajectory(park_joints))


def execute_chess_move(
    caps: Capabilities,
    geometry: BoardGeometry,
    effects: MoveEffects,
    graveyard: dict[str, list[str]],
    narrate: Narrate,
):
    """Carry out a move physically, including its side effects.

    Captures first (the captured piece goes to the next free graveyard slot and
    is recorded in ``graveyard``),
    then the moving piece, then the castling rook. Promotion pieces are not
    fetched yet: the pawn is left on the promotion square.
    """
    src, dst = effects.uci[:2], effects.uci[2:4]
    yaws = geometry.diagonal_yaws()
    if effects.captured and effects.capture_square:
        colour = piece_colour(effects.captured)
        slot = len(graveyard[colour])
        narrate(f"Capture: moving {effects.captured} from {effects.capture_square} to graveyard slot {colour}/{slot}.")
        transfer(caps, geometry.square_centre(effects.capture_square), geometry.graveyard_slot(colour, slot), yaws)
        graveyard[colour].append(effects.captured)
    narrate(f"Moving {src} to {dst}.")
    transfer(caps, geometry.square_centre(src), geometry.square_centre(dst), yaws)
    if effects.rook_from and effects.rook_to:
        narrate(f"Castling: moving the rook {effects.rook_from} to {effects.rook_to}.")
        transfer(caps, geometry.square_centre(effects.rook_from), geometry.square_centre(effects.rook_to), yaws)
    if effects.promotion:
        narrate(f"Promotion to {effects.promotion}: swapping pieces is not implemented yet; please swap it by hand.")
