# Copyright 2026 Yadunund Vijay
# SPDX-License-Identifier: Apache-2.0
"""Skills: sequences composed from capabilities.

Plain functions for now; each could later become a node in a behaviour tree
that Gemma assembles. Skills never wait on a human and keep no state between
calls: they do one physical thing and either return or raise.
"""

from __future__ import annotations

from typing import Callable

from chessbot_brain.board import MoveEffects, piece_colour
from chessbot_brain.capabilities import Capabilities
from chessbot_brain.geometry import BoardGeometry

# Heights above the playing surface, metres.
GRASP_HEIGHT = 0.015
TRANSIT_HEIGHT = 0.08

Narrate = Callable[[str], None]


def _above(xyz, height):
    x, y, z = xyz
    return (x, y, z + height)


def move_above(caps: Capabilities, xyz, height=TRANSIT_HEIGHT):
    """Joint-space move to hover over a point (fast, no straight-line constraint)."""
    target = caps.solve_ik(_above(xyz, height))
    caps.execute(caps.joint_move_trajectory(target))


def descend_and(caps: Capabilities, xyz, gripper_position: float):
    """Straight down to grasp height, set the gripper, straight back up."""
    caps.execute(caps.plan_cartesian([_above(xyz, GRASP_HEIGHT)]))
    caps.gripper(gripper_position)
    caps.execute(caps.plan_cartesian([_above(xyz, TRANSIT_HEIGHT)]))


def pick(caps: Capabilities, xyz):
    caps.gripper(caps.arm.gripper_open)
    move_above(caps, xyz)
    descend_and(caps, xyz, caps.arm.gripper_closed)


def place(caps: Capabilities, xyz):
    move_above(caps, xyz)
    descend_and(caps, xyz, caps.arm.gripper_open)


def transfer(caps: Capabilities, src_xyz, dst_xyz):
    pick(caps, src_xyz)
    place(caps, dst_xyz)


def park(caps: Capabilities, park_joints: list[float]):
    caps.execute(caps.joint_move_trajectory(park_joints))


def execute_chess_move(
    caps: Capabilities,
    geometry: BoardGeometry,
    effects: MoveEffects,
    graveyard_counts: dict[str, int],
    narrate: Narrate,
):
    """Carry out a move physically, including its side effects.

    Captures first (the captured piece goes to the next free graveyard slot),
    then the moving piece, then the castling rook. Promotion pieces are not
    fetched yet: the pawn is left on the promotion square.
    """
    src, dst = effects.uci[:2], effects.uci[2:4]
    if effects.captured and effects.capture_square:
        colour = piece_colour(effects.captured)
        slot = graveyard_counts[colour]
        narrate(f"Capture: moving {effects.captured} from {effects.capture_square} to graveyard slot {colour}/{slot}.")
        transfer(caps, geometry.square_centre(effects.capture_square), geometry.graveyard_slot(colour, slot))
        graveyard_counts[colour] += 1
    narrate(f"Moving {src} to {dst}.")
    transfer(caps, geometry.square_centre(src), geometry.square_centre(dst))
    if effects.rook_from and effects.rook_to:
        narrate(f"Castling: moving the rook {effects.rook_from} to {effects.rook_to}.")
        transfer(caps, geometry.square_centre(effects.rook_from), geometry.square_centre(effects.rook_to))
    if effects.promotion:
        narrate(f"Promotion to {effects.promotion}: swapping pieces is not implemented yet; please swap it by hand.")
