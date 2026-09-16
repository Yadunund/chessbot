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
from chessbot_brain.geometry import PIECE_HEIGHTS, BoardGeometry, jaw_clearances

# Heights of the grasp point above the playing surface, metres. Every piece is gripped at
# GRASP_HEIGHT, so its base hangs that far below the grasp point; placing releases it
# PLACE_DROP higher, so a piece that slipped in the jaws is never pushed into the board.
GRASP_HEIGHT = 0.015
PLACE_DROP = 0.003
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


def lift_if_low(caps: Capabilities, geometry: BoardGeometry | None):
    """Straight up to a transit height if the tool is low over the board or graveyards, so
    the next joint-space move cannot sweep through pieces. Keeps the current jaw yaw.
    Elsewhere (e.g. parked, or at start-up) there is nothing to hit, so nothing is done."""
    (x, y, z), yaw = caps.tool_pose()
    if z >= TRANSIT_HEIGHTS[-1] - 0.005:
        return
    if geometry is not None and not geometry.over_play_area(x, y):
        return
    last_error: CapabilityError | None = None
    for height in TRANSIT_HEIGHTS:
        try:
            caps.execute(caps.plan_cartesian([(x, y, height)], tool_down(yaw)))
            return
        except CapabilityError as exc:
            last_error = exc
    raise last_error or CapabilityError("could not lift the tool clear of the pieces")


def grasp_yaws(caps: Capabilities, piece_xyz, others_xyz: Sequence, offset: float) -> list[float]:
    """Jaw yaws to try, clearest of the other pieces first. Refuses when every yaw
    would put the gripper into another piece."""
    ranked = jaw_clearances(piece_xyz, list(others_xyz), offset, caps.arm.piece_radius)
    if ranked[0][1] < 0.0:
        raise CapabilityError(f"no jaw orientation clears the neighbouring pieces (best overlap {-ranked[0][1] * 1000:.0f} mm)")
    return [yaw for yaw, clearance in ranked if clearance >= 0.0]


def move_above(
    caps: Capabilities, geometry: BoardGeometry, piece_xyz, yaws: Sequence[float], offset: float
) -> tuple[float, float]:
    """Joint-space move to hover over a piece, as high as is reachable.

    Lifts first if the tool is low. Tries each jaw yaw at each hover height.
    Returns (height, yaw) used.
    """
    lift_if_low(caps, geometry)
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


def descend_and(caps: Capabilities, tool_xyz, yaw: float, gripper_position: float, hover_height: float, height: float):
    """Straight down to `height`, set the gripper, straight back up to the hover height."""
    quat = tool_down(yaw)
    caps.execute(caps.plan_cartesian([_above(tool_xyz, height)], quat))
    caps.gripper(gripper_position)
    caps.execute(caps.plan_cartesian([_above(tool_xyz, hover_height)], quat))


def pick(caps: Capabilities, geometry: BoardGeometry, xyz, others_xyz: Sequence):
    """Grasp the piece at xyz. others_xyz: every other believed piece."""
    yaws = grasp_yaws(caps, xyz, others_xyz, caps.arm.pick_offset)
    caps.gripper(caps.arm.gripper_open)
    height, yaw = move_above(caps, geometry, xyz, yaws, caps.arm.pick_offset)
    descend_and(caps, tool_point(xyz, yaw, caps.arm.pick_offset), yaw, caps.arm.gripper_closed, height, GRASP_HEIGHT)


def place(caps: Capabilities, geometry: BoardGeometry, xyz, others_xyz: Sequence):
    """Put the held piece down at xyz. others_xyz: every other believed piece."""
    yaws = grasp_yaws(caps, xyz, others_xyz, caps.arm.place_offset)
    height, yaw = move_above(caps, geometry, xyz, yaws, caps.arm.place_offset)
    descend_and(
        caps, tool_point(xyz, yaw, caps.arm.place_offset), yaw, caps.arm.gripper_open, height, GRASP_HEIGHT + PLACE_DROP
    )


def transfer(caps: Capabilities, geometry: BoardGeometry, src_xyz, dst_xyz, others_xyz: Sequence):
    """Move a piece. others_xyz: every believed piece except the one being moved."""
    pick(caps, geometry, src_xyz, others_xyz)
    place(caps, geometry, dst_xyz, others_xyz)


def park(caps: Capabilities, park_joints: list[float], geometry: BoardGeometry | None = None):
    """Lift clear, swing the base to the park side at that height, then settle into the pose.

    Swinging first keeps the arm high while it crosses the board, since the park pose is low.
    """
    lift_if_low(caps, geometry)
    current = caps.joint_positions()
    if abs(current[0] - park_joints[0]) > 0.05:
        caps.execute(caps.joint_move_trajectory([park_joints[0], *current[1:]]))
    caps.execute(caps.joint_move_trajectory(park_joints))


def execute_chess_move(
    caps: Capabilities,
    geometry: BoardGeometry,
    effects: MoveEffects,
    graveyard: dict[str, list[str]],
    narrate: Narrate,
    pieces: dict[str, tuple[float, float, float]],
):
    """Carry out a move physically, including its side effects.

    ``pieces`` maps every believed piece (by square or graveyard slot) to its position,
    so each grasp can keep the gripper clear of the others.

    Captures first (the captured piece goes to the next free graveyard slot and
    is recorded in ``graveyard``),
    then the moving piece, then the castling rook. Promotion pieces are not
    fetched yet: the pawn is left on the promotion square.
    """
    src, dst = effects.uci[:2], effects.uci[2:4]
    occupied = dict(pieces)
    if effects.captured and effects.capture_square:
        colour = piece_colour(effects.captured)
        slot = len(graveyard[colour])
        narrate(f"Capture: moving {effects.captured} from {effects.capture_square} to graveyard slot {colour}/{slot}.")
        occupied.pop(effects.capture_square, None)
        slot_xyz = geometry.graveyard_slot(colour, slot)
        transfer(caps, geometry, geometry.square_centre(effects.capture_square), slot_xyz, list(occupied.values()))
        graveyard[colour].append(effects.captured)
        occupied[f"grave_{colour}_{slot}"] = (*slot_xyz, PIECE_HEIGHTS[effects.captured.lower()])
    narrate(f"Moving {src} to {dst}.")
    moving = occupied.pop(src, None)
    transfer(caps, geometry, geometry.square_centre(src), geometry.square_centre(dst), list(occupied.values()))
    occupied[dst] = moving if moving is None else (*geometry.square_centre(dst), moving[3])
    if effects.rook_from and effects.rook_to:
        narrate(f"Castling: moving the rook {effects.rook_from} to {effects.rook_to}.")
        occupied.pop(effects.rook_from, None)
        transfer(caps, geometry, geometry.square_centre(effects.rook_from), geometry.square_centre(effects.rook_to), list(occupied.values()))
    if effects.promotion:
        narrate(f"Promotion to {effects.promotion}: swapping pieces is not implemented yet; please swap it by hand.")
