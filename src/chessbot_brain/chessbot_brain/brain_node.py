# Copyright 2026 Yadunund Vijay
# SPDX-License-Identifier: Apache-2.0
"""The brain: game engine and orchestrator.

Owns the game (position, moves, clocks, phase) and publishes it as
``/chessbot/game_state`` together with a readable ``/chessbot/thoughts`` feed.
It composes capabilities (perception, motion planning, control, calibration)
as a client, and serves a REST API the UI calls. It also serves the UI's static
files, so the browser talks to one origin.

Rules questions (legal moves, check) and move choice go to Stockfish over UCI
when the binary is available; otherwise a placeholder that checks no rules.
Not implemented yet, and said so in the thought feed: detecting the human's
move from perception. Until then the move is passed explicitly to
``press_clock``.
"""

from __future__ import annotations

import collections
import json
import math
import os
import threading
import time
from concurrent.futures import ThreadPoolExecutor

import rclpy
import uvicorn
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from builtin_interfaces.msg import Time
from chessbot_interfaces.msg import GameState, Thought
from chessbot_interfaces.srv import GetBoardState, Reason
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.middleware.gzip import GZipMiddleware
from fastapi.responses import FileResponse, Response
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from chessbot_brain import skills
from chessbot_brain.board import Board, piece_colour, square_name
from chessbot_brain import calibration_flow as calib
from chessbot_brain.capabilities import ArmConfig, Capabilities, CapabilityError, tool_down
from chessbot_brain.engine import EngineError, UciEngine, choose_reply
from chessbot_brain.geometry import PIECE_HEIGHTS, BoardGeometry, graveyard_cell
from chessbot_brain.move_reading import read_move

PHASE_NAMES = {
    GameState.PHASE_IDLE: "idle",
    GameState.PHASE_CALIBRATING: "calibrating",
    GameState.PHASE_SETUP: "setup",
    GameState.PHASE_HUMAN_TURN: "human_turn",
    GameState.PHASE_READING_BOARD: "reading_board",
    GameState.PHASE_THINKING: "thinking",
    GameState.PHASE_MOVING: "moving",
    GameState.PHASE_VERIFYING: "verifying",
    GameState.PHASE_NEEDS_HELP: "needs_help",
    GameState.PHASE_GAME_OVER: "game_over",
}
THOUGHT_NAMES = {
    Thought.PERCEIVE: "perceive",
    Thought.DECIDE: "decide",
    Thought.PLAN: "plan",
    Thought.ACT: "act",
    Thought.VERIFY: "verify",
    Thought.RECOVER: "recover",
    Thought.EXPLAIN: "explain",
}
BOARD_RESULT_TEXT = {
    GetBoardState.Response.RESULT_OK: "board located",
    GetBoardState.Response.RESULT_NO_IMAGE: "no recent camera frame",
    GetBoardState.Response.RESULT_NO_BOARD: "the board isn't located, or hasn't been seen in a known position yet",
}


# Robot meshes, and the piece set description the UI draws pieces from.
PACKAGE_FILE_EXTENSIONS = {".stl", ".dae", ".obj", ".glb", ".gltf", ".json"}

# Where manual control and calibration are allowed to put the tool, in base_link: forward of
# the base, within reach, and above the table. An uncalibrated rig does not know where the
# board is, so nothing may be commanded outside this.
JOG_BOX = ((0.08, 0.30), (-0.18, 0.18), (0.04, 0.30))
# Most one press of a per-joint jog may turn a joint, radians (about 17 degrees).
MAX_JOINT_JOG = 0.3


PIECE_NAMES = {"p": "pawn", "n": "knight", "b": "bishop", "r": "rook", "q": "queen", "k": "king"}


def describe_move(board: Board, effects) -> str:
    """A move in words, read from the position before it is applied."""
    source, destination = effects.uci[:2], effects.uci[2:4]
    name = PIECE_NAMES.get((board.piece_at(source) or "").lower(), "piece")
    if effects.rook_from is not None:
        side = "kingside" if effects.rook_from[0] == "h" else "queenside"
        return f"{side} castling, king {source} to {destination}, rook {effects.rook_from} to {effects.rook_to}"
    text = f"{name} {source} to {destination}"
    if effects.captured is not None:
        taken = PIECE_NAMES.get(effects.captured.lower(), "piece")
        text += f", taking the {taken} on {effects.capture_square}"
        if effects.capture_square != destination:
            text += " en passant"
    if effects.promotion is not None:
        text += f", promoting to {PIECE_NAMES.get(effects.promotion.lower(), 'piece')}"
    return text


class Busy(RuntimeError):
    pass


class BrainNode(Node):
    def __init__(self):
        super().__init__("brain")
        self.rest_port = int(self.declare_parameter("rest_port", 8000).value)
        self.web_root = self.declare_parameter("web_root", "").value
        self.default_robot_side = self.declare_parameter("robot_side", "black").value
        self.robot_name = self.declare_parameter("robot_name", "SO-101").value
        self.time_control_ms = int(self.declare_parameter("time_control_ms", 600_000).value)
        # Swung to the side and lying low along the board's near edge, clear of the board and
        # graveyards and out of the overhead camera's view (tools/dev/park_search.py --pan).
        self.park_joints = list(self.declare_parameter("park_joints", [-1.57, 1.59, -1.54, 0.22, 0.0]).value)
        arm = ArmConfig(
            joints=list(
                self.declare_parameter(
                    "arm_joints",
                    ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_flex_joint", "wrist_flex_joint", "wrist_roll_joint"],
                ).value
            ),
            gripper_joint=self.declare_parameter("gripper_joint", "gripper_joint").value,
            # 0.24 rad opens the jaws 20 mm at the grasp point: a 15 mm piece plus 2.5 mm each
            # side (tools/dev/gripper_geometry.py). Closing commands past contact so it grips.
            gripper_open=float(self.declare_parameter("gripper_open", 0.24).value),
            gripper_closed=float(self.declare_parameter("gripper_closed", 0.05).value),
            pick_offset=float(self.declare_parameter("pick_offset", 0.010).value),
            place_offset=float(self.declare_parameter("place_offset", 0.0075).value),
            joint_speed=float(self.declare_parameter("joint_speed", 1.6).value),
            hardware_component=self.declare_parameter("hardware_component", "SO_ARM101").value,
            controller=self.declare_parameter("arm_controller_name", "arm_controller").value,
        )
        zenoh_endpoint = self.declare_parameter("zenoh_endpoint", "tcp/127.0.0.1:7447").value
        default_stockfish = os.path.join(os.environ.get("PIXI_PROJECT_ROOT", ""), "generated", "bin", "stockfish")
        stockfish_path = self.declare_parameter("stockfish_path", default_stockfish).value
        self.engine_movetime_ms = int(self.declare_parameter("engine_movetime_ms", 800).value)
        # False reports the reply and leaves the arm alone, for checking perception and the
        # engine without motion in the loop.
        self.play_moves = bool(self.declare_parameter("play_moves", True).value)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.state_pub = self.create_publisher(GameState, "/chessbot/game_state", latched)
        self.thought_pub = self.create_publisher(Thought, "/chessbot/thoughts", 50)
        self.board_marker_pub = self.create_publisher(MarkerArray, "/chessbot/board_markers", latched)
        self.create_timer(1.0, self.publish_board_markers)
        # Kept for the UI's 3D view, which draws the robot from its URDF.
        self.robot_description = ""
        self.create_subscription(String, "/robot_description", self._on_robot_description, latched)

        self.caps = Capabilities(self, arm, zenoh_endpoint)

        self._lock = threading.RLock()
        self._jobs = ThreadPoolExecutor(max_workers=1)
        # Commentary runs beside the game, so a slow model never holds up play.
        self._commentary = ThreadPoolExecutor(max_workers=1)
        self._busy = threading.Lock()
        self.thoughts: collections.deque = collections.deque(maxlen=200)
        self.last_error = ""
        self.current_job = ""
        # What the guided calibration workflow has established so far.
        self.draft = calib.Draft()

        self.engine: UciEngine | None = None
        if stockfish_path and os.path.isfile(stockfish_path):
            try:
                self.engine = UciEngine(stockfish_path)
            except (OSError, EngineError) as exc:
                self.get_logger().error(f"Could not start Stockfish at {stockfish_path}: {exc}")
        else:
            self.get_logger().warning(f"No Stockfish at '{stockfish_path}' (run `pixi run stockfish`)")

        self.mode = GameState.MODE_PROTOTYPE
        self.engine_elo = 0
        self._reset_game(self.default_robot_side)
        self.phase = GameState.PHASE_IDLE
        self.publish_state()
        engine_text = "Stockfish" if self.engine else "a placeholder engine that checks no rules"
        self.think(Thought.EXPLAIN, f"Brain started, using {engine_text}. Start a new game when ready.")

    # --- game state ----------------------------------------------------------------

    def _reset_game(self, robot_side: str):
        with self._lock:
            self.board = Board()
            self.moves: list[str] = []
            self.robot_side = robot_side
            self.result = GameState.RESULT_IN_PROGRESS
            # Pieces the robot has put in each graveyard, in slot order.
            self.graveyard: dict[str, list[str]] = {"white": [], "black": []}
            self.white_ms = self.time_control_ms
            self.black_ms = self.time_control_ms
            self.clock_running = GameState.CLOCK_NONE
            self.clock_last_switch = time.time()

    def _on_robot_description(self, msg: String):
        self.robot_description = msg.data

    def side_to_move(self) -> str:
        return "white" if self.board.white_to_move else "black"

    def _switch_clock(self, to_side: str | None):
        with self._lock:
            now = time.time()
            elapsed_ms = int((now - self.clock_last_switch) * 1000)
            if self.clock_running == GameState.CLOCK_WHITE:
                self.white_ms -= elapsed_ms
            elif self.clock_running == GameState.CLOCK_BLACK:
                self.black_ms -= elapsed_ms
            self.clock_running = {
                None: GameState.CLOCK_NONE,
                "white": GameState.CLOCK_WHITE,
                "black": GameState.CLOCK_BLACK,
            }[to_side]
            self.clock_last_switch = now

    def set_phase(self, phase: int):
        with self._lock:
            self.phase = phase
        self.publish_state()

    def publish_board_markers(self):
        """The calibrated board as markers, for RViz."""
        geometry = self.geometry_if_calibrated()
        if geometry is None:
            return
        side = geometry.square_size_m * 8.0
        centre = geometry.board_to_robot(side / 2.0, side / 2.0)
        half_yaw = geometry.yaw_rad / 2.0
        markers = MarkerArray()

        surface = Marker()
        surface.header.frame_id = geometry.frame_id
        surface.ns, surface.id, surface.type, surface.action = "board", 0, Marker.CUBE, Marker.ADD
        surface.pose.position.x, surface.pose.position.y, surface.pose.position.z = centre
        surface.pose.orientation.z, surface.pose.orientation.w = math.sin(half_yaw), math.cos(half_yaw)
        surface.scale.x, surface.scale.y, surface.scale.z = side, side, 0.002
        surface.color.r, surface.color.g, surface.color.b, surface.color.a = 0.85, 0.80, 0.72, 0.85
        markers.markers.append(surface)

        label = Marker()
        label.header.frame_id = geometry.frame_id
        label.ns, label.id, label.type, label.action = "board", 1, Marker.TEXT_VIEW_FACING, Marker.ADD
        label.pose.position.x, label.pose.position.y = centre[0], centre[1]
        label.pose.position.z = centre[2] + 0.05
        label.pose.orientation.w = 1.0
        label.scale.z = 0.015
        label.color.r = label.color.g = label.color.b = label.color.a = 1.0
        label.text = (f"{side * 1000:.0f} mm board, {geometry.square_size_m * 1000:.1f} mm squares, "
                      f"{math.degrees(geometry.yaw_rad):.1f} deg")
        markers.markers.append(label)

        origin = Marker()
        origin.header.frame_id = geometry.frame_id
        origin.ns, origin.id, origin.type, origin.action = "board", 2, Marker.SPHERE, Marker.ADD
        origin.pose.position.x, origin.pose.position.y, origin.pose.position.z = geometry.origin_xyz
        origin.pose.orientation.w = 1.0
        origin.scale.x = origin.scale.y = origin.scale.z = 0.008
        origin.color.r, origin.color.a = 1.0, 1.0
        markers.markers.append(origin)

        self.board_marker_pub.publish(markers)

    def publish_state(self):
        with self._lock:
            msg = GameState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.phase = self.phase
            msg.mode = self.mode
            msg.fen = self.board.fen()
            msg.turn = GameState.SIDE_WHITE if self.board.white_to_move else GameState.SIDE_BLACK
            msg.robot_side = GameState.SIDE_WHITE if self.robot_side == "white" else GameState.SIDE_BLACK
            msg.moves = list(self.moves)
            msg.result = self.result
            msg.engine_elo = self.engine_elo
            msg.white_ms = self.white_ms
            msg.black_ms = self.black_ms
            msg.clock_running = self.clock_running
            last = self.clock_last_switch
            msg.clock_last_switch = Time(sec=int(last), nanosec=int((last - int(last)) * 1e9))
        self.state_pub.publish(msg)

    def think(self, kind: int, text: str, model_generated: bool = False):
        msg = Thought(kind=kind, text=text, model_generated=model_generated)
        msg.header.stamp = self.get_clock().now().to_msg()
        self.thought_pub.publish(msg)
        self.thoughts.append({"t": time.time(), "kind": THOUGHT_NAMES[kind], "text": text})
        self.get_logger().info(f"[{THOUGHT_NAMES[kind]}] {text}")

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "phase": PHASE_NAMES[self.phase],
                "mode": "prototype",
                "fen": self.board.fen(),
                "turn": self.side_to_move(),
                "robot_side": self.robot_side,
                "play_moves": self.play_moves,
                "robot_name": self.robot_name,
                "moves": list(self.moves),
                # Pieces the robot has captured, where it put them (board frame, in squares).
                "graveyard": [
                    {"piece": piece, "cell": graveyard_cell(colour, slot)}
                    for colour, pieces in self.graveyard.items()
                    for slot, piece in enumerate(pieces)
                ],
                "result": self.result,
                "engine_elo": self.engine_elo,
                "clock": {
                    "white_ms": self.white_ms,
                    "black_ms": self.black_ms,
                    "running": {0: None, 1: "white", 2: "black"}[self.clock_running],
                    "last_switch_unix": self.clock_last_switch,
                },
                "thoughts": list(self.thoughts)[-50:],
                "busy": self.current_job,
                "last_error": self.last_error,
                # So the per-joint jog UI can list this robot's actual joints instead of
                # hardcoding SO-101's - a short label derived generically from the name, not
                # assumed from it.
                "arm_joints": [
                    {"name": name, "label": name.removesuffix("_joint").replace("_", " ")}
                    for name in self.caps.arm.joints
                ],
                "capabilities": self.caps.status(),
            }

    # --- jobs ------------------------------------------------------------------------

    def submit(self, name: str, fn, *args):
        """Run one job at a time on the worker thread; reject if one is running."""
        if not self._busy.acquire(blocking=False):
            raise Busy(f"busy with {self.current_job}")
        self.current_job = name

        def run():
            try:
                fn(*args)
                self.last_error = ""
            except CapabilityError as exc:
                self.last_error = str(exc)
                self.think(Thought.RECOVER, f"{name} failed: {exc}. Needs help.")
                self.set_phase(GameState.PHASE_NEEDS_HELP)
            except Exception as exc:  # noqa: BLE001 - surface anything to the UI
                self.last_error = f"{type(exc).__name__}: {exc}"
                self.get_logger().error(f"{name} crashed: {self.last_error}")
                self.think(Thought.RECOVER, f"{name} hit an internal error: {self.last_error}")
                self.set_phase(GameState.PHASE_NEEDS_HELP)
            finally:
                self.current_job = ""
                self._busy.release()
                self.publish_state()

        self._jobs.submit(run)

    def run_now(self, name: str, fn, *args):
        """Run a short job inline and return its result, under `submit`'s one-at-a-time lock.

        For wizard steps that must answer with the state they produced, and must not run
        while the worker is moving the arm.
        """
        if not self._busy.acquire(blocking=False):
            raise Busy(f"busy with {self.current_job}")
        self.current_job = name
        try:
            return fn(*args)
        finally:
            self.current_job = ""
            self._busy.release()
            self.publish_state()

    def geometry(self) -> BoardGeometry:
        data = self.caps.kv_get("chessbot/kv/calibration")
        if data is None:
            raise CapabilityError("no calibration in the key-value store (is the calibration node running?)")
        return BoardGeometry.from_calibration(data)

    def job_new_game(self, robot_side: str, engine_elo: int):
        self._reset_game(robot_side)
        self.engine_elo = engine_elo
        self.think(Thought.DECIDE, f"New game. I play {robot_side}.")
        self.set_phase(GameState.PHASE_SETUP)
        self.job_park()
        if robot_side == "white":
            self._robot_turn()
        else:
            self._switch_clock("white")
            self.set_phase(GameState.PHASE_HUMAN_TURN)

    def job_set_position(self, fen: str, robot_side: str):
        """Dev: replace the believed position (for tests), without moving anything."""
        try:
            board = Board(fen)
        except (ValueError, IndexError) as exc:
            self.think(Thought.RECOVER, f"Not a valid FEN: {exc}")
            return
        self._reset_game(robot_side)
        with self._lock:
            self.board = board
        self.think(Thought.EXPLAIN, f"Dev: position set to {board.fen()}.")
        human_to_move = self.side_to_move() != robot_side
        self.set_phase(GameState.PHASE_HUMAN_TURN if human_to_move else GameState.PHASE_IDLE)

    def job_press_clock(self, move: str | None):
        with self._lock:
            if self.phase != GameState.PHASE_HUMAN_TURN:
                self.think(Thought.RECOVER, "Clock pressed, but it is not your turn.")
                return
        self.set_phase(GameState.PHASE_READING_BOARD)
        seen = self._read_move_from_camera()

        if move is None:
            if seen is None:
                self.think(Thought.RECOVER, "Type your move (e.g. e2e4) and press the clock again.")
                self.set_phase(GameState.PHASE_HUMAN_TURN)
                return
            move = seen
        elif seen is not None and seen != self._legal_form(move):
            self.think(Thought.VERIFY, f"The camera saw {seen}, but I'll go with the move you typed.")

        move = self._legal_form(move)
        if move is None:
            self.think(Thought.RECOVER, "That isn't a legal move here. Your clock keeps running.")
            self.set_phase(GameState.PHASE_HUMAN_TURN)
            return

        with self._lock:
            described = describe_move(self.board, self.board.effects(move))
            self.board.apply(move)
            self.moves.append(move)
        self.think(Thought.DECIDE, f"Your move: {move} - {described}.")
        if self._game_over():
            return
        self._robot_turn()

    def _read_move_from_camera(self) -> str | None:
        """The human's move as read from the overhead camera, or None (with the reason in the thought feed)."""
        try:
            response = self.caps.board_state(max_frame_age_s=1.0)
        except CapabilityError as exc:
            self.think(Thought.PERCEIVE, f"I can't read the board: {exc}.")
            return None
        if response.result != GetBoardState.Response.RESULT_OK:
            self.think(Thought.PERCEIVE, f"I can't read the board: {BOARD_RESULT_TEXT.get(response.result, 'unknown result')}.")
            return None
        if self.engine is None:
            self.think(Thought.PERCEIVE, "I can't read your move without the rules engine.")
            return None
        with self._lock:
            board = Board(self.board.fen())
        legal = self.engine.legal_moves(board.fen())
        reading = read_move(board, legal, list(response.state.squares), list(response.state.confidence))
        if reading.move is None and len(reading.candidates) >= 2 and reading.changed:
            self.think(Thought.PERCEIVE, f"I couldn't tell your move ({reading.reason}); asking Gemma to look.")
            choice = self._tiebreak(board, [move for move, _ in reading.candidates], legal)
            if choice is not None:
                return choice
        if reading.move is None:
            self.think(Thought.PERCEIVE, f"I couldn't tell your move: {reading.reason}.")
            return None
        self.think(Thought.PERCEIVE, f"I saw {reading.move}: {reading.reason}.")
        return reading.move

    def _ask_which_move(self, board: Board, frame, candidates: list[str]) -> tuple[str | None, float]:
        """Ask the model which of `candidates` the image shows. Returns (move, confidence)."""
        schema = {
            "type": "object",
            "properties": {"move": {"enum": [*candidates, "unclear"]}, "confidence": {"type": "number"}},
            "required": ["move", "confidence"],
        }
        prompt = (
            f"The image is the overhead camera view of a chess board. Before the human's move the position was "
            f"(FEN) {board.fen()}. {'White' if board.white_to_move else 'Black'} just moved. Which of these moves "
            f"(UCI notation) does the image show: {', '.join(candidates)}? Answer 'unclear' if you cannot tell."
        )
        try:
            answer = self.caps.reason(Reason.Request.ROLE_MOVE_TIEBREAK, prompt, [frame], json.dumps(schema))
        except CapabilityError:
            answer = None
        if answer is None:
            return None, 0.0
        try:
            parsed = json.loads(answer.text)
        except json.JSONDecodeError:
            self.think(Thought.PERCEIVE, f"Gemma's answer wasn't readable: {answer.text[:80]!r}.")
            return None, 0.0
        return parsed.get("move"), float(parsed.get("confidence", 0.0))

    def _tiebreak(self, board: Board, candidates: list[str], legal: list[str]) -> str | None:
        """Gemma picks which candidate move the image shows, if it can be shown to be looking.

        A vision model asked to choose between plausible chess moves can answer from what the
        opening book makes likely rather than from the image, and sound no less sure for it. So
        its answer only counts if it also declines a decoy - the same question with every real
        candidate replaced by moves that are legal but were not played. A model that picks one
        of those is guessing, and is not trusted for this frame.
        """
        frame = self.caps.camera_frame()
        if frame is None:
            self.think(Thought.PERCEIVE, "No camera frame to show Gemma.")
            return None
        move, confidence = self._ask_which_move(board, frame, candidates)
        if move is None and confidence == 0.0:
            self.think(Thought.PERCEIVE, "Gemma isn't available to look.")
            return None
        if move not in candidates or confidence < 0.7:
            self.think(Thought.PERCEIVE, f"Gemma couldn't tell either ({move}, confidence {confidence:.2f}).",
                       model_generated=True)
            return None

        decoys = [m for m in legal if m not in candidates][: len(candidates)]
        if decoys:
            decoy_move, decoy_confidence = self._ask_which_move(board, frame, decoys)
            if decoy_move in decoys and decoy_confidence >= 0.7:
                self.think(
                    Thought.PERCEIVE,
                    f"Gemma answered {move} but also picked {decoy_move} from moves you did not play, "
                    "so it is guessing rather than looking.",
                    model_generated=True,
                )
                return None
        self.think(Thought.PERCEIVE, f"Gemma says you played {move} (confidence {confidence:.2f}).", model_generated=True)
        return move

    def _comment_on(self, fen_before: str, human_move: str | None, reply: str):
        """A sentence or two explaining the robot's move, in the thought feed. Runs in the background."""
        side = "White" if Board(fen_before).white_to_move else "Black"
        prompt = (
            f"You play {side} in a chess game against a human. Position before your move (FEN): {fen_before}. "
            + (f"The human's last move was {human_move}. " if human_move else "")
            + f"You played {reply} (chosen by Stockfish). In one or two short sentences, speaking as the robot, "
            "say what your move does. Don't claim evaluations or lines you weren't given."
        )
        try:
            answer = self.caps.reason(Reason.Request.ROLE_COMMENTARY, prompt)
        except CapabilityError:
            return
        if answer is not None and answer.text:
            self.think(Thought.EXPLAIN, answer.text, model_generated=True)

    def _legal_form(self, move: str) -> str | None:
        """The move as the engine spells it if legal (adding a queen promotion when omitted), else None."""
        move = move.strip().lower()
        if self.engine is None:
            piece = self.board.piece_at(move[:2]) if len(move) >= 4 else None
            ok = piece is not None and piece_colour(piece) == self.side_to_move()
            return move if ok else None
        legal = self.engine.legal_moves(self.board.fen())
        if move in legal:
            return move
        if move + "q" in legal:
            return move + "q"
        return None

    def _game_over(self) -> bool:
        """Detect checkmate or stalemate for the side to move; update result and phase."""
        if self.engine is None or self.engine.legal_moves(self.board.fen()):
            return False
        loser = self.side_to_move()
        with self._lock:
            if self.engine.in_check(self.board.fen()):
                self.result = GameState.RESULT_BLACK_WON if loser == "white" else GameState.RESULT_WHITE_WON
                text = f"Checkmate. {'Black' if loser == 'white' else 'White'} wins."
            else:
                self.result = GameState.RESULT_DRAW
                text = "Stalemate. It's a draw."
        self._switch_clock(None)
        self.think(Thought.DECIDE, text)
        self.set_phase(GameState.PHASE_GAME_OVER)
        return True

    def job_robot_move(self, move: str):
        """Dev: the robot plays this move instead of the engine's (to test castling, en passant...)."""
        with self._lock:
            robot_to_move = self.side_to_move() == self.robot_side
        legal = self._legal_form(move) if robot_to_move else None
        if legal is None:
            self.think(Thought.RECOVER, f"Dev: {move} is not a legal robot move here.")
            return
        self._robot_turn(forced=legal)

    def _robot_turn(self, forced: str | None = None):
        self._switch_clock(self.robot_side)
        self.set_phase(GameState.PHASE_THINKING)
        if forced is not None:
            reply = forced
            self.think(Thought.DECIDE, f"Dev: playing {reply} as instructed.")
        elif self.engine is not None:
            reply = self.engine.best_move(self.board.fen(), self.engine_movetime_ms, self.engine_elo)
            strength = f"Elo {self.engine_elo}" if self.engine_elo else "full strength"
            self.think(Thought.DECIDE, f"I'll play {reply} (Stockfish, {strength}, {self.engine_movetime_ms} ms).")
        else:
            reply = choose_reply(self.board, self.robot_side)
            if reply is None:
                self.think(Thought.DECIDE, "I have no move to play.")
                self.set_phase(GameState.PHASE_GAME_OVER)
                return
            self.think(Thought.DECIDE, f"I'll play {reply} (placeholder engine, no rules checked).")

        geometry = self.geometry_if_calibrated() if not self.play_moves else self.geometry()
        effects = self.board.effects(reply)
        if not self.play_moves:
            self.think(Thought.PLAN, f"I would play {describe_move(self.board, effects)}, but play_moves is off.")
            with self._lock:
                self.board.apply(reply)
                self.moves.append(reply)
            self._switch_clock("white" if self.robot_side == "black" else "black")
            if not self._game_over():
                self.set_phase(GameState.PHASE_HUMAN_TURN)
            return

        self.set_phase(GameState.PHASE_MOVING)
        self.think(Thought.PLAN, f"Plan for {reply}: pick, place{', capture first' if effects.captured else ''}.")

        skills.execute_chess_move(
            self.caps, geometry, effects, self.graveyard, lambda text: self.think(Thought.ACT, text),
            self.believed_pieces(geometry),
        )
        with self._lock:
            fen_before = self.board.fen()
            human_move = self.moves[-1] if self.moves else None
            self.board.apply(reply)
            self.moves.append(reply)
        self.publish_state()
        self._commentary.submit(self._comment_on, fen_before, human_move, reply)

        self.set_phase(GameState.PHASE_VERIFYING)
        skills.park(self.caps, self.park_pose(), self.geometry_if_calibrated())
        response = self.caps.board_state(max_frame_age_s=1.0)
        self.think(Thought.VERIFY, f"Verification: {BOARD_RESULT_TEXT.get(response.result, 'unknown result')}.")
        if self._game_over():
            return

        self._switch_clock("white" if self.robot_side == "black" else "black")
        self.set_phase(GameState.PHASE_HUMAN_TURN)
        self.think(Thought.EXPLAIN, "Your move. Press the clock when you're done.")

    def job_calibrate(self):
        previous = self.phase
        self.set_phase(GameState.PHASE_CALIBRATING)
        result = self.caps.run_calibration()
        self.think(Thought.VERIFY, f"Calibration finished (result {result.result}) → {result.profile_path}")
        self.set_phase(previous)

    def park_pose(self) -> list[float]:
        """Where the arm waits: what calibration stored, else the launch parameter.

        Read on each park rather than cached, so a pose saved through the setup workflow
        applies immediately and still applies after a restart.
        """
        profile = self.caps.kv_get("chessbot/kv/calibration") or {}
        stored = [float(v) for v in (profile.get("park_joints") or [])]
        return stored if len(stored) == len(self.caps.arm.joints) else list(self.park_joints)

    def job_park(self):
        skills.park(self.caps, self.park_pose(), self.geometry_if_calibrated())
        self.think(Thought.ACT, "Arm parked.")

    def job_resume(self):
        self.think(Thought.RECOVER, "Resumed by operator.")
        self.set_phase(GameState.PHASE_HUMAN_TURN)

    # --- manual control, for bringing a real rig up ----------------------------------
    #
    # Every target is kept inside JOG_BOX: on a rig that is not calibrated yet the arm has no
    # idea where the board is, and the way to find that out is not by driving into it.

    def job_move_tool(self, xyz, yaw_deg: float, speed_pct: float | None = None):
        """Move to an absolute tool target.

        `speed_pct` is only given by the jog UI's press-and-hold loop: the step is published
        rather than sent as a goal, so the next tick supersedes it mid-motion. A one-off move
        leaves it unset and waits for the arm to arrive.
        """
        target = tuple(min(max(v, lo), hi) for v, (lo, hi) in zip(xyz, JOG_BOX))
        if tuple(xyz) != target:
            self.think(Thought.EXPLAIN, f"Kept inside the safe box: asked {tuple(round(v, 3) for v in xyz)}, "
                                        f"moving to {tuple(round(v, 3) for v in target)}.")
        solution = self.caps.solve_ik(target, tool_down(math.radians(yaw_deg)))
        if speed_pct is None:
            self.caps.execute(self.caps.joint_move_trajectory(solution))
            self.think(Thought.ACT, f"Tool at {tuple(round(v, 3) for v in target)}.")
        else:
            self.caps.stream(self.caps.jog_trajectory(solution, speed_pct / 100.0))

    def job_jog(self, axis: str, delta_m: float, speed_pct: float = 100.0):
        (x, y, z), yaw = self.caps.tool_pose()
        step = {"x": (delta_m, 0.0, 0.0), "y": (0.0, delta_m, 0.0), "z": (0.0, 0.0, delta_m)}.get(axis)
        if step is None:
            raise CapabilityError(f"unknown axis {axis!r}")
        self.job_move_tool((x + step[0], y + step[1], z + step[2]), math.degrees(yaw), speed_pct)

    def job_jog_joint(self, joint: str, delta_rad: float, speed_pct: float = 100.0):
        """Turn one joint by a small amount.

        Unlike the Cartesian jog this does not go through IK, so it can reach poses no tool
        target describes - which is the point when posing the arm by hand - and it is not
        bounded by the safe box. The step is capped instead, so a mistyped request cannot
        throw the arm across the table.
        """
        names = list(self.caps.arm.joints)
        if joint not in names:
            raise CapabilityError(f"unknown joint {joint!r}")
        step = max(-MAX_JOINT_JOG, min(MAX_JOINT_JOG, float(delta_rad)))
        target = self.caps.joint_positions()
        target[names.index(joint)] += step
        self.caps.stream(self.caps.jog_trajectory(target, speed_pct / 100.0))

    def job_gripper(self, position: float):
        self.caps.gripper(position)
        self.think(Thought.ACT, f"Gripper at {position:.3f} rad.")

    # --- guided calibration ----------------------------------------------------------

    def calibration_release_arm(self) -> dict:
        """Stop the arm controller and cut torque, so a person can hand-guide it.

        The gripper is the only calibration target a real rig always has, and the only way to
        get an honest 3D position for it is to ask the robot's own forward kinematics - not
        make the camera find it. Releasing the arm is step one of that: nothing is recorded
        until a person confirms a touch (`calibration_touch`). Doesn't require the board to be
        marked first - a touch is just an FK reading against a point name, independent of
        pixels, so it can happen before, after, or interleaved with marking the board.
        """
        self.caps.release_arm()
        self.draft.arm_released = True
        self.think(Thought.EXPLAIN, "Arm released - torque is off, guide it by hand.")
        return self.draft.steps()

    def calibration_touch(self) -> dict:
        """Record the next unrecorded touch point at the arm's current (hand-guided) position.

        What is recorded is where the closed jaw tips are, not the grasp point between them.

        Nothing is recorded on a whim - the arm has to actually be released first, and this is
        only ever called from an explicit "confirm" press once the person has the jaw tips on
        the point, so a person watching the live pose readout is always the one deciding when a
        touch counts.
        """
        if not self.draft.arm_released:
            raise CapabilityError("release the arm first")
        name = next((n for n in calib.CAMERA_TOUCH_POINTS if n not in self.draft.touched), None)
        if name is None:
            raise CapabilityError("every point is already touched")
        position = self.caps.touch_point()
        self.draft.touched[name] = position
        self.think(Thought.PERCEIVE, f"Touched {name} at {tuple(round(v, 3) for v in position)}.")
        if len(self.draft.touched) == len(calib.CAMERA_TOUCH_POINTS):
            self._apply_board()
            if self.draft.board_corners_px is not None:
                return self.calibration_camera_fit()
        return self.draft.steps()

    def calibration_reactivate_arm(self) -> dict:
        """Undo `calibration_release_arm`: torque back on, controller back in charge."""
        self.caps.reactivate_arm()
        self.draft.arm_released = False
        self.think(Thought.EXPLAIN, "Arm re-engaged.")
        return self.draft.steps()

    def calibration_camera_fit(self) -> dict:
        """Fit the camera's pose to the touched (3D) / clicked (2D) correspondences.

        Both halves come from points the person handled directly - FK for where they guided
        the gripper, the same clicks already taken to mark the board - so this is an ordinary
        camera resectioning fit (`calib.fit_camera`), not anything specific to how the points
        were gathered.
        """
        if self.draft.board_corners_px is None:
            raise CapabilityError("mark the board's corners first")
        touched = [(name, self.draft.touched[name]) for name in calib.CAMERA_TOUCH_POINTS
                   if name in self.draft.touched]
        if len(touched) < len(calib.CAMERA_TOUCH_POINTS):
            raise CapabilityError(
                f"only {len(touched)} of {len(calib.CAMERA_TOUCH_POINTS)} points touched; "
                "the fit needs all of them")
        points = [pos for _name, pos in touched]
        pixels = [self.draft.board_corners_px[calib.CAMERA_TOUCH_POINTS.index(name)] for name, _pos in touched]
        centre = self.caps.camera_centre()
        frame = self.caps.camera_raw()
        if frame is None:
            raise CapabilityError("no camera frames")
        centre_px = (centre[0], centre[1]) if centre and centre[0] else (frame.width / 2, frame.height / 2)
        focal_px = centre[2] if centre and centre[2] > 100.0 else calib.DEFAULT_FOCAL_PX
        model = calib.fit_camera(points, pixels, centre_px, focal_px)
        self.draft.camera = model
        self.think(Thought.VERIFY, f"Camera at {tuple(round(v, 3) for v in model.position)}, "
                                   f"fit {model.error_mean_px:.0f} px mean over {len(touched)} points.")
        return self.draft.steps()

    def calibration_board(self, corners) -> dict:
        """Four corners marked in the camera image (a1, h1, h8, a8): where the camera is.

        The board's own pose comes from touching those corners, not from these clicks. Paired
        with the touched positions they give the camera's pose, which is what perception needs
        to know which pixels are which square.
        """
        self.draft.board_corners_px = list(corners)
        if len(self.draft.touched) == len(calib.CAMERA_TOUCH_POINTS):
            return self.calibration_camera_fit()
        self.think(Thought.EXPLAIN, "Corners marked. Touch them with the gripper to place the camera.")
        return self.draft.steps()

    def calibration_set_square_size(self, millimetres: float) -> dict:
        """A known square size, checked against what the touches measure."""
        self.draft.expected_square_size_m = max(0.0, float(millimetres)) / 1000.0
        if self.draft.square_size_m:
            return self._apply_board()
        return self.draft.steps()

    def calibration_rotate_board(self) -> dict:
        """Relabel the marked corners half a turn: what was marked h8 becomes a1.

        Clicks and touches move together, since the fit pairs them by corner name.
        """
        if self.draft.board_corners_px is None:
            raise CapabilityError("mark the board's corners first")
        names = calib.CAMERA_TOUCH_POINTS
        half = len(names) // 2
        self.draft.board_corners_px = self.draft.board_corners_px[half:] + self.draft.board_corners_px[:half]
        self.draft.touched = {names[(i + half) % len(names)]: self.draft.touched[name]
                              for i, name in enumerate(names) if name in self.draft.touched}
        self.think(Thought.EXPLAIN, "Corners turned half a turn: what was marked h8 is now a1.")
        return self._apply_board()

    def calibration_recompute_board(self) -> dict:
        """Redo the board math against the touched corners."""
        return self._apply_board()

    def _apply_board(self) -> dict:
        board = calib.board_from_touches(self.draft.touched)
        if board is None:
            raise CapabilityError("touch all four corners first")
        self.draft.surface_height_m = board.origin_xyz[2]
        self.draft.board_origin_xyz = board.origin_xyz
        self.draft.board_yaw_rad = board.yaw_rad
        self.draft.square_size_m = board.square_size_m
        self.draft.squareness_m = board.squareness_m
        self.draft.unreachable = self.unreachable_squares(board.origin_xyz, board.yaw_rad, board.square_size_m)
        self.draft.orientation = self.orientation_check(board.origin_xyz, board.yaw_rad, board.square_size_m)
        expected = self.draft.expected_square_size_m
        check = ""
        if expected:
            error_mm = (board.square_size_m - expected) * 1000.0
            check = f" ({error_mm:+.1f} mm against the {expected * 1000:.1f} mm you gave)"
        self.think(
            Thought.VERIFY,
            f"Board at {tuple(round(v, 3) for v in board.origin_xyz)}, "
            f"{board.square_size_m * 1000:.1f} mm squares{check}, "
            f"{board.squareness_m * 1000:.1f} mm out of square, "
            f"{len(self.draft.unreachable)} squares out of reach.",
        )
        return self.draft.steps()

    def orientation_check(self, origin_xyz, yaw_rad: float, square_size_m: float) -> str:
        """Whether the board is the right way round, judged by which end the robot sits at.

        a1 is defined from White's seat, but the corners are marked on a photograph taken from
        somewhere else entirely, so it is easy to mark the board a half turn out. Nothing in the
        image says which way round it is - a board rotated 180 degrees still covers exactly the
        same squares - but the robot knows which colour it plays, and its own back rank has to
        be the end nearest it.
        """
        centres = calib.square_centres(origin_xyz, yaw_rad, square_size_m)
        near_rank_1 = min(math.hypot(*centres[f"{f}1"][:2]) for f in "abcdefgh")
        near_rank_8 = min(math.hypot(*centres[f"{f}8"][:2]) for f in "abcdefgh")
        robot_back_rank_is_near = near_rank_8 < near_rank_1 if self.robot_side == "black" else near_rank_1 < near_rank_8
        if robot_back_rank_is_near:
            return ""
        return (f"The robot plays {self.robot_side}, so rank {8 if self.robot_side == 'black' else 1} should be "
                "the end nearest it, and it is not. The corners are probably marked half a turn out.")

    def unreachable_squares(self, origin_xyz, yaw_rad: float, square_size_m: float) -> list[str]:
        """Which squares the arm cannot reach, at the height it grasps a piece."""
        out = []
        for square, (x, y, _z) in calib.square_centres(origin_xyz, yaw_rad, square_size_m).items():
            try:
                self.caps.solve_ik((x, y, skills.GRASP_HEIGHT), tool_down(0.0))
            except CapabilityError:
                out.append(square)
        return out

    def calibration_park_here(self) -> dict:
        """Take the arm's current joints as the park pose: jog it there, then save it.

        Applied straight away, so Home goes to it before the calibration is saved; saving is
        what makes it survive a restart.
        """
        self.draft.park_joints = self.caps.joint_positions()
        self.park_joints = list(self.draft.park_joints)
        self.think(Thought.EXPLAIN, f"Park pose set to {[round(v, 3) for v in self.draft.park_joints]}.")
        return self.draft.steps()

    def calibration_save(self) -> dict:
        """Hand the finished draft to the calibration node, which owns the stored profile."""
        if self.draft.board_origin_xyz is None or not self.draft.square_size_m:
            raise CapabilityError("mark the board's corners first")
        result = self.caps.set_calibration(
            self.draft.board_origin_xyz,
            self.draft.board_yaw_rad or 0.0,
            self.draft.square_size_m,
            self.draft.park_joints or [],
            self.draft.camera,
        )
        if self.draft.park_joints:
            self.park_joints = list(self.draft.park_joints)
        self.think(Thought.VERIFY, f"Calibration saved to {result.profile_path}.")
        return {"ok": result.ok, "message": result.message, "profile_path": result.profile_path,
                "steps": self.draft.steps()}

    def geometry_if_calibrated(self) -> BoardGeometry | None:
        try:
            return self.geometry()
        except CapabilityError:
            return None

    def believed_pieces(self, geometry: BoardGeometry) -> dict[str, tuple[float, float, float, float]]:
        """Every piece the brain believes is on the table, keyed by square or graveyard slot,
        as (x, y, z, height)."""
        with self._lock:
            pieces = {
                square_name(i): (*geometry.square_centre(square_name(i)), PIECE_HEIGHTS[p.lower()])
                for i, p in enumerate(self.board.squares)
                if p
            }
            for colour, captured in self.graveyard.items():
                for slot, piece in enumerate(captured):
                    pieces[f"grave_{colour}_{slot}"] = (*geometry.graveyard_slot(colour, slot), PIECE_HEIGHTS[piece.lower()])
        return pieces

    def job_demo_transfer(self, src: str, dst: str):
        previous = self.phase
        self.set_phase(GameState.PHASE_MOVING)
        geometry = self.geometry()
        self.think(Thought.ACT, f"Demo: transfer {src} → {dst}.")
        others = self.believed_pieces(geometry)
        others.pop(src, None)
        skills.transfer(self.caps, geometry, geometry.square_centre(src), geometry.square_centre(dst), list(others.values()))
        skills.park(self.caps, self.park_pose(), self.geometry_if_calibrated())
        self.set_phase(previous)


# --- REST API ------------------------------------------------------------------------


class NewGame(BaseModel):
    robot_side: str = "black"
    engine_elo: int = 0


class PressClock(BaseModel):
    move: str | None = None


class RobotMove(BaseModel):
    move: str


class SetPosition(BaseModel):
    fen: str
    robot_side: str = "black"


class DemoTransfer(BaseModel):
    src: str
    dst: str


class MoveTool(BaseModel):
    x: float
    y: float
    z: float
    yaw_deg: float = 0.0


class Jog(BaseModel):
    axis: str
    delta_m: float
    speed_pct: float = 100.0


class JogJoint(BaseModel):
    joint: str
    delta_rad: float
    speed_pct: float = 100.0


class Gripper(BaseModel):
    position: float | None = None
    open: bool | None = None


class BoardCorners(BaseModel):
    """The board's four outer corners in the camera image, in the order a1, h1, h8, a8."""

    corners: list[list[float]]


class SquareSize(BaseModel):
    """A known square size in millimetres, to check the measurement against. 0 clears it."""

    millimetres: float = 0.0


def build_app(node: BrainNode) -> FastAPI:
    app = FastAPI(title="chessbot brain")
    app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"])
    # Meshes and the vendored renderer are large and compress well.
    app.add_middleware(GZipMiddleware, minimum_size=1024)

    def submit(name, fn, *args):
        try:
            node.submit(name, fn, *args)
        except Busy as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        return {"accepted": name}

    def run_now(name, fn, *args):
        """A wizard step, run under the job lock, answering with the state it produced."""
        try:
            return node.run_now(name, fn, *args)
        except Busy as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        except CapabilityError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    @app.get("/api/state")
    def state():
        return node.snapshot()

    @app.post("/api/new_game")
    def new_game(body: NewGame):
        if body.robot_side not in ("white", "black"):
            raise HTTPException(status_code=422, detail="robot_side must be white or black")
        return submit("new_game", node.job_new_game, body.robot_side, body.engine_elo)

    @app.post("/api/press_clock")
    def press_clock(body: PressClock):
        return submit("press_clock", node.job_press_clock, body.move)

    @app.post("/api/calibrate")
    def calibrate():
        return submit("calibrate", node.job_calibrate)

    @app.post("/api/park")
    def park():
        return submit("park", node.job_park)

    @app.post("/api/resume")
    def resume():
        return submit("resume", node.job_resume)

    @app.post("/api/dev/set_position")
    def set_position(body: SetPosition):
        if body.robot_side not in ("white", "black"):
            raise HTTPException(status_code=422, detail="robot_side must be white or black")
        return submit("set_position", node.job_set_position, body.fen, body.robot_side)

    @app.post("/api/dev/robot_move")
    def robot_move(body: RobotMove):
        return submit("robot_move", node.job_robot_move, body.move)

    @app.post("/api/demo_transfer")
    def demo_transfer(body: DemoTransfer):
        return submit("demo_transfer", node.job_demo_transfer, body.src, body.dst)

    # --- manual control, for bringing a rig up and for debugging --------------------

    @app.get("/api/dev/tool_pose")
    def tool_pose():
        try:
            (x, y, z), yaw = node.caps.tool_pose()
        except CapabilityError as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc
        return {"xyz": [round(x, 4), round(y, 4), round(z, 4)], "yaw_deg": round(math.degrees(yaw), 1),
                "joints": node.caps.joint_positions(), "box": [list(a) for a in JOG_BOX]}

    @app.post("/api/dev/move_tool")
    def move_tool(body: MoveTool):
        return submit("move_tool", node.job_move_tool, (body.x, body.y, body.z), body.yaw_deg)

    @app.post("/api/dev/jog")
    def jog(body: Jog):
        if body.axis not in ("x", "y", "z"):
            raise HTTPException(status_code=422, detail="axis must be x, y or z")
        return submit("jog", node.job_jog, body.axis, body.delta_m, body.speed_pct)

    @app.post("/api/dev/jog_joint")
    def jog_joint(body: JogJoint):
        return submit("jog_joint", node.job_jog_joint, body.joint, body.delta_rad, body.speed_pct)

    @app.post("/api/dev/gripper")
    def gripper(body: Gripper):
        position = body.position
        if position is None:
            position = node.caps.arm.gripper_open if body.open else node.caps.arm.gripper_closed
        return submit("gripper", node.job_gripper, float(position))

    @app.get("/api/camera/frame.png")
    def camera_frame(scale: int = 2):
        """The latest overhead frame, for the calibration workflow and for looking at."""
        frame = node.caps.camera_frame(scale=max(1, min(scale, 8)))
        if frame is None:
            raise HTTPException(status_code=503, detail="no camera frames")
        return Response(bytes(frame.data), media_type="image/png",
                        headers={"Cache-Control": "no-store"})

    # --- guided calibration ---------------------------------------------------------

    @app.get("/api/calibration")
    def calibration():
        stored = node.caps.kv_get("chessbot/kv/calibration")
        return {"stored": stored, "draft": node.draft.steps()}

    @app.post("/api/calibration/camera")
    def calibration_camera():
        return run_now("calibrate_camera", node.calibration_camera_fit)

    @app.post("/api/calibration/camera/release")
    def calibration_camera_release():
        return run_now("release_arm", node.calibration_release_arm)

    @app.post("/api/calibration/camera/touch")
    def calibration_camera_touch():
        return run_now("touch", node.calibration_touch)

    @app.post("/api/calibration/camera/reactivate")
    def calibration_camera_reactivate():
        return run_now("reactivate_arm", node.calibration_reactivate_arm)

    @app.post("/api/calibration/board")
    def calibration_board(body: BoardCorners):
        if len(body.corners) != 4:
            raise HTTPException(status_code=422, detail="give four corners: a1, h1, h8, a8")
        try:
            return node.calibration_board([(float(c[0]), float(c[1])) for c in body.corners])
        except CapabilityError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    @app.post("/api/calibration/board/recompute")
    def calibration_board_recompute():
        try:
            return node.calibration_recompute_board()
        except CapabilityError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    @app.post("/api/calibration/square_size")
    def calibration_square_size(body: SquareSize):
        try:
            return node.calibration_set_square_size(body.millimetres)
        except CapabilityError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    @app.post("/api/calibration/board/rotate")
    def calibration_board_rotate():
        try:
            return node.calibration_rotate_board()
        except CapabilityError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    @app.post("/api/calibration/park_here")
    def calibration_park_here():
        try:
            return node.calibration_park_here()
        except CapabilityError as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc

    @app.post("/api/calibration/save")
    def calibration_save():
        try:
            return node.calibration_save()
        except CapabilityError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    @app.get("/api/robot_description")
    def robot_description():
        if not node.robot_description:
            raise HTTPException(status_code=503, detail="no robot_description received yet")
        return Response(node.robot_description, media_type="application/xml")

    @app.get("/packages/{package}/{path:path}")
    def package_file(package: str, path: str):
        """Resolves package:// URIs for the browser (meshes and asset descriptions only)."""
        if os.path.splitext(path)[1].lower() not in PACKAGE_FILE_EXTENSIONS:
            raise HTTPException(status_code=404)
        try:
            share = os.path.normpath(get_package_share_directory(package))
        except (PackageNotFoundError, ValueError) as exc:
            raise HTTPException(status_code=404) from exc
        # normpath, not realpath: symlink-installed files point outside the share directory.
        full = os.path.normpath(os.path.join(share, path))
        if not full.startswith(share + os.sep) or not os.path.isfile(full):
            raise HTTPException(status_code=404)
        return FileResponse(full)

    if node.web_root and os.path.isdir(node.web_root):
        # follow_symlink: a --symlink-install workspace installs the files as symlinks.
        app.mount("/", StaticFiles(directory=node.web_root, html=True, follow_symlink=True), name="web")
    return app


def main():
    rclpy.init()
    node = BrainNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    server = uvicorn.Server(uvicorn.Config(build_app(node), host="0.0.0.0", port=node.rest_port, log_level="warning"))
    node.get_logger().info(f"REST API on :{node.rest_port}")
    try:
        server.run()
    except KeyboardInterrupt:
        pass
    finally:
        if node.engine is not None:
            node.engine.close()
        node.caps.close()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
