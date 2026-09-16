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
import os
import threading
import time
from concurrent.futures import ThreadPoolExecutor

import rclpy
import uvicorn
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from builtin_interfaces.msg import Time
from chessbot_interfaces.msg import GameState, Thought
from chessbot_interfaces.srv import GetBoardState
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

from chessbot_brain import skills
from chessbot_brain.board import Board, piece_colour
from chessbot_brain.capabilities import ArmConfig, Capabilities, CapabilityError
from chessbot_brain.engine import EngineError, UciEngine, choose_reply
from chessbot_brain.geometry import BoardGeometry, graveyard_cell

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
    GetBoardState.Response.RESULT_NO_BOARD: "camera frame received but no board located",
}


MESH_EXTENSIONS = {".stl", ".dae", ".obj", ".glb", ".gltf"}


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
        # Folded upright over the base, clear of the board and graveyards (tools/dev/park_search.py).
        self.park_joints = list(self.declare_parameter("park_joints", [0.0, -0.54, -1.02, -0.30, 0.0]).value)
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
            joint_speed=float(self.declare_parameter("joint_speed", 0.8).value),
        )
        zenoh_endpoint = self.declare_parameter("zenoh_endpoint", "tcp/127.0.0.1:7447").value
        default_stockfish = os.path.join(os.environ.get("PIXI_PROJECT_ROOT", ""), "generated", "bin", "stockfish")
        stockfish_path = self.declare_parameter("stockfish_path", default_stockfish).value
        self.engine_movetime_ms = int(self.declare_parameter("engine_movetime_ms", 800).value)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.state_pub = self.create_publisher(GameState, "/chessbot/game_state", latched)
        self.thought_pub = self.create_publisher(Thought, "/chessbot/thoughts", 50)
        # Kept for the UI's 3D view, which draws the robot from its URDF.
        self.robot_description = ""
        self.create_subscription(String, "/robot_description", self._on_robot_description, latched)

        self.caps = Capabilities(self, arm, zenoh_endpoint)

        self._lock = threading.RLock()
        self._jobs = ThreadPoolExecutor(max_workers=1)
        self._busy = threading.Lock()
        self.thoughts: collections.deque = collections.deque(maxlen=200)
        self.last_error = ""
        self.current_job = ""

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
        response = self.caps.board_state()
        self.think(Thought.PERCEIVE, f"Perception: {BOARD_RESULT_TEXT.get(response.result, 'unknown result')}.")

        if move is None:
            # TODO(chessbot): infer the move by comparing board states against legal moves.
            self.think(
                Thought.RECOVER,
                "I can't work out your move from the camera yet. Pass it explicitly (e.g. e2e4).",
            )
            self.set_phase(GameState.PHASE_HUMAN_TURN)
            return

        move = self._legal_form(move)
        if move is None:
            self.think(Thought.RECOVER, "That isn't a legal move here. Your clock keeps running.")
            self.set_phase(GameState.PHASE_HUMAN_TURN)
            return

        with self._lock:
            self.board.apply(move)
            self.moves.append(move)
        self.think(Thought.DECIDE, f"Your move: {move}.")
        if self._game_over():
            return
        self._robot_turn()

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

    def _robot_turn(self):
        self._switch_clock(self.robot_side)
        self.set_phase(GameState.PHASE_THINKING)
        if self.engine is not None:
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

        self.set_phase(GameState.PHASE_MOVING)
        geometry = self.geometry()
        effects = self.board.effects(reply)
        self.think(Thought.PLAN, f"Plan for {reply}: pick, place{', capture first' if effects.captured else ''}.")
        skills.execute_chess_move(
            self.caps, geometry, effects, self.graveyard, lambda text: self.think(Thought.ACT, text)
        )
        with self._lock:
            self.board.apply(reply)
            self.moves.append(reply)
        self.publish_state()

        self.set_phase(GameState.PHASE_VERIFYING)
        skills.park(self.caps, self.park_joints)
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

    def job_park(self):
        skills.park(self.caps, self.park_joints)
        self.think(Thought.ACT, "Arm parked.")

    def job_resume(self):
        self.think(Thought.RECOVER, "Resumed by operator.")
        self.set_phase(GameState.PHASE_HUMAN_TURN)

    def job_demo_transfer(self, src: str, dst: str):
        previous = self.phase
        self.set_phase(GameState.PHASE_MOVING)
        geometry = self.geometry()
        self.think(Thought.ACT, f"Demo: transfer {src} → {dst}.")
        skills.transfer(self.caps, geometry.square_centre(src), geometry.square_centre(dst), geometry.diagonal_yaws())
        skills.park(self.caps, self.park_joints)
        self.set_phase(previous)


# --- REST API ------------------------------------------------------------------------


class NewGame(BaseModel):
    robot_side: str = "black"
    engine_elo: int = 0


class PressClock(BaseModel):
    move: str | None = None


class SetPosition(BaseModel):
    fen: str
    robot_side: str = "black"


class DemoTransfer(BaseModel):
    src: str
    dst: str


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

    @app.post("/api/demo_transfer")
    def demo_transfer(body: DemoTransfer):
        return submit("demo_transfer", node.job_demo_transfer, body.src, body.dst)

    @app.get("/api/robot_description")
    def robot_description():
        if not node.robot_description:
            raise HTTPException(status_code=503, detail="no robot_description received yet")
        return Response(node.robot_description, media_type="application/xml")

    @app.get("/packages/{package}/{path:path}")
    def package_file(package: str, path: str):
        """Resolves the URDF's package:// mesh URIs for the browser (meshes only)."""
        if os.path.splitext(path)[1].lower() not in MESH_EXTENSIONS:
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
