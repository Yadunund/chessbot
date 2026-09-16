#!/usr/bin/env python3
# Copyright 2026 Yadunund Vijay
# SPDX-License-Identifier: Apache-2.0
"""Check that a running chessbot stack is wired up correctly.

Exercises every interface boundary: router and its REST plugin, the key-value
store, camera and robot topics, perception, motion planning, the controllers,
calibration, the brain's REST API and state topics, the browser's SSE path, and
the Rerun stream.

    pixi run check            # connectivity only
    pixi run check --e2e      # also play one move in simulation
"""

from __future__ import annotations

import argparse
import json
import socket
import sys
import threading
import time
import urllib.error
import urllib.request

import rclpy
from chessbot_interfaces.action import Calibrate
from chessbot_interfaces.msg import GameState, Thought
from chessbot_interfaces.srv import GetBoardState
from control_msgs.action import FollowJointTrajectory, ParallelGripperCommand
from geometry_msgs.msg import Pose
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetCartesianPath, GetPositionIK
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, JointState

ROUTER = "http://127.0.0.1:8080"
BRAIN = "http://127.0.0.1:8000"
RESULTS: list[tuple[str, bool, str]] = []


def record(name: str, ok: bool, detail: str = ""):
    RESULTS.append((name, ok, detail))
    print(f"  {'PASS' if ok else 'FAIL'}  {name}{('  -  ' + detail) if detail else ''}", flush=True)


def http(method: str, url: str, body: dict | None = None, timeout: float = 5.0):
    data = json.dumps(body).encode() if body is not None else None
    request = urllib.request.Request(url, data=data, method=method, headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(request, timeout=timeout) as response:
        raw = response.read()
        return response.status, (json.loads(raw) if raw else None)


def wait_future(future, timeout: float):
    done = threading.Event()
    future.add_done_callback(lambda _f: done.set())
    return future.result() if done.wait(timeout) else None


class Probe(Node):
    def __init__(self):
        super().__init__("chessbot_check")
        self.received: dict[str, object] = {}
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Clock, "/clock", lambda m: self._got("/clock", m), qos_profile_sensor_data)
        self.create_subscription(JointState, "/joint_states", lambda m: self._got("/joint_states", m), 10)
        for topic in ("/overhead_camera/image_raw", "/wrist_camera/image_raw"):
            self.create_subscription(Image, topic, lambda m, t=topic: self._got(t, m), qos_profile_sensor_data)
        self.create_subscription(GameState, "/chessbot/game_state", lambda m: self._got("/chessbot/game_state", m), latched)
        self.thought_count = 0
        self.create_subscription(Thought, "/chessbot/thoughts", self._on_thought, 50)

    def _got(self, topic, msg):
        self.received[topic] = msg

    def _on_thought(self, _msg):
        self.thought_count += 1


def wait_for(predicate, timeout: float, interval: float = 0.2) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return False


def check_router():
    print("Router and key-value store")
    try:
        status, _ = http("GET", f"{ROUTER}/@/*/router")
        record("router REST plugin", status == 200)
    except Exception as exc:  # noqa: BLE001
        record("router REST plugin", False, str(exc))
    try:
        status, replies = http("GET", f"{ROUTER}/chessbot/kv/calibration")
        value = replies[0]["value"] if replies else None
        record("calibration in key-value store", bool(value) and value.get("version") == 1, f"{len(replies or [])} reply")
    except Exception as exc:  # noqa: BLE001
        record("calibration in key-value store", False, str(exc))


def check_topics(probe: Probe):
    print("Topics")
    for topic in ("/clock", "/joint_states", "/overhead_camera/image_raw", "/wrist_camera/image_raw", "/chessbot/game_state"):
        ok = wait_for(lambda t=topic: t in probe.received, 20.0)
        detail = ""
        if ok and topic.endswith("image_raw"):
            img = probe.received[topic]
            detail = f"{img.width}x{img.height} {img.encoding}"
        record(f"receiving {topic}", ok, detail)


def check_services(probe: Probe):
    print("Capabilities")
    perception = probe.create_client(GetBoardState, "/perception/get_board_state")
    if perception.wait_for_service(timeout_sec=10.0):
        response = wait_future(perception.call_async(GetBoardState.Request(max_frame_age_s=0.0)), 5.0)
        record(
            "perception /perception/get_board_state",
            response is not None and response.result in (GetBoardState.Response.RESULT_OK, GetBoardState.Response.RESULT_NO_BOARD),
            "no response" if response is None else f"result={response.result}, frame stamp {response.state.header.stamp.sec}s",
        )
    else:
        record("perception /perception/get_board_state", False, "service not available")

    ik = probe.create_client(GetPositionIK, "/compute_ik")
    solution = None
    if ik.wait_for_service(timeout_sec=10.0):
        request = GetPositionIK.Request()
        request.ik_request.group_name = "arm"
        request.ik_request.pose_stamped.header.frame_id = "base_link"
        request.ik_request.pose_stamped.pose = _down_pose(0.234, 0.0, 0.08)
        response = wait_future(ik.call_async(request), 15.0)
        ok = response is not None and response.error_code.val == MoveItErrorCodes.SUCCESS
        solution = response.solution if ok else None
        record("motion /compute_ik (board centre, tool down)", ok, "" if ok else f"error {getattr(response, 'error_code', None)}")
    else:
        record("motion /compute_ik", False, "service not available")

    cartesian = probe.create_client(GetCartesianPath, "/compute_cartesian_path")
    if cartesian.wait_for_service(timeout_sec=10.0) and solution is not None:
        request = GetCartesianPath.Request()
        request.header.frame_id = "base_link"
        request.group_name = "arm"
        request.start_state = solution
        request.waypoints = [_down_pose(0.234, 0.0, 0.02)]
        request.max_step = 0.005
        response = wait_future(cartesian.call_async(request), 15.0)
        ok = response is not None and response.fraction > 0.999
        record(
            "motion /compute_cartesian_path (6 cm descent)",
            ok,
            "no response" if response is None else f"fraction={response.fraction:.2f}, {len(response.solution.joint_trajectory.points)} points",
        )
    else:
        record("motion /compute_cartesian_path", False, "service not available or no IK seed")

    for name, action_type, action in (
        ("arm controller", FollowJointTrajectory, "/arm_controller/follow_joint_trajectory"),
        ("gripper controller", ParallelGripperCommand, "/gripper_controller/gripper_cmd"),
    ):
        client = ActionClient(probe, action_type, action)
        record(f"{name} {action}", client.wait_for_server(timeout_sec=10.0))

    calibrate = ActionClient(probe, Calibrate, "/calibration/calibrate")
    if calibrate.wait_for_server(timeout_sec=10.0):
        handle = wait_future(calibrate.send_goal_async(Calibrate.Goal(force=True)), 5.0)
        result = wait_future(handle.get_result_async(), 30.0) if handle and handle.accepted else None
        ok = result is not None and result.result.result == Calibrate.Result.RESULT_OK
        record("calibration /calibration/calibrate", ok, result.result.profile_path if ok else "no result")
    else:
        record("calibration /calibration/calibrate", False, "action not available")


def _down_pose(x, y, z) -> Pose:
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = x, y, z
    pose.orientation.x, pose.orientation.w = 1.0, 0.0
    return pose


def check_brain_and_ui():
    print("Brain, UI and visualisation")
    try:
        status, snapshot = http("GET", f"{BRAIN}/api/state")
        caps = snapshot.get("capabilities", {})
        missing = [name for name, ok in caps.items() if not ok]
        record("brain REST /api/state", status == 200, f"phase={snapshot.get('phase')}")
        record("brain sees every capability", not missing, "missing: " + ", ".join(missing) if missing else "")
    except Exception as exc:  # noqa: BLE001
        record("brain REST /api/state", False, str(exc))
    try:
        with urllib.request.urlopen(f"{BRAIN}/", timeout=5.0) as response:
            page = response.read().decode(errors="replace")
        record("UI served at /", response.status == 200 and "<title>Chessbot</title>" in page)
    except Exception as exc:  # noqa: BLE001
        record("UI served at /", False, str(exc))

    # SSE: listen on the game state topic through the router, then make the brain publish.
    events: list[str] = []

    def listen():
        try:
            request = urllib.request.Request(
                f"{ROUTER}/0/chessbot/game_state/**", headers={"Accept": "text/event-stream"}
            )
            with urllib.request.urlopen(request, timeout=40) as stream:
                for line in stream:
                    if line.startswith(b"data:"):
                        events.append(line.decode())
                        return
        except Exception:  # noqa: BLE001 - the timeout ends the listener
            return

    listener = threading.Thread(target=listen, daemon=True)
    listener.start()
    time.sleep(1.0)
    try:
        http("POST", f"{BRAIN}/api/park", {})
    except urllib.error.HTTPError:
        pass  # busy is fine: any state publication will do
    ok = wait_for(lambda: bool(events), 35.0)
    detail = ""
    if ok:
        sample = json.loads(events[0].split("data:", 1)[1])
        detail = sample["key"].split("/RIHS01")[0]
    record("browser path: game state over router SSE", ok, detail)

    try:
        with socket.create_connection(("127.0.0.1", 9876), timeout=3.0):
            record("Rerun gRPC stream on :9876", True)
    except OSError as exc:
        record("Rerun gRPC stream on :9876", False, str(exc))


def check_end_to_end(probe: Probe):
    print("End to end (simulation)")

    def phase() -> str:
        return http("GET", f"{BRAIN}/api/state")[1]["phase"]

    def wait_phase(target: str, timeout: float) -> bool:
        return wait_for(lambda: phase() == target, timeout, interval=1.0)

    wait_for(lambda: phase() not in ("moving", "setup", "verifying", "thinking"), 60.0, interval=1.0)
    http("POST", f"{BRAIN}/api/new_game", {"robot_side": "black"})
    record("new game reaches human_turn", wait_phase("human_turn", 90.0), f"phase={phase()}")

    before = list(probe.received["/joint_states"].position)
    thoughts_before = probe.thought_count
    http("POST", f"{BRAIN}/api/press_clock", {"move": "e2e4"})
    wait_for(lambda: phase() != "human_turn", 10.0, interval=0.5)
    moved = wait_phase("human_turn", 240.0)
    snapshot = http("GET", f"{BRAIN}/api/state")[1]
    after = list(probe.received["/joint_states"].position)
    record(
        "human e2e4 → robot replies and hands the turn back",
        moved and len(snapshot["moves"]) == 2,
        f"moves={snapshot['moves']} phase={snapshot['phase']} error={snapshot['last_error'] or '-'}",
    )
    record("arm moved in simulation", any(abs(a - b) > 0.05 for a, b in zip(before, after)))
    record("thought feed published", probe.thought_count - thoughts_before >= 4, f"{probe.thought_count - thoughts_before} thoughts")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--e2e", action="store_true", help="also play one move end to end in simulation")
    args = parser.parse_args()

    rclpy.init()
    probe = Probe()
    executor = MultiThreadedExecutor()
    executor.add_node(probe)
    threading.Thread(target=executor.spin, daemon=True).start()
    try:
        check_router()
        check_topics(probe)
        check_services(probe)
        check_brain_and_ui()
        if args.e2e:
            check_end_to_end(probe)
    finally:
        executor.shutdown()
        probe.destroy_node()
        rclpy.try_shutdown()

    failed = [name for name, ok, _ in RESULTS if not ok]
    print(f"\n{len(RESULTS) - len(failed)}/{len(RESULTS)} checks passed")
    for name in failed:
        print(f"  failed: {name}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
