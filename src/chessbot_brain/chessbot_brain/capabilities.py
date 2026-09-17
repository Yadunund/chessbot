# Copyright 2026 Yadunund Vijay
# SPDX-License-Identifier: Apache-2.0
"""Clients for every capability the brain composes.

The brain is a client only: perception, motion planning, the arm and gripper
controllers, calibration, and the key-value store. Each call here blocks the
calling (worker) thread while the node's executor spins elsewhere, so skills
can be written as plain sequential code.
"""

from __future__ import annotations

import json
import math
import struct
import threading
import zlib
from dataclasses import dataclass

import zenoh
from chessbot_interfaces.action import Calibrate
from chessbot_interfaces.srv import GetBoardState, Reason
from control_msgs.action import FollowJointTrajectory, ParallelGripperCommand
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetCartesianPath, GetPositionIK
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from rclpy.wait_for_message import wait_for_message
from sensor_msgs.msg import CompressedImage, Image
from tf2_ros import Buffer, TransformException, TransformListener
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Tool Z pointing straight down: 180 degrees about X.
DOWN = (1.0, 0.0, 0.0, 0.0)


def tool_down(yaw: float) -> tuple[float, float, float, float]:
    """Tool pointing straight down with the jaws opening along `yaw` (x, y, z, w).

    Tool frame: +Z approach, +X from the fixed jaw towards the moving jaw. This is
    a half turn about the horizontal axis at yaw/2.
    """
    return (math.cos(yaw / 2), math.sin(yaw / 2), 0.0, 0.0)


class CapabilityError(RuntimeError):
    """A capability call failed; the message says which and why."""


def encode_png(data: bytes, width: int, height: int, step: int, bgr: bool, scale: int = 1) -> bytes:
    """RGB PNG from packed rows (every `scale`-th pixel), without an image library."""
    rows = []
    for y in range(0, height, scale):
        row = data[y * step: y * step + width * 3]
        pixels = bytearray(row[0::1])
        if scale > 1:
            pixels = bytearray(b"".join(row[x * 3: x * 3 + 3] for x in range(0, width, scale)))
        if bgr:
            pixels[0::3], pixels[2::3] = pixels[2::3], pixels[0::3]
        rows.append(b"\x00" + bytes(pixels))
    out_w = (width + scale - 1) // scale
    out_h = len(rows)

    def chunk(kind: bytes, payload: bytes) -> bytes:
        return struct.pack(">I", len(payload)) + kind + payload + struct.pack(">I", zlib.crc32(kind + payload) & 0xFFFFFFFF)

    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", out_w, out_h, 8, 2, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(b"".join(rows), 6)) + chunk(b"IEND", b""))


def _wait(future, timeout_s: float, what: str):
    done = threading.Event()
    future.add_done_callback(lambda _f: done.set())
    if not done.wait(timeout_s):
        future.cancel()
        raise CapabilityError(f"{what}: no response within {timeout_s:.0f}s")
    return future.result()


@dataclass
class ArmConfig:
    joints: list[str]
    gripper_joint: str
    gripper_open: float
    gripper_closed: float
    joint_speed: float  # rad/s used for timing joint-space moves
    # The grasp point is on the fixed jaw's inner surface. To pick, it stops this far
    # from the piece's centre (piece radius plus clearance); to place, one piece
    # radius, so the released piece lands centred.
    pick_offset: float = 0.010
    place_offset: float = 0.0075
    piece_radius: float = 0.0075
    # Least acceptable gap between the gripper and any other piece while grasping.
    clearance: float = 0.005
    tip_frame: str = "gripper_frame_link"
    base_frame: str = "base_link"
    # The jaws' opening direction in tip_frame (matches the motion node's opening_axis).
    opening_axis: tuple[float, float, float] = (-1.0, 0.0, 0.0)


class Capabilities:
    def __init__(self, node: Node, arm: ArmConfig, zenoh_endpoint: str):
        self.node = node
        self.arm = arm
        self._joint_lock = threading.Lock()
        self._joints: dict[str, float] = {}
        node.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)

        self.perception = node.create_client(GetBoardState, "/perception/get_board_state")
        self.reasoner = node.create_client(Reason, "/inference/reason")
        self.ik = node.create_client(GetPositionIK, "/compute_ik")
        self.cartesian = node.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.arm_action = ActionClient(node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
        self.gripper_action = ActionClient(node, ParallelGripperCommand, "/gripper_controller/gripper_cmd")
        self.calibrate_action = ActionClient(node, Calibrate, "/calibration/calibrate")

        config = zenoh.Config()
        config.insert_json5("mode", '"client"')
        config.insert_json5("connect/endpoints", f'["{zenoh_endpoint}"]')
        self.zenoh = zenoh.open(config)

    def close(self):
        self.zenoh.close()

    # --- state -------------------------------------------------------------------

    def _on_joint_states(self, msg: JointState):
        with self._joint_lock:
            self._joints.update(zip(msg.name, msg.position))

    def joint_positions(self) -> list[float]:
        with self._joint_lock:
            if not all(j in self._joints for j in self.arm.joints):
                raise CapabilityError("joint states not received yet")
            return [self._joints[j] for j in self.arm.joints]

    def tool_pose(self) -> tuple[tuple[float, float, float], float]:
        """Current grasp point position and jaw yaw (opening direction) in the base frame."""
        try:
            t = self.tf_buffer.lookup_transform(self.arm.base_frame, self.arm.tip_frame, Time()).transform
        except TransformException as exc:
            raise CapabilityError(f"no tool pose: {exc}") from exc
        x, y, z, w = t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w
        rotation = [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
        ox = sum(rotation[0][i] * self.arm.opening_axis[i] for i in range(3))
        oy = sum(rotation[1][i] * self.arm.opening_axis[i] for i in range(3))
        return (t.translation.x, t.translation.y, t.translation.z), math.atan2(oy, ox)

    def status(self) -> dict:
        """Which capabilities are reachable right now (for the UI and checks)."""
        return {
            "perception": self.perception.service_is_ready(),
            "ik": self.ik.service_is_ready(),
            "cartesian_path": self.cartesian.service_is_ready(),
            "arm_controller": self.arm_action.server_is_ready(),
            "gripper_controller": self.gripper_action.server_is_ready(),
            "calibration": self.calibrate_action.server_is_ready(),
            "reasoning": self.reasoner.service_is_ready(),
        }

    # --- key-value store --------------------------------------------------------------

    def kv_get(self, key: str, timeout_s: float = 2.0) -> dict | None:
        for reply in self.zenoh.get(key, timeout=timeout_s):
            if reply.ok is not None:
                return json.loads(reply.ok.payload.to_string())
        return None

    # --- perception ---------------------------------------------------------------

    def board_state(self, max_frame_age_s: float = 2.0) -> GetBoardState.Response:
        if not self.perception.wait_for_service(timeout_sec=2.0):
            raise CapabilityError("perception is not available")
        request = GetBoardState.Request(max_frame_age_s=max_frame_age_s)
        return _wait(self.perception.call_async(request), 5.0, "perception")

    # --- reasoning ----------------------------------------------------------------------

    def reason(self, role: int, prompt: str, images=(), json_schema: str = "", timeout_s: float = 60.0) -> Reason.Response | None:
        """Ask the reasoning model; None when it is not running."""
        if not self.reasoner.wait_for_service(timeout_sec=1.0):
            return None
        request = Reason.Request(role=role, prompt=prompt, images=list(images), json_schema=json_schema)
        response = _wait(self.reasoner.call_async(request), timeout_s, "reasoning")
        return response if response.result == Reason.Response.RESULT_OK else None

    def camera_frame(self, topic: str = "/overhead_camera/image_raw", scale: int = 2, timeout_s: float = 3.0) -> CompressedImage | None:
        """One camera frame as PNG (downscaled), for questions to the reasoning model."""
        ok, msg = wait_for_message(Image, self.node, topic, qos_profile=qos_profile_sensor_data, time_to_wait=timeout_s)
        if not ok or msg.encoding not in ("rgb8", "bgr8"):
            return None
        png = encode_png(bytes(msg.data), msg.width, msg.height, msg.step, msg.encoding == "bgr8", scale)
        return CompressedImage(header=msg.header, format="png", data=png)

    # --- motion planning ------------------------------------------------------------

    @staticmethod
    def _pose(xyz, quat=DOWN) -> Pose:
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = xyz
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quat
        return pose

    def solve_ik(self, xyz, quat=DOWN, frame_id: str = "base_link") -> list[float]:
        if not self.ik.wait_for_service(timeout_sec=2.0):
            raise CapabilityError("IK is not available")
        request = GetPositionIK.Request()
        request.ik_request.group_name = "arm"
        request.ik_request.pose_stamped = PoseStamped()
        request.ik_request.pose_stamped.header.frame_id = frame_id
        request.ik_request.pose_stamped.pose = self._pose(xyz, quat)
        response = _wait(self.ik.call_async(request), 10.0, "IK")
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise CapabilityError(f"no IK solution for {tuple(round(v, 3) for v in xyz)}")
        by_name = dict(zip(response.solution.joint_state.name, response.solution.joint_state.position))
        return [by_name[j] for j in self.arm.joints]

    def plan_cartesian(self, waypoints_xyz, quat=DOWN, frame_id: str = "base_link") -> JointTrajectory:
        if not self.cartesian.wait_for_service(timeout_sec=2.0):
            raise CapabilityError("Cartesian planning is not available")
        request = GetCartesianPath.Request()
        request.header.frame_id = frame_id
        request.group_name = "arm"
        request.waypoints = [self._pose(xyz, quat) for xyz in waypoints_xyz]
        request.max_step = 0.005
        response = _wait(self.cartesian.call_async(request), 10.0, "Cartesian planning")
        if response.fraction < 0.999:
            raise CapabilityError(f"Cartesian path only {response.fraction:.0%} feasible")
        return response.solution.joint_trajectory

    def joint_move_trajectory(self, target: list[float]) -> JointTrajectory:
        """A two-point trajectory to a joint target, timed at the configured speed."""
        current = self.joint_positions()
        travel = max(abs(a - b) for a, b in zip(current, target))
        duration = max(1.0, travel / self.arm.joint_speed)
        trajectory = JointTrajectory()
        trajectory.joint_names = list(self.arm.joints)
        start = JointTrajectoryPoint(positions=current, time_from_start=Duration(seconds=0.0).to_msg())
        end = JointTrajectoryPoint(positions=list(target), time_from_start=Duration(seconds=duration).to_msg())
        trajectory.points = [start, end]
        return trajectory

    # --- control -----------------------------------------------------------------------

    def execute(self, trajectory: JointTrajectory, timeout_s: float = 60.0):
        if not self.arm_action.wait_for_server(timeout_sec=2.0):
            raise CapabilityError("arm controller is not available")
        goal = FollowJointTrajectory.Goal(trajectory=trajectory)
        handle = _wait(self.arm_action.send_goal_async(goal), 5.0, "arm controller goal")
        if not handle.accepted:
            raise CapabilityError("arm controller rejected the trajectory")
        result = _wait(handle.get_result_async(), timeout_s, "arm controller")
        if result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise CapabilityError(f"trajectory failed: {result.result.error_string or result.result.error_code}")

    def gripper(self, position: float, timeout_s: float = 30.0):
        if not self.gripper_action.wait_for_server(timeout_sec=2.0):
            raise CapabilityError("gripper controller is not available")
        goal = ParallelGripperCommand.Goal()
        goal.command.name = [self.arm.gripper_joint]
        goal.command.position = [position]
        handle = _wait(self.gripper_action.send_goal_async(goal), 5.0, "gripper goal")
        if not handle.accepted:
            raise CapabilityError("gripper controller rejected the command")
        # A closed gripper stalls on the piece; that still counts as done.
        _wait(handle.get_result_async(), timeout_s, "gripper")

    # --- calibration --------------------------------------------------------------------

    def run_calibration(self, force: bool = True, timeout_s: float = 300.0) -> Calibrate.Result:
        if not self.calibrate_action.wait_for_server(timeout_sec=2.0):
            raise CapabilityError("calibration is not available")
        handle = _wait(self.calibrate_action.send_goal_async(Calibrate.Goal(force=force)), 5.0, "calibration goal")
        if not handle.accepted:
            raise CapabilityError("calibration rejected the request")
        return _wait(handle.get_result_async(), timeout_s, "calibration").result
