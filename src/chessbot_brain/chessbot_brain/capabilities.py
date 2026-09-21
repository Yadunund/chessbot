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
import time
import zlib
from dataclasses import dataclass

import zenoh
from chessbot_interfaces.action import Calibrate
from chessbot_interfaces.srv import GetBoardState, Reason, SetCalibration
from control_msgs.action import FollowJointTrajectory, ParallelGripperCommand
from controller_manager_msgs.srv import SetHardwareComponentState, SwitchController
from geometry_msgs.msg import Pose, PoseStamped
from lifecycle_msgs.msg import State as LifecycleState
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetCartesianPath, GetPositionIK
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
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


def open_zenoh(config, endpoint: str, logger=None, timeout_s: float = 30.0):
    """Open a Zenoh session, waiting for the router instead of dying if it is not up yet.

    Launch order should not decide whether the application comes up. A node that started a
    moment before the router used to exit with a bare traceback, leaving a stack that looked
    half alive - the nodes that do not use the store kept running, and the UI simply never
    appeared.
    """
    deadline = time.monotonic() + timeout_s
    warned = False
    while True:
        try:
            return zenoh.open(config)
        except Exception as exc:  # noqa: BLE001 - any failure to reach the router
            if time.monotonic() >= deadline:
                raise CapabilityError(
                    f"no Zenoh router at {endpoint} after {timeout_s:.0f}s; start one with `pixi run router`"
                ) from exc
            if logger is not None and not warned:
                logger.warning(f"Waiting for the Zenoh router at {endpoint}...")
                warned = True
            time.sleep(1.0)


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
    # Height of the grasp point above the playing surface, metres. High enough that a tilted
    # tool at the far rank keeps its heel off the board, and that the arm sagging under its own
    # weight (see the controller tolerances) still leaves the jaws above the squares.
    grasp_height: float = 0.028
    # Placing releases this much higher again, so a piece that slipped in the jaws is never
    # pushed into the board.
    place_drop: float = 0.003
    pick_offset: float = 0.010
    place_offset: float = 0.0075
    piece_radius: float = 0.0075
    # Least acceptable gap between the gripper and any other piece while grasping.
    clearance: float = 0.005
    tip_frame: str = "gripper_frame_link"
    base_frame: str = "base_link"
    # The jaws' opening direction in tip_frame (matches the motion node's opening_axis).
    opening_axis: tuple[float, float, float] = (-1.0, 0.0, 0.0)
    # The closed jaw tips, which are what touches a board corner. Offset from tip_frame in the
    # description, so RViz shows it and nothing here has to carry the number.
    touch_frame: str = "gripper_tip_link"
    # ros2_control hardware component name (the <ros2_control name="..."> in the URDF), for
    # releasing/reactivating the arm - e.g. to hand-guide it during camera calibration.
    hardware_component: str = "SO_ARM101"
    # ros2_control controller name commanding the arm, for the same release/reactivate cycle.
    controller: str = "arm_controller"
    # Its joint shares the arm's hardware component, so both controllers switch together.
    gripper_controller: str = "gripper_controller"


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
        # The controller's topic interface, for commands that must not wait. See `stream`.
        self.arm_stream = node.create_publisher(
            JointTrajectory, f"/{self.arm.controller}/joint_trajectory", 10)
        self.calibrate_action = ActionClient(node, Calibrate, "/calibration/calibrate")
        self.set_calibration_client = node.create_client(SetCalibration, "/calibration/set")
        self.switch_controller_client = node.create_client(SwitchController, "/controller_manager/switch_controller")
        self.set_hw_state_client = node.create_client(
            SetHardwareComponentState, "/controller_manager/set_hardware_component_state")

        config = zenoh.Config()
        config.insert_json5("mode", '"client"')
        config.insert_json5("connect/endpoints", f'["{zenoh_endpoint}"]')
        self.zenoh = open_zenoh(config, zenoh_endpoint, node.get_logger())

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

    def set_calibration(self, origin_xyz, yaw_rad: float, square_size_m: float, park_joints, camera=None):
        """Store a measured calibration. The calibration node owns the profile, so it writes it."""
        if not self.set_calibration_client.wait_for_service(timeout_sec=5.0):
            raise CapabilityError("the calibration node is not available")
        request = SetCalibration.Request()
        request.board_origin_xyz = [float(v) for v in origin_xyz]
        request.board_yaw_rad = float(yaw_rad)
        request.square_size_m = float(square_size_m)
        request.park_joints = [float(v) for v in park_joints]
        if camera is not None:
            request.camera_position_xyz = [float(v) for v in camera.position]
            request.camera_rpy = [float(v) for v in camera.rpy]
            request.camera_focal_px = float(camera.focal_px)
        response = _wait(self.set_calibration_client.call_async(request), 10.0, "storing the calibration")
        if not response.ok:
            raise CapabilityError(f"calibration was not stored: {response.message}")
        return response

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

    def camera_raw(self, topic: str = "/overhead_camera/image_rect", timeout_s: float = 3.0) -> Image | None:
        """One camera frame as it arrived, for measurements that work in pixels."""
        arrived = threading.Event()
        frames: list[Image] = []

        def on_image(msg: Image):
            if not frames:
                frames.append(msg)
                arrived.set()

        subscription = self.node.create_subscription(Image, topic, on_image, qos_profile_sensor_data)
        try:
            arrived.wait(timeout_s)
        finally:
            self.node.destroy_subscription(subscription)
        return frames[0] if frames else None

    def touch_point(self) -> tuple[float, float, float]:
        """Where the closed jaw tips are, in the base frame.

        `tool_pose` reports the grasp point between the jaws. The tips are what a person can
        put on a corner, so that is what a touch records.
        """
        try:
            t = self.tf_buffer.lookup_transform(self.arm.base_frame, self.arm.touch_frame, Time()).transform
        except TransformException as exc:
            raise CapabilityError(f"no {self.arm.touch_frame}: {exc}") from exc
        return (float(t.translation.x), float(t.translation.y), float(t.translation.z))

    def camera_centre(self, topic: str = "/overhead_camera/camera_info", timeout_s: float = 3.0):
        """(cx, cy, fx) from camera_info, or None. Used as the starting point for a fit."""
        arrived = threading.Event()
        infos: list[CameraInfo] = []

        def on_info(msg: CameraInfo):
            if not infos:
                infos.append(msg)
                arrived.set()

        subscription = self.node.create_subscription(CameraInfo, topic, on_info, qos_profile_sensor_data)
        try:
            arrived.wait(timeout_s)
        finally:
            self.node.destroy_subscription(subscription)
        if not infos:
            return None
        info = infos[0]
        # P describes the rectified image these measurements are taken from; K the raw one.
        if float(info.p[0]) > 0.0:
            return (float(info.p[2]), float(info.p[6]), float(info.p[0]))
        return (float(info.k[2]), float(info.k[5]), float(info.k[0]))

    def camera_frame(self, topic: str = "/overhead_camera/image_rect", scale: int = 2, timeout_s: float = 3.0) -> CompressedImage | None:
        """One camera frame as PNG (downscaled), for questions to the reasoning model.

        Subscribed on demand rather than through `wait_for_message`, whose own wait set
        competes with the node's executor for the message and loses.
        """
        arrived = threading.Event()
        frames: list[Image] = []

        def on_image(msg: Image):
            if not frames:
                frames.append(msg)
                arrived.set()

        subscription = self.node.create_subscription(Image, topic, on_image, qos_profile_sensor_data)
        try:
            arrived.wait(timeout_s)
        finally:
            self.node.destroy_subscription(subscription)
        if not frames:
            return None
        msg = frames[0]
        if msg.encoding not in ("rgb8", "bgr8"):
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

    def plan_cartesian(
        self, waypoints_xyz, quat=DOWN, frame_id: str = "base_link", speed_scale: float = 1.0
    ) -> JointTrajectory:
        if not self.cartesian.wait_for_service(timeout_sec=2.0):
            raise CapabilityError("Cartesian planning is not available")
        request = GetCartesianPath.Request()
        request.header.frame_id = frame_id
        request.group_name = "arm"
        request.waypoints = [self._pose(xyz, quat) for xyz in waypoints_xyz]
        request.max_step = 0.005
        request.max_velocity_scaling_factor = float(speed_scale)
        response = _wait(self.cartesian.call_async(request), 10.0, "Cartesian planning")
        if response.fraction < 0.999:
            raise CapabilityError(f"Cartesian path only {response.fraction:.0%} feasible")
        return response.solution.joint_trajectory

    def joint_move_trajectory(self, target: list[float]) -> JointTrajectory:
        """A two-point trajectory to a joint target, timed at the configured speed.

        Both points state a velocity of zero, so the controller interpolates a cubic and
        the arm eases in and out instead of jumping to speed. That costs half again as
        long, since such a profile peaks at 1.5 times its average speed.
        """
        current = self.joint_positions()
        travel = max(abs(a - b) for a, b in zip(current, target))
        duration = 1.5 * max(1.0, travel / self.arm.joint_speed)
        zero = [0.0] * len(self.arm.joints)
        trajectory = JointTrajectory()
        trajectory.joint_names = list(self.arm.joints)
        start = JointTrajectoryPoint(positions=current, velocities=list(zero), time_from_start=Duration(seconds=0.0).to_msg())
        end = JointTrajectoryPoint(positions=list(target), velocities=list(zero), time_from_start=Duration(seconds=duration).to_msg())
        trajectory.points = [start, end]
        return trajectory

    def joint_path_trajectory(self, waypoints: list[list[float]], speed_fraction: float = 1.0) -> JointTrajectory:
        """One trajectory through several joint waypoints, without stopping at each.

        Intermediate waypoints carry the centred difference of the segments either side, so
        the controller splines through them. The ends are at rest.
        """
        path = [self.joint_positions(), *[list(w) for w in waypoints]]
        speed = max(0.05, min(1.0, speed_fraction)) * self.arm.joint_speed
        durations = [max(0.05, max(abs(a - b) for a, b in zip(first, second)) / speed)
                     for first, second in zip(path, path[1:])]

        trajectory = JointTrajectory()
        trajectory.joint_names = list(self.arm.joints)
        elapsed = 0.0
        for index, position in enumerate(path):
            if index == 0 or index == len(path) - 1:
                velocities = [0.0] * len(position)
            else:
                before, after = durations[index - 1], durations[index]
                velocities = [(nxt - prv) / (before + after)
                              for prv, nxt in zip(path[index - 1], path[index + 1])]
            if index > 0:
                elapsed += durations[index - 1]
            trajectory.points.append(JointTrajectoryPoint(
                positions=list(position), velocities=velocities,
                time_from_start=Duration(seconds=elapsed).to_msg()))
        return trajectory

    def stream(self, trajectory: JointTrajectory) -> None:
        """Publish a trajectory and return, for commands meant to be replaced mid-motion.

        Each publication supersedes the last, splined from the velocity the arm has then,
        which is what makes a held jog continuous. `execute` waits instead.
        """
        self.arm_stream.publish(trajectory)

    def jog_trajectory(self, target: list[float], speed_fraction: float = 1.0) -> JointTrajectory:
        """A single-waypoint trajectory, for jogging and hand-guide-assist motion.

        `joint_move_trajectory` manufactures a start point at the current position with zero
        velocity, so chaining calls resets velocity to zero every time - fine for one
        deliberate move, but it makes repeated small steps (a held jog button, touch-off
        nudging) feel stepped rather than continuous. Publishing only the target lets the
        controller spline from whatever it is actually doing right now - including mid-motion,
        with whatever velocity that carries - instead of forcing a stop in between.
        """
        current = self.joint_positions()
        travel = max(abs(a - b) for a, b in zip(current, target))
        speed = max(0.05, min(1.0, speed_fraction)) * self.arm.joint_speed
        duration = max(0.05, travel / speed)
        trajectory = JointTrajectory()
        trajectory.joint_names = list(self.arm.joints)
        trajectory.points = [
            JointTrajectoryPoint(positions=list(target), time_from_start=Duration(seconds=duration).to_msg())
        ]
        return trajectory

    # --- control -----------------------------------------------------------------------

    def release_arm(self, timeout_s: float = 10.0) -> None:
        """Stop the arm controller and drop its hardware component to inactive.

        The hardware component's on_deactivate is what cuts torque. State interfaces stay
        live while it is inactive, so tool_pose keeps tracking a hand-guided arm. The gripper
        shares the component, so its controller goes down here and comes back in
        `reactivate_arm`.
        """
        self._switch_controller(
            deactivate=[self.arm.controller, self.arm.gripper_controller], timeout_s=timeout_s)
        self._set_hardware_state(LifecycleState.PRIMARY_STATE_INACTIVE, timeout_s)

    def reactivate_arm(self, timeout_s: float = 10.0) -> None:
        """Undo `release_arm`: torque back on, then hand control back to the controller.

        Order matters - the hardware component has to be active (and its command seeded to
        the arm's current position, which it does on its own) before the controller is handed
        anything to hold, or there is nothing yet for it to hold onto.
        """
        self._set_hardware_state(LifecycleState.PRIMARY_STATE_ACTIVE, timeout_s)
        self._switch_controller(
            activate=[self.arm.controller, self.arm.gripper_controller], timeout_s=timeout_s)

    def _switch_controller(self, activate: list[str] = (), deactivate: list[str] = (), timeout_s: float = 10.0):
        if not self.switch_controller_client.wait_for_service(timeout_sec=timeout_s):
            raise CapabilityError("controller_manager is not available")
        request = SwitchController.Request(
            activate_controllers=list(activate),
            deactivate_controllers=list(deactivate),
            strictness=SwitchController.Request.STRICT,
            activate_asap=True,
            timeout=Duration(seconds=timeout_s).to_msg(),
        )
        response = _wait(self.switch_controller_client.call_async(request), timeout_s, "switch_controller")
        if not response.ok:
            raise CapabilityError(f"switch_controller failed (activate={list(activate)}, deactivate={list(deactivate)})")

    def _set_hardware_state(self, target_state_id: int, timeout_s: float = 10.0):
        if not self.set_hw_state_client.wait_for_service(timeout_sec=timeout_s):
            raise CapabilityError("controller_manager is not available")
        request = SetHardwareComponentState.Request(
            name=self.arm.hardware_component, target_state=LifecycleState(id=target_state_id))
        response = _wait(self.set_hw_state_client.call_async(request), timeout_s, "set_hardware_component_state")
        if not response.ok:
            raise CapabilityError(f"could not set {self.arm.hardware_component} to lifecycle state {target_state_id}")

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
