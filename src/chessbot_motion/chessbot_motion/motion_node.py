# Copyright 2026 Yadunund Vijay
# SPDX-License-Identifier: Apache-2.0
"""Motion planning capability.

Implements MoveIt's standard services, so the brain works unchanged against
this planner or against MoveIt itself:

- ``/compute_ik`` (moveit_msgs/srv/GetPositionIK)
- ``/compute_cartesian_path`` (moveit_msgs/srv/GetCartesianPath)

Tool frame convention for target poses: +Z is the approach direction and +X the
direction the jaws open (from the fixed jaw towards the moving jaw). The target
position is the grasp point on the fixed jaw's inner surface. Both axes are
honoured; the approach may tilt up to max_approach_tilt_deg where a vertical
tool cannot reach.
The start state comes from the request if given, otherwise from /joint_states,
and the model from the latched /robot_description topic.
"""

from __future__ import annotations

import threading

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetCartesianPath, GetPositionIK
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from chessbot_motion.kinematics import ArmKinematics

TOOL_X = np.array([1.0, 0.0, 0.0])
TOOL_Z = np.array([0.0, 0.0, 1.0])


def quat_to_matrix(q) -> np.ndarray:
    x, y, z, w = q.x, q.y, q.z, q.w
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array(
        [
            [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
            [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
            [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)],
        ]
    )


def pose_target(pose) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    position = np.array([pose.position.x, pose.position.y, pose.position.z])
    rotation = quat_to_matrix(pose.orientation)
    return position, rotation @ TOOL_Z, rotation @ TOOL_X


class MotionNode(Node):
    def __init__(self):
        super().__init__("motion")
        self.joint_names = self.declare_parameter(
            "joints",
            ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_flex_joint", "wrist_flex_joint", "wrist_roll_joint"],
        ).value
        self.tip_frame = self.declare_parameter("tip_frame", "gripper_frame_link").value
        self.approach_ref_frame = self.declare_parameter("approach_ref_frame", "gripper_link").value
        # Direction the jaws open (fixed towards moving jaw) in tip_frame, measured with
        # tools/dev/gripper_geometry.py.
        self.opening_axis = list(self.declare_parameter("opening_axis", [-1.0, 0.0, 0.0]).value)
        self.max_joint_velocity = float(self.declare_parameter("max_joint_velocity", 0.8).value)
        self.accepted_frames = set(self.declare_parameter("accepted_frames", ["", "world", "base_link"]).value)
        self.max_approach_tilt_deg = float(self.declare_parameter("max_approach_tilt_deg", 25.0).value)

        self._lock = threading.Lock()
        self._kin: ArmKinematics | None = None
        self._joint_positions: dict[str, float] = {}

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String, "/robot_description", self._on_description, latched)
        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)

        # Planning is CPU-bound; keep it off the subscription thread.
        planning = MutuallyExclusiveCallbackGroup()
        self.create_service(GetPositionIK, "/compute_ik", self._on_ik, callback_group=planning)
        self.create_service(GetCartesianPath, "/compute_cartesian_path", self._on_cartesian, callback_group=planning)
        self.get_logger().info("Serving /compute_ik and /compute_cartesian_path")

    # --- inputs -----------------------------------------------------------------

    def _on_description(self, msg: String):
        try:
            kin = ArmKinematics(
                msg.data,
                self.joint_names,
                self.tip_frame,
                self.approach_ref_frame,
                max_approach_tilt_rad=np.radians(self.max_approach_tilt_deg),
                opening_axis=tuple(self.opening_axis),
            )
        except Exception as exc:  # noqa: BLE001 - report any model error and keep serving
            self.get_logger().error(f"Could not build kinematics from robot_description: {exc}")
            return
        with self._lock:
            self._kin = kin
        self.get_logger().info("Kinematics model ready")

    def _on_joint_states(self, msg: JointState):
        with self._lock:
            for name, position in zip(msg.name, msg.position):
                self._joint_positions[name] = position

    def _start_q(self, state: RobotState) -> np.ndarray | None:
        given = dict(zip(state.joint_state.name, state.joint_state.position))
        with self._lock:
            known = {**self._joint_positions, **given}
        if not all(name in known for name in self.joint_names):
            return None
        return np.array([known[name] for name in self.joint_names])

    # --- services ---------------------------------------------------------------

    def _on_ik(self, request: GetPositionIK.Request, response: GetPositionIK.Response):
        ik = request.ik_request
        kin = self._kin
        seed = self._start_q(ik.robot_state)
        if kin is None or seed is None:
            response.error_code.val = MoveItErrorCodes.FAILURE
            return response
        if ik.pose_stamped.header.frame_id not in self.accepted_frames:
            response.error_code.val = MoveItErrorCodes.FRAME_TRANSFORM_FAILURE
            return response

        position, approach, opening = pose_target(ik.pose_stamped.pose)
        result = kin.solve_with_restarts(position, approach, seed, target_opening=opening)
        if not result.success:
            response.error_code.val = MoveItErrorCodes.NO_IK_SOLUTION
            return response
        response.solution.joint_state.name = list(self.joint_names)
        response.solution.joint_state.position = [float(v) for v in result.q]
        response.error_code.val = MoveItErrorCodes.SUCCESS
        return response

    def _on_cartesian(self, request: GetCartesianPath.Request, response: GetCartesianPath.Response):
        kin = self._kin
        start = self._start_q(request.start_state)
        if kin is None or start is None:
            response.error_code.val = MoveItErrorCodes.FAILURE
            return response
        if request.header.frame_id not in self.accepted_frames:
            response.error_code.val = MoveItErrorCodes.FRAME_TRANSFORM_FAILURE
            return response

        waypoints = [pose_target(p) for p in request.waypoints]
        max_step = request.max_step if request.max_step > 0 else 0.005
        configs, fraction = kin.cartesian_path(start, waypoints, max_step=max_step)

        response.start_state.joint_state.name = list(self.joint_names)
        response.start_state.joint_state.position = [float(v) for v in start]
        response.solution.joint_trajectory = self._timed_trajectory(configs)
        response.fraction = float(fraction)
        response.error_code.val = MoveItErrorCodes.SUCCESS if fraction > 0.0 else MoveItErrorCodes.NO_IK_SOLUTION
        return response

    def _timed_trajectory(self, configs: list[np.ndarray]) -> JointTrajectory:
        """Time each step by the largest joint move at the configured velocity."""
        traj = JointTrajectory()
        traj.joint_names = list(self.joint_names)
        t = 0.0
        for i, q in enumerate(configs):
            if i > 0:
                t += max(0.02, float(np.max(np.abs(q - configs[i - 1]))) / self.max_joint_velocity)
            point = JointTrajectoryPoint()
            point.positions = [float(v) for v in q]
            point.time_from_start = Duration(sec=int(t), nanosec=int((t - int(t)) * 1e9))
            traj.points.append(point)
        return traj


def main():
    rclpy.init()
    node = MotionNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
