# Copyright 2026 Yadunund Vijay
# SPDX-License-Identifier: Apache-2.0
"""Measure where the overhead camera is, using the arm's own gripper as the target.

Nothing about a real setup is known well enough to assume: the camera is bolted wherever it
fits, and the board sits wherever it was put. But the arm knows where its own gripper is, so
it can be the calibration target:

  for each sample pose
      move the tool there (slowly, tool pointing down)
      photograph it with the jaws closed, then open, and difference the two frames
      the only thing that moved is the moving jaw, so the blob is the gripper

That gives pixel/position pairs, and a pinhole model fits the camera pose to them. The jaw's
offset from the tool frame and the focal length are solved for as well, since neither the
nominal intrinsics nor the jaw geometry can be trusted on a real rig.

    pixi run python tools/dev/camera_extrinsics.py                      # measure and report
    pixi run python tools/dev/camera_extrinsics.py --dry-run            # poses only, no motion

Prints the result as the launch arguments to pass (overhead_xyz, overhead_rpy), and saves an
annotated frame showing where each sample landed.

Safety: every target is clamped into a box well above the table, each move is slow, and a
pose whose IK fails, or whose gripper cannot be seen, is skipped rather than forced.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import threading
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, ParallelGripperCommand
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from scipy.optimize import least_squares
from sensor_msgs.msg import CameraInfo, Image, JointState
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from save_image import write_png  # noqa: E402

JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_flex_joint", "wrist_flex_joint", "wrist_roll_joint"]
# Well clear of the table and of anything standing on it.
SAFE_BOX = ((0.08, 0.30), (-0.18, 0.18), (0.04, 0.30))
# Tool pointing straight down, jaws opening along base x (quaternion x, y, z, w).
DOWN = (1.0, 0.0, 0.0, 0.0)
# Tool-down poses the arm can actually hold: it is short, so anything higher than about
# 10 cm over the table is out of reach at this radius (--probe reports what is).
SAMPLE_XY = [(0.16, -0.10), (0.16, 0.10), (0.20, -0.06), (0.20, 0.06), (0.23, -0.07), (0.23, 0.07)]
SAMPLE_Z = (0.08, 0.11)
SAMPLES = [(x, y, z) for z in SAMPLE_Z for x, y in SAMPLE_XY]
# A Logitech C920 at 1280x720: 70.4 degrees across, so about this many pixels of focal
# length. Fixed by default, because samples the arm can reach are nearly coplanar and a free
# focal length simply trades against the camera's height.
C920_FOCAL_PX = 907.0


def clamp(xyz):
    return tuple(min(max(v, lo), hi) for v, (lo, hi) in zip(xyz, SAFE_BOX))


class Rig:
    def __init__(self, node, speed: float):
        self.node = node
        self.speed = speed
        self.ik = node.create_client(GetPositionIK, "/compute_ik")
        self.arm = ActionClient(node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
        self.gripper = ActionClient(node, ParallelGripperCommand, "/gripper_controller/gripper_cmd")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)
        self._joints: dict[str, float] = {}
        self._image: Image | None = None
        self.camera_info: CameraInfo | None = None
        node.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        node.create_subscription(Image, "/overhead_camera/image_raw", self._on_image, qos_profile_sensor_data)
        node.create_subscription(CameraInfo, "/overhead_camera/camera_info", self._on_info, qos_profile_sensor_data)

    def _on_joints(self, msg: JointState):
        self._joints.update(dict(zip(msg.name, msg.position)))

    def _on_image(self, msg: Image):
        self._image = msg

    def _on_info(self, msg: CameraInfo):
        self.camera_info = msg

    def wait_ready(self, timeout_s: float = 20.0):
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if all(j in self._joints for j in JOINTS) and self._image is not None:
                return
            time.sleep(0.2)
        raise SystemExit("no joint states or no camera frames")

    def positions(self) -> list[float]:
        return [self._joints[j] for j in JOINTS]

    def frame(self) -> np.ndarray:
        """The latest frame as an RGB array, after letting a fresh one arrive."""
        self._image = None
        deadline = time.time() + 5.0
        while self._image is None and time.time() < deadline:
            time.sleep(0.05)
        if self._image is None:
            raise SystemExit("camera stopped publishing")
        msg = self._image
        data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        image = data.reshape(msg.height, msg.step // 3, 3)[:, : msg.width, :]
        return image[:, :, ::-1].copy() if msg.encoding == "bgr8" else image.copy()

    def tool_position(self) -> np.ndarray | None:
        try:
            tf = self.tf_buffer.lookup_transform("base_link", "gripper_frame_link", rclpy.time.Time())
        except Exception:  # noqa: BLE001 - no transform yet is a skip, not a failure
            return None
        t = tf.transform.translation
        return np.array([t.x, t.y, t.z])

    def solve_ik(self, xyz) -> list[float] | None:
        if not self.ik.wait_for_service(timeout_sec=5.0):
            raise SystemExit("/compute_ik is not available")
        request = GetPositionIK.Request()
        request.ik_request.group_name = "arm"
        request.ik_request.pose_stamped = PoseStamped()
        request.ik_request.pose_stamped.header.frame_id = "base_link"
        pose = request.ik_request.pose_stamped.pose
        pose.position.x, pose.position.y, pose.position.z = xyz
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = DOWN
        future = self.ik.call_async(request)
        while not future.done():
            time.sleep(0.02)
        response = future.result()
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            return None
        by_name = dict(zip(response.solution.joint_state.name, response.solution.joint_state.position))
        return [by_name[j] for j in JOINTS]

    def move_to(self, target: list[float]) -> bool:
        """Slow joint move, easing in and out (both ends state zero velocity)."""
        current = self.positions()
        travel = max(abs(a - b) for a, b in zip(current, target))
        duration = 1.5 * max(1.0, travel / self.speed)
        if not self.arm.wait_for_server(timeout_sec=10.0):
            raise SystemExit("arm controller is not available")
        trajectory = JointTrajectory(joint_names=JOINTS)
        zero = [0.0] * len(JOINTS)
        trajectory.points = [
            JointTrajectoryPoint(positions=current, velocities=list(zero), time_from_start=Duration(sec=0)),
            JointTrajectoryPoint(positions=list(target), velocities=list(zero),
                                 time_from_start=Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))),
        ]
        return self._run(self.arm, FollowJointTrajectory.Goal(trajectory=trajectory), duration + 10.0)

    def set_jaws(self, position: float) -> bool:
        if not self.gripper.wait_for_server(timeout_sec=10.0):
            raise SystemExit("gripper controller is not available")
        goal = ParallelGripperCommand.Goal()
        goal.command.name = ["gripper_joint"]
        goal.command.position = [position]
        return self._run(self.gripper, goal, 15.0)

    def _run(self, client, goal, timeout_s: float) -> bool:
        handle = client.send_goal_async(goal)
        deadline = time.time() + timeout_s
        while not handle.done() and time.time() < deadline:
            time.sleep(0.02)
        if not handle.done():
            return False
        result = handle.result().get_result_async()
        while not result.done() and time.time() < deadline:
            time.sleep(0.02)
        return result.done()


def jaw_pixel(closed: np.ndarray, opened: np.ndarray, threshold: int = 35, min_pixels: int = 40):
    """Centre of what moved between the two frames, or None if nothing did."""
    difference = np.abs(opened.astype(np.int16) - closed.astype(np.int16)).sum(axis=2)
    mask = difference > threshold * 3
    if int(mask.sum()) < min_pixels:
        return None, int(mask.sum())
    ys, xs = np.nonzero(mask)
    # Weight by how much each pixel changed, so faint noise does not drag the centre.
    weights = difference[ys, xs].astype(float)
    return (float((xs * weights).sum() / weights.sum()), float((ys * weights).sum() / weights.sum())), int(mask.sum())


def rotation(rpy) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def project(params, points, centre):
    """Pixels for `points` (base_link) given camera pose, jaw offset and focal length."""
    position, rpy, offset, focal = params[0:3], params[3:6], params[6:9], params[9]
    # The optical frame looks along +z with +x right and +y down; world_to_optical is its
    # inverse rotation applied to the point relative to the camera.
    world_to_optical = rotation(rpy).T
    local = (points + offset - position) @ world_to_optical.T
    depth = np.clip(local[:, 2], 1e-3, None)
    return np.stack([centre[0] + focal * local[:, 0] / depth, centre[1] + focal * local[:, 1] / depth], axis=1)


def fit(points: np.ndarray, pixels: np.ndarray, centre, focal_guess: float, fixed_focal: bool):
    def residual(params):
        if fixed_focal:
            params = np.append(params, focal_guess)
        return (project(params, points, centre) - pixels).ravel()

    best, best_cost = None, np.inf
    # The camera looks down, so pitch is near 180 degrees about x; try a few yaws, since the
    # rig can be mounted facing any way and the fit is otherwise happy in a local minimum.
    for yaw in np.linspace(-math.pi, math.pi, 8, endpoint=False):
        guess = np.array([0.2, 0.0, 0.5, math.pi, 0.0, yaw, 0.0, 0.0, -0.05, focal_guess])
        result = least_squares(residual, guess[:9] if fixed_focal else guess, method="lm", max_nfev=20000)
        if result.cost < best_cost:
            best, best_cost = result, result.cost
    return best


def annotate(image: np.ndarray, pixels, fitted):
    out = image.copy()
    for (x, y), (fx, fy) in zip(pixels, fitted):
        for colour, (px, py) in ((np.array([255, 60, 60]), (x, y)), (np.array([60, 255, 60]), (fx, fy))):
            cx, cy = int(round(px)), int(round(py))
            for dx in range(-6, 7):
                for dy in (-1, 0, 1):
                    if 0 <= cy + dy < out.shape[0] and 0 <= cx + dx < out.shape[1]:
                        out[cy + dy, cx + dx] = colour
                    if 0 <= cy + dx < out.shape[0] and 0 <= cx + dy < out.shape[1]:
                        out[cy + dx, cx + dy] = colour
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--speed", type=float, default=0.3, help="rad/s for the sample moves")
    parser.add_argument("--open", type=float, default=0.24, help="jaw position that counts as open")
    parser.add_argument("--closed", type=float, default=0.02)
    parser.add_argument("--out", default="/tmp/extrinsics.png")
    parser.add_argument("--dry-run", action="store_true", help="print the sample poses and stop")
    parser.add_argument("--rest", type=float, nargs=3, metavar=("X", "Y", "Z"),
                        help="move the tool to this point and exit, e.g. somewhere low and clear "
                             "before shutting the stack down")
    parser.add_argument("--probe", action="store_true",
                        help="ask IK which sample poses are reachable, without moving the arm")
    parser.add_argument("--height", type=float, nargs="*", help="override the sample heights, metres")
    parser.add_argument("--focal", type=float, default=C920_FOCAL_PX,
                        help="focal length in pixels; 0 solves for it too (needs samples at several heights)")
    args = parser.parse_args()

    if args.dry_run:
        for xyz in SAMPLES:
            print(f"sample {clamp(xyz)}")
        return 0

    samples = SAMPLES
    if args.height:
        samples = [(x, y, z) for z in args.height for x, y in SAMPLE_XY]

    rclpy.init()
    node = rclpy.create_node("camera_extrinsics")
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    rig = Rig(node, args.speed)
    rig.wait_ready()

    if args.rest:
        target = clamp(tuple(args.rest))
        solution = rig.solve_ik(target)
        if solution is None:
            print(f"{target}: no IK")
            return 1
        print(f"moving to {target}: {'reached' if rig.move_to(solution) else 'failed'}")
        return 0

    if args.probe:
        for xyz in samples:
            target = clamp(xyz)
            print(f"{target}: {'reachable' if rig.solve_ik(target) else 'no IK'}")
        return 0

    points, pixels, frame = [], [], None
    rig.set_jaws(args.closed)
    for xyz in samples:
        target = clamp(xyz)
        solution = rig.solve_ik(target)
        if solution is None:
            print(f"{target}: no IK, skipped")
            continue
        if not rig.move_to(solution):
            print(f"{target}: move failed, skipped")
            continue
        time.sleep(1.0)
        position = rig.tool_position()
        closed_frame = rig.frame()
        rig.set_jaws(args.open)
        time.sleep(0.8)
        open_frame = rig.frame()
        rig.set_jaws(args.closed)
        pixel, changed = jaw_pixel(closed_frame, open_frame)
        if pixel is None or position is None:
            print(f"{target}: gripper not visible ({changed} pixels changed), skipped")
            continue
        print(f"{target}: tool at {np.round(position, 4).tolist()} seen at ({pixel[0]:.1f}, {pixel[1]:.1f}), "
              f"{changed} pixels changed")
        points.append(position)
        pixels.append(pixel)
        frame = open_frame

    if len(points) < 5:
        print(f"\nonly {len(points)} usable samples; need at least 5 to fit a camera pose")
        return 1

    points, pixels = np.array(points), np.array(pixels)
    info = rig.camera_info
    centre = (info.k[2], info.k[5]) if info and info.k[2] else (frame.shape[1] / 2, frame.shape[0] / 2)
    fixed_focal = args.focal > 0.0
    focal_guess = args.focal if fixed_focal else (info.k[0] if info and info.k[0] else C920_FOCAL_PX)
    result = fit(points, pixels, centre, focal_guess, fixed_focal)
    solution = np.append(result.x, focal_guess) if fixed_focal else result.x
    fitted = project(solution, points, centre)
    error = np.linalg.norm(fitted - pixels, axis=1)

    position, rpy, offset, focal = solution[0:3], solution[3:6], solution[6:9], solution[9]
    print(f"\nfit over {len(points)} samples: {error.mean():.1f} px mean, {error.max():.1f} px worst")
    print(f"focal length {focal:.0f} px ({'fixed' if fixed_focal else 'solved'}), "
          f"jaw offset {np.round(offset, 3).tolist()} m")
    print("\npass these to the launch:")
    print(f"  overhead_xyz:=\"{position[0]:.4f} {position[1]:.4f} {position[2]:.4f}\"")
    print(f"  overhead_rpy:=\"{rpy[0]:.4f} {rpy[1]:.4f} {rpy[2]:.4f}\"")
    write_png(args.out, annotate(frame, pixels, fitted))
    print(f"\nsaved {args.out}: measured in red, fitted in green")
    return 0


if __name__ == "__main__":
    sys.exit(main())
