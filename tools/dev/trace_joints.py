# Copyright 2026 Yadunund Vijay
# SPDX-License-Identifier: Apache-2.0
"""Record how the arm actually follows a trajectory, to judge smoothness.

Listens to the trajectory controller's own state (reference, feedback and error per
joint) and to /joint_states, optionally while commanding a move, then reports lag,
overshoot and how jerky the motion is.

    pixi run python tools/dev/trace_joints.py --move -0.6 1.0 -1.0 0.2 0.0 --seconds 2
    pixi run python tools/dev/trace_joints.py --seconds 20          # just watch

Smoothness is reported as the peak jerk of the measured velocity: a trajectory whose
waypoints carry no velocities is interpolated linearly, which shows up here as velocity
steps between segments.
"""

from __future__ import annotations

import argparse
import threading
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_flex_joint", "wrist_flex_joint", "wrist_roll_joint"]


class Trace:
    """Samples of (time, reference, feedback, velocity) for one joint set."""

    def __init__(self):
        self.lock = threading.Lock()
        self.samples: list[tuple[float, list[float], list[float], list[float]]] = []

    def add(self, t: float, reference: list[float], feedback: list[float], velocity: list[float]):
        with self.lock:
            self.samples.append((t, reference, feedback, velocity))

    def window(self, t0: float, t1: float):
        with self.lock:
            return [s for s in self.samples if t0 <= s[0] <= t1]


def report(samples, joints: list[str]):
    if len(samples) < 3:
        print("not enough samples")
        return
    t = [s[0] for s in samples]
    print(f"{len(samples)} samples over {t[-1] - t[0]:.2f} s")
    print(f"{'joint':<22}{'lag mm/rad':>12}{'overshoot':>11}{'peak vel':>10}{'peak acc':>10}{'vel steps':>10}")
    worst: list[tuple[float, int, str]] = []
    for j, name in enumerate(joints):
        reference = [s[1][j] for s in samples]
        feedback = [s[2][j] for s in samples]
        velocity = [s[3][j] for s in samples]
        target = reference[-1]
        lag = max(abs(r - f) for r, f in zip(reference, feedback))
        start = feedback[0]
        # Overshoot past the commanded end, in the direction of travel.
        direction = 1.0 if target >= start else -1.0
        overshoot = max(0.0, max(direction * (f - target) for f in feedback))
        accel = [(velocity[i + 1] - velocity[i]) / max(1e-3, t[i + 1] - t[i]) for i in range(len(velocity) - 1)]
        # Steps in commanded velocity: a linearly interpolated trajectory changes velocity
        # abruptly at each waypoint, which the joint cannot follow smoothly.
        steps = sum(1 for a in accel if abs(a) > 5.0)
        print(f"{name:<22}{lag:>12.4f}{overshoot:>11.4f}{max(abs(v) for v in velocity):>10.3f}"
              f"{max(abs(a) for a in accel):>10.2f}{steps:>10d}")
        worst.append((max(abs(a) for a in accel), j, name))
    if not worst:
        return
    # The joint that jerks hardest, in context: what was commanded and what happened.
    _, j, name = max(worst)
    peak = max(range(1, len(samples)), key=lambda i: abs(samples[i][3][j] - samples[i - 1][3][j]))
    print(f"\naround the worst jerk ({name}, {samples[peak][0] - t[0]:.2f} s into the trace):")
    print(f"{'t':>8}{'reference':>12}{'feedback':>12}{'velocity':>10}")
    for i in range(max(0, peak - 6), min(len(samples), peak + 7)):
        s = samples[i]
        print(f"{s[0] - t[0]:>8.3f}{s[1][j]:>12.4f}{s[2][j]:>12.4f}{s[3][j]:>10.3f}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--move", type=float, nargs=5, help="command this joint configuration and trace it")
    parser.add_argument("--seconds", type=float, default=3.0, help="trajectory duration, or watch time without --move")
    parser.add_argument("--settle", type=float, default=2.0, help="seconds to keep recording after the move")
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("trace_joints")
    trace, joint_trace = Trace(), Trace()

    def on_state(msg: JointTrajectoryControllerState):
        trace.add(time.time(), list(msg.reference.positions), list(msg.feedback.positions),
                  list(msg.feedback.velocities) or [0.0] * len(msg.joint_names))

    def on_joints(msg: JointState):
        by_name = dict(zip(msg.name, msg.position))
        speeds = dict(zip(msg.name, msg.velocity))
        if all(j in by_name for j in JOINTS):
            positions = [by_name[j] for j in JOINTS]
            joint_trace.add(time.time(), positions, positions, [speeds.get(j, 0.0) for j in JOINTS])

    node.create_subscription(JointTrajectoryControllerState, "/arm_controller/controller_state", on_state, 10)
    node.create_subscription(JointState, "/joint_states", on_joints, qos_profile_sensor_data)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    if args.move:
        client = ActionClient(node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
        if not client.wait_for_server(timeout_sec=10.0):
            raise SystemExit("arm controller not available")
        trajectory = JointTrajectory(joint_names=JOINTS)
        trajectory.points = [JointTrajectoryPoint(
            positions=list(args.move),
            time_from_start=Duration(sec=int(args.seconds), nanosec=int((args.seconds % 1) * 1e9)),
        )]
        goal = FollowJointTrajectory.Goal(trajectory=trajectory)
        t0 = time.time()
        handle = client.send_goal_async(goal)
        while not handle.done():
            time.sleep(0.02)
        result = handle.result().get_result_async()
        while not result.done():
            time.sleep(0.02)
        arrived = time.time()
        code = result.result().result.error_code
        print(f"goal finished in {arrived - t0:.2f} s (error_code {code})")
        time.sleep(args.settle)
        t1 = time.time()
    else:
        t0 = time.time()
        time.sleep(args.seconds)
        t1 = arrived = time.time()

    print("\ncontroller state (reference vs feedback):")
    report(trace.window(t0, t1), JOINTS)
    print("\n/joint_states velocities:")
    report(joint_trace.window(t0, t1), JOINTS)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
