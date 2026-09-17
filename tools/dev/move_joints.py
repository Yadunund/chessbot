# Copyright 2026 Yadunund Vijay
# SPDX-License-Identifier: Apache-2.0
"""Move the arm to a joint configuration (dev use: trying poses in simulation).

    pixi run python tools/dev/move_joints.py -1.72 -0.49 -1.20 0.65 0.0 [--seconds 4]
"""

import argparse
import threading

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_flex_joint", "wrist_flex_joint", "wrist_roll_joint"]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("positions", type=float, nargs=5)
    parser.add_argument("--seconds", type=float, default=4.0)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node("move_joints")
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    client = ActionClient(node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
    if not client.wait_for_server(timeout_sec=10.0):
        raise SystemExit("arm controller not available")
    point = JointTrajectoryPoint(positions=args.positions, time_from_start=Duration(sec=int(args.seconds)))
    goal = FollowJointTrajectory.Goal(trajectory=JointTrajectory(joint_names=JOINTS, points=[point]))
    handle = client.send_goal_async(goal)
    done = threading.Event()
    handle.add_done_callback(lambda f: f.result().get_result_async().add_done_callback(lambda _: done.set()))
    print("reached" if done.wait(60.0) else "timed out")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
