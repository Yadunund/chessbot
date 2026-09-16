# Copyright 2026 Yadunund Vijay
# SPDX-License-Identifier: Apache-2.0
"""Trace the gripper joint (ROS) and one piece's height (Gazebo) while an action runs.

    pixi run python tools/dev/grasp_trace.py pc_g8_n -- python tools/sim/scenario.py transfer:g8f6
"""

import json
import subprocess
import sys
import threading
import time

import rclpy
from sensor_msgs.msg import JointState


def main():
    piece = sys.argv[1]
    command = sys.argv[sys.argv.index("--") + 1 :]
    rclpy.init()
    node = rclpy.create_node("grasp_trace")
    grip = []
    node.create_subscription(
        JointState, "/joint_states",
        lambda m: grip.append((time.time(), dict(zip(m.name, m.position)).get("gripper_joint"))), 10,
    )
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    heights = []
    proc = subprocess.Popen(["gz", "topic", "-e", "-t", "/world/chessbot/dynamic_pose/info", "--json-output"],
                            stdout=subprocess.PIPE, text=True)

    def read():
        for line in proc.stdout:
            try:
                msg = json.loads(line)
            except json.JSONDecodeError:
                continue
            for p in msg.get("pose", []):
                if p.get("name") == piece:
                    heights.append((time.time(), p.get("position", {}).get("z", 0.0)))

    threading.Thread(target=read, daemon=True).start()
    subprocess.run(command)
    proc.terminate()
    # Print a coarse timeline: every 0.5 s, gripper angle and piece height.
    if not grip:
        print("no joint states")
        return
    t0 = grip[0][0]
    t = t0
    while t < grip[-1][0]:
        g = min(grip, key=lambda s: abs(s[0] - t))[1]
        h = min(heights, key=lambda s: abs(s[0] - t))[1] if heights else float("nan")
        print(f"{t - t0:6.1f}s gripper {g:+.3f} rad  {piece} z {h * 1000:6.1f} mm")
        t += 1.0
    rclpy.shutdown()


if __name__ == "__main__":
    main()
