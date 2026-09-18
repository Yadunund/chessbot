# Copyright 2026 Yadunund Vijay
# SPDX-License-Identifier: Apache-2.0
"""Average the colour of rectangles in a camera frame, to match simulation to the real thing.

The simulated board and pieces only need to be as realistic as perception is sensitive to, and
perception compares colours. So take the colours from the real camera rather than inventing
them: sample a patch of each surface and read off what the camera actually sees.

    pixi run python tools/dev/sample_colours.py light_square:600,380,30 dark_piece:400,540,20

Each argument is name:x,y,half-width in pixels. Prints mean RGB (0-255 and 0-1) per patch, so
the 0-1 triples can go straight into an SDF material.
"""

from __future__ import annotations

import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    topic = "/overhead_camera/image_raw"
    patches = []
    for argument in sys.argv[1:]:
        if argument.startswith("--topic="):
            topic = argument.split("=", 1)[1]
            continue
        name, _, spec = argument.partition(":")
        x, y, half = (int(v) for v in spec.split(","))
        patches.append((name, x, y, half))

    rclpy.init()
    node = rclpy.create_node("sample_colours")
    latest: list[Image] = []
    node.create_subscription(Image, topic, lambda msg: latest.append(msg), qos_profile_sensor_data)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    deadline = time.time() + 10.0
    while not latest and time.time() < deadline:
        time.sleep(0.05)
    if not latest:
        print(f"no frames on {topic}")
        return 1

    msg = latest[-1]
    data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    image = data.reshape(msg.height, msg.step // 3, 3)[:, : msg.width, :]
    if msg.encoding == "bgr8":
        image = image[:, :, ::-1]

    print(f"{'patch':<16}{'R':>5}{'G':>5}{'B':>5}   as SDF colour")
    for name, x, y, half in patches:
        region = image[max(0, y - half): y + half + 1, max(0, x - half): x + half + 1].reshape(-1, 3)
        mean = region.mean(axis=0)
        normalised = " ".join(f"{v / 255.0:.2f}" for v in mean)
        print(f"{name:<16}{mean[0]:>5.0f}{mean[1]:>5.0f}{mean[2]:>5.0f}   {normalised} 1")
    return 0


if __name__ == "__main__":
    sys.exit(main())
