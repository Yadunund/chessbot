# Copyright 2026 Yadunund Vijay
# SPDX-License-Identifier: Apache-2.0
"""Draw where each square's sample patch lands on a camera frame, to check the projection.

Uses the same inputs as perception: calibration from the key-value store, camera_info,
and the camera pose from TF.

    pixi run python tools/dev/project_squares.py /overhead_camera/image_raw out.png
"""

import json
import math
import sys
import threading
import time
import urllib.request

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from save_image import write_png  # noqa: E402


def main():
    topic, path = sys.argv[1], sys.argv[2]
    info_topic = topic.rsplit("/", 1)[0] + "/camera_info"
    board = json.loads(urllib.request.urlopen("http://127.0.0.1:8080/chessbot/kv/calibration").read())[0]["value"]
    board = (json.loads(board) if isinstance(board, str) else board)["board"]
    rclpy.init()
    node = rclpy.create_node("project_squares")
    buffer = Buffer()
    TransformListener(buffer, node)
    got = {}
    node.create_subscription(Image, topic, lambda m: got.setdefault("image", m), qos_profile_sensor_data)
    node.create_subscription(CameraInfo, info_topic, lambda m: got.setdefault("info", m), qos_profile_sensor_data)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    deadline = time.time() + 20
    while time.time() < deadline and not ("image" in got and "info" in got):
        time.sleep(0.2)
    image, info = got["image"], got["info"]
    time.sleep(1.0)
    t = buffer.lookup_transform(image.header.frame_id, "base_link", rclpy.time.Time()).transform
    q = t.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    T = np.array([t.translation.x, t.translation.y, t.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    print(f"frame {image.header.frame_id}, K fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}, {image.width}x{image.height}")
    img = np.frombuffer(bytes(image.data), np.uint8).reshape(image.height, image.step)[:, : image.width * 3]
    img = img.reshape(image.height, image.width, 3).copy()
    s, (ox, oy, oz), yaw = board["square_size_m"], board["origin_xyz"], board["yaw_rad"]
    for index in range(64):
        bx, by = (index % 8 + 0.5) * s, (index // 8 + 0.5) * s
        p = np.array([ox + math.cos(yaw) * bx - math.sin(yaw) * by, oy + math.sin(yaw) * bx + math.cos(yaw) * by, oz + 0.003])
        c = R @ p + T
        u, v = int(round(fx * c[0] / c[2] + cx)), int(round(fy * c[1] / c[2] + cy))
        colour = (255, 0, 0) if index in (0, 7, 56) else (0, 200, 255)  # a1, h1, a8 in red
        if index in (0, 7, 56, 3, 4):
            print(f"{'abcdefgh'[index % 8]}{index // 8 + 1}: world ({p[0]:.3f}, {p[1]:.3f}) -> pixel ({u}, {v})")
        for d in range(-6, 7):
            for uu, vv in ((u + d, v), (u, v + d)):
                if 0 <= uu < image.width and 0 <= vv < image.height:
                    img[vv, uu] = colour
    write_png(path, img)
    print(f"saved {path}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
