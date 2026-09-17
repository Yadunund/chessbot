# Copyright 2026 Yadunund Vijay
# SPDX-License-Identifier: Apache-2.0
"""Save one frame from a sensor_msgs/Image topic as PNG (rgb8/bgr8/mono8), without OpenCV.

    pixi run python tools/dev/save_image.py /overhead_camera/image_raw overhead.png
"""

import struct
import sys
import threading
import zlib

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def write_png(path, rgb: np.ndarray):
    h, w, _ = rgb.shape
    raw = b"".join(b"\x00" + rgb[y].tobytes() for y in range(h))

    def chunk(kind, data):
        return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data) & 0xFFFFFFFF)

    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
                + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))


def main():
    topic, path = sys.argv[1], sys.argv[2]
    rclpy.init()
    node = rclpy.create_node("save_image")
    got = threading.Event()
    frame = {}

    def on_image(msg: Image):
        if got.is_set():
            return
        data = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.step)
        channels = {"rgb8": 3, "bgr8": 3, "mono8": 1}[msg.encoding]
        img = data[:, : msg.width * channels].reshape(msg.height, msg.width, channels)
        if msg.encoding == "bgr8":
            img = img[:, :, ::-1]
        if channels == 1:
            img = np.repeat(img, 3, axis=2)
        frame["img"] = np.ascontiguousarray(img)
        got.set()

    node.create_subscription(Image, topic, on_image, qos_profile_sensor_data)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    if not got.wait(20.0):
        raise SystemExit(f"no image on {topic}")
    write_png(path, frame["img"])
    print(f"saved {path} {frame['img'].shape}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
