# Copyright 2026 Yadunund Vijay
# SPDX-License-Identifier: Apache-2.0
"""Ask the reasoning model about a live camera frame, through the same service the brain uses.

Useful for checking the reasoning layer against the real board without playing a game: grab a
frame, send it with a question, print what comes back.

    pixi run python tools/dev/ask_gemma.py "What do you see on the chess board?"
    pixi run python tools/dev/ask_gemma.py --role tiebreak --candidates e2e4,d2d4 \\
        "Which move does the image show?"

--save writes the exact frame that was sent, which is the frame to look at when the answer
disagrees with the board.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time

import numpy as np
import rclpy
from chessbot_interfaces.srv import Reason
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from save_image import write_png  # noqa: E402

ROLES = {
    "commentary": Reason.Request.ROLE_COMMENTARY,
    "tiebreak": Reason.Request.ROLE_MOVE_TIEBREAK,
    "explain": Reason.Request.ROLE_EXPLAIN,
}


def _encode(rgb: np.ndarray) -> bytes:
    import struct
    import zlib

    height, width, _ = rgb.shape
    raw = b"".join(b"\x00" + rgb[y].tobytes() for y in range(height))

    def chunk(kind: bytes, payload: bytes) -> bytes:
        return struct.pack(">I", len(payload)) + kind + payload + struct.pack(">I", zlib.crc32(kind + payload) & 0xFFFFFFFF)

    return (b"\x89PNG\r\n\x1a\n"
            + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("prompt")
    parser.add_argument("--role", choices=sorted(ROLES), default="tiebreak")
    parser.add_argument("--topic", default="/overhead_camera/image_raw")
    parser.add_argument("--scale", type=int, default=2, help="send every Nth pixel")
    parser.add_argument("--candidates", help="comma separated moves; asks for one of them as JSON")
    parser.add_argument("--no-image", action="store_true", help="ask without a picture")
    parser.add_argument("--save", help="write the frame that was sent to this path")
    parser.add_argument("--timeout", type=float, default=120.0)
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("ask_gemma")
    frames: list[Image] = []
    if not args.no_image:
        node.create_subscription(Image, args.topic, lambda msg: frames.append(msg), qos_profile_sensor_data)
    client = node.create_client(Reason, "/inference/reason")
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    if not client.wait_for_service(timeout_sec=10.0):
        print("/inference/reason is not available")
        return 1

    request = Reason.Request()
    request.role = ROLES[args.role]
    request.prompt = args.prompt
    if args.candidates:
        moves = [m.strip() for m in args.candidates.split(",") if m.strip()]
        request.json_schema = json.dumps({
            "type": "object",
            "properties": {"move": {"enum": [*moves, "unclear"]}, "confidence": {"type": "number"}},
            "required": ["move", "confidence"],
        })

    if not args.no_image:
        deadline = time.time() + 10.0
        while not frames and time.time() < deadline:
            time.sleep(0.05)
        if not frames:
            print(f"no frames on {args.topic}")
            return 1
        msg = frames[-1]
        data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        image = data.reshape(msg.height, msg.step // 3, 3)[:, : msg.width, :]
        if msg.encoding == "bgr8":
            image = image[:, :, ::-1]
        png = _encode(np.ascontiguousarray(image[:: args.scale, :: args.scale]))
        request.images = [CompressedImage(header=msg.header, format="png", data=list(png))]
        if args.save:
            write_png(args.save, np.ascontiguousarray(image[:: args.scale, :: args.scale]))
            print(f"sent frame saved to {args.save}")

    started = time.time()
    future = client.call_async(request)
    while not future.done() and time.time() - started < args.timeout:
        time.sleep(0.05)
    if not future.done():
        print("timed out waiting for the model")
        return 1
    response = future.result()
    print(f"result {response.result} in {response.latency_s:.1f} s ({response.model})")
    print(response.text or "(empty)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
