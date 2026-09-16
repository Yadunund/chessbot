#!/usr/bin/env python3
# Copyright 2026 Yadunund Vijay
# SPDX-License-Identifier: Apache-2.0
"""Ad-hoc probes for debugging bringup (calls that the ros2 CLI hangs on)."""

import sys
import threading
import time

import rclpy
from chessbot_interfaces.srv import GetBoardState
from rcl_interfaces.srv import GetParameters
from rclpy.executors import MultiThreadedExecutor


def wait(future, timeout):
    done = threading.Event()
    future.add_done_callback(lambda _f: done.set())
    return future.result() if done.wait(timeout) else None


def main():
    rclpy.init()
    node = rclpy.create_node("probe")
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    what = sys.argv[1]
    if what == "board":
        client = node.create_client(GetBoardState, sys.argv[2])
        print("service ready:", client.wait_for_service(timeout_sec=10))
        t0 = time.time()
        res = wait(client.call_async(GetBoardState.Request(max_frame_age_s=0.0)), 10)
        print("response:", None if res is None else f"result={res.result} stamp={res.state.header.stamp.sec}", f"{time.time()-t0:.2f}s")
    elif what == "param":
        client = node.create_client(GetParameters, f"{sys.argv[2]}/get_parameters")
        print("service ready:", client.wait_for_service(timeout_sec=10))
        res = wait(client.call_async(GetParameters.Request(names=[sys.argv[3]])), 10)
        print("value:", None if res is None else res.values[0].string_value)
    ex.shutdown()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
