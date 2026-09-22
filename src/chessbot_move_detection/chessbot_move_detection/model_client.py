# Copyright 2026 Yadunund Vijay
# SPDX-License-Identifier: Apache-2.0
"""Asking the reasoning model a question whose answer must fit a schema.

Both detectors reach the model the same way, through ``/inference/reason``, and both
treat every failure the same: an unreadable, late or missing answer is a refusal, not
something to retry. A retry only gives the model another chance to guess.
"""

from __future__ import annotations

import json
import threading

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from chessbot_interfaces.srv import Reason


class ModelClient:
    """The Reason service, called from inside another service's callback."""

    def __init__(self, node: Node, timeout_s: float = 60.0):
        self.node = node
        self.timeout_s = timeout_s
        # A reentrant group, so the model's reply is handled while the callback that
        # asked for it is still waiting. The service being answered stays in the
        # default group, which is where rmw_zenoh will actually execute it.
        self.client = node.create_client(Reason, "/inference/reason", callback_group=ReentrantCallbackGroup())

    def available(self) -> bool:
        return self.client.service_is_ready()

    def ask(self, role: int, prompt: str, images, schema: dict) -> dict | None:
        """The model's answer parsed against `schema`, or None if it did not give one."""
        if not self.available():
            return None
        request = Reason.Request(
            role=role, prompt=prompt, images=[i for i in images if i.data], json_schema=json.dumps(schema)
        )
        future = self.client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(self.timeout_s):
            future.cancel()
            self.node.get_logger().warning(f"the model did not answer within {self.timeout_s:.0f} s")
            return None
        response = future.result()
        if response is None or response.result != Reason.Response.RESULT_OK:
            return None
        try:
            return json.loads(response.text)
        except json.JSONDecodeError:
            self.node.get_logger().warning(f"the model's answer wasn't readable: {response.text[:120]!r}")
            return None
