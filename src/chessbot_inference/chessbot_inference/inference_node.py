# Copyright 2026 Yadunund Vijay
# SPDX-License-Identifier: Apache-2.0
"""Reasoning capability: answers chessbot_interfaces/Reason with a vision-language model.

The model runs in llama-server (llama.cpp), reached over its OpenAI-compatible HTTP
API, so it can be on this host or another. Every request and answer is logged to the
node's log with its latency.
"""

from __future__ import annotations

import base64
import json
import time
import urllib.error
import urllib.request

import rclpy
from chessbot_interfaces.srv import Reason
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

SYSTEM_PROMPT = (
    "You are the reasoning module of a chess-playing robot arm. Answer briefly and only from the facts "
    "and images you are given. Never invent moves, evaluations or positions."
)
ROLE_NAMES = {Reason.Request.ROLE_COMMENTARY: "commentary", Reason.Request.ROLE_MOVE_TIEBREAK: "move_tiebreak",
              Reason.Request.ROLE_EXPLAIN: "explain"}


class InferenceNode(Node):
    def __init__(self):
        super().__init__("inference")
        self.url = self.declare_parameter("server_url", "http://127.0.0.1:8081").value.rstrip("/")
        self.timeout_s = float(self.declare_parameter("timeout_s", 30.0).value)
        self.max_tokens = int(self.declare_parameter("max_tokens", 200).value)
        self._model = ""
        self.create_service(Reason, "/inference/reason", self._on_reason)
        self.get_logger().info(f"Serving /inference/reason with the model at {self.url}")

    def _post(self, path: str, body: dict) -> dict:
        request = urllib.request.Request(
            self.url + path, data=json.dumps(body).encode(), headers={"Content-Type": "application/json"}
        )
        with urllib.request.urlopen(request, timeout=self.timeout_s) as response:
            return json.loads(response.read())

    def _on_reason(self, request: Reason.Request, response: Reason.Response) -> Reason.Response:
        role = ROLE_NAMES.get(request.role, str(request.role))
        content: list[dict] = [{"type": "text", "text": request.prompt}]
        for image in request.images:
            kind = "png" if "png" in image.format.lower() else "jpeg"
            data = base64.b64encode(bytes(image.data)).decode()
            content.append({"type": "image_url", "image_url": {"url": f"data:image/{kind};base64,{data}"}})
        body: dict = {
            "messages": [{"role": "system", "content": SYSTEM_PROMPT}, {"role": "user", "content": content}],
            "max_tokens": self.max_tokens,
            "temperature": 0.0 if request.role != Reason.Request.ROLE_COMMENTARY else 0.6,
            # These are short, grounded answers; thinking would spend the token budget before answering.
            "chat_template_kwargs": {"enable_thinking": False},
        }
        if request.json_schema:
            body["response_format"] = {"type": "json_schema", "json_schema": {"name": role, "schema": json.loads(request.json_schema)}}

        started = time.monotonic()
        try:
            answer = self._post("/v1/chat/completions", body)
        except urllib.error.URLError as exc:
            reason = getattr(exc, "reason", exc)
            unreachable = isinstance(reason, (ConnectionRefusedError, TimeoutError, OSError)) and not isinstance(exc, urllib.error.HTTPError)
            response.result = Reason.Response.RESULT_UNAVAILABLE if unreachable else Reason.Response.RESULT_FAILED
            self.get_logger().warning(f"{role}: model server error: {exc}")
            return response
        except Exception as exc:  # noqa: BLE001 - report any failure to the caller
            response.result = Reason.Response.RESULT_FAILED
            self.get_logger().warning(f"{role}: {exc}")
            return response

        response.latency_s = float(time.monotonic() - started)
        response.text = answer["choices"][0]["message"]["content"].strip()
        response.model = answer.get("model", "")
        response.result = Reason.Response.RESULT_OK
        self.get_logger().info(
            f"{role} ({response.latency_s:.1f} s, {len(request.images)} images): {request.prompt[:120]!r} -> {response.text[:200]!r}"
        )
        return response


def main():
    rclpy.init()
    node = InferenceNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
