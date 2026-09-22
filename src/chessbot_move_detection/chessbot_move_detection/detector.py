# Copyright 2026 Yadunund Vijay
# SPDX-License-Identifier: Apache-2.0
"""What every move detector has in common: the service, the timing, and the shape of an answer.

A detector is given the position, the moves the rules allow, and the board before and
after the human moved. It names one of those moves or says it cannot tell. It never
proposes a move of its own, and it never decides what happens next: the caller does.
"""

from __future__ import annotations

import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_services_default

from chessbot_interfaces.srv import DetectMove


def answer(
    result: int,
    reason: str,
    *,
    move: str = "",
    confidence: float = 0.0,
    candidates: list[tuple[str, float]] | None = None,
    model_used: bool = False,
) -> DetectMove.Response:
    """A response with its two parallel candidate arrays kept in step."""
    response = DetectMove.Response()
    response.result = result
    response.move = move
    response.confidence = float(confidence)
    response.reason = reason
    response.candidates = [name for name, _ in candidates or []]
    response.scores = [float(score) for _, score in candidates or []]
    response.model_used = model_used
    return response


class MoveDetector(Node):
    """Serves DetectMove. Subclasses implement `detect`."""

    def __init__(self, name: str, default_service: str = "/perception/detect_move"):
        super().__init__(name)
        service_name = self.declare_parameter("service_name", default_service).value
        # The service stays in the default callback group: on Lyrical with rmw_zenoh, a
        # service created in a separately created group is never executed. Clients a
        # subclass needs while answering go in a reentrant group of their own, so their
        # replies are handled while this callback is still waiting.
        self.create_service(DetectMove, service_name, self._on_detect, qos_profile=qos_profile_services_default)
        self.get_logger().info(f"Serving {service_name}")

    def detect(self, request: DetectMove.Request) -> DetectMove.Response:
        raise NotImplementedError

    def _on_detect(self, request: DetectMove.Request, _response: DetectMove.Response) -> DetectMove.Response:
        started = time.monotonic()
        if not request.fen_before or not request.legal_moves:
            response = answer(DetectMove.Response.RESULT_FAILED, "no position or no legal moves were given")
        else:
            try:
                response = self.detect(request)
            except Exception as exc:  # noqa: BLE001 - a detector failing is an answer, not a crash
                self.get_logger().error(f"detection failed: {exc}")
                response = answer(DetectMove.Response.RESULT_FAILED, f"move detection failed: {exc}")
        response.latency_s = float(time.monotonic() - started)
        self.get_logger().info(
            f"{request.fen_before} -> {response.move or 'nothing'} "
            f"(result {response.result}, {response.latency_s:.2f} s): {response.reason}"
        )
        return response


def spin(node: Node):
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
