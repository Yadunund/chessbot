# Copyright 2026 Yadunund Vijay
# SPDX-License-Identifier: Apache-2.0
"""Calibration capability.

On startup: loads the calibration YAML if it exists (otherwise writes the
nominal profile) and primes the Zenoh key-value store with it as JSON. Other
nodes read calibration only from the key-value store.

Serves ``/calibration/calibrate`` (chessbot_interfaces/action/Calibrate) to run
a fresh calibration, which overwrites the YAML and re-primes the store. The
measurement itself is not implemented yet: the action walks through its stages
and re-publishes the nominal geometry, so the contract and the flow can be
exercised end to end.
"""

from __future__ import annotations

import time

import rclpy
import zenoh
from chessbot_interfaces.action import Calibrate
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from chessbot_calibration import profile as prof


class CalibrationNode(Node):
    def __init__(self):
        super().__init__("calibration")
        self.profile_path = self.declare_parameter("profile_path", "calibration.yaml").value
        self.zenoh_endpoint = self.declare_parameter("zenoh_endpoint", "tcp/127.0.0.1:7447").value

        self.session = zenoh.open(self._zenoh_config())
        self.profile = prof.load(self.profile_path)
        if self.profile is None:
            self.get_logger().warning(f"No usable calibration at {self.profile_path}; writing the nominal profile")
            self.profile = prof.CalibrationProfile()
            prof.save(self.profile_path, self.profile)
        self._prime()

        self._server = ActionServer(
            self,
            Calibrate,
            "/calibration/calibrate",
            execute_callback=self._execute,
            goal_callback=lambda _goal: GoalResponse.ACCEPT,
            cancel_callback=lambda _goal: CancelResponse.ACCEPT,
        )
        self.get_logger().info("Serving /calibration/calibrate")

    def _zenoh_config(self) -> zenoh.Config:
        config = zenoh.Config()
        config.insert_json5("mode", '"client"')
        config.insert_json5("connect/endpoints", f'["{self.zenoh_endpoint}"]')
        return config

    def _prime(self):
        self.session.put(prof.KV_KEY, self.profile.to_json(), encoding=zenoh.Encoding.APPLICATION_JSON)
        self.get_logger().info(f"Primed {prof.KV_KEY} (source={self.profile.source})")

    def _execute(self, goal_handle):
        feedback = Calibrate.Feedback()
        result = Calibrate.Result()
        stages = [
            (Calibrate.Feedback.STAGE_LOCATING_BOARD, 0.2),
            (Calibrate.Feedback.STAGE_SURVEYING, 0.6),
            (Calibrate.Feedback.STAGE_SOLVING, 0.8),
            (Calibrate.Feedback.STAGE_VERIFYING, 1.0),
        ]
        for stage, progress in stages:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.result = Calibrate.Result.RESULT_CANCELLED
                return result
            feedback.stage = stage
            feedback.progress = progress
            goal_handle.publish_feedback(feedback)
            # TODO(chessbot): locate the board, survey with the wrist camera,
            # solve, verify. Until then each stage only reports progress.
            time.sleep(0.2)

        self.profile = prof.CalibrationProfile(board=self.profile.board, source=self.profile.source)
        prof.save(self.profile_path, self.profile)
        self._prime()
        goal_handle.succeed()
        result.result = Calibrate.Result.RESULT_OK
        result.profile_path = self.profile_path
        return result

    def destroy_node(self):
        self.session.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = CalibrationNode()
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
