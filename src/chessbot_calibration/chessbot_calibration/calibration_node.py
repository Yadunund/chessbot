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
from chessbot_interfaces.srv import SetCalibration
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from chessbot_calibration import profile as prof


class CalibrationNode(Node):
    def __init__(self):
        super().__init__("calibration")
        self.profile_path = self.declare_parameter("profile_path", "calibration.yaml").value
        self.zenoh_endpoint = self.declare_parameter("zenoh_endpoint", "tcp/127.0.0.1:7447").value

        self.session = self._open_session()
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
        self.create_service(SetCalibration, "/calibration/set", self._on_set)
        self.get_logger().info("Serving /calibration/calibrate and /calibration/set")

    def _open_session(self, timeout_s: float = 30.0):
        """Wait for the router rather than dying if this node starts before it.

        Launch order should not decide whether the application comes up.
        """
        deadline = time.monotonic() + timeout_s
        warned = False
        while True:
            try:
                return zenoh.open(self._zenoh_config())
            except Exception as exc:  # noqa: BLE001 - any failure to reach the router
                if time.monotonic() >= deadline:
                    raise RuntimeError(
                        f"no Zenoh router at {self.zenoh_endpoint} after {timeout_s:.0f}s; "
                        "start one with `pixi run router`"
                    ) from exc
                if not warned:
                    self.get_logger().warning(f"Waiting for the Zenoh router at {self.zenoh_endpoint}...")
                    warned = True
                time.sleep(1.0)

    def _zenoh_config(self) -> zenoh.Config:
        config = zenoh.Config()
        config.insert_json5("mode", '"client"')
        config.insert_json5("connect/endpoints", f'["{self.zenoh_endpoint}"]')
        return config

    def _prime(self):
        self.session.put(prof.KV_KEY, self.profile.to_json(), encoding=zenoh.Encoding.APPLICATION_JSON)
        self.get_logger().info(f"Primed {prof.KV_KEY} (source={self.profile.source})")

    def _on_set(self, request: SetCalibration.Request, response: SetCalibration.Response):
        """Store a calibration measured elsewhere (the guided workflow in the UI).

        This node owns the stored profile and the key-value store, so a measurement taken by
        a node that has the arm and the camera is written here rather than there.
        """
        if request.square_size_m <= 0.0:
            response.ok = False
            response.message = "square_size_m must be positive"
            return response

        board = prof.BoardCalibration(
            origin_xyz=[float(v) for v in request.board_origin_xyz],
            yaw_rad=float(request.board_yaw_rad),
            square_size_m=float(request.square_size_m),
        )
        camera = self.profile.camera
        if len(request.camera_position_xyz) == 3 and len(request.camera_rpy) == 3:
            camera = prof.CameraCalibration(
                position_xyz=[float(v) for v in request.camera_position_xyz],
                rpy=[float(v) for v in request.camera_rpy],
                focal_px=float(request.camera_focal_px),
            )
        park = [float(v) for v in request.park_joints] or list(self.profile.park_joints)

        self.profile = prof.CalibrationProfile(board=board, camera=camera, park_joints=park, source="measured")
        prof.save(self.profile_path, self.profile)
        self._prime()
        self.get_logger().info(f"Stored a measured calibration in {self.profile_path}")
        response.ok = True
        response.message = "stored"
        response.profile_path = self.profile_path
        return response

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

        self.profile = prof.CalibrationProfile(
            board=self.profile.board, camera=self.profile.camera,
            park_joints=self.profile.park_joints, source=self.profile.source)
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
