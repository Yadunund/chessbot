# Copyright 2026 Yadunund Vijay
# SPDX-License-Identifier: Apache-2.0
"""Inverse kinematics and Cartesian interpolation for a serial arm, on Pinocchio.

ROS-free so it can be tested on its own. The IK solves for tool *position* and
*approach direction*, which is exactly what a 5-DoF arm like the SO-101 can
control: roll about the approach axis is left free.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pinocchio as pin


@dataclass
class IkResult:
    q: np.ndarray
    success: bool
    position_error: float
    approach_error_rad: float


class ArmKinematics:
    """Kinematics for one chain of a robot model.

    Args:
        urdf_xml: robot description.
        joint_names: the joints this solver moves, in order.
        tip_frame: frame whose position is controlled (the grasp point).
        approach_ref_frame: frame such that (tip - ref) is the tool's approach direction.
    """

    def __init__(
        self,
        urdf_xml: str,
        joint_names: list[str],
        tip_frame: str,
        approach_ref_frame: str,
        max_approach_tilt_rad: float = np.radians(25.0),
    ):
        # Far from the base a top-down approach may be out of reach; the tool is
        # allowed to tilt up to this much from the requested approach direction.
        self.max_approach_tilt_rad = max_approach_tilt_rad
        self.model = pin.buildModelFromXML(urdf_xml)
        self.data = self.model.createData()
        self.joint_names = list(joint_names)
        self.tip = self.model.getFrameId(tip_frame)
        self.ref = self.model.getFrameId(approach_ref_frame)
        if self.tip >= self.model.nframes or self.ref >= self.model.nframes:
            raise ValueError(f"unknown frame {tip_frame!r} or {approach_ref_frame!r}")
        self.q_idx = []
        self.v_idx = []
        for name in self.joint_names:
            jid = self.model.getJointId(name)
            if jid >= self.model.njoints:
                raise ValueError(f"unknown joint {name!r}")
            self.q_idx.append(self.model.joints[jid].idx_q)
            self.v_idx.append(self.model.joints[jid].idx_v)
        self.lower = self.model.lowerPositionLimit[self.q_idx]
        self.upper = self.model.upperPositionLimit[self.q_idx]
        # The approach axis expressed in the tip frame is constant for a rigid tool.
        q0 = pin.neutral(self.model)
        pin.framesForwardKinematics(self.model, self.data, q0)
        a_world = self.data.oMf[self.tip].translation - self.data.oMf[self.ref].translation
        a_world /= np.linalg.norm(a_world)
        self.approach_local = self.data.oMf[self.tip].rotation.T @ a_world

    def _full_q(self, q_chain: np.ndarray) -> np.ndarray:
        q = pin.neutral(self.model)
        q[self.q_idx] = q_chain
        return q

    def forward(self, q_chain: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Tool position and approach direction (unit vector) in the model root frame."""
        q = self._full_q(q_chain)
        pin.framesForwardKinematics(self.model, self.data, q)
        tip = self.data.oMf[self.tip]
        return tip.translation.copy(), tip.rotation @ self.approach_local

    def tool_pose(self, q_chain: np.ndarray) -> pin.SE3:
        q = self._full_q(q_chain)
        pin.framesForwardKinematics(self.model, self.data, q)
        return pin.SE3(self.data.oMf[self.tip])

    def solve(
        self,
        target_position: np.ndarray,
        target_approach: np.ndarray,
        seed: np.ndarray,
        *,
        position_tolerance: float = 1e-3,
        approach_tolerance_rad: float = np.radians(3.0),
        approach_weight: float = 0.3,
        max_iterations: int = 300,
        damping: float = 1e-3,
    ) -> IkResult:
        """Damped least-squares IK on position (3) and approach direction (2)."""
        target_approach = np.asarray(target_approach, dtype=float)
        target_approach /= np.linalg.norm(target_approach)
        q_chain = np.clip(np.asarray(seed, dtype=float), self.lower, self.upper)
        pos_err = np.inf
        ang_err = np.inf
        for _ in range(max_iterations):
            q = self._full_q(q_chain)
            pin.computeJointJacobians(self.model, self.data, q)
            pin.framesForwardKinematics(self.model, self.data, q)
            tip = self.data.oMf[self.tip]
            approach = tip.rotation @ self.approach_local

            e_pos = target_position - tip.translation
            # Rotation that would take the current approach onto the target one.
            e_rot = np.cross(approach, target_approach)
            pos_err = float(np.linalg.norm(e_pos))
            ang_err = float(np.arccos(np.clip(approach @ target_approach, -1.0, 1.0)))
            if pos_err < position_tolerance and ang_err < approach_tolerance_rad:
                return IkResult(q_chain, True, pos_err, ang_err)

            J = pin.getFrameJacobian(self.model, self.data, self.tip, pin.LOCAL_WORLD_ALIGNED)[:, self.v_idx]
            J_task = np.vstack([J[:3], approach_weight * J[3:]])
            e_task = np.concatenate([e_pos, approach_weight * e_rot])
            JJt = J_task @ J_task.T + damping * np.eye(6)
            dq = J_task.T @ np.linalg.solve(JJt, e_task)
            q_chain = np.clip(q_chain + dq, self.lower, self.upper)
        return IkResult(q_chain, False, pos_err, ang_err)

    def solve_with_restarts(
        self,
        target_position: np.ndarray,
        target_approach: np.ndarray,
        seed: np.ndarray,
        *,
        restarts: int = 20,
        rng: np.random.Generator | None = None,
    ) -> IkResult:
        """Try the seed first, then random seeds within joint limits.

        Damped least squares converges to local minima from far-away seeds; a
        handful of restarts makes a one-off IK query reliable. Among successful
        solutions, the one closest to the given seed wins, so the arm does not
        swing to a distant configuration.
        """
        seed = np.asarray(seed, dtype=float)
        # Prefer the requested approach; only tilt when it cannot be met.
        for tolerance in (np.radians(3.0), self.max_approach_tilt_rad):
            rng = rng or np.random.default_rng(0)
            best = self.solve(target_position, target_approach, seed, approach_tolerance_rad=tolerance)
            if best.success:
                return best
            for _ in range(restarts):
                candidate_seed = rng.uniform(self.lower, self.upper)
                result = self.solve(target_position, target_approach, candidate_seed, approach_tolerance_rad=tolerance)
                if result.success and (
                    not best.success or np.linalg.norm(result.q - seed) < np.linalg.norm(best.q - seed)
                ):
                    best = result
            if best.success:
                return best
        return best

    def cartesian_path(
        self,
        start_q: np.ndarray,
        waypoints: list[tuple[np.ndarray, np.ndarray]],
        max_step: float = 0.005,
    ) -> tuple[list[np.ndarray], float]:
        """Follow straight lines through waypoints (position, approach).

        Returns the joint configurations reached and the fraction of the path
        achieved (1.0 when every interpolated point was solved).
        """
        configs = [np.asarray(start_q, dtype=float)]
        position, approach = self.forward(configs[0])
        segments = []
        total = 0.0
        for target_pos, target_app in waypoints:
            length = float(np.linalg.norm(np.asarray(target_pos) - position))
            segments.append((position, approach, np.asarray(target_pos, float), np.asarray(target_app, float), length))
            total += length
            position, approach = np.asarray(target_pos, float), np.asarray(target_app, float)

        done = 0.0
        for p0, a0, p1, a1, length in segments:
            steps = max(1, int(np.ceil(length / max_step)))
            for i in range(1, steps + 1):
                s = i / steps
                p = (1 - s) * p0 + s * p1
                a = (1 - s) * a0 + s * a1
                a /= np.linalg.norm(a)
                result = self.solve(
                    p, a, configs[-1], max_iterations=100, approach_tolerance_rad=self.max_approach_tilt_rad
                )
                if not result.success:
                    return configs, (done + (i - 1) / steps * length) / total if total > 0 else 0.0
                configs.append(result.q)
            done += length
        return configs, 1.0
