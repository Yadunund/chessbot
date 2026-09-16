# Copyright 2026 Yadunund Vijay
# SPDX-License-Identifier: Apache-2.0
"""Inverse kinematics and Cartesian interpolation for a serial arm, on Pinocchio and SciPy.

ROS-free so it can be tested on its own. The IK solves for tool *position*,
*approach direction* and, optionally, the *opening direction* of the jaws (roll
about the approach axis). A 5-DoF arm like the SO-101 can set all three for a
vertical approach; roll is left free when no opening direction is given.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pinocchio as pin
from scipy.optimize import least_squares


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
        opening_axis: tuple[float, float, float] = (-1.0, 0.0, 0.0),
        roll_tolerance_rad: float = np.radians(5.0),
    ):
        # Far from the base a top-down approach may be out of reach; the tool is
        # allowed to tilt up to this much from the requested approach direction.
        self.max_approach_tilt_rad = max_approach_tilt_rad
        self.roll_tolerance_rad = roll_tolerance_rad
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
        # The jaws' opening direction (fixed jaw towards moving jaw) in the tip frame,
        # made perpendicular to the approach axis.
        opening = np.asarray(opening_axis, dtype=float)
        opening = opening - (opening @ self.approach_local) * self.approach_local
        self.opening_local = opening / np.linalg.norm(opening)

    def _full_q(self, q_chain: np.ndarray) -> np.ndarray:
        q = pin.neutral(self.model)
        q[self.q_idx] = q_chain
        return q

    def forward(self, q_chain: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Tool position and approach direction (unit vector) in the model root frame."""
        position, approach, _ = self.forward_full(q_chain)
        return position, approach

    def forward_full(self, q_chain: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Tool position, approach direction and jaw opening direction in the model root frame."""
        q = self._full_q(q_chain)
        pin.framesForwardKinematics(self.model, self.data, q)
        tip = self.data.oMf[self.tip]
        return tip.translation.copy(), tip.rotation @ self.approach_local, tip.rotation @ self.opening_local

    def tool_pose(self, q_chain: np.ndarray) -> pin.SE3:
        q = self._full_q(q_chain)
        pin.framesForwardKinematics(self.model, self.data, q)
        return pin.SE3(self.data.oMf[self.tip])

    def _residual(self, q_chain, target_position, target_approach, weight, target_opening=None):
        position, approach, opening = self.forward_full(q_chain)
        parts = [position - target_position, weight * (approach - target_approach)]
        if target_opening is not None:
            parts.append(weight * (opening - target_opening))
        return np.concatenate(parts)

    @staticmethod
    def _roll_error(opening, approach, target_opening) -> float:
        """Angle between the jaw opening and the target, about the approach axis."""
        projected = target_opening - (target_opening @ approach) * approach
        norm = np.linalg.norm(projected)
        if norm < 1e-9:
            return np.pi
        return float(np.arccos(np.clip(opening @ (projected / norm), -1.0, 1.0)))

    def solve(
        self,
        target_position: np.ndarray,
        target_approach: np.ndarray,
        seed: np.ndarray,
        *,
        position_tolerance: float = 1e-3,
        approach_tolerance_rad: float = np.radians(3.0),
        weights: tuple[float, ...] = (0.3, 0.1, 0.03, 0.01),
        max_evaluations: int = 300,
        target_opening: np.ndarray | None = None,
    ) -> IkResult:
        """Bounded least squares on position and approach direction from one seed.

        Joint limits are handled by the optimiser (trust-region reflective)
        rather than by clipping, which otherwise pins the arm against a limit.
        The approach term is tried with decreasing weight: a strong weight keeps
        the tool as vertical as possible, a weak one lets position win where a
        vertical tool cannot reach, as long as the tilt stays within tolerance.
        """
        target_position = np.asarray(target_position, dtype=float)
        target_approach = np.asarray(target_approach, dtype=float)
        target_approach = target_approach / np.linalg.norm(target_approach)
        if target_opening is not None:
            target_opening = np.asarray(target_opening, dtype=float)
            target_opening = target_opening / np.linalg.norm(target_opening)
        x0 = np.clip(np.asarray(seed, dtype=float), self.lower + 1e-6, self.upper - 1e-6)
        best: IkResult | None = None
        for weight in weights:
            fit = least_squares(
                self._residual,
                x0,
                bounds=(self.lower, self.upper),
                args=(target_position, target_approach, weight, target_opening),
                method="trf",
                xtol=1e-10,
                ftol=1e-10,
                max_nfev=max_evaluations,
            )
            position, approach, opening = self.forward_full(fit.x)
            pos_err = float(np.linalg.norm(position - target_position))
            ang_err = float(np.arccos(np.clip(approach @ target_approach, -1.0, 1.0)))
            roll_ok = target_opening is None or self._roll_error(opening, approach, target_opening) < self.roll_tolerance_rad
            result = IkResult(
                fit.x, pos_err < position_tolerance and ang_err < approach_tolerance_rad and roll_ok, pos_err, ang_err
            )
            if result.success:
                return result
            if best is None or pos_err < best.position_error:
                best = result
        return best

    def solve_with_restarts(
        self,
        target_position: np.ndarray,
        target_approach: np.ndarray,
        seed: np.ndarray,
        *,
        samples: int = 500,
        seeds_to_refine: int = 4,
        restarts: int = 2,
        rng: np.random.Generator | None = None,
        target_opening: np.ndarray | None = None,
    ) -> IkResult:
        """IK that does not depend on a good seed.

        Tries the given seed, then the sampled configurations whose tool lands
        closest to the target, then a few random ones. The requested approach is
        preferred; the tool only tilts (up to `max_approach_tilt_rad`) when it
        has to. Among successes, the one closest to the given seed wins so the
        arm does not swing needlessly.
        """
        rng = rng or np.random.default_rng(0)
        seed = np.asarray(seed, dtype=float)
        target_position = np.asarray(target_position, dtype=float)
        target_approach = np.asarray(target_approach, dtype=float) / np.linalg.norm(target_approach)

        candidates = rng.uniform(self.lower, self.upper, size=(samples, len(self.joint_names)))
        distances = np.array([np.linalg.norm(self.forward(q)[0] - target_position) for q in candidates])
        seeds = [seed, *candidates[np.argsort(distances)[:seeds_to_refine]]]
        seeds += list(rng.uniform(self.lower, self.upper, size=(restarts, len(self.joint_names))))

        last: IkResult | None = None
        for tolerance in (np.radians(3.0), self.max_approach_tilt_rad):
            best: IkResult | None = None
            for candidate_seed in seeds:
                result = self.solve(
                    target_position,
                    target_approach,
                    candidate_seed,
                    approach_tolerance_rad=tolerance,
                    target_opening=target_opening,
                )
                last = result
                if result.success and (best is None or np.linalg.norm(result.q - seed) < np.linalg.norm(best.q - seed)):
                    best = result
            if best is not None:
                return best
        return last

    def cartesian_path(
        self,
        start_q: np.ndarray,
        waypoints: list[tuple],
        max_step: float = 0.005,
    ) -> tuple[list[np.ndarray], float]:
        """Follow straight lines through waypoints: (position, approach) or (position, approach, opening).

        Returns the joint configurations reached and the fraction of the path
        achieved (1.0 when every interpolated point was solved).
        """
        configs = [np.asarray(start_q, dtype=float)]
        position, approach, opening = self.forward_full(configs[0])
        segments = []
        total = 0.0
        for waypoint in waypoints:
            target_pos, target_app = np.asarray(waypoint[0], float), np.asarray(waypoint[1], float)
            target_open = np.asarray(waypoint[2], float) if len(waypoint) > 2 and waypoint[2] is not None else None
            length = float(np.linalg.norm(target_pos - position))
            segments.append((position, approach, opening, target_pos, target_app, target_open, length))
            total += length
            position, approach, opening = target_pos, target_app, target_open

        done = 0.0
        for p0, a0, o0, p1, a1, o1, length in segments:
            steps = max(1, int(np.ceil(length / max_step)))
            for i in range(1, steps + 1):
                s = i / steps
                p = (1 - s) * p0 + s * p1
                a = (1 - s) * a0 + s * a1
                a /= np.linalg.norm(a)
                o = None
                if o1 is not None:
                    o = o1 if o0 is None else (1 - s) * o0 + s * o1
                    o = o / np.linalg.norm(o)
                result = self.solve(
                    p, a, configs[-1], approach_tolerance_rad=self.max_approach_tilt_rad, target_opening=o
                )
                if not result.success:
                    return configs, (done + (i - 1) / steps * length) / total if total > 0 else 0.0
                configs.append(result.q)
            done += length
        return configs, 1.0
