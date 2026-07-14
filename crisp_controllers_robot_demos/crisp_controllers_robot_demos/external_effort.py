"""Estimate external joint effort by subtracting model gravity torque.

Current-derived joint effort (e.g. the UR's /joint_states effort) is the *total*
torque, including the torque spent holding the arm against gravity. This builds
a Pinocchio model from the robot URDF (ideally /robot_description, which already
carries the gripper masses), locks the non-arm joints, and subtracts the
gravity term g(q):

    tau_ext = tau_measured - (scale * g(q) + offset)

scale/offset are optional per-joint calibration gains fitted from contact-free
motion (they absorb current-to-torque scale errors and static friction).
Quasi-static assumption: inertial/Coriolis torques are not subtracted, so
readings during fast motion overestimate contact.

This module lives in the robot bring-up package so the (heavy) pinocchio
dependency stays on the robot side; crisp_gym consumes the published topic as a
plain float32_array sensor.
"""

from __future__ import annotations

import numpy as np
import pinocchio as pin
from numpy.typing import NDArray


class ExternalEffortEstimator:
    """Gravity-free joint effort from measured effort and a Pinocchio model."""

    def __init__(
        self,
        urdf: str,
        joint_names: list[str],
        scale: NDArray | None = None,
        offset: NDArray | None = None,
    ):
        """Build the estimator.

        Args:
            urdf: URDF as an XML string (e.g. from /robot_description).
            joint_names: Actuated arm joints, in the order tau/q are provided.
                All other movable joints in the URDF (e.g. gripper fingers) are
                locked at their neutral configuration, so their mass still loads
                the wrist.
            scale: Optional per-joint calibration gain a (default 1).
            offset: Optional per-joint calibration offset b (default 0).
        """
        full_model = pin.buildModelFromXML(urdf)
        lock_ids = [
            full_model.getJointId(name)
            for name in full_model.names
            if name != "universe" and name not in joint_names
        ]
        self.model = pin.buildReducedModel(full_model, lock_ids, pin.neutral(full_model))
        self.data = self.model.createData()

        missing = [n for n in joint_names if not self.model.existJointName(n)]
        if missing:
            raise ValueError(f"Joints not found in URDF: {missing}")
        # Map joint order -> pinocchio q indices (1-DOF joints only).
        self._q_index = np.array(
            [self.model.joints[self.model.getJointId(n)].idx_q for n in joint_names]
        )
        n = len(joint_names)
        self.scale = np.ones(n) if scale is None else np.asarray(scale, dtype=float)
        self.offset = np.zeros(n) if offset is None else np.asarray(offset, dtype=float)

    def gravity_effort(self, q: NDArray) -> NDArray:
        """Model gravity torque g(q) in the configured joint order."""
        q_pin = pin.neutral(self.model)
        q_pin[self._q_index] = q
        tau_g = pin.computeGeneralizedGravity(self.model, self.data, q_pin)
        return tau_g[self._q_index]

    def external_effort(self, q: NDArray, tau_measured: NDArray) -> NDArray:
        """tau_ext = tau_measured - (scale * g(q) + offset)."""
        return np.asarray(tau_measured) - (self.scale * self.gravity_effort(q) + self.offset)

    def fit_calibration(self, qs: NDArray, taus_measured: NDArray) -> tuple[NDArray, NDArray]:
        """Fit per-joint scale/offset from contact-free samples.

        Record (q, tau_measured) pairs while the arm moves slowly with nothing
        touching it, then least-squares fit tau_measured ~ a * g(q) + b per
        joint. Stores and returns (scale, offset).
        """
        qs = np.asarray(qs)
        taus = np.asarray(taus_measured)
        gravity = np.stack([self.gravity_effort(q) for q in qs])
        for j in range(gravity.shape[1]):
            A = np.stack([gravity[:, j], np.ones(len(qs))], axis=1)
            (a, b), *_ = np.linalg.lstsq(A, taus[:, j], rcond=None)
            self.scale[j], self.offset[j] = a, b
        return self.scale, self.offset
