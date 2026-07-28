"""Estimate external joint effort from the measured signal and model gravity.

The UR's /joint_states "effort" is filled from RTDE actual_current, i.e. motor
CURRENT (Amps), not torque (ur_robot_driver: readData(..., "actual_current",
urcl_joint_efforts_)). To get a gravity-free external *torque* we convert the
current to torque with a per-joint gain and subtract the Pinocchio gravity:

    tau_ext[Nm] = effort_gain * I  -  g(q)  -  offset

- effort_gain (k, Nm/A): per-joint current->torque constant (torque constant x
  gear ratio, ~10-12 for the UR7e big joints). The gravity term keeps coefficient
  1 — g(q) is the physically correct RNEA torque and is not rescaled; the unit
  conversion lives on the measurement, where it physically belongs.
- offset (Nm): per-joint constant (current bias / static friction).

Both are identified from contact-free motion (see fit_calibration /
calibrate_external_effort). CALIBRATION IS REQUIRED: with effort_gain=1 the
current and the Nm gravity are in different units and the output is meaningless.
For g(q) to be accurate the URDF must carry the true link masses, including the
tool payload (see identify_payload). Quasi-static assumption: inertial/Coriolis
torques are not subtracted, so readings during fast motion overestimate contact.

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
        effort_gain: NDArray | None = None,
        offset: NDArray | None = None,
    ):
        """Build the estimator.

        Args:
            urdf: URDF as an XML string (e.g. from /robot_description).
            joint_names: Actuated arm joints, in the order I/q are provided.
                All other movable joints in the URDF (e.g. gripper fingers) are
                locked at their neutral configuration, so their mass still loads
                the wrist.
            effort_gain: Optional per-joint current->torque gain k (Nm/A,
                default 1 — but 1 gives meaningless output; calibrate first).
            offset: Optional per-joint calibration offset (Nm, default 0).
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
        self.effort_gain = (
            np.ones(n) if effort_gain is None else np.asarray(effort_gain, dtype=float)
        )
        self.offset = np.zeros(n) if offset is None else np.asarray(offset, dtype=float)

    def gravity_effort(self, q: NDArray) -> NDArray:
        """Model gravity torque g(q) in the configured joint order."""
        q_pin = pin.neutral(self.model)
        q_pin[self._q_index] = q
        tau_g = pin.computeGeneralizedGravity(self.model, self.data, q_pin)
        return tau_g[self._q_index]

    def external_effort(self, q: NDArray, currents: NDArray) -> NDArray:
        """tau_ext = effort_gain * I - g(q) - offset (I = measured current)."""
        return (
            self.effort_gain * np.asarray(currents)
            - self.gravity_effort(q)
            - self.offset
        )

    def fit_calibration(self, qs: NDArray, currents: NDArray) -> tuple[NDArray, NDArray]:
        """Fit the per-joint current->torque gain and offset from contact-free samples.

        Record (q, I) pairs while the arm moves slowly with nothing touching it.
        Contact-free, effort_gain*I - g(q) - offset == 0, so per joint we fit

            g(q)_j ~ k_j * I_j + c_j     ->   effort_gain_j = k_j, offset_j = -c_j

        by least squares. Identifying k_j needs gravity to load the joint across
        the samples (drive shoulder_lift/elbow/wrist_1/wrist_2 through their
        gravity span); near-vertical joints (shoulder_pan, wrist_3) are barely
        loaded and their k is not identifiable here — the caller should fall back
        to a nominal gain for those. Stores and returns (effort_gain, offset).
        """
        qs = np.asarray(qs)
        currents = np.asarray(currents)
        gravity = np.stack([self.gravity_effort(q) for q in qs])
        n = gravity.shape[1]
        k = np.ones(n)
        c = np.zeros(n)
        for j in range(n):
            A = np.stack([currents[:, j], np.ones(len(qs))], axis=1)
            (k[j], c[j]), *_ = np.linalg.lstsq(A, gravity[:, j], rcond=None)
        self.effort_gain = k
        self.offset = -c
        return self.effort_gain, self.offset
