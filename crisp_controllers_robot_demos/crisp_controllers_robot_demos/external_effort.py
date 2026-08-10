"""Estimate external joint effort from the measured signal and a dynamics model.

The UR's /joint_states "effort" is filled from RTDE actual_current, i.e. motor
CURRENT (Amps), not torque (ur_robot_driver: readData(..., "actual_current",
urcl_joint_efforts_)). We convert the current to torque with a per-joint gain and
subtract the model torque and joint friction, so the residual is the external
(contact) torque:

    tau_ext[Nm] = effort_gain * I  -  rnea(q, v, 0)  -  friction(v)  -  offset

- effort_gain (k, Nm/A): per-joint current->torque constant (~10-12 for the UR7e
  big joints). The unit conversion lives on the measurement.
- rnea(q, v, 0): full inverse dynamics with zero acceleration = gravity g(q) PLUS
  the Coriolis/centrifugal term C(q,v)v (both physically correct, coefficient 1).
  The inertia term M(q)*a is dropped (a would need noisy differentiation); keep
  motions slow so it stays negligible. See the momentum-observer approach for a
  formulation that avoids acceleration entirely.
- friction(v) = coulomb * tanh(v/eps) + viscous * v: the Coulomb+viscous joint
  friction model, identified from a *moving* contact-free recording. This is what
  velocity buys us over the quasi-static model. The Coulomb term uses a REGULARIZED
  sign (tanh) so it decays to zero at standstill; plain sign(v) would inject the
  full +-coulomb Nm on velocity noise while the robot is stationary.
- offset (Nm): per-joint constant (current bias / static holding term).

CALIBRATION IS REQUIRED (see fit_calibration / calibrate_external_effort): with
effort_gain=1 the current and the Nm model are in different units and the output
is meaningless. For rnea(q, v, 0) to be accurate the URDF must carry the true
link masses, including the tool payload (see identify_payload).

This module lives in the robot bring-up package so the (heavy) pinocchio
dependency stays on the robot side; crisp_gym consumes the published topic as a
plain float32_array sensor.
"""

from __future__ import annotations

import numpy as np
import pinocchio as pin
from numpy.typing import NDArray

#: Velocity scale (rad/s) over which Coulomb friction ramps up from zero.
DEFAULT_FRICTION_EPS = 0.05


def smooth_sign(v: NDArray, eps: float = DEFAULT_FRICTION_EPS) -> NDArray:
    """Regularized sign: tanh(v/eps), i.e. sign(v) smoothed around zero.

    Plain sign(v) is discontinuous: at |v| = 1e-3 (sensor noise at standstill) it
    already commands the FULL Coulomb magnitude, injecting a +-coulomb Nm jump into
    the estimate of a robot that is standing still. tanh(v/eps) goes smoothly to 0
    as v -> 0 and saturates to +-1 once |v| >> eps, which is the standard
    regularization for Coulomb friction in this kind of model.
    """
    return np.tanh(np.asarray(v, dtype=float) / eps)


class ExternalEffortEstimator:
    """External joint torque from measured current and a Pinocchio dynamics model."""

    def __init__(
        self,
        urdf: str,
        joint_names: list[str],
        effort_gain: NDArray | None = None,
        offset: NDArray | None = None,
        coulomb: NDArray | None = None,
        viscous: NDArray | None = None,
        friction_eps: float = DEFAULT_FRICTION_EPS,
    ):
        """Build the estimator.

        Args:
            urdf: URDF as an XML string (e.g. from /robot_description).
            joint_names: Actuated arm joints, in the order I/q/v are provided.
                All other movable joints in the URDF (e.g. gripper fingers) are
                locked at their neutral configuration, so their mass still loads
                the wrist.
            effort_gain: Per-joint current->torque gain k (Nm/A, default 1 — but
                1 gives meaningless output; calibrate first).
            offset: Per-joint constant offset (Nm, default 0).
            coulomb: Per-joint Coulomb friction magnitude (Nm, default 0).
            viscous: Per-joint viscous friction coefficient (Nm/(rad/s), default 0).
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
        # Map joint order -> pinocchio configuration (idx_q) and velocity (idx_v)
        # indices (1-DOF joints; for revolute joints idx_q == idx_v).
        self._q_index = np.array(
            [self.model.joints[self.model.getJointId(n)].idx_q for n in joint_names]
        )
        self._v_index = np.array(
            [self.model.joints[self.model.getJointId(n)].idx_v for n in joint_names]
        )
        n = len(joint_names)
        self.effort_gain = (
            np.ones(n) if effort_gain is None else np.asarray(effort_gain, dtype=float)
        )
        self.offset = np.zeros(n) if offset is None else np.asarray(offset, dtype=float)
        self.coulomb = np.zeros(n) if coulomb is None else np.asarray(coulomb, dtype=float)
        self.viscous = np.zeros(n) if viscous is None else np.asarray(viscous, dtype=float)
        self.friction_eps = float(friction_eps)

    def gravity_effort(self, q: NDArray) -> NDArray:
        """Model gravity torque g(q) in the configured joint order (v = a = 0)."""
        q_pin = pin.neutral(self.model)
        q_pin[self._q_index] = q
        tau_g = pin.computeGeneralizedGravity(self.model, self.data, q_pin)
        return tau_g[self._v_index]

    def model_effort(self, q: NDArray, v: NDArray) -> NDArray:
        """rnea(q, v, 0) = gravity g(q) + Coriolis/centrifugal C(q,v)v (Nm)."""
        q_pin = pin.neutral(self.model)
        q_pin[self._q_index] = q
        v_pin = np.zeros(self.model.nv)
        v_pin[self._v_index] = np.asarray(v)
        a_pin = np.zeros(self.model.nv)
        tau = pin.rnea(self.model, self.data, q_pin, v_pin, a_pin)
        return tau[self._v_index]

    def friction_effort(self, v: NDArray) -> NDArray:
        """Joint friction: coulomb*tanh(v/eps) + viscous*v (Nm).

        The Coulomb term uses the regularized sign so it vanishes at standstill
        instead of jumping to +-coulomb on velocity noise.
        """
        v = np.asarray(v)
        return self.coulomb * smooth_sign(v, self.friction_eps) + self.viscous * v

    def external_effort(self, q: NDArray, v: NDArray, currents: NDArray) -> NDArray:
        """tau_ext = gain*I - rnea(q,v,0) - friction(v) - offset (I = measured current)."""
        return (
            self.effort_gain * np.asarray(currents)
            - self.model_effort(q, v)
            - self.friction_effort(v)
            - self.offset
        )

    def fit_calibration(
        self, qs: NDArray, vs: NDArray, currents: NDArray
    ) -> tuple[NDArray, NDArray, NDArray, NDArray]:
        """Fit per-joint gain, friction and offset from contact-free samples.

        Record (q, v, I) while the arm moves slowly with nothing touching it,
        covering both STATIC holds (across each joint's gravity range, to fix the
        gain) and slow back-and-forth MOTION in both directions (to fix friction).
        Contact-free, gain*I - rnea(q,v,0) - friction(v) - offset == 0, so per
        joint we least-squares fit

            rnea_j ~ k_j*I_j - coulomb_j*sign(v_j) - viscous_j*v_j - offset_j

        Returns (effort_gain, offset, coulomb, viscous). calibrate_external_effort
        adds identifiability fallbacks (nominal gain for gravity-blind joints,
        zero friction where there is no motion); this helper does the plain fit.
        """
        qs = np.asarray(qs)
        vs = np.asarray(vs)
        currents = np.asarray(currents)
        model = np.stack([self.model_effort(q, v) for q, v in zip(qs, vs)])
        n = model.shape[1]
        k = np.ones(n)
        coul = np.zeros(n)
        visc = np.zeros(n)
        off = np.zeros(n)
        ones = np.ones(len(qs))
        for j in range(n):
            ss = smooth_sign(vs[:, j], self.friction_eps)
            A = np.stack([currents[:, j], -ss, -vs[:, j], -ones], axis=1)
            (k[j], coul[j], visc[j], off[j]), *_ = np.linalg.lstsq(
                A, model[:, j], rcond=None
            )
        self.effort_gain, self.coulomb, self.viscous, self.offset = k, coul, visc, off
        return self.effort_gain, self.offset, self.coulomb, self.viscous
