"""Generalized momentum observer for external joint torque (De Luca & Mattone).

Estimates external joint torque from proprioception only (q, v, applied torque),
WITHOUT joint acceleration — sidestepping the noisy differentiation and the
unmodelled-inertia problem that limit the quasi-static estimator during motion.

For the rigid-body model  M(q)q̈ + C(q,v)v + g(q) = tau + tau_ext,  the generalised
momentum p = M(q)v obeys (using Ṁ = C + Cᵀ):

    ṗ = tau + tau_ext + C(q,v)ᵀ v - g(q)

so the observed residual

    r = K_O · ( p - ∫[ tau + C(q,v)ᵀ v - g(q) + r ] dt - p(0) )

follows  ṙ = K_O ( tau_ext - r ):  a first-order low-pass estimate of the external
torque with per-joint bandwidth K_O (observer_gain). Inertia and Coriolis are
handled exactly; the remaining error is model uncertainty — mainly joint friction,
so `tau` is the friction-compensated applied torque

    tau = effort_gain * I - offset - coulomb*sign(v) - viscous*v

(I = /joint_states current). This reuses ExternalEffortEstimator to build the
reduced Pinocchio model and to hold the current->torque / friction calibration.
"""

from __future__ import annotations

import numpy as np
import pinocchio as pin
from numpy.typing import NDArray

from crisp_controllers_robot_demos.external_effort import ExternalEffortEstimator


class MomentumObserver:
    """Filtered external-torque residual from the generalized momentum."""

    def __init__(
        self,
        urdf: str,
        joint_names: list[str],
        effort_gain: NDArray | None = None,
        offset: NDArray | None = None,
        coulomb: NDArray | None = None,
        viscous: NDArray | None = None,
        observer_gain: float | NDArray = 20.0,
    ):
        # Reuse the estimator for the reduced model, indices and calibration.
        self._est = ExternalEffortEstimator(
            urdf, joint_names, effort_gain, offset, coulomb, viscous
        )
        self.model = self._est.model
        self.data = self._est.data
        self._q_index = self._est._q_index
        self._v_index = self._est._v_index
        self.effort_gain = self._est.effort_gain
        self.offset = self._est.offset
        self.coulomb = self._est.coulomb
        self.viscous = self._est.viscous

        n = len(joint_names)
        self.observer_gain = (
            np.full(n, float(observer_gain))
            if np.isscalar(observer_gain)
            else np.asarray(observer_gain, dtype=float)
        )
        self._p0: NDArray | None = None
        self._integral = np.zeros(n)
        self._r = np.zeros(n)
        self._initialized = False

    def reset(self) -> None:
        """Drop the observer state (call after a time gap in the data)."""
        self._initialized = False

    def _dynamics(self, q: NDArray, v: NDArray):
        """Return (p, Cᵀv, g) in the configured joint order."""
        q_pin = pin.neutral(self.model)
        q_pin[self._q_index] = q
        v_pin = np.zeros(self.model.nv)
        v_pin[self._v_index] = v

        m = pin.crba(self.model, self.data, q_pin)  # upper-triangular mass matrix
        m = np.triu(m) + np.triu(m, 1).T  # symmetrize
        g = pin.computeGeneralizedGravity(self.model, self.data, q_pin)
        c = pin.computeCoriolisMatrix(self.model, self.data, q_pin, v_pin)

        p = (m @ v_pin)[self._v_index]
        cor = (c.T @ v_pin)[self._v_index]
        return p, cor, g[self._v_index]

    def update(self, q: NDArray, v: NDArray, currents: NDArray, dt: float) -> NDArray:
        """Advance the observer one step and return the external-torque estimate.

        dt is the time since the previous call (s). A non-positive or missing dt
        (first call / time gap) re-initialises the state and returns zeros.
        """
        p, cor, g = self._dynamics(q, v)
        tau = (
            self.effort_gain * np.asarray(currents)
            - self.offset
            - self.coulomb * np.sign(v)
            - self.viscous * np.asarray(v)
        )
        if not self._initialized or dt <= 0.0:
            self._p0 = p.copy()
            self._integral = np.zeros_like(p)
            self._r = np.zeros_like(p)
            self._initialized = True
            return self._r

        # r follows dr = K_O (tau_ext - r); the integrand carries the previous r.
        self._integral = self._integral + (tau + cor - g + self._r) * dt
        self._r = self.observer_gain * (p - self._p0 - self._integral)
        return self._r
