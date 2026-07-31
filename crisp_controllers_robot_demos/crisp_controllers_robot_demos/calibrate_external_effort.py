"""Record contact-free joint effort samples and fit the external-effort calibration.

The UR's /joint_states "effort" is motor CURRENT (Amps), not torque, so per joint
we fit a current->torque gain k (Nm/A), Coulomb+viscous friction and a constant
offset such that, contact-free,

    k*I - rnea(q, v, 0) - (coulomb*sign(v) + viscous*v) - offset == 0

rnea(q, v, 0) is gravity + Coriolis (coefficient 1; the model is physically
correct). This script records (q, v, I) with NOTHING touching the arm and
least-squares fits  model ~ k*I - coulomb*sign(v) - viscous*v - offset. For the
model to be accurate the URDF must carry the true masses, including the tool
payload (see identify_payload). The YAML is loaded by external_effort_node.

Record BOTH, per joint, because they identify different terms:
  (1) STATIC holds across each joint's gravity range -> identify the gain k:
    - shoulder_lift : raise/lower the whole arm, horizontal -> up -> down.
    - elbow         : fully fold and fully extend the elbow.
    - wrist_1       : pitch the wrist up and down.
    - wrist_2       : ROLL the wrist so its axis tilts between vertical/horizontal.
    - shoulder_pan and wrist_3 rotate about near-vertical axes: gravity barely
      loads them, so their k is unidentifiable (span < min_span) and falls back to
      the mean of the identified gains. Expected, not an error.
  (2) slow STEADY sweeps at a CONSTANT speed, both directions, at 2-3 speeds ->
      identify Coulomb (sign(v)) and viscous (v) friction. Only near-constant-
      velocity samples (|accel| <= accel_max) are used for the friction fit, so
      the unmodeled inertia M(q)*a does not corrupt it; a joint without enough
      such samples keeps zero friction (friction ID needs both signs and >=2
      distinct speeds, or Coulomb and viscous cannot be separated).
Keep motion SLOW: the inertia term M(q)*a is not modeled at run time. Recording
stops on ENTER (default) or after 'duration' seconds.

Usage (UR7e example):

    ros2 run crisp_controllers_robot_demos calibrate_external_effort \\
        --ros-args -p joint_names:="[shoulder_pan_joint, shoulder_lift_joint, \\
        elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint]"

Parameters:
    joint_names (string[])   REQUIRED — actuated arm joints, in order.
    stop_on_key (bool)       stop when ENTER is pressed (default True).
    duration (double)        max recording seconds / safety cap (default 120).
    sample_rate (double)     sampling rate in Hz (default 5).
    friction_min_vel (double) max |velocity| (rad/s) below which a joint is treated
                             as static and its friction stays 0 (default 0.05).
    accel_max (double)       max |acceleration| (rad/s^2) for a sample to be used in
                             the friction fit; larger = inertia-contaminated, so it
                             is dropped (default 0.2).
    min_span (double)        gravity span (Nm) below which a joint's gain is
                             unidentifiable, so it uses the nominal gain (default 1).
    fixed_gain (double[])    pin the current->torque gain (Nm/A) instead of fitting
                             it: one value for all joints, or one per joint (0 = fit
                             that joint). Only the offset is fit for pinned joints.
                             The gain is a physical constant, so pinning it removes
                             the run-to-run variance of re-estimating it.
    nominal_gain (double)    gain for joints whose gain is unidentifiable
                             (0 = mean of the identified/fixed gains, the default).
    output_file (str)        YAML path. Default is config/ur/
                             external_effort_calibration.yaml, which
                             external_effort.launch.py auto-loads.
    joint_state_topic (str)  default "joint_states".
"""

import os
import sys
import threading

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from crisp_controllers_robot_demos.external_effort import (
    ExternalEffortEstimator,
    smooth_sign,
)

# Default calibration path in the SOURCE tree (resolved from this file, which
# --symlink-install points into the bind-mounted source), so the calibration
# persists across container rebuilds. external_effort.launch.py auto-loads this
# same path, so calibrating with the default makes the launch pick it up.
_PKG_DIR = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
DEFAULT_CALIBRATION = os.path.join(
    _PKG_DIR, "config", "ur", "external_effort_calibration.yaml"
)


class CalibrateExternalEffort(Node):
    """Record contact-free (q, v, I) and fit gain/friction/offset per joint."""

    def __init__(self):
        super().__init__("calibrate_external_effort")

        prefix = self.get_namespace().strip("/")
        self._prefix = prefix + "_" if prefix else ""

        self._joint_names = [
            j for j in self.declare_parameter("joint_names", [""]).value if j
        ]
        if not self._joint_names:
            raise RuntimeError("calibrate_external_effort requires 'joint_names'.")
        self._stop_on_key = self.declare_parameter("stop_on_key", True).value
        self._duration = self.declare_parameter("duration", 120.0).value
        self._sample_rate = self.declare_parameter("sample_rate", 5.0).value
        self._min_span = self.declare_parameter("min_span", 1.0).value
        # Friction (coulomb/viscous) is only identified for a joint that actually
        # MOVES: a joint whose max |velocity| across the recording stays below this
        # is treated as static and its friction is left at 0 (rad/s).
        self._friction_min_vel = self.declare_parameter("friction_min_vel", 0.05).value
        # Friction is only fit on NEAR-CONSTANT-VELOCITY samples (|accel| below
        # this): where the joint accelerates, the unmodeled inertia M*a swamps the
        # ~1 Nm friction (worst on the big joints) and corrupts the fit (rad/s^2).
        self._accel_max = self.declare_parameter("accel_max", 0.2).value
        # Fixed current->torque gain(s), Nm/A. The gain is a physical constant
        # (torque constant x gear ratio), so once known it is more robust to pin it
        # than to re-estimate it from every recording. One value = all joints, or
        # one per joint (0 = fit that joint normally). Only the offset is then fit.
        self._fixed_gain = list(self.declare_parameter("fixed_gain", [0.0]).value)
        # Gain used for joints whose gain is unidentifiable; 0 = mean of the
        # identified/fixed gains (the previous behaviour).
        self._nominal_gain = float(self.declare_parameter("nominal_gain", 0.0).value)
        # Viscous friction is off by default: tanh(v/eps) and v are near-collinear
        # unless the sweeps span clearly different speeds, and the degenerate pair
        # blows up (observed coulomb +16.4 with viscous -18.3 cancelling).
        self._fit_viscous = bool(self.declare_parameter("fit_viscous", False).value)
        self._output_file = self.declare_parameter(
            "output_file", DEFAULT_CALIBRATION
        ).value
        joint_state_topic = self.declare_parameter(
            "joint_state_topic", "joint_states"
        ).value

        self._urdf: str | None = None
        self._last_msg: JointState | None = None
        self._msg_index: list[int] | None = None
        self._model_joint_names: list[str] | None = None
        self._estimator: ExternalEffortEstimator | None = None
        self._qs: list[np.ndarray] = []
        self._vs: list[np.ndarray] = []
        self._taus: list[np.ndarray] = []
        self._gravity: list[np.ndarray] = []
        self._model: list[np.ndarray] = []
        self._times: list[float] = []
        self._started = False
        self._stop_requested = False
        self.done = False

        self.create_subscription(
            String,
            "robot_description",
            self._on_urdf,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.create_subscription(
            JointState, joint_state_topic, self._on_joint_state, qos_profile_sensor_data
        )
        self._timer = self.create_timer(1.0 / self._sample_rate, self._sample)
        if self._stop_on_key:
            threading.Thread(target=self._wait_for_key, daemon=True).start()

        self.get_logger().info(
            f"Waiting for /robot_description and '{joint_state_topic}'...\n"
            "  Nothing touching the arm. Do BOTH, per joint:\n"
            "   (1) STATIC holds across each joint's gravity range (fixes the gain)\n"
            "       shoulder_lift: raise/lower | elbow: fold/extend | wrist_1: pitch\n"
            "       | wrist_2: ROLL so its axis tilts\n"
            "   (2) slow STEADY sweeps at a CONSTANT speed, both directions, at\n"
            "       2-3 speeds (fixes Coulomb+viscous friction). Hold each speed —\n"
            "       accelerating samples are dropped (inertia would corrupt them).\n"
            "  (shoulder_pan / wrist_3 can't be gravity-excited — that's fine.)\n"
            + (
                "  Press ENTER to stop and fit.\n"
                if self._stop_on_key
                else f"  Recording for {self._duration:.0f}s.\n"
            )
        )

    def _wait_for_key(self) -> None:
        try:
            sys.stdin.readline()
        except Exception:  # noqa: BLE001 (stdin may be closed under some launchers)
            return
        self._stop_requested = True

    def _on_urdf(self, msg: String) -> None:
        self._urdf = msg.data

    def _on_joint_state(self, msg: JointState) -> None:
        self._last_msg = msg

    def _match_joints(self, msg: JointState) -> bool:
        name_to_i = {name: i for i, name in enumerate(msg.name)}
        for candidate in (
            [self._prefix + n for n in self._joint_names],
            list(self._joint_names),
        ):
            if all(n in name_to_i for n in candidate):
                self._model_joint_names = candidate
                self._msg_index = [name_to_i[n] for n in candidate]
                return True
        self.get_logger().warning(
            f"JointState does not contain all of {self._joint_names}; waiting.",
            throttle_duration_sec=5.0,
        )
        return False

    def _sample(self) -> None:
        if self.done or self._urdf is None or self._last_msg is None:
            return
        msg = self._last_msg
        if self._msg_index is None and not self._match_joints(msg):
            return
        if len(msg.effort) <= max(self._msg_index):
            self.get_logger().warning(
                "JointState has no effort field — cannot calibrate.",
                throttle_duration_sec=5.0,
            )
            return

        if not self._started:
            self._started = True
            self._estimator = ExternalEffortEstimator(
                self._urdf, self._model_joint_names
            )
            self._start_time = self.get_clock().now()
            self.get_logger().info("Recording started. Move the arm now.")

        q = np.array([msg.position[i] for i in self._msg_index])
        tau = np.array([msg.effort[i] for i in self._msg_index])
        if len(msg.velocity) > max(self._msg_index):
            v = np.array([msg.velocity[i] for i in self._msg_index])
        else:
            v = np.zeros_like(q)
        if np.isnan(tau).any() or np.isnan(q).any() or np.isnan(v).any():
            bad = [
                self._model_joint_names[j]
                for j in range(len(tau))
                if np.isnan(tau[j]) or np.isnan(q[j]) or np.isnan(v[j])
            ]
            self.get_logger().warning(
                f"Skipping sample: NaN effort/position/velocity on {bad}. This arm "
                "joint is not reporting a valid value — calibration cannot proceed "
                "until the driver publishes it.",
                throttle_duration_sec=5.0,
            )
            return  # a NaN on any recorded joint would poison the fit

        # Record BOTH static and moving samples: static ones fix the gain (gravity
        # variation), moving ones fix Coulomb+viscous friction (velocity). The
        # model term rnea(q, v, 0) accounts for gravity + Coriolis at each sample.
        self._qs.append(q)
        self._vs.append(v)
        self._taus.append(tau)
        self._gravity.append(self._estimator.gravity_effort(q))
        self._model.append(self._estimator.model_effort(q, v))
        self._times.append(self.get_clock().now().nanoseconds / 1e9)

        # Live diversity feedback: current gravity span per joint.
        n = len(self._qs)
        if n % int(3 * self._sample_rate) == 0:
            grav = np.stack(self._gravity)
            spans = np.ptp(grav, axis=0)
            report = "  ".join(
                f"{name.replace(self._prefix, '')}:{spans[j]:.1f}"
                f"{'OK' if spans[j] >= self._min_span else '..'}"
                for j, name in enumerate(self._model_joint_names)
            )
            self.get_logger().info(f"[{n} samples] gravity span Nm  {report}")

        elapsed = (self.get_clock().now() - self._start_time).nanoseconds / 1e9
        if self._stop_requested or elapsed >= self._duration:
            if n < 10:
                self.get_logger().warning(
                    f"Only {n} samples — record longer before stopping."
                )
                self._stop_requested = False
                return
            self._fit_and_save()
            self.done = True

    def _fit_and_save(self) -> None:
        qs = np.stack(self._qs)
        vs = np.stack(self._vs)
        currents = np.stack(self._taus)  # /joint_states effort is motor current (A)
        gravity = np.stack(self._gravity)
        model = np.stack(self._model)  # rnea(q, v, 0) = gravity + Coriolis
        times = np.asarray(self._times)
        # Per-joint acceleration (central difference over the recorded times) — used
        # only to select near-constant-velocity samples for the friction fit, so the
        # unmodeled inertia M*a does not corrupt it.
        accel = np.gradient(vs, times, axis=0) if len(times) > 2 else np.zeros_like(vs)
        vmax = np.max(np.abs(vs), axis=0)
        n = len(self._model_joint_names)
        ones = np.ones(len(qs))

        # The gain is fit on STATIC samples only, so identifiability must be judged
        # on the gravity span WITHIN that subset: a joint can sweep through a wide
        # gravity range while every *held* pose sits at nearly the same gravity,
        # which silently gives a degenerate (garbage) gain fit.
        arm_static = np.max(np.abs(vs), axis=1) < self._friction_min_vel
        stat = arm_static if arm_static.sum() >= 3 else np.ones(len(qs), dtype=bool)
        spans = np.ptp(gravity[stat], axis=0)  # span over the samples used for the gain
        spans_all = np.ptp(gravity, axis=0)  # span over everything (reported only)

        # TWO-STAGE fit so the gain is not confounded by motion. On moving samples
        # current and velocity are collinear (and inertia M*a leaks in), so a joint
        # combined fit trades the gain against the viscous term. Instead:
        #   Stage 1 (gain + offset): STATIC samples only (whole arm settled), where
        #     friction and Coriolis vanish, so gravity == k*I - offset.
        #   Stage 2 (Coulomb + viscous): that joint's MOVING samples with the gain
        #     and offset FIXED, so the residual k*I - model - offset is friction(v).
        effort_gain = np.ones(n)
        coulomb = np.zeros(n)
        viscous = np.zeros(n)
        offset = np.zeros(n)
        has_gain = spans >= self._min_span
        if arm_static.sum() < 10:
            self.get_logger().warning(
                f"Only {int(arm_static.sum())} settled samples — also hold the arm "
                "STILL at several poses (not only sweeps) so the gain fits cleanly."
            )
        for j in range(n):
            if spans_all[j] >= self._min_span > spans[j]:
                self.get_logger().warning(
                    f"'{self._model_joint_names[j]}': gravity span is {spans_all[j]:.1f} "
                    f"Nm while moving but only {spans[j]:.1f} Nm across the HELD poses "
                    "— hold this joint still at several DIFFERENT gravity loads "
                    "(e.g. elbow folded, half, extended), not just sweep through them."
                )

        # Resolve any fixed gains: one value = every joint, or one per joint (0 =
        # fit that joint from data as usual).
        fixed = np.full(n, np.nan)
        fg = np.asarray(self._fixed_gain, dtype=float)
        if fg.size == 1 and fg[0] > 0:
            fixed[:] = fg[0]
        elif fg.size == n:
            fixed[fg > 0] = fg[fg > 0]
        if np.isfinite(fixed).any():
            self.get_logger().info(
                "Using fixed effort_gain for "
                f"{[self._model_joint_names[j] for j in range(n) if np.isfinite(fixed[j])]}"
                " — only their offset is fit."
            )

        # Stage 1: gain + offset from static samples (gravity == k*I - offset).
        # A fixed gain skips the gain fit and keeps only the offset, which removes
        # the run-to-run variance of estimating a physical constant from noisy data.
        for j in range(n):
            if np.isfinite(fixed[j]):
                effort_gain[j] = fixed[j]
                offset[j] = float(
                    np.mean(effort_gain[j] * currents[stat, j] - gravity[stat, j])
                )
            elif has_gain[j]:
                A = np.stack([currents[stat, j], -ones[stat]], axis=1)
                (k, off), *_ = np.linalg.lstsq(A, gravity[stat, j], rcond=None)
                effort_gain[j], offset[j] = k, off

        # Gain-blind joints (pan, wrist_3): nominal gain, offset from static residual.
        resolved = has_gain | np.isfinite(fixed)
        if self._nominal_gain > 0:
            nominal = self._nominal_gain
        else:
            nominal = float(np.mean(effort_gain[resolved])) if resolved.any() else 1.0
        for j in range(n):
            if resolved[j]:
                continue
            effort_gain[j] = nominal
            offset[j] = float(np.mean(effort_gain[j] * currents[stat, j] - gravity[stat, j]))
            self.get_logger().warning(
                f"'{self._model_joint_names[j]}': gravity span {spans[j]:.2f} Nm "
                f"< {self._min_span} — gain unidentifiable, using nominal "
                f"{nominal:.2f} Nm/A (expected for pan/wrist_3)."
            )

        # Stage 2: Coulomb + viscous from each joint's MOVING, near-CONSTANT-VELOCITY
        # samples (|accel| <= accel_max), gain and offset fixed. Excluding
        # accelerating samples keeps the unmodeled inertia M*a out of the residual,
        # which is then just friction: k*I - model - offset ~ coulomb*sign(v)+viscous*v.
        # If a joint has too few clean samples, its friction stays 0 (safe — better
        # than an inertia-corrupted fit that would over-subtract during motion).
        for j in range(n):
            usable = (np.abs(vs[:, j]) >= self._friction_min_vel) & (
                np.abs(accel[:, j]) <= self._accel_max
            )
            if usable.sum() < 10:
                if vmax[j] >= self._friction_min_vel:
                    self.get_logger().warning(
                        f"'{self._model_joint_names[j]}': only {int(usable.sum())} "
                        "constant-velocity samples — friction left at 0. Move it in "
                        "slow, STEADY sweeps (hold a constant speed) at 2-3 speeds."
                    )
                continue
            r = effort_gain[j] * currents[usable, j] - model[usable, j] - offset[j]
            ss = smooth_sign(vs[usable, j], self._friction_min_vel)
            if self._fit_viscous:
                # tanh(v/eps) and v are near-collinear unless the sweeps cover
                # clearly different speeds, which makes the pair blow up in equal
                # and opposite directions (seen: coulomb +16.4 with viscous -18.3).
                A = np.stack([ss, vs[usable, j]], axis=1)
                (cf, vf), *_ = np.linalg.lstsq(A, r, rcond=None)
            else:
                # Coulomb only: single, well-conditioned parameter.
                cf = float(np.linalg.lstsq(ss[:, None], r, rcond=None)[0][0])
                vf = 0.0
            # Sanity guard: a Coulomb magnitude far above the joint's own gravity
            # scale is not friction, it is a degenerate fit. Drop it rather than
            # publish an estimate that jumps by that much whenever the joint moves.
            limit = max(2.0, 0.5 * float(spans_all[j]))
            if abs(cf) > limit:
                self.get_logger().warning(
                    f"'{self._model_joint_names[j]}': friction fit gave coulomb="
                    f"{cf:+.2f} Nm (limit {limit:.1f}) — implausible, leaving "
                    "friction at 0. Record steadier sweeps at 2-3 distinct speeds."
                )
                continue
            coulomb[j], viscous[j] = cf, vf

        residual = (
            effort_gain * currents
            - model
            - (coulomb * smooth_sign(vs, self._friction_min_vel) + viscous * vs)
            - offset
        )
        rms = np.sqrt((residual**2).mean(axis=0))
        # Static RMS is the at-rest noise floor (what you see standing still) and is
        # the honest measure of gain/offset quality; the overall RMS mixes in motion.
        rms_static = np.sqrt((residual[stat] ** 2).mean(axis=0))
        self.get_logger().info(
            f"{int(arm_static.sum())} static / {len(qs) - int(arm_static.sum())} "
            f"moving samples of {len(qs)}."
        )
        for j, name in enumerate(self._model_joint_names):
            self.get_logger().info(
                f"{name}: gain={effort_gain[j]:+.3f}Nm/A coulomb={coulomb[j]:+.3f}Nm "
                f"viscous={viscous[j]:+.3f} offset={offset[j]:+.3f}Nm "
                f"span_held={spans[j]:.1f}Nm (all {spans_all[j]:.1f}) "
                f"vmax={vmax[j]:.2f} rms={rms[j]:.3f} rms_static={rms_static[j]:.3f}Nm"
            )

        data = {
            "joint_names": list(self._model_joint_names),
            "effort_gain": [round(float(v), 6) for v in effort_gain],
            "offset": [round(float(v), 6) for v in offset],
            "coulomb": [round(float(v), 6) for v in coulomb],
            "viscous": [round(float(v), 6) for v in viscous],
            "gravity_span": [round(float(v), 4) for v in spans],
            "gravity_span_all": [round(float(v), 4) for v in spans_all],
            "velocity_max": [round(float(v), 4) for v in vmax],
            "residual_rms": [round(float(v), 6) for v in rms],
            "residual_rms_static": [round(float(v), 6) for v in rms_static],
            "n_samples": int(len(qs)),
            "n_static": int(arm_static.sum()),
        }
        with open(self._output_file, "w") as f:
            yaml.safe_dump(data, f, sort_keys=False)
        self.get_logger().info(
            f"Calibration written to '{self._output_file}'. Load it with:\n"
            "  ros2 launch crisp_controllers_robot_demos external_effort.launch.py "
            f"calibration_file:={self._output_file}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = CalibrateExternalEffort()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
