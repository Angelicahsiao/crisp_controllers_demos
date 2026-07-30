"""Record contact-free joint effort samples and fit the external-effort calibration.

The UR's /joint_states "effort" is motor CURRENT (Amps), not torque, so we fit a
per-joint current->torque gain k (Nm/A) and a constant offset such that,
contact-free, k*I - g(q) - offset == 0. This script records (q, I) while the arm
moves slowly with NOTHING touching it and per joint least-squares fits

    g(q)_j ~ k_j * I_j + c_j   ->   effort_gain_j = k_j,  offset_j = -c_j

g(q) keeps coefficient 1 (the RNEA gravity is physically correct); the unit
conversion lives on the current, where it belongs. For g(q) to be accurate the
URDF must carry the true masses, including the tool payload (see identify_payload).
The result is written to a YAML that external_effort_node loads via its
'calibration_file' parameter.

Samples are only taken when the arm is SETTLED (all recorded joints below
'vel_threshold'): the fit assumes each sample is a static holding point
(current = gravity/k + stiction); sampling mid-motion injects acceleration and
kinetic friction and corrupts the gain. So MOVE, then PAUSE and dwell a moment at
each pose. Recording stops when you press ENTER (default) or after 'duration'
seconds. Identifying k needs gravity to LOAD each joint differently across the
poses, so cover each joint's gravity range (watch the live per-joint "span"):
    - shoulder_lift : raise/lower the whole arm, horizontal -> up -> down.
    - elbow         : fully fold and fully extend the elbow.
    - wrist_1       : pitch the wrist up and down.
    - wrist_2       : ROLL the wrist so its axis tilts between vertical and
                      horizontal (otherwise its gravity span stays ~0).
    - shoulder_pan and wrist_3 rotate about near-vertical axes: gravity barely
      loads them in ANY pose, so their k is physically unidentifiable. The script
      detects this (span below --min_span) and falls back to the mean of the
      identified gains, fitting only their offset. This is expected, not an error.

Usage (UR7e example):

    ros2 run crisp_controllers_robot_demos calibrate_external_effort \\
        --ros-args -p joint_names:="[shoulder_pan_joint, shoulder_lift_joint, \\
        elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint]" \\
        -p output_file:=external_effort_calibration.yaml

Parameters:
    joint_names (string[])   REQUIRED — actuated arm joints, in order.
    stop_on_key (bool)       stop when ENTER is pressed (default True).
    duration (double)        max recording seconds / safety cap (default 120).
    sample_rate (double)     sampling rate in Hz (default 5).
    vel_threshold (double)   max |joint velocity| (rad/s) to count as settled;
                             samples are only recorded below it (default 0.02).
    min_span (double)        gravity span (Nm) below which a joint's gain is
                             unidentifiable, so it uses the nominal gain and only
                             its offset is fit (default 1).
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
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from crisp_controllers_robot_demos.external_effort import ExternalEffortEstimator

# Default calibration path — external_effort.launch.py auto-loads this same file,
# so calibrating with the default output makes the launch pick it up with no args.
DEFAULT_CALIBRATION = os.path.join(
    get_package_share_directory("crisp_controllers_robot_demos"),
    "config",
    "ur",
    "external_effort_calibration.yaml",
)


class CalibrateExternalEffort(Node):
    """Record contact-free (q, I) samples and fit effort_gain/offset per joint."""

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
        # Only record when the arm is settled: the gain fit assumes each sample is
        # a STATIC holding point (current = gravity/k + stiction). Sampling while
        # moving injects acceleration + kinetic friction and corrupts the gain.
        self._vel_threshold = self.declare_parameter("vel_threshold", 0.02).value
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
        self._taus: list[np.ndarray] = []
        self._gravity: list[np.ndarray] = []
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
            "  MOVE then PAUSE: samples are only taken when the arm is settled, so\n"
            "  dwell a second or two at each pose. Nothing touching the arm.\n"
            "  Cover diverse poses across each joint's gravity range:\n"
            "    shoulder_lift: raise/lower the arm | elbow: fold/extend\n"
            "    wrist_1: pitch up/down | wrist_2: ROLL so its axis tilts\n"
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
        if np.isnan(tau).any() or np.isnan(q).any():
            bad = [
                self._model_joint_names[j]
                for j in range(len(tau))
                if np.isnan(tau[j]) or np.isnan(q[j])
            ]
            self.get_logger().warning(
                f"Skipping sample: NaN effort/position on {bad}. This arm joint "
                "is not reporting a valid value — calibration cannot proceed "
                "until the driver publishes it.",
                throttle_duration_sec=5.0,
            )
            return  # a NaN on any recorded joint would poison the fit

        # Only record STATIC holding points: if any recorded joint is moving, the
        # current is dominated by acceleration + kinetic friction, not gravity, and
        # corrupts the gain fit. Move the arm, then PAUSE and let it settle.
        if len(msg.velocity) > max(self._msg_index):
            vel = np.array([msg.velocity[i] for i in self._msg_index])
            if np.max(np.abs(vel)) > self._vel_threshold:
                self.get_logger().info(
                    "  ...moving, waiting for the arm to settle before sampling.",
                    throttle_duration_sec=2.0,
                )
                return

        self._qs.append(q)
        self._taus.append(tau)
        self._gravity.append(self._estimator.gravity_effort(q))

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
        currents = np.stack(self._taus)  # /joint_states effort is motor current (A)
        gravity = np.stack(self._gravity)
        spans = np.ptp(gravity, axis=0)

        n = len(self._model_joint_names)
        # /joint_states effort is CURRENT, so we fit a per-joint current->torque
        # gain k (Nm/A) plus a constant offset such that, contact-free,
        #     k*I - g(q) - offset == 0.
        # Per joint that means g ~ k*I + c (least squares), with offset = -c.
        # g(q) keeps coefficient 1 — the unit conversion lives on the current.
        effort_gain = np.ones(n)
        offset = np.zeros(n)
        identifiable = spans >= self._min_span
        for j in range(n):
            if identifiable[j]:
                A = np.stack([currents[:, j], np.ones(len(qs))], axis=1)
                (k, c), *_ = np.linalg.lstsq(A, gravity[:, j], rcond=None)
                effort_gain[j], offset[j] = k, -c

        # Near-vertical joints (shoulder_pan, wrist_3) are barely gravity-loaded,
        # so k is unidentifiable. Fall back to the mean of the identified gains
        # (keeps every joint's output in ~Nm) and fit only the offset.
        nominal = float(np.mean(effort_gain[identifiable])) if identifiable.any() else 1.0
        for j in range(n):
            if not identifiable[j]:
                effort_gain[j] = nominal
                offset[j] = float(np.mean(effort_gain[j] * currents[:, j] - gravity[:, j]))
                self.get_logger().warning(
                    f"'{self._model_joint_names[j]}': gravity span {spans[j]:.2f} Nm "
                    f"< {self._min_span} — gain unidentifiable, using nominal "
                    f"{nominal:.2f} Nm/A (expected for pan/wrist_3, or move this "
                    "joint through more gravity)."
                )

        residual = effort_gain * currents - gravity - offset
        rms = np.sqrt((residual**2).mean(axis=0))
        for j, name in enumerate(self._model_joint_names):
            self.get_logger().info(
                f"{name}: gain={effort_gain[j]:+.3f}Nm/A offset={offset[j]:+.3f}Nm "
                f"span={spans[j]:.2f}Nm residual_rms={rms[j]:.3f}Nm"
            )

        data = {
            "joint_names": list(self._model_joint_names),
            "effort_gain": [round(float(v), 6) for v in effort_gain],
            "offset": [round(float(v), 6) for v in offset],
            "gravity_span": [round(float(v), 4) for v in spans],
            "residual_rms": [round(float(v), 6) for v in rms],
            "n_samples": int(len(qs)),
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
