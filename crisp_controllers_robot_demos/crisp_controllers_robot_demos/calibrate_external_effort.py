"""Record contact-free joint effort samples and fit the external-effort calibration.

The UR's current-derived effort has per-joint scale errors and static offsets,
so pure gravity subtraction (scale=1, offset=0) leaves a pose-dependent
residual. This script records (q, tau_measured) while the arm moves slowly
with NOTHING touching it, fits tau_measured ~ scale * g(q) + offset per joint
(ExternalEffortEstimator.fit_calibration), and writes the result to a YAML
that external_effort_node loads via its 'calibration_file' parameter.

Usage (move the arm through diverse poses during recording, e.g. freedrive or
slow teleop — the more the shoulder/elbow travel, the better the fit):

    ros2 run crisp_controllers_robot_demos calibrate_external_effort \\
        --ros-args -p joint_names:="[shoulder_pan_joint, shoulder_lift_joint, \\
        elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint]" \\
        -p duration:=30.0 -p output_file:=external_effort_calibration.yaml

Parameters:
    joint_names (string[])   REQUIRED — actuated arm joints, in order.
    duration (double)        recording time in seconds (default 30).
    sample_rate (double)     sampling rate in Hz (default 20).
    output_file (str)        YAML path (default external_effort_calibration.yaml).
    joint_state_topic (str)  default "joint_states".
"""

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from crisp_controllers_robot_demos.external_effort import ExternalEffortEstimator


class CalibrateExternalEffort(Node):
    """Record contact-free (q, tau) samples and fit scale/offset per joint."""

    def __init__(self):
        super().__init__("calibrate_external_effort")

        prefix = self.get_namespace().strip("/")
        self._prefix = prefix + "_" if prefix else ""

        self._joint_names = [
            j for j in self.declare_parameter("joint_names", [""]).value if j
        ]
        if not self._joint_names:
            raise RuntimeError("calibrate_external_effort requires 'joint_names'.")
        self._duration = self.declare_parameter("duration", 30.0).value
        self._sample_rate = self.declare_parameter("sample_rate", 20.0).value
        self._output_file = self.declare_parameter(
            "output_file", "external_effort_calibration.yaml"
        ).value
        joint_state_topic = self.declare_parameter(
            "joint_state_topic", "joint_states"
        ).value

        self._urdf: str | None = None
        self._last_msg: JointState | None = None
        self._msg_index: list[int] | None = None
        self._model_joint_names: list[str] | None = None
        self._qs: list[np.ndarray] = []
        self._taus: list[np.ndarray] = []
        self._started = False
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

        self.get_logger().info(
            f"Waiting for /robot_description and '{joint_state_topic}'... "
            f"Will record {self._duration:.0f}s of contact-free motion. "
            "Move the arm slowly through diverse poses; do NOT touch the arm "
            "with anything other than the leader/freedrive."
        )

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
            self._n_samples = int(self._duration * self._sample_rate)
            self.get_logger().info(
                f"Recording started: {self._n_samples} samples at "
                f"{self._sample_rate:.0f} Hz. Move the arm now."
            )

        self._qs.append(np.array([msg.position[i] for i in self._msg_index]))
        self._taus.append(np.array([msg.effort[i] for i in self._msg_index]))
        n = len(self._qs)
        if n % int(5 * self._sample_rate) == 0:
            self.get_logger().info(f"{n}/{self._n_samples} samples...")
        if n >= self._n_samples:
            self._fit_and_save()
            self.done = True

    def _fit_and_save(self) -> None:
        estimator = ExternalEffortEstimator(self._urdf, self._model_joint_names)
        qs = np.stack(self._qs)
        taus = np.stack(self._taus)

        # Warn if a joint barely changed its gravity torque during recording —
        # its scale fit is then poorly conditioned (offset still fine).
        gravity = np.stack([estimator.gravity_effort(q) for q in qs])
        for j, name in enumerate(self._model_joint_names):
            span = float(np.ptp(gravity[:, j]))
            if span < 1.0:
                self.get_logger().warning(
                    f"'{name}': gravity torque only spanned {span:.2f} Nm during "
                    "recording — move it through more diverse poses for a "
                    "better scale fit."
                )

        scale, offset = estimator.fit_calibration(qs, taus)
        residual = taus - (scale * gravity + offset)
        rms = np.sqrt((residual**2).mean(axis=0))

        for j, name in enumerate(self._model_joint_names):
            self.get_logger().info(
                f"{name}: scale={scale[j]:+.3f} offset={offset[j]:+.3f} "
                f"residual_rms={rms[j]:.3f} Nm"
            )

        data = {
            "joint_names": list(self._model_joint_names),
            "scale": [round(float(v), 6) for v in scale],
            "offset": [round(float(v), 6) for v in offset],
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
