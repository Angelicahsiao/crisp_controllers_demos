"""ROS node publishing gravity-free external joint effort.

Subscribes the latched /robot_description and /joint_states, builds a Pinocchio
gravity model once, and publishes tau_ext = tau_measured - g(q) as a
std_msgs/Float32MultiArray. crisp_gym records it via a plain float32_array
sensor (no pinocchio dependency on the crisp_gym side).

Parameters:
    joint_names (string[])   REQUIRED — actuated arm joints, in order.
    joint_state_topic (str)  default "joint_states".
    output_topic (str)       default "external_joint_effort".
    scale (double[])         optional per-joint calibration gain (default all 1).
    offset (double[])        optional per-joint calibration offset (default all 0).

Namespaced joint names: a node namespace prefix (e.g. "right") is prepended to
each configured joint name when matching /joint_states, mirroring crisp_py.

Example:
    ros2 run crisp_controllers_robot_demos external_effort_node \\
        --ros-args -p joint_names:="[shoulder_pan_joint, shoulder_lift_joint, \\
        elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint]"
"""

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, String


class ExternalEffortNode(Node):
    """Publish external (gravity-compensated) joint effort from /joint_states."""

    def __init__(self):
        super().__init__("external_effort_node")

        prefix = self.get_namespace().strip("/")
        self._prefix = prefix + "_" if prefix else ""

        self._joint_names = self.declare_parameter(
            "joint_names",
            [""],
            descriptor=ParameterDescriptor(
                description="Actuated arm joints (in order) to estimate effort for."
            ),
        ).value
        self._joint_names = [j for j in self._joint_names if j]
        if not self._joint_names:
            raise RuntimeError(
                "external_effort_node requires the 'joint_names' parameter "
                "(the arm joints, in order)."
            )

        joint_state_topic = self.declare_parameter("joint_state_topic", "joint_states").value
        output_topic = self.declare_parameter("output_topic", "external_joint_effort").value

        n = len(self._joint_names)
        scale = list(self.declare_parameter("scale", [1.0] * n).value)
        offset = list(self.declare_parameter("offset", [0.0] * n).value)

        # A calibration YAML (written by calibrate_external_effort) overrides
        # the scale/offset parameters.
        calibration_file = self.declare_parameter("calibration_file", "").value
        if calibration_file:
            import yaml

            with open(calibration_file) as f:
                calib = yaml.safe_load(f)
            scale = list(calib["scale"])
            offset = list(calib["offset"])
            calib_joints = calib.get("joint_names")
            if calib_joints is not None and [
                j.removeprefix(self._prefix) for j in calib_joints
            ] != list(self._joint_names):
                raise RuntimeError(
                    f"calibration_file '{calibration_file}' was fitted for joints "
                    f"{calib_joints}, but this node is configured for "
                    f"{self._joint_names}."
                )
            self.get_logger().info(
                f"Loaded calibration from '{calibration_file}' "
                f"({calib.get('n_samples', '?')} samples)."
            )

        for name, arr in (("scale", scale), ("offset", offset)):
            if len(arr) != n:
                raise RuntimeError(f"'{name}' has {len(arr)} values but joint_names has {n}.")
        self._scale = np.asarray(scale, dtype=float)
        self._offset = np.asarray(offset, dtype=float)

        # matched joint index in the JointState message (filled on first msg)
        self._msg_index: list[int] | None = None
        self._estimator = None
        self._urdf: str | None = None

        # latched robot description
        self.create_subscription(
            String,
            "robot_description",
            self._on_urdf,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.create_subscription(
            JointState, joint_state_topic, self._on_joint_state, qos_profile_sensor_data
        )
        self._publisher = self.create_publisher(Float32MultiArray, output_topic, 10)

        self.get_logger().info(
            f"external_effort_node: {n} joints, publishing on '{output_topic}' "
            f"from '{joint_state_topic}' (waiting for /robot_description)."
        )

    def _on_urdf(self, msg: String) -> None:
        self._urdf = msg.data

    def _build_estimator(self) -> bool:
        if self._urdf is None:
            self.get_logger().warning(
                "No /robot_description yet — is the robot bring-up publishing it?",
                throttle_duration_sec=5.0,
            )
            return False
        from crisp_controllers_robot_demos.external_effort import ExternalEffortEstimator

        prefixed = [self._prefix + n for n in self._joint_names]
        try:
            self._estimator = ExternalEffortEstimator(
                self._urdf, prefixed, scale=self._scale, offset=self._offset
            )
        except ValueError:
            # namespace prefix may already be baked into the URDF joint names
            self._estimator = ExternalEffortEstimator(
                self._urdf, list(self._joint_names), scale=self._scale, offset=self._offset
            )
            prefixed = list(self._joint_names)
        self._model_joint_names = prefixed
        self.get_logger().info("external_effort_node: Pinocchio model built.")
        return True

    def _on_joint_state(self, msg: JointState) -> None:
        if self._estimator is None and not self._build_estimator():
            return

        if self._msg_index is None:
            name_to_i = {name: i for i, name in enumerate(msg.name)}
            missing = [n for n in self._model_joint_names if n not in name_to_i]
            if missing:
                self.get_logger().warning(
                    f"JointState is missing joints {missing}; cannot estimate effort.",
                    throttle_duration_sec=5.0,
                )
                return
            self._msg_index = [name_to_i[n] for n in self._model_joint_names]

        if len(msg.effort) <= max(self._msg_index):
            self.get_logger().warning(
                "JointState has no effort field — the UR/robot must publish "
                "current-derived effort for external-effort estimation.",
                throttle_duration_sec=5.0,
            )
            return

        q = np.array([msg.position[i] for i in self._msg_index], dtype=float)
        tau = np.array([msg.effort[i] for i in self._msg_index], dtype=float)
        tau_ext = self._estimator.external_effort(q, tau)

        out = Float32MultiArray()
        out.data = [float(v) for v in tau_ext]
        self._publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ExternalEffortNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
