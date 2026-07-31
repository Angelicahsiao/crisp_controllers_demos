"""ROS node publishing gravity-free external joint effort.

Subscribes the latched /robot_description and /joint_states, builds a Pinocchio
model once, and publishes
    tau_ext = effort_gain * I - rnea(q, v, 0) - friction(v) - offset
as a std_msgs/Float32MultiArray (I = /joint_states effort = motor current on the
UR; v = /joint_states velocity). rnea(q,v,0) is gravity + Coriolis; friction(v) =
coulomb*sign(v) + viscous*v. crisp_gym records it via a plain float32_array
sensor (no pinocchio dependency on the crisp_gym side).

Parameters:
    joint_names (string[])   REQUIRED — actuated arm joints, in order.
    joint_state_topic (str)  default "joint_states".
    output_topic (str)       default "external_joint_effort".
    effort_gain (double[])   per-joint current->torque gain k, Nm/A (default all
                             1 — calibrate first, 1 gives meaningless output).
    offset (double[])        per-joint offset, Nm (default all 0).
    coulomb (double[])       per-joint Coulomb friction, Nm (default all 0).
    viscous (double[])       per-joint viscous friction, Nm/(rad/s) (default 0).
    use_coulomb (bool)       apply the Coulomb term (default True).
    use_viscous (bool)       apply the viscous term (default False — often poorly
                             identified).
    method (str)             "gravity" (quasi-static) or "momentum" (generalized
                             momentum observer; handles inertia during motion).
    observer_gain (double)   momentum-observer bandwidth K_O, rad/s (default 20).

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
        # effort_gain (k, Nm/A): per-joint current->torque gain; 1.0 gives
        # meaningless output because /joint_states effort is current, not torque.
        effort_gain = list(self.declare_parameter("effort_gain", [1.0] * n).value)
        offset = list(self.declare_parameter("offset", [0.0] * n).value)
        # Coulomb + viscous joint friction (default 0 = no friction compensation).
        coulomb = list(self.declare_parameter("coulomb", [0.0] * n).value)
        viscous = list(self.declare_parameter("viscous", [0.0] * n).value)

        # A calibration YAML (written by calibrate_external_effort) overrides
        # the effort_gain/offset/coulomb/viscous parameters.
        calibration_file = self.declare_parameter("calibration_file", "").value
        if calibration_file:
            import yaml

            with open(calibration_file) as f:
                calib = yaml.safe_load(f)
            if "effort_gain" not in calib:
                raise RuntimeError(
                    f"calibration_file '{calibration_file}' has no 'effort_gain' "
                    "key. It is an old scale-based calibration; re-run "
                    "calibrate_external_effort to produce the current->torque gain."
                )
            effort_gain = list(calib["effort_gain"])
            offset = list(calib["offset"])
            # Friction fields are optional (older calibrations have no friction).
            coulomb = list(calib.get("coulomb", [0.0] * n))
            viscous = list(calib.get("viscous", [0.0] * n))
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

        for name, arr in (
            ("effort_gain", effort_gain),
            ("offset", offset),
            ("coulomb", coulomb),
            ("viscous", viscous),
        ):
            if len(arr) != n:
                raise RuntimeError(f"'{name}' has {len(arr)} values but joint_names has {n}.")
        self._effort_gain = np.asarray(effort_gain, dtype=float)
        self._offset = np.asarray(offset, dtype=float)
        self._coulomb = np.asarray(coulomb, dtype=float)
        self._viscous = np.asarray(viscous, dtype=float)

        # Friction toggles (viscous is often poorly identified, so default off).
        if not self.declare_parameter("use_coulomb", True).value:
            self._coulomb = np.zeros(n)
        if not self.declare_parameter("use_viscous", False).value:
            self._viscous = np.zeros(n)

        # Estimation method: "gravity" (quasi-static tau=gain*I-rnea(q,v,0)-...) or
        # "momentum" (generalized momentum observer; handles inertia during motion).
        self._method = self.declare_parameter("method", "gravity").value
        self._observer_gain = float(self.declare_parameter("observer_gain", 20.0).value)
        # Velocity scale over which Coulomb friction ramps up; must match the value
        # used at calibration time (calibrate uses friction_min_vel, default 0.05).
        self._friction_eps = float(self.declare_parameter("friction_eps", 0.05).value)
        self._last_stamp: float | None = None

        if not calibration_file and np.allclose(self._effort_gain, 1.0):
            self.get_logger().warning(
                "Running UNCALIBRATED (effort_gain = 1). /joint_states effort is "
                "motor current, not torque, so the output is meaningless until you "
                "run calibrate_external_effort and load its YAML (auto-loaded from "
                "config/ur/external_effort_calibration.yaml)."
            )

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

        kwargs = dict(
            effort_gain=self._effort_gain,
            offset=self._offset,
            coulomb=self._coulomb,
            viscous=self._viscous,
            friction_eps=self._friction_eps,
        )
        if self._method == "momentum":
            from crisp_controllers_robot_demos.momentum_observer import MomentumObserver

            def build(names):
                return MomentumObserver(
                    self._urdf, names, observer_gain=self._observer_gain, **kwargs
                )
        else:
            def build(names):
                return ExternalEffortEstimator(self._urdf, names, **kwargs)

        prefixed = [self._prefix + n for n in self._joint_names]
        try:
            self._estimator = build(prefixed)
        except ValueError:
            # namespace prefix may already be baked into the URDF joint names
            self._estimator = build(list(self._joint_names))
            prefixed = list(self._joint_names)
        self._model_joint_names = prefixed
        self.get_logger().info(
            f"external_effort_node: Pinocchio model built ('{self._method}' method)."
        )
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
        # Velocity drives the Coriolis + friction terms; if the message omits it
        # (or is too short), fall back to zeros (quasi-static / gravity-only).
        if len(msg.velocity) > max(self._msg_index):
            v = np.array([msg.velocity[i] for i in self._msg_index], dtype=float)
        else:
            v = np.zeros_like(q)

        if self._method == "momentum":
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            dt = 0.0 if self._last_stamp is None else stamp - self._last_stamp
            # Reset on a gap or clock jump so the integral does not blow up.
            if dt < 0.0 or dt > 0.5:
                self._estimator.reset()
                dt = 0.0
            self._last_stamp = stamp
            tau_ext = self._estimator.update(q, v, tau, dt)
        else:
            tau_ext = self._estimator.external_effort(q, v, tau)

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
