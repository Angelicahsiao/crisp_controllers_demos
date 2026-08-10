"""Identify the tool payload mass and center of mass from the wrist F/T sensor.

The external-effort estimator's gravity model g(q) is only as good as the mass
it knows about. The robotiq_description URDF models ~0.36 kg of the 2F-140, but a
real gripper (+ coupling) is ~1 kg, so g(q) under-compensates and the residual
looks like a phantom external force. This node measures the *true* payload from
the UR's built-in wrist F/T sensor so you can put real numbers into the URDF
payload instead of guessing a COM.

Method (classic least-squares payload identification):
    With nothing grasped and the arm quasi-static, everything distal to the
    wrist is a rigid gravitational load. In the sensor frame (tool0):

        f_meas = m * g_tool            + f_bias        (force)
        t_meas = (m * c) x g_tool      + t_bias        (torque about tool0)

    where g_tool = R(base->tool) * [0, 0, -9.81] changes with wrist orientation.
    Recording f_meas/t_meas at several DIVERSE orientations makes both systems
    solvable:
        - forces  -> mass m and a constant force bias f_bias
        - torques -> first moment p = m*c and a torque bias t_bias, then c = p/m
    Sensor bias is absorbed by f_bias/t_bias, so you do NOT need a perfectly
    zeroed sensor - but see the payload note below.

IMPORTANT before recording:
    * Set the robot's configured PAYLOAD (mass, CoG, TCP) to ZERO in PolyScope.
      If a payload is configured, the streamed wrench is partly compensated and
      the identified mass will be wrong (too small by the configured payload).
    * Nothing grasped; keep the gripper fingers at a fixed position.
    * Move to at least ~4-6 clearly different wrist orientations so gravity
      points in different directions in the tool frame (tilt AND roll wrist_2/3).
      The script reports an orientation-diversity number; keep going until it is
      comfortably > 0 and the residual is small.
    * Move SLOWLY and pause at poses: inertial/contact wrench is not modeled.

Usage (UR7e example):

    ros2 run crisp_controllers_robot_demos identify_payload \\
        --ros-args -p wrench_topic:=/force_torque_sensor_broadcaster/wrench \\
        -p base_frame:=base_link -p sensor_frame:=tool0

Parameters:
    wrench_topic (str)   WrenchStamped topic (default
                         "/force_torque_sensor_broadcaster/wrench").
    base_frame (str)     gravity-fixed frame (default "base_link").
    sensor_frame (str)   frame the wrench is expressed in / F/T sensor frame,
                         also the frame the COM is reported in (default "tool0").
    stop_on_key (bool)   stop when ENTER is pressed (default True).
    duration (double)    max recording seconds / safety cap (default 120).
    sample_rate (double) sampling rate in Hz (default 10).
    gravity (double)     |g| in m/s^2 (default 9.80665).
    output_file (str)    YAML path (default "payload_identification.yaml").
"""

import sys
import threading

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener


def _quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Rotation matrix from a (x, y, z, w) quaternion (normalized)."""
    n = np.sqrt(x * x + y * y + z * z + w * w)
    if n == 0.0:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _skew(v: np.ndarray) -> np.ndarray:
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


class IdentifyPayload(Node):
    """Record (orientation, wrench) samples and fit payload mass + COM."""

    def __init__(self):
        super().__init__("identify_payload")

        self._wrench_topic = self.declare_parameter(
            "wrench_topic", "/force_torque_sensor_broadcaster/wrench"
        ).value
        self._base_frame = self.declare_parameter("base_frame", "base_link").value
        self._sensor_frame = self.declare_parameter("sensor_frame", "tool0").value
        self._stop_on_key = self.declare_parameter("stop_on_key", True).value
        self._duration = self.declare_parameter("duration", 120.0).value
        self._sample_rate = self.declare_parameter("sample_rate", 10.0).value
        self._g = self.declare_parameter("gravity", 9.80665).value
        self._output_file = self.declare_parameter(
            "output_file", "payload_identification.yaml"
        ).value

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._last_wrench: WrenchStamped | None = None
        self._g_tool: list[np.ndarray] = []  # gravity dir (unit) in sensor frame
        self._forces: list[np.ndarray] = []
        self._torques: list[np.ndarray] = []
        self._started = False
        self._stop_requested = False
        self.done = False

        self.create_subscription(
            WrenchStamped, self._wrench_topic, self._on_wrench, qos_profile_sensor_data
        )
        self._timer = self.create_timer(1.0 / self._sample_rate, self._sample)
        if self._stop_on_key:
            threading.Thread(target=self._wait_for_key, daemon=True).start()

        self.get_logger().info(
            f"identify_payload: waiting for '{self._wrench_topic}' and TF "
            f"{self._base_frame}->{self._sensor_frame}.\n"
            "  FIRST set the robot payload/CoG/TCP to ZERO in PolyScope, nothing\n"
            "  grasped. Then move SLOWLY through >=4-6 diverse wrist orientations\n"
            "  (tilt + roll), pausing at each. "
            + (
                "Press ENTER to stop and fit.\n"
                if self._stop_on_key
                else f"Recording for {self._duration:.0f}s.\n"
            )
        )

    def _wait_for_key(self) -> None:
        try:
            sys.stdin.readline()
        except Exception:  # noqa: BLE001 (stdin may be closed under some launchers)
            return
        self._stop_requested = True

    def _on_wrench(self, msg: WrenchStamped) -> None:
        self._last_wrench = msg

    def _sample(self) -> None:
        if self.done or self._last_wrench is None:
            return
        try:
            tf = self._tf_buffer.lookup_transform(
                self._sensor_frame, self._base_frame, rclpy.time.Time()
            )
        except Exception:  # noqa: BLE001 (TF may not be available yet)
            self.get_logger().warning(
                f"No TF {self._base_frame}->{self._sensor_frame} yet.",
                throttle_duration_sec=5.0,
            )
            return

        q = tf.transform.rotation
        # R maps base-frame vectors -> sensor-frame vectors (lookup_transform
        # target=sensor, source=base). Gravity is -z in the base frame.
        R = _quat_to_matrix(q.x, q.y, q.z, q.w)
        g_tool = R @ np.array([0.0, 0.0, -1.0])  # unit gravity dir in sensor frame

        w = self._last_wrench.wrench
        f = np.array([w.force.x, w.force.y, w.force.z])
        t = np.array([w.torque.x, w.torque.y, w.torque.z])
        if np.isnan(f).any() or np.isnan(t).any():
            return

        if not self._started:
            self._started = True
            self._start_time = self.get_clock().now()
            self.get_logger().info("Recording started. Move the wrist now.")

        self._g_tool.append(g_tool)
        self._forces.append(f)
        self._torques.append(t)

        n = len(self._forces)
        if n % int(3 * self._sample_rate) == 0:
            self.get_logger().info(
                f"[{n} samples] orientation diversity={self._diversity():.2f} "
                "(keep tilting/rolling until it stops rising)"
            )

        elapsed = (self.get_clock().now() - self._start_time).nanoseconds / 1e9
        if self._stop_requested or elapsed >= self._duration:
            if n < 20 or self._diversity() < 0.3:
                self.get_logger().warning(
                    f"Only {n} samples / diversity {self._diversity():.2f} — move "
                    "through more distinct orientations before stopping."
                )
                self._stop_requested = False
                return
            self._fit_and_save()
            self.done = True

    def _diversity(self) -> float:
        """Smallest singular value of the stacked gravity directions.

        ~0 means all gravity directions are (nearly) collinear -> COM and mass
        are ill-conditioned; > ~0.5 means the orientations span 3D well.
        """
        if len(self._g_tool) < 3:
            return 0.0
        return float(np.linalg.svd(np.stack(self._g_tool), compute_uv=False)[-1])

    def _fit_and_save(self) -> None:
        g_tool = np.stack(self._g_tool) * self._g  # gravity vectors (m/s^2)
        forces = np.stack(self._forces)
        torques = np.stack(self._torques)
        n = len(forces)

        # Force system: f = m * g_tool + f_bias  ->  unknowns [m, f_bias(3)].
        Af = np.zeros((3 * n, 4))
        bf = forces.reshape(-1)
        for i in range(n):
            Af[3 * i : 3 * i + 3, 0] = g_tool[i]
            Af[3 * i : 3 * i + 3, 1:4] = np.eye(3)
        xf, *_ = np.linalg.lstsq(Af, bf, rcond=None)
        m = xf[0]
        f_bias = xf[1:4]

        # Torque system: t = p x g_tool + t_bias = -skew(g_tool) p + t_bias
        #   -> unknowns [p(3), t_bias(3)], with p = m * c.
        At = np.zeros((3 * n, 6))
        bt = torques.reshape(-1)
        for i in range(n):
            At[3 * i : 3 * i + 3, 0:3] = -_skew(g_tool[i])
            At[3 * i : 3 * i + 3, 3:6] = np.eye(3)
        xt, *_ = np.linalg.lstsq(At, bt, rcond=None)
        p = xt[0:3]
        t_bias = xt[3:6]

        # Sign convention of the sensor is unknown; mass is a magnitude and the
        # COM c = p/m is invariant if both f and t flip sign together.
        com = p / m if abs(m) > 1e-9 else np.zeros(3)
        mass = abs(m)

        f_res = (Af @ xf - bf).reshape(n, 3)
        t_res = (At @ xt - bt).reshape(n, 3)
        f_rms = float(np.sqrt((f_res**2).sum(axis=1).mean()))
        t_rms = float(np.sqrt((t_res**2).sum(axis=1).mean()))

        self.get_logger().info(
            "\n=== Payload identification result ({} samples) ===\n"
            "  mass            = {:.4f} kg\n"
            "  COM in {:>8s} = [{:+.4f}, {:+.4f}, {:+.4f}] m\n"
            "  force bias      = [{:+.3f}, {:+.3f}, {:+.3f}] N\n"
            "  torque bias     = [{:+.4f}, {:+.4f}, {:+.4f}] Nm\n"
            "  residual RMS    = {:.3f} N (force), {:.4f} Nm (torque)\n"
            "  orientation diversity = {:.2f}\n".format(
                n, mass, self._sensor_frame, com[0], com[1], com[2],
                f_bias[0], f_bias[1], f_bias[2],
                t_bias[0], t_bias[1], t_bias[2],
                f_rms, t_rms, self._diversity(),
            )
        )
        self.get_logger().info(
            "This is the TOTAL payload (gripper + coupling + fingers) at the\n"
            f"  {self._sensor_frame} frame. The robotiq URDF already models "
            "~0.364 kg, so the\n"
            "  supplement to add in ur_single_robotiq.urdf.xacro is "
            "(mass - 0.364) kg\n"
            "  at this COM. Give me these numbers and I'll wire the xacro."
        )

        data = {
            "mass": round(float(mass), 6),
            "com": [round(float(v), 6) for v in com],
            "com_frame": self._sensor_frame,
            "force_bias": [round(float(v), 6) for v in f_bias],
            "torque_bias": [round(float(v), 6) for v in t_bias],
            "force_residual_rms": round(f_rms, 6),
            "torque_residual_rms": round(t_rms, 6),
            "orientation_diversity": round(self._diversity(), 6),
            "n_samples": int(n),
        }
        with open(self._output_file, "w") as fh:
            yaml.safe_dump(data, fh, sort_keys=False)
        self.get_logger().info(f"Written to '{self._output_file}'.")


def main(args=None):
    rclpy.init(args=args)
    node = IdentifyPayload()
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
