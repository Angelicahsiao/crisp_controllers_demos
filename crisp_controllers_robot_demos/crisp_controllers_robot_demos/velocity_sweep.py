"""Drive each joint through constant-velocity sweeps for friction identification.

Friction (Coulomb/viscous) can only be identified from samples where a joint moves
at a steady speed: while it accelerates, the unmodelled inertia M(q)*a swamps the
~1 Nm friction. Hand-guiding is far too jerky — in practice the calibration's
constant-velocity filter rejected all but 0-9 samples per joint. This script drives
the motion instead, so every sweep is a clean constant-velocity segment.

For each selected joint, and each speed, it oscillates the joint around its
starting position: + until it has travelled `amplitude` rad, - until `amplitude`
the other way, then back to start. Commands are ramped over `ramp_time` so the
robot is not jerked; the ramps are short and get filtered out by the calibration's
accel_max gate, leaving the steady middle of each segment.

RUN THE CALIBRATION AT THE SAME TIME (it just records /joint_states):

    ros2 run crisp_controllers_robot_demos calibrate_external_effort \\
        --ros-args -p joint_names:="[...]" -p fixed_gain:="[11.1]" &
    ros2 run crisp_controllers_robot_demos velocity_sweep

SAFETY
    * The arm moves UNDER POWER. Clear the workspace and keep the e-stop in reach.
    * Start with --dry-run (default true): it prints the exact plan and validates
      joint limits WITHOUT switching controllers or sending any command.
    * Velocities are clamped to `max_speed`; each segment also has a timeout, and
      motion stops if a joint leaves [`q_min`, `q_max`].
    * Zeros are published and the controller is deactivated on exit, including on
      Ctrl-C and on any exception.
    * Activating the velocity controller DEACTIVATES the effort controllers
      (cartesian/joint impedance, gravity compensation) — a joint can only be
      claimed by one command interface at a time. They are restored on exit.

Parameters:
    joint_names (string[])   arm joints in command order (must match the
                             forward_velocity_controller's `joints` parameter).
    sweep_joints (string[])  which joints to sweep ([] = all of joint_names).
    speeds (double[])        constant speeds to hold, rad/s (default 0.1/0.2/0.35).
    amplitude (double)       travel each way from the start pose, rad (default 0.5).
    ramp_time (double)       s to ramp between 0 and the target speed (default 0.4).
    max_speed (double)       hard clamp on any commanded speed (default 0.5).
    q_min / q_max (double[]) per-joint position bounds, rad; empty = start +- 1.2*amplitude.
    dry_run (bool)           print the plan and exit without moving (DEFAULT True).
    controller (str)         velocity controller name.
    deactivate (string[])    controllers to stand down while sweeping.
"""

import sys
import time

import rclpy
from controller_manager_msgs.srv import SwitchController
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

DEFAULT_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
DEFAULT_DEACTIVATE = [
    "cartesian_impedance_controller",
    "joint_impedance_controller",
    "gravity_compensation",
    "joint_trajectory_controller",
]


class VelocitySweep(Node):
    """Oscillate joints at constant velocity so friction can be identified."""

    def __init__(self):
        super().__init__("velocity_sweep")

        self._joints = list(self.declare_parameter("joint_names", DEFAULT_JOINTS).value)
        sweep = [j for j in self.declare_parameter("sweep_joints", [""]).value if j]
        self._sweep_joints = sweep or list(self._joints)
        self._speeds = list(self.declare_parameter("speeds", [0.1, 0.2, 0.35]).value)
        self._amplitude = float(self.declare_parameter("amplitude", 0.5).value)
        self._ramp_time = float(self.declare_parameter("ramp_time", 0.4).value)
        self._max_speed = float(self.declare_parameter("max_speed", 0.5).value)
        self._dry_run = bool(self.declare_parameter("dry_run", True).value)
        self._rate = float(self.declare_parameter("command_rate", 100.0).value)
        self._controller = self.declare_parameter(
            "controller", "forward_velocity_controller"
        ).value
        self._deactivate = [
            c for c in self.declare_parameter("deactivate", DEFAULT_DEACTIVATE).value if c
        ]
        self._q_min = list(self.declare_parameter("q_min", [0.0]).value)
        self._q_max = list(self.declare_parameter("q_max", [0.0]).value)

        unknown = [j for j in self._sweep_joints if j not in self._joints]
        if unknown:
            raise RuntimeError(f"sweep_joints {unknown} are not in joint_names.")
        if any(abs(s) > self._max_speed for s in self._speeds):
            raise RuntimeError(
                f"speeds {self._speeds} exceed max_speed {self._max_speed} rad/s."
            )
        if self._amplitude <= 0.0:
            raise RuntimeError("amplitude must be > 0.")

        self._positions: dict[str, float] | None = None
        self._abort = False
        self.create_subscription(
            JointState, "joint_states", self._on_joint_state, qos_profile_sensor_data
        )
        self._pub = self.create_publisher(
            Float64MultiArray, f"/{self._controller}/commands", 10
        )
        self._switch = self.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )

    def _on_joint_state(self, msg: JointState) -> None:
        self._positions = dict(zip(msg.name, msg.position))

    # ---------------------------------------------------------------- helpers

    def _wait_for_state(self, timeout: float = 10.0) -> bool:
        end = time.time() + timeout
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._positions is not None and all(
                j in self._positions for j in self._joints
            ):
                return True
        return False

    def _bounds(self, joint: str, q0: float) -> tuple[float, float]:
        """Position bounds for a joint: explicit params, else start +- 1.2*amplitude."""
        i = self._joints.index(joint)
        margin = 1.2 * self._amplitude
        lo, hi = q0 - margin, q0 + margin
        if len(self._q_min) == len(self._joints) and self._q_min[i] != 0.0:
            lo = max(lo, self._q_min[i])
        if len(self._q_max) == len(self._joints) and self._q_max[i] != 0.0:
            hi = min(hi, self._q_max[i])
        return lo, hi

    def _publish(self, joint: str | None, value: float) -> None:
        cmd = Float64MultiArray()
        data = [0.0] * len(self._joints)
        if joint is not None:
            data[self._joints.index(joint)] = float(value)
        cmd.data = data
        self._pub.publish(cmd)

    def _switch_controllers(self, activate: list[str], deactivate: list[str]) -> bool:
        if not self._switch.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("/controller_manager/switch_controller unavailable.")
            return False
        req = SwitchController.Request()
        req.activate_controllers = activate
        req.deactivate_controllers = deactivate
        req.strictness = SwitchController.Request.BEST_EFFORT
        future = self._switch.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        ok = future.result() is not None and future.result().ok
        if not ok:
            self.get_logger().error(
                f"switch_controller failed (activate={activate}, deactivate={deactivate})."
            )
        return ok

    # ------------------------------------------------------------------ sweep

    def _segment(self, joint: str, speed: float, target: float, lo: float, hi: float) -> bool:
        """Ramp to `speed` and hold until `target` is passed. False if aborted."""
        idx_sign = 1.0 if speed > 0 else -1.0
        period = 1.0 / self._rate
        timeout = time.time() + abs(2.5 * self._amplitude / max(abs(speed), 1e-3)) + 5.0
        ramp_start = time.time()
        while rclpy.ok() and not self._abort:
            rclpy.spin_once(self, timeout_sec=0.0)
            q = self._positions.get(joint) if self._positions else None
            if q is None:
                self._publish(None, 0.0)
                self.get_logger().error(f"lost /joint_states for '{joint}' — stopping.")
                return False
            if q < lo or q > hi:
                self.get_logger().warning(
                    f"'{joint}' at {q:+.3f} rad left bounds [{lo:+.3f}, {hi:+.3f}] — "
                    "stopping this segment."
                )
                break
            if (idx_sign > 0 and q >= target) or (idx_sign < 0 and q <= target):
                break
            if time.time() > timeout:
                self.get_logger().warning(f"'{joint}' segment timed out — stopping.")
                break
            # Ramp in so the start of the segment is not a jerk.
            frac = min(1.0, (time.time() - ramp_start) / max(self._ramp_time, 1e-3))
            self._publish(joint, speed * frac)
            time.sleep(period)

        # Ramp out to zero.
        steps = max(1, int(self._ramp_time * self._rate))
        for i in range(steps, -1, -1):
            self._publish(joint, speed * i / steps)
            time.sleep(period)
        self._publish(None, 0.0)
        return not self._abort

    def _sweep_joint(self, joint: str) -> bool:
        q0 = self._positions[joint]
        lo, hi = self._bounds(joint, q0)
        self.get_logger().info(
            f"--- {joint}: start {q0:+.3f} rad, bounds [{lo:+.3f}, {hi:+.3f}] ---"
        )
        for speed in self._speeds:
            speed = min(abs(speed), self._max_speed)
            self.get_logger().info(f"  {joint} @ {speed:.2f} rad/s (+ then -)")
            if not self._segment(joint, +speed, q0 + self._amplitude, lo, hi):
                return False
            if not self._segment(joint, -speed, q0 - self._amplitude, lo, hi):
                return False
            # return to the starting pose before the next speed
            if not self._segment(joint, +speed, q0, lo, hi):
                return False
        return True

    def _plan(self) -> str:
        lines = [
            f"Velocity sweep plan ({'DRY RUN — nothing will move' if self._dry_run else 'LIVE'}):",
            f"  controller : {self._controller}",
            f"  deactivate : {self._deactivate}",
            f"  speeds     : {self._speeds} rad/s (clamped to {self._max_speed})",
            f"  amplitude  : +-{self._amplitude} rad about the start pose",
            f"  ramp/rate  : {self._ramp_time}s ramp, {self._rate} Hz commands",
            f"  joints     : {self._sweep_joints}",
        ]
        if self._positions:
            for j in self._sweep_joints:
                q0 = self._positions.get(j)
                if q0 is None:
                    lines.append(f"    {j}: NOT in /joint_states!")
                    continue
                lo, hi = self._bounds(j, q0)
                lines.append(
                    f"    {j}: start {q0:+.3f} -> sweeps {q0 - self._amplitude:+.3f} .. "
                    f"{q0 + self._amplitude:+.3f} (bounds [{lo:+.3f}, {hi:+.3f}])"
                )
        else:
            lines.append("    (no /joint_states yet — cannot validate ranges)")
        per_joint = sum(3 * (2 * self._amplitude / max(s, 1e-3)) for s in self._speeds)
        lines.append(
            f"  estimated duration ~{per_joint * len(self._sweep_joints) / 60.0:.1f} min"
        )
        return "\n".join(lines)

    def run(self) -> int:
        have_state = self._wait_for_state(timeout=5.0 if self._dry_run else 15.0)
        self.get_logger().info(self._plan())

        if self._dry_run:
            self.get_logger().info(
                "DRY RUN complete — no controller switched, no command sent. "
                "Re-run with -p dry_run:=false to move the robot."
            )
            return 0
        if not have_state:
            self.get_logger().error("No /joint_states — refusing to move.")
            return 1

        if not self._switch_controllers([self._controller], self._deactivate):
            return 1
        self.get_logger().info(
            f"'{self._controller}' active. Sweeping — keep clear, e-stop ready."
        )
        ok = True
        try:
            for joint in self._sweep_joints:
                if not self._sweep_joint(joint):
                    ok = False
                    break
        except KeyboardInterrupt:
            self.get_logger().warning("Interrupted — stopping.")
            ok = False
        finally:
            # Always zero the command and hand the joints back.
            for _ in range(5):
                self._publish(None, 0.0)
                time.sleep(0.01)
            self._switch_controllers(self._deactivate, [self._controller])
            self.get_logger().info("Commands zeroed, controllers restored.")
        return 0 if ok else 1


def main(args=None):
    rclpy.init(args=args)
    node = VelocitySweep()
    try:
        code = node.run()
    except KeyboardInterrupt:
        code = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
