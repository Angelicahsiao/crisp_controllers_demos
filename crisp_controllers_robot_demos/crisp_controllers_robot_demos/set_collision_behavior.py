#!/usr/bin/env python3

import sys

import rclpy
from rclpy.node import Node
from franka_msgs.srv import SetFullCollisionBehavior

# Thresholds raised ~40% over the previous set to stop nuisance reflex aborts
# during teleop (jerky streamed deltas, gripper contact). Crossing a LOWER
# threshold only flags a contact; crossing an UPPER one aborts the motion, so
# the upper values are what actually keep the arm running.
#
# Joints 5-7 are rated to ~12 Nm (joints 1-4 to ~87 Nm), so the wrist entries
# below already sit above what those joints can produce and their reflexes
# effectively never trip -- the real tolerance gained here is on joints 1-4 and
# on the Cartesian force thresholds.
#
# A genuine collision still aborts. Do NOT raise these further without a caged
# cell: the arm's ability to stop on contact scales with them.
#
# Nominal and acceleration thresholds are deliberately identical.
TORQUE_LOWER = [35.0, 35.0, 32.0, 30.0, 27.0, 24.0, 20.0]
TORQUE_UPPER = [50.0, 50.0, 45.0, 42.0, 38.0, 34.0, 28.0]
FORCE_LOWER = [45.0, 45.0, 45.0, 35.0, 35.0, 35.0]
FORCE_UPPER = [60.0, 60.0, 60.0, 50.0, 50.0, 50.0]


def validate_thresholds() -> list:
    """Sanity-check the threshold tables before they reach the robot.

    A structurally valid request can still be catastrophic: an all-zero request
    (what `ros2 service call ... SetFullCollisionBehavior` sends when you omit
    the payload) is accepted by the service and makes the robot treat ANY
    external force or torque as a collision, tripping "Configured force/torque
    thresholds reached" immediately. Catch that class of mistake here rather
    than on the hardware.

    Returns:
        A list of human-readable problems. Empty means the tables are sane.
    """
    problems = []

    for name, arr, expected_len in (
        ("TORQUE_LOWER", TORQUE_LOWER, 7),
        ("TORQUE_UPPER", TORQUE_UPPER, 7),
        ("FORCE_LOWER", FORCE_LOWER, 6),
        ("FORCE_UPPER", FORCE_UPPER, 6),
    ):
        if len(arr) != expected_len:
            problems.append(f"{name} has {len(arr)} entries, expected {expected_len}")
        non_positive = [i for i, v in enumerate(arr) if v <= 0.0]
        if non_positive:
            problems.append(
                f"{name} has non-positive values at indices {non_positive}: "
                "a threshold of 0 makes every contact a collision"
            )

    for lower_name, lower, upper_name, upper in (
        ("TORQUE_LOWER", TORQUE_LOWER, "TORQUE_UPPER", TORQUE_UPPER),
        ("FORCE_LOWER", FORCE_LOWER, "FORCE_UPPER", FORCE_UPPER),
    ):
        if len(lower) != len(upper):
            continue  # already reported above
        inverted = [i for i, (lo, hi) in enumerate(zip(lower, upper)) if lo >= hi]
        if inverted:
            problems.append(
                f"{lower_name} >= {upper_name} at indices {inverted}: "
                "the contact threshold must stay below the abort threshold"
            )

    return problems


class CollisionBehaviorSetter(Node):
    def __init__(self):
        super().__init__("collision_behavior_setter")

        self.ok = False

        self.cli = self.create_client(
            SetFullCollisionBehavior, "/service_server/set_full_collision_behavior"
        )
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting again...")
        self.ok = self.send_request()

    def send_request(self) -> bool:
        """Send the collision-behavior request. Returns True only if the robot accepted it."""
        problems = validate_thresholds()
        if problems:
            for problem in problems:
                self.get_logger().error(f"Refusing to send: {problem}")
            return False

        req = SetFullCollisionBehavior.Request()
        self.get_logger().info("Sending request.")

        req.lower_torque_thresholds_nominal = TORQUE_LOWER
        req.upper_torque_thresholds_nominal = TORQUE_UPPER
        req.lower_torque_thresholds_acceleration = TORQUE_LOWER
        req.upper_torque_thresholds_acceleration = TORQUE_UPPER
        req.lower_force_thresholds_nominal = FORCE_LOWER
        req.upper_force_thresholds_nominal = FORCE_UPPER
        req.lower_force_thresholds_acceleration = FORCE_LOWER
        req.upper_force_thresholds_acceleration = FORCE_UPPER

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        if response is None:
            self.get_logger().error("No response from the collision behavior service.")
            return False

        # A reply is NOT the same as acceptance: the old code only checked that
        # future.result() was non-None, so a rejected request still logged
        # success. Inspect the response's own status field.
        success = getattr(response, "success", None)
        if success is None:
            self.get_logger().warning(
                "Response has no 'success' field; treating the reply as acceptance. "
                "Check 'ros2 interface show franka_msgs/srv/SetFullCollisionBehavior'."
            )
            return True

        if not success:
            reason = getattr(response, "error", "") or getattr(response, "message", "")
            self.get_logger().error(
                f"Robot rejected the collision behavior request: {reason or '<no reason given>'}"
            )
            return False

        self.get_logger().info("Collision behavior set successfully")
        return True


def main(args=None):
    rclpy.init(args=args)
    node = CollisionBehaviorSetter()
    ok = node.ok
    node.destroy_node()
    rclpy.shutdown()
    # Non-zero exit so a bring-up script notices a rejected request.
    return 0 if ok else 1

if __name__ == "__main__":
    sys.exit(main())
