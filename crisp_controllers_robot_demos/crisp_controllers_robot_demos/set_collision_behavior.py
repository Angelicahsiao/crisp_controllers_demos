#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from franka_msgs.srv import SetFullCollisionBehavior


class CollisionBehaviorSetter(Node):
    def __init__(self):
        super().__init__("collision_behavior_setter")

        self.cli = self.create_client(
            SetFullCollisionBehavior, "/service_server/set_full_collision_behavior"
        )
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting again...")
        self.send_request()

    def send_request(self):
        req = SetFullCollisionBehavior.Request()
        self.get_logger().info("Sending request.")

        # Thresholds raised ~40% over the previous set to stop nuisance reflex
        # aborts during teleop (jerky streamed deltas, gripper contact). Crossing
        # a LOWER threshold only flags a contact; crossing an UPPER one aborts the
        # motion, so the upper values are what actually keep the arm running.
        #
        # Joints 5-7 are rated to ~12 Nm (joints 1-4 to ~87 Nm), so the wrist
        # entries below already sit above what those joints can produce and their
        # reflexes effectively never trip -- the real tolerance gained here is on
        # joints 1-4 and on the Cartesian force thresholds.
        #
        # A genuine collision still aborts. Do NOT raise these further without a
        # caged cell: the arm's ability to stop on contact scales with them.
        req.lower_torque_thresholds_nominal = [35.0, 35.0, 32.0, 30.0, 27.0, 24.0, 20.0]
        req.upper_torque_thresholds_nominal = [50.0, 50.0, 45.0, 42.0, 38.0, 34.0, 28.0]
        req.lower_torque_thresholds_acceleration = [
            35.0,
            35.0,
            32.0,
            30.0,
            27.0,
            24.0,
            20.0,
        ]
        req.upper_torque_thresholds_acceleration = [
            50.0,
            50.0,
            45.0,
            42.0,
            38.0,
            34.0,
            28.0,
        ]
        req.lower_force_thresholds_nominal = [45.0, 45.0, 45.0, 35.0, 35.0, 35.0]
        req.upper_force_thresholds_nominal = [60.0, 60.0, 60.0, 50.0, 50.0, 50.0]
        req.lower_force_thresholds_acceleration = [45.0, 45.0, 45.0, 35.0, 35.0, 35.0]
        req.upper_force_thresholds_acceleration = [60.0, 60.0, 60.0, 50.0, 50.0, 50.0]

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info("Collision behavior set successfully")
        else:
            self.get_logger().error("Failed to set collision behavior")


def main(args=None):
    rclpy.init(args=args)
    node = CollisionBehaviorSetter()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
