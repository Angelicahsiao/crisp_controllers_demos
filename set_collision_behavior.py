#!/usr/bin/env python3
"""Loosen (or set) the Franka FR3 collision-behavior thresholds via FCI.

Higher thresholds = the robot tolerates more contact torque/force before
triggering a reflex stop. Useful for contact-rich teleop/manipulation, but it
REDUCES the built-in safety margin — the arm will push harder before stopping.
Start conservative and only raise as needed, with people clear of the workspace.

Run with the franka bringup active (launch_franka_gripper / launch_franka):
    python3 src/crisp_controllers_demos/set_collision_behavior.py

The service name defaults to the franka_ros2 service_server, but bringups differ
-- if the call hangs on "Service not available", check the real name with:
    ros2 service list | grep -i collision
and pass it:
    python3 src/crisp_controllers_demos/set_collision_behavior.py \
        --ros-args -p service_name:=/your/set_full_collision_behavior
"""

import rclpy
from rclpy.node import Node
from franka_msgs.srv import SetFullCollisionBehavior


class CollisionBehaviorSetter(Node):
    def __init__(self):
        super().__init__("collision_behavior_setter")

        # Override with -p service_name:=... if your bringup advertises it elsewhere.
        service_name = self.declare_parameter(
            "service_name", "/service_server/set_full_collision_behavior"
        ).value

        self.cli = self.create_client(SetFullCollisionBehavior, service_name)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                f"Service '{service_name}' not available, waiting again... "
                "(check 'ros2 service list | grep -i collision')"
            )
        self.send_request()

    def send_request(self):
        req = SetFullCollisionBehavior.Request()
        self.get_logger().info("Sending request.")

        req.lower_torque_thresholds_nominal = [25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0]
        req.upper_torque_thresholds_nominal = [35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0]
        req.lower_torque_thresholds_acceleration = [
            25.0,
            25.0,
            22.0,
            20.0,
            19.0,
            17.0,
            14.0,
        ]
        req.upper_torque_thresholds_acceleration = [
            35.0,
            35.0,
            32.0,
            30.0,
            29.0,
            27.0,
            24.0,
        ]
        req.lower_force_thresholds_nominal = [30.0, 30.0, 30.0, 25.0, 25.0, 25.0]
        req.upper_force_thresholds_nominal = [40.0, 40.0, 40.0, 35.0, 35.0, 35.0]
        req.lower_force_thresholds_acceleration = [30.0, 30.0, 30.0, 25.0, 25.0, 25.0]
        req.upper_force_thresholds_acceleration = [40.0, 40.0, 40.0, 35.0, 35.0, 35.0]

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
