"""Standalone Robotiq 2F-140 bringup on its own controller_manager.

Runs the gripper independently of any arm, so it never enters franka's
controller_manager (which rejects a partial position-interface start). Publishes
the GripperCommand action on /robotiq_gripper_controller/gripper_cmd, which
crisp_py's Gripper uses unchanged.

Example:
    ros2 launch crisp_controllers_robot_demos robotiq_gripper_standalone.launch.py \\
        com_port:=/dev/ttyUSB0
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    com_port = LaunchConfiguration("com_port").perform(context)
    use_fake_hardware = LaunchConfiguration("use_fake_hardware").perform(context)
    use_rsp = LaunchConfiguration("use_robot_state_publisher").perform(context)

    pkg = get_package_share_directory("crisp_controllers_robot_demos")
    urdf = os.path.join(pkg, "config", "robotiq", "robotiq_2f140_standalone.urdf.xacro")
    controllers = os.path.join(
        pkg, "config", "robotiq", "robotiq_standalone_controllers.yaml"
    )
    robot_description = xacro.process_file(
        urdf, mappings={"com_port": com_port, "use_fake_hardware": use_fake_hardware}
    ).toprettyxml(indent="  ")

    cm = "robotiq_controller_manager"
    nodes = [
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            name=cm,
            parameters=[{"robot_description": robot_description}, controllers],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["robotiq_joint_state_broadcaster", "-c", cm],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["robotiq_activation_controller", "-c", cm],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["robotiq_gripper_controller", "-c", cm],
            output="screen",
        ),
    ]
    # Optional: publish TF for the standalone gripper (off by default; when
    # integrated with the arm, the arm's robot_state_publisher owns TF).
    if use_rsp.lower() in ("true", "1", "yes"):
        nodes.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            )
        )
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("com_port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("use_fake_hardware", default_value="false"),
            DeclareLaunchArgument("use_robot_state_publisher", default_value="false"),
            OpaqueFunction(function=_setup),
        ]
    )
