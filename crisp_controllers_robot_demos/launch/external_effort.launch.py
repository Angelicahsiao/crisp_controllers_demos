"""Launch the external joint-effort estimator node.

Publishes gravity-free joint effort (tau_ext = tau_measured - g(q)) on
`external_joint_effort` (std_msgs/Float32MultiArray). crisp_gym records it as a
float32_array sensor. Requires the robot bring-up to publish /robot_description
and /joint_states (with current-derived effort).

Example (UR7e):
    ros2 launch crisp_controllers_robot_demos external_effort.launch.py \\
        joint_names:="['shoulder_pan_joint','shoulder_lift_joint','elbow_joint',\\
'wrist_1_joint','wrist_2_joint','wrist_3_joint']"
"""

import ast

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration("namespace").perform(context)
    output_topic = LaunchConfiguration("output_topic").perform(context)
    joint_state_topic = LaunchConfiguration("joint_state_topic").perform(context)
    joint_names = ast.literal_eval(LaunchConfiguration("joint_names").perform(context))

    return [
        Node(
            package="crisp_controllers_robot_demos",
            executable="external_effort_node",
            name="external_effort_node",
            namespace=namespace,
            output="screen",
            parameters=[
                {
                    "joint_names": joint_names,
                    "output_topic": output_topic,
                    "joint_state_topic": joint_state_topic,
                }
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("output_topic", default_value="external_joint_effort"),
            DeclareLaunchArgument("joint_state_topic", default_value="joint_states"),
            DeclareLaunchArgument(
                "joint_names",
                default_value="['shoulder_pan_joint','shoulder_lift_joint',"
                "'elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']",
                description="Actuated arm joints, in order (Python list literal).",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
