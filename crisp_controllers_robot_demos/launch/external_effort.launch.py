"""Launch the external joint-effort estimator node.

Publishes gravity-free joint effort (tau_ext = effort_gain * I - g(q) - offset)
on `external_joint_effort` (std_msgs/Float32MultiArray); I is /joint_states
effort, which is motor current on the UR. crisp_gym records it as a
float32_array sensor. Requires the robot bring-up to publish /robot_description
and /joint_states.

The calibration is auto-loaded: with no explicit `calibration_file`, the launch
uses config/ur/external_effort_calibration.yaml (written there by
calibrate_external_effort) if it exists. Set `visualize:=true` to open rqt_plot
with one live trace per joint.

Example (UR7e):
    ros2 launch crisp_controllers_robot_demos external_effort.launch.py \\
        visualize:=true
"""

import ast
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Resolve the calibration path from the SOURCE tree (this file), not the install
# share: with --symlink-install realpath(__file__) points into the bind-mounted
# source, so a calibration written here persists across container rebuilds and
# both calibrate_external_effort and this launch agree on the location.
_PKG_DIR = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
DEFAULT_CALIBRATION = os.path.join(
    _PKG_DIR, "config", "ur", "external_effort_calibration.yaml"
)


def _launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration("namespace").perform(context)
    output_topic = LaunchConfiguration("output_topic").perform(context)
    joint_state_topic = LaunchConfiguration("joint_state_topic").perform(context)
    joint_names = ast.literal_eval(LaunchConfiguration("joint_names").perform(context))
    calibration_file = LaunchConfiguration("calibration_file").perform(context)
    visualize = LaunchConfiguration("visualize").perform(context).lower() in (
        "true",
        "1",
        "yes",
    )

    # Auto-load: no explicit file -> use the canonical calibration path if present,
    # so `ros2 launch ... external_effort.launch.py` just works after calibrating.
    # If it's missing the node runs uncalibrated (gain 1 = meaningless) and warns.
    if not calibration_file and os.path.exists(DEFAULT_CALIBRATION):
        calibration_file = DEFAULT_CALIBRATION

    nodes = [
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
                    "calibration_file": calibration_file,
                }
            ],
        )
    ]

    if visualize:
        topic = f"/{namespace}/{output_topic}" if namespace else f"/{output_topic}"
        # One rqt_plot trace per joint (each element of the Float32MultiArray).
        fields = [f"{topic}/data[{i}]" for i in range(len(joint_names))]
        nodes.append(
            Node(
                package="rqt_plot",
                executable="rqt_plot",
                name="external_effort_plot",
                arguments=fields,
                output="screen",
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("output_topic", default_value="external_joint_effort"),
            DeclareLaunchArgument("joint_state_topic", default_value="joint_states"),
            DeclareLaunchArgument(
                "calibration_file",
                default_value="",
                description="YAML from calibrate_external_effort with per-joint "
                "effort_gain/offset. Empty auto-loads config/ur/"
                "external_effort_calibration.yaml if present; with no calibration "
                "the gain defaults to 1, which is meaningless (effort is current).",
            ),
            DeclareLaunchArgument(
                "joint_names",
                default_value="['shoulder_pan_joint','shoulder_lift_joint',"
                "'elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']",
                description="Actuated arm joints, in order (Python list literal).",
            ),
            DeclareLaunchArgument(
                "visualize",
                default_value="false",
                description="Open rqt_plot with one live trace per joint's external effort.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
