import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def robot_description_dependent_nodes_spawner(
    context: LaunchContext,
    ur_type,
    robot_ip,
    use_fake_hardware,
    tf_prefix,
    start_robot_state_publisher,
):
    ur_type_str = context.perform_substitution(ur_type)
    robot_ip_str = context.perform_substitution(robot_ip)
    use_fake_hardware_str = context.perform_substitution(use_fake_hardware)
    tf_prefix_str = context.perform_substitution(tf_prefix)

    ur_xacro_filepath = os.path.join(
        get_package_share_directory("crisp_controllers_robot_demos"),
        "config",
        "ur",
        "ur_single.urdf.xacro",
    )
    robot_description = xacro.process_file(
        ur_xacro_filepath,
        mappings={
            "ur_type": ur_type_str,
            "robot_ip": robot_ip_str,
            "use_fake_hardware": use_fake_hardware_str,
            "tf_prefix": tf_prefix_str,
        },
    ).toprettyxml(indent="  ")

    ur_controllers = PathJoinSubstitution(
        [
            FindPackageShare("crisp_controllers_robot_demos"),
            "config",
            "ur",
            "controllers.yaml",
        ]
    )

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
            condition=IfCondition(start_robot_state_publisher),
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                ur_controllers,
                {"robot_description": robot_description},
            ],
            output={
                "stdout": "screen",
                "stderr": "screen",
            },
            on_exit=Shutdown(),
        ),
    ]


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    tf_prefix = LaunchConfiguration("tf_prefix")
    use_rviz = LaunchConfiguration("use_rviz")
    start_robot_state_publisher = LaunchConfiguration("start_robot_state_publisher")

    rviz_file = os.path.join(
        get_package_share_directory("ur_description"),
        "rviz",
        "view_robot.rviz",
    )

    robot_description_dependent_nodes_spawner_opaque_function = OpaqueFunction(
        function=robot_description_dependent_nodes_spawner,
        args=[
            ur_type,
            robot_ip,
            use_fake_hardware,
            tf_prefix,
            start_robot_state_publisher,
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "ur_type",
                default_value="ur5e",
                description="UR robot type (ur3, ur3e, ur5, ur5e, ur10, ur10e, ur16e, ur20, etc.)",
            ),
            DeclareLaunchArgument(
                "robot_ip",
                default_value="192.168.56.101",
                description="Hostname or IP address of the robot.",
            ),
            DeclareLaunchArgument(
                "use_fake_hardware",
                default_value="false",
                description="Use mock_components/GenericSystem instead of the real UR driver.",
            ),
            DeclareLaunchArgument(
                "tf_prefix",
                default_value="",
                description="Prefix applied to all joint and link names (without trailing underscore).",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                description="Launch RViz2 for visualization.",
            ),
            DeclareLaunchArgument(
                "start_robot_state_publisher",
                default_value="true",
                description="Start the robot_state_publisher node.",
            ),
            robot_description_dependent_nodes_spawner_opaque_function,
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["cartesian_impedance_controller", "--inactive"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_impedance_controller", "--inactive"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_trajectory_controller"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["twist_broadcaster"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["pose_broadcaster"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["gravity_compensation", "--inactive"],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["--display-config", rviz_file],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
