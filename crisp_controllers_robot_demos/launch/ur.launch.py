import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def robot_description_dependent_nodes_spawner(
    context: LaunchContext,
    ur_type,
    robot_ip,
    use_fake_hardware,
    tf_prefix,
    headless_mode,
    start_robot_state_publisher,
    use_gripper,
    com_port,
):
    ur_type_str = context.perform_substitution(ur_type)
    robot_ip_str = context.perform_substitution(robot_ip)
    use_fake_hardware_str = context.perform_substitution(use_fake_hardware)
    tf_prefix_str = context.perform_substitution(tf_prefix)
    headless_mode_str = context.perform_substitution(headless_mode)
    use_gripper_str = context.perform_substitution(use_gripper)
    com_port_str = context.perform_substitution(com_port)

    pkg_share = get_package_share_directory("crisp_controllers_robot_demos")
    calibration_file = os.path.join(pkg_share, "config", "ur", f"{ur_type_str}_calibration.yaml")
    default_kinematics = os.path.join(
        get_package_share_directory("ur_description"),
        "config", ur_type_str, "default_kinematics.yaml",
    )
    kinematics_file = calibration_file if os.path.exists(calibration_file) else default_kinematics

    # With a gripper, use the combined UR + Robotiq 2F-140 description so both
    # hardware systems are loaded by the same controller_manager.
    # Fake hardware runs the arm in MuJoCo (crisp_mujoco_sim) so the effort-based
    # CIC/JIC controllers work — mock_components rejects effort interfaces.
    use_gripper_bool = use_gripper_str.lower() in ("true", "1", "yes")
    use_fake_hardware_bool = use_fake_hardware_str.lower() in ("true", "1", "yes")
    if use_fake_hardware_bool:
        xacro_filename = "ur_single_mujoco.urdf.xacro"
        xacro_mappings = {
            "ur_type": ur_type_str,
            "tf_prefix": tf_prefix_str,
            "use_gripper": use_gripper_str,
            "mujoco_model": os.path.join(pkg_share, "config", "ur", "ur7e_scene.xml"),
            "kinematics_parameters_file": kinematics_file,
        }
    else:
        xacro_filename = "ur_single_robotiq.urdf.xacro" if use_gripper_bool else "ur_single.urdf.xacro"
        xacro_mappings = {
            "ur_type": ur_type_str,
            "robot_ip": robot_ip_str,
            "use_fake_hardware": use_fake_hardware_str,
            "tf_prefix": tf_prefix_str,
            "headless_mode": headless_mode_str,
            "kinematics_parameters_file": kinematics_file,
        }
        if use_gripper_bool:
            xacro_mappings["com_port"] = com_port_str
    ur_xacro_filepath = os.path.join(pkg_share, "config", "ur", xacro_filename)

    robot_description = xacro.process_file(
        ur_xacro_filepath,
        mappings=xacro_mappings,
    ).toprettyxml(indent="  ")

    # Real hardware: use_gravity_compensation=false (UR firmware's direct_torque
    # applies gravity automatically; enabling it in the controller double-counts).
    # Fake hardware: use_gravity_compensation=true for physically correct torques.
    controllers_yaml = "controllers_fake.yaml" if use_fake_hardware_bool else "controllers.yaml"
    ur_controllers = os.path.join(
        get_package_share_directory("crisp_controllers_robot_demos"),
        "config",
        "ur",
        controllers_yaml,
    )

    # The ros2_control node moved between distros: Humble's ur_robot_driver ships a
    # custom ur_ros2_control_node; Jazzy removed it and relies on the generic
    # controller_manager/ros2_control_node (which loads the same ur_robot_driver
    # plugin). Both take the controllers YAML + robot_description the same way.
    ros_distro = os.environ.get("ROS_DISTRO", "humble")
    if ros_distro == "humble":
        control_node_pkg, control_node_exe = "ur_robot_driver", "ur_ros2_control_node"
    else:
        control_node_pkg, control_node_exe = "controller_manager", "ros2_control_node"

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
            package=control_node_pkg,
            executable=control_node_exe,
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
    headless_mode = LaunchConfiguration("headless_mode")
    use_rviz = LaunchConfiguration("use_rviz")
    start_robot_state_publisher = LaunchConfiguration("start_robot_state_publisher")
    use_gripper = LaunchConfiguration("use_gripper")
    com_port = LaunchConfiguration("com_port")

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
            headless_mode,
            start_robot_state_publisher,
            use_gripper,
            com_port,
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
                "headless_mode",
                default_value="true",
                description=(
                    "Send URScript directly to port 30001 (headless), bypassing URCapX ScriptBuilder. "
                    "Required for torque control with URCapX ExternalControl < 1.2.0, which generates "
                    "its own program loop and omits MODE_TORQUE dispatch. "
                    "Prerequisite: pendant must be in Remote Control mode."
                ),
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
            DeclareLaunchArgument(
                "use_gripper",
                default_value="false",
                description="Attach a Robotiq 2F-140 to tool0 and spawn its controllers.",
            ),
            DeclareLaunchArgument(
                "com_port",
                default_value="/dev/ttyUSB0",
                description="Serial port for the Robotiq 2F-140 (USB-to-RS485 adapter).",
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
                arguments=["joint_trajectory_controller", "--inactive"],
                output="screen",
            ),
            # Velocity command interface, used by the velocity_sweep script for
            # friction calibration. Inactive: it conflicts with the effort
            # controllers, which claim the same joints.
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["forward_velocity_controller", "--inactive"],
                output="screen",
                condition=UnlessCondition(use_fake_hardware),
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
                arguments=["force_torque_sensor_broadcaster"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["gravity_compensation", "--inactive"],
                output="screen",
            ),
            # Robotiq 2F-140 controllers (only when use_gripper:=true)
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["robotiq_joint_state_broadcaster"],
                output="screen",
                condition=IfCondition(use_gripper),
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["robotiq_activation_controller"],
                output="screen",
                condition=IfCondition(use_gripper),
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["robotiq_gripper_controller"],
                output="screen",
                condition=IfCondition(use_gripper),
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
