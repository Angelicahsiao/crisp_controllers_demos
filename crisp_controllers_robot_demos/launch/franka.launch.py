#  Copyright (c) 2024 Franka Robotics GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.


import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    AndSubstitution,
    LaunchConfiguration,
    NotSubstitution,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def robot_description_dependent_nodes_spawner(
    context: LaunchContext,
    robot_ip,
    arm_id,
    use_fake_hardware,
    fake_sensor_commands,
    load_gripper,
    arm_prefix,
    start_robot_state_publisher,
    use_gripper,
    com_port,
):
    robot_ip_str = context.perform_substitution(robot_ip)
    arm_id_str = context.perform_substitution(arm_id)
    arm_prefix_str = context.perform_substitution(arm_prefix)
    use_fake_hardware_str = context.perform_substitution(use_fake_hardware)
    fake_sensor_commands_str = context.perform_substitution(fake_sensor_commands)
    load_gripper_str = context.perform_substitution(load_gripper)
    use_gripper_str = context.perform_substitution(use_gripper)
    com_port_str = context.perform_substitution(com_port)
    use_gripper_bool = use_gripper_str.lower() in ("true", "1", "yes")

    # use_gripper selects the Robotiq 2F-85 variant (which replaces the
    # Franka Hand at the flange); otherwise the plain arm, with the Franka
    # Hand governed by load_gripper.
    franka_xacro_filepath = os.path.join(
        get_package_share_directory("crisp_controllers_robot_demos"),
        "config",
        "fr3",
        "fr3_single_robotiq.urdf.xacro" if use_gripper_bool else "fr3_single.urdf.xacro",
    )
    # Only pass mappings the fr3 xacros actually declare as <xacro:arg>. Newer
    # xacro rejects undeclared mappings ("Invalid parameter ..."); the fr3
    # macro hardcodes arm_id/ros2_control and never reads mujoco_model, so
    # those three (previously silently ignored) are dropped.
    xacro_mappings = {
        "arm_prefix": arm_prefix_str,
        "robot_ip": robot_ip_str,
        "load_gripper": load_gripper_str,
        "use_fake_hardware": use_fake_hardware_str,
        "fake_sensor_commands": fake_sensor_commands_str,
    }
    if use_gripper_bool:
        xacro_mappings["com_port"] = com_port_str
        # the robotiq xacro has no load_gripper arg (the hand is always off)
        del xacro_mappings["load_gripper"]
    robot_description = xacro.process_file(
        franka_xacro_filepath,
        mappings=xacro_mappings,
    ).toprettyxml(indent="  ")

    franka_controllers = PathJoinSubstitution(
        [
            FindPackageShare("crisp_controllers_robot_demos"),
            "config",
            "fr3",
            f"{arm_prefix_str}_controllers.yaml"
            if arm_prefix_str != ""
            else "controllers.yaml",
        ]
    )

    nodes = [
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
                franka_controllers,
                {"robot_description": robot_description},
            ],
            remappings=[("joint_states", "franka/joint_states")],
            output={
                "stdout": "screen",
                "stderr": "screen",
            },
            on_exit=Shutdown(),
        ),
    ]

    # Robotiq 2F-85 on its OWN controller_manager (node name
    # robotiq_controller_manager) so it never enters franka's controller_manager
    # (franka_hardware rejects a single finger position interface). Controllers
    # stay at root namespace -> crisp_py reaches /robotiq_gripper_controller
    # unchanged. The franka URDF carries the Robotiq visual only
    # (include_ros2_control=false); this node owns the RS485 hardware.
    if use_gripper_bool:
        pkg = get_package_share_directory("crisp_controllers_robot_demos")
        robotiq_description = xacro.process_file(
            os.path.join(pkg, "config", "robotiq", "robotiq_2f85_standalone.urdf.xacro"),
            mappings={"com_port": com_port_str, "use_fake_hardware": use_fake_hardware_str},
        ).toprettyxml(indent="  ")
        robotiq_controllers = os.path.join(
            pkg, "config", "robotiq", "robotiq_standalone_controllers.yaml"
        )
        ns = "robotiq_2f85"
        nodes += [
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                namespace=ns,
                parameters=[{"robot_description": robotiq_description}, robotiq_controllers],
                output="screen",
            ),
            Node(package="controller_manager", executable="spawner", namespace=ns,
                 arguments=["robotiq_joint_state_broadcaster", "-c", "controller_manager"], output="screen"),
            Node(package="controller_manager", executable="spawner", namespace=ns,
                 arguments=["robotiq_activation_controller", "-c", "controller_manager"], output="screen"),
            Node(package="controller_manager", executable="spawner", namespace=ns,
                 arguments=["robotiq_gripper_controller", "-c", "controller_manager"], output="screen"),
        ]
    return nodes


def generate_launch_description():
    arm_id_parameter_name = "arm_id"
    arm_prefix_parameter_name = "arm_prefix"
    robot_ip_parameter_name = "robot_ip"
    load_gripper_parameter_name = "load_gripper"
    use_fake_hardware_parameter_name = "use_fake_hardware"
    fake_sensor_commands_parameter_name = "fake_sensor_commands"
    use_rviz_parameter_name = "use_rviz"
    start_robot_state_publisher_name = "start_robot_state_publisher"
    use_gripper_parameter_name = "use_gripper"
    com_port_parameter_name = "com_port"

    arm_id = LaunchConfiguration(arm_id_parameter_name)
    arm_prefix = LaunchConfiguration(arm_prefix_parameter_name)
    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    load_gripper = LaunchConfiguration(load_gripper_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    fake_sensor_commands = LaunchConfiguration(fake_sensor_commands_parameter_name)
    use_rviz = LaunchConfiguration(use_rviz_parameter_name)
    start_robot_state_publisher = LaunchConfiguration(start_robot_state_publisher_name)
    use_gripper = LaunchConfiguration(use_gripper_parameter_name)
    com_port = LaunchConfiguration(com_port_parameter_name)
    use_franka_state_broadcaster = LaunchConfiguration("use_franka_state_broadcaster")

    rviz_file = os.path.join(
        get_package_share_directory("franka_description"),
        "rviz",
        "visualize_franka.rviz",
    )

    robot_description_dependent_nodes_spawner_opaque_function = OpaqueFunction(
        function=robot_description_dependent_nodes_spawner,
        args=[
            robot_ip,
            arm_id,
            use_fake_hardware,
            fake_sensor_commands,
            load_gripper,
            arm_prefix,
            start_robot_state_publisher,
            use_gripper,
            com_port,
        ],
    )

    launch_description = LaunchDescription(
        [
            DeclareLaunchArgument(
                robot_ip_parameter_name,
                description="Hostname or IP address of the robot.",
            ),
            DeclareLaunchArgument(
                arm_id_parameter_name,
                description="ID of the type of arm used. Supported values: fer, fr3, fp3",
            ),
            DeclareLaunchArgument(
                use_rviz_parameter_name,
                default_value="false",
                description="Visualize the robot in Rviz",
            ),
            DeclareLaunchArgument(
                use_fake_hardware_parameter_name,
                default_value="false",
                description="Use fake hardware",
            ),
            DeclareLaunchArgument(
                fake_sensor_commands_parameter_name,
                default_value="false",
                description='Fake sensor commands. Only valid when "{}" is true'.format(
                    use_fake_hardware_parameter_name
                ),
            ),
            DeclareLaunchArgument(
                load_gripper_parameter_name,
                default_value="true",
                description="Use Franka Gripper as an end-effector, otherwise, the robot is loaded "
                "without an end-effector.",
            ),
            DeclareLaunchArgument(
                use_gripper_parameter_name,
                default_value="false",
                description="Mount a Robotiq 2F-140 at the flange instead of the "
                "Franka Hand (overrides load_gripper).",
            ),
            DeclareLaunchArgument(
                com_port_parameter_name,
                default_value="/dev/ttyUSB0",
                description="Serial device of the Robotiq USB-to-RS485 adapter.",
            ),
            DeclareLaunchArgument(
                "use_franka_state_broadcaster",
                default_value="false",
                description="Spawn franka_robot_state_broadcaster (publishes "
                "tau_ext_hat_filtered and the external wrench). OFF by default and NOT "
                "recommended at 1 kHz: it publishes nine topics per cycle from the "
                "realtime loop, leaving so little headroom that any burst of "
                "non-realtime work (rviz, a teleop client attaching, an action-goal "
                "callback) overruns the FCI's 1 ms deadline and aborts the robot with "
                "communication_constraints_violation. It survived ~100 s idle without "
                "rviz and still aborted on the first trajectory goal. Prefer "
                "external_effort.launch.py, which runs out-of-process.",
            ),
            DeclareLaunchArgument(
                arm_prefix_parameter_name,
                default_value="",
                description="The prefix of the arm.",
            ),
            DeclareLaunchArgument(
                start_robot_state_publisher_name,
                default_value="true",
                description="",
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                parameters=[
                    {
                        "source_list": [
                            "franka/joint_states",
                            "franka_gripper/joint_states",
                        ],
                        "rate": 1000,
                    }
                ],
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
            # external_torques_broadcaster is NOT spawned: its plugin lives in
            # franka_force_feedback_controllers (danielsanjosepro/franka_broadcasters),
            # which is absent from BOTH the preserved image and the from-source
            # rebuild (confirmed via `ros2 control list_controller_types`), so
            # spawning it only produced a FATAL on every startup.
            #
            # DANGER -- franka_robot_state_broadcaster is OFF BY DEFAULT.
            # Its update() publishes NINE topics per cycle (one realtime publisher
            # plus eight plain ->publish() calls) directly inside the 1 kHz RT loop.
            # On this setup that overruns the control cycle: the realtime publisher
            # fails to lock, update() returns ERROR, and the FCI's 1 ms deadline is
            # missed until the robot aborts with
            #   libfranka: Move command aborted: motion aborted by reflex!
            #   ["communication_constraints_violation"]
            # i.e. enabling it CAN STOP THE ROBOT. Watch for "Failed to lock the
            # realtime publisher" -- that message is the precursor to the abort.
            #
            # Measured on this setup (fr3 5.8.0, CM at 1 kHz, FIFO prio 50):
            #   use_rviz:=True  -> trylock failures within seconds, then abort
            #   use_rviz:=false -> ~100 s clean while IDLE, then aborted anyway as
            #                      soon as a client sent a joint-trajectory goal
            # So rviz is NOT the deciding factor -- it was simply one source of extra
            # load. The broadcaster leaves so little realtime headroom that ANY burst
            # of non-realtime work (rviz, a teleop client attaching, an action-goal
            # callback) is enough to miss the FCI deadline and stop the arm. An idle
            # run proves nothing.
            #
            # Recommendation: do NOT enable this at 1 kHz. Use
            # external_effort.launch.py, which computes tau_ext out-of-process and
            # cannot miss the deadline. Enable this only if you have throttled the
            # broadcaster's publishing or are running a controller manager well below
            # 1 kHz -- and never on a robot doing real work without a soak test.
            # For a gravity-free torque signal prefer external_effort.launch.py,
            # which runs in its own process and cannot miss the FCI deadline.
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["franka_robot_state_broadcaster"],
                output="screen",
                condition=IfCondition(
                    AndSubstitution(
                        use_franka_state_broadcaster, NotSubstitution(use_fake_hardware)
                    )
                ),
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["torque_feedback_controller", "--inactive"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["gravity_compensation", "--inactive"],
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("franka_gripper"),
                                "launch",
                                "gripper.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    robot_ip_parameter_name: robot_ip,
                    use_fake_hardware_parameter_name: use_fake_hardware,
                    "namespace": arm_prefix,
                }.items(),
                # Franka Hand node only when the hand is mounted (not the Robotiq)
                condition=IfCondition(
                    AndSubstitution(load_gripper, NotSubstitution(use_gripper))
                ),
            ),
            # Robotiq controllers are spawned on their own controller_manager
            # (robotiq_controller_manager) in the description-dependent spawner
            # above — NOT here in franka's controller_manager.
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["--display-config", rviz_file],
                condition=IfCondition(use_rviz),
            ),
            # Franka Hand adapter — only for the Franka Hand, not the Robotiq.
            # It waits on the franka_gripper action server, which doesn't exist
            # with use_gripper:=true, so it must not run there.
            Node(
                package="crisp_controllers_robot_demos",
                executable="crisp_py_franka_hand_adapter",
                name="crisp_py_franka_hand_adapter",
                output="screen",
                condition=IfCondition(
                    AndSubstitution(load_gripper, NotSubstitution(use_gripper))
                ),
            ),
        ]
    )

    return launch_description
