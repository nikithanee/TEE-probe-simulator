"""
TEE Probe Gazebo Simulation - ROS2 Humble Launch File (fixed)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_tee_probe = get_package_share_directory("tee_probe")
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")

    urdf_file = os.path.join(pkg_tee_probe, "urdf", "tee_probe.urdf")

    declare_gui = DeclareLaunchArgument(
        "gui", default_value="true",
        description="Start Gazebo GUI. Set false for headless.")
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true")
    declare_spawn_x = DeclareLaunchArgument("spawn_x", default_value="0.0")
    declare_spawn_y = DeclareLaunchArgument("spawn_y", default_value="0.0")
    declare_spawn_z = DeclareLaunchArgument("spawn_z", default_value="0.2")

    gui          = LaunchConfiguration("gui")
    use_sim_time = LaunchConfiguration("use_sim_time")
    spawn_x      = LaunchConfiguration("spawn_x")
    spawn_y      = LaunchConfiguration("spawn_y")
    spawn_z      = LaunchConfiguration("spawn_z")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gazebo.launch.py")
        ),
        launch_arguments={"gui": gui, "pause": "false", "verbose": "false"}.items(),
    )

    with open(urdf_file, "r") as f:
        robot_description_content = f.read()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description_content},
            {"use_sim_time": use_sim_time},
        ],
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_tee_probe",
        output="screen",
        arguments=[
            "-entity", "tee_probe",
            "-topic", "/robot_description",
            "-x", spawn_x, "-y", spawn_y, "-z", spawn_z,
            "-R", "0.0", "-P", "0.0", "-Y", "0.0",
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="jsb_spawner",
        output="screen",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    insertion_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="insertion_spawner",
        output="screen",
        arguments=["insertion_controller", "--controller-manager", "/controller_manager"],
    )

    axial_rotation_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="axial_spawner",
        output="screen",
        arguments=["axial_rotation_controller", "--controller-manager", "/controller_manager"],
    )

    large_wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="large_wheel_spawner",
        output="screen",
        arguments=["large_wheel_controller", "--controller-manager", "/controller_manager"],
    )

    small_wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="small_wheel_spawner",
        output="screen",
        arguments=["small_wheel_controller", "--controller-manager", "/controller_manager"],
    )

    spawn_after_gazebo = TimerAction(period=3.0, actions=[spawn_entity])

    jsb_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    controllers_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                insertion_controller_spawner,
                axial_rotation_controller_spawner,
                large_wheel_controller_spawner,
                small_wheel_controller_spawner,
            ],
        )
    )

    return LaunchDescription([
        declare_gui,
        declare_use_sim_time,
        declare_spawn_x,
        declare_spawn_y,
        declare_spawn_z,
        gazebo,
        robot_state_publisher,
        spawn_after_gazebo,
        jsb_after_spawn,
        controllers_after_jsb,
    ])
