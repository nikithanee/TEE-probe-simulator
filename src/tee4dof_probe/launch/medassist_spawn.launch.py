from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    robot_name = 'tee4dof_probe'


    # URDF path
    urdf_file = PathJoinSubstitution([
        FindPackageShare('tee4dof_probe'),
        'urdf',
        'medassist_4dof-1.urdf'
    ])

    # Load and publish robot description
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    # Gazebo Classic
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('gazebo_ros'),
            '/launch/gazebo.launch.py'
        ])
    )

    # Robot state publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }]
    )

    # Spawn entity into Gazebo
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', robot_name],
        output='screen'
    )

    # Spawn joint_state_broadcaster
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', f'/{robot_name}/controller_manager'],
        output='screen'
    )

    # Spawn arm controller
    arm_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller',
                   '--controller-manager', f'/{robot_name}/controller_manager'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        gazebo,
        rsp,
        spawn,
        jsb_spawner,
        arm_spawner,
    ])
