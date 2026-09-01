from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    urdf_path = '/tmp/tee_probe.urdf'
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            output='screen'
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen'
        ),
    ])
