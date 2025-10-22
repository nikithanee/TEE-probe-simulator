from setuptools import setup
import os
from glob import glob

package_name = 'tee4dof_probe'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # Install package manifest
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Install launch files
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),

        # Install URDF models
        (os.path.join('share', package_name, 'urdf'),
         glob('urdf/*.urdf') + glob('urdf/*.xacro')),

        # Install config files (ros2_control yaml)
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='4-DOF TEE probe simulation in ROS2 + Gazebo Classic',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
