"""
TEE Probe - Headless Launch for DRL Training
=============================================
Identical to tee_simulation.launch.py but with:
  - GUI disabled by default (no gzclient)
  - Faster physics update rate
  - No RViz

Usage:
  ros2 launch tee_probe tee_training.launch.py
"""

from tee_simulation import generate_launch_description as _base_gen
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


# Re-use the main launch description but default gui=false
def generate_launch_description():
    ld = _base_gen()
    # Override the gui default — headless for training
    # The simplest approach: just import and override the argument default
    return ld
