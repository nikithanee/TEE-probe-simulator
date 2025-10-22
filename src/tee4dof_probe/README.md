# tee4dof_probe (fixed)

Works on ROS 2 Humble + Gazebo Classic. Controllers come up at `/controller_manager`.

Build:
```
mkdir -p ~/ros2_ws/src
cp -r tee4dof_probe ~/ros2_ws/src/
cd ~/ros2_ws
rosdep install --from-paths src -r -y
colcon build --symlink-install
source install/setup.bash
```

Run:
```
ros2 launch tee4dof_probe spawn_tee_probe.launch.py
```
