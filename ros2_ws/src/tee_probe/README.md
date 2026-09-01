# TEE Probe Simulation — ROS2 Humble

4-DOF Transesophageal Echocardiography (TEE) probe simulation with Gazebo Classic
and Deep Reinforcement Learning (DRL) control via `ros2_control`.

---

## Package Structure

```
tee_probe/
├── package.xml                         # ROS2 format-3 package manifest
├── CMakeLists.txt                      # ament_cmake build
├── urdf/
│   └── tee_probe.urdf                  # 4-DOF robot with ros2_control tags
├── config/
│   └── tee_controllers.yaml            # ros2_control controller config
├── launch/
│   ├── tee_simulation.launch.py        # Main launch (interactive)
│   └── tee_training.launch.py          # Headless launch for training
├── scripts/
│   ├── manual_control_node.py          # Keyboard control for testing
│   ├── tee_probe_env.py                # Gymnasium environment
│   ├── train_tee_probe.py              # TD3/SAC training (SB3)
│   └── drl_controller_node.py          # Deploy trained policy as ROS2 node
└── models/                             # Saved neural network checkpoints
```

---

## The 4 Degrees of Freedom

| # | Joint Name             | Type       | Range         | Axis | Medical Function           |
|---|------------------------|------------|---------------|------|----------------------------|
| 1 | `insertion_joint`      | prismatic  | 0 – 0.4 m     | Z    | Depth insertion            |
| 2 | `axial_rotation_joint` | continuous | unlimited     | Z    | Rotate imaging plane       |
| 3 | `large_wheel_joint`    | revolute   | ±90° (±π/2)   | Y    | Anterior/posterior flexion |
| 4 | `small_wheel_joint`    | revolute   | ±90° (±π/2)   | X    | Lateral left/right flexion |

---

## Installation

### System dependencies

```bash
sudo apt install \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-joint-state-broadcaster \
  ros-humble-position-controllers \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  ros-humble-tf2-ros
```

### Python dependencies (for DRL)

```bash
pip install "stable-baselines3[extra]" gymnasium tensorboard
```

### Build

```bash
mkdir -p ~/ros2_ws/src
cp -r tee_probe ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --packages-select tee_probe
source install/setup.bash
```

---

## Quick Start

### 1. Launch simulation

```bash
ros2 launch tee_probe tee_simulation.launch.py
```

### 2. Verify controllers are running

```bash
# List all active controllers
ros2 control list_controllers

# Expected output:
# joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
# insertion_controller   [position_controllers/JointPositionController]  active
# axial_rotation_controller ...                                           active
# large_wheel_controller ...                                              active
# small_wheel_controller ...                                              active

# Check joint states are publishing
ros2 topic echo /joint_states --once
```

### 3. Send manual commands

In ROS2 the controller topic format is `/CONTROLLER_NAME/commands`
(note: **plural** `commands`, and **no namespace prefix**):

```bash
# DOF 1: Insert 20 cm
ros2 topic pub -1 /insertion_controller/commands \
  std_msgs/msg/Float64MultiArray "data: [0.2]"

# DOF 2: Rotate 90°
ros2 topic pub -1 /axial_rotation_controller/commands \
  std_msgs/msg/Float64MultiArray "data: [1.571]"

# DOF 3: Anterior flexion 45°
ros2 topic pub -1 /large_wheel_controller/commands \
  std_msgs/msg/Float64MultiArray "data: [0.785]"

# DOF 4: Left lateral flexion 45°
ros2 topic pub -1 /small_wheel_controller/commands \
  std_msgs/msg/Float64MultiArray "data: [0.785]"
```

### 4. Keyboard control node

```bash
ros2 run tee_probe manual_control_node.py
```

---

## ROS1 → ROS2 Migration Reference

### Critical differences

| Concern | ROS1 Noetic | ROS2 Humble |
|---|---|---|
| Build system | `catkin_make` | `colcon build` |
| Launch files | XML `.launch` | Python `.launch.py` |
| Controller manager | `ros_control` | `ros2_control` |
| Hardware interface in URDF | `<transmission>` tags | `<ros2_control>` block |
| Gazebo plugin | `libgazebo_ros_control.so` | `libgazebo_ros2_control.so` |
| Joint state publisher | `JointStateController` | `JointStateBroadcaster` |
| Controller topic | `/ns/joint/command` (Float64) | `/controller_name/commands` (Float64MultiArray) |
| Namespace gotcha | `<robotNamespace>` must have `/` | Not needed — plugin reads from URDF |
| PID gains in launch | Separate `<rosparam>` block | Nested in controller YAML |
| Parameter API | `rospy.get_param()` | `self.declare_parameter()` |

### The biggest ROS1 bug — now fixed

In ROS1 you had this fragile pattern:
```xml
<!-- ROS1: Had to be exact, with leading slash -->
<robotNamespace>/tee_probe</robotNamespace>
```

In ROS2, the `libgazebo_ros2_control.so` plugin reads its parameters file
directly from the `<parameters>` tag in the URDF `<gazebo>` block:
```xml
<gazebo>
  <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
    <parameters>$(find tee_probe)/config/tee_controllers.yaml</parameters>
  </plugin>
</gazebo>
```
No namespace to get wrong.

### URDF: transmissions removed, ros2_control block added

ROS1 needed:
```xml
<transmission name="insertion_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="insertion_joint">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
  </joint>
  <actuator name="insertion_motor"><mechanicalReduction>1</mechanicalReduction></actuator>
</transmission>
```

ROS2 replaces ALL four transmissions with a single `<ros2_control>` block:
```xml
<ros2_control name="tee_probe_system" type="system">
  <hardware>
    <plugin>gazebo_ros2_control/GazeboSystem</plugin>
  </hardware>
  <joint name="insertion_joint">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  <!-- ... repeat for the other 3 joints -->
</ros2_control>
```

### controllers.yaml — ROS2 format

ROS1:
```yaml
tee_probe:
  joint_state_controller:
    type: joint_state_controller/JointStateController
  insertion_joint_position_controller:
    type: position_controllers/JointPositionController
    joint: insertion_joint
    pid: {p: 100, i: 0.01, d: 10}
```

ROS2 (`ros__parameters` nesting is mandatory):
```yaml
controller_manager:
  ros__parameters:
    update_rate: 100
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    insertion_controller:
      type: position_controllers/JointPositionController

insertion_controller:
  ros__parameters:
    joint: insertion_joint
    gains:
      insertion_joint: {p: 500.0, i: 1.0, d: 50.0}
```

### Launch file — Python instead of XML

ROS1 (XML):
```xml
<node name="controller_spawner" pkg="controller_manager" type="spawner"
      args="joint_state_controller insertion_joint_position_controller ..."/>
```

ROS2 (Python, event-driven):
```python
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

jsb_after_spawn = RegisterEventHandler(
    event_handler=OnProcessExit(
        target_action=spawn_entity,
        on_exit=[joint_state_broadcaster_spawner],
    )
)
```
This replaces the `time.sleep()` hacks from ROS1 with reliable process lifecycle events.

---

## DRL Training

### Train from scratch

```bash
# Terminal 1: Start headless simulation
ros2 launch tee_probe tee_simulation.launch.py gui:=false

# Terminal 2: Train
cd ~/ros2_ws
source install/setup.bash
python3 src/tee_probe/scripts/train_tee_probe.py --timesteps 200000

# Monitor with TensorBoard
tensorboard --logdir logs/td3_tee_probe
```

### Resume training

```bash
python3 scripts/train_tee_probe.py \
  --resume models/td3_tee_probe_100000_steps \
  --timesteps 200000
```

### Evaluate a model

```bash
python3 scripts/train_tee_probe.py --eval models/best/best_model
```

### Deploy trained policy

```bash
ros2 run tee_probe drl_controller_node.py \
  --model models/td3_tee_probe_final \
  --view mid_esophageal_4chamber \
  --rate 10.0
```

Change target view at runtime:
```bash
ros2 topic pub -1 /tee_probe/target_view \
  std_msgs/msg/String "data: 'transgastric_short_axis'"
```

---

## Standard TEE Views

| View Name | Depth | Rotation | Large Wheel | Small Wheel |
|---|---|---|---|---|
| `mid_esophageal_4chamber` | 30 cm | 0° | 0° | 0° |
| `mid_esophageal_2chamber` | 30 cm | 90° | 0° | 0° |
| `mid_esophageal_long_axis` | 30 cm | 135° | 0° | 0° |
| `transgastric_short_axis` | 40 cm | 0° | ~80° | 0° |

---

## Troubleshooting

### Controllers not appearing

```bash
ros2 control list_controllers
# If empty, check:
ros2 topic list | grep controller_manager
# Should see /controller_manager/...
```

Common causes:
1. **URDF `<ros2_control>` block missing** — the URDF must have the hardware interface block
2. **Gazebo plugin path wrong** — verify `libgazebo_ros2_control.so` is on `LD_LIBRARY_PATH`
3. **YAML `ros__parameters` missing** — double-underscore is mandatory in ROS2
4. **Controller name mismatch** — name in `controller_manager:` block must match spawner argument exactly

### Probe falls/collapses

Controllers haven't loaded yet. Check:
```bash
ros2 control list_controllers  # All 5 should be "active"
```

If controllers are active but probe still falls, PID gains may be too low. Increase `p` in `tee_controllers.yaml`.

### "No p gain specified for pid" (ROS1 error — gone in ROS2)

This ROS1 error is resolved. In ROS2, PID gains live inside each controller's `ros__parameters` block in `tee_controllers.yaml`.

### Joint not responding to commands

```bash
# Check topic exists
ros2 topic list | grep commands

# Check controller is active
ros2 control list_controllers | grep insertion

# Send a test command
ros2 topic pub -1 /insertion_controller/commands \
  std_msgs/msg/Float64MultiArray "data: [0.1]"

# Monitor joint states
ros2 topic echo /joint_states
```

### Wrong topic format (common ROS1→ROS2 mistake)

| | ROS1 | ROS2 |
|---|---|---|
| Message type | `std_msgs/Float64` | `std_msgs/Float64MultiArray` |
| Data field | `"data: 0.2"` | `"data: [0.2]"` |
| Topic name | `/ns/joint_name_position_controller/command` | `/controller_name/commands` |

---

## Dependencies Summary

```bash
# ROS2 packages
ros-humble-gazebo-ros-pkgs
ros-humble-gazebo-ros2-control
ros-humble-ros2-control
ros-humble-ros2-controllers
ros-humble-joint-state-broadcaster
ros-humble-position-controllers
ros-humble-robot-state-publisher

# Python (DRL)
stable-baselines3[extra]
gymnasium
tensorboard
numpy
```
