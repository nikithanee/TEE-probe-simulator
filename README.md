# TEE Probe Simulator

Autonomous navigation of a 4-DOF Transesophageal Echocardiography (TEE) probe using deep reinforcement learning, built in ROS 2 Humble and Gazebo.

A TEE probe is a flexible ultrasound device inserted through the esophagus during cardiac surgery. Positioning it correctly requires an operator to coordinate insertion depth, axial rotation, and two independent tip-flexion controls simultaneously. This project trains a reinforcement learning policy to acquire standard cardiac imaging views autonomously, as a step toward reducing operator workload and positioning error in cardiac procedures.

Developed as research at **Rikshospitalet, The Intervention Centre** (Oslo University Hospital) in collaboration with the **University of Oslo**.

<img width="800" height="366" alt="ScreenRecording2026-08-14at18 36 29-ezgif com-video-to-gif-converter" src="https://github.com/user-attachments/assets/7ccc7fba-6e6d-4317-b8d5-1ad9d224350a" />


---

## Kinematics

The probe is modeled as a 4-DOF continuum manipulator:

| DOF | Joint type | Range | Function |
|---|---|---|---|
| Insertion | Prismatic | 0 – 0.4 m | Advance / withdraw through esophagus |
| Axial rotation | Continuous | ±π | Orient the imaging plane |
| Large wheel | Revolute | ±90° | Anterior / posterior tip flexion |
| Small wheel | Revolute | ±90° | Lateral tip flexion |

---

## Results

Trained TD3 policy deployed to the Gazebo simulation at 20 Hz, 5 repeats per target view:

| View | Success | Mean steps | Mean final distance (m) |
|---|---|---|---|
| Mid-esophageal 4-chamber | 5/5 | 7.0 | 0.1325 |
| Mid-esophageal 2-chamber | 5/5 | 6.8 | 0.1270 |
| Mid-esophageal long-axis | 5/5 | 7.4 | 0.0962 |
| Transgastric short-axis | 0/5 | — | 0.6079 |
| **Overall** | **15/20 (75%)** | | |

Raw data: `results/deployment_full.csv` and `results/deployment_full_summary.csv`.

**Known limitation.** The transgastric short-axis view fails consistently. The policy enters a visible limit cycle, repeating the same arc without converging, and times out at 500 steps. This target sits at the edge of the reachable workspace given the current joint limits, and characterizing this failure mode is ongoing work.

---

## Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic 11 with `gazebo_ros2_control`
- Python 3.10+
- MuJoCo, Gymnasium, Stable-Baselines3, PyTorch
- Foxglove Studio (optional, for visualization)

```bash
sudo apt install ros-humble-desktop ros-humble-gazebo-ros-pkgs \
                 ros-humble-gazebo-ros2-control ros-humble-ros2-control \
                 ros-humble-ros2-controllers ros-humble-foxglove-bridge

pip install mujoco gymnasium stable-baselines3[extra] torch tensorboard
```

---

## Build

```bash
git clone https://github.com/nikithanee/TEE-probe-simulator.git tee_probe_ws
cd tee_probe_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Usage

### Run the simulation

```bash
ros2 launch tee_probe tee_simulation.launch.py
```

Useful arguments:

```bash
ros2 launch tee_probe tee_simulation.launch.py gui:=false        # headless
ros2 launch tee_probe tee_simulation.launch.py world:=path/to/world.sdf
```

The launch file brings up Gazebo, the robot state publisher, spawns the probe, then starts the joint state broadcaster followed by one position controller per DOF.

Verify everything came up:

```bash
ros2 control list_controllers    # all 5 should report "active"
```

### Move a joint manually

```bash
ros2 topic pub -1 /insertion_controller/commands \
  std_msgs/msg/Float64MultiArray "data: [0.30]"
```

### Train a policy

```bash
# Terminal 1 — headless sim
ros2 launch tee_probe tee_simulation.launch.py gui:=false

# Terminal 2 — training
python3 src/tee_probe/scripts/train_tee_probe.py --timesteps 200000

# Monitor
tensorboard --logdir logs/td3_tee_probe
```

Resume or evaluate:

```bash
python3 scripts/train_tee_probe.py --resume models/td3_tee_probe_100000_steps --timesteps 200000
python3 scripts/train_tee_probe.py --eval models/best/best_model
```

### Deploy a trained policy

```bash
ros2 run tee_probe drl_controller_node.py \
  --model models/td3_tee_probe_final \
  --view mid_esophageal_4chamber \
  --rate 10.0
```

Change the target view at runtime:

```bash
ros2 topic pub -1 /tee_probe/target_view std_msgs/msg/String \
  "data: 'transgastric_short_axis'"
```

---

## Standard TEE views

| View key | Depth | Axial rotation | Large wheel | Small wheel |
|---|---|---|---|---|
| `mid_esophageal_4chamber` | 30 cm | 0° | 0° | 0° |
| `mid_esophageal_2chamber` | 30 cm | 90° | 0° | 0° |
| `mid_esophageal_long_axis` | 30 cm | 135° | 0° | 0° |
| `transgastric_short_axis` | 40 cm | 0° | ~80° | 0° |

---

## Learning setup

**Observation (12-D):** four joint positions, four joint velocities, tip position in Cartesian space (x, y, z), and Euclidean distance to the target pose.

**Action (4-D):** normalized joint commands in [-1, 1], rescaled to each joint's physical range.

**Reward:** negative distance to the target pose, a bonus on reaching within 1 cm, and a small penalty on action change to encourage smooth trajectories.

**Algorithm:** TD3 (Stable-Baselines3), trained in a MuJoCo/Gymnasium environment and deployed to the Gazebo simulation for evaluation.

---

## Repository structure

```
tee_probe_ws/
└── src/
    └── tee_probe/
        ├── config/       # tee_controllers.yaml — ros2_control controller definitions
        ├── launch/       # tee_simulation.launch.py, tee_training.launch.py
        ├── urdf/         # tee_probe.urdf — 4-DOF chain + ros2_control block
        ├── worlds/       # Gazebo world files
        ├── scripts/      # train_tee_probe.py, drl_controller_node.py, environment
        ├── models/       # trained policy checkpoints
        ├── results/      # deployment CSVs
        ├── package.xml
        └── CMakeLists.txt
```

---

## Troubleshooting

**Controllers do not appear.** Run `ros2 control list_controllers`. If the list is empty, check that the `<ros2_control>` block is present in the URDF, that `libgazebo_ros2_control.so` is on `LD_LIBRARY_PATH`, and that every controller name in `tee_controllers.yaml` matches the spawner argument exactly.

**Probe collapses on spawn.** The controllers have not loaded yet, or the PID gains are too low. Confirm all five controllers are active, then raise `p` in `tee_controllers.yaml`.

**`ros__parameters` errors in YAML.** The double underscore is mandatory in ROS 2 and is a common carry-over mistake from ROS 1 config files.

---

## Status and next steps

- Characterize the transgastric limit cycle and the reachability limits that cause it
- Add baselines: analytic IK, a scripted controller, and multiple training seeds
- Move toward image-based or image-proxy observations, since a real operator navigates from the ultrasound image rather than ground-truth tip pose
- Phantom validation at The Intervention Centre

---

## Acknowledgements

Supervisor: **Prof. Ole Jakob Elle**, University of Oslo / The Intervention Centre, Oslo University Hospital.
AI and reinforcement learning guidance: **Afsah Asif Rashid**.

## Author

**Nikitha Neerupudi** — BSc Informatics: Robotics and Intelligent Systems, University of Oslo
[neerupudi.com](https://neerupudi.com) · [github.com/nikithanee](https://github.com/nikithanee)
