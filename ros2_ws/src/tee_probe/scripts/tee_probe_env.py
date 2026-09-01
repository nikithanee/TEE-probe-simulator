#!/usr/bin/env python3
"""
TEE Probe Gym Environment - ROS2 Humble
========================================
OpenAI Gym-compatible environment that interfaces with the
ROS2 Gazebo simulation for DRL training.
 
State space  (12D): joint positions (4) + velocities (4) + tip xyz (3) + dist_to_goal (1)
Action space  (4D): normalized position commands [-1, 1] for each DOF
Reward: dense negative distance + progress bonus + goal bonus + smoothness penalty
 
Usage:
  from tee_probe_env import TEEProbeEnv
  env = TEEProbeEnv()
  obs, _ = env.reset()
  obs, reward, terminated, truncated, info = env.step(action)
"""
 
import math
import time
import numpy as np
 
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
 
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
import tf2_ros
 
import gymnasium as gym
from gymnasium import spaces
 
 
# Standard TEE imaging views (insertion_m, rotation_rad, large_rad, small_rad)
STANDARD_TEE_VIEWS = {
    "mid_esophageal_4chamber":  (0.30,  0.000,  0.000,  0.000),
    "mid_esophageal_2chamber":  (0.30,  1.571,  0.000,  0.000),
    "mid_esophageal_long_axis": (0.30,  2.356,  0.000,  0.000),
    "transgastric_short_axis":  (0.40,  0.000,  1.400,  0.000),
}
VIEW_NAMES = list(STANDARD_TEE_VIEWS.keys())
 
# Action scaling (maps [-1,1] -> physical units)
ACTION_SCALE  = np.array([0.2, math.pi, 1.5708, 1.5708], dtype=np.float32)
ACTION_OFFSET = np.array([0.2, 0.0,     0.0,    0.0],    dtype=np.float32)
 
# Joint limits
JOINT_MIN = np.array([0.0,  -6.28,  -1.5708, -1.5708], dtype=np.float32)
JOINT_MAX = np.array([0.4,   6.28,   1.5708,  1.5708], dtype=np.float32)
 
# Goal tolerance in joint space (mixed units: metres + radians)
GOAL_TOLERANCE = 0.10
 
 
class TEEProbeEnv(gym.Env):
    """
    Gym environment wrapping the ROS2 TEE probe simulation.
 
    Observation (12,):
      [0]    insertion_joint position (m)
      [1]    axial_rotation_joint position (rad)
      [2]    large_wheel_joint position (rad)
      [3]    small_wheel_joint position (rad)
      [4]    insertion_joint velocity (m/s)
      [5]    axial_rotation_joint velocity (rad/s)
      [6]    large_wheel_joint velocity (rad/s)
      [7]    small_wheel_joint velocity (rad/s)
      [8-10] tip_x, tip_y, tip_z (m)
      [11]   joint-space distance to goal
 
    Action (4,) in [-1, 1]:
      Scaled to joint ranges via ACTION_SCALE and ACTION_OFFSET.
    """
 
    metadata = {"render_modes": ["human"]}
 
    def __init__(self, node: Node = None, max_steps: int = 500):
        super().__init__()
 
        self.max_steps      = max_steps
        self._step_count    = 0
        self._goal_position = np.zeros(4, dtype=np.float32)
        self._goal_tip_xyz  = np.zeros(3, dtype=np.float32)  # unused
        self._prev_action   = np.zeros(4, dtype=np.float32)
        self._prev_dist     = 0.0
 
        # ROS2 node
        self._owns_node = node is None
        if self._owns_node:
            if not rclpy.ok():
                rclpy.init()
            self._node = rclpy.create_node("tee_probe_env")
        else:
            self._node = node
 
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
 
        # Publishers
        self._pub_insertion   = self._node.create_publisher(Float64MultiArray, "/insertion_controller/commands",      qos)
        self._pub_axial       = self._node.create_publisher(Float64MultiArray, "/axial_rotation_controller/commands", qos)
        self._pub_large_wheel = self._node.create_publisher(Float64MultiArray, "/large_wheel_controller/commands",    qos)
        self._pub_small_wheel = self._node.create_publisher(Float64MultiArray, "/small_wheel_controller/commands",    qos)
 
        # Subscriber
        self._joint_pos   = np.zeros(4, dtype=np.float32)
        self._joint_vel   = np.zeros(4, dtype=np.float32)
        self._joint_names = ["insertion_joint", "axial_rotation_joint",
                             "large_wheel_joint", "small_wheel_joint"]
        self._js_received = False
        self._node.create_subscription(JointState, "/joint_states", self._joint_state_cb, qos)
 
        # TF listener for tip position
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self._node)
        self._tip_xyz     = np.zeros(3, dtype=np.float32)
 
        # Observation space
        obs_low  = np.array([0.0,  -6.28, -1.5708, -1.5708,
                             -0.5,  -5.0,  -2.0,   -2.0,
                             -2.0,  -2.0,  -2.0,
                              0.0], dtype=np.float32)
        obs_high = np.array([0.4,   6.28,  1.5708,  1.5708,
                              0.5,  5.0,   2.0,     2.0,
                              2.0,  2.0,   2.0,
                             10.0], dtype=np.float32)  # 10.0 for joint-space dist
 
        self.observation_space = spaces.Box(obs_low, obs_high, dtype=np.float32)
        self.action_space      = spaces.Box(low=-1.0, high=1.0, shape=(4,), dtype=np.float32)
 
        self._node.get_logger().info("TEEProbeEnv initialised (ROS2 Humble).")
 
    # ------------------------------------------------------------------
    # ROS2 callbacks
    # ------------------------------------------------------------------
 
    def _joint_state_cb(self, msg: JointState):
        for i, name in enumerate(self._joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self._joint_pos[i] = msg.position[idx]
                if len(msg.velocity) > idx:
                    self._joint_vel[i] = msg.velocity[idx]
        self._js_received = True
 
    def _get_tip_xyz(self):
        try:
            t = self._tf_buffer.lookup_transform(
                "world", "ultrasound_tip", rclpy.time.Time()
            )
            self._tip_xyz[:] = [
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z,
            ]
        except Exception:
            pass  # keep last known value
        return self._tip_xyz.copy()
 
    # ------------------------------------------------------------------
    # Gym interface
    # ------------------------------------------------------------------
 
    def _spin_once(self, timeout_sec=0.05):
        rclpy.spin_once(self._node, timeout_sec=timeout_sec)
 
    def _send_action(self, joint_commands: np.ndarray):
        def pub(publisher, value):
            msg = Float64MultiArray()
            msg.data = [float(value)]
            publisher.publish(msg)
        pub(self._pub_insertion,   joint_commands[0])
        pub(self._pub_axial,       joint_commands[1])
        pub(self._pub_large_wheel, joint_commands[2])
        pub(self._pub_small_wheel, joint_commands[3])
 
    def _wait_for_joint_states(self, timeout=5.0):
        t0 = time.time()
        while not self._js_received and (time.time() - t0) < timeout:
            self._spin_once()
        if not self._js_received:
            self._node.get_logger().warn("Timeout waiting for /joint_states")
 
    def _get_observation(self):
        tip_xyz   = self._get_tip_xyz()
        joint_dist = np.linalg.norm(self._joint_pos - self._goal_position)
        obs = np.concatenate([
            self._joint_pos,
            self._joint_vel,
            tip_xyz,
            [joint_dist],
        ]).astype(np.float32)
        return obs, joint_dist
 
    def _action_to_joints(self, action: np.ndarray) -> np.ndarray:
        raw = action * ACTION_SCALE + ACTION_OFFSET
        return np.clip(raw, JOINT_MIN, JOINT_MAX)
 
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self._step_count  = 0
        self._prev_action = np.zeros(4, dtype=np.float32)
 
        # Choose random goal view
        view_name = self.np_random.choice(VIEW_NAMES)
        goal = STANDARD_TEE_VIEWS[view_name]
        self._goal_position = np.array(goal, dtype=np.float32)
        self._goal_tip_xyz  = np.zeros(3, dtype=np.float32)  # unused
 
        # Send home command and wait for physics to settle
        self._send_action(np.zeros(4))
        time.sleep(0.3)
 
        # Get fresh joint states
        self._js_received = False
        self._wait_for_joint_states()
        self._spin_once()
 
        obs, dist = self._get_observation()
        self._prev_dist = dist  # initialise to actual distance from goal
 
        info = {"goal_view": view_name, "goal_joints": goal}
        return obs, info
 
    def step(self, action: np.ndarray):
        self._step_count += 1
 
        joint_cmds = self._action_to_joints(action)
        self._send_action(joint_cmds)
 
        time.sleep(0.05)
        self._spin_once()
        obs, dist = self._get_observation()
 
        # Reward
        reward  = -5.0 * dist                          # distance penalty
        reward += 2.0 * (self._prev_dist - dist)       # progress bonus
        if dist < GOAL_TOLERANCE:
            reward += 100.0                            # goal bonus
 
        action_delta = np.linalg.norm(action - self._prev_action)
        reward -= 0.05 * action_delta                  # smoothness penalty
 
        self._prev_dist = dist
 
        # Soft joint limit penalty
        margin = 0.05
        for i in range(4):
            if (joint_cmds[i] < JOINT_MIN[i] + margin or
                    joint_cmds[i] > JOINT_MAX[i] - margin):
                reward -= 1.0
 
        self._prev_action = action.copy()
 
        terminated = bool(dist < GOAL_TOLERANCE)
        truncated  = bool(self._step_count >= self.max_steps)
        info = {
            "distance_to_goal": dist,
            "joint_positions":  self._joint_pos.copy(),
            "tip_xyz":          self._tip_xyz.copy(),
        }
 
        return obs, reward, terminated, truncated, info
 
    def render(self):
        pass  # Gazebo provides visualisation
 
    def close(self):
        if self._owns_node:
            self._node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
 
