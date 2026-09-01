#!/usr/bin/env python3
"""
drl_controller_node.py
======================
Deploy a trained TD3 policy to control the 4-DOF TEE probe in Gazebo in real time.

Project : TEE Probe DRL  (UiO IFI / Rikshospitalet Intervention Centre)
Author  : Nikitha Neerupudi
Model   : models/best/best_model.zip  (TD3, Stable-Baselines3)

Architecture
------------
    /joint_states  ──►  this node  ──►  /insertion_controller/commands
    /tf (world→tip)     TD3 policy      /axial_rotation_controller/commands
                        inference       /large_wheel_controller/commands
                                        /small_wheel_controller/commands

Usage
-----
    # Terminal 1 (already working)
    ros2 launch tee_probe tee_simulation.launch.py

    # Terminal 2
    python3 src/tee_probe/scripts/drl_controller_node.py \
        --model models/best/best_model.zip \
        --view mid_esophageal_4chamber

    # Data collection for the paper: every view, 5 repeats, homing in between
    python3 src/tee_probe/scripts/drl_controller_node.py \
        --model models/best/best_model.zip \
        --view all --repeats 5 --home-between \
        --csv results/deployment_run1.csv

    # Goal can also be sent at runtime:
    ros2 topic pub --once /tee/goal_view std_msgs/String "{data: 'transgastric_short_axis'}"


>>> VERIFY THESE FIVE THINGS AGAINST tee_probe_env.py BEFORE TRUSTING RESULTS <<<

The policy will only behave as it did in training if the observation this node
builds is byte-for-byte the same construction as TEEProbeEnv._get_observation().
Check each of these and adjust the marked sections if they differ:

  1. OBSERVATION ORDER   — assumed [pos(4), vel(4), tip_xyz(3), joint_dist(1)].
  2. OBSERVATION SCALING — assumed raw values, no normalisation. If training used
                           VecNormalize, you MUST load the saved statistics here
                           (see load_vecnormalize() below) or the policy sees
                           garbage.
  3. TIP FRAME           — assumed TF "world" → "ultrasound_tip". If the env used
                           a different parent frame (e.g. "base_link"), change
                           --world-frame / --tip-frame.
  4. CONTROL PERIOD      — assumed 10 Hz. Set --rate to match the step period the
                           env actually used (whatever sleep / rate / physics-step
                           count TEEProbeEnv.step() applied). Timing mismatch is
                           the most common reason a deployed policy behaves worse
                           than it did in training.
  5. DISTANCE METRIC     — assumed plain Euclidean over the 4 joints with no angle
                           wrapping. If the env wrapped axial rotation into
                           [-pi, pi] before subtracting, set --wrap-axial.
"""

import argparse
import csv
import math
import os
import sys
import time
from datetime import datetime

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

import tf2_ros
from tf2_ros import TransformException

from stable_baselines3 import TD3


# ----------------------------------------------------------------------------
# Constants — must match tee_probe_env.py exactly
# ----------------------------------------------------------------------------

# Joint order everywhere in this file: [insertion, axial_rotation, large_wheel, small_wheel]
JOINT_ORDER = ["insertion", "axial_rotation", "large_wheel", "small_wheel"]

# Names as they appear in /joint_states. Override with --joint-names if these
# do not match your URDF (the node prints the names it actually receives).
DEFAULT_JOINT_NAMES = [
    "insertion_joint",
    "axial_rotation_joint",
    "large_wheel_joint",
    "small_wheel_joint",
]

# Controller command topics, same order as JOINT_ORDER
COMMAND_TOPICS = [
    "/insertion_controller/commands",
    "/axial_rotation_controller/commands",
    "/large_wheel_controller/commands",
    "/small_wheel_controller/commands",
]

ACTION_SCALE = np.array([0.2, math.pi, 1.5708, 1.5708], dtype=np.float64)
ACTION_OFFSET = np.array([0.2, 0.0, 0.0, 0.0], dtype=np.float64)

JOINT_MIN = np.array([0.0, -6.28, -1.5708, -1.5708], dtype=np.float64)
JOINT_MAX = np.array([0.4, 6.28, 1.5708, 1.5708], dtype=np.float64)

GOAL_TOLERANCE = 0.15          # joint-space, mixed rad/m
MAX_STEPS = 500                # same episode cap as training
HOME_POSITION = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float64)

STANDARD_TEE_VIEWS = {
    "mid_esophageal_4chamber":  (0.30, 0.000, 0.000, 0.000),
    "mid_esophageal_2chamber":  (0.30, 1.571, 0.000, 0.000),
    "mid_esophageal_long_axis": (0.30, 2.356, 0.000, 0.000),
    "transgastric_short_axis":  (0.40, 0.000, 1.400, 0.000),
}

OBS_DIM = 12
ACT_DIM = 4


# ----------------------------------------------------------------------------
# Helpers
# ----------------------------------------------------------------------------

def wrap_to_pi(angle: float) -> float:
    """Wrap an angle into [-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def joint_space_distance(pos: np.ndarray, goal: np.ndarray, wrap_axial: bool = False) -> float:
    """
    Euclidean distance in joint space — the exact metric used for the reward and
    the goal check during training (no FK approximation).

    wrap_axial=True treats the axial rotation joint as continuous, so 0.0 and
    2*pi count as the same configuration. Training almost certainly did NOT do
    this; only enable it if tee_probe_env.py does.
    """
    diff = pos - goal
    if wrap_axial:
        diff[1] = wrap_to_pi(diff[1])
    return float(np.linalg.norm(diff))


class EpisodeRecord:
    """One deployment episode: goal, per-step rows, and outcome."""

    def __init__(self, view_name: str, goal: np.ndarray, repeat: int):
        self.view_name = view_name
        self.goal = goal
        self.repeat = repeat
        self.rows = []
        self.steps = 0
        self.success = False
        self.final_distance = float("nan")
        self.min_distance = float("inf")
        self.wall_time = 0.0


# ----------------------------------------------------------------------------
# Node
# ----------------------------------------------------------------------------

class DRLControllerNode(Node):

    # Episode state machine
    WAITING = "waiting_for_data"
    HOMING = "homing"
    RUNNING = "running"
    FINISHED = "finished"

    def __init__(self, args):
        super().__init__("drl_controller_node")
        self.args = args

        self.joint_names = args.joint_names
        self.wrap_axial = args.wrap_axial
        self.tolerance = args.tolerance
        self.max_steps = args.max_steps

        # ---- policy ---------------------------------------------------------
        model_path = os.path.abspath(args.model)
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"Model not found: {model_path}")
        self.get_logger().info(f"Loading TD3 policy from {model_path} (device={args.device})")
        self.model = TD3.load(model_path, device=args.device)
        self._check_model_spaces()

        # Optional observation normalisation (see verification note 2 at top)
        self.vecnorm = self._load_vecnormalize(args.vecnormalize)

        # ---- state ----------------------------------------------------------
        self.joint_pos = None          # np.ndarray(4)
        self.joint_vel = None          # np.ndarray(4)
        self.last_tip_xyz = np.zeros(3, dtype=np.float64)
        self.tip_ever_seen = False
        self._warned_missing_joints = False
        self._warned_tf = False

        self.state = self.WAITING
        self.homing_until = 0.0
        self.episode_queue = self._build_episode_queue()
        self.current = None            # EpisodeRecord
        self.completed = []            # list[EpisodeRecord]
        self.episode_start_wall = 0.0

        # ---- ROS interfaces -------------------------------------------------
        # Best-effort/depth-1 is compatible with both reliable and best-effort
        # publishers, and we only ever care about the newest joint state.
        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, state_qos)
        self.create_subscription(String, "/tee/goal_view", self.on_goal_view, 10)

        self.cmd_pubs = [self.create_publisher(Float64MultiArray, t, 10) for t in COMMAND_TOPICS]

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        period = 1.0 / float(args.rate)
        self.create_timer(period, self.control_step)

        self.get_logger().info(
            f"Control rate {args.rate:.1f} Hz | tolerance {self.tolerance} | "
            f"max {self.max_steps} steps/episode | {len(self.episode_queue)} episode(s) queued"
        )
        self.get_logger().info("Waiting for /joint_states ...")

    # ------------------------------------------------------------------ setup

    def _check_model_spaces(self):
        obs_shape = self.model.observation_space.shape
        act_shape = self.model.action_space.shape
        if obs_shape != (OBS_DIM,):
            raise ValueError(
                f"Model expects observation shape {obs_shape}, this node builds ({OBS_DIM},). "
                "The observation construction here does not match the trained env."
            )
        if act_shape != (ACT_DIM,):
            raise ValueError(f"Model expects action shape {act_shape}, expected ({ACT_DIM},).")
        self.get_logger().info(f"Policy spaces OK: obs {obs_shape}, act {act_shape}")

    def _load_vecnormalize(self, path):
        """Load VecNormalize statistics if training used them. Returns None otherwise."""
        if not path:
            return None
        from stable_baselines3.common.vec_env import VecNormalize
        vn = VecNormalize.load(path, venv=None)
        vn.training = False
        self.get_logger().warn(f"Applying VecNormalize observation statistics from {path}")
        return vn

    def _build_episode_queue(self):
        """Expand --view / --goal / --repeats into a flat list of EpisodeRecords."""
        queue = []
        if self.args.goal is not None:
            goal = np.array(self.args.goal, dtype=np.float64)
            for r in range(self.args.repeats):
                queue.append(EpisodeRecord("custom_goal", goal.copy(), r + 1))
            return queue

        views = list(STANDARD_TEE_VIEWS.keys()) if self.args.view == "all" else [self.args.view]
        for r in range(self.args.repeats):
            for v in views:
                if v not in STANDARD_TEE_VIEWS:
                    raise ValueError(
                        f"Unknown view '{v}'. Choose from: {', '.join(STANDARD_TEE_VIEWS)} or 'all'."
                    )
                queue.append(EpisodeRecord(v, np.array(STANDARD_TEE_VIEWS[v], dtype=np.float64), r + 1))
        return queue

    # -------------------------------------------------------------- callbacks

    def on_joint_state(self, msg: JointState):
        """
        Map by NAME, never by index — /joint_states ordering is not guaranteed
        and ros2_control does not promise URDF order.
        """
        try:
            idx = [msg.name.index(n) for n in self.joint_names]
        except ValueError:
            if not self._warned_missing_joints:
                self._warned_missing_joints = True
                self.get_logger().error(
                    "Joint names not found in /joint_states.\n"
                    f"  expected : {self.joint_names}\n"
                    f"  received : {list(msg.name)}\n"
                    "  fix with : --joint-names name1 name2 name3 name4  (in the order "
                    "insertion, axial_rotation, large_wheel, small_wheel)"
                )
            return

        self.joint_pos = np.array([msg.position[i] for i in idx], dtype=np.float64)
        if msg.velocity and len(msg.velocity) > max(idx):
            self.joint_vel = np.array([msg.velocity[i] for i in idx], dtype=np.float64)
        else:
            self.joint_vel = np.zeros(4, dtype=np.float64)

    def on_goal_view(self, msg: String):
        """Runtime goal override via topic. Aborts the current episode."""
        view = msg.data.strip()
        if view not in STANDARD_TEE_VIEWS:
            self.get_logger().warn(f"Ignoring unknown view on /tee/goal_view: '{view}'")
            return
        self.get_logger().info(f"New goal received on /tee/goal_view: {view}")
        if self.current is not None and self.state == self.RUNNING:
            self._finish_episode(reason="preempted")
        self.episode_queue.insert(
            0, EpisodeRecord(view, np.array(STANDARD_TEE_VIEWS[view], dtype=np.float64), 1)
        )
        self.state = self.WAITING

    # ------------------------------------------------------------ observation

    def get_tip_xyz(self) -> np.ndarray:
        """
        Tip position in the world frame, straight from TF — the same source the
        training env used. Falls back to the last known value if a lookup fails
        so a single dropped transform does not corrupt the observation.
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                self.args.world_frame, self.args.tip_frame, rclpy.time.Time()
            )
            t = tf.transform.translation
            self.last_tip_xyz = np.array([t.x, t.y, t.z], dtype=np.float64)
            self.tip_ever_seen = True
        except TransformException as exc:
            if not self._warned_tf:
                self._warned_tf = True
                self.get_logger().warn(
                    f"TF lookup {self.args.world_frame} -> {self.args.tip_frame} failed: {exc}. "
                    "Using last known tip position. If this never resolves, the observation "
                    "is wrong and the policy output is meaningless."
                )
        return self.last_tip_xyz

    def build_observation(self, dist: float) -> np.ndarray:
        """
        12D observation — MUST match TEEProbeEnv._get_observation().
        Layout: [joint_pos(4), joint_vel(4), tip_xyz(3), joint_dist(1)]
        """
        obs = np.concatenate([
            self.joint_pos,
            self.joint_vel,
            self.get_tip_xyz(),
            np.array([dist]),
        ]).astype(np.float32)

        if self.vecnorm is not None:
            obs = self.vecnorm.normalize_obs(obs)
        return obs

    # --------------------------------------------------------------- commands

    def publish_joint_targets(self, targets: np.ndarray):
        """One Float64MultiArray per JointGroupPositionController (single joint each)."""
        for pub, value in zip(self.cmd_pubs, targets):
            msg = Float64MultiArray()
            msg.data = [float(value)]
            pub.publish(msg)

    # ---------------------------------------------------------- control loop

    def control_step(self):
        if self.joint_pos is None:
            return

        if self.state == self.WAITING:
            self._start_next_episode()
            return

        if self.state == self.HOMING:
            self.publish_joint_targets(HOME_POSITION)
            if time.time() >= self.homing_until:
                self.state = self.WAITING
            return

        if self.state != self.RUNNING:
            return

        ep = self.current
        dist = joint_space_distance(self.joint_pos, ep.goal, self.wrap_axial)
        ep.min_distance = min(ep.min_distance, dist)

        # --- goal check BEFORE acting, so we stop cleanly on arrival ---------
        if dist < self.tolerance:
            ep.final_distance = dist
            ep.success = True
            self._finish_episode(reason="goal reached")
            return

        if ep.steps >= self.max_steps:
            ep.final_distance = dist
            ep.success = False
            self._finish_episode(reason=f"timeout after {self.max_steps} steps")
            return

        # --- policy inference ------------------------------------------------
        obs = self.build_observation(dist)
        action, _ = self.model.predict(obs, deterministic=True)
        action = np.asarray(action, dtype=np.float64).reshape(ACT_DIM)

        # Actions are ABSOLUTE joint targets, not deltas:
        #   [-1, 1] -> [OFFSET - SCALE, OFFSET + SCALE], which spans the joint range.
        targets = np.clip(action * ACTION_SCALE + ACTION_OFFSET, JOINT_MIN, JOINT_MAX)
        self.publish_joint_targets(targets)

        ep.steps += 1
        ep.rows.append({
            "view": ep.view_name,
            "repeat": ep.repeat,
            "step": ep.steps,
            "t": round(time.time() - self.episode_start_wall, 4),
            "pos_insertion": self.joint_pos[0],
            "pos_axial": self.joint_pos[1],
            "pos_large": self.joint_pos[2],
            "pos_small": self.joint_pos[3],
            "vel_insertion": self.joint_vel[0],
            "vel_axial": self.joint_vel[1],
            "vel_large": self.joint_vel[2],
            "vel_small": self.joint_vel[3],
            "tip_x": self.last_tip_xyz[0],
            "tip_y": self.last_tip_xyz[1],
            "tip_z": self.last_tip_xyz[2],
            "joint_dist": dist,
            "cmd_insertion": targets[0],
            "cmd_axial": targets[1],
            "cmd_large": targets[2],
            "cmd_small": targets[3],
            "goal_insertion": ep.goal[0],
            "goal_axial": ep.goal[1],
            "goal_large": ep.goal[2],
            "goal_small": ep.goal[3],
        })

        if ep.steps % 25 == 0:
            self.get_logger().info(f"  step {ep.steps:3d}  dist {dist:.4f}  (best {ep.min_distance:.4f})")

    # ------------------------------------------------------- episode handling

    def _start_next_episode(self):
        if not self.episode_queue:
            self._shutdown_cleanly()
            return
        self.current = self.episode_queue.pop(0)
        self.state = self.RUNNING
        self.episode_start_wall = time.time()
        g = self.current.goal
        self.get_logger().info(
            f"--- {self.current.view_name} (repeat {self.current.repeat}) --- "
            f"goal [{g[0]:.3f} m, {g[1]:.3f} rad, {g[2]:.3f} rad, {g[3]:.3f} rad]"
        )

    def _finish_episode(self, reason: str):
        ep = self.current
        if ep is None:
            return
        ep.wall_time = time.time() - self.episode_start_wall
        if math.isnan(ep.final_distance):
            ep.final_distance = joint_space_distance(self.joint_pos, ep.goal, self.wrap_axial)

        verdict = "SUCCESS" if ep.success else "FAILED "
        self.get_logger().info(
            f"{verdict} {ep.view_name} (repeat {ep.repeat}): {reason} | "
            f"{ep.steps} steps | final dist {ep.final_distance:.4f} | "
            f"min dist {ep.min_distance:.4f} | {ep.wall_time:.1f} s"
        )

        # Hold the final pose rather than leaving the last command mid-motion.
        self.publish_joint_targets(np.clip(self.joint_pos, JOINT_MIN, JOINT_MAX))

        self.completed.append(ep)
        self.current = None

        if self.episode_queue and self.args.home_between:
            self.get_logger().info(f"Homing for {self.args.home_time:.1f} s ...")
            self.homing_until = time.time() + self.args.home_time
            self.state = self.HOMING
        else:
            self.state = self.WAITING

    def _shutdown_cleanly(self):
        self.state = self.FINISHED
        self.print_summary()
        self.save_csv()
        self.get_logger().info("All episodes complete. Shutting down.")
        rclpy.shutdown()

    # ----------------------------------------------------------- results / IO

    def print_summary(self):
        if not self.completed:
            return
        print("\n" + "=" * 74)
        print("DEPLOYMENT SUMMARY")
        print("=" * 74)
        print(f"{'View':<28}{'Success':>10}{'Mean steps':>13}{'Mean final d':>14}")
        print("-" * 74)

        by_view = {}
        for ep in self.completed:
            by_view.setdefault(ep.view_name, []).append(ep)

        for view, eps in by_view.items():
            n = len(eps)
            hits = sum(1 for e in eps if e.success)
            succeeded = [e for e in eps if e.success]
            mean_steps = np.mean([e.steps for e in succeeded]) if succeeded else float("nan")
            mean_dist = np.mean([e.final_distance for e in eps])
            print(f"{view:<28}{hits}/{n:<8}{mean_steps:>13.1f}{mean_dist:>14.4f}")

        total = len(self.completed)
        hits = sum(1 for e in self.completed if e.success)
        print("-" * 74)
        print(f"{'OVERALL':<28}{hits}/{total:<8}  ({100.0 * hits / total:.0f}% goal rate)")
        print("=" * 74 + "\n")

    def save_csv(self):
        if not self.args.csv or not self.completed:
            return
        path = os.path.abspath(self.args.csv)
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)

        rows = [r for ep in self.completed for r in ep.rows]
        if not rows:
            self.get_logger().warn("No steps recorded — nothing written to CSV.")
            return

        with open(path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)
        self.get_logger().info(f"Wrote {len(rows)} rows to {path}")

        # Per-episode summary alongside the trajectory file, for the results table.
        summary_path = path.replace(".csv", "_summary.csv")
        with open(summary_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "view", "repeat", "success", "steps",
                "final_distance", "min_distance", "wall_time_s", "timestamp",
            ])
            stamp = datetime.now().isoformat(timespec="seconds")
            for ep in self.completed:
                writer.writerow([
                    ep.view_name, ep.repeat, int(ep.success), ep.steps,
                    f"{ep.final_distance:.6f}", f"{ep.min_distance:.6f}",
                    f"{ep.wall_time:.2f}", stamp,
                ])
        self.get_logger().info(f"Wrote episode summary to {summary_path}")


# ----------------------------------------------------------------------------
# Entry point
# ----------------------------------------------------------------------------

def parse_args(argv):
    p = argparse.ArgumentParser(
        description="Deploy a trained TD3 policy to drive the TEE probe in Gazebo.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--model", default="models/best/best_model.zip",
                   help="Path to the trained SB3 TD3 .zip")
    p.add_argument("--view", default="mid_esophageal_4chamber",
                   help="Target view name, or 'all' to run every standard view")
    p.add_argument("--goal", type=float, nargs=4, default=None,
                   metavar=("INS", "AXIAL", "LARGE", "SMALL"),
                   help="Explicit joint-space goal; overrides --view")
    p.add_argument("--repeats", type=int, default=1,
                   help="Repeats per view (for per-view success statistics)")
    p.add_argument("--rate", type=float, default=10.0,
                   help="Control frequency in Hz — set this to the training step rate")
    p.add_argument("--tolerance", type=float, default=GOAL_TOLERANCE,
                   help="Joint-space goal tolerance")
    p.add_argument("--max-steps", type=int, default=MAX_STEPS,
                   help="Episode step cap before declaring failure")
    p.add_argument("--home-between", action="store_true",
                   help="Drive back to the home pose between episodes")
    p.add_argument("--home-time", type=float, default=3.0,
                   help="Seconds to hold the home command")
    p.add_argument("--csv", default=None,
                   help="Write per-step trajectory CSV (plus a _summary.csv) here")
    p.add_argument("--joint-names", nargs=4, default=DEFAULT_JOINT_NAMES,
                   help="Names in /joint_states, ordered insertion axial large small")
    p.add_argument("--world-frame", default="world", help="TF parent frame")
    p.add_argument("--tip-frame", default="ultrasound_tip", help="TF tip frame")
    p.add_argument("--wrap-axial", action="store_true",
                   help="Wrap axial rotation error to [-pi, pi] in the distance metric")
    p.add_argument("--vecnormalize", default=None,
                   help="Path to vecnormalize.pkl if training normalised observations")
    p.add_argument("--device", default="cpu", choices=["cpu", "cuda", "auto"],
                   help="Torch device for inference")
    return p.parse_known_args(argv)


def main():
    args, ros_argv = parse_args(sys.argv[1:])
    rclpy.init(args=ros_argv)

    node = None
    try:
        node = DRLControllerNode(args)
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info("Interrupted — saving what we have.")
            if node.current is not None:
                node._finish_episode(reason="user interrupt")
            node.print_summary()
            node.save_csv()
    except Exception as exc:  # noqa: BLE001 - surface setup errors clearly
        print(f"\n[drl_controller_node] FATAL: {exc}\n", file=sys.stderr)
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
