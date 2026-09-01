#!/usr/bin/env python3
"""
TEE Probe DRL Training Script - ROS2 Humble
=============================================
Trains a TD3 (Twin Delayed Deep Deterministic Policy Gradient) agent
to autonomously navigate the TEE probe to standard cardiac imaging views.

Prerequisites:
  pip install stable-baselines3 gymnasium tensorboard

Usage:
  # From your ROS2 workspace, with the simulation running:
  ros2 launch tee_probe tee_simulation.launch.py gui:=false &
  python3 scripts/train_tee_probe.py

  # Resume from checkpoint:
  python3 scripts/train_tee_probe.py --resume models/td3_tee_probe_50000_steps

  # Custom settings:
  python3 scripts/train_tee_probe.py --timesteps 500000 --learning-rate 3e-4
"""

import argparse
import os
import sys
import time
from pathlib import Path

import numpy as np
import rclpy

# Stable-Baselines3
try:
    from stable_baselines3 import TD3, SAC
    from stable_baselines3.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise
    from stable_baselines3.common.callbacks import (
        CheckpointCallback,
        EvalCallback,
        StopTrainingOnRewardThreshold,
    )
    from stable_baselines3.common.monitor import Monitor
    from stable_baselines3.common.vec_env import DummyVecEnv
    from stable_baselines3.common.logger import configure
except ImportError:
    print("ERROR: stable-baselines3 not installed.")
    print("  pip install stable-baselines3[extra]")
    sys.exit(1)

from tee_probe_env import TEEProbeEnv


# -----------------------------------------------------------------------
# Hyper-parameters (good starting point, tune as needed)
# -----------------------------------------------------------------------
DEFAULT_CONFIG = {
    "algorithm":         "TD3",        # TD3 or SAC
    "total_timesteps":   1_000_000,
    "learning_rate": 3e-4,  # was 1e-3
    "batch_size":        256,
    "buffer_size":       100_000,
    "learning_starts":   5_000,        # random exploration before training
    "gamma":             0.99,         # discount factor
    "tau":               0.005,        # soft target update
    "policy":            "MlpPolicy",
    "policy_kwargs":     dict(net_arch=[256, 256]),   # 2-layer 256-unit MLP
    "train_freq":        1,
    "gradient_steps":    1,
    # TD3-specific
    "noise_type":        "normal",     # "normal" or "ou" (Ornstein-Uhlenbeck)
    "noise_sigma":       0.1,
    # Evaluation
    "eval_freq":         5_000,
    "n_eval_episodes":   5,
    # Checkpointing
    "checkpoint_freq":   10_000,
    "log_dir":           "logs/td3_tee_probe",
    "model_dir":         "models",
    "model_name":        "td3_tee_probe",
}


def make_env():
    """Factory for creating the gym environment (for VecEnv compatibility)."""
    def _init():
        env = TEEProbeEnv()
        env = Monitor(env)
        return env
    return _init


def build_noise(action_dim: int, noise_type: str, sigma: float):
    mean = np.zeros(action_dim)
    if noise_type == "ou":
        return OrnsteinUhlenbeckActionNoise(mean=mean, sigma=sigma * np.ones(action_dim))
    return NormalActionNoise(mean=mean, sigma=sigma * np.ones(action_dim))


def train(config: dict, resume_path: str = None):
    # -----------------------------------------------------------------------
    # Directories
    # -----------------------------------------------------------------------
    os.makedirs(config["log_dir"],   exist_ok=True)
    os.makedirs(config["model_dir"], exist_ok=True)

    # -----------------------------------------------------------------------
    # Environment
    # -----------------------------------------------------------------------
    print("[TRAIN] Creating environment...")
    env = DummyVecEnv([make_env()])
    eval_env = DummyVecEnv([make_env()])

    # -----------------------------------------------------------------------
    # Model
    # -----------------------------------------------------------------------
    action_noise = build_noise(
        action_dim=4,
        noise_type=config["noise_type"],
        sigma=config["noise_sigma"],
    )

    model_kwargs = dict(
        policy=config["policy"],
        env=env,
        learning_rate=config["learning_rate"],
        buffer_size=config["buffer_size"],
        learning_starts=config["learning_starts"],
        batch_size=config["batch_size"],
        gamma=config["gamma"],
        tau=config["tau"],
        train_freq=config["train_freq"],
        gradient_steps=config["gradient_steps"],
        action_noise=action_noise,
        policy_kwargs=config["policy_kwargs"],
        verbose=1,
        tensorboard_log=config["log_dir"],
    )

    if resume_path:
        print(f"[TRAIN] Resuming from: {resume_path}")
        model = TD3.load(resume_path, env=env, **{
            k: v for k, v in model_kwargs.items()
            if k not in ("policy", "env")
        })
    else:
        print(f"[TRAIN] Building new {config['algorithm']} model...")
        AlgoClass = TD3 if config["algorithm"] == "TD3" else SAC
        model = AlgoClass(**model_kwargs)

    # -----------------------------------------------------------------------
    # Callbacks
    # -----------------------------------------------------------------------
    checkpoint_cb = CheckpointCallback(
        save_freq=config["checkpoint_freq"],
        save_path=config["model_dir"],
        name_prefix=config["model_name"],
        verbose=1,
    )

    # Stop training if mean reward exceeds threshold (goal reached consistently)
    stop_cb = StopTrainingOnRewardThreshold(reward_threshold=80.0, verbose=1)

    eval_cb = EvalCallback(
        eval_env,
        best_model_save_path=os.path.join(config["model_dir"], "best"),
        log_path=config["log_dir"],
        eval_freq=config["eval_freq"],
        n_eval_episodes=config["n_eval_episodes"],
        deterministic=True,
        render=False,
        callback_on_new_best=stop_cb,
        verbose=1,
    )

    callbacks = [checkpoint_cb, eval_cb]

    # -----------------------------------------------------------------------
    # Training
    # -----------------------------------------------------------------------
    print(f"\n[TRAIN] Starting training for {config['total_timesteps']:,} timesteps...")
    print(f"[TRAIN] TensorBoard logs: {config['log_dir']}")
    print(f"[TRAIN] Models saved to:  {config['model_dir']}")
    print("[TRAIN] Monitor with: tensorboard --logdir", config["log_dir"])
    print()

    t0 = time.time()
    model.learn(
        total_timesteps=config["total_timesteps"],
        callback=callbacks,
        progress_bar=True,
        reset_num_timesteps=resume_path is None,
    )
    elapsed = time.time() - t0

    # -----------------------------------------------------------------------
    # Save final model
    # -----------------------------------------------------------------------
    final_path = os.path.join(config["model_dir"], f"{config['model_name']}_final")
    model.save(final_path)
    print(f"\n[TRAIN] Training complete in {elapsed/60:.1f} minutes.")
    print(f"[TRAIN] Final model saved: {final_path}.zip")

    return model


def evaluate(model_path: str, n_episodes: int = 10):
    """Evaluate a saved model and print per-episode stats."""
    print(f"\n[EVAL] Loading model: {model_path}")
    env = TEEProbeEnv()
    model = TD3.load(model_path)

    episode_rewards = []
    episode_lengths = []
    goal_reached    = 0

    for ep in range(n_episodes):
        obs, _ = env.reset()
        ep_reward = 0.0
        step = 0
        done = False

        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env.step(action)
            ep_reward += reward
            step += 1
            done = terminated or truncated

        episode_rewards.append(ep_reward)
        episode_lengths.append(step)
        if info.get("distance_to_goal", 1.0) < 0.01:
            goal_reached += 1

        print(f"  Episode {ep+1:3d}: reward={ep_reward:8.2f}  "
              f"steps={step:4d}  dist={info.get('distance_to_goal', -1):.4f} m")

    print(f"\n[EVAL] Summary over {n_episodes} episodes:")
    print(f"  Mean reward:  {np.mean(episode_rewards):.2f} ± {np.std(episode_rewards):.2f}")
    print(f"  Mean length:  {np.mean(episode_lengths):.1f} steps")
    print(f"  Goal rate:    {goal_reached}/{n_episodes} ({100*goal_reached/n_episodes:.0f}%)")
    env.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Train TD3 agent on TEE probe sim")
    parser.add_argument("--timesteps",     type=int,   default=DEFAULT_CONFIG["total_timesteps"])
    parser.add_argument("--learning-rate", type=float, default=DEFAULT_CONFIG["learning_rate"])
    parser.add_argument("--resume",        type=str,   default=None,
                        help="Path to saved model to resume training")
    parser.add_argument("--eval",          type=str,   default=None,
                        help="Evaluate a saved model (no training)")
    parser.add_argument("--algo",          type=str,   default="TD3",
                        choices=["TD3", "SAC"])
    args = parser.parse_args()

    if args.eval:
        evaluate(args.eval)
    else:
        config = DEFAULT_CONFIG.copy()
        config["total_timesteps"] = args.timesteps
        config["learning_rate"]   = args.learning_rate
        config["algorithm"]       = args.algo
        train(config, resume_path=args.resume)
