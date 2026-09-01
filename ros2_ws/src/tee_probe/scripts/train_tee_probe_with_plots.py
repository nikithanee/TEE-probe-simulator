#!/usr/bin/env python3
"""
TEE Probe DRL Training Script - ROS2 Humble
With live trajectory plotting every N episodes.
"""

import argparse
import os
import sys
import time
import numpy as np
import rclpy

try:
    from stable_baselines3 import TD3, SAC
    from stable_baselines3.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise
    from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback, StopTrainingOnRewardThreshold
    from stable_baselines3.common.monitor import Monitor
    from stable_baselines3.common.vec_env import DummyVecEnv
except ImportError:
    print("ERROR: stable-baselines3 not installed.")
    sys.exit(1)

from tee_probe_env import TEEProbeEnv
from trajectory_callback import TrajectoryPlotCallback   # <-- NEW

DEFAULT_CONFIG = {
    "algorithm":         "TD3",
    "total_timesteps":   1_000_000,
    "learning_rate":     3e-4,
    "batch_size":        256,
    "buffer_size":       100_000,
    "learning_starts":   5_000,
    "gamma":             0.99,
    "tau":               0.005,
    "policy":            "MlpPolicy",
    "policy_kwargs":     dict(net_arch=[256, 256]),
    "train_freq":        1,
    "gradient_steps":    1,
    "noise_type":        "normal",
    "noise_sigma":       0.1,
    "eval_freq":         5_000,
    "n_eval_episodes":   5,
    "checkpoint_freq":   10_000,
    "log_dir":           "logs/td3_tee_probe",
    "model_dir":         "models",
    "model_name":        "td3_tee_probe",
    "plot_every":        10,       # <-- save trajectory plot every 10 episodes
    "plot_dir":          "plots/trajectories",  # <-- where plots are saved
}


def make_env():
    def _init():
        env = TEEProbeEnv()
        env = Monitor(env)
        return env
    return _init


def build_noise(action_dim, noise_type, sigma):
    mean = np.zeros(action_dim)
    if noise_type == "ou":
        return OrnsteinUhlenbeckActionNoise(mean=mean, sigma=sigma * np.ones(action_dim))
    return NormalActionNoise(mean=mean, sigma=sigma * np.ones(action_dim))


def train(config, resume_path=None):
    os.makedirs(config["log_dir"],   exist_ok=True)
    os.makedirs(config["model_dir"], exist_ok=True)
    os.makedirs(config["plot_dir"],  exist_ok=True)

    print("[TRAIN] Creating environment...")
    env      = DummyVecEnv([make_env()])
    eval_env = DummyVecEnv([make_env()])

    action_noise = build_noise(4, config["noise_type"], config["noise_sigma"])

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

    # Callbacks
    checkpoint_cb = CheckpointCallback(
        save_freq=config["checkpoint_freq"],
        save_path=config["model_dir"],
        name_prefix=config["model_name"],
        verbose=1,
    )
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

    # Trajectory plot callback — NEW
    traj_cb = TrajectoryPlotCallback(
        plot_every=config["plot_every"],
        save_dir=config["plot_dir"],
        verbose=1,
    )

    callbacks = [checkpoint_cb, eval_cb, traj_cb]   # <-- traj_cb added

    print(f"\n[TRAIN] Starting training for {config['total_timesteps']:,} timesteps...")
    print(f"[TRAIN] Trajectory plots saved every {config['plot_every']} episodes to: {config['plot_dir']}")
    print(f"[TRAIN] TensorBoard logs: {config['log_dir']}")
    print()

    t0 = time.time()
    model.learn(
        total_timesteps=config["total_timesteps"],
        callback=callbacks,
        progress_bar=True,
        reset_num_timesteps=resume_path is None,
    )
    elapsed = time.time() - t0

    final_path = os.path.join(config["model_dir"], f"{config['model_name']}_final")
    model.save(final_path)
    print(f"\n[TRAIN] Done in {elapsed/60:.1f} minutes. Model saved: {final_path}.zip")
    return model


def evaluate(model_path, n_episodes=10):
    print(f"\n[EVAL] Loading model: {model_path}")
    env   = TEEProbeEnv()
    model = TD3.load(model_path)

    episode_rewards, episode_lengths, goal_reached = [], [], 0
    for ep in range(n_episodes):
        obs, _ = env.reset()
        ep_reward, step, done = 0.0, 0, False
        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env.step(action)
            ep_reward += reward
            step += 1
            done = terminated or truncated
        episode_rewards.append(ep_reward)
        episode_lengths.append(step)
        if info.get("distance_to_goal", 1.0) < 0.15:
            goal_reached += 1
        print(f"  Episode {ep+1:3d}: reward={ep_reward:8.2f}  steps={step:4d}  dist={info.get('distance_to_goal',-1):.4f}")

    print(f"\n[EVAL] Mean reward:  {np.mean(episode_rewards):.2f} ± {np.std(episode_rewards):.2f}")
    print(f"[EVAL] Mean length:  {np.mean(episode_lengths):.1f} steps")
    print(f"[EVAL] Goal rate:    {goal_reached}/{n_episodes} ({100*goal_reached/n_episodes:.0f}%)")
    env.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--timesteps",     type=int,   default=DEFAULT_CONFIG["total_timesteps"])
    parser.add_argument("--learning-rate", type=float, default=DEFAULT_CONFIG["learning_rate"])
    parser.add_argument("--resume",        type=str,   default=None)
    parser.add_argument("--eval",          type=str,   default=None)
    parser.add_argument("--algo",          type=str,   default="TD3", choices=["TD3","SAC"])
    parser.add_argument("--plot-every",    type=int,   default=DEFAULT_CONFIG["plot_every"])
    args = parser.parse_args()

    if args.eval:
        evaluate(args.eval)
    else:
        config = DEFAULT_CONFIG.copy()
        config["total_timesteps"] = args.timesteps
        config["learning_rate"]   = args.learning_rate
        config["algorithm"]       = args.algo
        config["plot_every"]      = args.plot_every
        train(config, resume_path=args.resume)
