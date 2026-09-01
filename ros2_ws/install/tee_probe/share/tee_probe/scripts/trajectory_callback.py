#!/usr/bin/env python3
"""
Trajectory Plotting Callback for TEE Probe DRL Training
========================================================
Saves trajectory plots every N episodes during training.
Shows: 3D tip path, 2D projections, joint-space path, reward over episode.
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from stable_baselines3.common.callbacks import BaseCallback

# Standard TEE view names for labelling
VIEW_NAMES_DISPLAY = {
    "mid_esophageal_4chamber":  "ME 4-Chamber",
    "mid_esophageal_2chamber":  "ME 2-Chamber",
    "mid_esophageal_long_axis": "ME Long Axis",
    "transgastric_short_axis":  "TG Short Axis",
}


class TrajectoryPlotCallback(BaseCallback):
    """
    Records probe tip XYZ and joint positions during rollout.
    Saves trajectory + reward plots every `plot_every` episodes.
    """

    def __init__(self, plot_every=10, save_dir='plots/trajectories', verbose=0):
        super().__init__(verbose)
        self.plot_every  = plot_every
        self.save_dir    = save_dir
        self.episode_num = 0

        # Per-episode buffers
        self._tip_path    = []   # list of [x, y, z]
        self._joint_path  = []   # list of [j0, j1, j2, j3]
        self._step_rewards = []  # reward at each step
        self._goal_joints  = None
        self._goal_view    = None

        # Cross-episode tracking
        self._ep_rewards  = []
        self._ep_dists    = []
        self._ep_numbers  = []
        self._ep_reached  = []   # bool: goal reached?

        os.makedirs(save_dir, exist_ok=True)

    def _on_step(self) -> bool:
        infos = self.locals.get('infos', [{}])
        info  = infos[0] if infos else {}

        # Record tip position
        if 'tip_xyz' in info:
            self._tip_path.append(info['tip_xyz'].copy())

        # Record joint positions from env
        try:
            env = self.training_env.envs[0].env
            self._joint_path.append(env._joint_pos.copy())
            if self._goal_joints is None and hasattr(env, '_goal_position'):
                self._goal_joints = env._goal_position.copy()
            if self._goal_view is None and hasattr(env, '_goal_view'):
                self._goal_view = env._goal_view
        except Exception:
            pass

        # Record step reward
        rewards = self.locals.get('rewards', [0])
        self._step_rewards.append(float(rewards[0]))

        # Episode end
        dones = self.locals.get('dones', [False])
        if dones[0]:
            self.episode_num += 1
            ep_reward = float(np.sum(self._step_rewards))
            ep_dist   = float(info.get('distance_to_goal', -1))
            reached   = ep_dist >= 0 and ep_dist < 0.15

            self._ep_rewards.append(ep_reward)
            self._ep_dists.append(ep_dist)
            self._ep_numbers.append(self.episode_num)
            self._ep_reached.append(reached)

            if self.episode_num % self.plot_every == 0:
                self._save_episode_plot(ep_dist, reached)
                self._save_summary_plot()

            # Reset buffers
            self._tip_path     = []
            self._joint_path   = []
            self._step_rewards = []
            self._goal_joints  = None
            self._goal_view    = None

        return True

    def _save_episode_plot(self, final_dist, reached):
        if len(self._joint_path) < 2:
            return

        tip   = np.array(self._tip_path)    if self._tip_path   else None
        jpath = np.array(self._joint_path)  # (N, 4)
        rews  = np.array(self._step_rewards)
        goal  = self._goal_joints            # [ins, axial, large, small]
        view  = VIEW_NAMES_DISPLAY.get(self._goal_view, self._goal_view or 'Unknown')

        color_path = '#1baf7a' if reached else '#2a78d6'
        status     = 'GOAL REACHED ✓' if reached else f'missed  (dist={final_dist:.3f})'
        title_col  = '#1baf7a' if reached else '#eb6834'

        fig = plt.figure(figsize=(16, 9))
        fig.suptitle(
            f'Episode {self.episode_num}  —  {status}  |  Target: {view}  |  Steps: {len(jpath)}',
            fontsize=12, fontweight='bold', color=title_col, y=0.98
        )

        # ── Row 1: Joint-space paths (most informative) ────────────────
        # Plot theta1 (axial) vs theta2 (large wheel) — these matter most
        ax1 = fig.add_subplot(2, 4, 1)
        ax1.plot(np.degrees(jpath[:, 1]), np.degrees(jpath[:, 2]),
                 color=color_path, linewidth=1.5, alpha=0.8, zorder=3)
        ax1.scatter(np.degrees(jpath[0,  1]), np.degrees(jpath[0,  2]),
                    color='#2a78d6', marker='o', s=80, zorder=5, label='Start')
        ax1.scatter(np.degrees(jpath[-1, 1]), np.degrees(jpath[-1, 2]),
                    color=color_path, marker='s', s=80, zorder=5, label='End')
        if goal is not None:
            ax1.scatter(np.degrees(goal[1]), np.degrees(goal[2]),
                        color='red', marker='*', s=200, zorder=6, label='Goal ★')
            # goal tolerance circle
            circle = plt.Circle((np.degrees(goal[1]), np.degrees(goal[2])),
                                 np.degrees(0.15), color='red', fill=False,
                                 linewidth=1.2, linestyle='--', alpha=0.5, label='Tolerance')
            ax1.add_patch(circle)
        ax1.set_xlabel('Axial rotation θ₁ (°)', fontsize=8)
        ax1.set_ylabel('Large wheel θ₂ (°)', fontsize=8)
        ax1.set_title('Joint space\n(axial vs large wheel)', fontsize=9, fontweight='bold')
        ax1.legend(fontsize=7, loc='upper right')
        ax1.grid(True, alpha=0.3)
        ax1.set_aspect('equal')

        # Insertion vs small wheel
        ax2 = fig.add_subplot(2, 4, 2)
        ax2.plot(jpath[:, 0]*100, np.degrees(jpath[:, 3]),
                 color=color_path, linewidth=1.5, alpha=0.8)
        ax2.scatter(jpath[0,  0]*100, np.degrees(jpath[0,  3]),
                    color='#2a78d6', marker='o', s=80, zorder=5)
        ax2.scatter(jpath[-1, 0]*100, np.degrees(jpath[-1, 3]),
                    color=color_path, marker='s', s=80, zorder=5)
        if goal is not None:
            ax2.scatter(goal[0]*100, np.degrees(goal[3]),
                        color='red', marker='*', s=200, zorder=6)
        ax2.set_xlabel('Insertion depth (cm)', fontsize=8)
        ax2.set_ylabel('Small wheel θ₃ (°)', fontsize=8)
        ax2.set_title('Joint space\n(insertion vs small wheel)', fontsize=9, fontweight='bold')
        ax2.grid(True, alpha=0.3)

        # ── Step reward over episode ───────────────────────────────────
        ax3 = fig.add_subplot(2, 4, 3)
        cumulative = np.cumsum(rews)
        ax3.plot(rews,       color='#4a3aa7', linewidth=1, alpha=0.6, label='Step reward')
        ax3.axhline(0, color='gray', linewidth=0.8, linestyle='--')
        ax3.set_xlabel('Step', fontsize=8)
        ax3.set_ylabel('Reward', fontsize=8)
        ax3.set_title('Per-step reward\nduring episode', fontsize=9, fontweight='bold')
        ax3.legend(fontsize=7)
        ax3.grid(True, alpha=0.3)

        # Cumulative reward
        ax4 = fig.add_subplot(2, 4, 4)
        ax4.plot(cumulative, color='#eb6834', linewidth=1.5)
        ax4.axhline(0, color='gray', linewidth=0.8, linestyle='--')
        ax4.set_xlabel('Step', fontsize=8)
        ax4.set_ylabel('Cumulative reward', fontsize=8)
        ax4.set_title(f'Cumulative reward\nTotal: {cumulative[-1]:.1f}', fontsize=9, fontweight='bold')
        ax4.grid(True, alpha=0.3)

        # ── Row 2: Tip XYZ (if available) ─────────────────────────────
        if tip is not None and len(tip) > 1:
            ax5 = fig.add_subplot(2, 4, 5, projection='3d')
            ax5.plot(tip[:, 0], tip[:, 1], tip[:, 2],
                     color=color_path, linewidth=1.2, alpha=0.8)
            ax5.scatter(*tip[0],  color='#2a78d6', marker='o', s=60, zorder=5)
            ax5.scatter(*tip[-1], color=color_path, marker='s', s=60, zorder=5)
            ax5.set_xlabel('X (m)', fontsize=7)
            ax5.set_ylabel('Y (m)', fontsize=7)
            ax5.set_zlabel('Z (m)', fontsize=7)
            ax5.set_title('Tip 3D path', fontsize=9, fontweight='bold')
            ax5.view_init(elev=25, azim=-60)

            for col_idx, (xlabel, ylabel, title), (xi, yi) in zip(
                [6,7,8],
                [('X(m)','Y(m)','XY top'), ('Y(m)','Z(m)','YZ side'), ('X(m)','Z(m)','XZ front')],
                [(0,1),(1,2),(0,2)]
            ):
                ax = fig.add_subplot(2, 4, col_idx)
                ax.plot(tip[:, xi], tip[:, yi], color=color_path, linewidth=1.2, alpha=0.8)
                ax.scatter(tip[0,  xi], tip[0,  yi], color='#2a78d6', marker='o', s=40, zorder=5)
                ax.scatter(tip[-1, xi], tip[-1, yi], color=color_path, marker='s', s=40, zorder=5)
                ax.set_xlabel(xlabel, fontsize=8)
                ax.set_ylabel(ylabel, fontsize=8)
                ax.set_title(title, fontsize=9, fontweight='bold')
                ax.grid(True, alpha=0.3)

        # Goal info box
        if goal is not None:
            goal_text = (f'Goal: {view}\n'
                         f'Insertion: {goal[0]*100:.1f} cm\n'
                         f'Axial:     {np.degrees(goal[1]):.1f}°\n'
                         f'Large whl: {np.degrees(goal[2]):.1f}°\n'
                         f'Small whl: {np.degrees(goal[3]):.1f}°\n'
                         f'Final dist: {final_dist:.4f}')
            fig.text(0.01, 0.02, goal_text, fontsize=7.5,
                     family='monospace', color='#444441',
                     bbox=dict(boxstyle='round', facecolor='#f1efe8', alpha=0.8))

        plt.tight_layout(rect=[0, 0.06, 1, 0.97])
        fname = os.path.join(self.save_dir, f'ep_{self.episode_num:05d}_trajectory.png')
        plt.savefig(fname, dpi=120, bbox_inches='tight')
        plt.close()

    def _save_summary_plot(self):
        if len(self._ep_rewards) < 2:
            return

        fig, axes = plt.subplots(1, 3, figsize=(15, 4))

        eps = np.array(self._ep_numbers)
        rew = np.array(self._ep_rewards)
        dst = np.array(self._ep_dists)
        hit = np.array(self._ep_reached)

        # -- Reward --
        axes[0].plot(eps, rew, color='#2a78d6', linewidth=1, alpha=0.5)
        if len(rew) >= 10:
            roll = np.convolve(rew, np.ones(10)/10, mode='valid')
            axes[0].plot(eps[9:], roll, color='#eb6834', linewidth=2, label='10-ep mean')
        axes[0].axhline(0, color='#1baf7a', linewidth=1.2, linestyle='--', alpha=0.7)
        axes[0].scatter(eps[hit], rew[hit], color='#1baf7a', s=20, zorder=5, label='Goal reached')
        axes[0].set_xlabel('Episode', fontsize=10)
        axes[0].set_ylabel('Total reward', fontsize=10)
        axes[0].set_title('Reward per episode', fontsize=11, fontweight='bold')
        axes[0].legend(fontsize=8)
        axes[0].grid(True, alpha=0.3)

        # -- Distance --
        dst_valid = dst[dst >= 0]
        eps_valid = eps[dst >= 0]
        if len(dst_valid):
            axes[1].plot(eps_valid, dst_valid, color='#4a3aa7', linewidth=1, alpha=0.6)
            axes[1].axhline(0.15, color='#1baf7a', linewidth=1.5,
                            linestyle='--', label='Goal tolerance')
            axes[1].fill_between(eps_valid, 0, dst_valid,
                                 where=dst_valid < 0.15, alpha=0.3,
                                 color='#1baf7a', label='Within goal')
            axes[1].set_xlabel('Episode', fontsize=10)
            axes[1].set_ylabel('Final distance to goal', fontsize=10)
            axes[1].set_title('Distance to goal', fontsize=11, fontweight='bold')
            axes[1].legend(fontsize=8)
            axes[1].grid(True, alpha=0.3)

        # -- Goal rate (rolling) --
        if len(hit) >= 10:
            roll_hit = np.convolve(hit.astype(float), np.ones(20)/20, mode='valid') * 100
            axes[2].plot(eps[19:], roll_hit, color='#1baf7a', linewidth=2)
            axes[2].fill_between(eps[19:], 0, roll_hit, alpha=0.3, color='#1baf7a')
        axes[2].set_xlabel('Episode', fontsize=10)
        axes[2].set_ylabel('Goal rate (%)', fontsize=10)
        axes[2].set_title('20-episode rolling goal rate', fontsize=11, fontweight='bold')
        axes[2].set_ylim(0, 105)
        axes[2].grid(True, alpha=0.3)

        total_goals = hit.sum()
        axes[2].set_title(
            f'Goal rate  (total: {total_goals}/{len(hit)} = {100*total_goals/len(hit):.1f}%)',
            fontsize=10, fontweight='bold'
        )

        plt.suptitle(f'Training summary — episode {self.episode_num}',
                     fontsize=12, fontweight='bold')
        plt.tight_layout()
        fname = os.path.join(self.save_dir, 'training_summary.png')
        plt.savefig(fname, dpi=120, bbox_inches='tight')
        plt.close()

