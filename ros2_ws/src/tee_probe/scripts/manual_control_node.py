#!/usr/bin/env python3
"""
TEE Probe Manual Control Node - ROS2 Humble
============================================
Interactive terminal UI for manually commanding all 4 DOFs.
Use this to verify controllers work before starting DRL training.

Usage:
  ros2 run tee_probe manual_control_node.py

Or after sourcing workspace:
  python3 scripts/manual_control_node.py

Controls:
  [i/o] - Insert / Withdraw (DOF 1)
  [r/l] - Rotate CW / CCW (DOF 2)
  [a/p] - Anteflex / Retroflex large wheel (DOF 3)
  [d/e] - Left / Right lateral small wheel (DOF 4)
  [0]   - Home all joints
  [s]   - Print current joint states
  [q]   - Quit
"""

import sys
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState


# ROS2 controller command topics
# Format: /CONTROLLER_NAME/commands  (note: plural, Float64MultiArray)
TOPIC_INSERTION    = "/insertion_controller/commands"
TOPIC_AXIAL        = "/axial_rotation_controller/commands"
TOPIC_LARGE_WHEEL  = "/large_wheel_controller/commands"
TOPIC_SMALL_WHEEL  = "/small_wheel_controller/commands"

# Joint limits
LIMITS = {
    "insertion_joint":      (0.0,    0.4),
    "axial_rotation_joint": (-6.28,  6.28),   # effectively unlimited
    "large_wheel_joint":    (-1.5708, 1.5708),
    "small_wheel_joint":    (-1.5708, 1.5708),
}

STEP = {
    "insertion_joint":      0.02,    # 2 cm per keypress
    "axial_rotation_joint": 0.1571,  # ~9 degrees per keypress
    "large_wheel_joint":    0.0785,  # ~4.5 degrees per keypress
    "small_wheel_joint":    0.0785,
}

# Standard TEE imaging views for quick recall
STANDARD_VIEWS = {
    "mid_esophageal_4chamber":    (0.30,  0.0,   0.0,   0.0),
    "mid_esophageal_2chamber":    (0.30,  1.571, 0.0,   0.0),
    "mid_esophageal_long_axis":   (0.30,  2.356, 0.0,   0.0),  # 135°
    "transgastric_short_axis":    (0.40,  0.0,   1.4,   0.0),
    "home":                       (0.0,   0.0,   0.0,   0.0),
}


class ManualControlNode(Node):
    def __init__(self):
        super().__init__("tee_probe_manual_control")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        # Publishers - one per controller
        self.pub_insertion   = self.create_publisher(Float64MultiArray, TOPIC_INSERTION,   qos)
        self.pub_axial       = self.create_publisher(Float64MultiArray, TOPIC_AXIAL,       qos)
        self.pub_large_wheel = self.create_publisher(Float64MultiArray, TOPIC_LARGE_WHEEL, qos)
        self.pub_small_wheel = self.create_publisher(Float64MultiArray, TOPIC_SMALL_WHEEL, qos)

        # Subscriber - joint states feedback
        self.joint_states = {}
        self.create_subscription(JointState, "/joint_states", self._joint_state_cb, qos)

        # Current commanded positions (start at home)
        self.cmd = {
            "insertion_joint":      0.0,
            "axial_rotation_joint": 0.0,
            "large_wheel_joint":    0.0,
            "small_wheel_joint":    0.0,
        }

        self.get_logger().info("Manual control node ready.")
        self.get_logger().info("Publishing to controller topics. Verify with:")
        self.get_logger().info("  ros2 topic list | grep controller")

    def _joint_state_cb(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self.joint_states[name] = pos

    def _clamp(self, joint, value):
        lo, hi = LIMITS[joint]
        return max(lo, min(hi, value))

    def send_command(self):
        """Publish current commanded positions to all 4 controllers."""
        def pub(publisher, value):
            msg = Float64MultiArray()
            msg.data = [value]
            publisher.publish(msg)

        pub(self.pub_insertion,   self.cmd["insertion_joint"])
        pub(self.pub_axial,       self.cmd["axial_rotation_joint"])
        pub(self.pub_large_wheel, self.cmd["large_wheel_joint"])
        pub(self.pub_small_wheel, self.cmd["small_wheel_joint"])

    def go_to_view(self, view_name):
        ins, axial, large, small = STANDARD_VIEWS[view_name]
        self.cmd["insertion_joint"]      = ins
        self.cmd["axial_rotation_joint"] = axial
        self.cmd["large_wheel_joint"]    = large
        self.cmd["small_wheel_joint"]    = small
        self.send_command()
        print(f"\n→ Moving to view: {view_name}")
        self.print_status()

    def print_status(self):
        print("\n--- Current Commands ---")
        print(f"  DOF1 Insertion:    {self.cmd['insertion_joint']:.4f} m  "
              f"(range 0–0.4 m, {self.cmd['insertion_joint']*100:.1f} cm)")
        print(f"  DOF2 Axial Rot:    {self.cmd['axial_rotation_joint']:.4f} rad  "
              f"({self.cmd['axial_rotation_joint']*57.3:.1f}°)")
        print(f"  DOF3 Large Wheel:  {self.cmd['large_wheel_joint']:.4f} rad  "
              f"({self.cmd['large_wheel_joint']*57.3:.1f}°)")
        print(f"  DOF4 Small Wheel:  {self.cmd['small_wheel_joint']:.4f} rad  "
              f"({self.cmd['small_wheel_joint']*57.3:.1f}°)")
        if self.joint_states:
            print("--- Actual (from /joint_states) ---")
            for jn in ["insertion_joint","axial_rotation_joint","large_wheel_joint","small_wheel_joint"]:
                v = self.joint_states.get(jn, float('nan'))
                print(f"  {jn}: {v:.4f}")

    def run_keyboard_loop(self):
        print("\n" + "="*50)
        print("  TEE Probe Manual Controller (ROS2)")
        print("="*50)
        print("  [i/o]  DOF1: Insert / Withdraw")
        print("  [r/l]  DOF2: Rotate CW / CCW")
        print("  [a/p]  DOF3: Anteflex / Retroflex (large wheel)")
        print("  [d/e]  DOF4: Left / Right (small wheel)")
        print("  [0]    Home all joints")
        print("  [1-4]  Go to standard TEE view")
        print("  [s]    Print status")
        print("  [q]    Quit")
        print("  Views: 1=ME4C, 2=ME2C, 3=MELAX, 4=TG-SAX")
        print("="*50)

        view_list = list(STANDARD_VIEWS.keys())

        while rclpy.ok():
            try:
                key = input("cmd> ").strip().lower()
            except EOFError:
                break

            if key == "q":
                print("Exiting manual control.")
                break
            elif key == "i":
                self.cmd["insertion_joint"] = self._clamp(
                    "insertion_joint", self.cmd["insertion_joint"] + STEP["insertion_joint"])
                self.send_command()
            elif key == "o":
                self.cmd["insertion_joint"] = self._clamp(
                    "insertion_joint", self.cmd["insertion_joint"] - STEP["insertion_joint"])
                self.send_command()
            elif key == "r":
                self.cmd["axial_rotation_joint"] += STEP["axial_rotation_joint"]
                self.send_command()
            elif key == "l":
                self.cmd["axial_rotation_joint"] -= STEP["axial_rotation_joint"]
                self.send_command()
            elif key == "a":
                self.cmd["large_wheel_joint"] = self._clamp(
                    "large_wheel_joint", self.cmd["large_wheel_joint"] + STEP["large_wheel_joint"])
                self.send_command()
            elif key == "p":
                self.cmd["large_wheel_joint"] = self._clamp(
                    "large_wheel_joint", self.cmd["large_wheel_joint"] - STEP["large_wheel_joint"])
                self.send_command()
            elif key == "d":
                self.cmd["small_wheel_joint"] = self._clamp(
                    "small_wheel_joint", self.cmd["small_wheel_joint"] + STEP["small_wheel_joint"])
                self.send_command()
            elif key == "e":
                self.cmd["small_wheel_joint"] = self._clamp(
                    "small_wheel_joint", self.cmd["small_wheel_joint"] - STEP["small_wheel_joint"])
                self.send_command()
            elif key == "0":
                for k in self.cmd:
                    self.cmd[k] = 0.0
                self.send_command()
                print("→ Homed all joints.")
            elif key in ("1", "2", "3", "4"):
                idx = int(key) - 1
                if idx < len(view_list):
                    self.go_to_view(view_list[idx])
            elif key == "s":
                self.print_status()
            else:
                print(f"Unknown key: '{key}'")


def main(args=None):
    rclpy.init(args=args)
    node = ManualControlNode()

    # Spin in background thread so keyboard input doesn't block callbacks
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.run_keyboard_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
