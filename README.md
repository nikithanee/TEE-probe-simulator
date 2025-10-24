# TEE Probe Simulator – ROS2 | Gazebo

A research-grade simulation of a Transoesophageal Echocardiography (TEE) probe modelled as a continuum manipulator in ROS2 + Gazebo, developed as part of autonomous probe navigation work.

![](https://github.com/nikithanee/TEE-probe-simulator/blob/main/Untitled%20design.gif)

## Early Development - Work in progress

<img width="1423" height="811" alt="Screenshot 2025-10-22 at 23 59 11" src="https://github.com/user-attachments/assets/d6a9c987-cfcc-4bf7-89cd-d9774566d5ba" />

The simulator currently loads a basic 4-DOF URDF model of the TEE probe in ROS2 + Gazebo Classic. We are working on configuring the ros2_control controllers for joint actuation and preparing for integration with AI-based motion planning.

### Completed so far:
- URDF model of the TEE probe
- Basic kinematic structure reflecting flexion and axial rotation
- ROS2 workspace setup
- Gazebo Classic launch with robot_state_publisher
  
###  In Progress:
- Controller configuration and hardware interfaces
- Joint control testing in simulation
- Motion commands and teleoperation testing
  
### Upcoming Goals:
- Train and deploy an AI agent to navigate the TEE probe
- Implement reinforcement learning-based control
- Integrate with visual guidance and safety constraints

## Background

A TEE probe is a clinical device used for cardiac ultrasound imaging by inserting the probe through the esophagus. Unlike rigid-link robots, a TEE probe behaves as a continuum manipulator — it bends smoothly using tensioned cables and manual control wheels.
Key motion capabilities:
- Lateral flexion (θ₁): Left/right bending
- Vertical flexion (θ₂): Up/down bending
- Axial rotation (θ₃): Rotate around backbone
- Insertion/withdrawal (d₄): Translation
This simulator aims to automate probe manipulation to improve safety and reduce human error during TEE procedures. It will form the foundation for autonomous navigation research in surgical robotics.

## 👥 Credits

- Nikitha Neerupudi - Developer
- Afsah Asif Rashid (Supervisor) - AI Model & Guidance
- Other works and research by Ole Jakob Elle and Seyed MohammadReza Sajadi

## Disclaimer 
This simulation is strictly for research purposes. It is not for clinical use or real-patient deployment in its current state.
