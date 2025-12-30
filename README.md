# CynLr_Dual_Arm

# Robot Simulation Workspace (ROS 2 + Gazebo + MoveIt)

Monorepo-style ROS 2 workspace that combines multi-robot Gazebo simulation, MoveIt-based motion action servers, vision-based tracking, and Gazebo link attachment for grasping.

## Repository layout

- **`ball_tracker/`**  
  Vision/tracking package used to detect and track a target (e.g., ball) and publish its estimated position for downstream controllers (e.g., end-effector tracking).

- **`IFRA_LinkAttacher/`**  
  Gazebo *link attachment* plugin + ROS 2 interfaces providing attach/detach services (e.g., `/ATTACHLINK`, `/DETACHLINK`) to “grasp” objects by rigidly attaching a model/link to a robot link.

- **`multi_robot_arm/`**  
  Multi-robot Gazebo simulation stack: robot spawning, namespacing, `ros2_control` setup, and launch files for running multiple arms in the same world (UR5 + Panda). It is the entry point for launching the full simulation.

- **`ros2_RobotSimulation/`**  
  IFRA-style MoveIt action infrastructure (messages + action servers + scripts). Provides motion execution primitives (e.g., `MoveL`) and serves as the base used for custom Cartesian trajectories (circle, Lissajous, tracking controllers, etc.).

## Workflow (high level)

1. Launch simulation from `ros2 launch multi_robot_arm multi_robot_arm.launch.py`.
2. Launch the vision identification via `ros2 launch ball_tracker ball_tracker.launch.py`.

## Build

colcon build --symlink-install
source install/setup.bash

## Demo

[Watch the demo video](https://github.com/Oussama1209/CynLr_Dual_Arm/releases/download/video/Screen.Recording.2025-12-26.130635.mp4)
