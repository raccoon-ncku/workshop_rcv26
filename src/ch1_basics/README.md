# Chapter 1 — The Robotic Nervous System

From industrial controller to ROS 2. In a traditional Kuka setup, the KRC controller is the brain. In ROS 2, that brain is split into small, specialized nodes.

## Quick Map: SmartPad → ROS 2

This is a simplified analogy to help you understand the ROS 2 ecosystem. In practice, ROS 2 nodes can be more complex and interconnected than this table suggests.

| Industrial concept | ROS 2 equivalent | What it means |
| --- | --- | --- |
| The controller | Node | Small program for one task (read camera, move robot, etc.). |
| Background task | Package | Folder with code, configs, and 3D assets. |
| System variable | Topic | Shared channel where nodes publish/subscribe (e.g., joint positions). |
| Pendant screen | RViz2 | 3D visualization to see the robot move. |
| Program | Launch file | Starts multiple nodes together with one command. |

## Movement Pipeline

To move, every link in the chain must be present.

- **URDF (skeleton):** Defines links (base, arm, wrist), joints (revolute/prismatic), and visuals (meshes like KR300).
- **TF tree (hierarchy):** Tracks frames from base to tool; when `joint_a1` moves $10^{\circ}$, TF updates the tool pose.
- **Joint heartbeat:** Topic `/joint_states` with message `sensor_msgs/msg/JointState` flows as: your Python script → `/joint_states` → `robot_state_publisher` → RViz.

## Hands-On: Move Your First Joint

1) **Launch the robot (gui mode)**

```bash
ros2 launch ch1_basics ch1_exercise.launch.py mode:=gui
```

2) **Launch the robot (manual mode)**

```bash
ros2 launch ch1_basics ch1_exercise.launch.py mode:=manual
```

The robot may look white/broken in RViz while it waits for your data.

3) **Start your joint publisher**

```bash
ros2 run ch1_basics subset_mover
```

This initializes joints to zero and moves A1 and A3 with a sine wave.

## Troubleshooting

- Flickering robot: another publisher is active (e.g., GUI sliders). Close all terminals and relaunch with `mode:=manual`.
- `Command not found`: forgot to `source install/setup.bash` in the terminal you are using.
- Robot in pieces: joint names in your script must match the URDF prefix (for example `east_joint_a1`).

## Command Cheat Sheet

| Action | Command |
| --- | --- |
| Build | `colcon build --packages-select ch1_basics --symlink-install` |
| Source | `source install/setup.bash` |
| Launch | `ros2 launch ch1_basics ch1_exercise.launch.py mode:=manual` |
| Run node | `ros2 run ch1_basics subset_mover` |