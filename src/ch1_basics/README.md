# Chapter 1 — The Robotic Nervous System

From industrial controller to ROS 2. In a traditional Kuka setup, the KRC controller is the brain. In ROS 2, that brain is split into small, specialized nodes.

## ROS 2: A Distributed System

ROS 2 is a distributed system where multiple nodes (small programs) run independently but communicate through topics, services, and actions. This allows for modularity and flexibility, as you can mix and match nodes for different tasks.

![](./images/Nodes-TopicandService.gif)

Nodes can be written in various programming languages (Python, C++, etc.) and can run on different machines, as long as they are connected to the same ROS 2 network. This is a key advantage of ROS 2, enabling scalability and distributed computing.

## Node Examples

Nodes can perform a wide range of functions, such as:
- Reading sensor data (e.g., cameras, LiDAR)
- Controlling actuators (e.g., robot arms, grippers)
- Processing data (e.g., computer vision, path planning)
- Communicating with other nodes (e.g., publishing/subscribing to topics, providing services)

## Packages and Launch Files

ROS 2 organizes nodes into packages, which are directories containing code, configuration files, and assets. A launch file is a script that starts multiple nodes together, making it easier to manage complex systems.

For this workshop, we have a package called `ch1_basics` that contains all the necessary nodes and launch files to get you started with moving a robot arm, along with robot descriptions (`rccn_kuka_robot_cell`, `kuka_kr300_support`) and configurations (`rccn_kuka_robot_cell_moveit_config`).



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
ros2 launch ch1_basics robot.launch.py mode:=gui
```

in the other terminal, run:

```bash
ros2 run rqt_tf_tree rqt_tf_tree
```

Once finished, press `Ctrl+C` to stop the launch.

2) **Launch the robot (manual mode)**

```bash
ros2 launch ch1_basics robot.launch.py mode:=manual
```

The robot may look white/broken in RViz while it waits for your data.

3) **Start your joint publisher**

Keep the launch running and open a new terminal. Run the joint publisher script:

```bash
ros2 run ch1_basics subset_mover
```

This initializes joints to zero and moves A1 and A3 with a sine wave.

## Troubleshooting

- Flickering robot: another publisher is active (e.g., GUI sliders). Close all terminals and relaunch with `mode:=manual`.
- `Command not found`: forgot to `source install/setup.bash` in the terminal you are using.
- Robot in pieces: joint names in your script must match the URDF prefix (for example `east_joint_a1`).
