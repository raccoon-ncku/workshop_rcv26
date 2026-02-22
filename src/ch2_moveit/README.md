# Chapter 2: The Safety Brain (MoveIt 2)

In Chapter 1, we moved joints manually by guessing angles. In Chapter 2, we introduce MoveIt 2, the industry-standard motion planning framework for ROS 2. Instead of "jiggling" joints, we now think in terms of goals and obstacles.

## 1. What is MoveIt 2?

MoveIt 2 is the "Safety Brain" of the robot. It handles:

* **Inverse Kinematics (IK):** You tell the robot where the tool should be in 3D space (X,Y,Z), and MoveIt calculates the 6 joint angles needed to get there.
* **Collision Avoidance:** It ensures the robot never hits itself, the floor, or any virtual obstacles you define.
* **Motion Planning:** It finds the smoothest, most efficient path between two points.

### Use Case in Architecture

In robotic fabrication (e.g., 3D printing or timber milling), we don't care about the angle of Joint 4. We care about the Tool Center Point (TCP) following a specific geometry. MoveIt allows us to "Attach" a tool to the robot and plan paths for that tool while avoiding structural columns or other equipment.

## 2. Learning Goals

* **Understand the Planning Scene:** Learn how to inject virtual objects into the robot's world.
* **Tool Attachment:** Learn how to tell the robot it is "holding" an object so that the object itself is included in collision checks.
* **Interactive Planning:** Use RViz markers to plan and execute motions in a collision-aware environment.

## 3. Execution Steps

### Step 1: Launch the MoveIt Environment

This starts the "Brain" and the 3D Interface. Note that we are using the `rccn_kuka_robot_cell` configuration.

```bash
ros2 launch rccn_kuka_robot_cell robot_moveit.launch.py
```

### Step 2: Publish the Workshop Obstacle

In a new terminal, run the node that places a virtual "Table" in front of the robot. This node is a "one-shot" publisher to prevent the screen from flickering.

```bash
ros2 run ch2_moveit obstacle_publisher
```

### Step 3: Attach the Fabrication Tool

Now, tell the robot it is holding a 30cm extruder. This ensures the robot knows its "Hand" is now longer.

```bash
ros2 run ch2_moveit tool_attacher
```

## 4. What to Expect (The Results)

### In RViz (The Visuals):

* **The Obstacle:** A red translucent box should appear in front of the Kuka.
* **The Tool:** A green/grey cylinder should appear attached to the robot's flange (`link_6`).
* **Interactive Marker:** You will see a "Ball" with arrows at the tip of the robot. This is your target goal.

### In Practice:

* **Drag the Marker:** Try to move the marker behind or under the red box.
* **Plan:** Click the "Plan" button in the MoveIt panel. You will see a "Ghost" robot find a path that curves around the obstacle.
* **Collision Check:** If you drag the marker inside the red box, the robot will turn red, indicating a collision. MoveIt will refuse to plan a path into this forbidden zone.

## 5. Summary of MoveIt Communication

MoveIt 2 listens to the Planning Scene. Unlike the constant stream of joint angles in Chapter 1, the Planning Scene is Event-Based. We only send a message when an object is added or removed.

| Command | Role |
| :--- | :--- |
| `/planning_scene` | The topic used to add/remove obstacles and tools. |
| `is_diff = True` | Ensures we add to the world rather than replacing it. |
| `touch_links` | Tells the robot "don't be afraid of the tool you are holding." |

---

**Next Step:** Now that we can navigate safely around obstacles, it's time to find them! In Chapter 3, we will use Computer Vision to detect real-world objects and automatically turn them into MoveIt obstacles.

*Would you like me to begin the setup for Chapter 3: Computer Vision, including how to simulate an Aruco marker for the robot to "see"?*
