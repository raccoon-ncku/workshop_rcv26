#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # --- 1. Paths & Setup ---
    pkg_name = "rccn_kuka_robot_cell"
    pkg_share = FindPackageShare(pkg_name)

    # --- 2. Declare Arguments ---
    # mode: 'gui' (sliders), 'headless' (zeros), 'manual' (waiting for student script)
    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="gui",
        description="Launch mode: 'gui' (sliders), 'headless' (zeros), or 'manual'."
    )

    model_arg = DeclareLaunchArgument(
        "model",
        default_value=PathJoinSubstitution([pkg_share, "urdf", "rccn_kuka_robot_cell.urdf"]),
        description="Absolute path to URDF/Xacro file"
    )

    rviz_config_arg = DeclareLaunchArgument(
        "rvizconfig",
        default_value=PathJoinSubstitution([pkg_share, "rviz", "urdf.rviz"]),
        description="Path to RViz config file"
    )

    # --- 3. Process Robot Description (Xacro to String) ---
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]), 
        value_type=str
    )

    # --- 4. Logic for Joint State Publishers ---
    # Mode Logic: We use Python expressions to evaluate the 'mode' argument
    is_gui = PythonExpression(["'", LaunchConfiguration('mode'), "' == 'gui'"])
    is_headless = PythonExpression(["'", LaunchConfiguration('mode'), "' == 'headless'"])

    # A: The Sliders (User manual interaction)
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        condition=IfCondition(is_gui)
    )

    # B: The Headless "Ghost" (Publishes zeros to keep frames valid)
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        condition=IfCondition(is_headless)
    )

    # --- 5. Essential Nodes (Always Running) ---
    # Robot State Publisher (The Skeleton)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_description}]
    )

    # RViz2 (The Eyes)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
        parameters=[{"robot_description": robot_description}]
    )

    # --- 6. Return Launch Description ---
    return LaunchDescription([
        mode_arg,
        model_arg,
        rviz_config_arg,
        joint_state_publisher_gui_node,
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node
    ])