#!/bin/bash
set -e

# 1. Setup Environment
cd /home/ubuntu/workshop_ws
source /opt/ros/jazzy/setup.bash

# Fix dubious ownership for git repositories in the workspace
git config --global --add safe.directory '*'

# 2. Automate X11 Permissions
xhost +local:root > /dev/null 2>&1 || echo "export DISPLAY=:1" >> ~/.bashrc

# 3. Import & Dependency Sync
echo "--- Importing Repositories ---"
vcs import src < workshop.repos

echo "--- Installing System Dependencies ---"
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt-get update
sudo apt install -y ros-jazzy-moveit-ros-perception ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-qt*
rosdep update
rosdep install --from-paths src --ignore-src -y --rosdistro jazzy

# 4. Build
echo "--- Building Workspace ---"
colcon build --symlink-install

# 5. Clean Bashrc
sed -i '/workshop_ws\/install\/setup.bash/d' ~/.bashrc
echo "source /home/ubuntu/workshop_ws/install/setup.bash" >> ~/.bashrc

echo "--- Setup Complete! ---"